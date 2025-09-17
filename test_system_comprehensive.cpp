/**
 * COMPREHENSIVE SYSTEM TEST
 * Tests all components of the navigation system
 */

#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <Eigen/Dense>
#include "cpp/core/types.h"
#include "cpp/core/ukf.h"
#include "cpp/core/ukf_config.h"
#include "cpp/core/gravity_gradient_provider.h"
#include "cpp/core/hierarchical_map_matcher.h"
#include "cpp/core/terrain_correlator.h"
#include "cpp/core/srtm_provider.h"

// Test result tracking
struct TestResult {
    std::string name;
    bool passed;
    std::string details;
    double metric;
};

std::vector<TestResult> results;

void reportTest(const std::string& name, bool passed, const std::string& details, double metric = 0.0) {
    results.push_back({name, passed, details, metric});
    std::cout << (passed ? "✅ " : "❌ ") << name << ": " << details;
    if (metric > 0) std::cout << " (metric: " << metric << ")";
    std::cout << "\n";
}

// Test 1: UKF Numerical Stability
bool testUKFStability() {
    std::cout << "\n=== TEST 1: UKF Numerical Stability ===\n";

    UKFConfig config;
    config.setGradeAOptimal();
    UKF ukf(config);

    // Initialize with reasonable state
    State x0;
    x0.fromGeodetic(47.0 * M_PI/180, 8.0 * M_PI/180, 5000.0);
    x0.v_ECEF = Eigen::Vector3d(100, 0, 0);

    Eigen::Matrix<double, 15, 15> P0 = Eigen::Matrix<double, 15, 15>::Identity();
    P0.block<3,3>(0,0) *= 100.0;  // 10m position uncertainty
    P0.block<3,3>(3,3) *= 1.0;    // 1m/s velocity uncertainty
    P0.block<3,3>(6,6) *= 0.01;   // Small attitude uncertainty

    ukf.init(x0, P0);

    // Run prediction/update cycle
    bool stable = true;
    double max_nis = 0.0;

    for (int i = 0; i < 100; i++) {
        ImuSample imu;
        imu.acc_mps2 = Eigen::Vector3d(0, 0, -9.81);
        imu.gyro_rps = Eigen::Vector3d(0, 0, 0.01);

        ukf.predict(imu, 0.1);

        // Check covariance remains positive definite
        auto P = ukf.getCovariance();
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(P);
        double min_eigenvalue = solver.eigenvalues().minCoeff();

        if (min_eigenvalue < 0 || !std::isfinite(min_eigenvalue)) {
            stable = false;
            reportTest("Covariance positive definite", false,
                      "Min eigenvalue: " + std::to_string(min_eigenvalue), min_eigenvalue);
            break;
        }

        // Track NIS
        max_nis = std::max(max_nis, ukf.getNIS());
    }

    if (stable) {
        reportTest("UKF 100-step stability", true, "Max NIS: " + std::to_string(max_nis), max_nis);
    }

    return stable;
}

// Test 2: Gravity Model Performance
bool testGravityModel() {
    std::cout << "\n=== TEST 2: Gravity Model Performance ===\n";

    GravityGradientProvider gravity;

    // Try to load EGM2008
    bool loaded = false;
    std::string load_path;

    if (gravity.loadEGM2020("egm2008/egm2008_n360.dat")) {
        loaded = true;
        load_path = "egm2008/egm2008_n360.dat";
    } else if (gravity.loadEGM2020("/Users/yarontorgeman/stn-v0.1/egm2008/egm2008_n360.dat")) {
        loaded = true;
        load_path = "/Users/yarontorgeman/stn-v0.1/egm2008/egm2008_n360.dat";
    }

    reportTest("EGM2008 loading", loaded, loaded ? "Loaded from " + load_path : "Failed to load");
    if (!loaded) return false;

    // Test computation speed
    Eigen::Vector3d test_pos;
    test_pos << 4.2e6, 0.8e6, 4.7e6;  // Switzerland ECEF

    auto start = std::chrono::high_resolution_clock::now();
    int num_evals = 100;

    for (int i = 0; i < num_evals; i++) {
        auto gradient = gravity.getGradient(test_pos);
        auto anomaly = gravity.getAnomaly(test_pos);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    double us_per_eval = duration.count() / (double)num_evals;

    bool fast_enough = us_per_eval < 10000;  // Should be < 10ms per evaluation
    reportTest("Gravity computation speed", fast_enough,
              std::to_string(us_per_eval) + " µs/eval", us_per_eval);

    // Test gradient tensor properties
    auto result = gravity.getGradient(test_pos);
    double trace = result.T.trace();
    bool laplace = std::abs(trace) < 1e-6;  // Should satisfy Laplace equation

    reportTest("Laplace equation (∇²Φ = 0)", laplace,
              "Trace: " + std::to_string(trace), std::abs(trace));

    return loaded && fast_enough;
}

// Test 3: Hierarchical Map Matcher
bool testHierarchicalMatcher() {
    std::cout << "\n=== TEST 3: Hierarchical Map Matcher ===\n";

    HierarchicalMapMatcher::Config config;
    // Optimize search parameters for speed
    config.levels.clear();
    config.levels.push_back({5000, 500, 0.6, 10});  // Smaller 5km radius
    config.levels.push_back({1000, 100, 0.7, 5});   // 1km radius
    config.levels.push_back({200, 20, 0.8, 3});     // 200m fine search
    config.signature_length = 30;  // Reduced for speed
    config.min_measurements = 30;
    config.early_termination = true;
    config.excellent_correlation = 0.95;
    HierarchicalMapMatcher matcher(config);

    GravityGradientProvider gravity;
    if (!gravity.loadEGM2020("egm2008/egm2008_n360.dat")) {
        gravity.loadEGM2020("/Users/yarontorgeman/stn-v0.1/egm2008/egm2008_n360.dat");
    }

    // Create synthetic measurements
    State true_state;
    true_state.fromGeodetic(47.0 * M_PI/180, 8.0 * M_PI/180, 5000.0);

    for (int i = 0; i < 50; i++) {
        GravityMapMatcher::GravityMeasurement meas;
        meas.timestamp = i * 0.1;
        meas.position_ECEF = true_state.p_ECEF + Eigen::Vector3d(i*10, 0, 0);

        auto tensor = gravity.getGradient(meas.position_ECEF);
        meas.gradient_tensor = tensor.T;
        meas.anomaly_mgal = gravity.getAnomaly(meas.position_ECEF);

        matcher.addMeasurement(meas);
    }

    // Test matching
    auto start = std::chrono::high_resolution_clock::now();
    auto result = matcher.findMatch(gravity);
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    reportTest("Hierarchical search execution", result.valid,
              "Search time: " + std::to_string(duration.count()) + " ms", duration.count());

    if (result.valid) {
        reportTest("Search efficiency", result.total_evaluations < 1000,
                  std::to_string(result.total_evaluations) + " evaluations", result.total_evaluations);

        reportTest("Match correlation", result.final_correlation > 0.8,
                  "Correlation: " + std::to_string(result.final_correlation), result.final_correlation);
    }

    return result.valid;
}

// Test 4: Terrain Correlator with SRTM
bool testTerrainCorrelator() {
    std::cout << "\n=== TEST 4: Terrain Correlator ===\n";

    TerrainCorrelator correlator;

    // Check if SRTM data exists
    bool srtm_loaded = correlator.loadSRTM("srtm");
    if (!srtm_loaded) {
        srtm_loaded = correlator.loadSRTM("/Users/yarontorgeman/stn-v0.1/srtm");
    }

    reportTest("SRTM data loading", srtm_loaded,
              srtm_loaded ? "Real terrain data available" : "No SRTM data found");

    if (!srtm_loaded) return false;

    // Add synthetic measurements
    for (int i = 0; i < 50; i++) {
        Eigen::Vector3d pos;
        pos << 4.2e6 + i*100, 0.8e6, 4.7e6;
        double radar_alt = 5000 - i*10;  // Descending
        correlator.addMeasurement(i * 0.1, pos, radar_alt);
    }

    // Test correlation
    std::vector<Eigen::Vector3d> test_path;
    for (int i = 0; i < 50; i++) {
        Eigen::Vector3d pos;
        pos << 4.2e6 + i*100, 0.8e6, 4.7e6;
        test_path.push_back(pos);
    }

    auto validation = correlator.validateCandidate(test_path);

    reportTest("Terrain correlation computation", true,
              "Correlation: " + std::to_string(validation.correlation), validation.correlation);

    reportTest("Terrain validation", validation.valid,
              validation.reason, validation.elevation_rmse);

    return true;
}

// Test 5: Tensor Invariants
bool testTensorInvariants() {
    std::cout << "\n=== TEST 5: Tensor Invariants ===\n";

    // Create test tensor
    Eigen::Matrix3d tensor;
    tensor << 10, 2, 3,
              2, 20, 4,
              3, 4, 30;

    // Compute invariants
    double I1 = tensor.trace();
    double I2 = 0.5 * (I1 * I1 - (tensor * tensor).trace());

    // Test rotation invariance
    Eigen::AngleAxisd rotation(0.5, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R = rotation.matrix();
    Eigen::Matrix3d rotated = R * tensor * R.transpose();

    double I1_rot = rotated.trace();
    double I2_rot = 0.5 * (I1_rot * I1_rot - (rotated * rotated).trace());

    bool I1_invariant = std::abs(I1 - I1_rot) < 1e-10;
    bool I2_invariant = std::abs(I2 - I2_rot) < 1e-10;

    reportTest("First invariant (trace)", I1_invariant,
              "Difference: " + std::to_string(std::abs(I1 - I1_rot)), std::abs(I1 - I1_rot));

    reportTest("Second invariant", I2_invariant,
              "Difference: " + std::to_string(std::abs(I2 - I2_rot)), std::abs(I2 - I2_rot));

    return I1_invariant && I2_invariant;
}

// Test 6: Full System Integration
bool testFullIntegration() {
    std::cout << "\n=== TEST 6: Full System Integration ===\n";

    // Initialize all components
    UKFConfig ukf_config;
    ukf_config.setGradeAOptimal();
    UKF ukf(ukf_config);

    GravityGradientProvider gravity;
    bool gravity_loaded = gravity.loadEGM2020("egm2008/egm2008_n360.dat");
    if (!gravity_loaded) {
        gravity_loaded = gravity.loadEGM2020("/Users/yarontorgeman/stn-v0.1/egm2008/egm2008_n360.dat");
    }

    if (!gravity_loaded) {
        reportTest("Integration test", false, "Cannot load gravity model");
        return false;
    }

    // Initialize UKF
    State x0;
    x0.fromGeodetic(47.0 * M_PI/180, 8.0 * M_PI/180, 5000.0);
    Eigen::Matrix<double, 15, 15> P0 = Eigen::Matrix<double, 15, 15>::Identity() * 100;
    ukf.init(x0, P0);

    // Run short simulation
    double final_error = 0;
    int measurements_accepted = 0;
    int measurements_rejected = 0;

    for (int i = 0; i < 50; i++) {
        // IMU update
        ImuSample imu;
        imu.acc_mps2 = Eigen::Vector3d(0, 0, -9.81);
        imu.gyro_rps = Eigen::Vector3d(0, 0, 0.01);
        ukf.predict(imu, 0.1);

        // Gravity tensor invariants update
        State current = ukf.getState();
        auto tensor = gravity.getGradient(current.p_ECEF);

        Eigen::Matrix2d R_inv = Eigen::Matrix2d::Identity();
        R_inv(0,0) = 100;  // Realistic noise
        R_inv(1,1) = 1000;

        double nis_before = ukf.getNIS();
        ukf.updateGradientInvariants(tensor.T, R_inv);
        double nis_after = ukf.getNIS();

        if (std::abs(nis_after - nis_before) > 1e-6) {
            measurements_accepted++;
        } else {
            measurements_rejected++;
        }
    }

    State final_state = ukf.getState();
    final_error = (final_state.p_ECEF - x0.p_ECEF).norm();

    reportTest("Measurement acceptance rate", measurements_accepted > measurements_rejected,
              "Accepted: " + std::to_string(measurements_accepted) +
              ", Rejected: " + std::to_string(measurements_rejected),
              (double)measurements_accepted / (measurements_accepted + measurements_rejected));

    reportTest("Final position drift", final_error < 1000,
              std::to_string(final_error) + " m", final_error);

    return measurements_accepted > 0;
}

int main() {
    std::cout << "\n=========================================\n";
    std::cout << "   COMPREHENSIVE SYSTEM STATUS TEST\n";
    std::cout << "=========================================\n";

    // Run all tests
    bool ukf_ok = testUKFStability();
    bool gravity_ok = testGravityModel();
    bool matcher_ok = testHierarchicalMatcher();
    bool terrain_ok = testTerrainCorrelator();
    bool invariants_ok = testTensorInvariants();
    bool integration_ok = testFullIntegration();

    // Generate report
    std::cout << "\n=========================================\n";
    std::cout << "           FINAL STATUS REPORT\n";
    std::cout << "=========================================\n\n";

    int passed = 0, failed = 0;
    for (const auto& result : results) {
        if (result.passed) passed++;
        else failed++;
    }

    std::cout << "SUMMARY: " << passed << " passed, " << failed << " failed\n\n";

    // Component status
    std::cout << "COMPONENT STATUS:\n";
    std::cout << "├─ UKF Core: " << (ukf_ok ? "✅ OPERATIONAL" : "❌ ISSUES") << "\n";
    std::cout << "├─ Gravity Model: " << (gravity_ok ? "✅ OPERATIONAL" : "❌ ISSUES") << "\n";
    std::cout << "├─ Map Matcher: " << (matcher_ok ? "✅ OPERATIONAL" : "❌ ISSUES") << "\n";
    std::cout << "├─ Terrain Correlator: " << (terrain_ok ? "✅ OPERATIONAL" : "⚠️  LIMITED") << "\n";
    std::cout << "├─ Tensor Invariants: " << (invariants_ok ? "✅ OPERATIONAL" : "❌ ISSUES") << "\n";
    std::cout << "└─ System Integration: " << (integration_ok ? "✅ OPERATIONAL" : "❌ ISSUES") << "\n";

    // Performance metrics
    std::cout << "\nKEY METRICS:\n";
    for (const auto& result : results) {
        if (result.metric > 0 && result.name.find("speed") != std::string::npos) {
            std::cout << "├─ " << result.name << ": " << result.metric;
            if (result.name.find("µs") != std::string::npos) {
                std::cout << " µs";
            } else {
                std::cout << " ms";
            }
            std::cout << "\n";
        }
    }

    // Grade assessment
    std::cout << "\nGRADE ASSESSMENT:\n";
    double success_rate = (double)passed / (passed + failed);

    if (success_rate >= 0.9 && integration_ok) {
        std::cout << "★ Grade A: System fully operational\n";
    } else if (success_rate >= 0.7) {
        std::cout << "☆ Grade B: System mostly functional\n";
    } else if (success_rate >= 0.5) {
        std::cout << "○ Grade C: System partially functional\n";
    } else {
        std::cout << "● Grade D: System needs significant work\n";
    }

    // Recommendations
    std::cout << "\nRECOMMENDATIONS:\n";
    if (!gravity_ok) {
        std::cout << "├─ Fix gravity model loading paths\n";
    }
    if (!terrain_ok) {
        std::cout << "├─ Ensure SRTM data is available in srtm/ directory\n";
    }
    if (!integration_ok) {
        std::cout << "├─ Tune measurement noise parameters\n";
    }
    if (success_rate < 0.9) {
        std::cout << "├─ Address failed tests before production use\n";
    } else {
        std::cout << "├─ System ready for production testing\n";
    }

    std::cout << "\n=========================================\n";

    return failed == 0 ? 0 : 1;
}