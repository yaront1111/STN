/**
 * Performance Benchmarking Suite for STN
 * Measures computational performance of critical components
 */

#include <iostream>
#include <chrono>
#include <vector>
#include <random>
#include <iomanip>
#include <algorithm>
#include <Eigen/Dense>

#include "ins.h"
#include "ekf.h"
#include "terrain_provider.h"
#include "gravity_model.h"
#include "../cpp/core/trn_update_adaptive.h"

using namespace std::chrono;

// Benchmark result structure
struct BenchmarkResult {
    std::string name;
    double mean_us;      // microseconds
    double std_us;       // standard deviation
    double min_us;
    double max_us;
    double percentile_95;
    double percentile_99;
    int iterations;
};

// Benchmark runner template
template<typename Func>
BenchmarkResult runBenchmark(const std::string& name, Func func, int warmup = 100, int iterations = 1000) {
    std::vector<double> timings;
    timings.reserve(iterations);
    
    // Warmup runs
    for (int i = 0; i < warmup; i++) {
        func();
    }
    
    // Timed runs
    for (int i = 0; i < iterations; i++) {
        auto start = high_resolution_clock::now();
        func();
        auto end = high_resolution_clock::now();
        
        auto duration = duration_cast<nanoseconds>(end - start).count() / 1000.0;  // to microseconds
        timings.push_back(duration);
    }
    
    // Calculate statistics
    std::sort(timings.begin(), timings.end());
    
    double sum = 0.0;
    for (double t : timings) sum += t;
    double mean = sum / timings.size();
    
    double sq_sum = 0.0;
    for (double t : timings) sq_sum += (t - mean) * (t - mean);
    double std_dev = std::sqrt(sq_sum / timings.size());
    
    BenchmarkResult result;
    result.name = name;
    result.mean_us = mean;
    result.std_us = std_dev;
    result.min_us = timings.front();
    result.max_us = timings.back();
    result.percentile_95 = timings[static_cast<int>(timings.size() * 0.95)];
    result.percentile_99 = timings[static_cast<int>(timings.size() * 0.99)];
    result.iterations = iterations;
    
    return result;
}

// Print benchmark results
void printResult(const BenchmarkResult& r) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Mean:     " << std::setw(8) << r.mean_us << " μs";
    std::cout << " | Std: " << std::setw(8) << r.std_us << " μs\n";
    std::cout << "  Min:      " << std::setw(8) << r.min_us << " μs";
    std::cout << " | Max: " << std::setw(8) << r.max_us << " μs\n";
    std::cout << "  P95:      " << std::setw(8) << r.percentile_95 << " μs";
    std::cout << " | P99: " << std::setw(8) << r.percentile_99 << " μs\n";
    std::cout << "  Rate:     " << std::setw(8) << (1000000.0 / r.mean_us) << " Hz\n";
}

// Benchmark implementations

void benchmarkINS() {
    std::cout << "\n=== INS Benchmarks ===\n";
    
    // Setup
    StrapdownINS ins({0.005, 9.80665});  // 200Hz
    State state;
    state.p_NED = Eigen::Vector3d(100, 200, -500);
    state.v_NED = Eigen::Vector3d(50, 0, 0);
    state.q_BN = Eigen::Quaterniond::Identity();
    
    // Random IMU data
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> dist(0.0, 0.1);
    
    Eigen::Vector3d f_B(dist(gen), dist(gen), -9.80665 + dist(gen));
    Eigen::Vector3d omega_B(dist(gen) * 0.01, dist(gen) * 0.01, dist(gen) * 0.01);
    
    // Benchmark INS step
    auto ins_step = [&]() {
        state = ins.step(state, f_B, omega_B);
    };
    
    std::cout << "\n1. INS Step (full propagation):\n";
    auto result = runBenchmark("INS Step", ins_step, 1000, 10000);
    printResult(result);
    
    // Benchmark quaternion integration only
    auto quat_int = [&]() {
        Eigen::Quaterniond q = state.q_BN;
        Eigen::Quaterniond omega_quat(0, omega_B.x(), omega_B.y(), omega_B.z());
        Eigen::Quaterniond q_dot = q * omega_quat * 0.5;
        q.coeffs() += q_dot.coeffs() * 0.005;
        q.normalize();
    };
    
    std::cout << "\n2. Quaternion Integration:\n";
    result = runBenchmark("Quat Int", quat_int, 1000, 10000);
    printResult(result);
}

void benchmarkEKF() {
    std::cout << "\n=== EKF Benchmarks ===\n";
    
    // Setup
    EKF ekf;
    State state;
    state.p_NED = Eigen::Vector3d(100, 200, -500);
    state.v_NED = Eigen::Vector3d(50, 0, 0);
    state.q_BN = Eigen::Quaterniond::Identity();
    ekf.init(state);
    ekf.setSingerParams(1.0, 30.0);
    
    // Random acceleration
    Eigen::Vector3d a_N(1.0, 0.5, 0.1);
    Eigen::Vector3d omega(0.01, 0.02, 0.03);
    
    // Benchmark predict step
    auto ekf_predict = [&]() {
        ekf.predict(a_N, omega, 0.005);
    };
    
    std::cout << "\n1. EKF Predict (15-state):\n";
    auto result = runBenchmark("EKF Predict", ekf_predict, 100, 1000);
    printResult(result);
    
    // Benchmark AGL update
    auto ekf_update_agl = [&]() {
        ekf.update_agl(ekf.x, 100.0, -400.0, 0.1);
    };
    
    std::cout << "\n2. EKF Update AGL:\n";
    result = runBenchmark("EKF AGL", ekf_update_agl, 100, 1000);
    printResult(result);
    
    // Benchmark gravity update
    auto ekf_update_gravity = [&]() {
        ekf.update_gravity(ekf.x, -9.81);
    };
    
    std::cout << "\n3. EKF Update Gravity:\n";
    result = runBenchmark("EKF Gravity", ekf_update_gravity, 100, 1000);
    printResult(result);
}

void benchmarkTerrain() {
    std::cout << "\n=== Terrain Provider Benchmarks ===\n";
    
    TerrainProvider terrain;
    
    // Random positions
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(-5000, 5000);
    
    // Benchmark terrain sampling
    auto terrain_sample = [&]() {
        double n = dist(gen);
        double e = dist(gen);
        volatile TerrainSample s = terrain.sample(n, e);
    };
    
    std::cout << "\n1. Terrain Sample (synthetic):\n";
    auto result = runBenchmark("Terrain Sample", terrain_sample, 1000, 10000);
    printResult(result);
    
    // Test with real SRTM if available
    if (terrain.use_real_terrain) {
        std::cout << "\n2. Terrain Sample (SRTM):\n";
        result = runBenchmark("SRTM Sample", terrain_sample, 1000, 10000);
        printResult(result);
    }
}

void benchmarkGravity() {
    std::cout << "\n=== Gravity Model Benchmarks ===\n";
    
    GravityModel::initialize();
    
    // Random locations
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> lat_dist(-M_PI/2, M_PI/2);
    std::uniform_real_distribution<> lon_dist(-M_PI, M_PI);
    std::uniform_real_distribution<> alt_dist(0, 10000);
    
    // Benchmark normal gravity
    auto gravity_normal = [&]() {
        GravityQuery q;
        q.lat_rad = lat_dist(gen);
        q.lon_rad = lon_dist(gen);
        q.alt_m = alt_dist(gen);
        volatile GravityResult r = GravityModel::normal(q);
    };
    
    std::cout << "\n1. Normal Gravity (WGS84):\n";
    auto result = runBenchmark("Normal Gravity", gravity_normal, 1000, 10000);
    printResult(result);
    
    // Benchmark gravity with anomaly
    auto gravity_anomaly = [&]() {
        GravityQuery q;
        q.lat_rad = lat_dist(gen);
        q.lon_rad = lon_dist(gen);
        q.alt_m = alt_dist(gen);
        volatile GravityResult r = GravityModel::withAnomaly(q);
    };
    
    std::cout << "\n2. Gravity with Anomaly:\n";
    result = runBenchmark("Gravity Anomaly", gravity_anomaly, 1000, 10000);
    printResult(result);
}

void benchmarkTRN() {
    std::cout << "\n=== TRN Update Benchmarks ===\n";
    
    // Setup
    EKF ekf;
    State state;
    state.p_NED = Eigen::Vector3d(1000, 2000, -500);
    state.v_NED = Eigen::Vector3d(50, 10, -1);
    state.q_BN = Eigen::Quaterniond::Identity();
    ekf.init(state);
    
    TerrainProvider terrain;
    TrnAdaptiveCfg cfg;
    cfg.sigma_agl = 1.0;
    cfg.slope_thresh = 0.1;
    cfg.alpha_base = 0.001;
    
    // Benchmark adaptive TRN update
    auto trn_update = [&]() {
        double agl_meas = 100.0;
        volatile bool accepted = trn_update_adaptive(ekf, state, terrain, agl_meas, cfg);
    };
    
    std::cout << "\n1. Adaptive TRN Update:\n";
    auto result = runBenchmark("TRN Update", trn_update, 100, 1000);
    printResult(result);
}

void benchmarkFullSystem() {
    std::cout << "\n=== Full System Benchmark ===\n";
    
    // Setup complete navigation system
    StrapdownINS ins({0.005, 9.80665});
    EKF ekf;
    State state;
    state.p_NED = Eigen::Vector3d(100, 200, -500);
    state.v_NED = Eigen::Vector3d(50, 0, 0);
    state.q_BN = Eigen::Quaterniond::Identity();
    ekf.init(state);
    
    TerrainProvider terrain;
    GravityModel::initialize();
    
    // Random IMU data
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> dist(0.0, 0.1);
    
    // Full navigation cycle
    auto full_cycle = [&]() {
        // IMU step (200 Hz)
        Eigen::Vector3d f_B(dist(gen), dist(gen), -9.80665);
        Eigen::Vector3d omega_B(dist(gen) * 0.01, dist(gen) * 0.01, dist(gen) * 0.01);
        
        state = ins.step(state, f_B, omega_B);
        
        // EKF predict
        Eigen::Vector3d a_N = state.q_BN * f_B;
        ekf.predict(a_N, omega_B, 0.005);
        
        // Occasional TRN update (2 Hz)
        static int counter = 0;
        if (++counter >= 100) {
            counter = 0;
            TerrainSample ts = terrain.sample(state.p_NED.x(), state.p_NED.y());
            double agl = -state.p_NED.z() - ts.h;
            ekf.update_agl(ekf.x, agl, ts.h, ts.grad.norm());
        }
    };
    
    std::cout << "\n1. Full Navigation Cycle (200Hz IMU + 2Hz TRN):\n";
    auto result = runBenchmark("Full Cycle", full_cycle, 1000, 10000);
    printResult(result);
    
    // Calculate system capacity
    double cycle_time_ms = result.mean_us / 1000.0;
    double max_rate_hz = 1000.0 / cycle_time_ms;
    
    std::cout << "\n  System Capacity:\n";
    std::cout << "    Single-core max rate: " << std::setw(8) << max_rate_hz << " Hz\n";
    std::cout << "    Headroom at 200Hz:    " << std::setw(8) 
              << ((5.0 - cycle_time_ms) / 5.0 * 100) << " %\n";
    
    if (max_rate_hz > 1000) {
        std::cout << "    ✅ EXCELLENT: Can handle >1000Hz IMU\n";
    } else if (max_rate_hz > 400) {
        std::cout << "    ✅ GOOD: Can handle 400Hz IMU\n";
    } else if (max_rate_hz > 200) {
        std::cout << "    ⚠️  ADEQUATE: Can handle 200Hz IMU\n";
    } else {
        std::cout << "    ❌ INSUFFICIENT: Cannot handle 200Hz IMU\n";
    }
}

int main() {
    std::cout << "=====================================\n";
    std::cout << "   STN Performance Benchmark Suite   \n";
    std::cout << "=====================================\n";
    
    std::cout << "\nSystem Info:\n";
    std::cout << "  CPU cores: " << std::thread::hardware_concurrency() << "\n";
    std::cout << "  Eigen version: " << EIGEN_WORLD_VERSION << "." 
              << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << "\n";
    
    // Run benchmarks
    benchmarkINS();
    benchmarkEKF();
    benchmarkTerrain();
    benchmarkGravity();
    benchmarkTRN();
    benchmarkFullSystem();
    
    std::cout << "\n=====================================\n";
    std::cout << "   Benchmark Complete                \n";
    std::cout << "=====================================\n\n";
    
    return 0;
}