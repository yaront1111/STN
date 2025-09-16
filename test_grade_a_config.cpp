/**
 * GRADE A PERFORMANCE TEST WITH CALIBRATED CONFIGURATION
 * 
 * Test the navigation system using the properly calibrated
 * measurement noise parameters from the Grade A+ analysis
 */

#include <iostream>
#include <fstream>
#include <random>
#include <chrono>
#include <iomanip>
#include <Eigen/Dense>
#include "cpp/core/types.h"
#include "cpp/core/ukf.h"
#include "cpp/core/ukf_config.h"
#include "cpp/core/gravity_gradient_provider.h"
#include "cpp/core/gravity_map_matcher.h"

class GradeASimulator {
public:
    State true_state;
    double t = 0;
    
    // High-fidelity IMU model
    struct IMU {
        double gyro_ARW = 0.003 * M_PI / 180 / 60;
        double gyro_bias_instability = 0.01 * M_PI / 180 / 3600;
        double acc_VRW = 0.05e-3 * 9.81 / 60;
        double acc_bias_instability = 0.1e-3 * 9.81 / 3600;
        
        Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
        Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
    } imu;
    
    void init(double lat_deg, double lon_deg, double alt_m) {
        true_state.fromGeodetic(lat_deg * M_PI / 180, lon_deg * M_PI / 180, alt_m);
        true_state.v_ECEF = Eigen::Vector3d::Zero();
        true_state.q_ECEF_B = Eigen::Quaterniond::Identity();
        true_state.b_a = Eigen::Vector3d::Zero();
        true_state.b_g = Eigen::Vector3d::Zero();
        true_state.t = 0;
    }
    
    ImuSample step(double dt) {
        // Simple Figure-8 maneuver
        double omega = 0.05;  // rad/s
        double v_forward = 150.0;  // m/s
        
        // Figure-8 velocity profile
        double lateral_accel = 2.0 * std::sin(2 * omega * t);
        
        // Update true state
        Eigen::Vector3d accel_body(0, lateral_accel, 0);
        Eigen::Vector3d vel_change = true_state.q_ECEF_B * accel_body * dt;
        true_state.v_ECEF += vel_change;
        true_state.p_ECEF += true_state.v_ECEF * dt;
        
        // IMU measurement with realistic noise
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<> noise(0, 1);
        
        ImuSample imu_sample;
        imu_sample.t = t;
        imu_sample.a_B = accel_body + imu.acc_bias + 
                        Eigen::Vector3d(noise(gen), noise(gen), noise(gen)) * imu.acc_VRW / std::sqrt(dt);
        imu_sample.w_B = Eigen::Vector3d(0, 0, omega) + imu.gyro_bias +
                        Eigen::Vector3d(noise(gen), noise(gen), noise(gen)) * imu.gyro_ARW / std::sqrt(dt);
        
        t += dt;
        true_state.t = t;
        
        return imu_sample;
    }
};

int main() {
    std::cout << "╔════════════════════════════════════════════╗\n";
    std::cout << "║     GRADE A+ CALIBRATED PERFORMANCE TEST   ║\n";
    std::cout << "║     Target: <50m accuracy without GPS      ║\n";
    std::cout << "╚════════════════════════════════════════════╝\n\n";
    
    // Initialize gravity model
    GravityGradientProvider gravity_model;
    std::cout << "✓ Gravity model initialized with REAL EGM2008 data\n";
    
    // Initialize simulator
    GradeASimulator sim;
    sim.init(47.3977, 8.5456, 5000.0);
    std::cout << "✓ Simulator initialized (Figure-8 maneuvers)\n";
    
    // GRADE A+ CALIBRATED Configuration
    UKFConfig grade_a_config;
    grade_a_config.setGradeAOptimal();
    
    std::cout << "✓ Grade A+ Configuration Applied:\n";
    std::cout << "  - Gravity Gradient Noise: " << std::sqrt(grade_a_config.measurement_noise.gravity_gradient) << " Eötvös\n";
    std::cout << "  - Gravity Anomaly Noise:  " << std::sqrt(grade_a_config.measurement_noise.gravity_anomaly) << " mGal\n";
    std::cout << "  - Map Match Uncertainty:  " << grade_a_config.measurement_noise.map_match_position << " m\n";
    
    // Create old-style config for compatibility
    UKF::Config ukf_config;
    ukf_config.alpha = grade_a_config.alpha;
    ukf_config.beta = grade_a_config.beta;  
    ukf_config.kappa = grade_a_config.kappa;
    ukf_config.sigma_pos = grade_a_config.process_noise.position;
    ukf_config.sigma_vel = grade_a_config.process_noise.velocity;
    ukf_config.sigma_att = grade_a_config.process_noise.attitude;
    ukf_config.sigma_ba = grade_a_config.process_noise.accel_bias;
    ukf_config.sigma_bg = grade_a_config.process_noise.gyro_bias;
    
    UKF ukf(ukf_config);
    std::cout << "✓ UKF initialized with Grade A+ calibrated parameters\n";
    
    // Initialize filter
    State x0 = sim.true_state;
    x0.p_ECEF += Eigen::Vector3d(30, 30, 10);
    x0.v_ECEF += Eigen::Vector3d(1, 1, 0.5);
    
    Eigen::Matrix<double, UKF::ERROR_STATE_DIM, UKF::ERROR_STATE_DIM> P0 = 
        Eigen::Matrix<double, UKF::ERROR_STATE_DIM, UKF::ERROR_STATE_DIM>::Identity();
    P0.block<3,3>(0,0) *= 100;
    P0.block<3,3>(3,3) *= 10;
    P0.block<3,3>(6,6) *= 0.1;
    P0.block<3,3>(9,9) *= 0.01;
    P0.block<3,3>(12,12) *= 0.001;
    
    ukf.init(x0, P0);
    
    // Initialize map matcher
    GravityMapMatcher map_matcher(gravity_model);
    GravityMapMatcher::Config matcher_config;
    matcher_config.search_radius_km = 2.0;
    matcher_config.grid_resolution_m = 500.0;
    matcher_config.min_measurements = 20;
    map_matcher.configure(matcher_config);
    std::cout << "✓ Map matcher configured\n";
    
    // Run simulation
    std::cout << "\nRunning 60-second test with calibrated parameters...\n";
    std::cout << "─────────────────────────────────────────────\n";
    
    auto start_time = std::chrono::high_resolution_clock::now();
    double dt = 0.01;
    int steps = 6000;
    
    struct Performance {
        double max_error = 0;
        double sum_error = 0;
        int map_matches = 0;
        double last_match_time = 0;
    } perf;
    
    for (int i = 0; i < steps; ++i) {
        double current_time = i * dt;
        
        // IMU prediction
        ImuSample imu = sim.step(dt);
        ukf.predict(imu, dt);
        
        // Gravity gradient (10 Hz) with CALIBRATED noise
        if (i % 10 == 0) {
            State current = ukf.getState();
            auto tensor = gravity_model.getGradient(current.p_ECEF);
            
            if (tensor.T.allFinite() && tensor.T.norm() < 100.0) {
                // Use properly calibrated gradient noise  
                double gradient_std = std::sqrt(grade_a_config.measurement_noise.gravity_gradient);
                Eigen::Matrix3d R_grad = Eigen::Matrix3d::Identity() * (gradient_std * gradient_std);
                ukf.updateGradient(tensor.T, R_grad);
            }
            
            // Add to map matcher
            GravityMapMatcher::GravityMeasurement meas;
            meas.timestamp = current_time;
            meas.position_ECEF = current.p_ECEF;
            meas.anomaly_mgal = tensor.T.trace() * 1e9;
            meas.gradient_trace = tensor.T.trace();
            map_matcher.addMeasurement(meas);
        }
        
        // Gravity anomaly (50 Hz) with CALIBRATED noise
        if (i % 2 == 0) {
            State current = ukf.getState();
            auto tensor = gravity_model.getGradient(current.p_ECEF);
            double anomaly = tensor.T.trace() * 1e9;
            
            if (std::abs(anomaly) > 0.1 && std::isfinite(anomaly)) {
                double anomaly_std = std::sqrt(grade_a_config.measurement_noise.gravity_anomaly);
                ukf.updateAnomaly(anomaly, anomaly_std * anomaly_std);
            }
        }
        
        // Map matching with calibrated uncertainty
        if (current_time > 5.0 && current_time - perf.last_match_time >= grade_a_config.grade_a.map_match_interval && 
            map_matcher.getSignatureLength() >= matcher_config.min_measurements) {
            
            auto match_result = map_matcher.findMatch(gravity_model);
            if (match_result.valid && match_result.confidence > 0.85) {
                Eigen::Matrix3d R_pos = Eigen::Matrix3d::Identity() * 
                    (grade_a_config.measurement_noise.map_match_position * grade_a_config.measurement_noise.map_match_position);
                
                ukf.updateGravityMapMatch(match_result.matched_position_ECEF, R_pos);
                perf.map_matches++;
                perf.last_match_time = current_time;
                map_matcher.reset();
                
                State current = ukf.getState();
                double error = (current.p_ECEF - sim.true_state.p_ECEF).norm();
                std::cout << std::fixed << std::setprecision(1);
                std::cout << "t=" << current_time << "s: MAP MATCH #" << perf.map_matches;
                std::cout << " | Error: " << error << "m | Confidence: " << match_result.confidence << "\n";
            }
        }
        
        // Calculate performance metrics
        State current = ukf.getState();
        double pos_error = (current.p_ECEF - sim.true_state.p_ECEF).norm();
        perf.max_error = std::max(perf.max_error, pos_error);
        perf.sum_error += pos_error;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    // Final results
    double avg_error = perf.sum_error / steps;
    double real_time_factor = 60000.0 / duration.count();
    
    std::cout << "\n╔════════════════════════════════════════════╗\n";
    std::cout << "║           GRADE A+ RESULTS                 ║\n";
    std::cout << "╠════════════════════════════════════════════╣\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "║ Average Error:     " << std::setw(8) << avg_error << " m          ║\n";
    std::cout << "║ Maximum Error:     " << std::setw(8) << perf.max_error << " m          ║\n";
    std::cout << "║ Map Matches:       " << std::setw(8) << perf.map_matches << "             ║\n";
    std::cout << "║ Runtime:           " << std::setw(8) << duration.count() << " ms         ║\n";
    std::cout << "║ Real-time Factor:  " << std::setw(8) << real_time_factor << "x            ║\n";
    std::cout << "╠════════════════════════════════════════════╣\n";
    
    // Grade assessment
    std::string grade = "F";
    std::string status = "FAILED";
    if (avg_error < 50.0) {
        grade = "A+";
        status = "GRADE A+ ACHIEVED!";
    } else if (avg_error < 100.0) {
        grade = "A";
        status = "CLOSE TO TARGET";
    } else if (avg_error < 500.0) {
        grade = "B";
        status = "GOOD PERFORMANCE";
    } else if (avg_error < 1000.0) {
        grade = "C";
        status = "ACCEPTABLE";
    }
    
    std::cout << "║ GRADE:   " << std::setw(15) << grade << "               ║\n";
    std::cout << "║ " << std::setw(30) << status << "  ║\n";
    std::cout << "╚════════════════════════════════════════════╝\n\n";
    
    // Analysis
    double improvement_vs_baseline = 11551.0 / avg_error; // vs previous baseline
    std::cout << "📊 ANALYSIS:\n";
    std::cout << "   - Grade A+ Target:    <50m position accuracy\n";
    std::cout << "   - Current Performance: " << avg_error << "m average error\n";
    std::cout << "   - Improvement Factor:  " << improvement_vs_baseline << "x better than baseline\n";
    std::cout << "   - Calibrated Noise:    Applied from measurement analysis\n";
    
    return 0;
}