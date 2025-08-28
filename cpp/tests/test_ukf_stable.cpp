#include "../core/ukf_stable.h"
#include "../core/gravity_gradient_provider.h"
#include "../core/gravity_model.h"
#include <iostream>
#include <iomanip>
#include <random>
#include <chrono>

void printState(const State& state, const std::string& label) {
    std::cout << "\n=== " << label << " ===\n";
    std::cout << "Position (ECEF): " << state.p_ECEF.transpose() << " m\n";
    std::cout << "Velocity (ECEF): " << state.v_ECEF.transpose() << " m/s\n";
    std::cout << "Quaternion: " << state.q_ECEF_B.w() << " " 
              << state.q_ECEF_B.x() << " " 
              << state.q_ECEF_B.y() << " " 
              << state.q_ECEF_B.z() << "\n";
    std::cout << "Acc bias: " << state.b_a.transpose() << " m/s²\n";
    std::cout << "Gyro bias: " << state.b_g.transpose() << " rad/s\n";
}

void printCovariance(const Eigen::Matrix<double, 15, 15>& P, const std::string& label) {
    std::cout << "\n" << label << " - Covariance diagonal:\n";
    std::cout << "Pos std: " << std::sqrt(P(0,0)) << " " << std::sqrt(P(1,1)) << " " << std::sqrt(P(2,2)) << " m\n";
    std::cout << "Vel std: " << std::sqrt(P(3,3)) << " " << std::sqrt(P(4,4)) << " " << std::sqrt(P(5,5)) << " m/s\n";
    std::cout << "Att std: " << std::sqrt(P(6,6)) << " " << std::sqrt(P(7,7)) << " " << std::sqrt(P(8,8)) << " rad\n";
    std::cout << "Ba std: " << std::sqrt(P(9,9)) << " " << std::sqrt(P(10,10)) << " " << std::sqrt(P(11,11)) << " m/s²\n";
    std::cout << "Bg std: " << std::sqrt(P(12,12)) << " " << std::sqrt(P(13,13)) << " " << std::sqrt(P(14,14)) << " rad/s\n";
    
    // Check for NaN/Inf
    if (!P.allFinite()) {
        std::cerr << "WARNING: Covariance has NaN/Inf values!\n";
    }
    
    // Check positive definiteness
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(P);
    double min_eigenvalue = es.eigenvalues().minCoeff();
    if (min_eigenvalue < 0) {
        std::cerr << "WARNING: Covariance not positive definite! Min eigenvalue: " << min_eigenvalue << "\n";
    }
}

int main() {
    std::cout << "Testing Stable UKF with Error-State Formulation\n";
    std::cout << "================================================\n";
    
    // Initialize gravity models
    GravityModel::initialize();
    GravityGradientProvider gradient_provider;
    gradient_provider.loadEGM2020("data/egm2020_coeffs.dat");
    
    // Configure UKF
    UKF_Stable::Config ukf_cfg;
    ukf_cfg.alpha = 1e-3;
    ukf_cfg.beta = 2.0;
    ukf_cfg.kappa = 0.0;
    ukf_cfg.sigma_pos = 0.1;
    ukf_cfg.sigma_vel = 0.5;
    ukf_cfg.sigma_att = 0.01;
    ukf_cfg.sigma_ba = 1e-4;
    ukf_cfg.sigma_bg = 1e-5;
    
    UKF_Stable ukf(ukf_cfg);
    
    // Initial state: stationary at equator, 10km altitude
    State x0;
    x0.fromGeodetic(0.0, 0.0, 10000.0);
    x0.v_ECEF = Eigen::Vector3d::Zero();
    x0.q_ECEF_B = Eigen::Quaterniond::Identity();
    x0.b_a = Eigen::Vector3d::Zero();
    x0.b_g = Eigen::Vector3d::Zero();
    x0.t = 0.0;
    
    // Initial covariance (15x15 error state)
    Eigen::Matrix<double, 15, 15> P0 = Eigen::Matrix<double, 15, 15>::Identity();
    P0.block<3,3>(0,0) *= 100.0;    // 10m position uncertainty
    P0.block<3,3>(3,3) *= 1.0;      // 1m/s velocity uncertainty
    P0.block<3,3>(6,6) *= 0.01;     // 0.1 rad attitude uncertainty
    P0.block<3,3>(9,9) *= 1e-4;     // Small bias uncertainties
    P0.block<3,3>(12,12) *= 1e-6;
    
    ukf.init(x0, P0);
    printState(x0, "Initial State");
    printCovariance(P0, "Initial");
    
    // Simulation parameters
    double dt = 0.01;  // 100 Hz
    double total_time = 10.0;  // 10 seconds
    int steps = total_time / dt;
    
    // Random number generators for noise
    std::default_random_engine generator;
    std::normal_distribution<double> acc_noise(0.0, 0.01);  // 0.01 m/s² noise
    std::normal_distribution<double> gyro_noise(0.0, 0.001); // 0.001 rad/s noise
    
    // True trajectory: gentle circular motion
    double omega = 0.1;  // rad/s
    
    std::cout << "\nStarting simulation...\n";
    
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        
        // Generate true IMU measurements (circular motion + noise)
        ImuSample imu;
        imu.acc_mps2 = Eigen::Vector3d(
            -omega*omega * 10.0 + acc_noise(generator),  // Centripetal acceleration
            acc_noise(generator),
            9.8 + acc_noise(generator)  // Gravity
        );
        imu.gyro_rps = Eigen::Vector3d(
            gyro_noise(generator),
            gyro_noise(generator),
            omega + gyro_noise(generator)  // Rotation about z-axis
        );
        imu.t = t;
        imu.temperature_c = 25.0;
        
        // Predict step
        ukf.predict(imu, dt);
        
        // Every 1 second, do a measurement update (but skip first one to let filter initialize)
        if (i % 100 == 0 && i > 0) {
            State current_state = ukf.getState();
            
            // Simulate gravity gradient measurement
            auto true_gradient = gradient_provider.getGradient(current_state.p_ECEF);
            
            // Add measurement noise (0.1 Eötvös)
            Eigen::Matrix3d measured_gradient = true_gradient.T;
            measured_gradient += 0.1 * Eigen::Matrix3d::Random();
            
            // Measurement noise covariance
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.01;  // (0.1 Eötvös)²
            
            // Update
            ukf.updateGradient(measured_gradient, R);
            
            // Also update with gravity anomaly
            double true_anomaly = gradient_provider.getAnomaly(current_state.p_ECEF);
            double measured_anomaly = true_anomaly + std::normal_distribution<double>(0.0, 0.5)(generator);
            ukf.updateAnomaly(measured_anomaly, 0.25);  // 0.5² = 0.25 variance
            
            std::cout << "\n--- t=" << t << "s: Measurement Update ---\n";
        }
        
        // Print status every second
        if (i % 100 == 0) {
            State current = ukf.getState();
            auto P = ukf.getCovariance();
            
            std::cout << "t=" << std::fixed << std::setprecision(1) << t << "s: ";
            std::cout << "Pos=[" << std::setprecision(0) 
                      << current.p_ECEF.x() << "," 
                      << current.p_ECEF.y() << "," 
                      << current.p_ECEF.z() << "] ";
            std::cout << "Vel_norm=" << std::setprecision(2) << current.v_ECEF.norm() << " m/s";
            
            // Check health
            if (!P.allFinite()) {
                std::cerr << "\nERROR: Covariance became NaN at t=" << t << "s\n";
                printCovariance(P, "Failed");
                return 1;
            }
            
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(P);
            double min_eig = es.eigenvalues().minCoeff();
            if (min_eig < -1e-10) {
                std::cerr << "\nWARNING: Negative eigenvalue " << min_eig << " at t=" << t << "s\n";
            }
            std::cout << " Min_eig=" << std::scientific << min_eig << std::fixed << "\n";
        }
    }
    
    // Final results
    State final_state = ukf.getState();
    auto final_P = ukf.getCovariance();
    
    printState(final_state, "Final State");
    printCovariance(final_P, "Final");
    
    // Check if filter remained stable
    if (final_P.allFinite()) {
        std::cout << "\n✓ SUCCESS: UKF remained numerically stable throughout simulation!\n";
        
        // Check convergence
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(final_P);
        double max_eig = es.eigenvalues().maxCoeff();
        double min_eig = es.eigenvalues().minCoeff();
        double condition_number = max_eig / min_eig;
        
        std::cout << "Condition number: " << condition_number << "\n";
        if (condition_number < 1e6) {
            std::cout << "✓ Good numerical conditioning\n";
        } else {
            std::cout << "⚠ Poor conditioning (may need tuning)\n";
        }
        
        return 0;
    } else {
        std::cerr << "\n✗ FAILURE: UKF became unstable\n";
        return 1;
    }
}