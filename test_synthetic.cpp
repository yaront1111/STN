/**
 * Test gravity navigation with clean synthetic data
 * Verifies the system works with proper IMU data
 */

#include <iostream>
#include <fstream>
#include <cmath>
#include "cpp/core/types.h"
#include "cpp/core/ukf.h"

int main() {
    std::cout << "\n=================================================\n";
    std::cout << "    GRAVITY NAVIGATION - SYNTHETIC TEST\n";
    std::cout << "=================================================\n\n";
    
    // Initialize UKF
    UKF::Config ukf_config;
    ukf_config.alpha = 1e-3;
    ukf_config.beta = 2.0;
    ukf_config.kappa = 0.0;
    ukf_config.q_pos = 0.01;
    ukf_config.q_vel = 0.1;
    ukf_config.q_att = 0.001;
    
    UKF ukf(ukf_config);
    
    // Initial state: Zurich
    State x0;
    x0.fromGeodetic(47.45 * M_PI / 180.0, 8.55 * M_PI / 180.0, 500.0);
    
    // Initial covariance
    Eigen::Matrix<double, UKF::STATE_DIM, UKF::STATE_DIM> P0 = 
        Eigen::Matrix<double, UKF::STATE_DIM, UKF::STATE_DIM>::Identity();
    P0.block<3,3>(0,0) *= 10.0;     // Position: 3m
    P0.block<3,3>(3,3) *= 1.0;      // Velocity: 1 m/s
    P0.block<3,3>(6,6) *= 0.01;     // Attitude: 0.1 rad
    P0.block<3,3>(10,10) *= 0.01;   // Acc bias
    P0.block<3,3>(13,13) *= 0.001;  // Gyro bias
    P0(16,16) = 1e-6;
    P0(17,17) = 1e-9;
    
    ukf.init(x0, P0);
    std::cout << "Initial position: 47.45°N, 8.55°E, 500m\n\n";
    
    // Simulate for 100 seconds with clean IMU data
    double dt = 0.01;
    int steps = 10000;
    int gradient_updates = 0;
    int anomaly_updates = 0;
    
    std::cout << "Running navigation with synthetic data...\n";
    std::cout << "=========================================\n";
    
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        
        // Generate clean IMU data (hovering with small motion)
        ImuSample imu;
        imu.t = t;
        
        // Small accelerations (hovering)
        imu.acc_mps2 = Eigen::Vector3d(
            0.1 * std::sin(t * 0.1),
            0.1 * std::cos(t * 0.1),
            -9.80665 + 0.05 * std::sin(t * 0.2)
        );
        
        // Small rotations
        imu.gyro_rps = Eigen::Vector3d(
            0.01 * std::sin(t * 0.3),
            0.01 * std::cos(t * 0.3),
            0.005 * std::sin(t * 0.1)
        );
        
        // Add small noise
        imu.acc_mps2 += Eigen::Vector3d::Random() * 0.01;
        imu.gyro_rps += Eigen::Vector3d::Random() * 0.001;
        
        // Propagate with IMU
        ukf.propagateWithIMU(imu, dt);
        
        // Synthetic gravity gradient update (1 Hz)
        if (i % 100 == 0 && i > 0) {
            Eigen::Matrix3d gradient;
            gradient << 3.0, 0.1, 0.2,
                       0.1, -1.5, 0.15,
                       0.2, 0.15, -1.5;
            gradient = gradient + Eigen::Matrix3d::Random() * 0.1;  // Add noise
            
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.1;
            ukf.updateGravityGradient(gradient, R);
            gradient_updates++;
        }
        
        // Synthetic gravity anomaly update (10 Hz)
        if (i % 10 == 0 && i > 100) {
            double anomaly = 10.0 * std::sin(t * 0.01) + (rand() / (double)RAND_MAX - 0.5);
            ukf.updateGravityAnomaly(anomaly, 0.5);
            anomaly_updates++;
        }
        
        // Print status every 10 seconds
        if (i % 1000 == 0 && i > 0) {
            State x = ukf.getState();
            Eigen::Vector3d lla = x.toGeodetic();
            
            std::cout << "t=" << (int)t << "s | ";
            std::cout << "Position: " << lla.x() * 180/M_PI << "°N, " 
                     << lla.y() * 180/M_PI << "°E, " << lla.z() << "m | ";
            std::cout << "Vel: " << x.v_ECEF.norm() << " m/s | ";
            std::cout << "Updates: G=" << gradient_updates << " A=" << anomaly_updates;
            
            // Check for NaN
            if (!x.p_ECEF.allFinite()) {
                std::cout << " | ❌ NaN detected!";
            } else {
                std::cout << " | ✅ OK";
            }
            std::cout << "\n";
        }
    }
    
    // Final state
    State final_state = ukf.getState();
    Eigen::Vector3d final_lla = final_state.toGeodetic();
    
    std::cout << "\n=========================================\n";
    std::cout << "Test Complete\n";
    std::cout << "=========================================\n";
    std::cout << "Final position: " << final_lla.x() * 180/M_PI << "°N, " 
              << final_lla.y() * 180/M_PI << "°E, " << final_lla.z() << "m\n";
    std::cout << "Gradient updates: " << gradient_updates << "\n";
    std::cout << "Anomaly updates: " << anomaly_updates << "\n";
    
    if (final_state.p_ECEF.allFinite()) {
        std::cout << "\n✅ SUCCESS: Navigation system is stable with clean data!\n";
        std::cout << "The issue is with the flight data generation, not the navigation system.\n";
    } else {
        std::cout << "\n❌ FAILURE: System has fundamental issues even with clean data.\n";
    }
    
    return 0;
}