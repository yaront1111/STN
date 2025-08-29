/**
 * GPS-DENIED NAVIGATION - TUNED VERSION
 * 
 * Realistic parameters and update rates
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <Eigen/Dense>
#include "cpp/core/types.h"
#include "cpp/core/ukf.h"
#include "cpp/core/gravity_gradient_provider.h"

struct RealisticSimulator {
    double t = 0;
    State true_state;
    std::mt19937 rng{42};
    
    // High-quality IMU (tactical grade)
    std::normal_distribution<> acc_noise{0, 0.001};    // 1 mg
    std::normal_distribution<> gyro_noise{0, 0.00001}; // 0.01 deg/hr
    std::normal_distribution<> mag_noise{0, 50e-9};    // 50 nT
    std::normal_distribution<> baro_noise{0, 0.5};     // 0.5m
    
    void init(double lat_deg, double lon_deg, double alt_m) {
        true_state.fromGeodetic(lat_deg * M_PI/180, lon_deg * M_PI/180, alt_m);
        true_state.v_ECEF = Eigen::Vector3d(50, 0, 0);  // 50 m/s eastward
        t = 0;
    }
    
    ImuSample generateIMU(double dt) {
        ImuSample imu;
        
        // Realistic aircraft dynamics
        true_state.p_ECEF += true_state.v_ECEF * dt;
        
        // Very slow heading drift (1 deg/hr = 0.00027 rad/s)
        double heading_drift_rate = 0.00027;
        Eigen::AngleAxisd drift(heading_drift_rate * dt, Eigen::Vector3d::UnitZ());
        true_state.q_ECEF_B = true_state.q_ECEF_B * drift;
        
        // Generate IMU with gravity
        Eigen::Vector3d gravity_ECEF(0, 0, -9.81);
        Eigen::Vector3d gravity_body = true_state.q_ECEF_B.inverse() * gravity_ECEF;
        
        imu.acc_mps2 = gravity_body;
        imu.acc_mps2.x() += acc_noise(rng);
        imu.acc_mps2.y() += acc_noise(rng);
        imu.acc_mps2.z() += acc_noise(rng);
        
        imu.gyro_rps = Eigen::Vector3d(0, 0, heading_drift_rate);
        imu.gyro_rps.x() += gyro_noise(rng);
        imu.gyro_rps.y() += gyro_noise(rng);
        imu.gyro_rps.z() += gyro_noise(rng);
        
        t += dt;
        return imu;
    }
    
    Eigen::Vector3d generateMagnetometer() {
        // Realistic Earth field
        Eigen::Vector3d mag_ECEF(20e-6, 5e-6, -40e-6);
        Eigen::Vector3d mag_body = true_state.q_ECEF_B.inverse() * mag_ECEF;
        
        mag_body.x() += mag_noise(rng);
        mag_body.y() += mag_noise(rng);
        mag_body.z() += mag_noise(rng);
        
        return mag_body;
    }
    
    double generateBarometer() {
        Eigen::Vector3d lla = true_state.toGeodetic();
        return lla.z() + baro_noise(rng);
    }
};

int main() {
    std::cout << "=========================================\n";
    std::cout << "GPS-DENIED NAVIGATION - TUNED VERSION\n";
    std::cout << "=========================================\n\n";
    
    // Initialize gravity model
    GravityGradientProvider gravity_provider;
    
    // Single optimized configuration
    std::cout << "Configuration:\n";
    std::cout << "  - High-rate magnetometer (100 Hz)\n";
    std::cout << "  - Barometric altimeter (10 Hz)\n";
    std::cout << "  - Gravity gradients (100 Hz)\n";
    std::cout << "  - Low process noise\n\n";
    
    // Initialize simulator
    RealisticSimulator sim;
    sim.init(47.0, 8.0, 1000.0);
    
    // Initialize UKF with tuned parameters
    UKF::Config ukf_config;
    ukf_config.alpha = 0.001;
    ukf_config.beta = 2.0;
    ukf_config.kappa = 0.0;
    
    // CRITICAL: Low process noise for stability
    ukf_config.sigma_pos = 0.01;    // 1cm/√s
    ukf_config.sigma_vel = 0.1;     // 10cm/s/√s  
    ukf_config.sigma_att = 0.0001;  // Very low attitude noise
    ukf_config.sigma_ba = 1e-6;     // Very stable bias
    ukf_config.sigma_bg = 1e-7;     // Very stable gyro bias
    
    UKF ukf(ukf_config);
    
    // Initial state with small error
    State x0 = sim.true_state;
    x0.p_ECEF += Eigen::Vector3d(5, 5, 2);  // 5m horizontal, 2m vertical
    
    // Tight initial covariance
    Eigen::Matrix<double, 15, 15> P0 = Eigen::Matrix<double, 15, 15>::Identity();
    P0.block<3,3>(0,0) *= 25.0;    // 5m position
    P0.block<3,3>(3,3) *= 0.1;     // 0.3 m/s velocity
    P0.block<3,3>(6,6) *= 0.001;   // 0.03 rad attitude
    P0.block<3,3>(9,9) *= 1e-6;    // Small acc bias
    P0.block<3,3>(12,12) *= 1e-8;  // Small gyro bias
    
    ukf.init(x0, P0);
    
    // Accurate magnetic field reference
    Eigen::Vector3d mag_ref_ECEF(20e-6, 5e-6, -40e-6);
    
    // Simulation
    double dt = 0.01;  // 100 Hz
    double sim_time = 60.0;
    int steps = sim_time / dt;
    
    // Output file
    std::ofstream results("gps_denied_tuned.csv");
    results << "Time,TrueX,TrueY,TrueZ,EstX,EstY,EstZ,Error,HeadingError\n";
    
    // Statistics
    double max_error = 0;
    double sum_error = 0;
    int count = 0;
    
    for (int i = 0; i < steps; i++) {
        // IMU at 100 Hz
        ImuSample imu = sim.generateIMU(dt);
        ukf.predict(imu, dt);
        
        // Gravity gradient at 100 Hz
        State current = ukf.getState();
        GravityGradientTensor tensor = gravity_provider.getGradient(current.p_ECEF);
        Eigen::Matrix3d R_grad = Eigen::Matrix3d::Identity() * 1e-20;  // Very low noise
        ukf.updateGradient(tensor.T, R_grad);
        
        // CRITICAL: High-rate magnetometer for heading
        if (i % 1 == 0) {  // 100 Hz!
            Eigen::Vector3d mag = sim.generateMagnetometer();
            Eigen::Matrix3d R_mag = Eigen::Matrix3d::Identity() * 2.5e-15;  // 50 nT²
            ukf.updateMagnetometer(mag, mag_ref_ECEF, R_mag);
        }
        
        // Barometer at 10 Hz
        if (i % 10 == 0) {
            double baro_alt = sim.generateBarometer();
            ukf.updateBarometer(baro_alt, 0.25);  // 0.5m noise
        }
        
        // Calculate errors
        State est = ukf.getState();
        double pos_error = (est.p_ECEF - sim.true_state.p_ECEF).norm();
        
        // Heading error
        Eigen::Matrix3d C_true = sim.true_state.q_ECEF_B.toRotationMatrix();
        Eigen::Matrix3d C_est = est.q_ECEF_B.toRotationMatrix();
        Eigen::Matrix3d C_error = C_true.transpose() * C_est;
        double heading_error = std::acos((C_error.trace() - 1) / 2) * 180 / M_PI;
        
        max_error = std::max(max_error, pos_error);
        sum_error += pos_error;
        count++;
        
        // Log every 0.1 seconds
        if (i % 10 == 0) {
            results << sim.t << ",";
            results << sim.true_state.p_ECEF.x() << ",";
            results << sim.true_state.p_ECEF.y() << ",";
            results << sim.true_state.p_ECEF.z() << ",";
            results << est.p_ECEF.x() << ",";
            results << est.p_ECEF.y() << ",";
            results << est.p_ECEF.z() << ",";
            results << pos_error << ",";
            results << heading_error << "\n";
            
            if (i % 100 == 0) {  // Every second
                std::cout << "t=" << sim.t << "s";
                std::cout << " | Pos error: " << pos_error << "m";
                std::cout << " | Heading error: " << heading_error << "°\n";
            }
        }
    }
    
    results.close();
    
    // Final results
    double avg_error = sum_error / count;
    std::cout << "\n=========================================\n";
    std::cout << "FINAL RESULTS:\n";
    std::cout << "  Average position error: " << avg_error << " m\n";
    std::cout << "  Maximum position error: " << max_error << " m\n";
    
    // Grade
    std::string grade;
    if (avg_error < 10) grade = "A+ (Sub-10m without GPS!)";
    else if (avg_error < 50) grade = "A (Excellent GPS-denied)";
    else if (avg_error < 100) grade = "B+ (Very good)";
    else if (avg_error < 500) grade = "B (Good)";
    else grade = "C (Needs improvement)";
    
    std::cout << "  GRADE: " << grade << "\n";
    std::cout << "=========================================\n";
    
    return 0;
}