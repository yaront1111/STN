/**
 * GPS-DENIED NAVIGATION VALIDATION TEST
 * 
 * Demonstrates improved accuracy with magnetometer, barometer,
 * terrain correlation, and ZUPT
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <Eigen/Dense>
#include "cpp/core/types.h"
#include "cpp/core/ukf.h"
#include "cpp/core/gravity_gradient_provider.h"

// Simulate realistic flight trajectory
struct FlightSimulator {
    double t = 0;
    State true_state;
    std::mt19937 rng{42};
    
    // Noise distributions
    std::normal_distribution<> acc_noise{0, 0.01};   // 0.01 m/s² 
    std::normal_distribution<> gyro_noise{0, 0.001}; // 0.001 rad/s
    std::normal_distribution<> mag_noise{0, 100e-9}; // 100 nT
    std::normal_distribution<> baro_noise{0, 1.0};   // 1m altitude
    std::normal_distribution<> radar_noise{0, 5.0};  // 5m AGL
    
    void init(double lat_deg, double lon_deg, double alt_m) {
        true_state.fromGeodetic(lat_deg * M_PI/180, lon_deg * M_PI/180, alt_m);
        true_state.v_ECEF = Eigen::Vector3d(100, 0, 0);  // 100 m/s eastward
        t = 0;
    }
    
    ImuSample generateIMU(double dt) {
        ImuSample imu;
        
        // Simple dynamics: constant velocity with small perturbations
        true_state.p_ECEF += true_state.v_ECEF * dt;
        
        // Add slow heading drift (key problem without magnetometer!)
        double heading_drift_rate = 0.01;  // rad/s
        Eigen::AngleAxisd drift(heading_drift_rate * dt, Eigen::Vector3d::UnitZ());
        true_state.q_ECEF_B = true_state.q_ECEF_B * drift;
        
        // Generate IMU measurements
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
        // Earth's magnetic field in ECEF (simplified)
        Eigen::Vector3d mag_ECEF(20e-6, 0, -45e-6);  // ~50 μT
        
        // Transform to body frame
        Eigen::Vector3d mag_body = true_state.q_ECEF_B.inverse() * mag_ECEF;
        
        // Add noise
        mag_body.x() += mag_noise(rng);
        mag_body.y() += mag_noise(rng);
        mag_body.z() += mag_noise(rng);
        
        return mag_body;
    }
    
    double generateBarometer() {
        Eigen::Vector3d lla = true_state.toGeodetic();
        return lla.z() + baro_noise(rng);  // MSL altitude
    }
    
    double generateRadarAltimeter() {
        Eigen::Vector3d lla = true_state.toGeodetic();
        double terrain_height = 500.0;  // Assume 500m terrain
        return lla.z() - terrain_height + radar_noise(rng);
    }
};

int main() {
    std::cout << "=========================================\n";
    std::cout << "GPS-DENIED NAVIGATION VALIDATION TEST\n";
    std::cout << "=========================================\n\n";
    
    // Initialize gravity model
    GravityGradientProvider gravity_provider;
    
    // Test configurations
    struct TestCase {
        std::string name;
        bool use_magnetometer;
        bool use_barometer;
        bool use_terrain;
        bool use_zupt;
    };
    
    std::vector<TestCase> test_cases = {
        {"Baseline (Gravity only)", false, false, false, false},
        {"+ Magnetometer", true, false, false, false},
        {"+ Magnetometer + Barometer", true, true, false, false},
        {"+ All GPS-denied sensors", true, true, true, false},
        {"+ All sensors + ZUPT", true, true, true, true}
    };
    
    // Results file
    std::ofstream results("gps_denied_results.csv");
    results << "Test,Time,TrueX,TrueY,TrueZ,EstX,EstY,EstZ,Error\n";
    
    for (const auto& test : test_cases) {
        std::cout << "\nRunning: " << test.name << "\n";
        std::cout << "----------------------------------------\n";
        
        // Initialize simulator
        FlightSimulator sim;
        sim.init(47.0, 8.0, 1000.0);  // Start at 47°N, 8°E, 1000m
        
        // Initialize UKF
        UKF::Config ukf_config;
        UKF ukf(ukf_config);
        
        // Initial state with error
        State x0 = sim.true_state;
        x0.p_ECEF += Eigen::Vector3d(10, 10, 5);  // 10m horizontal, 5m vertical error
        
        // Initial covariance
        Eigen::Matrix<double, 15, 15> P0 = Eigen::Matrix<double, 15, 15>::Identity();
        P0.block<3,3>(0,0) *= 100.0;   // Position
        P0.block<3,3>(3,3) *= 1.0;     // Velocity
        P0.block<3,3>(6,6) *= 0.01;    // Attitude
        P0.block<3,3>(9,9) *= 0.01;    // Acc bias
        P0.block<3,3>(12,12) *= 0.001; // Gyro bias
        
        ukf.init(x0, P0);
        
        // Reference magnetic field
        Eigen::Vector3d mag_ref_ECEF(20e-6, 0, -45e-6);
        
        // Simulation parameters
        double dt = 0.01;  // 100 Hz
        double sim_time = 60.0;  // 1 minute flight
        int steps = sim_time / dt;
        
        // Statistics
        double max_error = 0;
        double sum_error = 0;
        int count = 0;
        
        for (int i = 0; i < steps; i++) {
            // Generate IMU
            ImuSample imu = sim.generateIMU(dt);
            
            // Predict
            ukf.predict(imu, dt);
            
            // Update with gravity gradient (always on)
            State current = ukf.getState();
            GravityGradientTensor tensor = gravity_provider.getGradient(current.p_ECEF);
            Eigen::Matrix3d R_grad = Eigen::Matrix3d::Identity() * 1e-18;  // 1 E² noise
            ukf.updateGradient(tensor.T, R_grad);
            
            // Magnetometer update
            if (test.use_magnetometer && i % 10 == 0) {  // 10 Hz
                Eigen::Vector3d mag = sim.generateMagnetometer();
                Eigen::Matrix3d R_mag = Eigen::Matrix3d::Identity() * 100e-18;
                ukf.updateMagnetometer(mag, mag_ref_ECEF, R_mag);
            }
            
            // Barometer update
            if (test.use_barometer && i % 100 == 0) {  // 1 Hz
                double baro_alt = sim.generateBarometer();
                ukf.updateBarometer(baro_alt, 1.0);
            }
            
            // Terrain correlation
            if (test.use_terrain && i % 100 == 0) {  // 1 Hz
                double radar_alt = sim.generateRadarAltimeter();
                double terrain_height = 500.0;  // Known terrain
                ukf.updateTerrainAltitude(radar_alt, terrain_height, 25.0);
            }
            
            // ZUPT (when velocity is low)
            if (test.use_zupt && sim.true_state.v_ECEF.norm() < 1.0) {
                Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity() * 0.01;
                ukf.updateZUPT(R_vel);
            }
            
            // Calculate error
            State est = ukf.getState();
            double error = (est.p_ECEF - sim.true_state.p_ECEF).norm();
            max_error = std::max(max_error, error);
            sum_error += error;
            count++;
            
            // Log every second
            if (i % 100 == 0) {
                results << test.name << "," << sim.t << ",";
                results << sim.true_state.p_ECEF.x() << ",";
                results << sim.true_state.p_ECEF.y() << ",";
                results << sim.true_state.p_ECEF.z() << ",";
                results << est.p_ECEF.x() << ",";
                results << est.p_ECEF.y() << ",";
                results << est.p_ECEF.z() << ",";
                results << error << "\n";
                
                if (i % 1000 == 0) {
                    std::cout << "  t=" << sim.t << "s, error=" << error << "m\n";
                }
            }
        }
        
        // Report results
        double avg_error = sum_error / count;
        std::cout << "\nResults for " << test.name << ":\n";
        std::cout << "  Average error: " << avg_error << " m\n";
        std::cout << "  Maximum error: " << max_error << " m\n";
        
        // Grade the performance
        std::string grade;
        if (avg_error < 10) grade = "A+ (Excellent)";
        else if (avg_error < 50) grade = "A (Very Good)";
        else if (avg_error < 100) grade = "B+ (Good)";
        else if (avg_error < 500) grade = "B (Acceptable)";
        else if (avg_error < 1000) grade = "C (Poor)";
        else grade = "F (Failed)";
        
        std::cout << "  Grade: " << grade << "\n";
    }
    
    results.close();
    
    std::cout << "\n=========================================\n";
    std::cout << "GPS-DENIED NAVIGATION TEST COMPLETE\n";
    std::cout << "Results saved to: gps_denied_results.csv\n";
    std::cout << "=========================================\n";
    
    return 0;
}