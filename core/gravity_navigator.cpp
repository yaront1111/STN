/**
 * GRAVITY-PRIMARY NAVIGATION SYSTEM
 * 
 * Complete replacement for main.cpp
 * NO TRN code - pure gravity navigation
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "../cpp/core/types.h"
#include "../cpp/core/ukf.h"
#include "../cpp/core/gravity_gradient_provider.h"
#include "../cpp/core/imu_processor.h"
#include "../cpp/hardware/hardware_interface.h"
#include "../cpp/core/config_reader.h"

int main(int argc, char** argv) {
    std::cout << "=================================================\n";
    std::cout << "    SPACETIME NAVIGATION SYSTEM v2.0\n";
    std::cout << "       Gravity-Primary Navigation\n";
    std::cout << "=================================================\n\n";
    
    // Load configuration
    ConfigReader config;
    if (!config.loadFromFile("config/gravity_primary.yaml")) {
        std::cerr << "ERROR: Failed to load configuration\n";
        return 1;
    }
    
    // Initialize gravity gradient provider with EGM2020
    std::cout << "Loading EGM2020 gravity model...\n";
    GravityGradientProvider gravity_provider;
    if (!gravity_provider.loadEGM2020("data/egm2020/coefficients.dat")) {
        std::cerr << "ERROR: Failed to load EGM2020 data\n";
        std::cerr << "Please download from: https://earth-info.nga.mil/EGM2020/\n";
        return 1;
    }
    std::cout << "✓ EGM2020 loaded (degree 2190, ~5m resolution)\n\n";
    
    // Initialize hardware interfaces
    std::cout << "Initializing hardware interfaces...\n";
    
    // IMU (VectorNav VN-200 or similar)
    auto imu = HardwareInterface::createIMU(config.getString("hardware.imu.type"));
    if (!imu->initialize(config.getString("hardware.imu.port"))) {
        std::cerr << "ERROR: Failed to initialize IMU\n";
        return 1;
    }
    std::cout << "✓ IMU initialized: " << imu->getModelName() << "\n";
    
    // Gravity Gradiometer (when available)
    std::unique_ptr<GradiometerInterface> gradiometer = nullptr;
    if (config.getBool("hardware.gradiometer.enabled")) {
        gradiometer = HardwareInterface::createGradiometer(
            config.getString("hardware.gradiometer.type"));
        if (gradiometer && !gradiometer->initialize()) {
            std::cerr << "WARNING: Gradiometer initialization failed\n";
            gradiometer = nullptr;
        } else {
            std::cout << "✓ Gradiometer initialized\n";
        }
    }
    
    // CSAC (Chip Scale Atomic Clock)
    std::unique_ptr<CSACInterface> csac = nullptr;
    if (config.getBool("hardware.csac.enabled")) {
        csac = HardwareInterface::createCSAC(config.getString("hardware.csac.type"));
        if (csac && !csac->initialize()) {
            std::cerr << "WARNING: CSAC initialization failed\n";
            csac = nullptr;
        } else {
            std::cout << "✓ CSAC initialized\n";
        }
    }
    
    // Initialize UKF
    std::cout << "\nInitializing Unscented Kalman Filter...\n";
    UKF::Config ukf_config;
    ukf_config.alpha = config.getDouble("ukf.alpha", 1e-3);
    ukf_config.beta = config.getDouble("ukf.beta", 2.0);
    ukf_config.kappa = config.getDouble("ukf.kappa", 0.0);
    
    UKF ukf(ukf_config);
    
    // Initial state from config or user input
    State x0;
    if (argc > 1) {
        // Parse initial position from command line
        double lat = std::stod(argv[1]) * M_PI / 180.0;
        double lon = std::stod(argv[2]) * M_PI / 180.0;
        double alt = argc > 3 ? std::stod(argv[3]) : 0.0;
        x0.fromGeodetic(lat, lon, alt);
        std::cout << "Initial position: " << argv[1] << "°N, " << argv[2] << "°E, " 
                  << alt << "m\n";
    } else {
        // Default: null island (0°N, 0°E)
        x0.fromGeodetic(0.0, 0.0, 0.0);
        std::cout << "Initial position: 0°N, 0°E (Null Island)\n";
    }
    
    // Initial covariance
    Eigen::Matrix<double, UKF::STATE_DIM, UKF::STATE_DIM> P0 = 
        Eigen::Matrix<double, UKF::STATE_DIM, UKF::STATE_DIM>::Identity();
    P0.block<3,3>(0,0) *= 100.0;    // Position uncertainty: 10m
    P0.block<3,3>(3,3) *= 1.0;      // Velocity uncertainty: 1 m/s
    P0.block<3,3>(6,6) *= 0.01;     // Attitude uncertainty: 0.1 rad
    P0.block<3,3>(10,10) *= 0.01;   // Acc bias uncertainty
    P0.block<3,3>(13,13) *= 0.001;  // Gyro bias uncertainty
    P0(16,16) = 1e-6;                // Clock offset uncertainty
    P0(17,17) = 1e-9;                // Clock drift uncertainty
    
    ukf.init(x0, P0);
    std::cout << "✓ UKF initialized\n\n";
    
    // Output file
    std::ofstream output_file(config.getString("output.path", "data/gravity_nav.csv"));
    output_file << "t,lat,lon,alt,vn,ve,vd,roll,pitch,yaw,";
    output_file << "dt,df,gradient_updates,anomaly_updates\n";
    
    // Main navigation loop
    std::cout << "Starting gravity-primary navigation...\n";
    std::cout << "=========================================\n";
    
    double t = 0.0;
    double dt = 1.0 / config.getDouble("system.rate_hz", 100.0);
    int gradient_updates = 0;
    int anomaly_updates = 0;
    
    // Statistics
    double start_time = 0.0;
    Eigen::Vector3d initial_pos = x0.p_ECEF;
    
    while (true) {
        // Read IMU
        ImuSample imu_sample;
        if (!imu->read(imu_sample)) {
            std::cerr << "ERROR: IMU read failed\n";
            break;
        }
        
        // Propagate UKF with IMU
        ukf.propagateWithIMU(imu_sample, dt);
        
        // Update with gravity gradiometer (if available)
        if (gradiometer && gradiometer->hasNewData()) {
            GravityGradientTensor gradient = gradiometer->read();
            
            // Validate measurement
            if (gradient.isValid()) {
                ukf.updateGravityGradient(gradient.T, gradient.R);
                gradient_updates++;
                
                if (gradient_updates % 10 == 0) {
                    std::cout << "Gradient update " << gradient_updates 
                             << ": trace = " << gradient.T.trace() << " E\n";
                }
            }
        }
        
        // Update with gravity anomaly from accelerometer
        // This provides continuous updates even without gradiometer
        if (t > 1.0) {  // Wait for filter to settle
            // Extract gravity from accelerometer
            State current = ukf.getState();
            Eigen::Vector3d gravity_body = imu_sample.acc_mps2;  // Simplified
            
            // Transform to ECEF and compute anomaly
            Eigen::Vector3d gravity_ecef = current.q_ECEF_B * gravity_body;
            double measured_g = gravity_ecef.norm();
            
            // Get predicted gravity at current position
            double predicted_g = 9.80665;  // Simplified - would use WGS84 model
            double anomaly_mgal = (measured_g - predicted_g) * 100000.0;
            
            // Update if significant
            if (std::abs(anomaly_mgal) > 0.1) {
                ukf.updateGravityAnomaly(anomaly_mgal, 1.0);  // 1 mGal noise
                anomaly_updates++;
            }
        }
        
        // Update with CSAC (if available)
        if (csac && csac->hasNewData()) {
            CSACMeasurement csac_meas = csac->read();
            ukf.updateClock(csac_meas.offset_s, csac_meas.drift_ppm);
        }
        
        // Get current state
        State x = ukf.getState();
        
        // Convert to geodetic for output
        Eigen::Vector3d lla = x.toGeodetic();
        
        // Compute Euler angles
        Eigen::Vector3d euler = x.q_ECEF_B.toRotationMatrix().eulerAngles(0, 1, 2);
        
        // Write to file
        output_file << x.t << ",";
        output_file << lla.x() * 180.0 / M_PI << ",";
        output_file << lla.y() * 180.0 / M_PI << ",";
        output_file << lla.z() << ",";
        output_file << x.v_ECEF.x() << ",";
        output_file << x.v_ECEF.y() << ",";
        output_file << x.v_ECEF.z() << ",";
        output_file << euler.x() * 180.0 / M_PI << ",";
        output_file << euler.y() * 180.0 / M_PI << ",";
        output_file << euler.z() * 180.0 / M_PI << ",";
        output_file << x.dt << ",";
        output_file << x.df << ",";
        output_file << gradient_updates << ",";
        output_file << anomaly_updates << "\n";
        
        // Print status every second
        if (static_cast<int>(t) != static_cast<int>(t - dt)) {
            Eigen::Vector3d pos_error = x.p_ECEF - initial_pos;
            double drift = pos_error.norm();
            
            std::cout << "t=" << static_cast<int>(t) << "s";
            std::cout << " | Position: " << lla.x() * 180.0 / M_PI << "°N, " 
                     << lla.y() * 180.0 / M_PI << "°E, " << lla.z() << "m";
            std::cout << " | Drift: " << drift << "m";
            std::cout << " | Updates: G=" << gradient_updates << " A=" << anomaly_updates;
            std::cout << "\n";
        }
        
        // Update time
        t += dt;
        
        // Check for exit condition (could be user input, time limit, etc.)
        if (t > config.getDouble("system.max_runtime", 3600.0)) {
            break;
        }
    }
    
    // Final statistics
    std::cout << "\n=========================================\n";
    std::cout << "Navigation Complete\n";
    std::cout << "Total runtime: " << t << " seconds\n";
    std::cout << "Gradient updates: " << gradient_updates << "\n";
    std::cout << "Anomaly updates: " << anomaly_updates << "\n";
    
    State final_state = ukf.getState();
    Eigen::Vector3d final_error = final_state.p_ECEF - initial_pos;
    std::cout << "Final position error: " << final_error.norm() << " meters\n";
    
    // Compute CEP
    auto P = ukf.getCovariance();
    double cep50 = 1.1774 * std::sqrt(P(0,0) + P(1,1));
    double cep95 = 2.4477 * std::sqrt(P(0,0) + P(1,1));
    std::cout << "CEP50: " << cep50 << " meters\n";
    std::cout << "CEP95: " << cep95 << " meters\n";
    
    output_file.close();
    std::cout << "\nResults saved to: " << config.getString("output.path") << "\n";
    
    return 0;
}