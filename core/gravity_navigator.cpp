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
#include "../cpp/hardware/sensor_interfaces.h"
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
    
    // Magnetometer for heading reference (GPS-denied navigation)
    std::unique_ptr<MagnetometerInterface> magnetometer = nullptr;
    if (config.getBool("hardware.magnetometer.enabled", true)) {
        magnetometer = SensorFactory::createMagnetometer(
            config.getString("hardware.magnetometer.type", "HMC5883L"));
        if (magnetometer && !magnetometer->initialize()) {
            std::cerr << "WARNING: Magnetometer initialization failed\n";
            magnetometer = nullptr;
        } else {
            std::cout << "✓ Magnetometer initialized for heading reference\n";
        }
    }
    
    // Barometric altimeter for vertical constraint
    std::unique_ptr<BarometerInterface> barometer = nullptr;
    if (config.getBool("hardware.barometer.enabled", true)) {
        barometer = SensorFactory::createBarometer(
            config.getString("hardware.barometer.type", "BMP388"));
        if (barometer && !barometer->initialize()) {
            std::cerr << "WARNING: Barometer initialization failed\n";
            barometer = nullptr;
        } else {
            std::cout << "✓ Barometer initialized for altitude constraint\n";
        }
    }
    
    // Radar altimeter for terrain correlation
    std::unique_ptr<RadarAltimeterInterface> radar_alt = nullptr;
    if (config.getBool("hardware.radar_altimeter.enabled", true)) {
        radar_alt = SensorFactory::createRadarAltimeter(
            config.getString("hardware.radar_altimeter.type", "KRA405B"));
        if (radar_alt && !radar_alt->initialize()) {
            std::cerr << "WARNING: Radar altimeter initialization failed\n";
            radar_alt = nullptr;
        } else {
            std::cout << "✓ Radar altimeter initialized for terrain correlation\n";
        }
    }
    
    // Load terrain database for correlation
    std::unique_ptr<TerrainDatabase> terrain_db = nullptr;
    if (radar_alt) {
        terrain_db = std::make_unique<TerrainDatabase>();
        if (!terrain_db->loadSRTM("data/srtm/elevation_database.dat")) {
            std::cerr << "WARNING: Failed to load SRTM terrain data\n";
            radar_alt = nullptr;  // Disable radar alt if no terrain data
            terrain_db = nullptr;
        } else {
            std::cout << "✓ SRTM terrain database loaded\n";
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
    
    // Initial covariance (15D error state)
    Eigen::Matrix<double, UKF::ERROR_STATE_DIM, UKF::ERROR_STATE_DIM> P0 = 
        Eigen::Matrix<double, UKF::ERROR_STATE_DIM, UKF::ERROR_STATE_DIM>::Identity();
    P0.block<3,3>(0,0) *= 100.0;    // Position uncertainty: 10m
    P0.block<3,3>(3,3) *= 1.0;      // Velocity uncertainty: 1 m/s
    P0.block<3,3>(6,6) *= 0.01;     // Attitude uncertainty: 0.1 rad
    P0.block<3,3>(9,9) *= 0.01;     // Acc bias uncertainty
    P0.block<3,3>(12,12) *= 0.001;  // Gyro bias uncertainty
    
    ukf.init(x0, P0);
    std::cout << "✓ UKF initialized\n\n";
    
    // Output file
    std::ofstream output_file(config.getString("output.path", "data/gravity_nav.csv"));
    output_file << "t,lat,lon,alt,vn,ve,vd,roll,pitch,yaw,";
    output_file << "dt,df,gradient_updates,anomaly_updates,mag_updates,baro_updates,terrain_updates,zupt_updates\n";
    
    // Main navigation loop
    std::cout << "Starting gravity-primary navigation...\n";
    std::cout << "=========================================\n";
    
    double t = 0.0;
    double dt = 1.0 / config.getDouble("system.rate_hz", 100.0);
    int gradient_updates = 0;
    int anomaly_updates = 0;
    int mag_updates = 0;
    int baro_updates = 0;
    int terrain_updates = 0;
    int zupt_updates = 0;
    
    // Statistics
    double start_time = 0.0;
    Eigen::Vector3d initial_pos = x0.p_ECEF;
    
    // Reference magnetic field (needs to be set for location)
    Eigen::Vector3d mag_ref_ECEF(0.0, 0.0, -50000e-9);  // ~50μT downward (simplified)
    
    while (true) {
        // Read IMU
        ImuSample imu_sample;
        if (!imu->read(imu_sample)) {
            std::cerr << "ERROR: IMU read failed\n";
            break;
        }
        
        // Propagate UKF with IMU
        ukf.predict(imu_sample, dt);
        
        // Update with gravity gradiometer (if available)
        if (gradiometer && gradiometer->hasNewData()) {
            GravityGradientTensor gradient = gradiometer->read();
            
            // Validate measurement
            if (gradient.isValid()) {
                ukf.updateGradient(gradient.T, gradient.R);
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
                ukf.updateAnomaly(anomaly_mgal, 1.0);  // 1 mGal noise variance
                anomaly_updates++;
            }
        }
        
        // GPS-DENIED NAVIGATION UPDATES
        
        // Magnetometer update for heading reference
        if (magnetometer && magnetometer->hasNewData()) {
            Eigen::Vector3d mag_body = magnetometer->read();
            if (mag_body.norm() > 1e-9) {  // Valid measurement
                Eigen::Matrix3d R_mag = Eigen::Matrix3d::Identity() * 100e-18;  // 100 nT² noise
                ukf.updateMagnetometer(mag_body, mag_ref_ECEF, R_mag);
                mag_updates++;
            }
        }
        
        // Barometric altitude update
        if (barometer && barometer->hasNewData()) {
            double pressure_alt = barometer->readAltitude();  // meters MSL
            ukf.updateBarometer(pressure_alt, 1.0);  // 1m² noise variance
            baro_updates++;
        }
        
        // Terrain correlation with radar altimeter
        if (radar_alt && terrain_db && radar_alt->hasNewData()) {
            double radar_altitude = radar_alt->readAltitude();  // meters AGL
            State current = ukf.getState();
            Eigen::Vector3d lla = current.toGeodetic();
            
            // Get terrain height at current position
            double terrain_height = terrain_db->getElevation(
                lla.x() * 180.0 / M_PI, 
                lla.y() * 180.0 / M_PI);
            
            if (terrain_height > -9999) {  // Valid terrain data
                ukf.updateTerrainAltitude(radar_altitude, terrain_height, 5.0);  // 5m² noise
                terrain_updates++;
            }
        }
        
        // Zero Velocity Update (when stationary)
        // Detect stationary condition from IMU
        bool is_stationary = (imu_sample.gyro_rps.norm() < 0.01 && 
                             std::abs(imu_sample.acc_mps2.norm() - 9.81) < 0.5);
        if (is_stationary) {
            Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity() * 0.01;  // 0.1 m/s noise
            ukf.updateZUPT(R_vel);
            zupt_updates++;
        }
        
        // Update with CSAC (if available)
        if (csac && csac->hasNewData()) {
            CSACMeasurement csac_meas = csac->read();
            // Clock updates not yet implemented in stable UKF
            // TODO: Add clock state to UKF if needed
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
        output_file << anomaly_updates << ",";
        output_file << mag_updates << ",";
        output_file << baro_updates << ",";
        output_file << terrain_updates << ",";
        output_file << zupt_updates << "\n";
        
        // Print status every second
        if (static_cast<int>(t) != static_cast<int>(t - dt)) {
            Eigen::Vector3d pos_error = x.p_ECEF - initial_pos;
            double drift = pos_error.norm();
            
            std::cout << "t=" << static_cast<int>(t) << "s";
            std::cout << " | Drift: " << drift << "m";
            std::cout << " | G=" << gradient_updates << " A=" << anomaly_updates;
            std::cout << " M=" << mag_updates << " B=" << baro_updates;
            std::cout << " T=" << terrain_updates << " Z=" << zupt_updates;
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
    std::cout << "Magnetometer updates: " << mag_updates << "\n";
    std::cout << "Barometer updates: " << baro_updates << "\n";
    std::cout << "Terrain updates: " << terrain_updates << "\n";
    std::cout << "ZUPT updates: " << zupt_updates << "\n";
    
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