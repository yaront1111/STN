/**
 * Test gravity navigation with real flight data
 * Analyzes performance and identifies improvements needed
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include "../cpp/core/types.h"
#include "../cpp/core/ukf.h"
#include "../cpp/core/gravity_gradient_provider.h"
#include "../cpp/core/csv_data_reader.h"

double computePositionError(const State& estimated, const Eigen::Vector3d& true_lla) {
    // Convert estimated position to lat/lon
    Eigen::Vector3d est_lla = estimated.toGeodetic();
    
    // Compute horizontal error in meters
    const double R_earth = 6371000.0;
    double dlat = est_lla.x() - true_lla.x() * M_PI / 180.0;
    double dlon = est_lla.y() - true_lla.y() * M_PI / 180.0;
    double dalt = est_lla.z() - true_lla.z();
    
    double north_error = dlat * R_earth;
    double east_error = dlon * R_earth * std::cos(true_lla.x() * M_PI / 180.0);
    double horizontal_error = std::sqrt(north_error * north_error + east_error * east_error);
    
    return horizontal_error;
}

int main(int argc, char** argv) {
    std::cout << "\n=================================================\n";
    std::cout << "    GRAVITY NAVIGATION - REAL DATA TEST\n";
    std::cout << "=================================================\n\n";
    
    // Load CSV data
    std::string csv_file = (argc > 1) ? argv[1] : "flight_data.csv";
    std::cout << "Loading flight data from: " << csv_file << "\n";
    
    CSVDataReader data_reader;
    if (!data_reader.loadCSV(csv_file)) {
        std::cerr << "ERROR: Failed to load CSV file\n";
        return 1;
    }
    
    // Initialize gravity gradient provider
    std::cout << "Initializing gravity gradient provider...\n";
    GravityGradientProvider gravity_provider;
    gravity_provider.loadEGM2020("data/egm2020/coefficients.dat");
    
    // Initialize stable UKF with error-state formulation
    std::cout << "Initializing Stable UKF...\n";
    UKF::Config ukf_config;
    ukf_config.alpha = 0.01;      // Spread of sigma points
    ukf_config.beta = 2.0;        // Gaussian prior
    ukf_config.kappa = 3 - 15;    // Standard for 15D error state
    
    // Tune process noise for aircraft dynamics
    ukf_config.sigma_pos = 1.0;   // m (higher for aircraft)
    ukf_config.sigma_vel = 5.0;   // m/s (much higher for maneuvers)
    ukf_config.sigma_att = 0.05;  // rad (higher for banking)
    ukf_config.sigma_ba = 1e-3;   // m/s² bias drift
    ukf_config.sigma_bg = 1e-4;   // rad/s bias drift
    
    UKF ukf(ukf_config);
    
    // Initialize with true starting position
    Eigen::Vector3d true_start = data_reader.getTruePosition(0);
    State x0;
    x0.fromGeodetic(true_start.x() * M_PI / 180.0, 
                    true_start.y() * M_PI / 180.0,
                    true_start.z());
    
    // Initial covariance (15D error state)
    Eigen::Matrix<double, UKF::ERROR_STATE_DIM, UKF::ERROR_STATE_DIM> P0 = 
        Eigen::Matrix<double, UKF::ERROR_STATE_DIM, UKF::ERROR_STATE_DIM>::Identity();
    P0.block<3,3>(0,0) *= 100.0;    // Position: 10m uncertainty
    P0.block<3,3>(3,3) *= 4.0;      // Velocity: 2 m/s
    P0.block<3,3>(6,6) *= 0.01;     // Attitude: 0.1 rad
    P0.block<3,3>(9,9) *= 1e-4;     // Acc bias
    P0.block<3,3>(12,12) *= 1e-6;   // Gyro bias
    
    ukf.init(x0, P0);
    std::cout << "Initial position: " << true_start.x() << "°N, " 
              << true_start.y() << "°E, " << true_start.z() << "m\n\n";
    
    // Output file for results
    std::ofstream output("gravity_nav_results.csv");
    output << "t,lat_est,lon_est,alt_est,lat_true,lon_true,alt_true,";
    output << "error_m,vn,ve,vd,gradient_updates,anomaly_updates\n";
    
    // Navigation loop
    std::cout << "Starting navigation with real data...\n";
    std::cout << "=========================================\n";
    
    int gradient_updates = 0;
    int anomaly_updates = 0;
    double max_error = 0.0;
    double total_error = 0.0;
    int error_count = 0;
    
    // Process data at 100 Hz
    double dt = 0.01;
    int print_interval = 1000;  // Print every 10 seconds
    int skipped_samples = 0;
    
    for (size_t i = 0; i < data_reader.size(); ++i) {
        // Get IMU sample
        ImuSample imu = data_reader.getIMUSample(i);
        
        // Skip unrealistic measurements
        if (imu.acc_mps2.norm() > 50.0 || imu.gyro_rps.norm() > 3.0) {
            skipped_samples++;
            continue;
        }
        
        // Propagate UKF with IMU
        ukf.predict(imu, dt);
        
        // Update with gravity gradient (at 1 Hz)
        if (i % 100 == 0 && i > 0) {
            GravityGradientTensor gradient = data_reader.getGradient(i);
            if (gradient.isValid()) {
                ukf.updateGradient(gradient.T, gradient.R);
                gradient_updates++;
            }
        }
        
        // Update with gravity anomaly (at 10 Hz)
        if (i % 10 == 0 && i > 100) {
            double anomaly = data_reader.getAnomaly(i);
            ukf.updateAnomaly(anomaly, 0.5*0.5);  // 0.5 mGal noise (variance)
            anomaly_updates++;
        }
        
        // Get current state
        State x = ukf.getState();
        Eigen::Vector3d true_pos = data_reader.getTruePosition(i);
        
        // Compute error
        double error = computePositionError(x, true_pos);
        total_error += error;
        error_count++;
        if (error > max_error) {
            max_error = error;
        }
        
        // Convert to geodetic for output
        Eigen::Vector3d est_lla = x.toGeodetic();
        
        // Write to output file
        output << imu.t << ",";
        output << est_lla.x() * 180.0 / M_PI << ",";
        output << est_lla.y() * 180.0 / M_PI << ",";
        output << est_lla.z() << ",";
        output << true_pos.x() << ",";
        output << true_pos.y() << ",";
        output << true_pos.z() << ",";
        output << error << ",";
        output << x.v_ECEF.x() << ",";
        output << x.v_ECEF.y() << ",";
        output << x.v_ECEF.z() << ",";
        output << gradient_updates << ",";
        output << anomaly_updates << "\n";
        
        // Print status
        if (i % print_interval == 0 && i > 0) {
            double t = imu.t;
            std::cout << "t=" << std::fixed << std::setprecision(0) << t << "s";
            std::cout << " | Error: " << std::fixed << std::setprecision(1) << error << "m";
            std::cout << " | True: " << true_pos.x() << "°N, " << true_pos.y() << "°E, " 
                     << true_pos.z() << "m";
            std::cout << " | Updates: G=" << gradient_updates << " A=" << anomaly_updates;
            
            // Compute velocity magnitude
            double vel_mag = x.v_ECEF.norm();
            std::cout << " | Vel: " << std::fixed << std::setprecision(1) << vel_mag << " m/s";
            std::cout << "\n";
        }
    }
    
    output.close();
    
    // Final statistics
    std::cout << "\n=========================================\n";
    std::cout << "Navigation Complete\n";
    std::cout << "=========================================\n";
    
    double avg_error = total_error / error_count;
    
    std::cout << "Total samples processed: " << data_reader.size() << "\n";
    std::cout << "Skipped (unrealistic) samples: " << skipped_samples << "\n";
    std::cout << "Gradient updates: " << gradient_updates << "\n";
    std::cout << "Anomaly updates: " << anomaly_updates << "\n";
    std::cout << "\nError Statistics:\n";
    std::cout << "  Average error: " << std::fixed << std::setprecision(1) << avg_error << " m\n";
    std::cout << "  Maximum error: " << std::fixed << std::setprecision(1) << max_error << " m\n";
    
    // Analyze error growth
    State final_state = ukf.getState();
    Eigen::Vector3d final_true = data_reader.getTruePosition(data_reader.size() - 1);
    double final_error = computePositionError(final_state, final_true);
    std::cout << "  Final error: " << std::fixed << std::setprecision(1) << final_error << " m\n";
    
    // Get covariance
    auto P = ukf.getCovariance();
    double pos_uncertainty = std::sqrt(P(0,0) + P(1,1));
    std::cout << "  Position uncertainty (1σ): " << std::fixed << std::setprecision(1) 
              << pos_uncertainty << " m\n";
    
    // Performance analysis
    std::cout << "\n=========================================\n";
    std::cout << "Performance Analysis\n";
    std::cout << "=========================================\n";
    
    if (avg_error > 1000.0) {
        std::cout << "❌ Large navigation errors detected\n";
        std::cout << "\nRecommended improvements:\n";
        std::cout << "1. Need real EGM2020 gravity model data\n";
        std::cout << "2. Tune UKF parameters for aircraft dynamics\n";
        std::cout << "3. Implement adaptive measurement noise\n";
        std::cout << "4. Add zero-velocity updates during taxi\n";
        std::cout << "5. Consider atmospheric corrections\n";
    } else if (avg_error > 100.0) {
        std::cout << "⚠️ Moderate navigation errors\n";
        std::cout << "\nRecommended improvements:\n";
        std::cout << "1. Improve gravity gradient matching\n";
        std::cout << "2. Add barometric altitude aiding\n";
        std::cout << "3. Implement map matching for anomalies\n";
    } else {
        std::cout << "✅ Good navigation performance\n";
        std::cout << "System performing within expected bounds\n";
    }
    
    std::cout << "\nResults saved to: gravity_nav_results.csv\n";
    std::cout << "Use Python to visualize and analyze results\n\n";
    
    return 0;
}