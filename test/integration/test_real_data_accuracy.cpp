/**
 * Real Data Accuracy Test
 * Tests navigation system with actual flight data and maps
 * This is the TRUE test of whether the system meets 50m accuracy
 */

#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include "core/hierarchical_filter.h"
#include "sensors/sensor_manager.h"
#include "maps/map_manager.h"
#include "maps/composite_map_manager.h"
#include "utils/logger.h"

using namespace Navigation;
using namespace Eigen;

class RealDataAccuracyTest : public ::testing::Test {
protected:
    std::unique_ptr<HierarchicalFilter> navigation_system;
    std::unique_ptr<SensorManager> sensor_manager;
    std::shared_ptr<MapManager> map_manager;

    // Ground truth data
    struct TruthPoint {
        double timestamp;
        Vector3d position;
        Vector3d velocity;
        Quaterniond quaternion;
    };
    std::vector<TruthPoint> truth_data;

    void SetUp() override {
        Logger::getInstance().initialize(".", "ERROR");

        // Use config file to initialize everything like production
        YAML::Node config = YAML::LoadFile("../../config.yaml");

        // Create navigation system from config - includes maps!
        auto ukf_config = config["ukf"];
        auto rbpf_config = config["rbpf"];

        // Create MapManager (use composite implementation)
        auto map_config = config["maps"];
        map_manager = std::make_shared<CompositeMapManager>(
            map_config["gravity"]["data_path"].as<std::string>(),
            map_config["terrain"]["data_path"].as<std::string>()
        );
        if (!map_manager->initialize()) {
            LOG_WARN("Failed to initialize maps");
        }

        // Initialize sensor manager from config
        sensor_manager = std::make_unique<SensorManager>(config["sensors"]);

        // Initialize navigation with actual maps
        navigation_system = std::make_unique<HierarchicalFilter>(
            ukf_config,
            rbpf_config,
            map_manager
        );

        // Load ground truth
        loadGroundTruth("../../data/flight/truth.csv");
    }

    void loadGroundTruth(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open truth file: " + filename);
        }

        std::string line;
        std::getline(file, line); // Skip header

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            TruthPoint point;

            ss >> point.timestamp;
            char comma;
            ss >> comma >> point.position.x();
            ss >> comma >> point.position.y();
            ss >> comma >> point.position.z();
            ss >> comma >> point.velocity.x();
            ss >> comma >> point.velocity.y();
            ss >> comma >> point.velocity.z();

            double w, x, y, z;
            ss >> comma >> w >> comma >> x >> comma >> y >> comma >> z;
            point.quaternion = Quaterniond(w, x, y, z);

            truth_data.push_back(point);
        }
    }

    TruthPoint getTruthAtTime(double timestamp) {
        // Find closest truth point
        auto it = std::lower_bound(truth_data.begin(), truth_data.end(), timestamp,
            [](const TruthPoint& a, double t) { return a.timestamp < t; });

        if (it == truth_data.end()) return truth_data.back();
        if (it == truth_data.begin()) return truth_data.front();

        // Interpolate between points
        auto prev = std::prev(it);
        double t1 = prev->timestamp;
        double t2 = it->timestamp;
        double alpha = (timestamp - t1) / (t2 - t1);

        TruthPoint result;
        result.timestamp = timestamp;
        result.position = prev->position + alpha * (it->position - prev->position);
        result.velocity = prev->velocity + alpha * (it->velocity - prev->velocity);
        result.quaternion = prev->quaternion.slerp(alpha, it->quaternion);

        return result;
    }
};

// Test: Real data 50m accuracy requirement
TEST_F(RealDataAccuracyTest, FiftyMeterAccuracyWithRealData) {
    // Initialize from first truth point
    if (truth_data.empty()) {
        FAIL() << "No ground truth data loaded";
    }

    StateVector initial;
    initial.position = truth_data[0].position;
    initial.velocity = truth_data[0].velocity;
    initial.quaternion = truth_data[0].quaternion;
    initial.accel_bias = Vector3d(0.01, -0.01, 0.02);  // Initial guess
    initial.gyro_bias = Vector3d(0.001, -0.001, 0.0005);

    navigation_system->initialize(initial);

    // Process real sensor data
    double max_position_error = 0.0;
    double max_horizontal_error = 0.0;
    double max_vertical_error = 0.0;
    int measurement_count = 0;

    // Track errors over time
    std::vector<double> position_errors;
    std::vector<double> timestamps;

    // Process data like production system
    double current_time = 0.0;
    double dt = 0.01; // 100 Hz
    int sample_count = 0;

    while (current_time < 600.0 && sample_count < 60000) { // 10 minutes
        // Get sensor readings at current time
        IMUData imu = sensor_manager->getIMUAtTime(current_time);
        navigation_system->processIMU(imu, dt);

        // Process other sensors at their rates
        if (sample_count % 10 == 0) { // 10 Hz
            BarometerData baro = sensor_manager->getBarometerAtTime(current_time);
            navigation_system->updateBarometer(baro);

            MagnetometerData mag = sensor_manager->getMagnetometerAtTime(current_time);
            navigation_system->updateMagnetometer(mag);
        }

        // CRITICAL: Process gradiometer for map matching!
        if (sample_count % 100 == 0) { // 1 Hz
            GradiometerData grad = sensor_manager->getGradiometerAtTime(current_time);
            navigation_system->updateGravity(grad);
        }

        // Check accuracy every second (100 measurements)
        if (++measurement_count % 100 == 0) {
            StateVector nav_state = navigation_system->getState();
            TruthPoint truth = getTruthAtTime(sensor_data.timestamp);

            Vector3d position_error = nav_state.position - truth.position;
            double total_error = position_error.norm();
            double horizontal_error = position_error.head<2>().norm();
            double vertical_error = std::abs(position_error.z());

            max_position_error = std::max(max_position_error, total_error);
            max_horizontal_error = std::max(max_horizontal_error, horizontal_error);
            max_vertical_error = std::max(max_vertical_error, vertical_error);

            position_errors.push_back(total_error);
            timestamps.push_back(sensor_data.timestamp);

            // Log progress
            if (measurement_count % 1000 == 0) {
                std::cout << "Time: " << sensor_data.timestamp
                         << "s, Position error: " << total_error << "m" << std::endl;
            }
        }

        // Test first 10 minutes of data
        if (sensor_data.timestamp > 600.0) {
            break;
        }
    }

    // Calculate statistics
    double mean_error = 0.0;
    for (double error : position_errors) {
        mean_error += error;
    }
    mean_error /= position_errors.size();

    // Find 95th percentile error
    std::sort(position_errors.begin(), position_errors.end());
    size_t p95_index = position_errors.size() * 0.95;
    double p95_error = position_errors[p95_index];

    // Report results
    std::cout << "\n=== REAL DATA ACCURACY TEST RESULTS ===" << std::endl;
    std::cout << "Test duration: " << timestamps.back() << " seconds" << std::endl;
    std::cout << "Samples analyzed: " << position_errors.size() << std::endl;
    std::cout << "Mean position error: " << mean_error << " meters" << std::endl;
    std::cout << "Maximum position error: " << max_position_error << " meters" << std::endl;
    std::cout << "95th percentile error: " << p95_error << " meters" << std::endl;
    std::cout << "Maximum horizontal error: " << max_horizontal_error << " meters" << std::endl;
    std::cout << "Maximum vertical error: " << max_vertical_error << " meters" << std::endl;
    std::cout << "REQUIREMENT: < 50 meters" << std::endl;
    std::cout << "STATUS: " << (max_position_error < 50.0 ? "PASS ✓" : "FAIL ✗") << std::endl;

    // Check 50m requirement
    EXPECT_LT(max_position_error, 50.0)
        << "Maximum position error (" << max_position_error
        << "m) exceeds 50m requirement with real data and maps";

    // Additional checks
    EXPECT_LT(mean_error, 30.0)
        << "Mean error too high: " << mean_error << "m";

    EXPECT_LT(p95_error, 45.0)
        << "95th percentile error too high: " << p95_error << "m";
}

// Test: Verify RBPF is actually working
TEST_F(RealDataAccuracyTest, VerifyRBPFCorrections) {
    StateVector initial;
    initial.position = Vector3d(0, 0, -3000);
    initial.velocity = Vector3d(100, 0, 0);
    initial.quaternion = Quaterniond::Identity();

    navigation_system->initialize(initial);

    // Track RBPF corrections
    int rbpf_correction_count = 0;
    Vector3d total_correction = Vector3d::Zero();

    for (int i = 0; i < 1000; ++i) {
        auto sensor_data = sensor_manager->readNext();

        // Get state before update
        StateVector state_before = navigation_system->getState();

        // Process sensor
        if (sensor_data.has_imu) {
            navigation_system->processIMU(sensor_data.imu, sensor_data.dt);
        }

        // Process gradiometer (triggers RBPF)
        if (sensor_data.has_grad) {
            navigation_system->updateGravity(sensor_data.gradiometer);

            // Check if RBPF provided correction
            StateVector state_after = navigation_system->getState();
            Vector3d correction = state_after.position - state_before.position;

            // Subtract expected motion
            Vector3d expected_motion = state_before.velocity * sensor_data.dt;
            correction -= expected_motion;

            if (correction.norm() > 0.01) {  // Significant correction
                rbpf_correction_count++;
                total_correction += correction;
            }
        }
    }

    // Verify RBPF is providing corrections
    EXPECT_GT(rbpf_correction_count, 0)
        << "RBPF is not providing any corrections - map matching not working!";

    std::cout << "RBPF provided " << rbpf_correction_count
              << " corrections with total magnitude: "
              << total_correction.norm() << "m" << std::endl;
}