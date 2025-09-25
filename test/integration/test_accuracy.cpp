/**
 * Position Accuracy Tests
 * Validates navigation system meets 50m accuracy requirement
 */

#include <gtest/gtest.h>
#include <chrono>
#include "core/hierarchical_filter.h"
#include "utils/logger.h"

using namespace Navigation;
using namespace Eigen;

class AccuracyTest : public ::testing::Test {
protected:
    std::unique_ptr<HierarchicalFilter> navigation_system;

    void SetUp() override {
        Logger::getInstance().initialize(".", "ERROR");

        HierarchicalConfig config;
        config.ukf_rate = 100.0;
        config.rbpf_rate = 10.0;
        navigation_system = std::make_unique<HierarchicalFilter>(config, nullptr);
    }
};

// Test: 50-meter accuracy requirement over 10 minutes
TEST_F(AccuracyTest, FiftyMeterAccuracyRequirement) {
    // Initialize at known position
    StateVector initial;
    initial.position = Vector3d(0, 0, -5000);  // 5km altitude
    initial.velocity = Vector3d(200, 0, 0);     // 200 m/s cruise
    initial.quaternion = Quaterniond::Identity();
    initial.accel_bias = Vector3d(0.01, -0.01, 0.02);
    initial.gyro_bias = Vector3d(0.001, -0.001, 0.0005);

    navigation_system->initialize(initial);

    // Track ground truth position (simple kinematics)
    Vector3d true_position = initial.position;
    Vector3d true_velocity = initial.velocity;

    // Maximum position error seen
    double max_position_error = 0.0;
    double max_horizontal_error = 0.0;
    double max_vertical_error = 0.0;

    // Run for 10 minutes at 100Hz
    const int duration_seconds = 600;  // 10 minutes
    const double dt = 0.01;  // 100Hz
    const int total_iterations = duration_seconds * 100;

    for (int i = 0; i < total_iterations; ++i) {
        double t = i * dt;

        // Simulate realistic flight with mild maneuvering
        Vector3d accel(0, 0, -9.81);
        Vector3d gyro(0, 0, 0);

        // Add gentle turns every 2 minutes
        int phase = (i / 12000) % 3;
        if (phase == 1) {
            gyro.z() = 0.01;  // Gentle right turn
            accel.y() = 2.0;  // Centripetal acceleration
        } else if (phase == 2) {
            gyro.z() = -0.01; // Gentle left turn
            accel.y() = -2.0;
        }

        // Process IMU
        IMUData imu;
        imu.accel = accel;
        imu.gyro = gyro;
        imu.timestamp = t;
        navigation_system->processIMU(imu, dt);

        // Update true position (simple integration)
        true_velocity += (accel + Vector3d(0, 0, 9.81)) * dt;
        true_position += true_velocity * dt;

        // Add sensor updates for stability
        if (i % 10 == 0) {  // 10Hz barometer
            // Barometer measures true altitude with small noise
            double true_altitude = -true_position.z();
            double pressure = 101325.0 * exp(-true_altitude / 8500.0);

            BarometerData baro;
            baro.pressure = pressure + 10.0 * sin(t * 0.1);  // Small noise
            baro.temperature = 288.15 - true_altitude * 0.0065;
            baro.timestamp = t;
            navigation_system->updateBarometer(baro);
        }

        if (i % 20 == 0) {  // 5Hz magnetometer
            MagnetometerData mag;
            mag.field = Vector3d(30e-6, 0, 40e-6);
            mag.timestamp = t;
            navigation_system->updateMagnetometer(mag);
        }

        // Check position error every second
        if (i % 100 == 0) {
            StateVector nav_state = navigation_system->getState();
            Vector3d position_error = nav_state.position - true_position;

            double total_error = position_error.norm();
            double horizontal_error = position_error.head<2>().norm();
            double vertical_error = std::abs(position_error.z());

            max_position_error = std::max(max_position_error, total_error);
            max_horizontal_error = std::max(max_horizontal_error, horizontal_error);
            max_vertical_error = std::max(max_vertical_error, vertical_error);

            // Log significant drift
            if (total_error > 40.0) {
                std::cout << "Warning: Position error at t=" << t << "s: " << total_error << "m" << std::endl;
            }
        }
    }

    // Check 50m accuracy requirement
    EXPECT_LT(max_position_error, 50.0)
        << "Maximum position error (" << max_position_error
        << "m) exceeds 50m requirement over 10 minutes";

    EXPECT_LT(max_horizontal_error, 40.0)
        << "Maximum horizontal error: " << max_horizontal_error << "m";

    EXPECT_LT(max_vertical_error, 20.0)
        << "Maximum vertical error: " << max_vertical_error << "m";

    // Report final accuracy
    StateVector final_state = navigation_system->getState();
    Vector3d final_error = final_state.position - true_position;

    std::cout << "\n=== ACCURACY TEST RESULTS ===" << std::endl;
    std::cout << "Test duration: " << duration_seconds << " seconds" << std::endl;
    std::cout << "Maximum position error: " << max_position_error << " meters" << std::endl;
    std::cout << "Maximum horizontal error: " << max_horizontal_error << " meters" << std::endl;
    std::cout << "Maximum vertical error: " << max_vertical_error << " meters" << std::endl;
    std::cout << "Final position error: " << final_error.norm() << " meters" << std::endl;
    std::cout << "REQUIREMENT: < 50 meters" << std::endl;
    std::cout << "STATUS: " << (max_position_error < 50.0 ? "PASS ✓" : "FAIL ✗") << std::endl;
}

// Test: Position accuracy with sensor failures
TEST_F(AccuracyTest, AccuracyWithSensorFailures) {
    StateVector initial;
    initial.position = Vector3d(0, 0, -3000);
    initial.velocity = Vector3d(150, 50, 0);
    initial.quaternion = Quaterniond::Identity();

    navigation_system->initialize(initial);

    Vector3d true_position = initial.position;
    Vector3d true_velocity = initial.velocity;
    double max_error_during_failure = 0.0;

    // Run for 5 minutes with sensor failure in middle
    for (int i = 0; i < 30000; ++i) {  // 5 minutes at 100Hz
        double t = i * 0.01;

        // Basic IMU data
        IMUData imu;
        imu.accel = Vector3d(0, 0, -9.81);
        imu.gyro = Vector3d(0, 0, 0);
        imu.timestamp = t;
        navigation_system->processIMU(imu, 0.01);

        true_velocity += Vector3d(0, 0, 0);  // Constant velocity
        true_position += true_velocity * 0.01;

        // Normal sensor updates for first 2 minutes
        if (i < 12000) {
            if (i % 10 == 0) {
                BarometerData baro;
                baro.pressure = 70000.0;
                baro.temperature = 260.0;
                baro.timestamp = t;
                navigation_system->updateBarometer(baro);
            }
        }
        // No barometer for 1 minute (sensor failure)
        // Resume barometer after 3 minutes
        else if (i >= 18000) {
            if (i % 10 == 0) {
                BarometerData baro;
                baro.pressure = 70000.0;
                baro.temperature = 260.0;
                baro.timestamp = t;
                navigation_system->updateBarometer(baro);
            }
        }

        // Check error during failure period
        if (i >= 12000 && i < 18000 && i % 100 == 0) {
            StateVector state = navigation_system->getState();
            double error = (state.position - true_position).norm();
            max_error_during_failure = std::max(max_error_during_failure, error);
        }
    }

    // Even with 1-minute sensor failure, should maintain < 50m accuracy
    EXPECT_LT(max_error_during_failure, 50.0)
        << "Position error during sensor failure exceeds 50m: "
        << max_error_during_failure << "m";
}

// Test: Long-term drift assessment
TEST_F(AccuracyTest, LongTermDrift) {
    StateVector initial;
    initial.position = Vector3d(0, 0, -10000);
    initial.velocity = Vector3d(250, 0, 0);
    initial.quaternion = Quaterniond::Identity();

    navigation_system->initialize(initial);

    // Track drift rate over time
    std::vector<double> drift_per_minute;
    Vector3d last_true_position = initial.position;
    Vector3d true_velocity = initial.velocity;

    for (int minute = 0; minute < 10; ++minute) {
        Vector3d minute_start_position = last_true_position;

        // Run for 1 minute
        for (int i = 0; i < 6000; ++i) {  // 1 minute at 100Hz
            double t = (minute * 6000 + i) * 0.01;

            IMUData imu;
            imu.accel = Vector3d(0, 0, -9.81);
            imu.gyro = Vector3d(0, 0, 0);
            imu.timestamp = t;
            navigation_system->processIMU(imu, 0.01);

            last_true_position += true_velocity * 0.01;

            // Regular sensor updates
            if (i % 10 == 0) {
                BarometerData baro;
                baro.pressure = 26436.0;  // 10km altitude
                baro.temperature = 223.15;
                baro.timestamp = t;
                navigation_system->updateBarometer(baro);
            }

            if (i % 20 == 0) {
                MagnetometerData mag;
                mag.field = Vector3d(25e-6, 0, 45e-6);
                mag.timestamp = t;
                navigation_system->updateMagnetometer(mag);
            }
        }

        // Calculate drift for this minute
        StateVector state = navigation_system->getState();
        double drift = (state.position - last_true_position).norm();
        drift_per_minute.push_back(drift);
    }

    // Calculate average drift rate
    double total_drift = 0;
    for (double drift : drift_per_minute) {
        total_drift += drift;
    }
    double avg_drift_per_minute = total_drift / drift_per_minute.size();

    // Maximum accumulated drift after 10 minutes
    double max_drift = *std::max_element(drift_per_minute.begin(), drift_per_minute.end());

    EXPECT_LT(max_drift, 50.0)
        << "Accumulated drift exceeds 50m after 10 minutes: " << max_drift << "m";

    EXPECT_LT(avg_drift_per_minute, 10.0)
        << "Average drift rate too high: " << avg_drift_per_minute << "m/minute";

    std::cout << "\n=== DRIFT ANALYSIS ===" << std::endl;
    std::cout << "Average drift rate: " << avg_drift_per_minute << " m/minute" << std::endl;
    std::cout << "Maximum drift: " << max_drift << " meters" << std::endl;
    std::cout << "Projected 1-hour drift: " << avg_drift_per_minute * 60 << " meters" << std::endl;
}