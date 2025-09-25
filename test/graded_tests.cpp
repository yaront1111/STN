/**
 * GPS-Free Navigation System - Graded Test Runner
 * Comprehensive test suite for accuracy validation
 * Target: <50m position error requirement
 */

#include <gtest/gtest.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <cmath>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "../src/core/hierarchical_filter.h"
#include "../src/sensors/sensor_manager.h"
#include "../src/maps/composite_map_manager.h"
#include "../src/utils/logger.h"
#include "../src/utils/data_validator.h"

using namespace Navigation;
using namespace Eigen;
using namespace std::chrono;

// Test configuration
struct TestConfig {
    int duration_seconds = 600;  // 10 minutes default
    double max_position_error = 50.0;  // 50m requirement
    double max_horizontal_error = 40.0;
    double max_vertical_error = 20.0;
    bool verbose = false;
    bool save_results = true;
};

// Test results structure
struct TestResult {
    std::string test_name;
    bool passed;
    double max_error;
    double mean_error;
    double final_error;
    int duration_seconds;
    std::string failure_reason;
};

class NavigationAccuracyTest : public ::testing::Test {
protected:
    std::unique_ptr<HierarchicalFilter> navigation_system;
    std::shared_ptr<CompositeMapManager> maps;
    TestConfig config;
    std::vector<TestResult> results;

    void SetUp() override {
        // Initialize logger with minimal output
        Logger::getInstance().initialize("test_logs/", "ERROR");

        // Load test configuration if available
        try {
            YAML::Node yaml_config = YAML::LoadFile("test_config.yaml");
            if (yaml_config["test"]["duration_s"]) {
                config.duration_seconds = yaml_config["test"]["duration_s"].as<int>();
            }
            if (yaml_config["test"]["max_position_error_m"]) {
                config.max_position_error = yaml_config["test"]["max_position_error_m"].as<double>();
            }
        } catch (...) {
            // Use defaults if config not found
        }

        // Initialize navigation system
        YAML::Node ukf_config, rbpf_config;
        ukf_config["state_dimension"] = 21;
        ukf_config["sigma_points"]["alpha"] = 0.1;
        ukf_config["sigma_points"]["beta"] = 2.0;
        ukf_config["sigma_points"]["kappa"] = -37;

        rbpf_config["num_particles"] = 1000;
        rbpf_config["resampling"]["threshold"] = 0.7;
        rbpf_config["reset"]["enable"] = true;
        rbpf_config["reset"]["min_interval_s"] = 15;

        navigation_system = std::make_unique<HierarchicalFilter>(ukf_config, rbpf_config, maps);
    }

    void TearDown() override {
        // Save test results
        if (config.save_results) {
            saveTestResults();
        }
    }

    void saveTestResults() {
        std::ofstream file("test_results.txt");
        file << "GPS-Free Navigation System - Test Results\n";
        file << "==========================================\n\n";

        int passed = 0;
        for (const auto& result : results) {
            if (result.passed) passed++;

            file << "Test: " << result.test_name << "\n";
            file << "  Status: " << (result.passed ? "PASSED" : "FAILED") << "\n";
            file << "  Max Error: " << std::fixed << std::setprecision(2)
                 << result.max_error << " m\n";
            file << "  Mean Error: " << result.mean_error << " m\n";
            file << "  Final Error: " << result.final_error << " m\n";
            if (!result.passed) {
                file << "  Failure: " << result.failure_reason << "\n";
            }
            file << "\n";
        }

        file << "Summary: " << passed << "/" << results.size() << " tests passed\n";
        file.close();
    }

    TestResult runAccuracyTest(
        const std::string& test_name,
        const StateVector& initial_state,
        int duration_seconds,
        bool with_maneuvers = false
    ) {
        TestResult result;
        result.test_name = test_name;
        result.duration_seconds = duration_seconds;
        result.passed = true;

        navigation_system->initialize(initial_state);

        // Track ground truth
        Vector3d true_position = initial_state.position;
        Vector3d true_velocity = initial_state.velocity;
        Quaterniond true_attitude = initial_state.quaternion;

        // Error tracking
        double max_position_error = 0.0;
        double sum_position_error = 0.0;
        int error_count = 0;

        const double dt = 0.01;  // 100Hz
        const int total_iterations = duration_seconds * 100;

        for (int i = 0; i < total_iterations; ++i) {
            double t = i * dt;

            // Generate realistic IMU data
            Vector3d accel(0, 0, -9.81);
            Vector3d gyro(0, 0, 0);

            // Add maneuvers if requested
            if (with_maneuvers) {
                int phase = (i / 12000) % 3;  // Change every 2 minutes
                if (phase == 1) {
                    gyro.z() = 0.01;  // Right turn
                    accel.y() = 2.0;  // Centripetal acceleration
                } else if (phase == 2) {
                    gyro.z() = -0.01;  // Left turn
                    accel.y() = -2.0;
                }
            }

            // Process IMU
            IMUData imu;
            imu.accel = accel + Vector3d::Random() * 0.01;  // Add noise
            imu.gyro = gyro + Vector3d::Random() * 0.001;
            imu.timestamp = t;
            navigation_system->processIMU(imu, dt);

            // Update ground truth
            true_velocity += (accel + Vector3d(0, 0, 9.81)) * dt;
            true_position += true_velocity * dt;

            // Add sensor measurements periodically
            if (i % 10 == 0) {  // 10Hz barometer
                BarometerData baro;
                double true_altitude = -true_position.z();
                baro.pressure = 101325.0 * exp(-true_altitude / 8500.0) + 5.0 * sin(t * 0.1);
                baro.temperature = 288.15 - true_altitude * 0.0065;
                baro.timestamp = t;
                navigation_system->updateBarometer(baro);
            }

            if (i % 20 == 0) {  // 5Hz magnetometer
                MagnetometerData mag;
                mag.field = Vector3d(30e-6, 0, 40e-6) + Vector3d::Random() * 1e-6;
                mag.timestamp = t;
                navigation_system->updateMagnetometer(mag);
            }

            // Check position error every second
            if (i % 100 == 0 && i > 0) {
                StateVector nav_state = navigation_system->getState();
                Vector3d position_error = nav_state.position - true_position;
                double total_error = position_error.norm();

                max_position_error = std::max(max_position_error, total_error);
                sum_position_error += total_error;
                error_count++;

                // Log significant errors
                if (config.verbose && total_error > 40.0) {
                    std::cout << "  [" << test_name << "] Warning at t=" << t
                              << "s: error=" << total_error << "m" << std::endl;
                }
            }
        }

        // Get final state
        StateVector final_state = navigation_system->getState();
        Vector3d final_error = final_state.position - true_position;

        // Fill test result
        result.max_error = max_position_error;
        result.mean_error = error_count > 0 ? sum_position_error / error_count : 0;
        result.final_error = final_error.norm();

        // Check pass criteria
        if (result.max_error >= config.max_position_error) {
            result.passed = false;
            result.failure_reason = "Maximum position error (" +
                std::to_string(result.max_error) + "m) exceeds " +
                std::to_string(config.max_position_error) + "m requirement";
        }

        results.push_back(result);
        return result;
    }
};

// PRIMARY TEST: 50-meter accuracy requirement over 10 minutes
TEST_F(NavigationAccuracyTest, Critical_FiftyMeterAccuracyRequirement) {
    StateVector initial;
    initial.position = Vector3d(0, 0, -5000);  // 5km altitude
    initial.velocity = Vector3d(200, 0, 0);     // 200 m/s cruise
    initial.quaternion = Quaterniond::Identity();
    initial.accel_bias = Vector3d(0.01, -0.01, 0.02);
    initial.gyro_bias = Vector3d(0.001, -0.001, 0.0005);

    std::cout << "\n=== PRIMARY ACCURACY TEST ===" << std::endl;
    std::cout << "Testing 50m accuracy requirement over 10 minutes..." << std::endl;

    auto result = runAccuracyTest(
        "50m_accuracy_requirement",
        initial,
        600,  // 10 minutes
        true  // with maneuvers
    );

    std::cout << "Results:" << std::endl;
    std::cout << "  Max Position Error: " << std::fixed << std::setprecision(2)
              << result.max_error << " m" << std::endl;
    std::cout << "  Mean Position Error: " << result.mean_error << " m" << std::endl;
    std::cout << "  Final Position Error: " << result.final_error << " m" << std::endl;
    std::cout << "  REQUIREMENT: < 50 m" << std::endl;
    std::cout << "  STATUS: " << (result.passed ? "PASS ✓" : "FAIL ✗") << std::endl;

    EXPECT_LT(result.max_error, 50.0)
        << "Maximum position error exceeds 50m requirement";
    EXPECT_LT(result.mean_error, 30.0)
        << "Mean position error is too high";
}

// Test with sensor failures
TEST_F(NavigationAccuracyTest, Critical_AccuracyWithSensorFailures) {
    StateVector initial;
    initial.position = Vector3d(0, 0, -3000);
    initial.velocity = Vector3d(150, 50, 0);
    initial.quaternion = Quaterniond::Identity();

    std::cout << "\n=== SENSOR FAILURE TEST ===" << std::endl;
    std::cout << "Testing accuracy with 1-minute sensor outage..." << std::endl;

    navigation_system->initialize(initial);

    Vector3d true_position = initial.position;
    Vector3d true_velocity = initial.velocity;
    double max_error_during_failure = 0.0;

    for (int i = 0; i < 30000; ++i) {  // 5 minutes at 100Hz
        double t = i * 0.01;

        IMUData imu;
        imu.accel = Vector3d(0, 0, -9.81);
        imu.gyro = Vector3d(0, 0, 0);
        imu.timestamp = t;
        navigation_system->processIMU(imu, 0.01);

        true_position += true_velocity * 0.01;

        // Normal sensor operation for first 2 minutes
        if (i < 12000) {
            if (i % 10 == 0) {
                BarometerData baro;
                baro.pressure = 70000.0;
                baro.temperature = 260.0;
                baro.timestamp = t;
                navigation_system->updateBarometer(baro);
            }
        }
        // No sensors for 1 minute (failure)
        // Resume after 3 minutes
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

    std::cout << "Results:" << std::endl;
    std::cout << "  Max Error During Failure: " << std::fixed << std::setprecision(2)
              << max_error_during_failure << " m" << std::endl;
    std::cout << "  STATUS: " << (max_error_during_failure < 50.0 ? "PASS ✓" : "FAIL ✗")
              << std::endl;

    EXPECT_LT(max_error_during_failure, 50.0)
        << "Position error during sensor failure exceeds 50m";
}

// Long-term drift assessment
TEST_F(NavigationAccuracyTest, Critical_LongTermDriftUnderFiftyMeters) {
    StateVector initial;
    initial.position = Vector3d(0, 0, -10000);
    initial.velocity = Vector3d(250, 0, 0);
    initial.quaternion = Quaterniond::Identity();

    std::cout << "\n=== LONG-TERM DRIFT TEST ===" << std::endl;
    std::cout << "Testing drift rate over 10 minutes..." << std::endl;

    auto result = runAccuracyTest(
        "long_term_drift",
        initial,
        600,  // 10 minutes
        false  // no maneuvers
    );

    double drift_rate_per_minute = result.mean_error / 10.0;

    std::cout << "Results:" << std::endl;
    std::cout << "  Maximum Drift: " << std::fixed << std::setprecision(2)
              << result.max_error << " m" << std::endl;
    std::cout << "  Average Drift Rate: " << drift_rate_per_minute << " m/minute" << std::endl;
    std::cout << "  Projected 30-min Drift: " << drift_rate_per_minute * 30 << " m" << std::endl;
    std::cout << "  STATUS: " << (result.max_error < 50.0 ? "PASS ✓" : "FAIL ✗")
              << std::endl;

    EXPECT_LT(result.max_error, 50.0)
        << "Accumulated drift exceeds 50m after 10 minutes";
    EXPECT_LT(drift_rate_per_minute, 10.0)
        << "Drift rate is too high";
}

// Grade calculation based on test results
class GradeCalculator : public ::testing::Environment {
public:
    void TearDown() override {
        // Calculate and display grade
        int total_tests = ::testing::UnitTest::GetInstance()->total_test_count();
        int failed_tests = ::testing::UnitTest::GetInstance()->failed_test_count();
        int passed_tests = total_tests - failed_tests;

        double percentage = (total_tests > 0) ? (100.0 * passed_tests / total_tests) : 0;

        std::string grade;
        if (percentage >= 95) grade = "A+";
        else if (percentage >= 90) grade = "A";
        else if (percentage >= 87) grade = "B+";
        else if (percentage >= 83) grade = "B";
        else if (percentage >= 80) grade = "B-";
        else if (percentage >= 77) grade = "C+";
        else if (percentage >= 73) grade = "C";
        else if (percentage >= 70) grade = "C-";
        else if (percentage >= 60) grade = "D";
        else grade = "F";

        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "GRADE ASSESSMENT" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "Total Tests: " << total_tests << std::endl;
        std::cout << "Passed: " << passed_tests << std::endl;
        std::cout << "Failed: " << failed_tests << std::endl;
        std::cout << "Success Rate: " << std::fixed << std::setprecision(1)
                  << percentage << "%" << std::endl;
        std::cout << "\nFINAL GRADE: " << grade << std::endl;

        if (percentage >= 100) {
            std::cout << "\n🎉 PERFECT SCORE! All accuracy requirements met!" << std::endl;
        } else if (percentage >= 87) {
            std::cout << "\n✓ System meets accuracy requirements for production use." << std::endl;
        } else {
            std::cout << "\n⚠ System needs improvement to meet accuracy requirements." << std::endl;
        }
        std::cout << std::string(60, '=') << std::endl;

        // Save grade to file
        std::ofstream grade_file("grade_report.txt");
        grade_file << "GPS-Free Navigation System Grade Report\n";
        grade_file << "========================================\n\n";
        grade_file << "Date: " << __DATE__ << " " << __TIME__ << "\n";
        grade_file << "Tests Passed: " << passed_tests << "/" << total_tests << "\n";
        grade_file << "Success Rate: " << percentage << "%\n";
        grade_file << "Final Grade: " << grade << "\n\n";
        grade_file << "Primary Requirement: <50m position error over 10 minutes\n";
        grade_file << "Status: " << (failed_tests == 0 ? "MET" : "NOT MET") << "\n";
        grade_file.close();
    }
};

int main(int argc, char **argv) {
    std::cout << "GPS-Free Navigation System - Graded Test Suite" << std::endl;
    std::cout << "Version: 2.0.0" << std::endl;
    std::cout << "Target Requirement: <50m position error" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new GradeCalculator());

    int result = RUN_ALL_TESTS();

    return result;
}