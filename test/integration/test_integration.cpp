/**
 * Integration Tests
 * Tests the complete navigation system end-to-end
 */

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <thread>
#include <chrono>
#include <random>
#include "core/hierarchical_filter.h"
#include "utils/trajectory_analyzer.h"
#include "utils/logger.h"
#include "maps/map_manager.h"

using namespace Navigation;
using namespace std::chrono_literals;

class IntegrationTest : public ::testing::Test {
protected:
    std::unique_ptr<HierarchicalFilter> navigation_system;
    std::unique_ptr<TrajectoryAnalyzer> analyzer;

    void SetUp() override {
        Logger::getInstance().initialize(".", "ERROR");

        // Initialize navigation system
        HierarchicalConfig nav_config;
        nav_config.ukf_rate = 100.0;
        nav_config.rbpf_rate = 10.0;
        navigation_system = std::make_unique<HierarchicalFilter>(nav_config, nullptr);

        // Initialize trajectory analyzer
        analyzer = std::make_unique<TrajectoryAnalyzer>();
    }

    // Helper to simulate a complete flight
    struct FlightData {
        double time;
        Vector3d accel;
        Vector3d gyro;
        double pressure;
        double temperature;
        Vector3d mag_field;
        bool has_baro;
        bool has_mag;
    };

    std::vector<FlightData> generateFlightProfile() {
        std::vector<FlightData> profile;

        // Takeoff phase (0-60s): climb to altitude
        for (int i = 0; i < 6000; ++i) {  // 60s at 100Hz
            FlightData data;
            data.time = i * 0.01;
            data.accel = Vector3d(0.5, 0, -9.81 + 2.0);  // Climbing
            data.gyro = Vector3d(0, 0.01, 0);  // Slight pitch up
            data.pressure = 101325.0 - i * 2.0;  // Pressure decreasing
            data.temperature = 288.15 - i * 0.001;
            data.mag_field = Vector3d(30e-6, 0, 40e-6);
            data.has_baro = (i % 10 == 0);  // 10Hz baro
            data.has_mag = (i % 20 == 0);   // 5Hz mag
            profile.push_back(data);
        }

        // Cruise phase (60-180s): level flight with turns
        for (int i = 6000; i < 18000; ++i) {
            FlightData data;
            data.time = i * 0.01;
            double phase = 2 * M_PI * (i - 6000) / 3000.0;  // Complete turn every 30s
            data.accel = Vector3d(sin(phase) * 5.0, cos(phase) * 5.0, -9.81);
            data.gyro = Vector3d(0, 0, 0.1 * sin(phase));
            data.pressure = 80000.0;  // Constant altitude
            data.temperature = 260.0;
            data.mag_field = Vector3d(30e-6 * cos(phase), 30e-6 * sin(phase), 40e-6);
            data.has_baro = (i % 10 == 0);
            data.has_mag = (i % 20 == 0);
            profile.push_back(data);
        }

        // Descent phase (180-240s): descend to landing
        for (int i = 18000; i < 24000; ++i) {
            FlightData data;
            data.time = i * 0.01;
            data.accel = Vector3d(-0.5, 0, -9.81 - 1.0);  // Descending
            data.gyro = Vector3d(0, -0.005, 0);  // Slight pitch down
            data.pressure = 80000.0 + (i - 18000) * 3.5;
            data.temperature = 260.0 + (i - 18000) * 0.0047;
            data.mag_field = Vector3d(30e-6, 0, 40e-6);
            data.has_baro = (i % 10 == 0);
            data.has_mag = (i % 20 == 0);
            profile.push_back(data);
        }

        return profile;
    }
};

// Test 1: Complete Flight Simulation
TEST_F(IntegrationTest, CompleteFlightSimulation) {
    // Initialize navigation at start position
    StateVector initial;
    initial.position = Vector3d(0, 0, -1000);  // 1000m altitude
    initial.velocity = Vector3d(100, 0, 0);    // 100 m/s forward
    initial.quaternion = Quaterniond::Identity();
    initial.accel_bias = Vector3d(0.01, -0.01, 0.02);
    initial.gyro_bias = Vector3d(0.001, -0.001, 0.0005);

    navigation_system->initialize(initial);

    // Run flight profile
    auto flight_data = generateFlightProfile();
    std::vector<StateVector> trajectory;

    for (const auto& data : flight_data) {
        // IMU data at 100Hz
        IMUData imu;
        imu.accel = data.accel;
        imu.gyro = data.gyro;
        imu.timestamp = data.time;
        navigation_system->processIMU(imu, 0.01);

        // Barometer at 10Hz
        if (data.has_baro) {
            BarometerData baro;
            baro.pressure = data.pressure;
            baro.temperature = data.temperature;
            baro.timestamp = data.time;
            navigation_system->updateBarometer(baro);
        }

        // Magnetometer at 5Hz
        if (data.has_mag) {
            MagnetometerData mag;
            mag.field = data.mag_field;
            mag.timestamp = data.time;
            navigation_system->updateMagnetometer(mag);
        }

        // Store trajectory
        if (static_cast<int>(data.time * 10) % 10 == 0) {  // Every 1s
            trajectory.push_back(navigation_system->getState());
        }
    }

    // Verify trajectory characteristics
    ASSERT_GT(trajectory.size(), 200) << "Should have >200 trajectory points";

    // Check altitude profile
    double max_altitude = -1e6;
    double min_altitude = 1e6;
    for (const auto& state : trajectory) {
        max_altitude = std::max(max_altitude, -state.position.z());
        min_altitude = std::min(min_altitude, -state.position.z());
    }

    EXPECT_GT(max_altitude, 2000) << "Should reach >2000m altitude";
    EXPECT_LT(min_altitude, 1500) << "Should descend below 1500m";

    // Check that state remains bounded
    StateVector final_state = navigation_system->getState();
    EXPECT_LT(final_state.position.norm(), 100000) << "Position should remain bounded";
    EXPECT_LT(final_state.velocity.norm(), 1000) << "Velocity should remain bounded";
    EXPECT_NEAR(final_state.quaternion.norm(), 1.0, 0.01) << "Quaternion should stay normalized";
}

// Test 2: Multi-Sensor Fusion
TEST_F(IntegrationTest, MultiSensorFusion) {
    StateVector initial;
    initial.position = Vector3d(100, 200, -3000);
    initial.velocity = Vector3d(50, -30, 2);
    initial.quaternion = Quaterniond::Identity();

    navigation_system->initialize(initial);

    // Create sensor data with known relationships
    double base_time = 0.0;

    for (int i = 0; i < 1000; ++i) {
        base_time += 0.01;

        // Constant turn rate
        Vector3d gyro(0, 0, 0.1);  // 0.1 rad/s yaw rate

        // Centripetal acceleration from turn
        double speed = 100.0;  // m/s
        double turn_radius = speed / 0.1;
        Vector3d accel(0, speed * 0.1, -9.81);  // Lateral accel from turn

        IMUData imu;
        imu.accel = accel;
        imu.gyro = gyro;
        imu.timestamp = base_time;
        navigation_system->processIMU(imu, 0.01);

        // Periodic sensor updates
        if (i % 10 == 0) {
            // Barometer shows constant altitude
            BarometerData baro;
            baro.pressure = 70000.0;  // ~3000m altitude
            baro.temperature = 268.15;
            baro.timestamp = base_time;
            navigation_system->updateBarometer(baro);
        }

        if (i % 20 == 0) {
            // Rotating magnetic field (due to yaw)
            double heading = 0.1 * base_time;  // Accumulated heading
            Vector3d mag_earth(30e-6, 0, 40e-6);
            Vector3d mag_body;
            mag_body.x() = mag_earth.x() * cos(heading) + mag_earth.y() * sin(heading);
            mag_body.y() = -mag_earth.x() * sin(heading) + mag_earth.y() * cos(heading);
            mag_body.z() = mag_earth.z();

            MagnetometerData mag;
            mag.field = mag_body;
            mag.timestamp = base_time;
            navigation_system->updateMagnetometer(mag);
        }
    }

    StateVector final_state = navigation_system->getState();

    // After 10s of 0.1 rad/s turn, heading should change by ~1 radian
    Vector3d euler = final_state.quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
    EXPECT_NEAR(std::abs(euler.z()), 1.0, 0.2) << "Heading should change by ~1 radian";

    // Altitude should remain relatively constant
    EXPECT_NEAR(final_state.position.z(), initial.position.z(), 100)
        << "Altitude should remain stable with baro updates";
}

// Test 3: Sensor Failure Recovery
TEST_F(IntegrationTest, SensorFailureRecovery) {
    StateVector initial;
    initial.position = Vector3d(0, 0, -2000);
    initial.velocity = Vector3d(100, 0, 0);
    initial.quaternion = Quaterniond::Identity();

    navigation_system->initialize(initial);

    // Phase 1: Normal operation with all sensors
    for (int i = 0; i < 500; ++i) {
        IMUData imu;
        imu.accel = Vector3d(0, 0, -9.81);
        imu.gyro = Vector3d(0, 0, 0);
        imu.timestamp = i * 0.01;
        navigation_system->processIMU(imu, 0.01);

        if (i % 10 == 0) {
            BarometerData baro;
            baro.pressure = 80000.0;
            baro.temperature = 268.15;
            baro.timestamp = i * 0.01;
            navigation_system->updateBarometer(baro);
        }

        if (i % 20 == 0) {
            MagnetometerData mag;
            mag.field = Vector3d(30e-6, 0, 40e-6);
            mag.timestamp = i * 0.01;
            navigation_system->updateMagnetometer(mag);
        }
    }

    StateVector pre_failure = navigation_system->getState();

    // Phase 2: Barometer failure (no updates for 10 seconds)
    for (int i = 500; i < 1500; ++i) {
        IMUData imu;
        imu.accel = Vector3d(0, 0, -9.81);
        imu.gyro = Vector3d(0, 0, 0);
        imu.timestamp = i * 0.01;
        navigation_system->processIMU(imu, 0.01);

        // No barometer updates!

        if (i % 20 == 0) {
            MagnetometerData mag;
            mag.field = Vector3d(30e-6, 0, 40e-6);
            mag.timestamp = i * 0.01;
            navigation_system->updateMagnetometer(mag);
        }
    }

    StateVector during_failure = navigation_system->getState();

    // Altitude uncertainty should increase without baro
    MatrixXd cov_during = navigation_system->getCovariance();
    double altitude_variance_during = cov_during(2, 2);

    // Phase 3: Barometer recovery
    for (int i = 1500; i < 2000; ++i) {
        IMUData imu;
        imu.accel = Vector3d(0, 0, -9.81);
        imu.gyro = Vector3d(0, 0, 0);
        imu.timestamp = i * 0.01;
        navigation_system->processIMU(imu, 0.01);

        if (i % 10 == 0) {
            BarometerData baro;
            baro.pressure = 80000.0;
            baro.temperature = 268.15;
            baro.timestamp = i * 0.01;
            navigation_system->updateBarometer(baro);
        }

        if (i % 20 == 0) {
            MagnetometerData mag;
            mag.field = Vector3d(30e-6, 0, 40e-6);
            mag.timestamp = i * 0.01;
            navigation_system->updateMagnetometer(mag);
        }
    }

    StateVector after_recovery = navigation_system->getState();
    MatrixXd cov_after = navigation_system->getCovariance();
    double altitude_variance_after = cov_after(2, 2);

    // System should recover after sensor returns
    EXPECT_LT(altitude_variance_after, altitude_variance_during * 0.5)
        << "Altitude uncertainty should decrease after baro recovery";

    // State should remain stable throughout
    EXPECT_LT((during_failure.position - pre_failure.position).norm(), 1000)
        << "Position shouldn't drift too much during sensor failure";
}

// Test 4: Performance Under Stress
TEST_F(IntegrationTest, PerformanceUnderStress) {
    StateVector initial;
    initial.position = Vector3d(0, 0, -5000);
    initial.velocity = Vector3d(200, 0, 0);  // High speed
    initial.quaternion = Quaterniond::Identity();

    navigation_system->initialize(initial);

    // Aggressive maneuvering
    for (int i = 0; i < 2000; ++i) {
        double t = i * 0.01;

        // High-G maneuvers
        Vector3d accel(
            30 * sin(10 * t),      // 3G lateral oscillation
            30 * cos(10 * t),      // 3G lateral oscillation
            -9.81 + 20 * sin(5 * t) // Vertical oscillation
        );

        // Rapid rotation
        Vector3d gyro(
            0.5 * sin(20 * t),
            0.5 * cos(20 * t),
            0.3 * sin(15 * t)
        );

        IMUData imu;
        imu.accel = accel;
        imu.gyro = gyro;
        imu.timestamp = t;
        navigation_system->processIMU(imu, 0.01);

        // Measurements with noise
        if (i % 10 == 0) {
            BarometerData baro;
            baro.pressure = 50000.0 + 1000 * sin(t);  // Oscillating pressure
            baro.temperature = 250.0 + 10 * sin(t * 0.5);
            baro.timestamp = t;
            navigation_system->updateBarometer(baro);
        }

        if (i % 20 == 0) {
            // Disturbed magnetic field
            MagnetometerData mag;
            mag.field = Vector3d(
                30e-6 + 10e-6 * sin(t * 2),
                10e-6 * cos(t * 2),
                40e-6
            );
            mag.timestamp = t;
            navigation_system->updateMagnetometer(mag);
        }

        // Verify state remains valid
        StateVector state = navigation_system->getState();
        ASSERT_TRUE(state.position.allFinite()) << "Position became invalid at t=" << t;
        ASSERT_TRUE(state.velocity.allFinite()) << "Velocity became invalid at t=" << t;
        ASSERT_NEAR(state.quaternion.norm(), 1.0, 0.01) << "Quaternion denormalized at t=" << t;
    }

    // System should survive aggressive maneuvers
    StateVector final_state = navigation_system->getState();
    EXPECT_LT(final_state.position.norm(), 50000) << "Position exploded under stress";
    EXPECT_LT(final_state.velocity.norm(), 1000) << "Velocity exploded under stress";
}

// Test 5: Long Duration Stability
TEST_F(IntegrationTest, LongDurationStability) {
    StateVector initial;
    initial.position = Vector3d(0, 0, -10000);
    initial.velocity = Vector3d(250, 0, 0);  // Cruise speed
    initial.quaternion = Quaterniond::Identity();
    initial.accel_bias = Vector3d(0.01, -0.01, 0.02);
    initial.gyro_bias = Vector3d(0.001, -0.001, 0.0005);

    navigation_system->initialize(initial);

    const int hours = 2;  // Simulate 2-hour flight
    const int total_steps = hours * 3600 * 100;  // 100Hz for 2 hours

    // Track drift metrics
    double total_distance = 0;
    Vector3d last_position = initial.position;

    for (int i = 0; i < total_steps; ++i) {
        double t = i * 0.01;

        // Gentle cruise with occasional turns
        double turn_phase = (i / 36000) % 4;  // Change every 6 minutes
        Vector3d accel(0, 0, -9.81);
        Vector3d gyro(0, 0, 0);

        if (turn_phase == 1) {
            gyro.z() = 0.02;  // Gentle right turn
            accel.y() = 5.0;  // Centripetal acceleration
        } else if (turn_phase == 3) {
            gyro.z() = -0.02; // Gentle left turn
            accel.y() = -5.0;
        }

        IMUData imu;
        imu.accel = accel;
        imu.gyro = gyro;
        imu.timestamp = t;
        navigation_system->processIMU(imu, 0.01);

        // Regular sensor updates
        if (i % 10 == 0) {  // 10Hz baro
            BarometerData baro;
            baro.pressure = 26436.0;  // 10km altitude
            baro.temperature = 223.15;  // -50°C at altitude
            baro.timestamp = t;
            navigation_system->updateBarometer(baro);
        }

        if (i % 20 == 0) {  // 5Hz mag
            MagnetometerData mag;
            mag.field = Vector3d(25e-6, 0, 45e-6);  // High altitude field
            mag.timestamp = t;
            navigation_system->updateMagnetometer(mag);
        }

        // Track distance traveled
        if (i % 100 == 0) {  // Every second
            StateVector state = navigation_system->getState();
            total_distance += (state.position - last_position).norm();
            last_position = state.position;
        }

        // Periodic health checks
        if (i % 36000 == 0) {  // Every 6 minutes
            StateVector state = navigation_system->getState();
            MatrixXd cov = navigation_system->getCovariance();

            ASSERT_TRUE(state.position.allFinite()) << "Position invalid at hour " << (t/3600.0);
            ASSERT_TRUE(cov.allFinite()) << "Covariance invalid at hour " << (t/3600.0);

            // Check covariance remains bounded
            double max_variance = cov.diagonal().maxCoeff();
            EXPECT_LT(max_variance, 10000.0) << "Covariance unbounded at hour " << (t/3600.0);
        }
    }

    // Expected distance: ~250 m/s * 2 hours = 1,800 km
    double expected_distance = 250.0 * hours * 3600.0;
    EXPECT_NEAR(total_distance, expected_distance, expected_distance * 0.1)
        << "Distance traveled doesn't match expected cruise";

    // Check bias estimation stability
    StateVector final_state = navigation_system->getState();
    EXPECT_LT(final_state.accel_bias.norm(), 1.0) << "Accelerometer bias grew too large";
    EXPECT_LT(final_state.gyro_bias.norm(), 0.1) << "Gyro bias grew too large";
}

// Test 6: Real-World Scenario - Turbulence
TEST_F(IntegrationTest, TurbulenceHandling) {
    StateVector initial;
    initial.position = Vector3d(0, 0, -8000);
    initial.velocity = Vector3d(200, 0, 0);
    initial.quaternion = Quaterniond::Identity();

    navigation_system->initialize(initial);

    std::default_random_engine generator(42);  // Fixed seed for reproducibility
    std::normal_distribution<double> turbulence(0.0, 5.0);  // 0.5G RMS turbulence

    for (int i = 0; i < 6000; ++i) {  // 1 minute of turbulence
        double t = i * 0.01;

        // Base motion plus turbulence
        Vector3d accel(
            turbulence(generator),
            turbulence(generator),
            -9.81 + turbulence(generator)
        );

        Vector3d gyro(
            turbulence(generator) * 0.01,
            turbulence(generator) * 0.01,
            turbulence(generator) * 0.01
        );

        IMUData imu;
        imu.accel = accel;
        imu.gyro = gyro;
        imu.timestamp = t;
        navigation_system->processIMU(imu, 0.01);

        // Sensor updates help stabilize
        if (i % 10 == 0) {
            BarometerData baro;
            baro.pressure = 35651.0 + 100 * sin(t * 0.5);  // Some altitude variation
            baro.temperature = 236.15;
            baro.timestamp = t;
            navigation_system->updateBarometer(baro);
        }

        if (i % 20 == 0) {
            MagnetometerData mag;
            mag.field = Vector3d(28e-6, 0, 42e-6);
            mag.timestamp = t;
            navigation_system->updateMagnetometer(mag);
        }
    }

    StateVector final_state = navigation_system->getState();

    // Should maintain approximate altitude despite turbulence
    EXPECT_NEAR(final_state.position.z(), initial.position.z(), 500)
        << "Altitude deviated too much in turbulence";

    // Should maintain approximate heading
    Vector3d euler = final_state.quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
    EXPECT_NEAR(euler.z(), 0.0, 0.5) << "Heading deviated too much in turbulence";
}

// Test 7: Configuration and Initialization
TEST_F(IntegrationTest, ConfigurationHandling) {
    // Test different initialization scenarios

    // Scenario 1: Ground start
    StateVector ground_start;
    ground_start.position = Vector3d(0, 0, 0);  // Sea level
    ground_start.velocity = Vector3d::Zero();
    ground_start.quaternion = Quaterniond::Identity();

    HierarchicalConfig config1;
    config1.ukf_rate = 100.0;
    config1.rbpf_rate = 10.0;
    auto system1 = std::make_unique<HierarchicalFilter>(config1, nullptr);
    system1->initialize(ground_start);

    // Should handle stationary start
    for (int i = 0; i < 100; ++i) {
        IMUData imu;
        imu.accel = Vector3d(0, 0, -9.81);
        imu.gyro = Vector3d::Zero();
        imu.timestamp = i * 0.01;
        system1->processIMU(imu, 0.01);
    }

    StateVector state1 = system1->getState();
    EXPECT_LT(state1.position.norm(), 10) << "Position drifted while stationary";

    // Scenario 2: Air start
    StateVector air_start;
    air_start.position = Vector3d(1000, 2000, -5000);
    air_start.velocity = Vector3d(150, -50, 10);
    air_start.quaternion = Quaterniond(0.9239, 0, 0.3827, 0);  // 45° heading

    HierarchicalConfig config2;
    config2.ukf_rate = 200.0;  // Higher rate
    config2.rbpf_rate = 20.0;
    auto system2 = std::make_unique<HierarchicalFilter>(config2, nullptr);
    system2->initialize(air_start);

    // Should handle moving start
    for (int i = 0; i < 100; ++i) {
        IMUData imu;
        imu.accel = Vector3d(0, 0, -9.81);
        imu.gyro = Vector3d(0, 0, 0.1);  // Turning
        imu.timestamp = i * 0.005;
        system2->processIMU(imu, 0.005);
    }

    StateVector state2 = system2->getState();
    EXPECT_TRUE(state2.position.allFinite()) << "Invalid state from air start";
}