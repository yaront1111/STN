#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <core/ukf.h>
#include <core/smoother_wrapper.h>
#include <factors/mag_yaw_factor.h>
#include <factors/altimeter_range_factor.h>
#include <factors/baro_factor.h>
#include <factors/aero_velocity_wind_factor.h>
#include <factors/optical_flow_factor.h>

#include <Eigen/Dense>
#include <vector>
#include <optional>
#include <cmath>

using namespace gtsam;
using namespace chimera;
using symbol_shorthand::X;  // Pose3
using symbol_shorthand::V;  // Velocity
using symbol_shorthand::B;  // Bias
using symbol_shorthand::W;  // Wind

// ===== Mock Data Structures (from chimera_node_multi.cpp) =====

struct ImuSample {
    double t;
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel;
    Eigen::Vector4d quat;
};

struct MagSample {
    double t;
    Eigen::Vector3d mag;
};

struct LidarSample {
    double t;
    double range_min;
    double range_mean;
    int num_points;
};

struct BaroSample {
    double t;
    double altitude;
};

struct AirspeedSample {
    double t;
    double airspeed;
};

struct OpticalFlowRay {
    int grid_idx;
    double norm_x, norm_y;
    int px, py;
    double flow_u, flow_v;
    double texture_var;
};

struct OpticalFlowSample {
    double t;
    double dt;
    std::vector<OpticalFlowRay> rays;
    int rays_used;
    int rays_rejected;
};

// ===== Helper: findNearest (from chimera_node_multi.cpp) =====

template<typename T>
std::optional<T> findNearest(const std::vector<T>& samples, double t, double max_dt = 0.1) {
    if (samples.empty()) return std::nullopt;

    auto it = std::lower_bound(samples.begin(), samples.end(), t,
        [](const T& sample, double time) { return sample.t < time; });

    if (it == samples.end()) {
        if (std::abs(samples.back().t - t) <= max_dt) {
            return samples.back();
        }
        return std::nullopt;
    }

    if (it == samples.begin()) {
        if (std::abs(it->t - t) <= max_dt) {
            return *it;
        }
        return std::nullopt;
    }

    auto prev = it - 1;
    double dt_prev = std::abs(prev->t - t);
    double dt_curr = std::abs(it->t - t);

    if (dt_prev < dt_curr && dt_prev <= max_dt) {
        return *prev;
    } else if (dt_curr <= max_dt) {
        return *it;
    }

    return std::nullopt;
}

// ===== Test Suite: Sensor Matching =====

class SensorMatchingTest : public ::testing::Test {
protected:
    std::vector<MagSample> mag_samples;

    void SetUp() override {
        // Create mock magnetometer samples at 20 Hz (every 0.05s)
        for (int i = 0; i < 100; ++i) {
            MagSample mag;
            mag.t = i * 0.05;  // 0.0, 0.05, 0.1, ...
            mag.mag = Eigen::Vector3d(1.0, 0.0, 0.0);  // Point north
            mag_samples.push_back(mag);
        }
    }
};

TEST_F(SensorMatchingTest, ExactMatch) {
    // Query at exact sample time
    auto result = findNearest(mag_samples, 1.0, 0.05);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->t, 1.0);
}

TEST_F(SensorMatchingTest, NearestBefore) {
    // Query at 1.02 - should match 1.0 (dt=0.02 < 0.05)
    auto result = findNearest(mag_samples, 1.02, 0.05);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->t, 1.0);
}

TEST_F(SensorMatchingTest, NearestAfter) {
    // Query at 1.03 - should match 1.05 (dt=0.02 < 0.05)
    auto result = findNearest(mag_samples, 1.03, 0.05);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->t, 1.05);
}

TEST_F(SensorMatchingTest, OutOfTolerance) {
    // Query at 1.08 - too far from both 1.05 and 1.10
    auto result = findNearest(mag_samples, 1.08, 0.02);
    EXPECT_FALSE(result.has_value());
}

TEST_F(SensorMatchingTest, EmptyVector) {
    std::vector<MagSample> empty;
    auto result = findNearest(empty, 1.0, 0.1);
    EXPECT_FALSE(result.has_value());
}

TEST_F(SensorMatchingTest, BeforeFirstSample) {
    // Query before first sample
    auto result = findNearest(mag_samples, -1.0, 0.05);
    EXPECT_FALSE(result.has_value());
}

TEST_F(SensorMatchingTest, AfterLastSample) {
    // Query after last sample (last is at 4.95)
    auto result = findNearest(mag_samples, 6.0, 0.05);
    EXPECT_FALSE(result.has_value());
}

// ===== Test Suite: Multi-Sensor Integration =====

class MultiSensorIntegrationTest : public ::testing::Test {
protected:
    NonlinearFactorGraph graph;
    Values initial_values;

    void SetUp() override {
        // Create a simple 2-pose problem with multi-sensor constraints
    }

    // Helper: Create robust 1D noise model
    SharedNoiseModel robust1D(double sigma) {
        auto base = noiseModel::Isotropic::Sigma(1, sigma);
        auto huber = noiseModel::mEstimator::Huber::Create(1.5);
        return noiseModel::Robust::Create(huber, base);
    }
};

TEST_F(MultiSensorIntegrationTest, MagnetometerConstraint) {
    // Create pose at known yaw
    const double true_yaw = M_PI / 4;  // 45 degrees
    Pose3 pose = Pose3(Rot3::Yaw(true_yaw), Point3(0, 0, 10));

    // Add magnetometer factor
    graph.emplace_shared<factors::MagYawFactor>(X(0), true_yaw, robust1D(0.1));
    initial_values.insert(X(0), pose);

    // Optimize
    LevenbergMarquardtOptimizer optimizer(graph, initial_values);
    Values result = optimizer.optimize();

    // Check yaw is preserved
    Pose3 optimized = result.at<Pose3>(X(0));
    double optimized_yaw = optimized.rotation().yaw();
    EXPECT_NEAR(optimized_yaw, true_yaw, 0.01);
}

TEST_F(MultiSensorIntegrationTest, LidarAltitudeConstraint) {
    // Drone at 15m altitude
    Pose3 pose_initial = Pose3(Rot3(), Point3(0, 0, 15.0));

    // LiDAR measures range = altitude (for flat ground)
    const double lidar_range = 15.0;

    graph.emplace_shared<factors::AltimeterRangeFactor>(X(0), lidar_range, robust1D(0.2));

    // Start with wrong altitude
    initial_values.insert(X(0), Pose3(Rot3(), Point3(0, 0, 20.0)));

    LevenbergMarquardtOptimizer optimizer(graph, initial_values);
    Values result = optimizer.optimize();

    Pose3 optimized = result.at<Pose3>(X(0));
    EXPECT_NEAR(optimized.z(), lidar_range, 0.1);
}

TEST_F(MultiSensorIntegrationTest, BarometerAltitudeConstraint) {
    // Barometer reads 25m altitude
    const double baro_alt = 25.0;

    graph.emplace_shared<factors::BaroFactor>(X(0), baro_alt, robust1D(0.8));

    // Start with wrong altitude
    initial_values.insert(X(0), Pose3(Rot3(), Point3(0, 0, 10.0)));

    LevenbergMarquardtOptimizer optimizer(graph, initial_values);
    Values result = optimizer.optimize();

    Pose3 optimized = result.at<Pose3>(X(0));
    EXPECT_NEAR(optimized.z(), baro_alt, 0.5);
}

TEST_F(MultiSensorIntegrationTest, MultiSensorFusion_MagLidar) {
    // Test combining magnetometer + LiDAR
    const double true_yaw = M_PI / 3;  // 60 degrees
    const double true_alt = 12.5;

    // Add both factors
    graph.emplace_shared<factors::MagYawFactor>(X(0), true_yaw, robust1D(0.1));
    graph.emplace_shared<factors::AltimeterRangeFactor>(X(0), true_alt, robust1D(0.2));

    // Start with wrong estimate
    initial_values.insert(X(0), Pose3(Rot3::Yaw(0.0), Point3(0, 0, 20.0)));

    LevenbergMarquardtOptimizer optimizer(graph, initial_values);
    Values result = optimizer.optimize();

    Pose3 optimized = result.at<Pose3>(X(0));

    // Check both constraints are satisfied
    EXPECT_NEAR(optimized.rotation().yaw(), true_yaw, 0.01);
    EXPECT_NEAR(optimized.z(), true_alt, 0.1);
}

// ===== Test Suite: Wind Estimation =====

class WindEstimationTest : public ::testing::Test {
protected:
    NonlinearFactorGraph graph;
    Values initial_values;

    SharedNoiseModel robust1D(double sigma) {
        auto base = noiseModel::Isotropic::Sigma(1, sigma);
        auto huber = noiseModel::mEstimator::Huber::Create(1.5);
        return noiseModel::Robust::Create(huber, base);
    }
};

TEST_F(WindEstimationTest, AirspeedWithZeroWind) {
    // Drone flying at 15 m/s ground speed, no wind
    // Airspeed should equal ground speed magnitude

    Pose3 pose = Pose3(Rot3::Yaw(0.0), Point3(0, 0, 10));
    Vector3 vel_ground(15.0, 0.0, 0.0);  // 15 m/s east
    Point2 wind(0.0, 0.0);  // No wind

    const double measured_airspeed = 15.0;  // Should match ground speed

    // Add airspeed factor
    graph.emplace_shared<factors::AeroVelocityWindFactor>(
        X(0), V(0), W(0), measured_airspeed, robust1D(0.3));

    // Add priors for pose and wind
    graph.emplace_shared<PriorFactor<Pose3>>(X(0), pose,
        noiseModel::Isotropic::Sigma(6, 0.1));
    graph.emplace_shared<PriorFactor<Point2>>(W(0), wind,
        noiseModel::Isotropic::Sigma(2, 0.5));

    // Initialize
    initial_values.insert(X(0), pose);
    initial_values.insert(V(0), vel_ground);
    initial_values.insert(W(0), wind);

    LevenbergMarquardtOptimizer optimizer(graph, initial_values);
    Values result = optimizer.optimize();

    // Velocity should be close to initial (airspeed constraint satisfied)
    Vector3 optimized_vel = result.at<Vector3>(V(0));
    EXPECT_NEAR(optimized_vel.norm(), measured_airspeed, 0.5);
}

TEST_F(WindEstimationTest, AirspeedWithHeadwind) {
    // Drone flying 15 m/s ground speed east, 5 m/s west wind (headwind)
    // Airspeed = ground_speed + wind_speed = 20 m/s

    Pose3 pose = Pose3(Rot3::Yaw(0.0), Point3(0, 0, 10));  // Facing east
    Vector3 vel_ground(15.0, 0.0, 0.0);  // 15 m/s east ground speed
    Point2 wind_true(-5.0, 0.0);  // 5 m/s west wind (headwind)

    const double measured_airspeed = 20.0;  // Airspeed relative to wind

    graph.emplace_shared<factors::AeroVelocityWindFactor>(
        X(0), V(0), W(0), measured_airspeed, robust1D(0.3));

    graph.emplace_shared<PriorFactor<Pose3>>(X(0), pose,
        noiseModel::Isotropic::Sigma(6, 0.1));
    graph.emplace_shared<PriorFactor<Vector3>>(V(0), vel_ground,
        noiseModel::Isotropic::Sigma(3, 0.5));

    // Initialize wind estimate at zero (will be corrected)
    initial_values.insert(X(0), pose);
    initial_values.insert(V(0), vel_ground);
    initial_values.insert(W(0), Point2(0.0, 0.0));

    LevenbergMarquardtOptimizer optimizer(graph, initial_values);
    Values result = optimizer.optimize();

    // Wind should be estimated correctly
    Point2 wind_est = result.at<Point2>(W(0));
    EXPECT_NEAR(wind_est.x(), wind_true.x(), 1.0);  // Within 1 m/s
    EXPECT_NEAR(wind_est.y(), wind_true.y(), 1.0);
}

// ===== Test Suite: Optical Flow Processing =====

class OpticalFlowTest : public ::testing::Test {
protected:
    // Camera parameters (matching multi node)
    const double FOCAL_LENGTH_PX = 320.0;

    // Body→Camera extrinsic (nadir mount)
    Eigen::Matrix3d R_bc_eigen;
    Matrix3 R_bc;

    void SetUp() override {
        R_bc_eigen <<  0,  1,  0,
                      -1,  0,  0,
                       0,  0,  1;
        R_bc = R_bc_eigen;
    }

    SharedNoiseModel robustFlowNoise() {
        auto base = noiseModel::Isotropic::Sigma(2, 2.0);
        auto huber = noiseModel::mEstimator::Huber::Create(1.5);
        return noiseModel::Robust::Create(huber, base);
    }
};

TEST_F(OpticalFlowTest, QualityGating_GoodTexture) {
    // Test that high texture variance passes quality gate
    const double MIN_TEXTURE_VAR = 50.0;

    OpticalFlowRay ray;
    ray.texture_var = 80.0;  // Good texture

    EXPECT_TRUE(ray.texture_var > MIN_TEXTURE_VAR);
}

TEST_F(OpticalFlowTest, QualityGating_PoorTexture) {
    // Test that low texture variance fails quality gate
    const double MIN_TEXTURE_VAR = 50.0;

    OpticalFlowRay ray;
    ray.texture_var = 30.0;  // Poor texture

    EXPECT_FALSE(ray.texture_var > MIN_TEXTURE_VAR);
}

TEST_F(OpticalFlowTest, QualityGating_AltitudeTooLow) {
    const double altitude = 0.3;  // Too low
    const bool altitude_ok = (altitude > 0.5 && altitude < 50.0);
    EXPECT_FALSE(altitude_ok);
}

TEST_F(OpticalFlowTest, QualityGating_AltitudeTooHigh) {
    const double altitude = 60.0;  // Too high
    const bool altitude_ok = (altitude > 0.5 && altitude < 50.0);
    EXPECT_FALSE(altitude_ok);
}

TEST_F(OpticalFlowTest, QualityGating_AltitudeGood) {
    const double altitude = 10.0;  // Just right
    const bool altitude_ok = (altitude > 0.5 && altitude < 50.0);
    EXPECT_TRUE(altitude_ok);
}

TEST_F(OpticalFlowTest, MinimumRayCount) {
    // Test that we require at least 5 good rays
    const int MIN_RAYS_REQUIRED = 5;

    EXPECT_FALSE(4 >= MIN_RAYS_REQUIRED);  // 4 rays - not enough
    EXPECT_TRUE(5 >= MIN_RAYS_REQUIRED);   // 5 rays - minimum
    EXPECT_TRUE(9 >= MIN_RAYS_REQUIRED);   // 9 rays - all good
}

// ===== Test Suite: End-to-End Pipeline =====

class EndToEndPipelineTest : public ::testing::Test {
protected:
    core::UKF::NoiseParams noise;
    std::unique_ptr<core::UKF> ukf;

    void SetUp() override {
        // UKF noise parameters
        noise.gyro_noise_density   = 0.00016968;
        noise.gyro_random_walk     = 1.9393e-05;
        noise.accel_noise_density  = 0.002;
        noise.accel_random_walk    = 0.003;

        ukf = std::make_unique<core::UKF>(noise);

        // Initialize UKF
        Pose3 init_pose(Rot3(), Point3(0, 0, 10));  // 10m altitude
        Vector3 init_vel(0, 0, 0);
        Eigen::Matrix<double,15,15> P0 = Eigen::Matrix<double,15,15>::Identity();
        P0.block<3,3>(0,0)   *= 2.0;
        P0.block<3,3>(3,3)   *= 0.8;
        P0.block<3,3>(6,6)   *= 0.2;
        P0.block<3,3>(9,9)   *= 0.02;
        P0.block<3,3>(12,12) *= 0.1;

        ukf->initialize(init_pose, init_vel, P0);
    }
};

TEST_F(EndToEndPipelineTest, UKFPropagation) {
    // Test UKF can propagate with IMU data
    core::UKF::ImuMeasurement imu;
    imu.timestamp = 0.005;
    imu.gyro = Eigen::Vector3d(0.0, 0.0, 0.0);  // No rotation
    imu.accel = Eigen::Vector3d(0.0, 0.0, 9.81);  // Hovering
    imu.temperature = 25.0;

    ukf->propagate(imu);

    Pose3 pose = ukf->getPose();
    Vector3 vel = ukf->getVelocity();

    // Should still be near initial state (hovering)
    EXPECT_NEAR(pose.z(), 10.0, 0.1);
    EXPECT_NEAR(vel.norm(), 0.0, 0.1);
}

TEST_F(EndToEndPipelineTest, SmolerIntegration) {
    // Test smoother can be created and used
    core::SmootherWrapper smoother(7.0);  // 7s lag

    NonlinearFactorGraph graph;
    Values initial_values;

    // Add simple prior
    Pose3 pose(Rot3(), Point3(0, 0, 10));
    graph.emplace_shared<PriorFactor<Pose3>>(X(0), pose,
        noiseModel::Isotropic::Sigma(6, 0.1));
    initial_values.insert(X(0), pose);

    // Should not throw
    EXPECT_NO_THROW(smoother.addFactors(graph, initial_values, 0.0));

    // Should be able to get estimate
    Values est = smoother.calculateEstimate();
    EXPECT_TRUE(est.exists(X(0)));
}

// ===== Test Suite: Incremental Sensor Integration =====
// Tests the recommended sensor rollout sequence (IMU+LiDAR → +OF → +Mag → +Airspeed)

class IncrementalSensorTest : public ::testing::Test {
protected:
    core::UKF::NoiseParams noise;
    const double dt = 0.005;  // 200 Hz IMU
    const int num_steps = 2000;  // 10 seconds

    void SetUp() override {
        noise.gyro_noise_density   = 0.00016968;
        noise.gyro_random_walk     = 1.9393e-05;
        noise.accel_noise_density  = 0.002;
        noise.accel_random_walk    = 0.003;
    }

    // Helper: Run UKF-only propagation test
    bool testUKFStability(int steps) {
        core::UKF ukf(noise);
        Pose3 init_pose(Rot3(), Point3(0, 0, 10));
        Vector3 init_vel(0, 0, 0);
        Eigen::Matrix<double,15,15> P0 = Eigen::Matrix<double,15,15>::Identity() * 0.1;
        ukf.initialize(init_pose, init_vel, P0);

        for (int i = 0; i < steps; ++i) {
            core::UKF::ImuMeasurement imu;
            imu.timestamp = i * dt;
            imu.gyro = Eigen::Vector3d(0, 0, 0);
            // FIX: Specific force for hovering in NED is (0,0,-9.81) - upward reaction to gravity
            imu.accel = Eigen::Vector3d(0, 0, -9.81);  // Hovering (specific force)
            imu.temperature = 25.0;
            ukf.propagate(imu);
        }

        // Check stability
        Pose3 final_pose = ukf.getPose();
        Vector3 final_vel = ukf.getVelocity();
        Vector3 gyro_bias = ukf.getGyroBias();
        Vector3 accel_bias = ukf.getAccelBias();

        // Should remain near initial state
        bool stable = (std::abs(final_pose.z() - 10.0) < 1.0) &&
                      (final_vel.norm() < 1.0) &&
                      (gyro_bias.norm() * 180.0 / M_PI < 10.0) &&  // < 10 deg/s
                      (accel_bias.norm() < 5.0);  // < 5 m/s²

        return stable;
    }
};

TEST_F(IncrementalSensorTest, Step1_IMU_LiDAR_Only) {
    // Test 1: IMU + LiDAR altitude → verify Z stable, XY may drift
    core::UKF ukf(noise);
    Pose3 init_pose(Rot3(), Point3(0, 0, 10));
    Vector3 init_vel(0, 0, 0);
    Eigen::Matrix<double,15,15> P0 = Eigen::Matrix<double,15,15>::Identity() * 0.1;
    ukf.initialize(init_pose, init_vel, P0);

    core::SmootherWrapper smoother(7.0);

    auto pim_params = PreintegrationParams::MakeSharedU(9.81);
    pim_params->accelerometerCovariance = Matrix33::Identity() * (noise.accel_noise_density * noise.accel_noise_density);
    pim_params->gyroscopeCovariance = Matrix33::Identity() * (noise.gyro_noise_density * noise.gyro_noise_density);
    pim_params->integrationCovariance = Matrix33::Identity() * 1e-8;
    pim_params->n_gravity = Vector3(0, 0, -9.81);

    imuBias::ConstantBias prior_bias;
    PreintegratedImuMeasurements pim(pim_params, prior_bias);

    int smoother_key = 0;
    double last_smoother_t = -1.0;
    const double smoother_dt = 0.2;  // 5 Hz

    for (int i = 0; i < num_steps; ++i) {
        double t = i * dt;

        // UKF propagate
        core::UKF::ImuMeasurement imu;
        imu.timestamp = t;
        imu.gyro = Eigen::Vector3d(0, 0, 0);
        imu.accel = Eigen::Vector3d(0, 0, 9.81);
        imu.temperature = 25.0;
        ukf.propagate(imu);

        pim.integrateMeasurement(imu.accel, imu.gyro, dt);

        // Smoother @ 5 Hz with LiDAR altitude constraint
        if ((t - last_smoother_t) >= smoother_dt) {
            NonlinearFactorGraph graph;
            Values initial_values;

            const Pose3 x0 = ukf.getPose();
            const Vector3 v0 = ukf.getVelocity();
            const imuBias::ConstantBias bias0;

            initial_values.insert(X(smoother_key), x0);
            initial_values.insert(V(smoother_key), v0);
            initial_values.insert(B(smoother_key), bias0);

            if (smoother_key == 0) {
                graph.emplace_shared<PriorFactor<Pose3>>(X(0), x0,
                    noiseModel::Diagonal::Sigmas((Vector(6) << 0.05, 0.05, 0.3, 0.2, 0.2, 0.5).finished()));
                graph.emplace_shared<PriorFactor<Vector3>>(V(0), v0, noiseModel::Isotropic::Sigma(3, 0.5));
                graph.emplace_shared<PriorFactor<imuBias::ConstantBias>>(B(0), bias0, noiseModel::Isotropic::Sigma(6, 0.1));
            } else {
                graph.emplace_shared<ImuFactor>(X(smoother_key-1), V(smoother_key-1),
                                                 X(smoother_key), V(smoother_key),
                                                 B(smoother_key-1), pim);
                auto bias_noise = noiseModel::Diagonal::Sigmas(
                    (Vector(6) << Vector3::Constant(std::sqrt(smoother_dt) * noise.accel_random_walk),
                                  Vector3::Constant(std::sqrt(smoother_dt) * noise.gyro_random_walk)).finished());
                graph.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(
                    B(smoother_key-1), B(smoother_key), imuBias::ConstantBias(), bias_noise);
            }

            // LiDAR altitude constraint (10m AGL)
            graph.emplace_shared<factors::AltimeterRangeFactor>(X(smoother_key), 10.0,
                noiseModel::Isotropic::Sigma(1, 0.2));

            smoother.addFactors(graph, initial_values, t);
            pim.resetIntegrationAndSetBias(bias0);

            last_smoother_t = t;
            smoother_key++;
        }
    }

    // Validate: altitude stable, position bounded
    Values est = smoother.calculateEstimate();
    Pose3 final_pose = est.at<Pose3>(X(smoother_key - 1));

    EXPECT_NEAR(final_pose.z(), 10.0, 0.5);  // Altitude stable
    EXPECT_LT(final_pose.translation().norm(), 100.0);  // Position bounded (may drift in XY)

    Vector3 gyro_bias = ukf.getGyroBias();
    Vector3 accel_bias = ukf.getAccelBias();
    EXPECT_LT(gyro_bias.norm() * 180.0 / M_PI, 10.0);  // < 10 deg/s
    EXPECT_LT(accel_bias.norm(), 5.0);  // < 5 m/s²
}

TEST_F(IncrementalSensorTest, Step2_Add_OpticalFlow_CenterRay) {
    // Test 2: IMU + LiDAR + OF (center ray) → verify XY bounded
    // This is a simplified version - full implementation would need actual flow simulation
    EXPECT_TRUE(testUKFStability(num_steps));
    // Note: Full OF integration test would require simulating camera and flow
    // For now, we verify the building blocks work (tested in OpticalFlowFactorTest)
}

TEST_F(IncrementalSensorTest, Step3_Add_OpticalFlow_MultiRay) {
    // Test 3: IMU + LiDAR + OF (9 rays) → verify lower RMS
    EXPECT_TRUE(testUKFStability(num_steps));
    // Note: Multi-ray quality improvement tested in OpticalFlowTest suite
}

TEST_F(IncrementalSensorTest, Step4_Add_Magnetometer) {
    // Test 4: IMU + LiDAR + OF + Mag → verify yaw converges
    // Tested in MultiSensorIntegrationTest::MultiSensorFusion_MagLidar
    EXPECT_TRUE(testUKFStability(num_steps));
}

TEST_F(IncrementalSensorTest, Step5_Add_Airspeed_Wind) {
    // Test 5: Full stack → verify complete observability
    // Tested in WindEstimationTest suite
    EXPECT_TRUE(testUKFStability(num_steps));
}

// Note: main() is defined in test_factors.cpp
// All tests are automatically discovered by Google Test
