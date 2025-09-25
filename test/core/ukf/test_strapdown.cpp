/**
 * Unit Tests for Strapdown Mechanization
 * Focus on coning and sculling compensation
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "core/ukf/strapdown.h"

using namespace Navigation;
using namespace Eigen;

class StrapdownTest : public ::testing::Test {
protected:
    std::unique_ptr<Navigation::StrapdownMechanization> mech;

    void SetUp() override {
        mech = std::make_unique<Navigation::StrapdownMechanization>();
        mech->reset();
    }
};

// Test basic compensation with zero input
TEST_F(StrapdownTest, ZeroInput) {
    Vector3d accel = Vector3d::Zero();
    Vector3d gyro = Vector3d::Zero();
    double dt = 0.01;

    auto result = mech->compensate(accel, gyro, dt);

    EXPECT_LT(result.delta_v.norm(), 1e-10)
        << "Non-zero velocity increment with zero input";
    EXPECT_LT(result.delta_theta.norm(), 1e-10)
        << "Non-zero angle increment with zero input";
    EXPECT_EQ(result.dt, dt)
        << "Time increment not preserved";
}

// Test constant acceleration
TEST_F(StrapdownTest, ConstantAcceleration) {
    Vector3d accel(1, 0, 0);  // 1 m/s^2 in X
    Vector3d gyro = Vector3d::Zero();
    double dt = 0.01;

    // First sample (no compensation yet)
    auto result1 = mech->compensate(accel, gyro, dt);

    // Second sample (compensation active)
    auto result2 = mech->compensate(accel, gyro, dt);

    // Delta velocity should be acceleration * dt
    EXPECT_NEAR(result2.delta_v.x(), accel.x() * dt, 1e-6)
        << "Incorrect velocity increment";
    EXPECT_NEAR(result2.delta_v.y(), 0, 1e-10)
        << "Spurious Y velocity";
    EXPECT_NEAR(result2.delta_v.z(), 0, 1e-10)
        << "Spurious Z velocity";
}

// Test constant rotation
TEST_F(StrapdownTest, ConstantRotation) {
    Vector3d accel = Vector3d::Zero();
    Vector3d gyro(0, 0, 0.1);  // 0.1 rad/s yaw rate
    double dt = 0.01;

    // First sample
    auto result1 = mech->compensate(accel, gyro, dt);

    // Second sample
    auto result2 = mech->compensate(accel, gyro, dt);

    // Delta angle should be gyro * dt
    EXPECT_NEAR(result2.delta_theta.z(), gyro.z() * dt, 1e-6)
        << "Incorrect angle increment";
    EXPECT_NEAR(result2.delta_theta.x(), 0, 1e-10)
        << "Spurious roll";
    EXPECT_NEAR(result2.delta_theta.y(), 0, 1e-10)
        << "Spurious pitch";
}

// Test coning motion compensation
TEST_F(StrapdownTest, ConingCompensation) {
    double freq = 10.0;  // Hz
    double amplitude = 0.1;  // radians
    double dt = 0.001;   // 1 kHz sampling

    // Initialize with first sample
    Vector3d gyro1(amplitude * sin(0), amplitude * cos(0), 0);
    mech->compensate(Vector3d::Zero(), gyro1, dt);

    // Accumulate angle over one cycle
    Vector3d total_angle = Vector3d::Zero();
    for (int i = 1; i < 100; ++i) {
        double t = i * dt;
        Vector3d gyro(
            amplitude * sin(2 * M_PI * freq * t),
            amplitude * cos(2 * M_PI * freq * t),
            0
        );

        auto result = mech->compensate(Vector3d::Zero(), gyro, dt);
        total_angle += result.delta_theta;
    }

    // With coning compensation, accumulated error should be small
    EXPECT_LT(total_angle.norm(), amplitude * 0.1)
        << "Coning compensation not effective";
}

// Test sculling motion compensation
TEST_F(StrapdownTest, ScullingCompensation) {
    double freq = 10.0;  // Hz
    double accel_amp = 1.0;  // m/s^2
    double gyro_amp = 0.1;   // rad/s
    double dt = 0.001;

    // Initialize
    Vector3d accel1(accel_amp * sin(0), 0, 0);
    Vector3d gyro1(0, gyro_amp * cos(0), 0);
    mech->compensate(accel1, gyro1, dt);

    // Accumulate velocity over one cycle
    Vector3d total_velocity = Vector3d::Zero();
    for (int i = 1; i < 100; ++i) {
        double t = i * dt;
        Vector3d accel(accel_amp * sin(2 * M_PI * freq * t), 0, 0);
        Vector3d gyro(0, gyro_amp * cos(2 * M_PI * freq * t), 0);

        auto result = mech->compensate(accel, gyro, dt);
        total_velocity += result.delta_v;
    }

    // With sculling compensation, velocity error should be small
    EXPECT_LT(total_velocity.norm(), accel_amp * dt * 10)
        << "Sculling compensation not effective";
}

// Test reset functionality
TEST_F(StrapdownTest, ResetFunctionality) {
    // Process some data
    Vector3d accel(5, 3, 9.81);
    Vector3d gyro(0.1, 0.2, 0.3);

    for (int i = 0; i < 10; ++i) {
        mech->compensate(accel, gyro, 0.01);
    }

    // Reset
    mech->reset();

    // First sample after reset should not have compensation
    auto result = mech->compensate(accel, gyro, 0.01);

    // Should be simple integration (no previous sample for compensation)
    EXPECT_NEAR(result.delta_v.norm(), accel.norm() * 0.01, 1e-4)
        << "Reset didn't clear previous samples";
}

// Test numerical stability with small dt
TEST_F(StrapdownTest, SmallTimestepStability) {
    Vector3d accel(1, 2, 9.81);
    Vector3d gyro(0.01, 0.02, 0.03);
    double dt = 1e-6;  // 1 microsecond

    // Process many small steps
    bool stable = true;
    for (int i = 0; i < 1000; ++i) {
        auto result = mech->compensate(accel, gyro, dt);

        if (!std::isfinite(result.delta_v.norm()) ||
            !std::isfinite(result.delta_theta.norm())) {
            stable = false;
            FAIL() << "Compensation became NaN/Inf at step " << i;
        }

        // Check reasonable bounds
        EXPECT_LT(result.delta_v.norm(), 1.0)
            << "Velocity increment too large for microsecond timestep";
        EXPECT_LT(result.delta_theta.norm(), 1.0)
            << "Angle increment too large for microsecond timestep";
    }

    EXPECT_TRUE(stable) << "Numerical instability with small timesteps";
}

// Test large rotation rates
TEST_F(StrapdownTest, LargeRotationRates) {
    Vector3d accel(0, 0, 9.81);
    Vector3d gyro(5, 5, 5);  // Large rotation rates (5 rad/s each axis)
    double dt = 0.01;

    // Initialize
    mech->compensate(accel, gyro, dt);

    // Process second sample
    auto result = mech->compensate(accel, gyro, dt);

    // Should still produce finite results
    EXPECT_TRUE(std::isfinite(result.delta_v.norm()))
        << "Velocity increment NaN with large rotation";
    EXPECT_TRUE(std::isfinite(result.delta_theta.norm()))
        << "Angle increment NaN with large rotation";

    // Check bounds
    EXPECT_LT(result.delta_theta.norm(), 0.1)
        << "Angle increment too large";
}

// Test alternating motion (worst case for compensation)
TEST_F(StrapdownTest, AlternatingMotion) {
    double dt = 0.01;
    Vector3d accel_pos(10, 0, 9.81);
    Vector3d accel_neg(-10, 0, 9.81);
    Vector3d gyro_pos(0, 0, 1);
    Vector3d gyro_neg(0, 0, -1);

    // Initialize
    mech->compensate(accel_pos, gyro_pos, dt);

    // Alternate for several cycles
    Vector3d accumulated_v = Vector3d::Zero();
    Vector3d accumulated_theta = Vector3d::Zero();

    for (int i = 0; i < 100; ++i) {
        auto result = (i % 2 == 0) ?
            mech->compensate(accel_neg, gyro_neg, dt) :
            mech->compensate(accel_pos, gyro_pos, dt);

        accumulated_v += result.delta_v;
        accumulated_theta += result.delta_theta;
    }

    // Should not accumulate significant error
    EXPECT_LT(accumulated_v.head<2>().norm(), 1.0)
        << "Velocity accumulated in alternating motion";
    EXPECT_LT(accumulated_theta.z(), 1.0)
        << "Angle accumulated in alternating motion";
}