/**
 * Unit Tests for StateVector conversions and operations
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "core/ukf/sr_ukf.h"

using namespace Navigation;
using namespace Eigen;

class StateVectorTest : public ::testing::Test {
protected:
    StateVector state;

    void SetUp() override {
        // Initialize a test state
        state.position = Vector3d(1000, 2000, -3000);
        state.velocity = Vector3d(50, -30, 2);
        state.quaternion = Quaterniond(0.9239, 0.3827, 0, 0);  // 45 deg yaw
        state.accel_bias = Vector3d(0.01, -0.02, 0.03);
        state.gyro_bias = Vector3d(0.001, -0.002, 0.003);
        state.gravity_bias << 1.0, 2.0, 3.0, 4.0, 5.0;
    }
};

// Test vector conversion to/from StateVector
TEST_F(StateVectorTest, VectorConversion) {
    // Convert to vector
    VectorXd vec = state.toVector();

    ASSERT_EQ(vec.size(), 21) << "State vector should be 21-dimensional";

    // Check position
    EXPECT_EQ(vec(0), state.position.x());
    EXPECT_EQ(vec(1), state.position.y());
    EXPECT_EQ(vec(2), state.position.z());

    // Check velocity
    EXPECT_EQ(vec(3), state.velocity.x());
    EXPECT_EQ(vec(4), state.velocity.y());
    EXPECT_EQ(vec(5), state.velocity.z());

    // Check quaternion
    EXPECT_EQ(vec(6), state.quaternion.w());
    EXPECT_EQ(vec(7), state.quaternion.x());
    EXPECT_EQ(vec(8), state.quaternion.y());
    EXPECT_EQ(vec(9), state.quaternion.z());

    // Check biases
    EXPECT_EQ(vec(10), state.accel_bias.x());
    EXPECT_EQ(vec(13), state.gyro_bias.x());
    EXPECT_EQ(vec(16), state.gravity_bias(0));

    // Convert back
    StateVector recovered;
    recovered.fromVector(vec);

    // Check recovery
    EXPECT_LT((recovered.position - state.position).norm(), 1e-10);
    EXPECT_LT((recovered.velocity - state.velocity).norm(), 1e-10);
    EXPECT_LT((recovered.accel_bias - state.accel_bias).norm(), 1e-10);
    EXPECT_LT((recovered.gyro_bias - state.gyro_bias).norm(), 1e-10);
    EXPECT_LT((recovered.gravity_bias - state.gravity_bias).norm(), 1e-10);

    // Check quaternion (note: might be negative of original due to double cover)
    double quat_diff = std::min(
        (recovered.quaternion.coeffs() - state.quaternion.coeffs()).norm(),
        (recovered.quaternion.coeffs() + state.quaternion.coeffs()).norm()
    );
    EXPECT_LT(quat_diff, 1e-4) << "Quaternion not properly recovered";
}

// Test quaternion normalization
TEST_F(StateVectorTest, QuaternionNormalization) {
    // Create unnormalized quaternion
    state.quaternion = Quaterniond(2.0, 0, 0, 0);  // Not normalized

    VectorXd vec = state.toVector();

    StateVector recovered;
    recovered.fromVector(vec);

    // Check quaternion is normalized after conversion
    EXPECT_NEAR(recovered.quaternion.norm(), 1.0, 1e-10)
        << "Quaternion should be normalized";
}

// Test error state dimensions
TEST_F(StateVectorTest, ErrorStateDimensions) {
    EXPECT_EQ(StateVector::ERROR_DIM, 20)
        << "Error state should be 20-dimensional (3 less than full state)";
    EXPECT_EQ(StateVector::FULL_DIM, 21)
        << "Full state should be 21-dimensional";
}

// Test extreme values don't cause overflow
TEST_F(StateVectorTest, ExtremeValues) {
    state.position = Vector3d(1e6, -1e6, -30000);  // Large but reasonable
    state.velocity = Vector3d(500, -500, 100);     // Fast aircraft

    VectorXd vec = state.toVector();

    // Check all values are finite
    EXPECT_TRUE(vec.allFinite()) << "Vector contains NaN/Inf with extreme values";

    StateVector recovered;
    recovered.fromVector(vec);

    EXPECT_TRUE(recovered.position.allFinite()) << "Recovered position not finite";
    EXPECT_TRUE(recovered.velocity.allFinite()) << "Recovered velocity not finite";
}

// Test zero state
TEST_F(StateVectorTest, ZeroState) {
    StateVector zero_state;
    zero_state.position = Vector3d::Zero();
    zero_state.velocity = Vector3d::Zero();
    zero_state.quaternion = Quaterniond::Identity();
    zero_state.accel_bias = Vector3d::Zero();
    zero_state.gyro_bias = Vector3d::Zero();
    zero_state.gravity_bias = Eigen::Matrix<double, 5, 1>::Zero();

    VectorXd vec = zero_state.toVector();

    // Position, velocity, biases should be zero
    EXPECT_EQ(vec.segment(0, 3).norm(), 0.0) << "Position should be zero";
    EXPECT_EQ(vec.segment(3, 3).norm(), 0.0) << "Velocity should be zero";
    EXPECT_EQ(vec.segment(10, 3).norm(), 0.0) << "Accel bias should be zero";
    EXPECT_EQ(vec.segment(13, 3).norm(), 0.0) << "Gyro bias should be zero";
    EXPECT_EQ(vec.segment(16, 5).norm(), 0.0) << "Gravity bias should be zero";

    // Quaternion should be identity [1, 0, 0, 0]
    EXPECT_EQ(vec(6), 1.0) << "Quaternion w should be 1";
    EXPECT_EQ(vec.segment(7, 3).norm(), 0.0) << "Quaternion xyz should be zero";
}