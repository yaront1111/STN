#include <gtest/gtest.h>
#include "utils/sensor_helpers.h"
#include <Eigen/Dense>
#include <gtsam/geometry/Rot3.h>
#include <cmath>

using namespace chimera::utils;

/**
 * @brief Test orientation picker with NED-fixed gravity
 *
 * Verifies that pickRwb_from_quat() correctly identifies quaternion convention
 * and aligns acceleration with NED gravity (Down = +Z, g = +9.81).
 */
TEST(OrientationBootTest, PickRwbFromQuatNED) {
  // Simulate IMU at rest in NED: a_body should measure specific force -g*Down = [0, 0, -9.81]
  // (body frame measures specific force, which is -gravity in world frame)
  const Eigen::Vector3d a_body(0.0, 0.0, -9.81);  // FRD body at rest measures upward specific force

  // Test quaternion in xyzw order: identity rotation (body = world)
  const Eigen::Vector4d q_xyzw(0.0, 0.0, 0.0, 1.0);  // [x, y, z, w] = identity

  auto result = pickRwb_from_quat(q_xyzw, a_body);

  // Expected: Should pick xyzw (or xyzw_inv) and report NED alignment
  EXPECT_LT(result.g_err, 0.5) << "Gravity residual should be < 0.5 m/s^2 for valid NED orientation";

  // Verify rotation aligns gravity correctly: R*a_body + g_NED ≈ 0
  const Eigen::Vector3d g_NED(0.0, 0.0, 9.81);
  const Eigen::Vector3d residual = result.Rwb.matrix() * a_body + g_NED;
  EXPECT_LT(residual.norm(), 0.5) << "Residual should be small for correct orientation";

  std::cout << "[TEST] NED picker: tag=" << result.tag
            << ", g_err=" << result.g_err << " m/s^2\n";
}

/**
 * @brief Test frame-agnostic orientation picker
 *
 * Verifies that pickRwb_from_quat_agnostic() works for both NED and ENU conventions
 * by choosing the best candidate regardless of world frame.
 */
TEST(OrientationBootTest, PickRwbFromQuatAgnostic) {
  // Test 1: NED convention (Down = +Z)
  {
    const Eigen::Vector3d a_body_ned(0.0, 0.0, -9.81);  // Body at rest measures upward
    const Eigen::Vector4d q_xyzw(0.0, 0.0, 0.0, 1.0);

    auto result = pickRwb_from_quat_agnostic(q_xyzw, a_body_ned, 9.81);

    EXPECT_LT(result.g_err, 0.5) << "Agnostic picker should handle NED with low error";
    EXPECT_TRUE(result.tag.find("/NED") != std::string::npos ||
                result.tag.find("/ENU") != std::string::npos)
        << "Tag should indicate detected frame";

    std::cout << "[TEST] Agnostic NED: tag=" << result.tag
              << ", g_err=" << result.g_err << " m/s^2\n";
  }

  // Test 2: ENU convention (Up = +Z)
  {
    const Eigen::Vector3d a_body_enu(0.0, 0.0, 9.81);  // Body at rest measures downward in ENU
    const Eigen::Vector4d q_xyzw(0.0, 0.0, 0.0, 1.0);

    auto result = pickRwb_from_quat_agnostic(q_xyzw, a_body_enu, 9.81);

    EXPECT_LT(result.g_err, 0.5) << "Agnostic picker should handle ENU with low error";
    EXPECT_TRUE(result.tag.find("/NED") != std::string::npos ||
                result.tag.find("/ENU") != std::string::npos)
        << "Tag should indicate detected frame";

    std::cout << "[TEST] Agnostic ENU: tag=" << result.tag
              << ", g_err=" << result.g_err << " m/s^2\n";
  }

  // Test 3: Verify agnostic picker chooses correct frame
  {
    const Eigen::Vector3d a_body_ned(0.0, 0.0, -9.81);
    const Eigen::Vector4d q_xyzw(0.0, 0.0, 0.0, 1.0);

    auto result = pickRwb_from_quat_agnostic(q_xyzw, a_body_ned, 9.81);

    // For NED body accel, should pick NED frame
    EXPECT_TRUE(result.tag.find("/NED") != std::string::npos)
        << "Should detect NED frame for NED-convention acceleration";
  }
}
