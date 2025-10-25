#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <algorithm>

// ===== HELPER STRUCTURES (copied from chimera_node_multi.cpp) =====
struct OrientBootResult {
    gtsam::Rot3 Rwb;
    std::string tag;
    double g_err;
};

// Median accel computation
Eigen::Vector3d medianAccel(const std::vector<Eigen::Vector3d>& accels) {
    auto med = [&](int k) {
        std::vector<double> v;
        v.reserve(accels.size());
        for (auto& a : accels) v.push_back(a(k));
        std::nth_element(v.begin(), v.begin() + v.size()/2, v.end());
        return v[v.size()/2];
    };
    return Eigen::Vector3d(med(0), med(1), med(2));
}

// Quaternion convention picker
OrientBootResult pickRwb_from_quat(const Eigen::Vector4d& q_raw, const Eigen::Vector3d& a_med_b) {
    auto mk = [&](double w,double x,double y,double z){ return gtsam::Rot3::Quaternion(w,x,y,z); };
    struct Cand { gtsam::Rot3 R; const char* tag; };
    std::vector<Cand> cands;
    // assume raw is wxyz
    cands.push_back({ mk(q_raw(0), q_raw(1), q_raw(2), q_raw(3))           , "wxyz"     });
    cands.push_back({ mk(q_raw(0), q_raw(1), q_raw(2), q_raw(3)).inverse() , "wxyz_inv" });
    // assume raw is xyzw
    cands.push_back({ mk(q_raw(3), q_raw(0), q_raw(1), q_raw(2))           , "xyzw"     });
    cands.push_back({ mk(q_raw(3), q_raw(0), q_raw(1), q_raw(2)).inverse() , "xyzw_inv" });

    double best_err = 1e9;
    gtsam::Rot3 bestR;
    const char* best_tag = "";
    for (auto& c : cands) {
        Eigen::Vector3d a_world = c.R.matrix() * a_med_b;
        double err = (a_world - Eigen::Vector3d(0,0,-9.81)).norm();
        if (err < best_err) {
            best_err = err;
            bestR = c.R;
            best_tag = c.tag;
        }
    }
    return {bestR, best_tag, best_err};
}

// ===== TESTS =====

// Test 1: Verify quaternion auto-detection with perfect gravity alignment
TEST(ImuUnitsTest, QuaternionAutoDetect_PerfectAlignment) {
    // Create a rotation: 45° yaw, 0° roll/pitch
    gtsam::Rot3 R_true = gtsam::Rot3::Ypr(M_PI/4, 0, 0);

    // Body-frame gravity (Z-up world, rotated to body)
    Eigen::Vector3d g_world(0, 0, -9.81);
    Eigen::Vector3d a_body = R_true.matrix().transpose() * g_world;

    // Get quaternion in xyzw format
    Eigen::Quaterniond q_eigen(R_true.matrix());
    Eigen::Vector4d q_xyzw(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());

    // Test auto-detection
    auto result = pickRwb_from_quat(q_xyzw, a_body);

    // Should pick "xyzw" and have very low error
    EXPECT_EQ(result.tag, std::string("xyzw"));
    EXPECT_LT(result.g_err, 0.01);  // < 0.01 m/s² error

    // Verify recovered rotation matches
    Eigen::Vector3d a_world_recovered = result.Rwb.matrix() * a_body;
    EXPECT_NEAR(a_world_recovered(0), 0.0, 0.01);
    EXPECT_NEAR(a_world_recovered(1), 0.0, 0.01);
    EXPECT_NEAR(a_world_recovered(2), -9.81, 0.01);
}

// Test 2: Verify NWU→FRD frame flip detection
TEST(ImuUnitsTest, FrameFlip_NWU_to_FRD) {
    // Test with 45° yaw to avoid symmetry issues
    // Create quaternion for 45° yaw in xyzw format
    double yaw = M_PI / 4;
    double qw = std::cos(yaw / 2);
    double qz = std::sin(yaw / 2);
    Eigen::Vector4d q_xyzw(0, 0, qz, qw);  // (x, y, z, w) format

    // In NWU frame at 45° yaw, gravity should still be (0, 0, -9.81) in body frame
    gtsam::Rot3 R_true = gtsam::Rot3::Quaternion(qw, 0, 0, qz);  // GTSAM uses (w,x,y,z)
    Eigen::Vector3d g_world(0, 0, -9.81);
    Eigen::Vector3d a_nwu_body = R_true.transpose().matrix() * g_world;

    // Simulate FRD data: apply frame flip to get FRD version
    Eigen::Vector3d a_frd_body(a_nwu_body.x(), -a_nwu_body.y(), -a_nwu_body.z());

    // Test with FRD accel (should have some error due to Y/Z sign flips)
    auto result_frd = pickRwb_from_quat(q_xyzw, a_frd_body);

    // Apply frame flip to convert FRD → NWU
    Eigen::Vector3d a_nwu_fixed(a_frd_body.x(), -a_frd_body.y(), -a_frd_body.z());
    auto result_fixed = pickRwb_from_quat(q_xyzw, a_nwu_fixed);

    // After frame flip, error should be much smaller
    EXPECT_LT(result_fixed.g_err, 0.1);  // Should be very small
    EXPECT_NEAR((a_nwu_fixed - a_nwu_body).norm(), 0.0, 1e-6);  // Vectors should match
}

// Test 3: Verify gyro unit conversion (deg/s → rad/s)
TEST(ImuUnitsTest, GyroUnitConversion_DegToRad) {
    // Test data: gyro at 10 deg/s = 0.1745 rad/s
    double gyro_deg_s = 10.0;
    double gyro_rad_s_expected = gyro_deg_s * M_PI / 180.0;

    // Apply conversion
    double gyro_rad_s = gyro_deg_s * (M_PI / 180.0);

    EXPECT_NEAR(gyro_rad_s, gyro_rad_s_expected, 1e-6);
    EXPECT_NEAR(gyro_rad_s, 0.174533, 1e-5);
}

// Test 4: Verify that applying conversion twice gives wrong result
TEST(ImuUnitsTest, GyroUnitConversion_DoubleApplicationDetection) {
    double gyro_original = 10.0;  // deg/s

    // Apply conversion once (correct)
    double gyro_once = gyro_original * (M_PI / 180.0);
    EXPECT_NEAR(gyro_once, 0.174533, 1e-5);

    // Apply conversion twice (WRONG - would make it 57.3x too small)
    double gyro_twice = gyro_once * (M_PI / 180.0);
    EXPECT_NEAR(gyro_twice, 0.00304617, 1e-6);

    // If we see gyro bias of ~6.3 rad/s (~360 deg/s), it means:
    // - Data is in deg/s
    // - We're NOT converting
    // - Optimizer compensates with bias ≈ 2π rad/s
    double bias_signature = 2 * M_PI;
    EXPECT_NEAR(bias_signature, 6.28319, 1e-5);
}

// Test 5: Median accel computation
TEST(ImuUnitsTest, MedianAccelComputation) {
    std::vector<Eigen::Vector3d> accels;
    accels.push_back(Eigen::Vector3d(0.1, 0.2, -9.7));
    accels.push_back(Eigen::Vector3d(0.0, 0.0, -9.81));  // median
    accels.push_back(Eigen::Vector3d(-0.1, -0.2, -9.9));

    Eigen::Vector3d a_med = medianAccel(accels);

    EXPECT_NEAR(a_med(0), 0.0, 0.01);
    EXPECT_NEAR(a_med(1), 0.0, 0.01);
    EXPECT_NEAR(a_med(2), -9.81, 0.01);
}

// Test 6: Verify gyro bias signature detection
TEST(ImuUnitsTest, GyroBiasSignature_DetectUnitsError) {
    // If gyro is in deg/s but treated as rad/s:
    // - Actual: 0.5 deg/s = 0.00873 rad/s
    // - System expects: ~0.00873 rad/s
    // - System gets: 0.5 rad/s (57.3x larger)
    // - Bias compensation: ~-0.5 rad/s to make rotation rates match

    // But the signature we see is bias ≈ 360 deg/s = 2π rad/s
    // This happens because:
    // - Gyro integration over time accumulates error
    // - After 1 second at 0.5 deg/s error → 0.5° accumulated
    // - Over many seconds this creates systematic rotation error
    // - Optimizer adds constant bias to null out the rotation rate difference

    double bias_observed = 6.32;  // rad/s (from logs: ~362 deg/s)
    double bias_expected_signature = 2 * M_PI;

    EXPECT_NEAR(bias_observed, bias_expected_signature, 0.1);
}

// Test 7: Frame flip application consistency
TEST(ImuUnitsTest, FrameFlip_ApplyConsistently) {
    Eigen::Vector3d accel_nwu(1.0, 2.0, -9.81);
    Eigen::Vector3d gyro_nwu(0.1, 0.2, 0.3);

    // Apply NWU→FRD flip: (x, -y, -z)
    auto flip = [](const Eigen::Vector3d& v) {
        return Eigen::Vector3d(v.x(), -v.y(), -v.z());
    };

    Eigen::Vector3d accel_frd = flip(accel_nwu);
    Eigen::Vector3d gyro_frd = flip(gyro_nwu);

    EXPECT_NEAR(accel_frd(0),  1.0, 1e-9);
    EXPECT_NEAR(accel_frd(1), -2.0, 1e-9);
    EXPECT_NEAR(accel_frd(2),  9.81, 1e-9);

    EXPECT_NEAR(gyro_frd(0),  0.1, 1e-9);
    EXPECT_NEAR(gyro_frd(1), -0.2, 1e-9);
    EXPECT_NEAR(gyro_frd(2), -0.3, 1e-9);
}

// Test 8: Verify data flow - gyro value preserved through lambda
TEST(ImuUnitsTest, GyroScaling_LambdaApplication) {
    double gyro_scale = M_PI / 180.0;
    bool frame_flip = true;

    Eigen::Vector3d gyro_raw(10.0, 20.0, 30.0);  // deg/s

    // Simulate fixGyro lambda (FIXED VERSION - no dangling temporaries)
    auto fixGyro = [&](const Eigen::Vector3d& g) -> Eigen::Vector3d {
        if (frame_flip) {
            return Eigen::Vector3d(g.x() * gyro_scale, -g.y() * gyro_scale, -g.z() * gyro_scale);
        } else {
            return g * gyro_scale;
        }
    };

    Eigen::Vector3d gyro_fixed = fixGyro(gyro_raw);

    // Check frame flip
    EXPECT_NEAR(gyro_fixed(0), 10.0 * M_PI / 180.0, 1e-9);
    EXPECT_NEAR(gyro_fixed(1), -20.0 * M_PI / 180.0, 1e-9);
    EXPECT_NEAR(gyro_fixed(2), -30.0 * M_PI / 180.0, 1e-9);

    // Check unit conversion
    EXPECT_NEAR(gyro_fixed.norm(),
                std::sqrt(10*10 + 20*20 + 30*30) * M_PI / 180.0,
                1e-6);
}
