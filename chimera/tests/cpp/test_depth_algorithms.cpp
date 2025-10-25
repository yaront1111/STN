/**
 * Unit tests for depth/altitude algorithms from chimera_node_multi.cpp
 * Tests: RangeStuckGuard, effectiveDepthMeters_AGL()
 */

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <Eigen/Dense>
#include <deque>
#include <numeric>
#include <cmath>

using namespace gtsam;

// ===== Replicate RangeStuckGuard from chimera_node_multi.cpp =====

struct RangeStuckGuard {
  static constexpr size_t N = 8;
  static constexpr double thresh_var = 0.001;
  std::deque<double> win;

  void add(double z){ win.push_back(z); if(win.size()>N) win.pop_front(); }
  bool isFull() const { return win.size()>=N; }
  bool check() const {
    if(!isFull()) return false;
    double mean = std::accumulate(win.begin(), win.end(), 0.0)/win.size();
    double var=0.0; for(double x: win){ double d=x-mean; var+=d*d; } var/=win.size();
    return var < thresh_var;
  }
  bool addAndCheck(double z){ add(z); return check(); }
  void reset(){ win.clear(); }
};

// ===== Replicate effectiveDepthMeters_AGL =====

double effectiveDepthMeters_AGL(const Pose3& x_wb, const Matrix3& R_bc, double agl_m){
  const Matrix3 R_wc = x_wb.rotation().matrix() * R_bc;
  const Eigen::Vector3d ez_cam_world = R_wc.col(2);
  const Eigen::Vector3d up(0,0,1);
  const double cosi = std::abs(up.dot(ez_cam_world));
  const double clamp = std::max(0.15, std::min(1.0, cosi));
  return agl_m / clamp;
}

// ===== Test Suite: RangeStuckGuard - Normal Operation =====

class RangeStuckGuardTest : public ::testing::Test {
protected:
    RangeStuckGuard guard;
};

TEST_F(RangeStuckGuardTest, InitiallyEmpty) {
    EXPECT_FALSE(guard.isFull());
    EXPECT_FALSE(guard.check());  // Can't check if not full
}

TEST_F(RangeStuckGuardTest, FillWindow) {
    // Add 8 samples
    for (int i = 0; i < 8; ++i) {
        guard.add(10.0 + i * 0.1);  // 10.0, 10.1, ..., 10.7
    }

    EXPECT_TRUE(guard.isFull());
    EXPECT_FALSE(guard.check());  // Not stuck (good variance)
}

TEST_F(RangeStuckGuardTest, PartialFill) {
    // Add only 5 samples
    for (int i = 0; i < 5; ++i) {
        guard.add(10.0);
    }

    EXPECT_FALSE(guard.isFull());  // Need 8 samples
    EXPECT_FALSE(guard.check());
}

TEST_F(RangeStuckGuardTest, SlidingWindow) {
    // Fill with 8 samples
    for (int i = 0; i < 8; ++i) {
        guard.add(10.0 + i * 0.1);
    }
    EXPECT_TRUE(guard.isFull());

    // Add one more - should evict oldest
    guard.add(11.0);
    EXPECT_TRUE(guard.isFull());
    EXPECT_EQ(guard.win.size(), 8);  // Still 8 samples
}

TEST_F(RangeStuckGuardTest, Reset) {
    for (int i = 0; i < 8; ++i) {
        guard.add(10.0);
    }
    EXPECT_TRUE(guard.isFull());

    guard.reset();

    EXPECT_FALSE(guard.isFull());
    EXPECT_EQ(guard.win.size(), 0);
}

// ===== Test Suite: Stuck Detection =====

class StuckDetectionTest : public ::testing::Test {
protected:
    RangeStuckGuard guard;
};

TEST_F(StuckDetectionTest, DetectStuck_ConstantValue) {
    // Add 8 identical samples (variance = 0)
    for (int i = 0; i < 8; ++i) {
        guard.add(10.0);
    }

    EXPECT_TRUE(guard.isFull());
    EXPECT_TRUE(guard.check());  // STUCK (zero variance)
}

TEST_F(StuckDetectionTest, DetectStuck_TinyVariance) {
    // Add 8 samples with very tiny variance
    for (int i = 0; i < 8; ++i) {
        guard.add(10.0 + i * 0.0001);  // Variance < 0.001
    }

    EXPECT_TRUE(guard.isFull());
    EXPECT_TRUE(guard.check());  // STUCK (variance < threshold)
}

TEST_F(StuckDetectionTest, NotStuck_GoodVariance) {
    // Add 8 samples with good variance
    for (int i = 0; i < 8; ++i) {
        guard.add(10.0 + i * 0.1);  // Variance > 0.001
    }

    EXPECT_TRUE(guard.isFull());
    EXPECT_FALSE(guard.check());  // NOT STUCK
}

TEST_F(StuckDetectionTest, NotStuck_LargeVariance) {
    // Add 8 samples with large variance
    double samples[] = {8.0, 12.0, 9.5, 11.0, 10.2, 9.8, 11.5, 10.5};
    for (double s : samples) {
        guard.add(s);
    }

    EXPECT_TRUE(guard.isFull());
    EXPECT_FALSE(guard.check());  // NOT STUCK
}

TEST_F(StuckDetectionTest, AddAndCheck_Stuck) {
    // Fill with varying samples first
    for (int i = 0; i < 7; ++i) {
        guard.add(10.0 + i * 0.1);
    }

    // Add 8th sample and check simultaneously
    bool stuck = guard.addAndCheck(10.7);
    EXPECT_FALSE(stuck);  // Not stuck (good variance)

    // Now add all identical samples
    guard.reset();
    for (int i = 0; i < 7; ++i) {
        guard.add(10.0);
    }
    stuck = guard.addAndCheck(10.0);
    EXPECT_TRUE(stuck);  // STUCK (all identical)
}

// ===== Test Suite: Boundary Cases =====

class StuckGuardBoundaryTest : public ::testing::Test {
protected:
    RangeStuckGuard guard;
};

TEST_F(StuckGuardBoundaryTest, ExactlyAtThreshold) {
    // Create variance exactly at threshold (0.001)
    // For uniform distribution, var = ((b-a)²)/12
    // So (b-a)² = 0.012, b-a = 0.1095
    double samples[] = {10.0, 10.015, 10.031, 10.046, 10.062, 10.077, 10.093, 10.109};
    for (double s : samples) {
        guard.add(s);
    }

    EXPECT_TRUE(guard.isFull());
    // Variance should be very close to threshold
    double mean = std::accumulate(guard.win.begin(), guard.win.end(), 0.0) / guard.win.size();
    double var = 0.0;
    for (double x : guard.win) {
        double d = x - mean;
        var += d * d;
    }
    var /= guard.win.size();

    EXPECT_NEAR(var, 0.001, 0.0005);  // Close to threshold
}

TEST_F(StuckGuardBoundaryTest, NegativeValues) {
    // Test with negative range values (shouldn't happen, but test robustness)
    for (int i = 0; i < 8; ++i) {
        guard.add(-5.0 + i * 0.1);
    }

    EXPECT_TRUE(guard.isFull());
    EXPECT_FALSE(guard.check());  // Not stuck (good variance)
}

TEST_F(StuckGuardBoundaryTest, VeryLargeValues) {
    // Test with very large values
    for (int i = 0; i < 8; ++i) {
        guard.add(10000.0 + i * 1.0);
    }

    EXPECT_TRUE(guard.isFull());
    EXPECT_FALSE(guard.check());  // Not stuck
}

// ===== Test Suite: Effective Depth (Tilt Compensation) =====

class EffectiveDepthTest : public ::testing::Test {
protected:
    // Nadir camera mount (pointing down in body frame)
    Matrix3 R_bc_nadir;

    void SetUp() override {
        R_bc_nadir <<  0,  1,  0,
                      -1,  0,  0,
                       0,  0,  1;
    }
};

TEST_F(EffectiveDepthTest, LevelFlight_NoTilt) {
    // Drone is level (no rotation), 10m AGL
    Pose3 pose_level(Rot3(), Point3(0, 0, 10));
    double agl = 10.0;

    double z_eff = effectiveDepthMeters_AGL(pose_level, R_bc_nadir, agl);

    // cos(0°) = 1, so z_eff = h / 1 = h
    EXPECT_NEAR(z_eff, 10.0, 0.01);
}

TEST_F(EffectiveDepthTest, Pitch30Degrees) {
    // Drone pitched 30° nose up
    Pose3 pose_pitch30(Rot3::Pitch(30.0 * M_PI / 180.0), Point3(0, 0, 10));
    double agl = 10.0;

    double z_eff = effectiveDepthMeters_AGL(pose_pitch30, R_bc_nadir, agl);

    // cos(30°) ≈ 0.866, z_eff = h / cos(30°) ≈ 11.55m
    EXPECT_NEAR(z_eff, 10.0 / std::cos(30.0 * M_PI / 180.0), 0.5);
}

TEST_F(EffectiveDepthTest, Roll45Degrees) {
    // Drone rolled 45° (banking turn)
    Pose3 pose_roll45(Rot3::Roll(45.0 * M_PI / 180.0), Point3(0, 0, 10));
    double agl = 10.0;

    double z_eff = effectiveDepthMeters_AGL(pose_roll45, R_bc_nadir, agl);

    // cos(45°) ≈ 0.707, z_eff = h / cos(45°) ≈ 14.14m
    EXPECT_NEAR(z_eff, 10.0 / std::cos(45.0 * M_PI / 180.0), 0.5);
}

TEST_F(EffectiveDepthTest, Pitch60Degrees) {
    // Steep pitch (60°)
    Pose3 pose_pitch60(Rot3::Pitch(60.0 * M_PI / 180.0), Point3(0, 0, 10));
    double agl = 10.0;

    double z_eff = effectiveDepthMeters_AGL(pose_pitch60, R_bc_nadir, agl);

    // cos(60°) = 0.5, z_eff = h / 0.5 = 20m
    EXPECT_NEAR(z_eff, 20.0, 1.0);
}

TEST_F(EffectiveDepthTest, ExtremeTilt_Clamping) {
    // Extreme tilt (85°) - should clamp to prevent division by near-zero
    Pose3 pose_tilt85(Rot3::Roll(85.0 * M_PI / 180.0), Point3(0, 0, 10));
    double agl = 10.0;

    double z_eff = effectiveDepthMeters_AGL(pose_tilt85, R_bc_nadir, agl);

    // cos(85°) ≈ 0.087, but clamped to 0.15 minimum
    // z_eff = h / 0.15 ≈ 66.67m
    EXPECT_NEAR(z_eff, 10.0 / 0.15, 2.0);
    EXPECT_LT(z_eff, 100.0);  // Should be clamped
}

TEST_F(EffectiveDepthTest, NearVertical_90Degrees) {
    // Camera nearly vertical (90° tilt) - extreme case
    Pose3 pose_90deg(Rot3::Roll(90.0 * M_PI / 180.0), Point3(0, 0, 10));
    double agl = 10.0;

    double z_eff = effectiveDepthMeters_AGL(pose_90deg, R_bc_nadir, agl);

    // Should be clamped to maximum (min clamp = 0.15)
    EXPECT_GT(z_eff, 50.0);  // Very large depth
    EXPECT_LT(z_eff, 100.0);  // But clamped
}

TEST_F(EffectiveDepthTest, CombinedPitchRoll) {
    // Combined pitch and roll (realistic maneuver)
    Rot3 rotation = Rot3::Ypr(0.0, 20.0 * M_PI / 180.0, 15.0 * M_PI / 180.0);
    Pose3 pose(rotation, Point3(0, 0, 10));
    double agl = 10.0;

    double z_eff = effectiveDepthMeters_AGL(pose, R_bc_nadir, agl);

    // Should be > 10m (tilted), but reasonable
    EXPECT_GT(z_eff, 10.0);
    EXPECT_LT(z_eff, 15.0);
}

TEST_F(EffectiveDepthTest, DifferentAltitudes) {
    Pose3 pose_level(Rot3(), Point3(0, 0, 5));

    // Test at different altitudes (linear scaling)
    double z_eff_5m = effectiveDepthMeters_AGL(pose_level, R_bc_nadir, 5.0);
    double z_eff_10m = effectiveDepthMeters_AGL(pose_level, R_bc_nadir, 10.0);
    double z_eff_20m = effectiveDepthMeters_AGL(pose_level, R_bc_nadir, 20.0);

    EXPECT_NEAR(z_eff_5m, 5.0, 0.01);
    EXPECT_NEAR(z_eff_10m, 10.0, 0.01);
    EXPECT_NEAR(z_eff_20m, 20.0, 0.01);

    // Should scale linearly
    EXPECT_NEAR(z_eff_10m / z_eff_5m, 2.0, 0.01);
}

TEST_F(EffectiveDepthTest, ForwardCameraMount) {
    // Different camera mount: forward-looking (not nadir)
    Matrix3 R_bc_forward;
    R_bc_forward << 1, 0, 0,
                    0, 0, -1,
                    0, 1, 0;  // Camera looking forward

    Pose3 pose_level(Rot3(), Point3(0, 0, 10));
    double agl = 10.0;

    double z_eff = effectiveDepthMeters_AGL(pose_level, R_bc_forward, agl);

    // Forward camera at level pitch should have cos(90°) ≈ 0, clamped to 0.15
    EXPECT_GT(z_eff, 50.0);  // Very large (camera not pointing down)
}

// ===== Test Suite: Edge Cases =====

class DepthAlgorithmsEdgeTest : public ::testing::Test {};

TEST_F(DepthAlgorithmsEdgeTest, ZeroAltitude) {
    Matrix3 R_bc_nadir;
    R_bc_nadir <<  0,  1,  0,
                  -1,  0,  0,
                   0,  0,  1;

    Pose3 pose(Rot3(), Point3(0, 0, 0));
    double z_eff = effectiveDepthMeters_AGL(pose, R_bc_nadir, 0.0);

    EXPECT_NEAR(z_eff, 0.0, 0.01);  // Should handle zero altitude
}

TEST_F(DepthAlgorithmsEdgeTest, NegativeAltitude) {
    Matrix3 R_bc_nadir;
    R_bc_nadir <<  0,  1,  0,
                  -1,  0,  0,
                   0,  0,  1;

    Pose3 pose(Rot3(), Point3(0, 0, -5));
    double z_eff = effectiveDepthMeters_AGL(pose, R_bc_nadir, -5.0);

    // Should compute correctly even with negative (below ground)
    EXPECT_NEAR(z_eff, -5.0, 0.01);
}

TEST_F(DepthAlgorithmsEdgeTest, VeryHighAltitude) {
    Matrix3 R_bc_nadir;
    R_bc_nadir <<  0,  1,  0,
                  -1,  0,  0,
                   0,  0,  1;

    Pose3 pose(Rot3(), Point3(0, 0, 1000));
    double z_eff = effectiveDepthMeters_AGL(pose, R_bc_nadir, 1000.0);

    EXPECT_NEAR(z_eff, 1000.0, 1.0);  // Should scale correctly
}

// Note: main() is defined in test_factors.cpp
// All tests are automatically discovered by Google Test
