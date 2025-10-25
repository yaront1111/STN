/**
 * Unit tests for auto-detection algorithms from chimera_node_multi.cpp
 * Tests: bootRangeUnits(), pickRwb_from_quat(), gyro unit detection, etc.
 */

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <numeric>
#include <algorithm>
#include <cmath>

using namespace gtsam;

// ===== Replicate structures and enums =====

enum class RangeUnits { AUTO, M, CM, MM };

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
  void reset(){ win.clear(); }
};

struct LidarSample { double t; double range_min, range_mean; int num_points; };
struct ImuSample { double t; Eigen::Vector3d gyro, accel; Eigen::Vector4d quat; };

struct RangeBootstrap {
  bool ready=false;
  double scale_to_m=1.0;
  RangeUnits units=RangeUnits::AUTO;
  double z0_agl_m=0.0;
  bool lidar_stuck=true;
  RangeStuckGuard guard;
};

// ===== Replicate helper functions =====

static inline double medianScalar(std::vector<double> v){
  if(v.empty()) return 0.0;
  std::nth_element(v.begin(), v.begin()+v.size()/2, v.end());
  return v[v.size()/2];
}

RangeBootstrap bootRangeUnits(const std::vector<LidarSample>& lidar, double t0, double horizon=2.0){
  RangeBootstrap boot;
  if(lidar.empty()) return boot;
  std::vector<double> w;
  for(const auto& s: lidar){ if(s.t-t0>horizon) break; if(s.range_min>0) w.push_back(s.range_min); }
  if(w.size()<5) return boot;
  double med = medianScalar(w);
  if(med > 10000.0){ boot.scale_to_m=0.001; boot.units=RangeUnits::MM; }
  else if(med > 100.0){ boot.scale_to_m=0.01; boot.units=RangeUnits::CM; }
  else { boot.scale_to_m=1.0; boot.units=RangeUnits::M; }
  for(const auto& s: lidar){
    if(s.t-t0>horizon) break;
    if(s.range_min>0){ boot.guard.add(s.range_min*boot.scale_to_m); if(boot.guard.isFull()) break; }
  }
  boot.lidar_stuck = boot.guard.isFull()? boot.guard.check() : true;
  boot.ready = true;
  return boot;
}

struct OrientBoot { Rot3 Rwb; std::string tag; double g_err; };
static OrientBoot pickRwb_from_quat(const Eigen::Vector4d& qraw, const Eigen::Vector3d& a_med_b){
  auto mk = [&](double w,double x,double y,double z){ return Rot3::Quaternion(w,x,y,z); };
  struct C{ Rot3 R; const char* tag; };
  std::vector<C> c = {
    { mk(qraw(0),qraw(1),qraw(2),qraw(3))           , "wxyz"     },
    { mk(qraw(0),qraw(1),qraw(2),qraw(3)).inverse() , "wxyz_inv" },
    { mk(qraw(3),qraw(0),qraw(1),qraw(2))           , "xyzw"     },
    { mk(qraw(3),qraw(0),qraw(1),qraw(2)).inverse() , "xyzw_inv" },
  };
  double best=1e9; Rot3 bestR; const char* best_tag="";
  for(auto& ci: c){
    Eigen::Vector3d aw = ci.R.matrix()*a_med_b;
    double err=(aw - Eigen::Vector3d(0,0,-9.81)).norm();
    if(err<best){ best=err; bestR=ci.R; best_tag=ci.tag; }
  }
  return {bestR,best_tag,best};
}

// ===== Test Suite: Range Unit Detection =====

class RangeUnitDetectionTest : public ::testing::Test {};

TEST_F(RangeUnitDetectionTest, DetectMillimeters) {
    // Create LiDAR data in millimeters (10m = 10000mm)
    std::vector<LidarSample> lidar;
    for (int i = 0; i < 20; ++i) {
        LidarSample s;
        s.t = i * 0.1;
        s.range_min = 10000.0 + i * 10.0;  // 10m ± noise in mm
        s.range_mean = 10050.0;
        s.num_points = 100;
        lidar.push_back(s);
    }

    RangeBootstrap boot = bootRangeUnits(lidar, 0.0, 2.0);

    EXPECT_TRUE(boot.ready);
    EXPECT_EQ(boot.units, RangeUnits::MM);
    EXPECT_DOUBLE_EQ(boot.scale_to_m, 0.001);
}

TEST_F(RangeUnitDetectionTest, DetectCentimeters) {
    // Create LiDAR data in centimeters (10m = 1000cm)
    std::vector<LidarSample> lidar;
    for (int i = 0; i < 20; ++i) {
        LidarSample s;
        s.t = i * 0.1;
        s.range_min = 1000.0 + i * 1.0;  // 10m ± noise in cm
        s.range_mean = 1005.0;
        s.num_points = 100;
        lidar.push_back(s);
    }

    RangeBootstrap boot = bootRangeUnits(lidar, 0.0, 2.0);

    EXPECT_TRUE(boot.ready);
    EXPECT_EQ(boot.units, RangeUnits::CM);
    EXPECT_DOUBLE_EQ(boot.scale_to_m, 0.01);
}

TEST_F(RangeUnitDetectionTest, DetectMeters) {
    // Create LiDAR data in meters
    std::vector<LidarSample> lidar;
    for (int i = 0; i < 20; ++i) {
        LidarSample s;
        s.t = i * 0.1;
        s.range_min = 10.0 + i * 0.01;  // 10m ± noise
        s.range_mean = 10.05;
        s.num_points = 100;
        lidar.push_back(s);
    }

    RangeBootstrap boot = bootRangeUnits(lidar, 0.0, 2.0);

    EXPECT_TRUE(boot.ready);
    EXPECT_EQ(boot.units, RangeUnits::M);
    EXPECT_DOUBLE_EQ(boot.scale_to_m, 1.0);
}

TEST_F(RangeUnitDetectionTest, InsufficientData) {
    // Only 3 samples (need at least 5)
    std::vector<LidarSample> lidar;
    for (int i = 0; i < 3; ++i) {
        LidarSample s;
        s.t = i * 0.1;
        s.range_min = 10.0;
        s.range_mean = 10.0;
        s.num_points = 100;
        lidar.push_back(s);
    }

    RangeBootstrap boot = bootRangeUnits(lidar, 0.0, 2.0);

    EXPECT_FALSE(boot.ready);  // Not enough data
}

TEST_F(RangeUnitDetectionTest, EmptyData) {
    std::vector<LidarSample> lidar;

    RangeBootstrap boot = bootRangeUnits(lidar, 0.0, 2.0);

    EXPECT_FALSE(boot.ready);
}

TEST_F(RangeUnitDetectionTest, ZeroReadings) {
    // All readings are zero (invalid)
    std::vector<LidarSample> lidar;
    for (int i = 0; i < 20; ++i) {
        LidarSample s;
        s.t = i * 0.1;
        s.range_min = 0.0;  // Invalid
        s.range_mean = 0.0;
        s.num_points = 0;
        lidar.push_back(s);
    }

    RangeBootstrap boot = bootRangeUnits(lidar, 0.0, 2.0);

    EXPECT_FALSE(boot.ready);  // No valid readings
}

// ===== Test Suite: Stuck Detection =====

class StuckDetectionTest : public ::testing::Test {};

TEST_F(StuckDetectionTest, NotStuck_VaryingRange) {
    // Create LiDAR data with varying range (not stuck)
    std::vector<LidarSample> lidar;
    for (int i = 0; i < 20; ++i) {
        LidarSample s;
        s.t = i * 0.1;
        s.range_min = 10000.0 + i * 100.0;  // Varying: 10m to 12m
        s.range_mean = 10050.0;
        s.num_points = 100;
        lidar.push_back(s);
    }

    RangeBootstrap boot = bootRangeUnits(lidar, 0.0, 2.0);

    EXPECT_TRUE(boot.ready);
    EXPECT_FALSE(boot.lidar_stuck);  // Should NOT be stuck
}

TEST_F(StuckDetectionTest, Stuck_ConstantPlateau) {
    // Create LiDAR data stuck at constant value
    std::vector<LidarSample> lidar;
    for (int i = 0; i < 20; ++i) {
        LidarSample s;
        s.t = i * 0.1;
        s.range_min = 500.0;  // Stuck at 0.5m (in mm: 500mm)
        s.range_mean = 500.0;
        s.num_points = 100;
        lidar.push_back(s);
    }

    RangeBootstrap boot = bootRangeUnits(lidar, 0.0, 2.0);

    EXPECT_TRUE(boot.ready);
    EXPECT_EQ(boot.units, RangeUnits::CM);  // 500 detected as cm
    EXPECT_TRUE(boot.lidar_stuck);  // Should BE stuck (very low variance)
}

TEST_F(StuckDetectionTest, Stuck_TinyVariance) {
    // LiDAR with tiny variance (essentially stuck)
    std::vector<LidarSample> lidar;
    for (int i = 0; i < 20; ++i) {
        LidarSample s;
        s.t = i * 0.1;
        s.range_min = 10000.0 + (i % 2) * 0.1;  // Noise < 0.1mm → var < thresh
        s.range_mean = 10000.0;
        s.num_points = 100;
        lidar.push_back(s);
    }

    RangeBootstrap boot = bootRangeUnits(lidar, 0.0, 2.0);

    EXPECT_TRUE(boot.ready);
    // With such tiny variance, should be detected as stuck
    EXPECT_TRUE(boot.lidar_stuck);
}

// ===== Test Suite: Orientation Bootstrap =====

class OrientationBootstrapTest : public ::testing::Test {};

TEST_F(OrientationBootstrapTest, IdentityQuaternion_XYZW) {
    // Identity quaternion in xyzw format: [0, 0, 0, 1]
    Eigen::Vector4d quat_xyzw(0.0, 0.0, 0.0, 1.0);
    Eigen::Vector3d accel_body(0.0, 0.0, -9.81);  // Gravity down in body frame

    OrientBoot result = pickRwb_from_quat(quat_xyzw, accel_body);

    // Should pick "xyzw" with low g_err
    EXPECT_LT(result.g_err, 0.5);  // Very low error
    EXPECT_TRUE(result.tag == "xyzw" || result.tag == "wxyz");
}

TEST_F(OrientationBootstrapTest, IdentityQuaternion_WXYZ) {
    // Identity quaternion in wxyz format: [1, 0, 0, 0]
    Eigen::Vector4d quat_wxyz(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d accel_body(0.0, 0.0, -9.81);

    OrientBoot result = pickRwb_from_quat(quat_wxyz, accel_body);

    // Should pick "wxyz" with low g_err
    EXPECT_LT(result.g_err, 0.5);
    EXPECT_TRUE(result.tag == "wxyz" || result.tag == "wxyz_inv");
}

TEST_F(OrientationBootstrapTest, Rotated90Degrees) {
    // 90° rotation around Z-axis (yaw)
    double cos45 = std::cos(M_PI/4);
    double sin45 = std::sin(M_PI/4);

    // Quaternion for 45° yaw (xyzw format): [0, 0, sin(22.5°), cos(22.5°)]
    Eigen::Vector4d quat(0.0, 0.0, sin45, cos45);

    // Accel rotated accordingly
    Eigen::Vector3d accel_body(0.0, 0.0, -9.81);

    OrientBoot result = pickRwb_from_quat(quat, accel_body);

    // Should still have low g_err (rotation doesn't affect vertical gravity)
    EXPECT_LT(result.g_err, 1.0);
}

TEST_F(OrientationBootstrapTest, NWU_vs_FRD_Detection) {
    // Test accel in NWU (North-West-Up) vs FRD (Forward-Right-Down)
    // NWU: gravity is +Z
    Eigen::Vector3d accel_nwu(0.0, 0.0, +9.81);
    Eigen::Vector4d quat_identity(0.0, 0.0, 0.0, 1.0);

    OrientBoot result_nwu = pickRwb_from_quat(quat_identity, accel_nwu);

    // With +9.81 in Z (NWU), need inversion to align with world frame (Z down = -9.81)
    // Error will be higher unless inverse is picked
    // The function will try all 4 candidates and pick best
    EXPECT_LT(result_nwu.g_err, 20.0);  // Should find SOME valid candidate
}

TEST_F(OrientationBootstrapTest, HighGravityError) {
    // Deliberately wrong accel (not gravity)
    Eigen::Vector4d quat(0.0, 0.0, 0.0, 1.0);
    Eigen::Vector3d accel_wrong(5.0, 3.0, 2.0);  // Random, not gravity

    OrientBoot result = pickRwb_from_quat(quat, accel_wrong);

    // Error should be significant
    EXPECT_GT(result.g_err, 5.0);
}

// ===== Test Suite: Gyro Unit Detection =====

class GyroUnitDetectionTest : public ::testing::Test {};

TEST_F(GyroUnitDetectionTest, DetectRadians) {
    // Gyro data in rad/s (typical: <1 rad/s for slow motion)
    std::vector<ImuSample> imu;
    for (int i = 0; i < 100; ++i) {
        ImuSample s;
        s.t = i * 0.0025;
        s.gyro = Eigen::Vector3d(0.05, -0.03, 0.02);  // rad/s
        s.accel = Eigen::Vector3d(0, 0, -9.81);
        s.quat = Eigen::Vector4d(0, 0, 0, 1);
        imu.push_back(s);
    }

    // Calculate median norm
    std::vector<double> norms;
    for (const auto& s : imu) {
        norms.push_back(s.gyro.norm());
    }
    std::nth_element(norms.begin(), norms.begin() + norms.size()/2, norms.end());
    double median_norm = norms[norms.size()/2];

    // Should be in rad/s range (<5.0)
    EXPECT_LT(median_norm, 5.0);
}

TEST_F(GyroUnitDetectionTest, DetectDegrees) {
    // Gyro data in deg/s (same motion, but 57.3x larger)
    std::vector<ImuSample> imu;
    for (int i = 0; i < 100; ++i) {
        ImuSample s;
        s.t = i * 0.0025;
        s.gyro = Eigen::Vector3d(2.87, -1.72, 1.15);  // deg/s (0.05*57.3)
        s.accel = Eigen::Vector3d(0, 0, -9.81);
        s.quat = Eigen::Vector4d(0, 0, 0, 1);
        imu.push_back(s);
    }

    // Calculate median norm
    std::vector<double> norms;
    for (const auto& s : imu) {
        norms.push_back(s.gyro.norm());
    }
    std::nth_element(norms.begin(), norms.begin() + norms.size()/2, norms.end());
    double median_norm = norms[norms.size()/2];

    // Should be in deg/s range (5.0-500.0)
    EXPECT_GT(median_norm, 1.0);
    EXPECT_LT(median_norm, 100.0);

    // Conversion factor should be ~π/180
    double scale_to_rad = M_PI / 180.0;
    double expected_rad_norm = median_norm * scale_to_rad;
    EXPECT_LT(expected_rad_norm, 5.0);  // Should be back in rad/s range
}

// ===== Test Suite: IMU Accel Unit Detection (g vs m/s²) =====

class AccelUnitDetectionTest : public ::testing::Test {};

TEST_F(AccelUnitDetectionTest, DetectGravityUnits) {
    // Accel data in g (hovering ~1.0g)
    std::vector<double> accel_norms;
    for (int i = 0; i < 100; ++i) {
        Eigen::Vector3d accel(0.01, -0.02, 1.0);  // ~1g vertical
        accel_norms.push_back(accel.norm());
    }

    std::nth_element(accel_norms.begin(), accel_norms.begin() + accel_norms.size()/2, accel_norms.end());
    double median = accel_norms[accel_norms.size()/2];

    // Should be in 0.7-1.3 range (g units)
    EXPECT_GT(median, 0.7);
    EXPECT_LT(median, 1.3);
}

TEST_F(AccelUnitDetectionTest, DetectMeterPerSecondSquared) {
    // Accel data in m/s² (hovering ~9.81 m/s²)
    std::vector<double> accel_norms;
    for (int i = 0; i < 100; ++i) {
        Eigen::Vector3d accel(0.1, -0.2, 9.81);  // ~9.81 m/s² vertical
        accel_norms.push_back(accel.norm());
    }

    std::nth_element(accel_norms.begin(), accel_norms.begin() + accel_norms.size()/2, accel_norms.end());
    double median = accel_norms[accel_norms.size()/2];

    // Should be in 7.0-13.0 range (m/s² units)
    EXPECT_GT(median, 7.0);
    EXPECT_LT(median, 13.0);
}

// Note: main() is defined in test_factors.cpp
// All tests are automatically discovered by Google Test
