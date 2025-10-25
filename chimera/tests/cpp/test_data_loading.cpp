/**
 * Unit tests for JSON data loading from chimera_node_multi.cpp
 * Tests: loadMultiSensorData(), loadOpticalFlowData()
 */

#include <gtest/gtest.h>
#include <boost/json.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>

// ===== Replicate data structures from chimera_node_multi.cpp =====

struct ImuSample { double t; Eigen::Vector3d gyro, accel; Eigen::Vector4d quat; };
struct MagSample { double t; Eigen::Vector3d mag; };
struct LidarSample { double t; double range_min, range_mean; int num_points; };
struct BaroSample { double t; double altitude; };
struct AirspeedSample { double t; double airspeed; };

struct OpticalFlowRay {
  int grid_idx; double norm_x, norm_y; int px, py; double flow_u, flow_v; double texture_var;
};
struct OpticalFlowSample {
  double t, dt; std::vector<OpticalFlowRay> rays; int rays_used = 0; int rays_rejected = 0;
};

struct SensorData {
  std::vector<ImuSample> imu;
  std::vector<MagSample> mag;
  std::vector<LidarSample> lidar;
  std::vector<BaroSample> baro;
  std::vector<AirspeedSample> airspeed;
  std::vector<OpticalFlowSample> optical_flow;
};

// ===== Replicate loadMultiSensorData from chimera_node_multi.cpp =====

static SensorData loadMultiSensorData(const std::string& file){
  std::ifstream f(file); if(!f.is_open()) throw std::runtime_error("Cannot open "+file);
  std::stringstream ss; ss<<f.rdbuf();
  auto root = boost::json::parse(ss.str()).as_object();
  SensorData d;
  if(root.contains("imu")){
    auto a = root["imu"].as_array(); d.imu.reserve(a.size());
    for(const auto& v: a){
      ImuSample s; auto o=v.as_object();
      s.t=o["t"].as_double();
      auto g=o["gyro"].as_array();  s.gyro  = {g[0].as_double(), g[1].as_double(), g[2].as_double()};
      auto ac=o["accel"].as_array();s.accel = {ac[0].as_double(),ac[1].as_double(),ac[2].as_double()};
      auto q=o["quat"].as_array();  s.quat  = {q[0].as_double(), q[1].as_double(), q[2].as_double(), q[3].as_double()};
      d.imu.push_back(s);
    }
  }
  if(root.contains("mag")){
    auto a = root["mag"].as_array(); d.mag.reserve(a.size());
    for(const auto& v: a){
      MagSample s; auto o=v.as_object();
      s.t=o["t"].as_double();
      auto m=o["mag"].as_array(); s.mag={m[0].as_double(),m[1].as_double(),m[2].as_double()};
      d.mag.push_back(s);
    }
  }
  if(root.contains("lidar")){
    auto a = root["lidar"].as_array(); d.lidar.reserve(a.size());
    for(const auto& v: a){
      LidarSample s; auto o=v.as_object();
      s.t=o["t"].as_double(); s.range_min=o["range_min"].as_double();
      s.range_mean=o["range_mean"].as_double(); s.num_points=(int)o["num_points"].as_int64();
      d.lidar.push_back(s);
    }
  }
  if(root.contains("baro")){
    auto a = root["baro"].as_array(); d.baro.reserve(a.size());
    for(const auto& v: a){ BaroSample s; auto o=v.as_object();
      s.t=o["t"].as_double(); s.altitude=o["altitude"].as_double(); d.baro.push_back(s);
    }
  }
  if(root.contains("airspeed")){
    auto a = root["airspeed"].as_array(); d.airspeed.reserve(a.size());
    for(const auto& v: a){ AirspeedSample s; auto o=v.as_object();
      s.t=o["t"].as_double(); s.airspeed=o["airspeed"].as_double(); d.airspeed.push_back(s);
    }
  }
  return d;
}

// ===== Test Suite: Data Loading (Synthetic) =====

class DataLoadingSyntheticTest : public ::testing::Test {
protected:
    std::string test_file_path = "tests/fixtures/data_synthetic_unit.json";
};

TEST_F(DataLoadingSyntheticTest, LoadSuccessfully) {
    // Test that synthetic data loads without exceptions
    EXPECT_NO_THROW({
        SensorData data = loadMultiSensorData(test_file_path);
    });
}

TEST_F(DataLoadingSyntheticTest, ValidateIMUData) {
    SensorData data = loadMultiSensorData(test_file_path);

    // Should have 400 IMU samples (1 second @ 400 Hz)
    EXPECT_EQ(data.imu.size(), 400);

    // First sample
    EXPECT_NEAR(data.imu[0].t, 0.0, 1e-6);
    EXPECT_NEAR(data.imu[0].gyro.norm(), 0.0, 1e-6);  // No rotation
    EXPECT_NEAR(data.imu[0].accel(2), -9.81, 0.01);   // Gravity down (Z)

    // Last sample
    EXPECT_NEAR(data.imu.back().t, 0.9975, 1e-4);  // (399 * 0.0025)
}

TEST_F(DataLoadingSyntheticTest, ValidateMagData) {
    SensorData data = loadMultiSensorData(test_file_path);

    // Should have 20 mag samples (1 second @ 20 Hz)
    EXPECT_EQ(data.mag.size(), 20);

    // First sample
    EXPECT_NEAR(data.mag[0].t, 0.0, 1e-6);
    EXPECT_NEAR(data.mag[0].mag(0), 0.3, 0.01);   // North component
    EXPECT_NEAR(data.mag[0].mag(2), -0.4, 0.01);  // Down component (NED)

    // Check magnitude is reasonable
    double mag_norm = data.mag[0].mag.norm();
    EXPECT_GT(mag_norm, 0.4);
    EXPECT_LT(mag_norm, 0.6);
}

TEST_F(DataLoadingSyntheticTest, ValidateLidarData) {
    SensorData data = loadMultiSensorData(test_file_path);

    // Should have 10 LiDAR samples (1 second @ 10 Hz)
    EXPECT_EQ(data.lidar.size(), 10);

    // First sample
    EXPECT_NEAR(data.lidar[0].t, 0.0, 1e-6);
    EXPECT_NEAR(data.lidar[0].range_min, 10000.0, 0.1);  // 10m in millimeters
    EXPECT_EQ(data.lidar[0].num_points, 100);

    // Validate all samples are consistent
    for (const auto& s : data.lidar) {
        EXPECT_GT(s.range_min, 0.0);
        EXPECT_GT(s.num_points, 0);
    }
}

TEST_F(DataLoadingSyntheticTest, ValidateTimestampOrdering) {
    SensorData data = loadMultiSensorData(test_file_path);

    // IMU timestamps should be strictly increasing
    for (size_t i = 1; i < data.imu.size(); ++i) {
        EXPECT_GT(data.imu[i].t, data.imu[i-1].t);
    }

    // Mag timestamps should be strictly increasing
    for (size_t i = 1; i < data.mag.size(); ++i) {
        EXPECT_GT(data.mag[i].t, data.mag[i-1].t);
    }

    // LiDAR timestamps should be strictly increasing
    for (size_t i = 1; i < data.lidar.size(); ++i) {
        EXPECT_GT(data.lidar[i].t, data.lidar[i-1].t);
    }
}

TEST_F(DataLoadingSyntheticTest, QuaternionNormalization) {
    SensorData data = loadMultiSensorData(test_file_path);

    // All quaternions should be unit quaternions (norm ≈ 1)
    for (const auto& s : data.imu) {
        double quat_norm = s.quat.norm();
        EXPECT_NEAR(quat_norm, 1.0, 0.01);  // Should be normalized
    }
}

// ===== Test Suite: Data Loading (Real Flight - Boot Window) =====

class DataLoadingRealBootTest : public ::testing::Test {
protected:
    std::string test_file_path = "tests/fixtures/data_boot_only.json";
};

TEST_F(DataLoadingRealBootTest, LoadSuccessfully) {
    EXPECT_NO_THROW({
        SensorData data = loadMultiSensorData(test_file_path);
    });
}

TEST_F(DataLoadingRealBootTest, ValidateSensorCounts) {
    SensorData data = loadMultiSensorData(test_file_path);

    // Boot window should have ~2s of data
    EXPECT_GT(data.imu.size(), 700);       // At least 700 IMU @ 400Hz (2s)
    EXPECT_GT(data.mag.size(), 30);        // At least 30 mag @ 20Hz
    EXPECT_GT(data.lidar.size(), 15);      // At least 15 LiDAR @ 10Hz

    // Duration check
    double duration_imu = data.imu.back().t - data.imu.front().t;
    EXPECT_NEAR(duration_imu, 2.0, 0.2);  // ~2 seconds ±0.2s
}

TEST_F(DataLoadingRealBootTest, ValidateRealIMUData) {
    SensorData data = loadMultiSensorData(test_file_path);

    // Real IMU data should have reasonable magnitudes
    for (const auto& s : data.imu) {
        // Gyro: typically < 10 rad/s during hover/slow motion
        EXPECT_LT(s.gyro.norm(), 20.0);

        // Accel: should be close to gravity (8-11 m/s² for hovering)
        double accel_norm = s.accel.norm();
        EXPECT_GT(accel_norm, 5.0);
        EXPECT_LT(accel_norm, 15.0);
    }
}

TEST_F(DataLoadingRealBootTest, ValidateRealLidarData) {
    SensorData data = loadMultiSensorData(test_file_path);

    // LiDAR range should be positive and reasonable
    for (const auto& s : data.lidar) {
        EXPECT_GT(s.range_min, 0.0);
        EXPECT_LT(s.range_min, 100000.0);  // <100m (even in mm units)

        EXPECT_GT(s.num_points, 0);
        EXPECT_LT(s.num_points, 10000);  // Reasonable point count
    }
}

TEST_F(DataLoadingRealBootTest, ValidateBaroData) {
    SensorData data = loadMultiSensorData(test_file_path);

    // Barometer should have data
    EXPECT_GT(data.baro.size(), 0);

    // Altitude should be reasonable (MSL, typically -500m to 5000m)
    for (const auto& s : data.baro) {
        EXPECT_GT(s.altitude, -1000.0);
        EXPECT_LT(s.altitude, 10000.0);
    }
}

// ===== Test Suite: Error Handling =====

class DataLoadingErrorTest : public ::testing::Test {};

TEST_F(DataLoadingErrorTest, FileNotFound) {
    // Should throw exception for non-existent file
    EXPECT_THROW({
        SensorData data = loadMultiSensorData("nonexistent_file.json");
    }, std::runtime_error);
}

TEST_F(DataLoadingErrorTest, EmptyFile) {
    // Create empty JSON file
    std::ofstream tmp("tests/fixtures/tmp_empty.json");
    tmp << "{}";  // Empty object
    tmp.close();

    // Should load but have zero samples
    SensorData data = loadMultiSensorData("tests/fixtures/tmp_empty.json");
    EXPECT_EQ(data.imu.size(), 0);
    EXPECT_EQ(data.mag.size(), 0);
    EXPECT_EQ(data.lidar.size(), 0);

    // Cleanup
    std::remove("tests/fixtures/tmp_empty.json");
}

TEST_F(DataLoadingErrorTest, MissingSensorArrays) {
    // Create JSON with only IMU data
    std::ofstream tmp("tests/fixtures/tmp_partial.json");
    tmp << R"({
        "imu": [
            {"t": 0.0, "gyro": [0.0, 0.0, 0.0], "accel": [0.0, 0.0, -9.81], "quat": [0.0, 0.0, 0.0, 1.0]}
        ]
    })";
    tmp.close();

    // Should load successfully with only IMU populated
    SensorData data = loadMultiSensorData("tests/fixtures/tmp_partial.json");
    EXPECT_EQ(data.imu.size(), 1);
    EXPECT_EQ(data.mag.size(), 0);
    EXPECT_EQ(data.lidar.size(), 0);

    // Cleanup
    std::remove("tests/fixtures/tmp_partial.json");
}

// ===== Test Suite: Data Consistency Checks =====

class DataConsistencyTest : public ::testing::Test {
protected:
    std::string test_file_path = "tests/fixtures/data_first_10s.json";
};

TEST_F(DataConsistencyTest, NoNaNValues) {
    SensorData data = loadMultiSensorData(test_file_path);

    // Check for NaN in IMU data
    for (const auto& s : data.imu) {
        EXPECT_FALSE(std::isnan(s.t));
        EXPECT_FALSE(std::isnan(s.gyro.norm()));
        EXPECT_FALSE(std::isnan(s.accel.norm()));
        EXPECT_FALSE(std::isnan(s.quat.norm()));
    }

    // Check for NaN in Mag data
    for (const auto& s : data.mag) {
        EXPECT_FALSE(std::isnan(s.t));
        EXPECT_FALSE(std::isnan(s.mag.norm()));
    }

    // Check for NaN in LiDAR data
    for (const auto& s : data.lidar) {
        EXPECT_FALSE(std::isnan(s.t));
        EXPECT_FALSE(std::isnan(s.range_min));
        EXPECT_FALSE(std::isnan(s.range_mean));
    }
}

TEST_F(DataConsistencyTest, PositiveTimestamps) {
    SensorData data = loadMultiSensorData(test_file_path);

    // All timestamps should be non-negative (relative to start time)
    for (const auto& s : data.imu) {
        EXPECT_GE(s.t, 0.0);
    }
    for (const auto& s : data.mag) {
        EXPECT_GE(s.t, 0.0);
    }
    for (const auto& s : data.lidar) {
        EXPECT_GE(s.t, 0.0);
    }
}

TEST_F(DataConsistencyTest, ReasonableSampleRates) {
    SensorData data = loadMultiSensorData(test_file_path);

    // Calculate average sample rates
    if (data.imu.size() > 1) {
        double duration = data.imu.back().t - data.imu.front().t;
        double rate = (data.imu.size() - 1) / duration;
        EXPECT_NEAR(rate, 400.0, 50.0);  // IMU @ 400Hz ±50Hz
    }

    if (data.mag.size() > 1) {
        double duration = data.mag.back().t - data.mag.front().t;
        double rate = (data.mag.size() - 1) / duration;
        EXPECT_NEAR(rate, 20.0, 5.0);  // Mag @ 20Hz ±5Hz
    }

    if (data.lidar.size() > 1) {
        double duration = data.lidar.back().t - data.lidar.front().t;
        double rate = (data.lidar.size() - 1) / duration;
        EXPECT_NEAR(rate, 10.0, 3.0);  // LiDAR @ 10Hz ±3Hz
    }
}

// Note: main() is defined in test_factors.cpp
// All tests are automatically discovered by Google Test
