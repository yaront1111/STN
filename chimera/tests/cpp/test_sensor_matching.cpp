/**
 * Unit tests for sensor temporal matching from chimera_node_multi.cpp
 * Tests: nearestByTime() template function
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <vector>
#include <optional>
#include <algorithm>
#include <cmath>

// ===== Replicate sensor structures =====

struct MagSample { double t; Eigen::Vector3d mag; };
struct LidarSample { double t; double range_min, range_mean; int num_points; };
struct BaroSample { double t; double altitude; };

// ===== Replicate nearestByTime from chimera_node_multi.cpp =====

template<typename T>
std::optional<T> nearestByTime(const std::vector<T>& v, double t, double max_dt){
  if(v.empty()) return std::nullopt;
  auto it = std::lower_bound(v.begin(), v.end(), t, [](const T& s, double tt){ return s.t<tt; });
  auto ok = [&](const T& s){ return std::abs(s.t-t)<=max_dt; };
  if(it==v.end()) return ok(v.back())? std::optional<T>(v.back()) : std::nullopt;
  if(it==v.begin()) return ok(*it)? std::optional<T>(*it) : std::nullopt;
  auto prev=it-1;
  return (std::abs(prev->t-t)<std::abs(it->t-t) ? (ok(*prev)? std::optional<T>(*prev):std::nullopt)
                                                : (ok(*it)?   std::optional<T>(*it)  :std::nullopt));
}

// ===== Test Suite: Exact Matches =====

class SensorMatchingExactTest : public ::testing::Test {
protected:
    std::vector<MagSample> mag_samples;

    void SetUp() override {
        // Create 100 samples @ 20 Hz (every 0.05s)
        for (int i = 0; i < 100; ++i) {
            MagSample mag;
            mag.t = i * 0.05;  // 0.0, 0.05, 0.1, ..., 4.95
            mag.mag = Eigen::Vector3d(0.3, 0.0, -0.4);  // North-pointing NED
            mag_samples.push_back(mag);
        }
    }
};

TEST_F(SensorMatchingExactTest, ExactMatch) {
    // Query at exact sample time
    auto result = nearestByTime(mag_samples, 1.0, 0.05);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->t, 1.0);
}

TEST_F(SensorMatchingExactTest, MultipleExactMatches) {
    // Test multiple exact matches
    auto r1 = nearestByTime(mag_samples, 0.0, 0.05);
    ASSERT_TRUE(r1.has_value());
    EXPECT_DOUBLE_EQ(r1->t, 0.0);

    auto r2 = nearestByTime(mag_samples, 2.5, 0.05);
    ASSERT_TRUE(r2.has_value());
    EXPECT_DOUBLE_EQ(r2->t, 2.5);

    auto r3 = nearestByTime(mag_samples, 4.95, 0.05);
    ASSERT_TRUE(r3.has_value());
    EXPECT_DOUBLE_EQ(r3->t, 4.95);
}

// ===== Test Suite: Nearest Neighbor =====

class SensorMatchingNearestTest : public ::testing::Test {
protected:
    std::vector<BaroSample> baro_samples;

    void SetUp() override {
        // Create 50 samples @ 10 Hz (every 0.1s)
        for (int i = 0; i < 50; ++i) {
            BaroSample baro;
            baro.t = i * 0.1;  // 0.0, 0.1, 0.2, ..., 4.9
            baro.altitude = 100.0;  // Constant altitude
            baro_samples.push_back(baro);
        }
    }
};

TEST_F(SensorMatchingNearestTest, NearestBefore) {
    // Query at 1.02 - should match 1.0 (dt=0.02 < 0.05)
    auto result = nearestByTime(baro_samples, 1.02, 0.05);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->t, 1.0);
}

TEST_F(SensorMatchingNearestTest, NearestAfter) {
    // Query at 1.08 - should match 1.1 (dt=0.02 < 0.05)
    auto result = nearestByTime(baro_samples, 1.08, 0.05);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->t, 1.1);
}

TEST_F(SensorMatchingNearestTest, Midpoint) {
    // Query exactly between two samples (1.05 between 1.0 and 1.1)
    auto result = nearestByTime(baro_samples, 1.05, 0.1);
    ASSERT_TRUE(result.has_value());
    // Should match one of them (either 1.0 or 1.1)
    EXPECT_TRUE(result->t == 1.0 || result->t == 1.1);
}

TEST_F(SensorMatchingNearestTest, NearestWithTightTolerance) {
    // Query at 1.03 with very tight tolerance (0.02)
    auto result = nearestByTime(baro_samples, 1.03, 0.02);
    // Should fail because 1.03 is 0.03s from both 1.0 and 1.1
    EXPECT_FALSE(result.has_value());
}

// ===== Test Suite: Tolerance Testing =====

class SensorMatchingToleranceTest : public ::testing::Test {
protected:
    std::vector<LidarSample> lidar_samples;

    void SetUp() override {
        // Create sparse samples @ irregular intervals
        std::vector<double> times = {0.0, 0.5, 1.2, 2.1, 3.5, 5.0};
        for (double t : times) {
            LidarSample lidar;
            lidar.t = t;
            lidar.range_min = 10.0;  // 10m
            lidar.range_mean = 10.5;
            lidar.num_points = 100;
            lidar_samples.push_back(lidar);
        }
    }
};

TEST_F(SensorMatchingToleranceTest, WithinTolerance) {
    // Query at 1.15 with tolerance 0.1 - should match 1.2
    auto result = nearestByTime(lidar_samples, 1.15, 0.1);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->t, 1.2);
}

TEST_F(SensorMatchingToleranceTest, OutOfTolerance) {
    // Query at 1.5 with tolerance 0.2 - nearest is 1.2 (dt=0.3 > 0.2)
    auto result = nearestByTime(lidar_samples, 1.5, 0.2);
    EXPECT_FALSE(result.has_value());
}

TEST_F(SensorMatchingToleranceTest, LargeTolerance) {
    // Query at 4.0 with large tolerance 2.0 - should match 3.5
    auto result = nearestByTime(lidar_samples, 4.0, 2.0);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->t, 3.5);
}

TEST_F(SensorMatchingToleranceTest, ZeroTolerance) {
    // Query at 1.2 with zero tolerance - only exact match allowed
    auto result = nearestByTime(lidar_samples, 1.2, 0.0);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->t, 1.2);

    // Query at 1.21 with zero tolerance - should fail
    auto result2 = nearestByTime(lidar_samples, 1.21, 0.0);
    EXPECT_FALSE(result2.has_value());
}

// ===== Test Suite: Edge Cases =====

class SensorMatchingEdgeTest : public ::testing::Test {};

TEST_F(SensorMatchingEdgeTest, EmptyVector) {
    std::vector<MagSample> empty;
    auto result = nearestByTime(empty, 1.0, 0.1);
    EXPECT_FALSE(result.has_value());
}

TEST_F(SensorMatchingEdgeTest, SingleElement) {
    std::vector<BaroSample> single;
    BaroSample baro;
    baro.t = 1.0;
    baro.altitude = 100.0;
    single.push_back(baro);

    // Within tolerance
    auto r1 = nearestByTime(single, 1.05, 0.1);
    ASSERT_TRUE(r1.has_value());
    EXPECT_DOUBLE_EQ(r1->t, 1.0);

    // Out of tolerance
    auto r2 = nearestByTime(single, 2.0, 0.5);
    EXPECT_FALSE(r2.has_value());
}

TEST_F(SensorMatchingEdgeTest, BeforeFirstSample) {
    std::vector<MagSample> samples;
    for (int i = 10; i < 20; ++i) {
        MagSample mag;
        mag.t = i * 0.1;  // Start at 1.0
        mag.mag = Eigen::Vector3d::Zero();
        samples.push_back(mag);
    }

    // Query before first sample
    auto result = nearestByTime(samples, 0.5, 0.1);
    EXPECT_FALSE(result.has_value());

    // Query close to first sample (within tolerance)
    auto result2 = nearestByTime(samples, 0.95, 0.1);
    ASSERT_TRUE(result2.has_value());
    EXPECT_DOUBLE_EQ(result2->t, 1.0);
}

TEST_F(SensorMatchingEdgeTest, AfterLastSample) {
    std::vector<LidarSample> samples;
    for (int i = 0; i < 10; ++i) {
        LidarSample lidar;
        lidar.t = i * 0.1;  // 0.0 to 0.9
        lidar.range_min = 10.0;
        lidar.range_mean = 10.0;
        lidar.num_points = 100;
        samples.push_back(lidar);
    }

    // Query after last sample (out of tolerance)
    auto result = nearestByTime(samples, 2.0, 0.1);
    EXPECT_FALSE(result.has_value());

    // Query close to last sample (within tolerance)
    auto result2 = nearestByTime(samples, 0.95, 0.1);
    ASSERT_TRUE(result2.has_value());
    EXPECT_DOUBLE_EQ(result2->t, 0.9);
}

TEST_F(SensorMatchingEdgeTest, NegativeTimestamps) {
    std::vector<BaroSample> samples;
    for (int i = -10; i < 10; ++i) {
        BaroSample baro;
        baro.t = i * 0.1;  // -1.0 to 0.9
        baro.altitude = 100.0;
        samples.push_back(baro);
    }

    // Query at negative time
    auto result = nearestByTime(samples, -0.5, 0.1);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->t, -0.5);
}

// ===== Test Suite: Performance with Large Datasets =====

class SensorMatchingPerformanceTest : public ::testing::Test {
protected:
    std::vector<MagSample> large_dataset;

    void SetUp() override {
        // Create 10,000 samples
        for (int i = 0; i < 10000; ++i) {
            MagSample mag;
            mag.t = i * 0.01;  // 0.0 to 99.99 seconds
            mag.mag = Eigen::Vector3d::Random();
            large_dataset.push_back(mag);
        }
    }
};

TEST_F(SensorMatchingPerformanceTest, FastLookup) {
    // Binary search should be fast even with 10k samples
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < 1000; ++i) {
        double query_time = (i % 10000) * 0.01;
        auto result = nearestByTime(large_dataset, query_time, 0.02);
        EXPECT_TRUE(result.has_value());
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // 1000 queries should complete in <100ms (typically <10ms)
    EXPECT_LT(duration.count(), 100);
}

TEST_F(SensorMatchingPerformanceTest, EdgeLookups) {
    // Test lookups at beginning, middle, end
    auto r1 = nearestByTime(large_dataset, 0.0, 0.01);
    ASSERT_TRUE(r1.has_value());
    EXPECT_DOUBLE_EQ(r1->t, 0.0);

    auto r2 = nearestByTime(large_dataset, 50.0, 0.01);
    ASSERT_TRUE(r2.has_value());
    EXPECT_DOUBLE_EQ(r2->t, 50.0);

    auto r3 = nearestByTime(large_dataset, 99.99, 0.01);
    ASSERT_TRUE(r3.has_value());
    EXPECT_DOUBLE_EQ(r3->t, 99.99);
}

// ===== Test Suite: Data Integrity =====

class SensorMatchingIntegrityTest : public ::testing::Test {};

TEST_F(SensorMatchingIntegrityTest, PreservesDataValues) {
    std::vector<LidarSample> samples;
    for (int i = 0; i < 10; ++i) {
        LidarSample lidar;
        lidar.t = i * 0.1;
        lidar.range_min = i * 1.5;  // Unique value for each sample
        lidar.range_mean = i * 1.6;
        lidar.num_points = 100 + i;
        samples.push_back(lidar);
    }

    // Query for sample at t=0.5
    auto result = nearestByTime(samples, 0.5, 0.05);
    ASSERT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result->t, 0.5);

    // Check that data values are preserved
    EXPECT_DOUBLE_EQ(result->range_min, 5 * 1.5);  // Index 5
    EXPECT_DOUBLE_EQ(result->range_mean, 5 * 1.6);
    EXPECT_EQ(result->num_points, 105);
}

TEST_F(SensorMatchingIntegrityTest, NoModification) {
    std::vector<MagSample> samples;
    for (int i = 0; i < 5; ++i) {
        MagSample mag;
        mag.t = i * 1.0;
        mag.mag = Eigen::Vector3d(i, i+1, i+2);
        samples.push_back(mag);
    }

    // Make a copy for comparison
    std::vector<MagSample> original = samples;

    // Perform multiple queries
    for (double t = 0.0; t < 5.0; t += 0.5) {
        nearestByTime(samples, t, 0.3);
    }

    // Verify original vector is unchanged
    ASSERT_EQ(samples.size(), original.size());
    for (size_t i = 0; i < samples.size(); ++i) {
        EXPECT_DOUBLE_EQ(samples[i].t, original[i].t);
        EXPECT_TRUE(samples[i].mag.isApprox(original[i].mag));
    }
}

// Note: main() is defined in test_factors.cpp
// All tests are automatically discovered by Google Test
