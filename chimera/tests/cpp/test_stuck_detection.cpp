// Unit tests for LiDAR stuck detection dual-gate logic
// Tests both variance and range spread thresholds

#include <gtest/gtest.h>
#include "core/sensor_health.h"
#include "utils/data_loaders.h"

using namespace chimera::core;
using namespace chimera::utils;

class StuckDetectionTest : public ::testing::Test {
protected:
    LidarHealthMonitor::Config cfg;

    void SetUp() override {
        cfg.stuck_eps = 0.01;  // 10mm stddev threshold
        cfg.stuck_range_spread = 0.005;  // 5mm range spread
        cfg.stuck_window = 8;
    }
};

TEST_F(StuckDetectionTest, TrulyStuckSensor_IdenticalReadings) {
    // Simulate truly stuck sensor: all readings identical
    LidarHealthMonitor monitor(cfg);

    for (int i = 0; i < 8; ++i) {
        LidarSample sample;
        sample.t = i * 0.1;
        sample.range_mean = 1.586923;  // Frozen value
        sample.range_min = 1.5;
        sample.num_points = 100;

        auto health = monitor.evaluate(sample);

        if (i >= 7) {  // Full window
            EXPECT_TRUE(health.is_stuck) << "Truly stuck sensor should be detected";
            EXPECT_EQ(health.score, 0.0);
        }
    }
}

TEST_F(StuckDetectionTest, StableHovering_NotStuck) {
    // Real flight data scenario: hovering at ~1.6m with normal 2cm noise
    LidarHealthMonitor monitor(cfg);

    // Realistic hovering measurements (from actual flight data)
    std::vector<double> ranges = {
        1.593, 1.621, 1.621, 1.577, 1.574,
        1.619, 1.573, 1.588
    };

    for (size_t i = 0; i < ranges.size(); ++i) {
        LidarSample sample;
        sample.t = i * 0.1;
        sample.range_mean = ranges[i];
        sample.range_min = ranges[i] - 0.01;
        sample.num_points = 135;

        auto health = monitor.evaluate(sample);

        if (i >= 7) {  // Full window
            EXPECT_FALSE(health.is_stuck) << "Stable hovering should NOT trigger stuck detection";
            EXPECT_GT(health.score, 0.0);
        }
    }
}

TEST_F(StuckDetectionTest, LowVariance_ButHighSpread_NotStuck) {
    // Edge case: Low variance but measurements have spread
    // (e.g., slow linear drift: 1.50, 1.51, 1.52, ...)
    LidarHealthMonitor monitor(cfg);

    for (int i = 0; i < 8; ++i) {
        LidarSample sample;
        sample.t = i * 0.1;
        sample.range_mean = 1.5 + i * 0.001;  // Slow drift, low variance but spread > 5mm
        sample.range_min = sample.range_mean - 0.01;
        sample.num_points = 100;

        auto health = monitor.evaluate(sample);

        if (i >= 7) {  // Full window, spread = 7mm > 5mm
            EXPECT_FALSE(health.is_stuck) << "Low variance with spread should NOT be stuck";
        }
    }
}

TEST_F(StuckDetectionTest, HighVariance_LowSpread_NotStuck) {
    // Edge case: Measurements tightly clustered but with outlier
    LidarHealthMonitor monitor(cfg);

    std::vector<double> ranges = {
        1.600, 1.601, 1.602, 1.601, 1.600,
        1.601, 1.602, 1.650  // Outlier boosts variance
    };

    for (size_t i = 0; i < ranges.size(); ++i) {
        LidarSample sample;
        sample.t = i * 0.1;
        sample.range_mean = ranges[i];
        sample.range_min = ranges[i] - 0.01;
        sample.num_points = 100;

        auto health = monitor.evaluate(sample);

        if (i >= 7) {  // High variance due to outlier
            EXPECT_FALSE(health.is_stuck) << "High variance should NOT be stuck";
        }
    }
}

TEST_F(StuckDetectionTest, NearThreshold_LowVariance_LowSpread_Stuck) {
    // Right at the threshold: very low variance AND very low spread
    LidarHealthMonitor monitor(cfg);

    // Measurements within 4mm range (< 5mm), very low noise
    std::vector<double> ranges = {
        1.5980, 1.5985, 1.5990, 1.5995,
        1.6000, 1.6005, 1.6010, 1.6015
    };

    for (size_t i = 0; i < ranges.size(); ++i) {
        LidarSample sample;
        sample.t = i * 0.1;
        sample.range_mean = ranges[i];
        sample.range_min = ranges[i] - 0.001;
        sample.num_points = 100;

        auto health = monitor.evaluate(sample);

        if (i >= 7) {  // Spread = 3.5mm < 5mm, variance very low
            EXPECT_TRUE(health.is_stuck) << "Both gates should trigger stuck";
            EXPECT_EQ(health.score, 0.0);
        }
    }
}

TEST_F(StuckDetectionTest, NormalFlight_VaryingAltitudes_NotStuck) {
    // Normal flight with varying altitudes
    LidarHealthMonitor monitor(cfg);

    std::vector<double> ranges = {
        1.5, 1.8, 2.1, 2.5, 3.0, 3.5, 4.0, 4.5
    };

    for (size_t i = 0; i < ranges.size(); ++i) {
        LidarSample sample;
        sample.t = i * 0.1;
        sample.range_mean = ranges[i];
        sample.range_min = ranges[i] - 0.05;
        sample.num_points = 120;

        auto health = monitor.evaluate(sample);

        if (i >= 7) {  // Large spread and variance
            EXPECT_FALSE(health.is_stuck) << "Normal flight should NOT be stuck";
            EXPECT_GT(health.score, 0.0);
        }
    }
}

TEST_F(StuckDetectionTest, Reset_ClearsWindow) {
    // Verify reset clears the detection window
    LidarHealthMonitor monitor(cfg);

    // Fill with stuck readings
    for (int i = 0; i < 8; ++i) {
        LidarSample sample;
        sample.t = i * 0.1;
        sample.range_mean = 1.5;
        sample.range_min = 1.4;
        sample.num_points = 100;
        monitor.evaluate(sample);
    }

    // Reset
    monitor.reset();

    // Add normal reading - should not be stuck (window not full)
    LidarSample sample;
    sample.t = 1.0;
    sample.range_mean = 2.0;
    sample.range_min = 1.9;
    sample.num_points = 100;
    auto health = monitor.evaluate(sample);

    EXPECT_FALSE(health.is_stuck) << "After reset, stuck should not trigger until window refills";
}

// Test real-world thresholds match specification
TEST_F(StuckDetectionTest, ThresholdValues_MatchSpecification) {
    EXPECT_NEAR(cfg.stuck_eps, 0.01, 1e-6) << "Stddev threshold should be 10mm";
    EXPECT_NEAR(cfg.stuck_range_spread, 0.005, 1e-6) << "Range spread threshold should be 5mm";
    EXPECT_EQ(cfg.stuck_window, 8) << "Window size should be 8 samples";
}
