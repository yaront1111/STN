#include <gtest/gtest.h>
#include <thread>
#include <vector>

#include "core/utils/time_utils.h"
#include "core/utils/logging.h"

TEST(TimeUtils, MonotonicTime) {
  aion::TimeUtils::initialize();

  // Get multiple timestamps
  std::vector<double> timestamps;
  for (int i = 0; i < 100; ++i) {
    timestamps.push_back(aion::TimeUtils::now());
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  // Check strict monotonicity
  for (size_t i = 1; i < timestamps.size(); ++i) {
    EXPECT_GT(timestamps[i], timestamps[i-1])
      << "Timestamp not strictly increasing at index " << i;
  }
}

TEST(TimeUtils, BoundedLatency) {
  aion::TimeUtils::initialize();

  // Create simulated sensor timestamps
  for (int i = 0; i < 100; ++i) {
    double t_meas = aion::TimeUtils::now();

    // Simulate small processing delay
    std::this_thread::sleep_for(std::chrono::microseconds(500));

    auto ts = aion::Timestamp::createNow(t_meas);

    // Check latency is bounded
    EXPECT_GE(ts.latency(), 0.0) << "Negative latency detected";
    EXPECT_LE(ts.latency(), 0.001) << "Latency exceeds 1ms bound";

    // Validate timestamp
    EXPECT_TRUE(ts.isValid(0.01)) << "Timestamp validation failed";
  }
}

TEST(TimeUtils, TimestampValidation) {
  // Initialize only if not already done
  aion::TimeUtils::initialize();

  double now = aion::TimeUtils::now();

  // Valid timestamp
  aion::Timestamp valid(now - 0.001, now);
  EXPECT_TRUE(valid.isValid(0.1));

  // Future timestamp (invalid)
  aion::Timestamp future(now + 1.0, now);
  EXPECT_FALSE(future.isValid(0.1));

  // Too old timestamp
  aion::Timestamp old(now - 10.0, now);
  EXPECT_FALSE(old.isValid(0.1));

  // Negative latency (invalid)
  aion::Timestamp negative(now, now - 0.1);
  EXPECT_FALSE(negative.isValid(0.1));
}

TEST(RateController, MaintainsRate) {
  const double target_rate = 100.0;  // 100Hz
  aion::RateController controller(target_rate);

  auto start = std::chrono::steady_clock::now();

  // Run for 100 cycles
  for (int i = 0; i < 100; ++i) {
    controller.sleep();
  }

  auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start).count();

  // Should take approximately 1 second
  EXPECT_NEAR(elapsed, 1.0, 0.1);

  // Check actual rate
  auto stats = controller.getStats();
  EXPECT_NEAR(stats.actual_rate_hz, target_rate, 5.0);  // Within 5Hz
  EXPECT_TRUE(controller.isHealthy(0.9));  // At least 90% of target

  // Check timing consistency
  EXPECT_LE(stats.max_dt - stats.min_dt, 0.005);  // Max jitter 5ms
}

TEST(RateController, HandlesOverruns) {
  const double target_rate = 1000.0;  // 1000Hz (1ms period)
  aion::RateController controller(target_rate);

  // Run with occasional delays
  for (int i = 0; i < 100; ++i) {
    if (i % 20 == 0) {
      // Simulate occasional processing spike
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    controller.sleep();
  }

  auto stats = controller.getStats();

  // Should have some overruns
  EXPECT_GT(stats.overruns, 0);

  // But should still maintain reasonable average rate
  EXPECT_GT(stats.actual_rate_hz, target_rate * 0.5);
}

int main(int argc, char** argv) {
  // Initialize logger to avoid hangs on log messages
  aion::Logger::init("/tmp/test_time_utils.log", spdlog::level::off);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}