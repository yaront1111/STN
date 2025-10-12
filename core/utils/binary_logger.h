#pragma once

#include <atomic>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <queue>

#include "core/fusion/state.h"
#include "core/fusion/sr_ukf.h"  // For IMUData

namespace aion {

namespace msgs {
  class LogFrame;
  class LogHeader;
  class IMUMessage;
  class StateMessage;
}

/**
 * Thread-safe binary logger for AION navigation data.
 * Records sensor data and state estimates to protobuf format.
 */
class BinaryLogger {
 public:
  struct Config {
    std::string filepath;
    size_t max_file_size_mb;
    size_t buffer_size;
    bool compress;
    bool enabled;

    Config()
        : filepath("logs/aion.bag"),
          max_file_size_mb(100),
          buffer_size(10000),
          compress(false),
          enabled(true) {}
  };

  explicit BinaryLogger(const Config& config = Config());
  ~BinaryLogger();

  // Start/stop logging
  bool start(const std::string& config_hash = "");
  void stop();
  bool isRunning() const { return is_running_.load(); }

  // Log different message types
  void logIMU(const IMUData& imu);
  void logState(const State& state, const StateCovariance& cov);

  // Flush pending messages to disk
  void flush();

  // Force file rotation
  void rotate();

  // Get statistics
  struct Stats {
    uint64_t imu_count{0};
    uint64_t state_count{0};
    uint64_t total_frames{0};
    size_t bytes_written{0};
    double start_time{0.0};
    double last_time{0.0};
  };
  Stats getStats() const;

 private:
  // Message conversion helpers
  std::unique_ptr<msgs::IMUMessage> createIMUMessage(const IMUData& imu);
  std::unique_ptr<msgs::StateMessage> createStateMessage(
      const State& state, const StateCovariance& cov);

  // Write header at file start
  void writeHeader(const std::string& config_hash);

  // Write a generic frame
  void writeFrame(msgs::LogFrame* frame);

  // Background writer thread
  void writerThread();

  // Check if rotation needed
  bool shouldRotate() const;
  void rotateFile();

  Config config_;
  std::ofstream file_;
  std::string current_filepath_;

  // Thread safety
  mutable std::mutex mutex_;
  std::queue<std::unique_ptr<msgs::LogFrame>> queue_;
  std::condition_variable cv_;

  // Background writer
  std::thread writer_thread_;
  std::atomic<bool> is_running_{false};
  std::atomic<bool> should_stop_{false};

  // Statistics
  mutable std::mutex stats_mutex_;
  Stats stats_;
};

}  // namespace aion