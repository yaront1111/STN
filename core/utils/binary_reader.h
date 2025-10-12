#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "core/fusion/state.h"
#include "core/fusion/sr_ukf.h"

namespace aion {

namespace msgs {
  class LogFrame;
  class LogHeader;
  class IMUMessage;
  class StateMessage;
  class SessionStats;
}

/**
 * Binary log reader for AION navigation data.
 * Reads protobuf-formatted logs for replay and analysis.
 */
class BinaryReader {
 public:
  BinaryReader();
  ~BinaryReader();

  // Open a log file for reading
  bool open(const std::string& filepath);
  void close();
  bool isOpen() const { return file_.is_open(); }

  // Get header information
  const msgs::LogHeader* getHeader() const { return header_.get(); }

  // Sequential reading
  std::unique_ptr<msgs::LogFrame> readNext();
  bool hasNext() const;
  void reset();  // Seek to beginning (after header)

  // Time-based seeking
  bool seekToTime(double timestamp);

  // Get all messages of a specific type
  std::vector<std::unique_ptr<msgs::LogFrame>> getMessagesByType(
      int type);  // Uses int to match protobuf enum

  // Statistics
  struct Stats {
    uint64_t total_frames{0};
    uint64_t imu_count{0};
    uint64_t state_count{0};
    double min_timestamp{0.0};
    double max_timestamp{0.0};
    double duration{0.0};
  };
  Stats computeStats();

  // Convert protobuf messages to AION types
  static IMUData parseIMUMessage(const msgs::IMUMessage& msg);
  static void parseStateMessage(const msgs::StateMessage& msg,
                                State& state,
                                StateCovariance& cov);

 private:
  // Read next frame from current position
  std::unique_ptr<msgs::LogFrame> readFrameInternal();

  // Read header at file start
  bool readHeader();

  std::ifstream file_;
  std::string filepath_;
  std::unique_ptr<msgs::LogHeader> header_;

  // Current file position (for sequential reading)
  std::streampos data_start_pos_;  // Position after header
  bool at_end_{false};
};

}  // namespace aion