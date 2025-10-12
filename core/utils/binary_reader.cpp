#include "core/utils/binary_reader.h"
#include "core/utils/logging.h"

#include <limits>

// Include generated protobuf headers
#include "aion_messages.pb.h"

namespace aion {

BinaryReader::BinaryReader() = default;

BinaryReader::~BinaryReader() {
  close();
}

bool BinaryReader::open(const std::string& filepath) {
  close();

  filepath_ = filepath;
  file_.open(filepath_, std::ios::binary | std::ios::in);

  if (!file_.is_open()) {
    LOG_ERROR("Failed to open binary log file: {}", filepath_);
    return false;
  }

  // Read and validate header
  if (!readHeader()) {
    LOG_ERROR("Failed to read header from: {}", filepath_);
    close();
    return false;
  }

  // Mark data start position
  data_start_pos_ = file_.tellg();
  at_end_ = false;

  LOG_INFO("Opened binary log: {} (version {})",
           filepath_, header_->version());
  return true;
}

void BinaryReader::close() {
  if (file_.is_open()) {
    file_.close();
  }
  header_.reset();
  at_end_ = false;
}

std::unique_ptr<msgs::LogFrame> BinaryReader::readNext() {
  if (!file_.is_open() || at_end_) {
    return nullptr;
  }

  return readFrameInternal();
}

bool BinaryReader::hasNext() const {
  if (!file_.is_open() || at_end_) {
    return false;
  }

  // Check if we can read size prefix
  std::streampos current = const_cast<std::ifstream&>(file_).tellg();
  uint32_t size;
  const_cast<std::ifstream&>(file_).read(
      reinterpret_cast<char*>(&size), sizeof(size));

  bool has_more = const_cast<std::ifstream&>(file_).good();

  // Restore position
  const_cast<std::ifstream&>(file_).seekg(current);

  return has_more;
}

void BinaryReader::reset() {
  if (!file_.is_open()) return;

  file_.clear();
  file_.seekg(data_start_pos_);
  at_end_ = false;
}

bool BinaryReader::seekToTime(double timestamp) {
  if (!file_.is_open()) return false;

  reset();

  // Linear search for now (could optimize with index)
  while (hasNext()) {
    auto frame = readNext();
    if (!frame) break;

    if (frame->timestamp() >= timestamp) {
      // Rewind to before this frame
      std::streampos current = file_.tellg();
      uint32_t size = static_cast<uint32_t>(frame->ByteSizeLong()) + sizeof(uint32_t);
      file_.seekg(current - static_cast<std::streamoff>(size));
      return true;
    }
  }

  return false;
}

std::vector<std::unique_ptr<msgs::LogFrame>>
BinaryReader::getMessagesByType(int type) {
  std::vector<std::unique_ptr<msgs::LogFrame>> messages;

  if (!file_.is_open()) return messages;

  // Save current position
  std::streampos current = file_.tellg();

  reset();

  while (hasNext()) {
    auto frame = readNext();
    if (!frame) break;

    if (frame->type() == type) {
      messages.push_back(std::move(frame));
    }
  }

  // Restore position
  file_.clear();
  file_.seekg(current);
  at_end_ = false;

  return messages;
}

BinaryReader::Stats BinaryReader::computeStats() {
  Stats stats;

  if (!file_.is_open()) return stats;

  // Save current position
  std::streampos current = file_.tellg();

  reset();

  stats.min_timestamp = std::numeric_limits<double>::max();
  stats.max_timestamp = std::numeric_limits<double>::lowest();

  while (hasNext()) {
    auto frame = readNext();
    if (!frame) break;

    stats.total_frames++;

    switch (frame->type()) {
      case msgs::LogFrame::IMU:
        stats.imu_count++;
        break;
      case msgs::LogFrame::STATE:
        stats.state_count++;
        break;
      default:
        break;
    }

    if (frame->timestamp() < stats.min_timestamp) {
      stats.min_timestamp = frame->timestamp();
    }
    if (frame->timestamp() > stats.max_timestamp) {
      stats.max_timestamp = frame->timestamp();
    }
  }

  stats.duration = stats.max_timestamp - stats.min_timestamp;

  // Restore position
  file_.clear();
  file_.seekg(current);
  at_end_ = false;

  return stats;
}

IMUData BinaryReader::parseIMUMessage(const msgs::IMUMessage& msg) {
  IMUData imu;
  imu.timestamp = msg.timestamp();
  imu.angular_velocity = Eigen::Vector3d(
      msg.gyro_x(), msg.gyro_y(), msg.gyro_z());
  imu.acceleration = Eigen::Vector3d(
      msg.accel_x(), msg.accel_y(), msg.accel_z());
  imu.temperature = msg.temperature();
  return imu;
}

void BinaryReader::parseStateMessage(const msgs::StateMessage& msg,
                                     State& state,
                                     StateCovariance& cov) {
  state.time = msg.timestamp();

  // Position
  state.position = Eigen::Vector3d(
      msg.pos_n(), msg.pos_e(), msg.pos_d());

  // Velocity
  state.velocity = Eigen::Vector3d(
      msg.vel_n(), msg.vel_e(), msg.vel_d());

  // Quaternion
  state.quaternion = Eigen::Quaterniond(
      msg.quat_w(), msg.quat_x(), msg.quat_y(), msg.quat_z());

  // Biases
  state.gyro_bias = Eigen::Vector3d(
      msg.gyro_bias_x(), msg.gyro_bias_y(), msg.gyro_bias_z());
  state.accel_bias = Eigen::Vector3d(
      msg.accel_bias_x(), msg.accel_bias_y(), msg.accel_bias_z());

  // Wind
  state.wind = Eigen::Vector2d(
      msg.wind_n(), msg.wind_e());

  // Covariance diagonal
  if (msg.covariance_diagonal_size() >= 17) {
    auto& P = cov.matrix();
    for (int i = 0; i < 17; ++i) {
      P(i, i) = msg.covariance_diagonal(i);
    }
  }
}

std::unique_ptr<msgs::LogFrame> BinaryReader::readFrameInternal() {
  // Read size prefix
  uint32_t size;
  file_.read(reinterpret_cast<char*>(&size), sizeof(size));

  if (!file_.good() || size == 0 || size > 10 * 1024 * 1024) {  // Max 10MB
    at_end_ = true;
    return nullptr;
  }

  // Read frame data
  std::vector<char> buffer(size);
  file_.read(buffer.data(), size);

  if (!file_.good()) {
    at_end_ = true;
    return nullptr;
  }

  // Parse frame
  auto frame = std::make_unique<msgs::LogFrame>();
  if (!frame->ParseFromArray(buffer.data(), size)) {
    LOG_ERROR("Failed to parse LogFrame");
    return nullptr;
  }

  return frame;
}

bool BinaryReader::readHeader() {
  auto frame = readFrameInternal();
  if (!frame || frame->type() != msgs::LogFrame::HEADER) {
    return false;
  }

  header_ = std::make_unique<msgs::LogHeader>();
  if (!header_->ParseFromString(frame->data())) {
    LOG_ERROR("Failed to parse LogHeader");
    header_.reset();
    return false;
  }

  return true;
}

}  // namespace aion