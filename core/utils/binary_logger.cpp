#include "core/utils/binary_logger.h"
#include "core/utils/logging.h"
#include "core/utils/time_utils.h"

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <condition_variable>

// Include generated protobuf headers
#include "aion_messages.pb.h"

namespace aion {

BinaryLogger::BinaryLogger(const Config& config) : config_(config) {
  stats_.start_time = TimeUtils::now();
}

BinaryLogger::~BinaryLogger() {
  stop();
}

bool BinaryLogger::start(const std::string& config_hash) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (is_running_) {
    LOG_WARN("BinaryLogger already running");
    return false;
  }

  // Create directory if needed
  std::filesystem::path filepath(config_.filepath);
  std::filesystem::create_directories(filepath.parent_path());

  // Generate timestamped filename
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << filepath.stem().string()
     << "_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
     << filepath.extension().string();

  current_filepath_ = (filepath.parent_path() / ss.str()).string();

  // Open file
  file_.open(current_filepath_, std::ios::binary | std::ios::out);
  if (!file_.is_open()) {
    LOG_ERROR("Failed to open binary log file: {}", current_filepath_);
    return false;
  }

  // Write header
  writeHeader(config_hash);

  // Start writer thread
  is_running_ = true;
  should_stop_ = false;
  writer_thread_ = std::thread(&BinaryLogger::writerThread, this);

  LOG_INFO("BinaryLogger started: {}", current_filepath_);
  return true;
}

void BinaryLogger::stop() {
  if (!is_running_) return;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    should_stop_ = true;
  }
  cv_.notify_all();

  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }

  // Write final statistics
  // TODO: Write SessionStats message

  if (file_.is_open()) {
    file_.close();
  }

  is_running_ = false;
  LOG_INFO("BinaryLogger stopped. Stats: IMU={}, State={}, Total={} frames",
           stats_.imu_count, stats_.state_count, stats_.total_frames);
}

void BinaryLogger::logIMU(const IMUData& imu) {
  if (!is_running_ || !config_.enabled) return;

  auto frame = std::make_unique<msgs::LogFrame>();
  frame->set_timestamp(imu.timestamp);
  frame->set_type(msgs::LogFrame::IMU);

  auto imu_msg = createIMUMessage(imu);
  std::string serialized;
  imu_msg->SerializeToString(&serialized);
  frame->set_data(serialized);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() >= config_.buffer_size) {
      LOG_WARN("BinaryLogger queue full, dropping IMU message");
      return;
    }
    queue_.push(std::move(frame));
  }
  cv_.notify_one();

  // Update stats
  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_.imu_count++;
  stats_.total_frames++;
  stats_.last_time = imu.timestamp;
}

void BinaryLogger::logState(const State& state, const StateCovariance& cov) {
  if (!is_running_ || !config_.enabled) return;

  auto frame = std::make_unique<msgs::LogFrame>();
  frame->set_timestamp(state.time);
  frame->set_type(msgs::LogFrame::STATE);

  auto state_msg = createStateMessage(state, cov);
  std::string serialized;
  state_msg->SerializeToString(&serialized);
  frame->set_data(serialized);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.size() >= config_.buffer_size) {
      LOG_WARN("BinaryLogger queue full, dropping state message");
      return;
    }
    queue_.push(std::move(frame));
  }
  cv_.notify_one();

  // Update stats
  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_.state_count++;
  stats_.total_frames++;
  stats_.last_time = state.time;
}

void BinaryLogger::flush() {
  if (!is_running_) return;

  // Wait for queue to empty
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait(lock, [this] { return queue_.empty(); });

  if (file_.is_open()) {
    file_.flush();
  }
}

void BinaryLogger::rotate() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (file_.is_open()) {
    rotateFile();
  }
}

BinaryLogger::Stats BinaryLogger::getStats() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

std::unique_ptr<msgs::IMUMessage> BinaryLogger::createIMUMessage(const IMUData& imu) {
  auto msg = std::make_unique<msgs::IMUMessage>();
  msg->set_timestamp(imu.timestamp);
  msg->set_gyro_x(imu.angular_velocity.x());
  msg->set_gyro_y(imu.angular_velocity.y());
  msg->set_gyro_z(imu.angular_velocity.z());
  msg->set_accel_x(imu.acceleration.x());
  msg->set_accel_y(imu.acceleration.y());
  msg->set_accel_z(imu.acceleration.z());
  msg->set_temperature(imu.temperature);
  return msg;
}

std::unique_ptr<msgs::StateMessage> BinaryLogger::createStateMessage(
    const State& state, const StateCovariance& cov) {
  auto msg = std::make_unique<msgs::StateMessage>();
  msg->set_timestamp(state.time);

  // Position
  msg->set_pos_n(state.position.x());
  msg->set_pos_e(state.position.y());
  msg->set_pos_d(state.position.z());

  // Velocity
  msg->set_vel_n(state.velocity.x());
  msg->set_vel_e(state.velocity.y());
  msg->set_vel_d(state.velocity.z());

  // Quaternion
  msg->set_quat_w(state.quaternion.w());
  msg->set_quat_x(state.quaternion.x());
  msg->set_quat_y(state.quaternion.y());
  msg->set_quat_z(state.quaternion.z());

  // Biases
  msg->set_gyro_bias_x(state.gyro_bias.x());
  msg->set_gyro_bias_y(state.gyro_bias.y());
  msg->set_gyro_bias_z(state.gyro_bias.z());
  msg->set_accel_bias_x(state.accel_bias.x());
  msg->set_accel_bias_y(state.accel_bias.y());
  msg->set_accel_bias_z(state.accel_bias.z());

  // Wind
  msg->set_wind_n(state.wind.x());
  msg->set_wind_e(state.wind.y());

  // Diagonal covariance
  const auto& P = cov.matrix();
  for (int i = 0; i < 17; ++i) {
    msg->add_covariance_diagonal(P(i, i));
  }

  return msg;
}

void BinaryLogger::writeHeader(const std::string& config_hash) {
  msgs::LogHeader header;
  header.set_version(1);
  header.set_start_timestamp(TimeUtils::now());
  header.set_config_hash(config_hash);
  header.set_frame_id("NED");

  // TODO: Get origin from config
  header.set_origin_lat(32.0853);
  header.set_origin_lon(34.7818);
  header.set_origin_alt(20.0);

  // Add sensor list
  header.add_sensor_list("IMU");
  header.add_sensor_list("STATE");

  // Wrap in frame
  msgs::LogFrame frame;
  frame.set_timestamp(header.start_timestamp());
  frame.set_type(msgs::LogFrame::HEADER);

  std::string serialized;
  header.SerializeToString(&serialized);
  frame.set_data(serialized);

  writeFrame(&frame);
}

void BinaryLogger::writeFrame(msgs::LogFrame* frame) {
  if (!file_.is_open()) return;

  // Write size prefix (4 bytes)
  uint32_t size = static_cast<uint32_t>(frame->ByteSizeLong());
  file_.write(reinterpret_cast<const char*>(&size), sizeof(size));

  // Write frame
  std::string buffer;
  frame->SerializeToString(&buffer);
  file_.write(buffer.data(), buffer.size());

  stats_.bytes_written += sizeof(size) + buffer.size();
}

void BinaryLogger::writerThread() {
  LOG_DEBUG("BinaryLogger writer thread started");

  while (!should_stop_ || !queue_.empty()) {
    std::unique_lock<std::mutex> lock(mutex_);

    // Wait for messages or stop signal
    cv_.wait(lock, [this] { return !queue_.empty() || should_stop_; });

    // Process all pending messages
    while (!queue_.empty()) {
      auto frame = std::move(queue_.front());
      queue_.pop();
      lock.unlock();

      writeFrame(frame.get());

      // Check if rotation needed
      if (shouldRotate()) {
        lock.lock();
        rotateFile();
        lock.unlock();
      }

      lock.lock();
    }
  }

  LOG_DEBUG("BinaryLogger writer thread stopped");
}

bool BinaryLogger::shouldRotate() const {
  if (config_.max_file_size_mb == 0) return false;

  size_t max_bytes = config_.max_file_size_mb * 1024 * 1024;
  return stats_.bytes_written >= max_bytes;
}

void BinaryLogger::rotateFile() {
  if (!file_.is_open()) return;

  file_.close();

  // Generate new filename
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::filesystem::path(config_.filepath).stem().string()
     << "_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
     << std::filesystem::path(config_.filepath).extension().string();

  std::filesystem::path new_path =
      std::filesystem::path(config_.filepath).parent_path() / ss.str();
  current_filepath_ = new_path.string();

  file_.open(current_filepath_, std::ios::binary | std::ios::out);
  if (file_.is_open()) {
    writeHeader("");  // Write header to new file
    stats_.bytes_written = 0;  // Reset byte counter
    LOG_INFO("Rotated log file to: {}", current_filepath_);
  } else {
    LOG_ERROR("Failed to open new log file: {}", current_filepath_);
  }
}

}  // namespace aion