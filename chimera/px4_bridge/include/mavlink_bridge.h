// CHIMERA PX4/MAVLink Bridge
// Publishes navigation state to PX4 autopilot via MAVLink protocol
// Implements ODOMETRY, ATTITUDE, HEARTBEAT messages

#pragma once

#include <cstdint>
#include <string>
#include <memory>
#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>

namespace chimera {
namespace px4_bridge {

// MAVLink message IDs (from common.xml and ardupilotmega.xml)
constexpr uint16_t MAVLINK_MSG_ID_HEARTBEAT = 0;
constexpr uint16_t MAVLINK_MSG_ID_ATTITUDE = 30;
constexpr uint16_t MAVLINK_MSG_ID_ODOMETRY = 331;
constexpr uint16_t MAVLINK_MSG_ID_CHIMERA_STATUS = 12_000;  // Custom extension

// System and component IDs
constexpr uint8_t MAV_SYS_ID_CHIMERA = 1;
constexpr uint8_t MAV_COMP_ID_CHIMERA = 191;  // MAV_COMP_ID_ODOMETRY

// MAV_TYPE
constexpr uint8_t MAV_TYPE_GENERIC = 0;

// MAV_STATE
constexpr uint8_t MAV_STATE_ACTIVE = 4;

// MAV_MODE_FLAG
constexpr uint8_t MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1;

// MAV_FRAME
constexpr uint8_t MAV_FRAME_LOCAL_NED = 1;
constexpr uint8_t MAV_FRAME_BODY_FRD = 12;

// Altitude source enumeration (for CHIMERA_STATUS)
enum class AltitudeSource : uint8_t {
  NONE = 0,
  LIDAR = 1,
  BARO = 2,
  BARO_TAKEOVER = 3
};

// CHIMERA navigation state
struct ChimeraState {
  double timestamp_us;              // Microseconds since boot
  gtsam::Pose3 pose;                // NED frame pose
  Eigen::Vector3d velocity;         // NED frame velocity (m/s)
  Eigen::Vector3d angular_velocity; // Body frame (rad/s)

  // Covariance (6x6: position + orientation)
  Eigen::Matrix<double, 6, 6> pose_covariance;
  Eigen::Matrix3d velocity_covariance;

  // Health and mode
  double sensor_health_lidar;
  double sensor_health_baro;
  double sensor_health_mag;
  double sensor_health_imu;
  double sensor_health_of;

  AltitudeSource altitude_source;
  bool watchdog_healthy;
  uint16_t mode_transitions;
};

// MAVLink Bridge Interface
class MavlinkBridge {
 public:
  struct Config {
    std::string serial_port = "/dev/ttyUSB0";
    uint32_t baudrate = 921600;
    uint8_t system_id = MAV_SYS_ID_CHIMERA;
    uint8_t component_id = MAV_COMP_ID_CHIMERA;
    double publish_rate_hz = 50.0;  // ODOMETRY/ATTITUDE rate
    double heartbeat_rate_hz = 1.0;
    bool enable_chimera_status = true;
  };

  explicit MavlinkBridge(const Config& cfg);
  ~MavlinkBridge();

  // Publish navigation state
  void publishState(const ChimeraState& state);

  // Heartbeat (call at 1Hz)
  void publishHeartbeat();

  // Get connection status
  bool isConnected() const;

  // Statistics
  struct Stats {
    uint64_t messages_sent;
    uint64_t messages_failed;
    uint64_t heartbeats_sent;
    double last_publish_time_s;
  };

  Stats getStats() const { return stats_; }

 private:
  Config cfg_;
  Stats stats_;
  int serial_fd_ = -1;
  bool connected_ = false;

  // Internal message builders
  void sendOdometry(const ChimeraState& state);
  void sendAttitude(const ChimeraState& state);
  void sendChimeraStatus(const ChimeraState& state);

  // Low-level MAVLink send
  bool sendMessage(const void* msg_buffer, size_t len);

  // Serial port management
  bool openSerialPort();
  void closeSerialPort();
};

// Helper functions for coordinate conversions
namespace conversions {
  // NED to ENU (ROS/MAVLink standard for some messages)
  inline Eigen::Vector3d ned_to_enu(const Eigen::Vector3d& ned) {
    return Eigen::Vector3d(ned.y(), ned.x(), -ned.z());
  }

  // Quaternion from rotation matrix
  inline Eigen::Quaterniond quat_from_rotation(const gtsam::Rot3& rot) {
    return rot.toQuaternion();
  }

  // Roll, pitch, yaw from rotation
  inline Eigen::Vector3d rpy_from_rotation(const gtsam::Rot3& rot) {
    auto ypr = rot.ypr();
    return Eigen::Vector3d(ypr(2), ypr(1), ypr(0));  // YPR -> RPY
  }
}

}  // namespace px4_bridge
}  // namespace chimera
