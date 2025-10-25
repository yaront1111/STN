// CHIMERA Navigation ROS2 Node
// GPS-denied multi-sensor navigation with health monitoring

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "core/flight_computer.h"
#include "core/flight_computer_stream.h"
#include "utils/data_loaders.h"

#include <deque>
#include <memory>
#include <string>

using namespace chimera::core;
using namespace chimera::utils;

class ChimeraNavNode : public rclcpp::Node {
public:
  ChimeraNavNode() : Node("chimera_nav") {
    // Declare ROS parameters
    declare_parameter("publish_rate", 50.0);  // Hz
    declare_parameter("frame_id_map", "map");
    declare_parameter("frame_id_odom", "odom");
    declare_parameter("frame_id_base", "base_link");
    declare_parameter("frame_id_imu", "imu_link");

    // Sensor parameters
    declare_parameter("lidar_sigma_validated", 0.05);
    declare_parameter("lidar_sigma_coldstart", 0.2);
    declare_parameter("baro_sigma", 1.5);
    declare_parameter("mag_sigma", 0.1);
    declare_parameter("of_sigma_px", 2.0);

    // Health thresholds
    declare_parameter("lidar_stuck_eps", 0.01);
    declare_parameter("lidar_stuck_spread", 0.005);
    declare_parameter("mag_gate_frac", 0.30);

    // Mode switching
    declare_parameter("baro_takeover_thresh_m", 50.0);

    // Get parameters
    publish_rate_ = get_parameter("publish_rate").as_double();
    frame_map_ = get_parameter("frame_id_map").as_string();
    frame_odom_ = get_parameter("frame_id_odom").as_string();
    frame_base_ = get_parameter("frame_id_base").as_string();
    frame_imu_ = get_parameter("frame_id_imu").as_string();

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/chimera/odometry", rclcpp::QoS(10));

    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/chimera/diagnostics", rclcpp::QoS(10));

    // Subscribers
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&ChimeraNavNode::imuCallback, this, std::placeholders::_1));

    lidar_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/lidar/range", rclcpp::SensorDataQoS(),
      std::bind(&ChimeraNavNode::lidarCallback, this, std::placeholders::_1));

    baro_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
      "/baro/pressure", rclcpp::SensorDataQoS(),
      std::bind(&ChimeraNavNode::baroCallback, this, std::placeholders::_1));

    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      "/mag/field", rclcpp::SensorDataQoS(),
      std::bind(&ChimeraNavNode::magCallback, this, std::placeholders::_1));

    // Timer for publishing
    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
      std::bind(&ChimeraNavNode::publishState, this));

    RCLCPP_INFO(get_logger(), "CHIMERA Navigation Node initialized");
    RCLCPP_INFO(get_logger(), "  Publish rate: %.1f Hz", publish_rate_);
    RCLCPP_INFO(get_logger(), "  Frames: %s -> %s -> %s",
                frame_map_.c_str(), frame_odom_.c_str(), frame_base_.c_str());
  }

private:
  // ROS infrastructure
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Parameters
  double publish_rate_;
  std::string frame_map_, frame_odom_, frame_base_, frame_imu_;

  // Sensor buffers (used during boot initialization)
  std::deque<ImuSample> imu_buffer_;
  std::deque<LidarSample> lidar_buffer_;
  std::deque<BaroSample> baro_buffer_;
  std::deque<MagSample> mag_buffer_;

  // Flight computer
  std::unique_ptr<FlightComputerStream> flight_computer_;
  bool state_initialized_ = false;

  // Callbacks
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    ImuSample sample;
    sample.t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    sample.accel = Eigen::Vector3d(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z);
    sample.gyro = Eigen::Vector3d(
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);

    // Extract quaternion
    sample.quat = Eigen::Quaterniond(
      msg->orientation.w,
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z);

    // If initialized, process directly; otherwise buffer for boot
    if (state_initialized_ && flight_computer_) {
      flight_computer_->processImu(sample);
    } else {
      imu_buffer_.push_back(sample);
      if (imu_buffer_.size() > 1000) imu_buffer_.pop_front();
    }
  }

  void lidarCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
    LidarSample sample;
    sample.t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    sample.range_mean = msg->range;
    sample.range_min = msg->min_range;  // Use min as proxy
    sample.range_max = msg->max_range;
    sample.num_points = 100;  // Dummy value

    if (state_initialized_ && flight_computer_) {
      flight_computer_->processLidar(sample);
    } else {
      lidar_buffer_.push_back(sample);
      if (lidar_buffer_.size() > 100) lidar_buffer_.pop_front();
    }
  }

  void baroCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
    BaroSample sample;
    sample.t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    // Convert pressure to altitude (simplified, assumes sea level)
    // altitude = 44330 * (1 - (P/P0)^0.1903)
    constexpr double P0 = 101325.0;  // Pa at sea level
    sample.altitude = 44330.0 * (1.0 - std::pow(msg->fluid_pressure / P0, 0.1903));

    if (state_initialized_ && flight_computer_) {
      flight_computer_->processBaro(sample);
    } else {
      baro_buffer_.push_back(sample);
      if (baro_buffer_.size() > 100) baro_buffer_.pop_front();
    }
  }

  void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    MagSample sample;
    sample.t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    sample.mag = Eigen::Vector3d(
      msg->magnetic_field.x,
      msg->magnetic_field.y,
      msg->magnetic_field.z);

    if (state_initialized_ && flight_computer_) {
      flight_computer_->processMag(sample);
    } else {
      mag_buffer_.push_back(sample);
      if (mag_buffer_.size() > 100) mag_buffer_.pop_front();
    }
  }

  void publishState() {
    if (!state_initialized_) {
      // Need to accumulate sufficient sensor data before initializing
      if (imu_buffer_.size() < 100) return;

      RCLCPP_INFO(get_logger(), "Initializing CHIMERA estimator with %zu IMU samples...",
                  imu_buffer_.size());

      // Create EstimatorConfig
      EstimatorConfig cfg;
      cfg.g = 9.80665;

      // UKF noise parameters
      cfg.ukf_noise.gyro_noise_density = 1.75e-4;
      cfg.ukf_noise.gyro_random_walk = 2.91e-5;
      cfg.ukf_noise.accel_noise_density = 2.0e-3;
      cfg.ukf_noise.accel_random_walk = 3.0e-3;

      // Sensor sigmas
      cfg.lidar_sigma_validated = get_parameter("lidar_sigma_validated").as_double();
      cfg.lidar_sigma_coldstart = get_parameter("lidar_sigma_coldstart").as_double();
      cfg.baro_sigma = get_parameter("baro_sigma").as_double();
      cfg.mag_sigma = get_parameter("mag_sigma").as_double();
      cfg.of_sigma_px = get_parameter("of_sigma_px").as_double();

      // Health thresholds
      cfg.lidar_stuck_eps = get_parameter("lidar_stuck_eps").as_double();
      cfg.lidar_stuck_spread = get_parameter("lidar_stuck_spread").as_double();
      cfg.mag_gate_frac = get_parameter("mag_gate_frac").as_double();

      // Mode switching
      cfg.baro_takeover_thresh_m = get_parameter("baro_takeover_thresh_m").as_double();

      // Smoother config
      cfg.lag_seconds = 3.0;
      cfg.smoother_hz = 5.0;
      cfg.reanchor_period_s = 3.0;
      cfg.max_gap_s = 0.5;

      // Create FlightComputerStream
      std::string telemetry_path = "";  // Optional: add ROS param for telemetry path
      flight_computer_ = std::make_unique<FlightComputerStream>(cfg, telemetry_path);

      // Convert imu_buffer_ to vector for initialization
      std::vector<ImuSample> boot_imu(imu_buffer_.begin(), imu_buffer_.end());

      // Initialize
      if (!flight_computer_->initialize(boot_imu)) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize CHIMERA estimator!");
        return;
      }

      RCLCPP_INFO(get_logger(), "CHIMERA estimator initialized successfully!");
      state_initialized_ = true;

      // Clear boot buffers (no longer needed)
      imu_buffer_.clear();
      lidar_buffer_.clear();
      baro_buffer_.clear();
      mag_buffer_.clear();

      return;
    }

    // Get current state from flight computer
    if (!flight_computer_ || !flight_computer_->isInitialized()) {
      return;
    }

    gtsam::Pose3 pose = flight_computer_->getPose();
    Eigen::Vector3d vel = flight_computer_->getVelocity();
    Eigen::Matrix<double, 15, 15> cov = flight_computer_->getCovariance();
    double timestamp = flight_computer_->getTimestamp();

    // Publish odometry
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = this->now();
    odom_msg->header.frame_id = frame_odom_;
    odom_msg->child_frame_id = frame_base_;

    // Position (NED to ENU conversion for ROS)
    odom_msg->pose.pose.position.x = pose.y();   // North -> East (swap)
    odom_msg->pose.pose.position.y = pose.x();   // East -> North (swap)
    odom_msg->pose.pose.position.z = -pose.z();  // Down -> Up (flip)

    // Orientation (NED to ENU: rotate 90deg about Z)
    auto quat_ned = pose.rotation().toQuaternion();
    Eigen::Quaterniond q_ned_to_enu(
      Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quat_enu = q_ned_to_enu * quat_ned;

    odom_msg->pose.pose.orientation.w = quat_enu.w();
    odom_msg->pose.pose.orientation.x = quat_enu.x();
    odom_msg->pose.pose.orientation.y = quat_enu.y();
    odom_msg->pose.pose.orientation.z = quat_enu.z();

    // Velocity (NED to ENU)
    odom_msg->twist.twist.linear.x = vel.y();   // North -> East (swap)
    odom_msg->twist.twist.linear.y = vel.x();   // East -> North (swap)
    odom_msg->twist.twist.linear.z = -vel.z();  // Down -> Up (flip)

    // Covariance (15x15 -> 6x6 pose, 6x6 twist)
    // Order in ROS: [x, y, z, rot_x, rot_y, rot_z]
    // Order in UKF: [pos, vel, rot, gyro_bias, accel_bias]

    // Pose covariance (position + rotation)
    // Position: cov.block<3,3>(0,0)
    // Rotation: cov.block<3,3>(6,6)
    std::array<double, 36> pose_cov = {};
    pose_cov[0]  = cov(1,1);  // y (North in NED -> x in ENU)
    pose_cov[7]  = cov(0,0);  // x (East in NED -> y in ENU)
    pose_cov[14] = cov(2,2);  // z
    pose_cov[21] = cov(6,6);  // rot_x
    pose_cov[28] = cov(7,7);  // rot_y
    pose_cov[35] = cov(8,8);  // rot_z
    odom_msg->pose.covariance = pose_cov;

    // Twist covariance (velocity + angular velocity)
    // Velocity: cov.block<3,3>(3,3)
    // Angular vel: approximated from rotation uncertainty
    std::array<double, 36> twist_cov = {};
    twist_cov[0]  = cov(4,4);  // vy (North -> x in ENU)
    twist_cov[7]  = cov(3,3);  // vx (East -> y in ENU)
    twist_cov[14] = cov(5,5);  // vz
    twist_cov[21] = cov(6,6);  // angular velocity uncertainty (approx)
    twist_cov[28] = cov(7,7);
    twist_cov[35] = cov(8,8);
    odom_msg->twist.covariance = twist_cov;

    odom_pub_->publish(std::move(odom_msg));

    // Publish TF transform
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->now();
    tf.header.frame_id = frame_odom_;
    tf.child_frame_id = frame_base_;

    tf.transform.translation.x = pose.y();
    tf.transform.translation.y = pose.x();
    tf.transform.translation.z = -pose.z();

    tf.transform.rotation.w = quat_enu.w();
    tf.transform.rotation.x = quat_enu.x();
    tf.transform.rotation.y = quat_enu.y();
    tf.transform.rotation.z = quat_enu.z();

    tf_broadcaster_->sendTransform(tf);

    // Publish diagnostics
    publishDiagnostics();
  }

  void publishDiagnostics() {
    auto diag_array = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
    diag_array->header.stamp = this->now();

    if (!state_initialized_ || !flight_computer_) {
      // System not initialized - publish basic status
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "chimera_nav: System";
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Not initialized";

      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "imu_buffer_size";
      kv.value = std::to_string(imu_buffer_.size());
      status.values.push_back(kv);

      diag_array->status.push_back(status);
      diag_pub_->publish(std::move(diag_array));
      return;
    }

    // Get diagnostics from flight computer
    DiagnosticsData diag = flight_computer_->getDiagnostics();

    // System status
    diagnostic_msgs::msg::DiagnosticStatus sys_status;
    sys_status.name = "chimera_nav: System";

    if (!diag.watchdog_ok) {
      sys_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      sys_status.message = "Watchdog timeout";
    } else if (diag.baro_takeover) {
      sys_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      sys_status.message = "Baro takeover active";
    } else {
      sys_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      sys_status.message = "Nominal";
    }

    diagnostic_msgs::msg::KeyValue kv;

    kv.key = "altitude_source";
    kv.value = diag.alt_source;
    sys_status.values.push_back(kv);

    kv.key = "baro_takeover";
    kv.value = diag.baro_takeover ? "true" : "false";
    sys_status.values.push_back(kv);

    kv.key = "mode_transitions";
    kv.value = std::to_string(diag.mode_transitions);
    sys_status.values.push_back(kv);

    kv.key = "graph_size";
    kv.value = std::to_string(diag.graph_size);
    sys_status.values.push_back(kv);

    kv.key = "graph_error";
    kv.value = std::to_string(diag.graph_total_error);
    sys_status.values.push_back(kv);

    diag_array->status.push_back(sys_status);

    // Sensor health status
    diagnostic_msgs::msg::DiagnosticStatus sensor_status;
    sensor_status.name = "chimera_nav: Sensors";

    float min_health = std::min({diag.lidar_health, diag.baro_health,
                                  diag.mag_health, diag.imu_health, diag.of_health});

    if (min_health < 0.3f) {
      sensor_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      sensor_status.message = "Sensor degraded";
    } else if (min_health < 0.7f) {
      sensor_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      sensor_status.message = "Sensor health low";
    } else {
      sensor_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      sensor_status.message = "All sensors healthy";
    }

    kv.key = "lidar_health";
    kv.value = std::to_string(diag.lidar_health);
    sensor_status.values.push_back(kv);

    kv.key = "baro_health";
    kv.value = std::to_string(diag.baro_health);
    sensor_status.values.push_back(kv);

    kv.key = "mag_health";
    kv.value = std::to_string(diag.mag_health);
    sensor_status.values.push_back(kv);

    kv.key = "imu_health";
    kv.value = std::to_string(diag.imu_health);
    sensor_status.values.push_back(kv);

    kv.key = "of_health";
    kv.value = std::to_string(diag.of_health);
    sensor_status.values.push_back(kv);

    kv.key = "imu_count";
    kv.value = std::to_string(diag.imu_count);
    sensor_status.values.push_back(kv);

    kv.key = "lidar_count";
    kv.value = std::to_string(diag.lidar_count);
    sensor_status.values.push_back(kv);

    kv.key = "baro_count";
    kv.value = std::to_string(diag.baro_count);
    sensor_status.values.push_back(kv);

    kv.key = "mag_count";
    kv.value = std::to_string(diag.mag_count);
    sensor_status.values.push_back(kv);

    kv.key = "of_count";
    kv.value = std::to_string(diag.of_count);
    sensor_status.values.push_back(kv);

    diag_array->status.push_back(sensor_status);

    // Bias estimation status
    diagnostic_msgs::msg::DiagnosticStatus bias_status;
    bias_status.name = "chimera_nav: Bias Estimates";
    bias_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    bias_status.message = "Converged";

    kv.key = "gyro_bias_deg_per_s";
    kv.value = std::to_string(diag.gyro_bias_deg_per_s);
    bias_status.values.push_back(kv);

    kv.key = "accel_bias_m_per_s2";
    kv.value = std::to_string(diag.accel_bias_m_per_s2);
    bias_status.values.push_back(kv);

    diag_array->status.push_back(bias_status);

    diag_pub_->publish(std::move(diag_array));
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChimeraNavNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
