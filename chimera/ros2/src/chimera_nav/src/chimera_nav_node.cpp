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

  // Sensor buffers
  std::deque<ImuSample> imu_buffer_;
  std::deque<LidarSample> lidar_buffer_;
  std::deque<BaroSample> baro_buffer_;
  std::deque<MagSample> mag_buffer_;

  // Latest state
  gtsam::Pose3 latest_pose_;
  Eigen::Vector3d latest_vel_;
  rclcpp::Time latest_timestamp_;
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

    imu_buffer_.push_back(sample);
    if (imu_buffer_.size() > 1000) imu_buffer_.pop_front();
  }

  void lidarCallback(const sensor_msgs::msg::Range::SharedPtr msg) {
    LidarSample sample;
    sample.t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    sample.range_mean = msg->range;
    sample.range_min = msg->min_range;  // Use min as proxy
    sample.num_points = 100;  // Dummy value

    lidar_buffer_.push_back(sample);
    if (lidar_buffer_.size() > 100) lidar_buffer_.pop_front();
  }

  void baroCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
    BaroSample sample;
    sample.t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    // Convert pressure to altitude (simplified, assumes sea level)
    // altitude = 44330 * (1 - (P/P0)^0.1903)
    constexpr double P0 = 101325.0;  // Pa at sea level
    sample.altitude = 44330.0 * (1.0 - std::pow(msg->fluid_pressure / P0, 0.1903));

    baro_buffer_.push_back(sample);
    if (baro_buffer_.size() > 100) baro_buffer_.pop_front();
  }

  void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    MagSample sample;
    sample.t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    sample.mag = Eigen::Vector3d(
      msg->magnetic_field.x,
      msg->magnetic_field.y,
      msg->magnetic_field.z);

    mag_buffer_.push_back(sample);
    if (mag_buffer_.size() > 100) mag_buffer_.pop_front();
  }

  void publishState() {
    if (!state_initialized_) {
      // Need to accumulate sufficient sensor data before initializing
      if (imu_buffer_.size() < 100) return;

      RCLCPP_INFO(get_logger(), "Initializing CHIMERA estimator...");
      // TODO: Initialize FlightComputer with buffered sensor data
      state_initialized_ = true;
      return;
    }

    // Publish odometry
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = latest_timestamp_;
    odom_msg->header.frame_id = frame_odom_;
    odom_msg->child_frame_id = frame_base_;

    // Position (NED to ENU conversion for ROS)
    odom_msg->pose.pose.position.x = latest_pose_.x();   // North -> East
    odom_msg->pose.pose.position.y = -latest_pose_.y();  // East -> North (flip)
    odom_msg->pose.pose.position.z = -latest_pose_.z();  // Down -> Up (flip)

    // Orientation
    auto quat = latest_pose_.rotation().toQuaternion();
    odom_msg->pose.pose.orientation.w = quat.w();
    odom_msg->pose.pose.orientation.x = quat.x();
    odom_msg->pose.pose.orientation.y = quat.y();
    odom_msg->pose.pose.orientation.z = quat.z();

    // Velocity (NED to ENU)
    odom_msg->twist.twist.linear.x = latest_vel_.x();   // North -> East
    odom_msg->twist.twist.linear.y = -latest_vel_.y();  // East -> North (flip)
    odom_msg->twist.twist.linear.z = -latest_vel_.z();  // Down -> Up (flip)

    odom_pub_->publish(std::move(odom_msg));

    // Publish TF transform
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = latest_timestamp_;
    tf.header.frame_id = frame_odom_;
    tf.child_frame_id = frame_base_;

    tf.transform.translation.x = latest_pose_.x();
    tf.transform.translation.y = -latest_pose_.y();
    tf.transform.translation.z = -latest_pose_.z();

    tf.transform.rotation.w = quat.w();
    tf.transform.rotation.x = quat.x();
    tf.transform.rotation.y = quat.y();
    tf.transform.rotation.z = quat.z();

    tf_broadcaster_->sendTransform(tf);

    // Publish diagnostics
    publishDiagnostics();
  }

  void publishDiagnostics() {
    auto diag_array = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
    diag_array->header.stamp = latest_timestamp_;

    // Main system status
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "chimera_nav: System";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Nominal";

    // Add key-value pairs
    diagnostic_msgs::msg::KeyValue kv;

    kv.key = "state_initialized";
    kv.value = state_initialized_ ? "true" : "false";
    status.values.push_back(kv);

    kv.key = "imu_buffer_size";
    kv.value = std::to_string(imu_buffer_.size());
    status.values.push_back(kv);

    kv.key = "lidar_buffer_size";
    kv.value = std::to_string(lidar_buffer_.size());
    status.values.push_back(kv);

    diag_array->status.push_back(status);
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
