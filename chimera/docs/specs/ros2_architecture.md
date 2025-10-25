# CHIMERA ROS2 Node - Detailed Architecture

**Version:** 1.0.0
**Target:** ROS2 Humble/Iron
**Package:** `chimera_nav`

---

## Package Structure

```
chimera_nav/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── chimera.launch.py          # Main launch file
│   ├── sensors.launch.py          # Sensor drivers
│   ├── rviz.launch.py             # Visualization
│   └── sim.launch.py              # Gazebo simulation
├── config/
│   ├── params.yaml                # Main parameters
│   ├── sensors/
│   │   ├── imu.yaml
│   │   ├── lidar.yaml
│   │   ├── baro.yaml
│   │   ├── mag.yaml
│   │   └── optical_flow.yaml
│   └── rviz/
│       └── chimera.rviz
├── msg/
│   ├── SystemHealth.msg           # Custom health message
│   └── OpticalFlow.msg            # Custom OF message
├── srv/
│   └── ResetState.srv             # Service to reset state
├── src/
│   ├── chimera_node.cpp
│   ├── sensor_adapters/
│   │   ├── imu_adapter.cpp
│   │   ├── lidar_adapter.cpp
│   │   ├── baro_adapter.cpp
│   │   ├── mag_adapter.cpp
│   │   └── optical_flow_adapter.cpp
│   └── publishers/
│       ├── odometry_publisher.cpp
│       └── diagnostics_publisher.cpp
├── include/chimera_nav/
│   ├── chimera_node.hpp
│   └── sensor_adapters/
│       └── *.hpp
└── README.md
```

---

## Node Architecture

### Lifecycle States

```
UNCONFIGURED
    ↓ [configure]
INACTIVE
    ↓ [activate]
ACTIVE  ←→  [deactivate]
    ↓ [cleanup]
FINALIZED
```

### Main Class

```cpp
class ChimeraNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit ChimeraNode(const rclcpp::NodeOptions& options);
  
  // Lifecycle callbacks
  CallbackReturn on_configure(const State&) override;
  CallbackReturn on_activate(const State&) override;
  CallbackReturn on_deactivate(const State&) override;
  CallbackReturn on_cleanup(const State&) override;
  CallbackReturn on_shutdown(const State&) override;
  
 private:
  // Core
  std::unique_ptr<chimera::core::FlightComputer> flight_computer_;
  
  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp_lifecycle::LifecyclePublisher<chimera_msgs::msg::SystemHealth>::SharedPtr health_pub_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<chimera_msgs::msg::OpticalFlow>::SharedPtr of_sub_;
  
  // TF
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  
  // Callbacks
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void lidarCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  void publishOdometry();
  void publishDiagnostics();
  void broadcastTF();
};
```

---

## Topic Specifications

### Published Topics

#### /chimera/odometry (nav_msgs/Odometry)

**QoS:** RELIABLE, depth=10
**Rate:** 100 Hz

```cpp
void publishOdometry() {
  auto msg = nav_msgs::msg::Odometry();
  
  msg.header.stamp = this->now();
  msg.header.frame_id = odom_frame_;
  msg.child_frame_id = base_frame_;
  
  // Get state from flight computer
  auto state = flight_computer_->getState();
  
  // Position
  msg.pose.pose.position.x = state.pose.x();
  msg.pose.pose.position.y = state.pose.y();
  msg.pose.pose.position.z = state.pose.z();
  
  // Orientation (quaternion)
  Eigen::Quaterniond q(state.pose.rotation().matrix());
  msg.pose.pose.orientation.w = q.w();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  
  // Velocity
  msg.twist.twist.linear.x = state.velocity.x();
  msg.twist.twist.linear.y = state.velocity.y();
  msg.twist.twist.linear.z = state.velocity.z();
  
  // Covariance (6x6 flattened)
  auto cov = state.getCovariance();
  for (int i = 0; i < 36; ++i) {
    msg.pose.covariance[i] = cov(i/6, i%6);
    msg.twist.covariance[i] = cov(i/6, i%6);  // Same for velocity
  }
  
  odom_pub_->publish(msg);
}
```

#### /chimera/health (chimera_msgs/SystemHealth)

**QoS:** RELIABLE, depth=1
**Rate:** 1 Hz

```msg
# SystemHealth.msg
std_msgs/Header header

SensorHealth lidar
SensorHealth baro
SensorHealth mag
SensorHealth of
SensorHealth imu

string altitude_source  # "lidar", "baro", "baro_takeover"
bool contract_ok
```

```msg
# SensorHealth.msg (nested)
float32 score           # [0,1]
string reason           # Human-readable status
```

---

## TF Broadcasting

### Dynamic Transforms

```cpp
void broadcastTF() {
  geometry_msgs::msg::TransformStamped transform;
  
  transform.header.stamp = this->now();
  transform.header.frame_id = odom_frame_;
  transform.child_frame_id = base_frame_;
  
  auto state = flight_computer_->getState();
  
  // Position
  transform.transform.translation.x = state.pose.x();
  transform.transform.translation.y = state.pose.y();
  transform.transform.translation.z = state.pose.z();
  
  // Rotation
  Eigen::Quaterniond q(state.pose.rotation().matrix());
  transform.transform.rotation.w = q.w();
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  
  tf_broadcaster_->sendTransform(transform);
}
```

### Static Transforms (from Calibration)

```cpp
void publishStaticTF() {
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  
  // base_link → imu_link
  transforms.push_back(createStaticTransform("base_link", "imu_link", 
                                              T_body_imu_));
  
  // base_link → lidar_link
  transforms.push_back(createStaticTransform("base_link", "lidar_link", 
                                              T_body_lidar_));
  
  // base_link → camera_link
  transforms.push_back(createStaticTransform("base_link", "camera_link", 
                                              T_body_camera_));
  
  static_tf_broadcaster_->sendTransform(transforms);
}
```

---

## Launch File

### chimera.launch.py

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value='config/params.yaml'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        
        LifecycleNode(
            package='chimera_nav',
            executable='chimera_node',
            name='chimera',
            namespace='',
            parameters=[LaunchConfiguration('config')],
            output='screen',
            emulate_tty=True
        )
    ])
```

---

## nav2 Integration

### Configuration

```yaml
# nav2_params.yaml
bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: odom
    robot_base_frame: base_link
    odom_topic: /chimera/odometry

controller_server:
  ros__parameters:
    odom_topic: /chimera/odometry
    controller_frequency: 20.0
    
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    
costmap:
  global_frame: odom
  robot_base_frame: base_link
```

---

## Services

### ResetState.srv

```srv
# Request
geometry_msgs/Pose initial_pose
geometry_msgs/Twist initial_velocity
---
# Response
bool success
string message
```

**Implementation:**

```cpp
void resetStateCallback(
    const chimera_msgs::srv::ResetState::Request::SharedPtr request,
    chimera_msgs::srv::ResetState::Response::SharedPtr response) {
  
  try {
    flight_computer_->resetPoseVelocity(request->initial_pose, 
                                         request->initial_velocity);
    response->success = true;
    response->message = "State reset successful";
  } catch (const std::exception& e) {
    response->success = false;
    response->message = e.what();
  }
}
```

---

## Build Configuration

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(chimera_nav)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(chimera_core REQUIRED)  # CHIMERA core library

# Executable
add_executable(chimera_node
  src/chimera_node.cpp
  src/sensor_adapters/imu_adapter.cpp
  src/sensor_adapters/lidar_adapter.cpp
  # ...
)

ament_target_dependencies(chimera_node
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  sensor_msgs
  diagnostic_msgs
  tf2_ros
)

target_link_libraries(chimera_node chimera_core)

install(TARGETS chimera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
```

---

## Testing

### Unit Tests

```cpp
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "chimera_nav/chimera_node.hpp"

TEST(ChimeraNodeTest, Lifecycle) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<ChimeraNode>(rclcpp::NodeOptions());
  
  // Test state transitions
  auto state = node->configure();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  
  state = node->activate();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  
  rclcpp::shutdown();
}
```

---

**See Also:**
- ROS2 Lifecycle: https://design.ros2.org/articles/node_lifecycle.html
- nav2 Integration: https://navigation.ros.org/
