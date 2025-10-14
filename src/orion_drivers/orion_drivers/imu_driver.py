#!/usr/bin/env python3
"""
IMU Driver Node for ORION Navigation System
Wraps IMU sensor interface and publishes sensor_msgs/Imu
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from orion_interfaces.msg import SensorHealth
from std_msgs.msg import Header
import numpy as np


class IMUDriver(Node):
    """
    IMU Driver Node

    Subscribes to raw IMU data (or interfaces with hardware)
    Publishes:
        - /imu/data (sensor_msgs/Imu)
        - /imu/health (orion_interfaces/SensorHealth)
    """

    def __init__(self):
        super().__init__('imu_driver')

        # Declare parameters
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('health_topic', '/imu/health')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 400.0)  # Hz

        # Get parameters
        self.imu_topic = self.get_parameter('imu_topic').value
        self.health_topic = self.get_parameter('health_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Publishers
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.health_pub = self.create_publisher(SensorHealth, self.health_topic, 10)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        # Health monitoring
        self.message_count = 0
        self.last_health_time = self.get_clock().now()

        self.get_logger().info(f'IMU Driver initialized')
        self.get_logger().info(f'  Publishing to: {self.imu_topic}')
        self.get_logger().info(f'  Health topic: {self.health_topic}')
        self.get_logger().info(f'  Rate: {self.publish_rate} Hz')

    def timer_callback(self):
        """
        Publish IMU data at specified rate

        NOTE: This is a placeholder implementation.
        In production, this would interface with actual hardware
        via serial, I2C, or hardware driver library.
        """
        # Create IMU message
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        # TODO: Replace with actual sensor readings
        # For now, publish identity orientation and zero motion
        imu_msg.orientation.w = 1.0
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81  # Gravity

        # Set covariances (placeholder values)
        # In production, these come from sensor calibration
        imu_msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        imu_msg.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]
        imu_msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        # Publish
        self.imu_pub.publish(imu_msg)
        self.message_count += 1

        # Publish health status every second
        now = self.get_clock().now()
        if (now - self.last_health_time).nanoseconds > 1e9:  # 1 second
            self.publish_health()
            self.last_health_time = now

    def publish_health(self):
        """Publish sensor health status"""
        health_msg = SensorHealth()
        health_msg.header = Header()
        health_msg.header.stamp = self.get_clock().now().to_msg()

        health_msg.sensor_name = 'imu'
        health_msg.sensor_type = 'imu'
        health_msg.health_status = SensorHealth.HEALTH_OK

        health_msg.update_rate_hz = float(self.message_count)
        health_msg.expected_rate_hz = self.publish_rate
        health_msg.data_quality = 1.0  # Perfect (placeholder)
        health_msg.dropped_messages = 0
        health_msg.last_update_time = self.get_clock().now().to_msg()

        health_msg.status_message = 'Operating normally'

        self.health_pub.publish(health_msg)

        # Reset counter
        self.message_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = IMUDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
