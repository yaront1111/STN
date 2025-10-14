#!/usr/bin/env python3
"""
LiDAR Driver Node for ORION Navigation System
Wraps LiDAR sensor interface and publishes sensor_msgs/PointCloud2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from orion_interfaces.msg import SensorHealth
from std_msgs.msg import Header
import numpy as np
import struct


class LiDARDriver(Node):
    """
    LiDAR Driver Node

    Interfaces with Livox Mid-360 LiDAR (or other sensors)
    Publishes:
        - /lidar/points (sensor_msgs/PointCloud2)
        - /lidar/health (orion_interfaces/SensorHealth)
    """

    def __init__(self):
        super().__init__('lidar_driver')

        # Declare parameters
        self.declare_parameter('lidar_topic', '/lidar/points')
        self.declare_parameter('health_topic', '/lidar/health')
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # Get parameters
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.health_topic = self.get_parameter('health_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Publishers
        self.lidar_pub = self.create_publisher(PointCloud2, self.lidar_topic, 10)
        self.health_pub = self.create_publisher(SensorHealth, self.health_topic, 10)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        # Health monitoring
        self.message_count = 0
        self.last_health_time = self.get_clock().now()

        self.get_logger().info(f'LiDAR Driver initialized')
        self.get_logger().info(f'  Publishing to: {self.lidar_topic}')
        self.get_logger().info(f'  Health topic: {self.health_topic}')
        self.get_logger().info(f'  Rate: {self.publish_rate} Hz')

    def create_dummy_pointcloud(self):
        """
        Create a dummy point cloud for testing

        NOTE: This is placeholder data.
        In production, this would come from the actual LiDAR sensor
        via livox_ros_driver2 or similar hardware driver.
        """
        # Generate random points in a 10m radius
        num_points = 1000
        points = []

        for _ in range(num_points):
            # Random spherical coordinates
            r = np.random.uniform(1.0, 10.0)
            theta = np.random.uniform(0, 2 * np.pi)
            phi = np.random.uniform(0, np.pi)

            # Convert to Cartesian
            x = r * np.sin(phi) * np.cos(theta)
            y = r * np.sin(phi) * np.sin(theta)
            z = r * np.cos(phi)

            intensity = np.random.uniform(0, 255)

            points.append([x, y, z, intensity])

        return np.array(points, dtype=np.float32)

    def create_pointcloud2_msg(self, points):
        """Convert numpy array to PointCloud2 message"""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.height = 1
        msg.width = len(points)

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 16  # 4 fields * 4 bytes each
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        msg.data = points.tobytes()

        return msg

    def timer_callback(self):
        """Publish LiDAR data at specified rate"""
        # Create and publish point cloud
        points = self.create_dummy_pointcloud()
        cloud_msg = self.create_pointcloud2_msg(points)
        self.lidar_pub.publish(cloud_msg)

        self.message_count += 1

        # Publish health status every second
        now = self.get_clock().now()
        if (now - self.last_health_time).nanoseconds > 1e9:
            self.publish_health()
            self.last_health_time = now

    def publish_health(self):
        """Publish sensor health status"""
        health_msg = SensorHealth()
        health_msg.header = Header()
        health_msg.header.stamp = self.get_clock().now().to_msg()

        health_msg.sensor_name = 'lidar'
        health_msg.sensor_type = 'lidar'
        health_msg.health_status = SensorHealth.HEALTH_OK

        health_msg.update_rate_hz = float(self.message_count)
        health_msg.expected_rate_hz = self.publish_rate
        health_msg.data_quality = 1.0
        health_msg.dropped_messages = 0
        health_msg.last_update_time = self.get_clock().now().to_msg()

        health_msg.status_message = 'Operating normally'

        self.health_pub.publish(health_msg)

        # Reset counter
        self.message_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = LiDARDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
