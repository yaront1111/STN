#!/usr/bin/env python3
"""
Confidence Estimator for ORION Navigation System
Analyzes EKF covariance and sensor health to estimate navigation confidence
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from orion_interfaces.msg import ConfidenceMetrics, NavigationState, SensorHealth
from std_msgs.msg import Header
import numpy as np


class ConfidenceEstimator(Node):
    """
    Confidence Estimator Node

    Subscribes to:
        - /odom/filtered (robot_localization EKF output)
        - /imu/health, /lidar/health (sensor health)

    Publishes:
        - /nav/confidence (ConfidenceMetrics)
        - /nav/state (NavigationState)
    """

    def __init__(self):
        super().__init__('confidence_estimator')

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom/filtered',
            self.odom_callback,
            10
        )

        self.imu_health_sub = self.create_subscription(
            SensorHealth,
            '/imu/health',
            lambda msg: self.health_callback(msg, 'imu'),
            10
        )

        self.lidar_health_sub = self.create_subscription(
            SensorHealth,
            '/lidar/health',
            lambda msg: self.health_callback(msg, 'lidar'),
            10
        )

        # Publishers
        self.confidence_pub = self.create_publisher(
            ConfidenceMetrics,
            '/nav/confidence',
            10
        )

        self.state_pub = self.create_publisher(
            NavigationState,
            '/nav/state',
            10
        )

        # State
        self.latest_odom = None
        self.sensor_health = {}

        self.get_logger().info('Confidence Estimator initialized')

    def health_callback(self, msg, sensor_name):
        """Store sensor health"""
        self.sensor_health[sensor_name] = msg

    def odom_callback(self, msg):
        """Estimate confidence from odometry covariance"""
        self.latest_odom = msg

        # Extract covariance
        pose_cov = np.array(msg.pose.covariance).reshape(6, 6)
        twist_cov = np.array(msg.twist.covariance).reshape(6, 6)

        # Calculate uncertainties (1-sigma)
        position_uncertainty = np.sqrt(np.trace(pose_cov[:3, :3]) / 3.0)
        velocity_uncertainty = np.sqrt(np.trace(twist_cov[:3, :3]) / 3.0)
        attitude_uncertainty = np.sqrt(np.trace(pose_cov[3:, 3:]) / 3.0)

        # Convert to confidence (0-1 scale)
        # Simple inverse mapping (can be improved with calibration)
        position_confidence = max(0.0, 1.0 - min(1.0, position_uncertainty / 5.0))
        velocity_confidence = max(0.0, 1.0 - min(1.0, velocity_uncertainty / 2.0))
        attitude_confidence = max(0.0, 1.0 - min(1.0, attitude_uncertainty / 0.5))

        # Overall confidence (weighted average)
        overall_confidence = (
            0.5 * position_confidence +
            0.3 * velocity_confidence +
            0.2 * attitude_confidence
        )

        # Publish confidence metrics
        conf_msg = ConfidenceMetrics()
        conf_msg.header = Header()
        conf_msg.header.stamp = self.get_clock().now().to_msg()

        conf_msg.overall_confidence = overall_confidence
        conf_msg.position_xy_confidence = position_confidence
        conf_msg.position_z_confidence = position_confidence
        conf_msg.velocity_confidence = velocity_confidence
        conf_msg.attitude_confidence = attitude_confidence

        conf_msg.position_uncertainty_m = position_uncertainty
        conf_msg.velocity_uncertainty_ms = velocity_uncertainty
        conf_msg.attitude_uncertainty_deg = np.degrees(attitude_uncertainty)

        # Placeholder values for other metrics
        conf_msg.imu_contribution = 0.5
        conf_msg.lidar_contribution = 0.5
        conf_msg.feature_richness = 0.8
        conf_msg.temporal_smoothness = 0.9

        self.confidence_pub.publish(conf_msg)

        # Publish navigation state
        nav_msg = NavigationState()
        nav_msg.header = msg.header
        nav_msg.pose = msg.pose
        nav_msg.twist = msg.twist

        # Determine status based on confidence
        if overall_confidence > 0.8:
            nav_msg.status = NavigationState.NAV_STATUS_CONVERGED
        elif overall_confidence > 0.5:
            nav_msg.status = NavigationState.NAV_STATUS_DEGRADED
        else:
            nav_msg.status = NavigationState.NAV_STATUS_LOST

        nav_msg.overall_confidence = overall_confidence
        nav_msg.position_confidence = position_confidence
        nav_msg.velocity_confidence = velocity_confidence
        nav_msg.attitude_confidence = attitude_confidence

        # Active sensors (bitmask)
        nav_msg.active_sensors = NavigationState.SENSOR_IMU | NavigationState.SENSOR_LIDAR

        nav_msg.estimated_drift = position_uncertainty  # Placeholder
        nav_msg.time_since_global_fix = 999.0  # No global fix yet in Phase 1

        self.state_pub.publish(nav_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConfidenceEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
