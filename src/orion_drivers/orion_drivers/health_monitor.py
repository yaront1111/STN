#!/usr/bin/env python3
"""
Sensor Health Monitor for ORION Navigation System
Aggregates health status from all sensors and publishes system-level diagnostics
"""

import rclpy
from rclpy.node import Node
from orion_interfaces.msg import SensorHealth
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class HealthMonitor(Node):
    """
    Health Monitor Node

    Subscribes to individual sensor health topics
    Publishes aggregated system health diagnostics
    """

    def __init__(self):
        super().__init__('health_monitor')

        # Declare parameters
        self.declare_parameter('sensors', ['imu', 'lidar'])
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter('publish_rate', 1.0)  # Hz

        # Get parameters
        self.sensors = self.get_parameter('sensors').value
        self.diagnostics_topic = self.get_parameter('diagnostics_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Sensor health status dictionary
        self.sensor_health = {}
        for sensor in self.sensors:
            self.sensor_health[sensor] = None

        # Subscribers for each sensor
        self.health_subs = []
        for sensor in self.sensors:
            topic = f'/{sensor}/health'
            sub = self.create_subscription(
                SensorHealth,
                topic,
                lambda msg, s=sensor: self.health_callback(msg, s),
                10
            )
            self.health_subs.append(sub)
            self.get_logger().info(f'Subscribing to: {topic}')

        # Publisher for aggregated diagnostics
        self.diag_pub = self.create_publisher(
            DiagnosticArray,
            self.diagnostics_topic,
            10
        )

        # Timer for publishing aggregated status
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

        self.get_logger().info('Health Monitor initialized')

    def health_callback(self, msg, sensor_name):
        """Store latest health status for each sensor"""
        self.sensor_health[sensor_name] = msg

    def timer_callback(self):
        """Publish aggregated diagnostics"""
        diag_array = DiagnosticArray()
        diag_array.header = Header()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # Create diagnostic status for each sensor
        for sensor_name, health in self.sensor_health.items():
            status = DiagnosticStatus()
            status.name = f'orion_sensors_{sensor_name}'
            status.hardware_id = sensor_name

            if health is None:
                # No data received
                status.level = DiagnosticStatus.ERROR
                status.message = 'No health data received'
            else:
                # Map health status to diagnostic level
                if health.health_status == SensorHealth.HEALTH_OK:
                    status.level = DiagnosticStatus.OK
                    status.message = 'Operating normally'
                elif health.health_status == SensorHealth.HEALTH_WARN:
                    status.level = DiagnosticStatus.WARN
                    status.message = health.status_message
                elif health.health_status in [SensorHealth.HEALTH_ERROR,
                                               SensorHealth.HEALTH_STALE,
                                               SensorHealth.HEALTH_DISCONNECTED]:
                    status.level = DiagnosticStatus.ERROR
                    status.message = health.status_message

                # Add key-value pairs with details
                status.values.append(
                    KeyValue(key='update_rate', value=f'{health.update_rate_hz:.1f} Hz')
                )
                status.values.append(
                    KeyValue(key='expected_rate', value=f'{health.expected_rate_hz:.1f} Hz')
                )
                status.values.append(
                    KeyValue(key='data_quality', value=f'{health.data_quality:.2f}')
                )
                status.values.append(
                    KeyValue(key='dropped_messages', value=str(health.dropped_messages))
                )

            diag_array.status.append(status)

        # Publish
        self.diag_pub.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
