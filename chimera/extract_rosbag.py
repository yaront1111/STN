#!/usr/bin/env python3
"""Extract sensor data from ROS bag using rosbags library"""

from rosbags.rosbag1 import Reader
from rosbags.serde import deserialize_cdr
import json
import sys
from pathlib import Path


def extract_imu_data(bagfile, output_file, max_messages=None):
    """Extract IMU data from ROS bag"""

    messages = []
    msg_count = 0

    print(f"Reading bag file: {bagfile}")

    with Reader(Path(bagfile)) as reader:
        # Print available topics
        print(f"\nAvailable topics:")
        for connection in reader.connections:
            print(f"  {connection.topic}: {connection.msgtype}")

        # Find IMU topics
        imu_topics = [conn.topic for conn in reader.connections
                      if 'imu' in conn.topic.lower() and 'data' in conn.topic.lower()]

        if not imu_topics:
            print("\nNo IMU data topics found!")
            return []

        print(f"\nExtracting from topics: {imu_topics}")

        # Extract messages
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic not in imu_topics:
                continue

            if max_messages and msg_count >= max_messages:
                break

            # Deserialize message
            msg = deserialize_cdr(rawdata, connection.msgtype)

            # Extract IMU data
            try:
                imu_msg = {
                    't': timestamp / 1e9,  # Convert to seconds
                    'gyro': [
                        msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z
                    ],
                    'accel': [
                        msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z
                    ],
                    'quat': [
                        msg.orientation.w,
                        msg.orientation.x,
                        msg.orientation.y,
                        msg.orientation.z
                    ]
                }
                messages.append(imu_msg)
                msg_count += 1

                if msg_count % 1000 == 0:
                    print(f"  Extracted {msg_count} messages...")

            except AttributeError as e:
                print(f"Warning: Could not parse message: {e}")
                continue

    print(f"\nTotal messages extracted: {len(messages)}")

    if messages:
        with open(output_file, 'w') as f:
            json.dump(messages, f, indent=2)
        print(f"Saved to: {output_file}")

    return messages


def main():
    if len(sys.argv) < 3:
        print("Usage: python3 extract_rosbag.py <bagfile> <output.json> [max_messages]")
        sys.exit(1)

    bagfile = sys.argv[1]
    output = sys.argv[2]
    max_msgs = int(sys.argv[3]) if len(sys.argv) > 3 else None

    extract_imu_data(bagfile, output, max_msgs)


if __name__ == '__main__':
    main()
