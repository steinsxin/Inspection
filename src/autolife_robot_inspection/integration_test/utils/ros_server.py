#!/usr/bin/env python3
import json
import pickle
import time
from typing import List, Tuple, Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from autolife_robot_sdk.utils import get_wifi_mac_address

DEVICE_ID = get_wifi_mac_address("enP8p1s0").replace(":", "")
print(DEVICE_ID)

class ServerSubscriber(Node):
    def __init__(self):
        """Initialize the ROS2 server subscriber node."""
        super().__init__('server_subscriber' + DEVICE_ID)

        # Module status flags
        self.imu = False
        self.imu_received_time = 0
        self.is_battery_live = False
        self.rplidar_front = False
        self.rplidar_back = False
        self.rplidar_front_received_time = 0
        self.rplidar_back_received_time = 0

        # Module data display flags and storage
        self.imu_data_show = False
        self.battery_date_show = False
        self.battery_percent = -1
        self.rplidar_front_data_show = False
        self.rplidar_back_data_show = False

        # IMU data subscription
        self.imu_subscription = self.create_subscription(
            String,
            'gv_imu_' + DEVICE_ID,
            self.imu_callback,
            10
        )

        # Battery data subscription
        self.battery_subscription = self.create_subscription(
            String,
            'gv_battery_' + DEVICE_ID,
            self.battery_callback,
            10
        )

        # Rear lidar data subscription
        self.rplidar_back_subscription = self.create_subscription(
            LaserScan,
            'gv_rear_lidar_' + DEVICE_ID,
            self.rplidar_back_callback,
            10
        )

        # Front lidar data subscription
        self.rplidar_front_subscription = self.create_subscription(
            LaserScan,
            'gv_front_lidar_' + DEVICE_ID,
            self.rplidar_front_callback,
            10
        )

        self.publisher = self.create_publisher(String, "control_topic_" + DEVICE_ID, 10)
        self.control_reset_pub = self.create_publisher(String, "control_reset_" + DEVICE_ID, 10)

    def control_reset(self) -> None:
        """Publish a control reset message."""
        msg = String()
        json_msg = {"type": "control_reset", "status": "async"}
        msg.data = json.dumps(json_msg)
        self.control_reset_pub.publish(msg)

    def replay(self, filepath: str) -> None:
        """Replay messages from a recorded pickle file.

        Args:
            filepath: Path to the pickle file containing recorded messages.
        """
        msg_list = self.load_pickle(filepath)
        if msg_list:
            self.replay_messages(msg_list)

    def load_pickle(self, filepath: str) -> List[Tuple[float, str]]:
        """Load message data from a pickle file.

        Args:
            filepath: Path to the pickle file.

        Returns:
            List of tuples containing (timestamp, message_data).
        """
        try:
            with open(filepath, 'rb') as f:
                data = pickle.load(f)
                self.get_logger().info(f"Successfully loaded recording: {filepath}")
                return data
        except Exception as e:
            self.get_logger().error(f"Failed to load recording: {e}")
            return []

    def replay_messages(self, msg_list: List[Tuple[float, str]]) -> None:
        """Replay messages with original timing.

        Args:
            msg_list: List of tuples containing (timestamp, message_data).
        """
        prev_time = 0.0
        start_time = time.time()

        self.get_logger().info("Arm Test Start")
        for index, item in enumerate(msg_list):
            current_time, msg_data = item

            # Calculate wait time
            wait_time = current_time if index == 0 else current_time - prev_time

            wait_time_start = time.time()
            while time.time() - wait_time_start < wait_time:
                time.sleep(0.001)

            # Publish message
            msg = String()
            try:
                command = json.loads(msg_data)
                command['ts'] = time.time() * 1000
                msg.data = json.dumps(command)
                self.publisher.publish(msg)
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Failed to decode JSON: {e}")
                continue

            prev_time = current_time

        self.get_logger().info("Replay completed")

    def imu_callback(self, msg: String) -> None:
        """Callback for IMU data.

        Args:
            msg: Received IMU message.
        """
        self.imu = True
        self.imu_received_time = time.time()

        if self.imu_data_show:
            self.get_logger().info(f'[IMU] Received data: {msg.data}')

    def battery_callback(self, msg: String) -> None:
        """Callback for battery data.

        Args:
            msg: Received battery message.
        """
        self.is_battery_live = True
        bat = json.loads(msg.data)
        self.battery_percent = bat['capacity_percentage']

        if self.battery_date_show:
            try:
                status = "Charging" if bat["current"] > 0 else "Discharging"
                
                self.get_logger().info(
                    f"""
                    ┌──────────────────────────────────────────────┐
                    │ 🔋 Battery System Status │ Status: {status:<8}│ Health: {bat['nominal_capacity']/50*100:.0f}% │
                    ├──────────────┬──────────────┬───────────────┤
                    │  Voltage      │  Current     │  Temperature  │
                    │  {bat['voltage']:<5.1f} V   │ {bat['current']:<+5.2f} A   │ {bat['temperature1']:<4.1f} °C  │
                    ├──────────────┼──────────────┼───────────────┤
                    │  Current Cap  │  Total Cap   │  Remaining    │
                    │  {bat['remaining_capacity']:<5.2f} Ah │ {bat['nominal_capacity']:<5.1f} Ah │ {bat['capacity_percentage']:>6} %   │
                    └──────────────┴──────────────┴───────────────┘
                    """
                )
            except Exception as e:
                self.get_logger().error(f"Battery data parsing error: {e}\nRaw data: {msg.data}")

    def rplidar_back_callback(self, msg: LaserScan) -> None:
        """Callback for rear lidar data.

        Args:
            msg: Received LaserScan message from rear lidar.
        """
        self.rplidar_back = True
        self.rplidar_back_received_time = time.time()

        if self.rplidar_front_data_show:
            self.get_logger().info(
                f"[Rear Lidar] Received scan data:"
                f"\n\tPoints: {len(msg.ranges)}"
                f"\n\tAngle range: [{msg.angle_min:.2f}, {msg.angle_max:.2f}] radians"
                f"\n\tMin distance: {min(r for r in msg.ranges if r > 0):.2f} meters"
            )

    def rplidar_front_callback(self, msg: LaserScan) -> None:
        """Callback for front lidar data.

        Args:
            msg: Received LaserScan message from front lidar.
        """
        self.rplidar_front = True
        self.rplidar_front_received_time = time.time()

        if self.rplidar_back_data_show:
            self.get_logger().info(
                f"[Front Lidar] Received scan data:"
                f"\n\tPoints: {len(msg.ranges)}"
                f"\n\tAngle range: [{msg.angle_min:.2f}, {msg.angle_max:.2f}] radians"
                f"\n\tMin distance: {min(r for r in msg.ranges if r > 0):.2f} meters"
            )

    def status_info(self) -> None:
        """Log the current status of all modules."""
        self.get_logger().info(
            f"""
            ----------------------------------------
            | Module Name       | Status           |
            ----------------------------------------
            | IMU Module        | {self.imu:<16} |
            | Battery Module    | {self.is_battery_live:<16} |
            | Front Lidar       | {self.rplidar_front:<16} |
            | Rear Lidar        | {self.rplidar_back:<16} |
            ----------------------------------------
            """
        )


def main(args=None):
    """Main function to initialize and run the ROS2 node."""
    rclpy.init(args=args)
    node = ServerSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()