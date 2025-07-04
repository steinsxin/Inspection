#!/usr/bin/env python3
import json
import pickle
import time
import socket
import threading
from typing import List, Tuple, Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from autolife_robot_sdk.utils import get_wifi_mac_address, get_mac_from_ip

def _get_device_id() -> str:
    """Determine DEVICE_ID based on device type (NX or other)."""
    device_id = "00000000"
    hostname = socket.gethostname().upper()
    if "NX" in hostname:
        device_id = get_wifi_mac_address("enP8p1s0")
    else:
        device_id = get_mac_from_ip("192.168.10.2")

    return device_id

DEVICE_ID = _get_device_id()
print(f"DEVICE_ID: {DEVICE_ID}")

class ReceiverNode(Node):
    def __init__(self, stop_event=None):
        super().__init__('inspection_receiver_' + DEVICE_ID)

        # battery percent
        self.battery_percent = -1

        # Module status flags
        self.is_imu_live = False
        self.is_battery_live = False
        self.is_rplidar_front_live = False
        self.is_rplidar_back_live = False

        # Timestamps
        self.imu_received_time = 0
        self.battery_received_time = 0
        self.rplidar_front_received_time = 0
        self.rplidar_back_received_time = 0

        # Flags to indicate whether to print data for each module
        self.imu_data_show = False
        self.battery_date_show = False
        self.rplidar_front_data_show = False
        self.rplidar_back_data_show = False

        # Subscriptions
        self.imu_subscription = self.create_subscription(
            String,
            f'gv_imu_{DEVICE_ID}',
            self.imu_callback,
            10
        )

        self.battery_subscription = self.create_subscription(
            String,
            f'gv_battery_{DEVICE_ID}',
            self.battery_callback,
            10
        )

        self.rplidar_back_subscription = self.create_subscription(
            LaserScan,
            f'gv_rear_lidar_{DEVICE_ID}',
            self.rplidar_back_callback,
            10
        )

        self.rplidar_front_subscription = self.create_subscription(
            LaserScan,
            f'gv_front_lidar_{DEVICE_ID}',
            self.rplidar_front_callback,
            10
        )

        # Start status monitor thread
        self.check_thread = threading.Thread(target=self._status_monitor_loop, daemon=True)
        self.check_thread.start()

    def _status_monitor_loop(self, check_interval: float = 1.0, timeout: float = 5.0) -> None:
        """
        Background thread that periodically checks whether sensor modules are still active.

        Args:
            check_interval (float): Time interval (in seconds) between checks.
            timeout (float): Duration (in seconds) after which a module is considered offline if no data is received.
        """
        while rclpy.ok():
            current_time = time.time()

            # Determine if each module is still active based on last received timestamp
            self.is_imu_live = (current_time - self.imu_received_time) < timeout
            self.is_battery_live = (current_time - self.battery_received_time) < timeout
            self.is_rplidar_front_live = (current_time - self.rplidar_front_received_time) < timeout
            self.is_rplidar_back_live = (current_time - self.rplidar_back_received_time) < timeout

            time.sleep(check_interval)

    def imu_callback(self, msg: String) -> None:
        """Callback for IMU data.

        Args:
            msg: Received IMU message.
        """
        self.imu_received_time = time.time()

        if self.imu_data_show:
            self.get_logger().info(f'[IMU] Received data: {msg.data}')

    def battery_callback(self, msg: String) -> None:
        """Callback for battery data.

        Args:
            msg: Received battery message.
        """
        self.battery_received_time = time.time()
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
        self.rplidar_front_received_time = time.time()

        if self.rplidar_back_data_show:
            self.get_logger().info(
                f"[Front Lidar] Received scan data:"
                f"\n\tPoints: {len(msg.ranges)}"
                f"\n\tAngle range: [{msg.angle_min:.2f}, {msg.angle_max:.2f}] radians"
                f"\n\tMin distance: {min(r for r in msg.ranges if r > 0):.2f} meters"
            )

    def status_info(self) -> None:
        """Return the current status of all modules as a formatted string."""
        return (
            f"""
            ----------------------------------------
            | Module Name       | Status           |
            ----------------------------------------
            | IMU Module        | {self.is_imu_live!s:<16} |
            | Battery Module    | {self.is_battery_live!s:<16} |
            | Front Lidar       | {self.is_rplidar_front_live!s:<16} |
            | Rear Lidar        | {self.is_rplidar_back_live!s:<16} |
            ----------------------------------------
            """
        )

def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    rclpy.init(args=args)
    node = ReceiverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down...")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()