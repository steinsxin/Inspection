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
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

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

class PublisherNode(Node):
    def __init__(self, stop_event=None):
        super().__init__('inspection_publisher_' + DEVICE_ID)

        # Publishers
        self.control_topic_pub = self.create_publisher(
            String,
            f'control_topic_{DEVICE_ID}',
            10
        )
        self.control_reset_pub = self.create_publisher(
            String,
            f'control_reset_{DEVICE_ID}',
            10
        )
        self.robot_target_ee_pose_pub = self.create_publisher(
            String, 
            f'robot_target_ee_pose{DEVICE_ID}',
            10
        )
        self.gv_target_cmd_vel_pub = self.create_publisher(
            Twist, 
            f'gv_target_cmd_vel_{DEVICE_ID}',
            10
        )

    def set_enable_gv_cmd_vel(self):
        return self.set_remote_parameter_once(
            f'arm_service_move_{DEVICE_ID}', 
            'enable_gv_target_cmd_vel', 
            False
        )

    def set_remote_parameter_once(self, target_node_name: str, param_name: str, param_value: bool) -> bool:
        """仅尝试一次设置远程参数，不阻塞等待"""
        client = self.create_client(SetParameters, f'/{target_node_name}/set_parameters')

        if not client.service_is_ready():
            return False

        param = Parameter(param_name, Parameter.Type.BOOL, param_value)
        request = SetParameters.Request()
        request.parameters = [param.to_parameter_msg()]

        future = client.call_async(request)

        def callback(fut):
            try:
                result = fut.result()
            except Exception as e:
                print(f"Exception in callback: {e}")

        future.add_done_callback(callback)
        return True


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
                self.control_topic_pub.publish(msg)
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Failed to decode JSON: {e}")
                continue

            prev_time = current_time

        self.get_logger().info("Replay completed")

def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    rclpy.init(args=args)
    node = PublisherNode()

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