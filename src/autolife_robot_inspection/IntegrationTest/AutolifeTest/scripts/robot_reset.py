import os
import sys
import json
import time
import traceback
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException
from autolife_robot_sdk.utils import get_wifi_mac_address

DEVICE_ID = get_wifi_mac_address("enP8p1s0").replace(":", "")
print(DEVICE_ID)

class ServerSubscriber(Node):
    def __init__(self):
        super().__init__("server_subscriber_" + DEVICE_ID)
        self.control_reset_pub = self.create_publisher(
            String, "control_reset_" + DEVICE_ID, 10
        )

    def control_reset(self):
        msg = String()
        json_msg = {"type": "control_reset", "status": "async"}
        msg.data = json.dumps(json_msg)
        self.control_reset_pub.publish(msg)


def main(args: Optional[list] = None) -> None:
    """
    Initialize and run the ROS2 node with reset control.
    
    Args:
        args: Command line arguments passed to ROS2 initialization.
    """
    rclpy.init(args=args)
    node = ServerSubscriber()

    print("Robot reset started")
    node.control_reset()

    # node.control_reset()执行后程序停止
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Program terminated by interrupt")
    except Exception as e:
        print(f"Unexpected error during execution: {e}")
        traceback.print_exc()