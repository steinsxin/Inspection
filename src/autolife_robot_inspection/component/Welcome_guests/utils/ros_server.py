import traceback
import pickle
import time
import json
from typing import List, Tuple, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from autolife_robot_sdk.utils import get_wifi_mac_address


DEVICE_ID = get_wifi_mac_address("enP8p1s0").replace(":", "")
print(DEVICE_ID)

class RobotTargetEEPoseDemo(Node):
    """
    A ROS2 node to demonstrate robot target end-effector pose control.
    """

    def __init__(self):
        """
        Initialize the RobotTargetEEPoseDemo node.
        """
        super().__init__('robot_target_ee_pose_demo')
        self.robot_target_ee_pose_pub = self.create_publisher(String, f'/robot_target_ee_pose{DEVICE_ID}', 10)
        self.publisher = self.create_publisher(String, "control_topic_" + DEVICE_ID, 10)
        self.control_reset_pub = self.create_publisher(String, "control_reset_" + DEVICE_ID, 10)

        # Initialize joint angles to zero
        self.target_left_arm_joints_deg = [0] * 11
        self.target_right_arm_joints_deg = [0] * 11

        self.robot_state = 0

        self.get_logger().info("Robot Target End-Effector Pose Demo Initialized")
        self.publish_joint_angles()

    def publish_joint_angles(self):
        """
        Publish the current joint angles.
        """
        message = String()
        message.data = json.dumps({
            "l": self.target_left_arm_joints_deg,
            "r": self.target_right_arm_joints_deg,
            "s": self.robot_state
        })
        self.robot_target_ee_pose_pub.publish(message)

    def set_joint_angles(self, joint_index: int, left_angle: float, right_angle: float):
        """
        Set the target angles for a specific joint.

        :param joint_index: The index of the joint (0-10).
        :param left_angle: The target angle for the left arm joint.
        :param right_angle: The target angle for the right arm joint.
        """
        if 0 <= joint_index < 11:
            self.target_left_arm_joints_deg[joint_index] = left_angle
            self.target_right_arm_joints_deg[joint_index] = right_angle
            self.publish_joint_angles()
            self.get_logger().info(f"Joint {joint_index + 1} set to left: {left_angle}°, right: {right_angle}°")
        else:
            self.get_logger().error(f"Invalid joint index: {joint_index}")

    def reset_joint_angles(self):
        """
        Reset all joint angles to zero.
        """
        self.target_left_arm_joints_deg = [0] * 11
        self.target_right_arm_joints_deg = [0] * 11
        self.publish_joint_angles()
        self.get_logger().info("All joints reset to home position")

    def bow_salute(self):
        """
        Run the robot target end-effector pose demo.
        """
        self.get_logger().info("Running Robot Target End-Effector Pose Demo")
        time.sleep(2)

        # 手臂抬前
        self.set_joint_angles(4, 50, 0)

        # 手臂旋转
        self.set_joint_angles(6, -70, 0)

        # 手臂收回
        self.set_joint_angles(7, 110, 0)

        # 手腕转动
        self.set_joint_angles(8, 40, 0)
        time.sleep(12)

        # 鞠躬
        self.set_joint_angles(2, 30, 0)
        time.sleep(5)

        # 重置关节角度
        self.reset_joint_angles()
        time.sleep(10)

        self.get_logger().info("Finished Demo")

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


def main() -> None:
    """
    Main function to run the RobotTargetEEPoseDemo node.
    """
    rclpy.init()
    robot_target_ee_pose_demo = RobotTargetEEPoseDemo()

    try:
        robot_target_ee_pose_demo.bow_salute()
        # time.sleep(1)
        # robot_target_ee_pose_demo.replay("../pkl/command_hello.pkl")
    except KeyboardInterrupt:
        robot_target_ee_pose_demo.get_logger().info("Quit by interrupt")
    except Exception as e:
        robot_target_ee_pose_demo.get_logger().error(f"Unexpected exception during playback: {e}")
        traceback.print_exc()
    finally:
        robot_target_ee_pose_demo.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()