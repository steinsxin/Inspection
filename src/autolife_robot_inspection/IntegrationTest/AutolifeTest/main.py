#!/usr/bin/env python3
import os
import sys
import time
import threading
import traceback
from typing import Dict

import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from autolife_robot_sdk.hardware.hw_api import HWAPI
from utils import PiperVoice, ServerSubscriber


def check_task(node: ServerSubscriber) -> None:
    """Check and test all hardware modules.

    Args:
        node: The ServerSubscriber node instance.
    """
    time.sleep(5)
    hw_api = HWAPI()

    current_dir = os.path.dirname(os.path.abspath(__file__))
    model_file = os.path.join(current_dir, "MediumOnnx", "zh_CN-huayan-medium.onnx")
    piper_voice = PiperVoice(hw_api, model_file)
    
    # Initialize hardware modules
    hw_api.initialize([
        'mod_microphone_a311', 
        'mod_speaker_a311',
        'mod_camera_head_left', 
        'mod_camera_head_right', 
        'mod_camera_head_rear',
        'mod_camera_hand_left', 
        'mod_camera_hand_right',
    ])

    hw_api.speaker_set_volume(100)

    node.status_info()
    piper_voice.play_voice("欢迎使用测试程序")
    piper_voice.play_voice("语音模块就绪")

    # Create directory for saving test images
    save_dir = "test_images"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    # Camera names and their corresponding file paths
    camera_names = {
        "head_left": os.path.join(save_dir, "head_left.jpg"),
        "head_right": os.path.join(save_dir, "head_right.jpg"),
        "head_rear": os.path.join(save_dir, "head_rear.jpg"),
        "hand_left": os.path.join(save_dir, "hand_left.jpg"),
        "hand_right": os.path.join(save_dir, "hand_right.jpg"),
    }

    # Chinese names for cameras
    camera_chinese_names = {
        "head_left": "头部左摄像头",
        "head_right": "头部右摄像头",
        "head_rear": "头部后摄像头",
        "hand_left": "左手摄像头",
        "hand_right": "右手摄像头",
    }

    time.sleep(2)
    print(f"测试开始")

    vision_status: Dict[str, bool] = {
        camera_chinese_names[camera_name]: False 
        for camera_name in camera_names
    }

    # Camera initialization and testing
    try:
        for camera_name, file_path in camera_names.items():
            # Get camera image
            image = hw_api.get_camera_image(camera_name)
            if isinstance(image, dict):
                color_, depth_ = image['color'], image['depth']
                cv2.imwrite(os.path.join(save_dir, "color.jpg"), color_)
                cv2.imwrite(os.path.join(save_dir, "depth.jpg"), depth_)
            else:
                cv2.imwrite(file_path, image)
            print(f"Saved image from {camera_name} to {file_path}")
            vision_status[camera_chinese_names[camera_name]] = True

    except Exception as e:
        traceback.print_exc()

    # Check camera status
    vision_model = True
    for chinese_name, status in vision_status.items():
        if not status:
            vision_model = False
            piper_voice.play_voice(f"{chinese_name} 丢失")
    
    if vision_model:
        piper_voice.play_voice("视觉模块就绪")

    # Check battery status
    if node.is_battery_live:
        piper_voice.play_voice(f"电源模块就绪, 当前电量 {node.battery_percent}")
    else:
        piper_voice.play_voice("电源模块丢失")

    # Check IMU status
    if time.time() - node.imu_received_time < 3:
        piper_voice.play_voice("陀螺仪模块就绪")
    else:
        piper_voice.play_voice("陀螺仪模块丢失")
        print("陀螺仪模块丢失", time.time() - node.imu_received_time)

    # Check lidar status
    if (time.time() - node.rplidar_front_received_time < 3 and
            time.time() - node.rplidar_back_received_time < 3):
        piper_voice.play_voice("激光雷达模块就绪")
    else:
        piper_voice.play_voice("激光雷达模块丢失")
        print(
            "激光雷达模块丢失", 
            time.time() - node.rplidar_front_received_time, 
            time.time() - node.rplidar_back_received_time
        )

    # Arm test
    piper_voice.play_voice("手臂测试开始")
    node.replay("pkl/command_full2.pkl")
    piper_voice.play_voice("手臂测试结束")

    piper_voice.play_voice("强制复位")
    node.control_reset()

    piper_voice.play_voice("测试结束")
    print("测试结束")


def main(args=None) -> None:
    """Main function to initialize and run the ROS2 node."""
    rclpy.init(args=args)
    node = ServerSubscriber()

    # Start the check task in a separate thread
    timer_thread = threading.Thread(target=check_task, args=(node,))
    timer_thread.daemon = True  # Set as daemon thread
    timer_thread.start()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Quit by interrupt")
    except Exception as e:
        print(f"Unexpected exception during spin: {e}")
        traceback.print_exc()