#!/usr/bin/env python3
import os
import io
import sys
import time
import threading
import traceback
from typing import Dict
import logging
import json

import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from autolife_robot_inspection import MODEL_CONFIG_PATH, ONNX_ROOT, PKL_ROOT
from autolife_robot_inspection.integration_test.utils import PiperVoice, ServerSubscriber
from contextlib import redirect_stdout, redirect_stderr

with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
    from autolife_robot_sdk.hardware.hw_api import HWAPI as RobotInterface


class AutolifeTest:

    CAMERA_NAME_MAP = {
        "mod_camera_head_left": "头部左摄像头",
        "mod_camera_head_right": "头部右摄像头",
        "mod_camera_head_rear": "头部后摄像头",
        "mod_camera_hand_left": "左手摄像头",
        "mod_camera_hand_right": "右手摄像头",
    }

    def __init__(self, save_dir: str):
        self.model_config = self._load_json_config(MODEL_CONFIG_PATH)
        self.camera_model = self.model_config['camera']['ENABLED_MODULES']
        self.audio_model = self.model_config['audio']['ENABLED_MODULES']
        self.all_model = self.camera_model + self.audio_model
        self.onnx_path = f'{ONNX_ROOT}/zh_CN-huayan-medium.onnx'
        self.robot_api = RobotInterface()
        self.save_dir = save_dir
        self.camera_names = self._generate_camera_names()
        self.vision_status: Dict[str, bool] = {chinese_name: False for chinese_name in self.CAMERA_NAME_MAP.values()}

    def _load_json_config(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            logging.error(f"Failed to load config '{path}': {e}")
            raise

    def _generate_camera_names(self):
        return {
            name: os.path.join(self.save_dir, f"{name}.jpg")
            for name in self.camera_model
        }

    def initialize_modules(self):
        self.robot_api.initialize(self.all_model)

    def check_task(self, number, node: ServerSubscriber) -> None:
        """Check and test all hardware modules.

        Args:
            node: The ServerSubscriber node instance.
        """
        # self.initialize_modules()
    
        # self.capture_and_save_images()
        
        # # Check battery status
        # if node.is_battery_live:
        #     piper_voice.play_voice(f"电源模块就绪, 当前电量 {node.battery_percent}")
        # else:
        #     piper_voice.play_voice("电源模块丢失")

        # # Check IMU status
        # if time.time() - node.imu_received_time < 3:
        #     piper_voice.play_voice("陀螺仪模块就绪")
        # else:
        #     piper_voice.play_voice("陀螺仪模块丢失")

        # # Check lidar status
        # if (time.time() - node.rplidar_front_received_time < 3 and
        #         time.time() - node.rplidar_back_received_time < 3):
        #     piper_voice.play_voice("激光雷达模块就绪")
        # else:
        #     piper_voice.play_voice("激光雷达模块丢失")

        # piper_voice.play_voice("手臂测试开始")
        # node.replay("pkl/command_full2.pkl")
        # piper_voice.play_voice("手臂测试结束")


    def capture_and_save_images(self):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        try:
            for camera_name, file_path in self.camera_names.items():
                # Get camera image
                image = self.robot_api.get_camera_image(camera_name)
                time.sleep(0.1)
                image = self.robot_api.get_camera_image(camera_name)
                if isinstance(image, dict):
                    color_, depth_ = image['color'], image['depth']
                    cv2.imwrite(os.path.join(self.save_dir, "color.jpg"), color_)
                    cv2.imwrite(os.path.join(self.save_dir, "depth.jpg"), depth_)
                else:
                    cv2.imwrite(file_path, image)
                print(f"Saved image from {camera_name} to {file_path}")
                self.vision_status[self.CAMERA_NAME_MAP[camera_name]] = True

        except Exception as e:
            traceback.print_exc()

        # Check camera status
        vision_model = all(status for status in self.vision_status.values()) 
        for chinese_name, status in self.vision_status.items():
            if not status:
                piper_voice.play_voice(f"{chinese_name} 丢失")

        if vision_model:
            self.robot_api.play_voice("视觉模块就绪")

    def main(self, args=None) -> None:
        """Main function to initialize and run the ROS2 node."""
        rclpy.init(args=args)
        node = ServerSubscriber()

        # Start the check task in a separate thread
        timer_thread = threading.Thread(target=self.check_task, args=(1, node,))
        timer_thread.daemon = True  # Set as daemon thread
        timer_thread.start()

        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    try:
        test = AutolifeTest(save_dir="camera_images")  # 确保传入一个保存目录
        test.main()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Quit by interrupt")
    except Exception as e:
        print(f"Unexpected exception during spin: {e}")
        traceback.print_exc()