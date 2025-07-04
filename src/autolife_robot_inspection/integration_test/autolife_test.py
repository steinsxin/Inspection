#!/usr/bin/env python3
import os
import io
import sys
import time
import threading
import traceback
import logging
import json
import re
from typing import Dict
from contextlib import redirect_stdout, redirect_stderr
import concurrent.futures

import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from autolife_robot_inspection import MODEL_CONFIG_PATH, ONNX_ROOT, PKL_ROOT
from autolife_robot_inspection.integration_test.utils import PiperVoice, PublisherNode, ReceiverNode

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

    def __init__(self, save_dir: str, shared_log: Dict):
        self.model_config = self._load_json_config(MODEL_CONFIG_PATH)
        self.camera_model = self.model_config['camera']['ENABLED_MODULES']
        self.audio_model = self.model_config['audio']['ENABLED_MODULES']
        self.all_model = self.camera_model + self.audio_model
        self.robot_api = RobotInterface()
        self.piper_voice = PiperVoice(self.robot_api, f"{ONNX_ROOT}/zh_CN-huayan-medium.onnx")
        self.save_dir = save_dir
        self.shared_log = shared_log
        self.camera_names = {
            name: os.path.join(self.save_dir, f"{name}.jpg")
            for name in self.camera_model
        }
        self.vision_status = {
            name: False for name in self.CAMERA_NAME_MAP.values()
        }

    def _load_json_config(self, path: str) -> dict:
        """Load a JSON configuration file."""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            logging.error(f"Failed to load config '{path}': {e}")
            raise

    def initialize_modules(self):
        """Initialize all required hardware modules."""
        self.robot_api.initialize(self.all_model)

    def check_and_log(self, condition: bool, success_msg: str, failure_msg: str):
        """Helper to check condition and log/play voice."""
        if condition:
            self.piper_voice.play_voice(success_msg)
            self.shared_log['log'] += f"{success_msg}\n"
        else:
            self.piper_voice.play_voice(failure_msg)
            self.shared_log['log'] += f"{failure_msg}\n"

    def check_task(self, number: int, pub_node: PublisherNode, rev_node: ReceiverNode, stop_event=None) -> None:
        """Perform all hardware checks and voice announcements."""
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            self.initialize_modules()

        output = buffer.getvalue()
        match = re.search(r"(?<=Hardware Initialization Report:).*", output, re.DOTALL)
        if match:
            report = (
                "==================================================\n"
                "Hardware Initialization Report\n"
                f"{match.group().strip()}\n"
            )
            self.shared_log['log'] += report
        else:
            self.shared_log['log'] += "No hardware initialization report found.\n"

        self.capture_and_save_images()
        if stop_event.is_set():
            return
        
        # Check battery status
        self.check_and_log(rev_node.is_battery_live,
            f"电源模块就绪, 当前电量 {rev_node.battery_percent}",
            "电源模块丢失"
        )
        if stop_event.is_set():
            return

        # Check IMU status
        self.check_and_log(rev_node.is_imu_live,
            "陀螺仪模块就绪",
            "陀螺仪模块丢失"
        )
        if stop_event.is_set():
            return

        # Check lidar status
        self.check_and_log(
            rev_node.is_rplidar_front_live and rev_node.is_rplidar_back_live,
            "激光雷达模块就绪",
            "激光雷达模块丢失"
        )
        if stop_event.is_set():
            return

        self.piper_voice.play_voice("手臂测试开始")
        self.shared_log['log'] += "手臂测试开始"
        pub_node.replay(f"{PKL_ROOT}/command_full2.pkl")
        self.piper_voice.play_voice("手臂测试结束")
        self.shared_log['log'] += "手臂测试结束"
        if stop_event.is_set():
            return

    def capture_and_save_images(self):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        try:
            self.shared_log['log'] += "capture_and_save_images Start.\n"

            for camera_name, file_path in self.camera_names.items():
                simple_name = camera_name.removeprefix("mod_camera_")
                image = None

                # Using thread pool and timeout mechanism to prevent blocking
                try:
                    with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
                        future = executor.submit(self.robot_api.get_camera_image, simple_name)
                        image = future.result(timeout=3.0)

                        time.sleep(0.1)  # try again
                        future = executor.submit(self.robot_api.get_camera_image, simple_name)
                        image = future.result(timeout=3.0)

                except concurrent.futures.TimeoutError:
                    self.shared_log['log'] += f"{camera_name} 获取图像超时\n"
                    continue
                except Exception as e:
                    self.shared_log['log'] += f"{camera_name} 获取图像失败: {str(e)}\n"
                    continue

                # Save Image
                if isinstance(image, dict):
                    color_ = image['color']
                    depth_ = image['depth']
                    cv2.imwrite(os.path.join(self.save_dir, f"{simple_name}_color.jpg"), color_)
                    cv2.imwrite(os.path.join(self.save_dir, f"{simple_name}_depth.jpg"), depth_)
                else:
                    cv2.imwrite(file_path, image)

                self.shared_log['log'] += f"Saved image from {camera_name} to {file_path}\n"
                self.vision_status[self.CAMERA_NAME_MAP[camera_name]] = True

        except Exception as e:
            traceback.print_exc()
            self.shared_log['log'] += f"捕获图像过程中出现异常: {str(e)}\n"

        # 检查结果
        self.shared_log['log'] += "vision_status 状态如下：\n"
        for chinese_name, status in self.vision_status.items():
            self.shared_log['log'] += f"  {chinese_name}: {'正常' if status else '异常'}\n"

        if all(self.vision_status.values()):
            self.piper_voice.play_voice("视觉模块就绪")
            self.shared_log['log'] += "视觉模块就绪\n"

    def run(self, args=None, stop_event=None) -> None:
        """Main function to initialize and run the ROS2 node."""
        rclpy.init(args=args)
        
        pub_node = PublisherNode(stop_event)
        rev_node = ReceiverNode(stop_event)

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(pub_node)
        executor.add_node(rev_node)

        # Start the check task in a separate thread
        check_task_thread = threading.Thread(target=self.check_task, args=(1, pub_node, rev_node, stop_event,))
        check_task_thread.daemon = True  # Set as daemon thread
        check_task_thread.start()

        try:
            while rclpy.ok() and not stop_event.is_set():
                executor.spin_once(timeout_sec=0.1)
        except (KeyboardInterrupt, ExternalShutdownException):
            print("Quit by interrupt")
        finally:
            check_task_thread.join()
            pub_node.destroy_node()
            rev_node.destroy_node()
            rclpy.shutdown()
        
def main(stop_event=None, key_queue=None, shared_log=None):
    try:
        autolife_test = AutolifeTest(save_dir="camera_images", shared_log=shared_log)
        autolife_test.run(stop_event=stop_event)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Quit by interrupt")
    except Exception as e:
        print(f"Unexpected exception during spin: {e}")
        traceback.print_exc()

if __name__ == "__main__":
    main()