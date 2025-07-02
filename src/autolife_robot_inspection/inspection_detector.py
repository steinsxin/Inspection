import json
import multiprocessing
import logging
import os

import socket
import subprocess
import sys

import time

import traceback
import threading
import importlib.util
from datetime import datetime

from autolife_robot_inspection.ModuleTest import (
    AudioDetector, BatteryDetector, MotorDetector, CameraDetector, LidarDetector,
    IMUDetector, HardwareDetector, InternalDetector, WifiDetector
)
from autolife_robot_inspection.OtherFunc import (
    RemoteScriptManager
)
from autolife_robot_inspection import MODEL_CONFIG_PATH, MENU_CONFIG_PATH, FUNC_CONFIG_PATH

class InspectionDetector:
    """
    A hardware diagnostic tool for Jetson-based systems
    supporting modular, multilingual detection workflows.
    """

    def __init__(self, language='en'):
        self.language = language
        self._log_screen = None
        self.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.model_config = self._load_json_config(MODEL_CONFIG_PATH)
        self.func_config = self._load_json_config(FUNC_CONFIG_PATH)

    def set_log_screen(self, log_screen_instance):
        self._log_screen = log_screen_instance

    def _load_json_config(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            logging.error(f"Failed to load config '{path}': {e}")
            raise

    def _run_direct_log_detector(self, DetectorClass, stop_event=None):
        """
        Runs a detector synchronously, fetches log, and optionally waits until stop_event is set.
        Used for detectors like WifiDetector and HardwareDetector that run directly and produce logs.
        """
        try:
            detector = DetectorClass(self.model_config)
            log_content = detector.get_log()
            if self._log_screen:
                self._log_screen.append_log(log_content)

            if stop_event:
                import queue
                input_q = queue.Queue()

                if self._log_screen:
                    self._log_screen.start_keyboard_capture(input_q)

                try:
                    while not stop_event.is_set():
                        try:
                            key = input_q.get_nowait()
                            if self._log_screen:
                                self._log_screen.append_log(f"Received command: {key}")

                        except queue.Empty:
                            pass
                        time.sleep(0.1)
                finally:
                    if self._log_screen:
                        self._log_screen.stop_keyboard_capture()

        except Exception as e:
            if self._log_screen:
                self._log_screen.append_log(
                    f"Error during {DetectorClass.__name__} detection: {e}"
                )


    def _module_detection(self, module_list, module_type, detector_class, check_func_name, parse_result):
        if self.ui_callback:
            self.ui_callback(f"Press Ctrl+C to stop {module_type.lower()} module detection...\n")

        manager = multiprocessing.Manager()
        shared_log = manager.dict()
        all_results = []

        try:
            for module in module_list:
                try:
                    time.sleep(0.1)
                except KeyboardInterrupt:
                    if self.ui_callback:
                        self.ui_callback("\nUser stopped detection via Ctrl+C.\n")
                    break

                shared_log.clear()
                if self.ui_callback:
                    self.ui_callback(f"Detecting {module_type.lower()} module: {module}")

                def detect(shared):
                    try:
                        detector = detector_class(self.model_config)
                        getattr(detector, check_func_name)(module)
                        shared['result'] = parse_result(detector, module)
                    except Exception as e:
                        shared['result'] = (module, f"Detection failed: {e}")

                process = multiprocessing.Process(target=detect, args=(shared_log,))
                process.start()
                process.join(timeout=15)

                if process.is_alive():
                    process.terminate()
                    process.join()
                    shared_log['result'] = (module, "Timeout after 15s")

                all_results.append(shared_log.get('result'))

                if self.ui_callback:
                    self.ui_callback(f"Module {module} detection completed.\n")

        except Exception as e:
            if self.ui_callback:
                self.ui_callback(f"[ERROR] {module_type} detection error: {e}")

        finally:
            if self.ui_callback:
                self.ui_callback(f"\n {module_type} Detection Results\n" + "-" * 40)
                self.ui_callback("{:<20} | {:<20}".format("Module", "Status"))
                self.ui_callback("-" * 21 + "|" + "-" * 20)
                for name, status in all_results:
                    self.ui_callback("{:<20} | {:<20}".format(
                        name.replace("mod_camera_", "").replace("mod_motor_", ""), status
                    ))

    def _run_integration_test(self, test_name, module_path):
        if self.ui_callback:
            self.ui_callback("Press Ctrl+C to exit...\n")

        def run_script():
            try:
                root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                script_path = os.path.join(root_dir, *module_path.split(".")) + ".py"
                if self.ui_callback:
                    self.ui_callback(f"[INFO] Running: {script_path}")

                # 将脚本所在目录临时添加到 sys.path
                script_dir = os.path.dirname(script_path)
                if script_dir not in sys.path:
                    sys.path.insert(0, script_dir)

                spec = importlib.util.spec_from_file_location(test_name, script_path)
                test_module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(test_module)

                if hasattr(test_module, "main"):
                    test_module.main()
                else:
                    if self.ui_callback:
                        self.ui_callback(f"[ERROR] No 'main()' found in {test_name}")
            except Exception:
                if self.ui_callback:
                    self.ui_callback(f"[ERROR] Exception in {test_name}:")
                    self.ui_callback(traceback.format_exc())

        try:
            thread = threading.Thread(target=run_script)
            thread.start()
            thread.join()
            if self.ui_callback:
                self.ui_callback(f"[INFO] {test_name} test finished.")
        except Exception as e:
            if self.ui_callback:
                self.ui_callback(f"[ERROR] Failed to run {test_name} test: {e}")


    ########### ModuleTest ###########

    def motor_module_detection(self):
        detector = MotorDetector(self.model_config)
        motor_modules = detector.config['motor']['ENABLED_MODULES']
        self._module_detection(
            module_list=motor_modules,
            module_type="Motor",
            detector_class=MotorDetector,
            check_func_name="_check_single",
            parse_result=lambda det, mod: (mod.replace("mod_motor_", ""), det.get_log().strip())
        )

    def visual_module_detection(self):
        detector = CameraDetector(self.model_config)
        camera_modules = detector.config['camera']['ENABLED_MODULES']
        self._module_detection(
            module_list=camera_modules,
            module_type="Camera",
            detector_class=CameraDetector,
            check_func_name="_check_single",
            parse_result=lambda det, mod: det.log_buffer[-1] if det.log_buffer else (mod.replace("mod_camera_", ""), "No result")
        )

    def audio_module_detection(self, stop_event=None):
        self._run_direct_log_detector(AudioDetector, stop_event)

    def lidar_module_detection(self, stop_event=None):
        self._run_direct_log_detector(LidarDetector, stop_event)

    def battery_module_detection(self, stop_event=None):
        self._run_direct_log_detector(BatteryDetector, stop_event)

    def imu_module_detection(self, stop_event=None):
        self._run_direct_log_detector(IMUDetector, stop_event)

    def hardware_info(self, stop_event=None):
        self._run_direct_log_detector(HardwareDetector, stop_event)

    def internal_network_test(self, stop_event=None):
        self._run_direct_log_detector(InternalDetector, stop_event)

    def wifi_test(self, stop_event=None):
        self._run_direct_log_detector(WifiDetector, stop_event)

    ########### IntegrationTest ###########

    def AutolifeTest(self):
        self._run_integration_test("AutolifeTest", "autolife_robot_inspection.IntegrationTest.AutolifeTest.main")

    def Car_movement(self):
        self._run_integration_test("Car_movement", "autolife_robot_inspection.IntegrationTest.Car_movement.main")

    def Welcome_guests(self):
        time.sleep(1)
        if self.ui_callback:
            self.ui_callback("Welcome_guests over")

    ########### OtherFunctions ###########
    def robot_arm_start(self):
        if self.ui_callback:
            self.ui_callback("Press any key to stop loopback...")

        RemoteScriptManager.ROS_DOMAIN_ID = self.model_config["ROS_DOMAIN_ID"]
        configs = {
            "vision": {
                "hostname": "192.168.10.2",
                "port": 22,
                "username": "nvidia",
                "password": "nvidia",
                "python": "/home/nvidia/miniconda3/envs/ros2/bin/python",
                "script_path": "autolife_robot_vision.main",
                "log_path": "/home/nvidia/Documents/vision_output.log"
            },
            "arm": {
                "hostname": "192.168.10.3",
                "port": 22,
                "username": "nvidia",
                "password": "nvidia",
                "python": "/home/nvidia/miniconda3/envs/ros2/bin/python",
                "script_path": "autolife_robot_arm.main",
                "log_path": "/home/nvidia/Documents/arm_output.log"
            }
        }

        managers = {
            "vision": RemoteScriptManager(
                hostname=configs["vision"]["hostname"],
                port=configs["vision"]["port"],
                username=configs["vision"]["username"],
                password=configs["vision"]["password"],
                python_path=configs["vision"]["python"]
            ),
            "arm": RemoteScriptManager(
                hostname=configs["arm"]["hostname"],
                port=configs["arm"]["port"],
                username=configs["arm"]["username"],
                password=configs["arm"]["password"],
                python_path=configs["arm"]["python"]
            )
        }

        config = configs['arm']
        manager = managers['arm']

        # 关闭已有程序
        manager.stop_arm_script()
        time.sleep(1)
        
        # 重启程序
        manager.run_script(config["script_path"], config["log_path"])
        time.sleep(1)
        
        if self.ui_callback:
            self.ui_callback("robot_arm_start over")
        time.sleep(1)

    def set_motor_zero(self):
        time.sleep(1)
        if self.ui_callback:
            self.ui_callback("set_motor_zero over")