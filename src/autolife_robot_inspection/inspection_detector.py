import json
import multiprocessing
import logging
import os

import socket
import subprocess
import sys

import time
import queue

import traceback
import threading
import importlib.util
from datetime import datetime

from autolife_robot_inspection.module_test import (
    AudioDetector, BatteryDetector, MotorDetector, CameraDetector, LidarDetector,
    IMUDetector, HardwareDetector, InternalDetector, WifiDetector
)
from autolife_robot_inspection.otherfunc import (
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

    def _run_detector(self, DetectorClass, stop_event=None):
        """
        多进程运行 detector（用于 WifiDetector、HardwareDetector 等直接运行的模块），
        实时刷新 log_screen，支持键盘交互。
        """
        manager = multiprocessing.Manager()
        shared_log = manager.dict()
        key_queue = multiprocessing.Queue()

        def run_detector(shared_log, key_queue, stop_event):

            try:
                detector = DetectorClass(self.model_config)
                shared_log["log"] = detector.get_log()

                while not stop_event.is_set():
                    if not key_queue.empty():
                        key = key_queue.get()
                        detector.get_keyboard_data(key)

                    shared_log["log"] = detector.get_log()
                    detector.run()

            except Exception as e:
                error_msg = f"Error during {DetectorClass.__name__} detection: {e}"
                shared_log["log"] = error_msg
            finally:
                shared_log["log"] = f"[{DetectorClass.__name__}] Detector stopped."

        process_stop_event = multiprocessing.Event()
        process = multiprocessing.Process(
            target=run_detector,
            args=(shared_log, key_queue, process_stop_event)
        )
        process.start()

        input_q = queue.Queue()
        if self._log_screen:
            self._log_screen.start_keyboard_capture(input_q)

        try:
            while process.is_alive() and not stop_event.is_set():
                if "log" in shared_log and self._log_screen:
                    self._log_screen.set_log(shared_log["log"])

                try:
                    last_key_event = None
                    while not input_q.empty():
                        last_key_event = input_q.get_nowait()

                    if last_key_event:
                        key_queue.put(last_key_event)
                except queue.Empty:
                    pass

                time.sleep(0.05)

        finally:
            process_stop_event.set()  # 通知子进程退出
            process.join()
            
            if self._log_screen:
                self._log_screen.stop_keyboard_capture()

    def _module_detection(self, module_list, module_type, detector_class,
                        check_func_name, parse_result, stop_event=None):

        manager = multiprocessing.Manager()
        shared_log = manager.dict()
        all_results = []

        full_log = ""  # 维护完整追加日志
        last_log = ""  # 记录上一次 shared_log["log"]，方便追加增量

        def detect_module(shared, module):
            try:
                shared["log"] = f"{module} Checking..."
                detector = detector_class(self.model_config)
                getattr(detector, check_func_name)(module)
                shared['result'] = parse_result(detector, module)
                shared["log"] = f"{module} Detection completed.\n"
            except Exception as e:
                shared["log"] = f"{module} Detection failed: {e}"
                shared['result'] = (module, f"Detection failed: {e}")
            time.sleep(0.5)

        try:
            for module in module_list:
                time.sleep(0.1)
                shared_log.clear()
                shared_log["log"] = f"Detecting {module_type.lower()} module: {module}..."

                process = multiprocessing.Process(target=detect_module,
                                                args=(shared_log, module))
                process.start()

                start_time = time.time()
                while process.is_alive() and time.time() - start_time < 15:
                    time.sleep(0.2)
                    if "log" in shared_log:
                        current_log = shared_log["log"]
                        if current_log != last_log:
                            full_log += current_log + "\n"
                            last_log = current_log
                        if self._log_screen:
                            self._log_screen.set_log(full_log)

                if process.is_alive():
                    process.terminate()
                    process.join()
                    timeout_msg = f"[{module}] Timeout after 15s"
                    shared_log['result'] = (module, "Timeout after 15s")
                    shared_log['log'] = timeout_msg
                    full_log += timeout_msg + "\n"
                    if self._log_screen:
                        self._log_screen.set_log(full_log)

                all_results.append(shared_log.get('result'))

        except Exception as e:
            error_msg = f"[ERROR] {module_type} detection error: {e}"
            full_log += error_msg + "\n"
            if self._log_screen:
                self._log_screen.set_log(full_log)

        finally:
            result_log = [
                f"\n{module_type} Detection Results",
                "-" * 40,
                "{:<20} | {:<20}".format("Module", "Status"),
                "-" * 21 + "|" + "-" * 20
            ]
            for name, status in all_results:
                clean_name = name.replace("mod_camera_", "").replace("mod_motor_", "")
                result_log.append("{:<20} | {:<20}".format(clean_name, status))

            final_log = "\n".join(result_log)
            full_log += final_log + "\n"

            try:
                while stop_event is None or not stop_event.is_set():
                    if self._log_screen:
                        self._log_screen.set_log(full_log)
                    time.sleep(0.1)
            finally:
                if process.is_alive():
                    process.terminate()
                process.join()

    def _run_integration_test(self, test_name, module_path, stop_event=None):
        manager = multiprocessing.Manager()
        shared_log = manager.dict()
        key_queue = multiprocessing.Queue()

        def run_script(shared_log, key_queue, stop_event):
            import sys, os, importlib.util, traceback

            try:
                root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                script_path = os.path.join(root_dir, *module_path.split(".")) + ".py"
                script_dir = os.path.dirname(script_path)
                if script_dir not in sys.path:
                    sys.path.insert(0, script_dir)

                spec = importlib.util.spec_from_file_location(test_name, script_path)
                test_module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(test_module)

                if hasattr(test_module, "main"):
                    shared_log["log"] = f"Running {test_name}...\n"
                    # 如果 main 支持传入 keyboard_input 或 stop_event, 可增强交互能力
                    if "stop_event" in test_module.main.__code__.co_varnames:
                        test_module.main(stop_event=stop_event, key_queue=key_queue, shared_log=shared_log)
                    else:
                        test_module.main()
                    shared_log["log"] += f"{test_name} test completed."
                else:
                    shared_log["log"] += f"No main() found in {test_name}"
        
            except Exception as e:
                shared_log["log"] = f"Exception in {test_name}:\n{traceback.format_exc()}"
            finally:
                shared_log["log"] += f"\n[{test_name}] Script stopped."

        process_stop_event = multiprocessing.Event()
        process = multiprocessing.Process(
            target=run_script,
            args=(shared_log, key_queue, process_stop_event)
        )
        process.start()

        input_q = queue.Queue()
        if self._log_screen:
            self._log_screen.start_keyboard_capture(input_q)

        try:
            while process.is_alive() and not stop_event.is_set():
                if "log" in shared_log and self._log_screen:
                    self._log_screen.set_log(shared_log["log"])

                try:
                    last_key_event = None
                    while not input_q.empty():
                        last_key_event = input_q.get_nowait()

                    if last_key_event:
                        key_queue.put(last_key_event)
                except queue.Empty:
                    pass

                time.sleep(0.05)

        finally:
            process_stop_event.set()
            process.join()

            if self._log_screen:
                self._log_screen.stop_keyboard_capture()


    ########### ModuleTest ###########

    def motor_module_detection(self, stop_event=None):
        detector = MotorDetector(self.model_config)
        motor_modules = detector.config['motor']['ENABLED_MODULES']
        self._module_detection(
            module_list=motor_modules,
            module_type="Motor",
            detector_class=MotorDetector,
            check_func_name="_check_single",
            parse_result=lambda det, mod: (mod.replace("mod_motor_", ""), det.get_log().strip()),
            stop_event=stop_event
        )

    def visual_module_detection(self, stop_event=None):
        detector = CameraDetector(self.model_config)
        camera_modules = detector.config['camera']['ENABLED_MODULES']
        self._module_detection(
            module_list=camera_modules,
            module_type="Camera",
            detector_class=CameraDetector,
            check_func_name="_check_single",
            parse_result=lambda det, mod: det.log_buffer[-1] if det.log_buffer else (mod.replace("mod_camera_", ""), "No result"),
            stop_event=stop_event
        )

    def audio_module_detection(self, stop_event=None):
        self._run_detector(AudioDetector, stop_event)

    def lidar_module_detection(self, stop_event=None):
        self._run_detector(LidarDetector, stop_event)

    def battery_module_detection(self, stop_event=None):
        self._run_detector(BatteryDetector, stop_event)

    def imu_module_detection(self, stop_event=None):
        self._run_detector(IMUDetector, stop_event)

    def hardware_info(self, stop_event=None):
        self._run_detector(HardwareDetector, stop_event)

    def internal_network_test(self, stop_event=None):
        self._run_detector(InternalDetector, stop_event)

    def wifi_test(self, stop_event=None):
        self._run_detector(WifiDetector, stop_event)

    ########### IntegrationTest ###########

    def autolife_test(self, stop_event=None):
        self._run_integration_test("autolife_test", "autolife_robot_inspection.integration_test.autolife_test", stop_event)

    def car_movement(self, stop_event=None):
        self._run_integration_test("car_movement", "autolife_robot_inspection.integration_test.car_movement", stop_event)

    def joint_control(self, stop_event=None):
        self._run_integration_test("joint_control", "autolife_robot_inspection.integration_test.joint_control", stop_event)

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