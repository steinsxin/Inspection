import json
import multiprocessing
import logging
import os
import select
import socket
import subprocess
import sys
import termios
import time
import tty
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
        self.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.original_stdout = sys.stdout
        self.original_stderr = sys.stderr
        self.model_config = self._load_json_config(MODEL_CONFIG_PATH)
        self.func_config = self._load_json_config(FUNC_CONFIG_PATH)

    def _load_json_config(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            logging.error(f"Failed to load config '{path}': {e}")
            raise

    def _run_detector_subprocess(self, detector_cls, refresh_log=False):
        os.system('cls' if os.name == 'nt' else 'clear')
        print("Press any key to stop loopback...")

        manager = multiprocessing.Manager()
        shared_log = manager.dict()
        key_queue = multiprocessing.Queue()

        def run_detection(shared_log):
            try:
                detector = detector_cls(self.model_config)
                shared_log['log'] = detector.get_log()
                print(shared_log['log'])
                sys.stdout = open(os.devnull, 'w')
                sys.stderr = open(os.devnull, 'w')
                while True:
                    if not key_queue.empty():
                        detector.get_keyboard_data(key_queue.get())
                    shared_log['log'] = detector.get_log()
                    detector.run()
            except Exception as e:
                print(f"Error during module detection: {e}")

        process = multiprocessing.Process(target=run_detection, args=(shared_log,))
        process.start()

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        try:
            while process.is_alive():
                if refresh_log and 'log' in shared_log:
                    sys.stdout = self.original_stdout
                    sys.stderr = self.original_stderr
                    os.system('cls' if os.name == 'nt' else 'clear')
                    print("Press any key to stop loopback...")
                    print(shared_log['log'])
                    time.sleep(0.1)
                if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                    char = sys.stdin.read(1)
                    key_queue.put(char)
                    if char not in ['+', '-']:
                        print("Stopping loopback...")
                        process.terminate()
                        break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            process.join()


    def _module_detection(self, module_list, module_type, detector_class, check_func_name, parse_result):
        os.system('cls' if os.name == 'nt' else 'clear')
        print(f"Press Ctrl+C to stop {module_type.lower()} module detection...\n")

        manager = multiprocessing.Manager()
        shared_log = manager.dict()
        all_results = []

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        try:
            for module in module_list:
                try:
                    time.sleep(0.1)
                    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                        sys.stdin.read(1)
                        print("\nUser interrupted detection.\n")
                        break
                except KeyboardInterrupt:
                    print("\nUser stopped detection via Ctrl+C.\n")
                    break

                shared_log.clear()
                print(f"Detecting {module_type.lower()} module: {module}")

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

                print(f"Module {module} detection completed.\n")

        except Exception as e:
            print(f"[ERROR] {module_type} detection error: {e}")

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            os.system('cls' if os.name == 'nt' else 'clear')

            print(f"\n {module_type} Detection Results\n" + "-" * 40)
            print("{:<20} | {:<20}".format("Module", "Status"))
            print("-" * 21 + "|" + "-" * 20)
            for name, status in all_results:
                print("{:<20} | {:<20}".format(
                    name.replace("mod_camera_", "").replace("mod_motor_", ""), status
                ))

            tty.setcbreak(fd)
            while True:
                if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                    sys.stdin.read(1)
                    break
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _run_integration_test(self, test_name, module_path):
        os.system('cls' if os.name == 'nt' else 'clear')
        print("Press Ctrl+C to exit...\n")

        def run_script():
            try:
                root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                script_path = os.path.join(root_dir, *module_path.split(".")) + ".py"
                print(f"[INFO] Running: {script_path}")

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
                    print(f"[ERROR] No 'main()' found in {test_name}")
            except Exception:
                print(f"[ERROR] Exception in {test_name}:")
                traceback.print_exc()

        try:
            thread = threading.Thread(target=run_script)
            thread.start()
            thread.join()
            print(f"[INFO] {test_name} test finished.")
        except Exception as e:
            print(f"[ERROR] Failed to run {test_name} test: {e}")


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

    def audio_module_detection(self):
        self._run_detector_subprocess(AudioDetector, refresh_log=True)

    def lidar_module_detection(self):
        self._run_detector_subprocess(LidarDetector)

    def battery_module_detection(self):
        self._run_detector_subprocess(BatteryDetector, refresh_log=True)

    def imu_module_detection(self):
        self._run_detector_subprocess(IMUDetector, refresh_log=True)

    def hardware_info(self):
        self._run_detector_subprocess(HardwareDetector)

    def internal_network_test(self):
        self._run_detector_subprocess(InternalDetector)

    def wifi_test(self):
        self._run_detector_subprocess(WifiDetector)

    ########### IntegrationTest ###########

    def AutolifeTest(self):
        self._run_integration_test("AutolifeTest", "autolife_robot_inspection.IntegrationTest.AutolifeTest.main")

    def Car_movement(self):
        self._run_integration_test("Car_movement", "autolife_robot_inspection.IntegrationTest.Car_movement.main")

    def Welcome_guests(self):
        time.sleep(1)
        print("Welcome_guests over")

    ########### OtherFunctions ###########
    def robot_arm_start(self):
        os.system('cls' if os.name == 'nt' else 'clear')
        print("Press any key to stop loopback...")
        config = {
            "hostname": "192.168.10.3",
            "port": 22,
            "username": "nvidia",
            "password": "nvidia",
            "python": "/home/nvidia/miniconda3/envs/ros2/bin/python",
            "script_path": "autolife_robot_arm.main",
            "log_path": "/home/nvidia/Documents/arm_output.log"
        }

        manager = RemoteScriptManager(
            hostname=config["hostname"],
            port=config["port"],
            username=config["username"],
            password=config["password"],
            python_path=config["python"]
        )

        # 关闭已有程序
        manager.stop_script()
        time.sleep(1)
        
        # 重启程序
        manager.run_script(config["script_path"], config["log_path"])
        time.sleep(1)
        
        print("robot_arm_start over")
        time.sleep(1)

    def set_motor_zero(self):
        time.sleep(1)
        print("set_motor_zero over")