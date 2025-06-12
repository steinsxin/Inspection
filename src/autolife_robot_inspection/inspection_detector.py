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
from datetime import datetime

from autolife_robot_inspection.utils import (
    AudioDetector, BatteryDetector, MotorDetector, CameraDetector, LidarDetector,
    ImuDetector, HardwareDetector, InternalDetector, WifiDetector
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
                    os.system('cls' if os.name != 'nt' else 'clear')
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

    def motor_module_detection(self):
        os.system('cls' if os.name == 'nt' else 'clear')
        print("Press Ctrl+C to stop motor module detection...\n")

        try:
            motor_parts = MotorDetector().config['motor']['ENABLED_MODULES']
            manager = multiprocessing.Manager()
            shared_log = manager.dict()
            all_results = {}

            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            tty.setcbreak(fd)

            for part in motor_parts:
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
                print(f"Detecting motor module: {part}")

                def detect_part(shared_log_):
                    try:
                        detector = MotorDetector()
                        detector._check_single_motor(part)
                        shared_log_['log'] = detector.get_log().strip()
                    except Exception as e:
                        shared_log_['log'] = f"[{part}] Detection failed: {e}"

                process = multiprocessing.Process(target=detect_part, args=(shared_log,))
                process.start()

                while process.is_alive():
                    time.sleep(0.2)
                process.join()

                all_results[part] = shared_log.get('log', f"[{part}] No detection result")
                print(f"Module {part} detection completed.\n")

        except Exception as e:
            print(f"[ERROR] Motor detection error: {e}")

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            os.system('cls' if os.name == 'nt' else 'clear')
            print("\nAll modules detected, results:\n")
            for part, result in all_results.items():
                print(f"Module: {part}\n{result}\n")
            tty.setcbreak(fd)
            while True:
                if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                    sys.stdin.read(1)
                    break
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # Shortcut wrapper methods
    def visual_module_detection(self):
        self._run_detector_subprocess(CameraDetector)

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

    def other_functions_menu(self):
        print("Other Functions: Coming soon...")

    def full_vehicle_test(self):
        os.system('cls' if os.name == 'nt' else 'clear')
        print("Press Ctrl+C to exit...\n")

        root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        script_path = os.path.join(root_dir, "component", "AutolifeTest", "main.py")
        print(f"[INFO] Running: {script_path}")

        try:
            process = subprocess.Popen(
                [sys.executable, script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=1,
                universal_newlines=True
            )
            for line in process.stdout:
                print(line, end='')
            process.wait()
            print(f"[INFO] Integration test finished with return code: {process.returncode}")
        except Exception as e:
            print(f"[ERROR] Failed to run integration test: {e}")

    def face_detection_test(self):
        time.sleep(1)
        print("face_detection_test")
