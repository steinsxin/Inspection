import io
import json
import os
import re
import sys
import time
import cv2
import traceback
import threading
from contextlib import redirect_stdout, redirect_stderr

# Add parent directory to sys.path for module resolution
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
)
from autolife_robot_inspection.interface import DeviceInterface

# Suppress stdout and stderr during SDK import
with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
    from autolife_robot_sdk.hardware.hw_api import HWAPI as RobotInterface

class CameraDetector(DeviceInterface):
    def __init__(self, config):
        self.config = config
        self.robot_api = RobotInterface()
        self.save_dir = os.path.join(os.path.dirname(__file__), "..", "camera_images")
        os.makedirs(self.save_dir, exist_ok=True)
        self.log_buffer = []

    def _load_config(self):
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'config.json'
        )
        with open(config_path, 'r', encoding='utf-8') as f:
            return json.load(f)

    def _check_single(self, module_name):
        """
        Initialize a single camera module and store status log.
        """
        output_buffer = io.StringIO()

        def task():
            try:
                with redirect_stdout(output_buffer):
                    robot_api = RobotInterface()
                    robot_api.initialize([module_name])
                    cam_name = module_name.replace("mod_camera_", "")
                    image = robot_api.get_camera_image(cam_name)
                    if isinstance(image, dict):
                        if 'color' in image:
                            cv2.imwrite(
                                os.path.join(self.save_dir, f"{cam_name}_color.jpg"),
                                image['color']
                            )
                        if 'depth' in image:
                            cv2.imwrite(
                                os.path.join(self.save_dir, f"{cam_name}_depth.jpg"),
                                image['depth']
                            )
                    else:
                        cv2.imwrite(
                            os.path.join(self.save_dir, f"{cam_name}.jpg"),
                            image
                        )
            except Exception as e:
                output_buffer.write(f"[{module_name}] Detection failed: {e}\n")

        thread = threading.Thread(target=task)
        thread.start()
        thread.join(timeout=10)

        cam_name = module_name.replace("mod_camera_", "")
        if thread.is_alive():
            status = "Timeout after 10s"
        else:
            output = output_buffer.getvalue()
            status = self._parse_status(module_name, output)

        self.log_buffer.append((cam_name, status))

    def _parse_status(self, module_name, output):
        """Parse output to determine camera online/offline."""
        if re.search(rf"Camera {module_name} could not capture frames", output):
            return "Offline"
        return "Online"

    def _parse_log(self, module_name, output):
        """
        Parse the initialization log to determine if the camera is online.
        """
        name = module_name.replace("mod_camera_", "")
        offline_match = re.search(
            rf"Camera {module_name} could not capture frames",
            output
        )
        status = "Offline" if offline_match else "Online"
        return f"{name.capitalize()}: {status}\n"

    def get_keyboard_data(self, keyboard_data):
        return None

    def get_log(self):
        """Return a neatly formatted table of camera detection results."""
        if not self.log_buffer:
            return "No camera modules checked.\n"

        log = "\n Camera Detection Results\n" + "-" * 40
        log += "\n{:<20} | {:<20}".format("Module", "Status")
        log += "\n" + "-" * 21 + "|" + "-" * 20

        for name, status in self.log_buffer:
            log += "\n{:<20} | {:<20}".format(name, status)

        return log + "\n"

    def run(self):
        modules = self.config['camera']['ENABLED_MODULES']
        for module in modules:
            print(f"Detecting camera module: {module}")
            self._check_single_camera(module)
            print(f"Module {module} detection completed.\n")
            time.sleep(0.5)  # slight delay between modules

if __name__ == "__main__":
    try:
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
            'configs', 'model_config.json'
        )
        with open(config_path, 'r', encoding='utf-8') as f:
            config = json.load(f)

        detector = CameraDetector(config)
        detector.run()
        print(detector.get_log())
    except KeyboardInterrupt:
        print("Camera detection interrupted.")
