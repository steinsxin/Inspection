import io
import json
import os
import re
import sys
import time
import cv2
import traceback
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
        self.save_dir = os.path.join(
            os.path.dirname(__file__), "..", "camera_images"
        )
        os.makedirs(self.save_dir, exist_ok=True)

        self.camera_names = self._generate_camera_names()
        self.hardware_log = self._get_hardware_report()
        self.camera_status = self._parse_camera_status()

    def _load_config(self):
        """Load configuration from config.json."""
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'config.json'
        )
        with open(config_path, 'r', encoding='utf-8') as f:
            return json.load(f)

    def _generate_camera_names(self):
        """Generate camera names and their image save paths."""
        names = {}
        for module in self.config['camera']['ENABLED_MODULES']:
            name = module.replace("mod_camera_", "")
            names[name] = os.path.join(self.save_dir, f"{name}.jpg")
        return names

    def _get_hardware_report(self):
        """Initialize hardware and capture initialization log."""
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            self.robot_api.initialize(
                self.config['camera']['ENABLED_MODULES']
            )
        return buffer.getvalue()

    def _parse_camera_status(self):
        """
        Determine if each camera is online based on the initialization log.
        """
        status = {}
        for name in self.camera_names:
            offline_match = re.search(
                rf"Camera mod_camera_{name} could not capture frames",
                self.hardware_log
            )
            status[name] = not bool(offline_match)
        return status

    def _capture_image(self, name):
        """Capture and save an image for a single camera."""
        image = None
        _buffer = io.StringIO()
        with redirect_stdout(_buffer):
            image = self.robot_api.get_camera_image(name)

        if isinstance(image, dict) and 'color' in image and 'depth' in image:
            cv2.imwrite(
                os.path.join(self.save_dir, f"{name}_color.jpg"),
                image['color']
            )
            cv2.imwrite(
                os.path.join(self.save_dir, f"{name}_depth.jpg"),
                image['depth']
            )
        else:
            cv2.imwrite(self.camera_names[name], image)

    def get_keyboard_data(self, keyboard_data):
        return None
        
    def get_log(self):
        """Return camera detection status as a log string."""
        log = "\n=== Camera Detection Results ===\n"
        for name, online in self.camera_status.items():
            status = "Online" if online else "Offline"
            log += f"{name.capitalize()}: {status}\n"
        log += "=" * 30 + "\n"
        return log

    def run(self):
        """Capture images from all online cameras."""
        time.sleep(1)
        for name, path in self.camera_names.items():
            if not self.camera_status[name]:
                continue
            try:
                self._capture_image(name)
            except Exception as e:
                traceback.print_exc()


if __name__ == "__main__":

    detector = CameraDetector()
    print(detector.log)
    try:
        while True:
            detector.run()
    except KeyboardInterrupt:
        print("Loopback interrupted.")
    finally:
        detector.robot_api.close(detector.config['visual']['ENABLED_MODULES'])
        print("Stopped audio loopback.")

    print("Camera detection complete.")