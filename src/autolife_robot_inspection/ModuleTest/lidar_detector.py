import io
import json
import os
import re
import sys
import time
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
    
class LidarDetector(DeviceInterface):
    """
    A Lidar detector class
    """

    def __init__(self, config):
        """Initialize by loading config and detecting audio devices."""
        self.config = config
        self.robot_api = RobotInterface()
        self.device_info = self._detect_devices()
        self.log = self.get_log()

    def _detect_devices(self):
        """
        Detect RPLidar devices from SDK initialization logs.

        Returns:
            dict: IMU device detection result
        """
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            self.robot_api.initialize(self.config["lidar"]["ENABLED_MODULES"])

        output = buffer.getvalue()
        self._init_output = output  # Save for get_log()
        return self._parse_device_output(output)


    def _parse_device_output(self, output: str):
        """
        Parse SDK logs to extract Lidar device information.

        Args:
            output (str): Initialization log output

        Returns:
            dict: Parsed Lidar status
        """
        device_info = {
            "lidar0": {
                "found": False,
                "status": "Not detected"
            },

            "lidar1": {
                "found": False,
                "status": "Not detected"
            }
        }
        pattern = re.compile(r"Lidar reader registered on (?P<id>\S+)")
        matches  = pattern.findall(output)
        for i, idx in enumerate(matches):
            device_info[f"lidar{i}"]["found"] = True
            device_info[f"lidar{i}"]["status"] = "registered"

        return device_info

    def _load_config(self):
        """Load configuration from config.json."""
        project_root = os.path.dirname(
            os.path.dirname(os.path.abspath(__file__))
        )
        config_path = os.path.join(project_root, 'config', 'config.json')
        with open(config_path, 'r', encoding='utf-8') as f:
            return json.load(f)

    def get_keyboard_data(self, keyboard_data):
        return None

    def get_log(self):
        """Format and return the detection result as a log string."""
        log_lines = [
            "\n============= Lidar Detection Results ============="]
        for i in range(len(self.device_info)):
            log_lines.append(f"lidar{i}: {self.device_info[f'lidar{i}']['status']}")
        log_lines.append("=" * 51)
        return '\n'.join(log_lines)

    def run(self):
        while True:
            time.sleep(1)
        return None


if __name__ == "__main__":
    detector = LidarDetector()
    print(detector.log)
    try:
        while True:
            detector.run()
    except KeyboardInterrupt:
        print("\nLidar monitoring interrupted by user.")
    finally:
        detector.robot_api.close(detector.config["lidar"]["ENABLED_MODULES"])
        print("Lidar monitoring stopped. Resources cleaned up.")

    print("Lidar detection complete.")


