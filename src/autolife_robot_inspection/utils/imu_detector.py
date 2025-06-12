import io
import json
import os
import re
import sys
import time
import traceback
from collections import defaultdict
from contextlib import redirect_stdout, redirect_stderr
from itertools import zip_longest
from tabulate import tabulate

# Add parent directory to sys.path for module resolution
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
)
from autolife_robot_inspection.interface import DeviceInterface

# Suppress stdout and stderr during SDK import
with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
    from autolife_robot_sdk.hardware.hw_api import HWAPI as RobotInterface


class IMUDetector(DeviceInterface):
    """
    IMU detection and monitoring class.
    Initializes IMU-related modules, detects devices, and continuously reads IMU data.
    """

    def __init__(self, config):
        """
        Initialize the IMU detector.
        Loads configuration, detects IMU modules, and prepares the log.
        """
        self.config = config
        self.robot_api = RobotInterface()
        self.device_info = self._detect_devices()
        self.imu_data = {}
        self.log = self.get_log()

    def _load_config(self):
        """
        Load configuration from the config.json file.

        Returns:
            dict: Configuration dictionary
        """
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_path = os.path.join(project_root, "config", "config.json")

        with open(config_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def _detect_devices(self):
        """
        Detect IMU devices from SDK initialization logs.

        Returns:
            dict: IMU device detection result
        """
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            self.robot_api.initialize(self.config["imu"]["ENABLED_MODULES"])

        output = buffer.getvalue()
        self._init_output = output  # Save for get_log()
        return self._parse_device_output(output)

    def _parse_device_output(self, output: str):
        """
        Parse SDK logs to extract IMU device information.

        Args:
            output (str): Initialization log output

        Returns:
            dict: Parsed IMU status
        """
        device_info = {
            "imu": {
                "found": False,
                "status": "Not detected"
            }
        }

        pattern = re.compile(r"IMU reader registered on (?P<id>\S+)")
        if pattern.search(output):
            device_info["imu"]["found"] = True
            device_info["imu"]["status"] = "registered"

        return device_info

    def get_keyboard_data(self, keyboard_data):
        return None

    def get_log(self) -> str:
        """
        Format and return IMU detection log string.

        Returns:
            str: IMU detection log
        """
        log_lines = [
            "\n============= IMU Detection Results =============",
            f"IMU: {self.device_info['imu']['status']}",
            "=" * 33
        ]

        if self.imu_data:
            formatted_data = self.group_and_format(self.imu_data)
            log_lines.append(formatted_data)
        else:
            log_lines.append("No IMU data available.")

        log_lines.append("=" * 45 + "\n")
        return "\n".join(log_lines)

    def group_and_format(self, data: dict) -> str:
        """
        Group and format IMU data into a table.

        Args:
            data (dict): Raw IMU data

        Returns:
            str: Formatted IMU data in tabular format
        """
        groups = defaultdict(dict)

        for k, v in data.items():
            try:
                value = round(v, 3)
            except Exception:
                value = v

            if k in ('pitch', 'roll', 'yaw'):
                groups['Attitude Angle'][k] = value
            elif k.startswith('acc_'):
                groups['Acceleration'][k] = value
            elif k.startswith('gyro_'):
                groups['Gyro'][k] = value
            elif k.startswith('q'):
                groups['Quaternion'][k] = value
            elif k.startswith('norm_mag_'):
                groups['Normed Mag'][k] = value
            elif k.startswith('raw_mag_'):
                groups['Raw Mag'][k] = value
            elif k in ('year', 'month', 'day', 'hour', 'minute', 'second'):
                groups['Timestamp'][k] = value
            elif k.startswith('vel_') or k in ('alt', 'longt'):
                groups['Pos Acceleration'][k] = value
            elif k in (
                'tid', 'status', 'sensor_temp', 'ms',
                'samp_timestamp', 'dataready_timestamp'
            ):
                groups['Info'][k] = value

        all_lines = []
        group_names = list(groups.keys())

        for left_name, right_name in zip_longest(group_names[::2], group_names[1::2]):
            left_data = groups[left_name] if left_name else {}
            right_data = groups[right_name] if right_name else {}

            left_rows = list(left_data.items())
            right_rows = list(right_data.items())
            combined_rows = list(zip_longest(left_rows, right_rows, fillvalue=('', '')))

            table_rows = []
            for (lk, lv), (rk, rv) in combined_rows:
                table_rows.append([f"{lk}", f"{lv}", f"{rk}", f"{rv}"])

            headers = [
                left_name or '', 'Value',
                right_name or '', 'Value'
            ]
            table = tabulate(
                table_rows, headers=headers, tablefmt='grid',
                colalign=("left", "right", "left", "right")
            )
            all_lines.append(table)

        return "\n\n".join(all_lines)

    def run(self):
        """
        Read and display IMU data in grouped format.
        """
        try:
            self.imu_data = self.robot_api.get_imu_data()
            self.group_and_format(self.imu_data)  # Rebuild table
            print(self.get_log())
            time.sleep(1.0)
        except Exception as e:
            print(f"Error reading IMU data: {e}")
            traceback.print_exc()
            time.sleep(5.0)


if __name__ == "__main__":
    detector = IMUDetector()
    print(detector.log)

    try:
        while True:
            detector.run()
    except KeyboardInterrupt:
        print("\nIMU monitoring interrupted by user.")
    finally:
        detector.robot_api.close(detector.config["imu"]["ENABLED_MODULES"])
        print("IMU monitoring stopped. Resources cleaned up.")

    print("IMU detection complete.")
