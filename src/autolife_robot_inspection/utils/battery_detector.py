import io
import json
import os
import re
import sys
import time
import traceback
import textwrap
from contextlib import redirect_stdout, redirect_stderr

# Add parent directory to sys.path for module resolution
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
)
from autolife_robot_inspection.interface import DeviceInterface

# Suppress stdout and stderr during SDK import
with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
    from autolife_robot_sdk.hardware.hw_api import HWAPI as RobotInterface

class BatteryDetector(DeviceInterface):
    """Battery detection and monitoring class.
    
    Initializes battery-related devices and provides continuous monitoring
    of battery data through the robot's hardware API.
    """

    def __init__(self, config):
        """Initialize the battery detector.
        
        Loads configuration, detects battery devices, and sets up logging.
        """
        self.config = config
        self.robot_api = RobotInterface()
        self.device_info = self._detect_devices()
        self.battery_data = ""
        self.log = self.get_log()

    def _detect_devices(self):
        """Detect battery devices from SDK initialization logs.
        
        Returns:
            dict: Dictionary containing battery device detection status
        """
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            self.robot_api.initialize(self.config["battery"]["ENABLED_MODULES"])

        output = buffer.getvalue()
        return self._parse_device_output(output)

    def _parse_device_output(self, output: str):
        """Parse SDK initialization logs to extract device information.
        
        Args:
            output: String containing the SDK initialization logs
            
        Returns:
            dict: Dictionary with parsed device information
        """
        device_info = {
            "battery": {
                "found": False,
                "status": "Not detected"
            }
        }

        # Regular expression to match battery status
        battery_status_pattern = re.compile(
            r"mod_battery_:\s*\n\s*No issues detected\s*\n\s*未检测到问题"
        )

        if battery_status_pattern.search(output):
            device_info["battery"]["found"] = True
            device_info["battery"]["status"] = "rs485 detected"

        return device_info


    def get_keyboard_data(self, keyboard_data):
        return None

    def get_log(self) -> str:
        """Format and return the detection result as a log string.
        
        Returns:
            str: Formatted log string with detection results
        """
        log_lines = [
            "\n=== Battery Device Detection Results ===",
            f"Battery: {self.device_info['battery']['status']}",
            "=" * 25 + "\n"
        ]
        log_lines.append(self.battery_data.strip())
        return "\n".join(log_lines)

    def run(self) -> None:
        """Continuously monitor battery data with aligned box-drawing display."""
        try:
            bat = self.robot_api.get_battery_data()
            status = "Charging" if bat.get("current", 0) > 0 else "Discharging"
            self.battery_data = textwrap.dedent(f"""\
                ┌─────────────────────────────────────────────────────┐
                │ 🔋   Status: {status:<11}    │ Health: {bat.get('health', 100):>3}%          │
                ├──────────────┬──────────────┬───────────────────────┤
                │  Voltage     │  Current     │  Temperature          │
                │  {bat.get('voltage', 0):<5.1f} V     │  {bat.get('current', 0):<+6.2f} A    │    {bat.get('temperature', 25.0):<4.1f} °C            │
                ├──────────────┼──────────────┼───────────────────────┤
                │  Current Cap │  Total Cap   │  Remaining            │
                │  {bat.get('remaining_capacity', 0):<5.2f} Ah    │  {bat.get('nominal_capacity', 0):<5.1f} Ah    │ {bat.get('capacity_percentage', 0):>6} %              │
                └──────────────┴──────────────┴───────────────────────┘
            """)
            # print(self.battery_data)
            time.sleep(1.0)

        except Exception as error:
            print(f"Error processing battery data: {error}")
            traceback.print_exc()
            time.sleep(5.0)

if __name__ == "__main__":
    detector = BatteryDetector()
    print(detector.log)
    try:
        while True:
            detector.run()
    except KeyboardInterrupt:
        print("\nBattery monitoring interrupted by user.")
    finally:
        detector.robot_api.close(detector.config["battery"]["ENABLED_MODULES"])
        print("Battery monitoring stopped. Resources cleaned up.")

    print("Battery detection complete.")