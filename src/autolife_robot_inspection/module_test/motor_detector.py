import io
import json
import os
import re
import sys
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

class MotorDetector(DeviceInterface):
    """
    A motor detector class that initializes motor modules and
    parses output logs to detect status.
    """

    def __init__(self, config):
        """Initialize detector and load configuration."""
        self.config = config
        self.robot_api = RobotInterface()
        self.log_buffer = []
        self.motor_map = {
            "Joint_Ankle": 0,
            "Joint_Knee": 1,
            "Joint_Waist_Pitch": 2,
            "Joint_Waist_Yaw": 3,
            "Joint_Left_Shoulder_Inner": 1,
            "Joint_Left_Shoulder_Outer": 2,
            "Joint_Left_UpperArm": 3,
            "Joint_Left_Elbow": 4,
            "Joint_Left_Forearm": 5,
            "Joint_Left_Wrist_Upper": 6,
            "Joint_Left_Wrist_Lower": 7,
            "Joint_Right_Shoulder_Inner": 1,
            "Joint_Right_Shoulder_Outer": 2,
            "Joint_Right_UpperArm": 3,
            "Joint_Right_Elbow": 4,
            "Joint_Right_Forearm": 5,
            "Joint_Right_Wrist_Upper": 6,
            "Joint_Right_Wrist_Lower": 7,
        }


    def _load_config(self):
        """Load configuration from config.json."""
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_path = os.path.join(project_root, 'config', 'config.json')
        with open(config_path, 'r', encoding='utf-8') as f:
            return json.load(f)

    def _check_single(self, motor_part):
        """
        Check a single motor module in a separate thread.
        Captures and stores the output log.
        """
        output_buffer = io.StringIO()

        def task():
            with redirect_stdout(output_buffer):
                robot_api = RobotInterface()
                robot_api.initialize([motor_part])

        thread = threading.Thread(target=task)
        thread.start()
        thread.join()

        output = output_buffer.getvalue()
        parsed_log = self._parse_log(output)
        self.log_buffer.append(parsed_log)

    def _parse_log(self, output):
        """
        Parse motor SDK output to detect success/failure per module.
        """
        result = []

        # 成功注册的模块（兼容 LK / EU / 任意 Motor Controller）
        success_pattern = re.compile(
            r"(mod_motor_[\w\-]+):\s+\w+\s+Motor Controller registered on (PCAN_USBBUS\d+)",
            re.IGNORECASE
        )

        # 电机无响应（指定 ID、关节名和 CAN 总线）
        no_response_pattern = re.compile(
            r"(mod_motor_[\w\-]+):\s*电机 ID\s+(\d+),\s*Joint Name:\s*([\w\-]+)\s*在\s*(PCAN_USBBUS\d+)\s*没有响应"
        )

        # 模块完全连接失败
        general_failure_pattern = re.compile(
            r"(mod_motor_[\w\-]+):\s*电机连接失败"
        )

        found = False

        for mod, bus in success_pattern.findall(output):
            result.append(f"模块 {mod:<22} | 状态: 正常             | 总线: {bus}")
            found = True

        for mod, motor_id, joint_name, bus in no_response_pattern.findall(output):
            result.append(
                f"模块 {mod:<22} | 电机 ID: {motor_id:<2} ({joint_name}) 无响应 | 总线: {bus}"
            )
            found = True

        for mod in general_failure_pattern.findall(output):
            result.append(f"模块 {mod:<22} | 状态: 电机连接失败")

        if not found:
            return "未检测到电机模块或日志格式异常。\n"

        return "\n".join(result)

    def run(self):
        """Run motor detection for all motor modules."""
        return None

    def get_log(self):
        """Concatenate and return all logs."""
        return "\n".join(self.log_buffer)
        
    def get_keyboard_data(self, keyboard_data):
        return None

if __name__ == "__main__":
    detector = MotorDetector()
    detector.run()
    print(detector.get_log())
