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

class AudioDetector(DeviceInterface):
    """
    A microphone detector class that initializes audio devices and
    runs an optional loopback playback of audio.
    """

    def __init__(self, config):
        """Initialize by loading config and detecting audio devices."""
        self.config = config
        self.volume = 100  # Initial volume (0-100)
        self.robot_api = RobotInterface()
        self.device_info = self._detect_devices()
        self.log = self.get_log()

    def volume_control(self, change=0):
        """Adjust the volume by a specified amount (positive or negative)."""
        self.volume = max(0, min(100, self.volume + change))  # Ensure volume stays within 0-100 range
        self.robot_api.speaker_set_volume(self.volume)

    def _detect_devices(self):
        """
        Detect audio and speaker devices from SDK initialization logs.
        """
        buffer = io.StringIO()
        with redirect_stdout(buffer):
            self.robot_api.initialize(self.config['audio']['ENABLED_MODULES'])
            self.robot_api.speaker_set_volume(self.volume)

        output = buffer.getvalue()

        audio_match = re.search(r"Found audio device:\s*([^\(]+)", output)
        speaker_match = re.search(r"Found speaker device:\s+(\S+)", output)

        return {
            "audio": {
                "found": bool(audio_match),
                "name": audio_match.group(1).strip() if audio_match else None
            },
            "speaker": {
                "found": bool(speaker_match),
                "name": speaker_match.group(1) if speaker_match else None
            }
        }

    def get_keyboard_data(self, keyboard_data):
        if keyboard_data == 'plus':
            self.volume_control(change=5)  # Increase volume by 5%
        elif keyboard_data == 'minus':
            self.volume_control(change=-5)  # Decrease volume by 5%

        return None

    def get_log(self):
        """Format and return the detection result as a log string."""
        log = "\n=== Audio Device Detection Results ===\n"
        log += f"Microphone: {self.device_info['audio']['name'] or 'Not detected'}\n"
        log += f"Speaker: {self.device_info['speaker']['name'] or 'Not detected'}\n"
        log += f"Current Volume: {self.volume}%\n"
        log += "Press '+' to increase volume\n"  # 按 + 提高音量
        log += "Press '-' to decrease volume\n"  # 添加提示：按 - 降低音量
        log += "=" * 25 + "\n"
        return log

    def run(self):
        """
        Run loopback audio from microphone to speaker.
        Intended to run in a subprocess.
        """
        try:

            audio_data = self.robot_api.get_microphone_audio_data()
            time.sleep(0.02)
            if audio_data:
                self.robot_api.speaker_play_audio_data(audio_data)
        except Exception as e:
            print(f"Error processing audio: {e}")

    def interactive_volume_control(self):
        """
        Provide an interactive interface to control volume using arrow keys.
        Note: This is a simple implementation. For full arrow key support,
        you might need a library like 'curses' or 'keyboard'.
        """
        print("Interactive Volume Control (Press 'q' to quit)")
        print("Use UP arrow to increase volume, DOWN arrow to decrease volume")
        
        # This is a simple implementation without proper arrow key detection
        # For a real application, consider using the 'curses' or 'keyboard' library
        while True:
            user_input = input("Enter '+' to increase volume, '-' to decrease volume, 'q' to quit: ")
            
            if user_input == 'q':
                break
            elif user_input == '+':
                self.volume_control(change=5)  # Increase volume by 5%
            elif user_input == '-':
                self.volume_control(change=-5)  # Decrease volume by 5%

if __name__ == "__main__":
    path = "../../configs/model_config.json"
    with open(path, "r", encoding="utf-8") as f:
        config = json.load(f)
    detector = AudioDetector(config)
    print(detector.log)
    
    # Start interactive volume control
    # detector.interactive_volume_control()
    
    try:
        while True:
            detector.run()
    except KeyboardInterrupt:
        print("Loopback interrupted.")
    finally:
        detector.robot_api.close(detector.config['audio']['ENABLED_MODULES'])
        print("Stopped audio loopback.")

    print("Audio detection complete.")