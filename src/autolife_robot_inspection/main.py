import json
import multiprocessing
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
from autolife_robot_inspection import MODEL_CONFIG_PATH, MENU_CONFIG_PATH, FUNC_CONFIG_PATH
from autolife_robot_inspection.inspection_detector import InspectionDetector

class InspectionUI:
    def __init__(self, language='en'):
        """Initialize UI with menu config, device info, and default state."""
        self.language = language
        self.current_menu = 'Jetson'
        self.border_width = 50

        try:
            with open(MENU_CONFIG_PATH, 'r', encoding='utf-8') as f:
                self.menu_config = json.load(f)
        except Exception as e:
            logging.error(f"Failed to load config '{MENU_CONFIG_PATH}': {e}")
            raise

        try:
            with open(FUNC_CONFIG_PATH, 'r', encoding='utf-8') as f:
                self.func_config = json.load(f)
        except Exception as e:
            logging.error(f"Failed to load config '{FUNC_CONFIG_PATH}': {e}")
            raise

        self.hostname = socket.gethostname()
        self.device_type = self._detect_device_type()
        self.detector = InspectionDetector(language)
        self.text = self._load_text()

    def start(self):
        """Start the UI loop."""
        self.display_ui()
        self._wait_for_key_press()

    def display_ui(self):
        """Render the menu interface."""
        os.system('cls' if os.name == 'nt' else 'clear')
        border = "*" * self.border_width

        title_line = self._center_text(self.text['title'], 46)
        hostname_line = self._center_text(f"{self.text['hostname']}: {self.hostname}", 46)
        device_line = self._center_text(f"{self.text['device']}: {self.device_type}", 46)

        print(f"\n{border}")
        print(f"* {title_line} *")
        print(f"* {hostname_line} *")
        print(f"* {device_line} *")
        print(f"{border}\n")

        print(f"{self.text['quit']}\n{self.text['language']}\n")
        for option in self.text['options']:
            print(option)

        if self.current_menu != 'Jetson':
            print(f"\n{self.text['back']}")

    def reset_ui(self):
        """Short delay and refresh UI."""
        time.sleep(1)
        self.display_ui()

    def _wait_for_key_press(self):
        """Main input loop for handling key presses and triggering actions."""
        while True:
            key = self._getch()
            key_lower = key.lower()

            if key_lower == 'q':
                print(f"\n{self.text['exiting']}")
                break
            elif key_lower == 'w':
                self._switch_language()
            elif key_lower == 'b' and self.current_menu != 'Jetson':
                self.current_menu = 'Jetson'
                self.text = self._load_text()
                self.display_ui()
            else:
                actions = self._get_current_actions()
                if key in actions:
                    action = actions[key]
                    if 'menu' in action:
                        self.current_menu = action['menu']
                        self.text = self._load_text()
                        self.display_ui()
                    elif 'function' in action:
                        func = getattr(self.detector, action['function'], None)
                        print(func)
                        if callable(func):
                            option_text = self.text['options'][int(key) - 1].split('. ')[1]
                            print(f"\n{option_text}: {self.text['running']}")
                            func()
                            self.reset_ui()

    def _switch_language(self):
        """Toggle between English and Chinese UI language."""
        self.language = 'zh' if self.language == 'en' else 'en'
        self.text = self._load_text()
        print(f"\n{self.text['language_switched']}")
        self.display_ui()

    def _load_text(self):
        """Load text elements from menu config for the current menu and language."""
        text = self.menu_config[self.language].copy()

        if self.current_menu == 'ModuleTest':
            device_info = self.func_config.get(self.device_type, {})
            options = device_info.get('options', {}).get(self.language, [])
        else:
            options = self.menu_config[self.language]['options'].get(self.current_menu, [])
        
        text['options'] = options
        return text

    def _get_current_actions(self):
        """Return key-action mappings based on current menu and device."""
        if self.current_menu == 'ModuleTest':
            device_info = self.func_config.get(self.device_type, {})
            return device_info.get('actions', {})
        return self.menu_config['actions'].get(self.current_menu, {})

    def _detect_device_type(self):
        """Infer the device type based on the hostname."""
        upper_hostname = self.hostname.upper()
        if "NX" in upper_hostname:
            return "Jetson NX"
        if "NANO" in upper_hostname:
            return "Jetson NANO"
        if "ORANGEPI" in upper_hostname:
            return "Orangepi"
        return "Unknown"

    def _center_text(self, text, width):
        """Return text centered with padding; handles double-width characters."""
        display_width = sum(2 if '\u4e00' <= c <= '\u9fff' else 1 for c in text)
        if display_width >= width:
            return text
        padding = width - display_width
        return ' ' * (padding // 2) + text + ' ' * (padding - padding // 2)

    def _getch(self):
        """Read a single character from stdin without echo."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    # 默认以中文启动
    UI = InspectionUI(language='zh')
    UI.start()