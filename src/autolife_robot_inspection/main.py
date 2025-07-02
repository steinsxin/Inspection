import json
import re
import multiprocessing
import os
import select
import socket
import subprocess
import sys
import time
import logging
import traceback
import threading
from datetime import datetime
from autolife_robot_inspection import MODEL_CONFIG_PATH, MENU_CONFIG_PATH, FUNC_CONFIG_PATH
from autolife_robot_inspection.inspection_detector import InspectionDetector

from textual.app import App, ComposeResult
from textual.containers import Container, VerticalScroll
from textual.widgets import Header, Footer, Label, Button, Switch
from textual.reactive import reactive

from textual.screen import Screen
from textual.css.query import NoMatches

class LogScreen(Screen):
    BINDINGS = [
        ("b", "app.pop_screen", "返回"),
        ("escape", "app.pop_screen", "返回"),
        ("s", "action_stop_process", "停止"),
    ]

    log_content = reactive("")  # 使用 reactive 自动触发 UI 更新

    def __init__(self, stop_event: threading.Event, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stop_event = stop_event

    def compose(self) -> ComposeResult:
        yield Header()
        with Container(id="log-container"):
            with VerticalScroll(id="output_scroll", classes="output-display"):
                yield Label("", id="output_label")  # 初始为空
            yield Button("停止", id="stop_button", classes="-hidden")
        yield Footer()

    def watch_log_content(self, old: str, new: str) -> None:
        """log_content 变化时自动更新 UI"""
        try:
            self.query_one("#output_label").update(new)  # 更新 Label 内容
            self.query_one("#output_scroll").scroll_end()  # 自动滚动到底部
        except NoMatches:
            pass  # 忽略未加载的情况

    def append_log(self, message: str) -> None:
        """添加日志（同时更新 UI）"""
        self.log_content = f"{self.log_content}\n{message}"  # 创建新字符串以触发 reactive

    def action_stop_process(self) -> None:
        """停止当前检测进程"""
        if self.stop_event:
            self.stop_event.set()
            self.notify("正在停止检测...")
            self.app.pop_screen()

class InspectionUI(App):
    CSS_PATH = "mouse_interaction.tcss"
    BINDINGS = [
        ("escape", "quit", "退出程序"),
        ("w", "switch_language", "切换语言"),
        ("b", "go_back", "返回主菜单"),
    ]

    language = reactive("zh")
    current_menu = reactive("Jetson")
    hostname = reactive("")
    device_type = reactive("")
    text = reactive({})
    _mounted = reactive(False)

    def __init__(self, language='zh', *args, **kwargs):
        self.menu_config = self._load_json_config(MENU_CONFIG_PATH)
        self.func_config = self._load_json_config(FUNC_CONFIG_PATH)
        super().__init__(*args, **kwargs)
        self.language = language
        self.hostname = "NX-100" # For testing purposes
        self.device_type = self._detect_device_type()
        self.detector = InspectionDetector(language)
        self.update_ros_domain_id(self.hostname, MODEL_CONFIG_PATH)

    def compose(self) -> ComposeResult:
        yield Header()
        with Container(id="main-container"):
            yield Label(id="title")
            yield Label(id="hostname")
            yield Label(id="device")
            yield Label(id="quit_language_info")
            yield VerticalScroll(id="options_container")
            yield Label(id="back_info")
        yield Footer()

    def _load_json_config(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            logging.error(f"Failed to load config '{path}': {e}")
            raise

    def update_ros_domain_id(self, hostname, config_file_path):
        """
        Extract a number from the hostname and update the ROS_DOMAIN_ID in the specified JSON configuration file.

        Parameters:
            hostname (str): The hostname string containing a number, e.g., "NX-104", "nano-104", or "orangepi-104".
            config_file_path (str): The path to the JSON configuration file.

        Returns:
            None
        """
        match = re.search(r'\d+', hostname)
        if match:
            # Extract the number from the hostname
            extracted_number = int(match.group())
            
            try:
                # Open and read the JSON configuration file
                with open(config_file_path, 'r') as file:
                    config_data = json.load(file)
                
                # Update the ROS_DOMAIN_ID
                config_data['ROS_DOMAIN_ID'] = extracted_number
                
                # Write the updated data back to the file
                with open(config_file_path, 'w') as file:
                    json.dump(config_data, file, indent=2)
                
                self.notify(f"ROS_DOMAIN_ID has been successfully updated to {extracted_number}")
            except FileNotFoundError:
                self.notify(f"File {config_file_path} not found. Please check the path.")
            except json.JSONDecodeError:
                self.notify(f"File {config_file_path} is not a valid JSON format.")
            except Exception as e:
                self.notify(f"Error occurred while updating the file: {e}")
        else:
            self.notify("No number found in the hostname.")

    

    def on_mount(self) -> None:
        self.text = self._load_text()
        self.update_ui()
        self._mounted = True

    def watch_language(self, language: str) -> None:
        if self._mounted:
            self.text = self._load_text()
            self.update_ui()

    def watch_current_menu(self, current_menu: str) -> None:
        if self._mounted:
            self.text = self._load_text()
            self.update_ui()

    

    def update_ui(self) -> None:
        self.query_one("#title", Label).update(self.text['title'])
        self.query_one("#hostname", Label).update(f"{self.text['hostname']}: {self.hostname}")
        self.query_one("#device", Label).update(f"{self.text['device']}: {self.device_type}")
        self.query_one("#quit_language_info", Label).update("")

        options_container = self.query_one("#options_container", VerticalScroll)
        
        # 方法2：完全重建
        options_container.remove_children()
        for i, option_text in enumerate(self.text['options']):
            button = Button(option_text.split('. ', 1)[-1], 
                        id=f"{self.current_menu}_option_{i+1}",  # 确保唯一性
                        classes="menu-option")
            options_container.mount(button)
        
        if self.current_menu != 'Jetson':
            self.query_one("#back_info", Label).update(self.text['back'])
        else:
            self.query_one("#back_info", Label).update("")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id and "_option_" in event.button.id:
            key = event.button.id.split("_option_")[-1]
            actions = self._get_current_actions()
            if key in actions:
                action = actions[key]
                if 'menu' in action:
                    self.current_menu = action['menu']
                elif 'function' in action:
                    func = getattr(self.detector, action['function'], None)
                    if callable(func):
                        self.stop_event = threading.Event()
                        log_screen = LogScreen(stop_event=self.stop_event)
                        self.notify(f"LogScreen created: {'OK' if log_screen else 'FAILED'}")
                        self.push_screen(log_screen)
                        self.detector.set_log_screen(log_screen)
                        self.notify(f"Detector log screen set: {'OK' if self.detector._log_screen else 'FAILED'}")
                        self.run_in_thread(func, stop_event=self.stop_event)

    def action_stop_process(self) -> None:
        if hasattr(self, 'stop_event') and self.stop_event:
            self.stop_event.set()
            self.notify("Stopping process...")
            self.app.pop_screen()
            

    def action_quit(self) -> None:
        self.exit(f"\n{self.text['exiting']}")

    def action_switch_language(self) -> None:
        self.language = 'zh' if self.language == 'en' else 'en'
        self.notify(f"{self.text['language_switched']}")

    def action_go_back(self) -> None:
        if self.current_menu != 'Jetson':
            self.current_menu = 'Jetson'

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

    def run_in_thread(self, func, *args, stop_event=None, **kwargs):
        def wrapper():
            try:
                func(*args, stop_event=stop_event, **kwargs)
            except Exception as e:
                logging.error(f"Error in threaded function {func.__name__}: {e}")
                self.call_from_thread(self.notify, f"Error: {e}")
            finally:
                self.call_from_thread(self.app.pop_screen)
        
        thread = threading.Thread(target=wrapper)
        thread.daemon = True
        thread.start()

if __name__ == "__main__":
    # 默认以中文启动
    app = InspectionUI(language='zh')
    app.run()