import json
import logging
import os
import re
import socket
import subprocess
import sys
import threading
import time
import traceback
from datetime import datetime
from multiprocessing import Event

from textual.app import App, ComposeResult
from textual.containers import Container, VerticalScroll
from textual.css.query import NoMatches
from textual.reactive import reactive
from textual.screen import Screen
from textual.widgets import Header, Footer, Label, Button

from autolife_robot_inspection import (
    MODEL_CONFIG_PATH, MENU_CONFIG_PATH, FUNC_CONFIG_PATH, UI_CSS_PATH
)
from autolife_robot_inspection.inspection_detector import InspectionDetector


class LogScreen(Screen):
    BINDINGS = [
        ("b", "stop_process", "Back"),
        ("escape", "stop_process", "Back"),
    ]

    log_content = reactive("")

    def __init__(self, stop_event: threading.Event, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stop_event = stop_event
        self.capturing_keyboard = False
        self.input_queue = None
        self.input_buffer = []
        self._external_log = ""  # 外部设置的最新日志内容
        self._log_timer: Timer | None = None

    def compose(self) -> ComposeResult:
        yield Header()
        with Container(id="log-container"):
            with VerticalScroll(id="output_scroll", classes="output-display"):
                yield Label("", id="output_label")
            yield Label("", id="input_line_label")
        yield Footer()

    def on_mount(self) -> None:
        # 每 0.5 秒刷新一次 log_content
        self._log_timer = self.set_interval(0.5, self.refresh_log)
        
    def on_unmount(self) -> None:
        if self._log_timer:
            self._log_timer.stop()

    def refresh_log(self):
        """定时器回调函数：从外部缓存更新到 log_content，触发 watch。"""
        self.log_content = self._external_log

    def set_log(self, full_log: str) -> None:
        """由外部调用，设置新的完整日志内容。"""
        self._external_log = full_log

    def watch_log_content(self, old: str, new: str) -> None:
        try:
            self.query_one("#output_label").update(new)
            self.query_one("#output_scroll").scroll_end()
        except NoMatches:
            pass    

    def on_key(self, event: 'Key') -> None:
        if self.capturing_keyboard and self.input_queue:
            self.input_queue.put(event.key)

        # Update the visual input line for user feedback
        if event.key == 'enter':
            self.input_buffer.clear()
        elif event.key == 'backspace':
            if self.input_buffer:
                self.input_buffer.pop()
        elif event.is_printable:
            self.input_buffer.append(event.key)

    def start_keyboard_capture(self, q: 'Queue'):
        """Called by the detector to start capturing keyboard input."""
        self.input_queue = q
        self.capturing_keyboard = True
        self.input_buffer = []

    def stop_keyboard_capture(self):
        """Called by the detector to stop capturing keyboard input."""
        self.capturing_keyboard = False
        self.input_queue = None
        self.input_buffer = None

    def action_stop_process(self) -> None:
        if self.stop_event:
            self.stop_event.set()
            self.notify("Stopping inspection...")
        self.app.pop_screen() 


class InspectionUI(App):
    CSS_PATH = UI_CSS_PATH
    BINDINGS = [
        ("escape", "quit", "Exit"),
        ("p", "switch_language", "Switch Language"),
        ("b", "go_back", "Back to Main"),
    ]

    language = reactive("zh")
    current_menu = reactive("Jetson")
    hostname = reactive("")
    device_type = reactive("")
    text = reactive({})
    _mounted = reactive(False)

    def __init__(self, language="zh", *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.language = language
        self.hostname = socket.gethostname()
        self.device_type = self._detect_device_type()
        self.detector = InspectionDetector(language)
        self.menu_config = self._load_json_config(MENU_CONFIG_PATH)
        self.func_config = self._load_json_config(FUNC_CONFIG_PATH)
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
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            logging.error(f"Failed to load config '{path}': {e}")
            raise

    def update_ros_domain_id(self, hostname, config_file_path):
        match = re.search(r"\d+", hostname)
        if not match:
            self.notify("No number found in hostname.")
            return

        domain_id = int(match.group())
        try:
            with open(config_file_path, "r") as file:
                config_data = json.load(file)

            config_data["ROS_DOMAIN_ID"] = domain_id

            with open(config_file_path, "w") as file:
                json.dump(config_data, file, indent=2)

            self.notify(f"ROS_DOMAIN_ID updated to {domain_id}")
        except FileNotFoundError:
            self.notify(f"File {config_file_path} not found.")
        except json.JSONDecodeError:
            self.notify(f"File {config_file_path} is not valid JSON.")
        except Exception as e:
            self.notify(f"Error updating file: {e}")

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
        self.query_one("#title", Label).update(self.text["title"])
        self.query_one("#hostname", Label).update(
            f"{self.text['hostname']}: {self.hostname}"
        )
        self.query_one("#device", Label).update(
            f"{self.text['device']}: {self.device_type}"
        )
        self.query_one("#quit_language_info", Label).update("")

        container = self.query_one("#options_container", VerticalScroll)
        container.remove_children()

        for i, option_text in enumerate(self.text["options"]):
            button = Button(
                option_text.split(". ", 1)[-1],
                id=f"{self.current_menu}_option_{i+1}",
                classes="menu-option"
            )
            container.mount(button)

        self.query_one("#back_info", Label).update(
            self.text["back"] if self.current_menu != "Jetson" else ""
        )

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if not event.button.id or "_option_" not in event.button.id:
            return

        key = event.button.id.split("_option_")[-1]
        actions = self._get_current_actions()

        if key not in actions:
            return

        action = actions[key]
        if "menu" in action:
            self.current_menu = action["menu"]
        elif "function" in action:
            func = getattr(self.detector, action["function"], None)
            if callable(func):
                self.stop_event = threading.Event()
                log_screen = LogScreen(stop_event=self.stop_event)
                self.push_screen(log_screen)
                self.detector.set_log_screen(log_screen)
                self.run_in_thread(func, stop_event=self.stop_event)

    def action_stop_process(self) -> None:
        if hasattr(self, "stop_event") and self.stop_event:
            self.stop_event.set()
            self.notify("Stopping process...")
            self.app.pop_screen()

    def action_quit(self) -> None:
        self.exit(f"\n{self.text['exiting']}")

    def action_switch_language(self) -> None:
        self.language = "zh" if self.language == "en" else "en"
        self.notify(self.text["language_switched"])

    def action_go_back(self) -> None:
        if self.current_menu != "Jetson":
            self.current_menu = "Jetson"

    def _load_text(self):
        text = self.menu_config[self.language].copy()

        if self.current_menu == "ModuleTest":
            device_info = self.func_config.get(self.device_type, {})
            options = device_info.get("options", {}).get(self.language, [])
        else:
            options = self.menu_config[self.language]["options"].get(
                self.current_menu, []
            )

        text["options"] = options
        return text

    def _get_current_actions(self):
        if self.current_menu == "ModuleTest":
            device_info = self.func_config.get(self.device_type, {})
            return device_info.get("actions", {})
        return self.menu_config["actions"].get(self.current_menu, {})

    def _detect_device_type(self):
        upper_hostname = self.hostname.upper()
        if "NX" in upper_hostname:
            return "Jetson NX"
        if "NANO" in upper_hostname:
            return "Jetson NANO"
        if "ORANGEPI" in upper_hostname or "PI" in upper_hostname:
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
    app = InspectionUI(language="zh")
    app.run()
