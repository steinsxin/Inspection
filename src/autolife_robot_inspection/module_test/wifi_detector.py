import io
import json
import os
import re
import sys
import time
import subprocess
import traceback
from contextlib import redirect_stdout

# Add parent directory to sys.path for module resolution
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from autolife_robot_inspection.interface import DeviceInterface

class WifiDetector(DeviceInterface):
    """A wifi detector class for detecting and logging WiFi information."""

    def __init__(self, config):
        """Initialize the WifiDetector."""
        self.config = config
        self.log = self.get_log()

    def _scan_wifi_networks(self):
        """Scan for available WiFi networks and return top 5 with signal strength."""
        try:
            # Use nmcli to scan WiFi networks
            scan_cmd = ["nmcli", "-t", "-f", "SSID,SIGNAL", "device", "wifi", "list"]
            result = subprocess.run(scan_cmd, capture_output=True, text=True, check=True)
            
            # Parse the output
            networks = []
            for line in result.stdout.splitlines():
                if line.strip():
                    parts = line.split(":")
                    if len(parts) >= 2:
                        ssid = parts[0]
                        signal = parts[1]
                        networks.append({"ssid": ssid, "signal": f"{signal}%"})
            
            # Sort by signal strength (descending) and get top 5
            networks.sort(key=lambda x: int(x["signal"].replace("%", "")), reverse=True)
            return networks[:5]
            
        except subprocess.CalledProcessError as e:
            print(f"WiFi scan failed: {e.stderr}")
            return []
        except Exception as e:
            print(f"Unexpected error during WiFi scan: {str(e)}")
            return []

    def _get_current_connection(self):
        """Check current WiFi connection status."""
        try:
            # Check if WiFi is connected
            status_cmd = ["nmcli", "-t", "-f", "GENERAL.STATE", "device", "show", "wlan0"]
            result = subprocess.run(status_cmd, capture_output=True, text=True, check=True)
            
            if "connected" in result.stdout.lower():
                # Get connected SSID
                ssid_cmd = ["nmcli", "-t", "-f", "GENERAL.CONNECTION", "device", "show", "wlan0"]
                ssid_result = subprocess.run(ssid_cmd, capture_output=True, text=True, check=True)
                return {
                    "connected": True,
                    "ssid": ssid_result.stdout.strip().split(":")[-1]
                }
            return {"connected": False, "ssid": None}
            
        except subprocess.CalledProcessError:
            return {"connected": False, "ssid": None}
        except Exception as e:
            print(f"Error checking connection status: {str(e)}")
            return {"connected": False, "ssid": None}

    def get_log(self):
        """Assemble a complete WiFi status report."""
        # Get network info
        networks = self._scan_wifi_networks()
        connection = self._get_current_connection()
        
        report_lines = [
            "========================= WiFi Detection Report =========================",
            f"Current WiFi Status: {'Connected' if connection['connected'] else 'Disconnected'}",
        ]
        
        if connection['connected']:
            report_lines.append(f"Connected to: {connection['ssid']}")
        
        report_lines.append("\nTop 5 Available WiFi Networks:")
        
        if networks:
            for i, network in enumerate(networks, 1):
                report_lines.append(f"{i}. {network['ssid']} - Signal: {network['signal']}")
        else:
            report_lines.append("No WiFi networks detected")
            
        report_lines.append("============================================================================")

        return "\n".join(report_lines)

    def get_keyboard_data(self, keyboard_data):
        """Placeholder for keyboard detection."""
        return None

    def run(self):
        """Idle loop to simulate ongoing monitoring."""
        while True:
            time.sleep(1)
        return None


if __name__ == "__main__":
    detector = WifiDetector()
    print(detector.log)

    try:
        while True:
            detector.run()
    except KeyboardInterrupt:
        print("\nWiFi monitoring interrupted by user.")
    finally:
        print("WiFi monitoring stopped. Resources cleaned up.")

    print("WiFi detection complete.")