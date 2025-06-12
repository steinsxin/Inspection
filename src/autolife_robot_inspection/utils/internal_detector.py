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


class InternalDetector(DeviceInterface):
    """A internal detector class for detecting and logging system internal info."""

    def __init__(self, config):
        """Initialize the InternalDetector."""
        self.config = config
        self.log = self.get_log()

    def _parse_nmap_output(self, output: str):
        """Parse nmap scan output to extract device information.
        
        Args:
            output: String containing the nmap scan output
            
        Returns:
            dict: Dictionary with parsed device information
        """
        device_info = {
            "hosts": []
        }
        
        # Regular expression to match host information
        host_pattern = re.compile(
            r"Nmap scan report for (?P<hostname>[\w.-]*)\s*\((?P<ip>[\d.]+)\)\s*"
            r"Host is up \((?P<rtt>[\d.]+)s latency\)\."
        )
        
        # Match cases without hostname
        host_pattern_no_hostname = re.compile(
            r"Nmap scan report for (?P<ip>[\d.]+)\s*"
            r"Host is up \((?P<rtt>[\d.]+)s latency\)\."
        )

        # Find all matches
        for match in host_pattern.finditer(output):
            host_info = match.groupdict()
            if host_info["ip"] != "192.168.10.1":  # Filter out 192.168.10.1
                device_info["hosts"].append({
                    "hostname": host_info["hostname"],
                    "ip": host_info["ip"],
                    "latency": host_info["rtt"]
                })

        # Handle cases without hostname
        for match in host_pattern_no_hostname.finditer(output):
            host_info = match.groupdict()
            if host_info["ip"] != "192.168.10.1":  # Filter out 192.168.10.1
                device_info["hosts"].append({
                    "hostname": None,  # No hostname
                    "ip": host_info["ip"],
                    "latency": host_info["rtt"]
                })

        return device_info

    def nmap_scan(self, target):
        """Perform nmap scan and return parsed results."""
        try:
            # Construct nmap command
            command = ["nmap", "-sn", target]
            
            # Execute command and capture output
            result = subprocess.run(command, capture_output=True, text=True, check=True)
            
            # Parse output results
            return self._parse_nmap_output(result.stdout)
            
        except subprocess.CalledProcessError as e:
            print(f"Nmap scan failed: {e.stderr}")
            return {"hosts": []}  # Return empty result on error
        except Exception as e:
            print(f"Unexpected error during nmap scan: {str(e)}")
            return {"hosts": []}  # Return empty result on error

    def get_log(self):
        """Assemble a complete log report with all internal sections."""
        nmap_result = self.nmap_scan("192.168.10.0/24")

        # Print parsed results
        report_lines = [
            "========================= Internal Detection Report ========================="
        ]
        
        if nmap_result and "hosts" in nmap_result and nmap_result["hosts"]:
            for host in nmap_result["hosts"]:
                hostname = host["hostname"] if host["hostname"] else "N/A"
                report_lines.append(
                    f"Host: {hostname} ({host['ip']}) - Latency: {host['latency']}s"
                )
        else:
            report_lines.append("No hosts detected (excluding 192.168.10.1).")
            
        report_lines.append(
            "============================================================================"
        )

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
    detector = InternalDetector()
    print(detector.log)

    try:
        while True:
            detector.run()
    except KeyboardInterrupt:
        print("\nInternal monitoring interrupted by user.")
    finally:
        print("Internal monitoring stopped. Resources cleaned up.")

    print("Internal detection complete.")