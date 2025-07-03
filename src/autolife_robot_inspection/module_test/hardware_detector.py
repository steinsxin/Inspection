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


class HardwareDetector(DeviceInterface):
    """
    A hardware detector class for detecting and logging system hardware info.
    """

    def __init__(self, config):
        self.config = config
        self.log = self.get_log()

    def get_wifi_info(self):
        """
        Get current Wi-Fi interface hardware information using lspci -nnk.
        Format output similar to other info sections.
        """
        try:
            output = subprocess.check_output("lspci -nnk", shell=True, text=True)

            # 组织为多个设备块
            blocks = []
            current_block = []

            for line in output.splitlines():
                if line and not line.startswith("\t") and current_block:
                    blocks.append("\n".join(current_block))
                    current_block = [line]
                else:
                    current_block.append(line)
            if current_block:
                blocks.append("\n".join(current_block))

            # 提取 Network controller 并格式化
            for block in blocks:
                if "Network controller" in block:
                    lines = ["[ Wi-Fi Information ]"]

                    # 提取主描述行，例如：
                    # 0001:01:00.0 Network controller [0280]: Realtek ... [10ec:c822]
                    header_line = block.splitlines()[0]
                    device_desc = header_line.split("]:", 1)[-1].strip()
                    id_match = header_line.split("[")[-1].strip("]")  # e.g., "10ec:c822"
                    vendor_id, device_id = id_match.split(":")

                    # 提取 driver 和 modules
                    driver = module = "Unknown"
                    for line in block.splitlines():
                        if "Kernel driver in use:" in line:
                            driver = line.split(":")[1].strip()
                        if "Kernel modules:" in line:
                            module = line.split(":")[1].strip()

                    lines.append(f"- Device         : {device_desc}")
                    lines.append(f"- Vendor ID      : {vendor_id}")
                    lines.append(f"- Device ID      : {device_id}")
                    lines.append(f"- Driver         : {driver}")
                    lines.append(f"- Kernel Module  : {module}")

                    return "\n".join(lines)

            return "[ Wi-Fi Information ]\n- No Wi-Fi interface found."

        except Exception as e:
            return f"[ Wi-Fi Information ]\n- Error           : {e}"
    
    def get_kernel_info(self):
        """
        Get basic kernel info using uname command.
        """
        try:
            kernel_info = subprocess.check_output(["uname", "-a"]).decode("utf-8").strip()
            parts = kernel_info.split()
            return (
                "[ Kernel Information ]\n"
                f"- Kernel Name    : {parts[0]}\n"
                f"- Node Name      : {parts[1]}\n"
                f"- Kernel Version : {parts[2]}"
            )
        except Exception as e:
            return f"[ Kernel Information ]\n- Error           : {e}"

    def get_memory_info(self):
        """
        Read total memory and CPU frequency from system files.
        """
        try:
            lines = ["[ Memory Information ]"]
            mem_total = 0

            with open("/proc/meminfo", "r") as meminfo:
                for line in meminfo:
                    if line.startswith("MemTotal:"):
                        mem_total = int(line.split()[1])
                        break

            mem_total_gb = mem_total / (1024 * 1024)
            lines.append(f"- Total Size     : {mem_total_gb:.2f} GB")

            try:
                with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", "r") as freq_file:
                    freq_khz = int(freq_file.read().strip())
                    freq_mhz = freq_khz / 1000
                    lines.append(f"- Frequency      : {freq_mhz:.2f} MHz")
            except Exception:
                lines.append("- Frequency      : Unknown")

            return "\n".join(lines)
        except Exception as e:
            return f"[ Memory Information ]\n- Error           : {e}"

    def get_cpu_info(self):
        """
        Get CPU model name, frequency, and architecture.
        """
        try:
            info = {}

            with open("/proc/cpuinfo", "r") as f:
                for line in f:
                    if "model name" in line:
                        info["Model Name"] = line.split(":")[1].strip()
                        break

            try:
                with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", "r") as f:
                    freq = int(f.read()) / 1000
                    info["Current freq"] = f"{freq:.2f} MHz"
            except Exception:
                info["Current freq"] = "Unknown"

            try:
                arch = subprocess.check_output("uname -m", shell=True, text=True).strip()
                info["Architecture"] = arch
            except Exception:
                info["Architecture"] = "Unknown"

            return (
                "[ CPU Information ]\n"
                f"- Model Name     : {info.get('Model Name', 'Unknown')}\n"
                f"- Current freq   : {info.get('Current freq')}\n"
                f"- Architecture   : {info.get('Architecture')}"
            )
        except Exception as e:
            return f"[ CPU Information ]\n- Error           : {e}"

    def get_disk_info(self):
        """
        Detect NVMe disk information using lsblk and smartctl/nvme.
        """
        try:
            output = subprocess.check_output("lsblk -d -o NAME,SIZE,MODEL", shell=True, text=True)
            lines = ["[ Disk Information ]"]
            found = False

            for line in output.splitlines()[1:]:
                parts = line.split()
                if parts and parts[0].startswith('nvme'):
                    name = f"/dev/{parts[0]}"
                    size = parts[1] if len(parts) > 1 else "Unknown"
                    model = " ".join(parts[2:]) if len(parts) > 2 else self._get_nvme_model(parts[0])
                    lines.extend([
                        f"- Device         : {name}",
                        f"  Model          : {model or 'Unknown'}",
                        f"  Size           : {size}"
                    ])
                    found = True

            if not found:
                lines.append("- No NVMe disk found.")

            return "\n".join(lines)
        except Exception as e:
            return f"[ Disk Information ]\n- Error           : {e}"

    def _get_nvme_model(self, disk):
        """
        Fallback method to detect NVMe model from smartctl or nvme CLI.
        """
        try:
            output = subprocess.check_output(
                f"sudo nvme list | grep {disk}", shell=True,
                stderr=subprocess.DEVNULL, text=True
            )
            return output.split()[2] if output else None
        except Exception:
            try:
                output = subprocess.check_output(
                    f"sudo smartctl -i /dev/{disk}", shell=True,
                    stderr=subprocess.DEVNULL, text=True
                )
                return re.search(r"Model Number:\s*(.+)", output).group(1).strip()
            except Exception:
                return None

    def get_keyboard_data(self, keyboard_data):
        """
        Placeholder for keyboard detection.
        """
        return None

    def get_log(self):
        """
        Assemble a complete log report with all hardware sections.
        """
        sections = [
            "========================= Hardware Detection Report =========================",
            self.get_kernel_info(),
            self.get_disk_info(),
            self.get_cpu_info(),
            self.get_memory_info(),
            self.get_wifi_info(), 
            "============================================================================"
        ]
        return "\n\n".join(sections)

    def run(self):
        """
        Perform a single hardware detection and update the log.
        """
        self.log = self.get_log()
        return None


if __name__ == "__main__":
    detector = HardwareDetector()
    print(detector.log)

    try:
        while True:
            detector.run()
    except KeyboardInterrupt:
        print("\nHardware monitoring interrupted by user.")
    finally:
        print("Hardware monitoring stopped. Resources cleaned up.")

    print("Hardware detection complete.")
