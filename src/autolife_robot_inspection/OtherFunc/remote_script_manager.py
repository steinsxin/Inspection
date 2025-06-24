import argparse
from paramiko import SSHClient, AutoAddPolicy

class RemoteScriptManager:
    """Manage execution and termination of a Python script on a remote device."""

    def __init__(self, hostname, port, username, password, python_path="python"):
        """Initialize connection information for the remote device.
        
        Args:
            hostname: IP address of the remote device
            port: SSH port number
            username: SSH username
            password: SSH password
            python_path: Path to Python interpreter (default: 'python')
        """
        self.hostname = hostname
        self.port = port
        self.username = username
        self.password = password
        self.python_path = python_path
        self.ssh_client = None

    def _connect(self):
        """Establish an SSH connection to the remote device."""
        self.ssh_client = SSHClient()
        self.ssh_client.set_missing_host_key_policy(AutoAddPolicy())
        self.ssh_client.connect(
            hostname=self.hostname,
            port=self.port,
            username=self.username,
            password=self.password
        )

    def _disconnect(self):
        """Close the SSH connection."""
        if self.ssh_client:
            self.ssh_client.close()
            print("SSH connection closed.")

    def _is_script_running(self, script_path):
        """Check if script is running on remote device."""
        check_cmd = f"pgrep -f '{self.python_path}.*{script_path}'"
        _, stdout, _ = self.ssh_client.exec_command(check_cmd)
        return bool(stdout.read().decode().strip())

    def run_script(self, script_path, log_path):
        """Run a Python script on the remote device."""
        try:
            self._connect()
            
            if self._is_script_running(script_path):
                print(f"Script {script_path} is already running.")
                return

            cmd = (
                f"nohup chrt -r 99 {self.python_path} -u -m {script_path} "
                f"> {log_path} 2>&1 &"
            )
            self.ssh_client.exec_command(cmd)
            print(f"Script {script_path} started successfully.")
        except Exception as e:
            print(f"Error starting script: {e}")
        finally:
            self._disconnect()

    def stop_arm_script(self):
        """Stop all Python processes on ARM device (original logic)."""
        try:
            self._connect()
            kill_cmd = "killall python"
            stdin, stdout, stderr = self.ssh_client.exec_command(kill_cmd)
            
            exit_status = stdout.channel.recv_exit_status()
            if exit_status == 0:
                print("All Python processes stopped successfully.")
            else:
                error = stderr.read().decode().strip()
                if "no process found" in error.lower():
                    print("No Python processes were running.")
                else:
                    print(f"Error stopping Python processes: {error}")
        except Exception as e:
            print(f"Error stopping ARM script: {e}")
        finally:
            self._disconnect()

    def stop_vision_script(self):
        """Stop vision script specifically (original logic)."""
        script_path = "autolife_robot_vision.main"
        try:
            self._connect()
            check_cmd = f"pgrep -f '{self.python_path}.*{script_path}'"
            _, stdout, _ = self.ssh_client.exec_command(check_cmd)
            pids = stdout.read().decode().strip()
            
            if not pids:
                print(f"No running instances of {script_path} found.")
                return
                
            kill_cmd = f"kill -9 {pids}"
            stdin, stdout, stderr = self.ssh_client.exec_command(kill_cmd)
            
            exit_status = stdout.channel.recv_exit_status()
            if exit_status == 0:
                print(f"Vision script stopped successfully.")
            else:
                error = stderr.read().decode().strip()
                print(f"Error stopping vision script: {error}")
        except Exception as e:
            print(f"Error stopping vision script: {e}")
        finally:
            self._disconnect()


def main():
    """Main function to handle command line arguments."""
    
    configs = {
        "vision": {
            "hostname": "192.168.10.2",
            "port": 22,
            "username": "nvidia",
            "password": "nvidia",
            "python": "/home/nvidia/miniconda3/envs/ros2/bin/python",
            "script_path": "autolife_robot_vision.main",
            "log_path": "/home/nvidia/Documents/vision_output.log"
        },
        "arm": {
            "hostname": "192.168.10.3",
            "port": 22,
            "username": "nvidia",
            "password": "nvidia",
            "python": "/home/nvidia/miniconda3/envs/ros2/bin/python",
            "script_path": "autolife_robot_arm.main",
            "log_path": "/home/nvidia/Documents/arm_output.log"
        }
    }

    parser = argparse.ArgumentParser(description="Manage remote robot scripts")
    parser.add_argument(
        "action",
        choices=["start", "stop"],
        help="Start or stop the script"
    )
    parser.add_argument(
        "targets",
        nargs="+",
        choices=["arm", "vision"],
        help="Target scripts to control"
    )
    args = parser.parse_args()

    # Initialize managers
    managers = {
        "vision": RemoteScriptManager(
            hostname=configs["vision"]["hostname"],
            port=configs["vision"]["port"],
            username=configs["vision"]["username"],
            password=configs["vision"]["password"],
            python_path=configs["vision"]["python"]
        ),
        "arm": RemoteScriptManager(
            hostname=configs["arm"]["hostname"],
            port=configs["arm"]["port"],
            username=configs["arm"]["username"],
            password=configs["arm"]["password"],
            python_path=configs["arm"]["python"]
        )
    }
    
    for target in args.targets:
        print(f"{args.action.capitalize()}ing {target} script...")
        config = configs[target]
        manager = managers[target]

        try:
            if args.action == "start":
                manager.run_script(config["script_path"], config["log_path"])
            else:
                if target == "arm":
                    manager.stop_arm_script()
                elif target == "vision":
                    manager.stop_vision_script()
        except Exception as e:
            print(f"Error handling {target} script: {str(e)}")


if __name__ == "__main__":
    main()