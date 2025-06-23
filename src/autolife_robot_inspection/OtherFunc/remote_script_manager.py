import paramiko


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
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh_client.connect(
            self.hostname, 
            self.port, 
            self.username, 
            self.password
        )

    def _disconnect(self):
        """Close the SSH connection."""
        if self.ssh_client:
            self.ssh_client.close()
            print("SSH connection closed.")

    def _is_script_running(self, script_path):
        """Check if the script is already running on the remote device.
        
        Args:
            script_path: Path to the Python script or module
            
        Returns:
            bool: True if script is running, False otherwise
        """
        check_cmd = f"pgrep -f '{self.python_path}.*{script_path}'"
        _, stdout, _ = self.ssh_client.exec_command(check_cmd)
        return bool(stdout.read().decode().strip())

    def run_script(self, script_module_path, log_file_path):
        """Run a Python script on the remote device in the background.
        
        Args:
            script_module_path: Python module path (e.g., 'package.module')
            log_file_path: Path to the log file on remote device
        """
        try:
            self._connect()
            
            if self._is_script_running(script_module_path):
                print("Script is already running. Failed to start.")
                return

            cmd = (f"nohup chrt -r 99 {self.python_path} -u -m {script_module_path} "
                   f"> {log_file_path} 2>&1 &")
            self.ssh_client.exec_command(cmd)
            print("Script started successfully.")
            
        except Exception as e:
            print(f"Error connecting/executing command: {e}")
        finally:
            self._disconnect()

    def stop_script(self):
        """Stop all Python processes on the remote device."""
        try:
            self._connect()
            
            kill_cmd = "killall python"
            stdin, stdout, stderr = self.ssh_client.exec_command(kill_cmd)
            
            # Check if killall command was successful
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
            print(f"Error connecting/executing command: {e}")
        finally:
            self._disconnect()


if __name__ == "__main__":
    # Configuration
    config = {
        "hostname": "192.168.10.3",
        "port": 22,
        "username": "nvidia",
        "password": "nvidia",
        "python": "/home/nvidia/miniconda3/envs/ros2/bin/python",
        "script_path": "autolife_robot_arm.main",
        "log_path": "/home/nvidia/Documents/arm_output.log"
    }

    manager = RemoteScriptManager(
        hostname=config["hostname"],
        port=config["port"],
        username=config["username"],
        password=config["password"],
        python_path=config["python"]
    )

    # Uncomment to run or stop script
    # manager.run_script(config["script_path"], config["log_path"])
    manager.stop_script()