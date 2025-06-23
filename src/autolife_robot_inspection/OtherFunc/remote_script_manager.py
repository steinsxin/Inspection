import paramiko


class RemoteScriptManager:
    """Manage execution and termination of a Python script on a remote device."""

    def __init__(self, hostname, port, username, password):
        """Initialize connection information for the remote device.
        
        Args:
            hostname: IP address of the remote device
            port: SSH port number
            username: SSH username
            password: SSH password
        """
        self.hostname = hostname
        self.port = port
        self.username = username
        self.password = password
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
            script_path: Path to the Python script
            
        Returns:
            bool: True if script is running, False otherwise
        """
        check_cmd = f"pgrep -f {script_path}"
        _, stdout, _ = self.ssh_client.exec_command(check_cmd)
        return bool(stdout.read().decode().strip())

    def run_script(self, remote_script_path, log_file_path):
        """Run a Python script on the remote device in the background.
        
        Args:
            remote_script_path: Path to the Python script on remote device
            log_file_path: Path to the log file on remote device
        """
        try:
            self._connect()
            
            if self._is_script_running(remote_script_path):
                print("Script is already running. Failed to start.")
                return

            cmd = f"nohup chrt -r 99 python -u -m {remote_script_path} > {log_file_path} 2>&1 &"
            self.ssh_client.exec_command(cmd)
            print("Script started successfully.")
            
        except Exception as e:
            print(f"Error connecting/executing command: {e}")
        finally:
            self._disconnect()

    def stop_script(self, remote_script_path):
        """Stop the running Python script on the remote device.
        
        Args:
            remote_script_path: Path to the Python script on remote device
        """
        try:
            self._connect()
            
            if not self._is_script_running(remote_script_path):
                print("Script is not running. No need to stop.")
                return

            check_cmd = f"pgrep -f {remote_script_path}"
            _, stdout, _ = self.ssh_client.exec_command(check_cmd)
            pids = stdout.read().decode().strip()
            
            kill_cmd = f"kill {pids}"
            self.ssh_client.exec_command(kill_cmd)
            print("Script stopped successfully.")
            
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
        "script_path": "autolife_robot_arm.main",
        "log_path": "/home/nvidia/Documents/arm_output.log"
    }

    manager = RemoteScriptManager(
        config["hostname"],
        config["port"],
        config["username"],
        config["password"]
    )

    # Uncomment to run or stop script
    # manager.run_script(config["script_path"], config["log_path"])
    manager.stop_script(config["script_path"])