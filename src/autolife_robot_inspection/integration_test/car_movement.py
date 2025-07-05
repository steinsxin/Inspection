#!/usr/bin/env python3
import time
import threading
import traceback
import json
from typing import Dict, Tuple
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from autolife_robot_inspection.integration_test.utils import PublisherNode


class CarMovement:
    def __init__(self, shared_log: Dict, key_queue=None):
        self.shared_log = shared_log
        self.key_queue = key_queue

        # Max velocity limits
        self.max_linear_vel_x = 0.5      # m/s
        self.max_linear_vel_y = 0.3      # m/s
        self.max_angular_vel_z = 0.8     # rad/s

        # Max acceleration limits
        self.max_linear_acc = 0.2        # m/s^2
        self.max_angular_acc = 0.8       # rad/s^2

        # Target velocity for smoothing
        self._target_linear_x = 0.0
        self._target_linear_y = 0.0
        self._target_angular_z = 0.0
        self._previous_timestamp = 0.0

        self._last_input_time = 0.0
        self.input_timeout_sec = 0.5     # Auto zero if no input for 0.5s

    def keyboard_input_task(self, stop_event=None):
        """Keyboard listener thread that updates target velocity."""
        self.shared_log['log'] += f"Keyboard input thread started\n"
        while not stop_event.is_set():
            now = time.time()

            if self.key_queue and not self.key_queue.empty():
                # Drain the queue, keep only the latest key
                latest_key = None
                while not self.key_queue.empty():
                    latest_key = self.key_queue.get()

                if latest_key:
                    raw_target = self.convert_key_to_velocity(latest_key)
                    smoothed = self.apply_acceleration_limit(raw_target)
                    self._last_input_time = now
                    # self.shared_log['log'] += f"Target velocity from key: {smoothed}\n"

            elif now - self._last_input_time > self.input_timeout_sec:
                smoothed = self.apply_acceleration_limit((0.0, 0.0, 0.0))
                # self.shared_log['log'] += f"Auto zero target (no input): {smoothed}\n"

            time.sleep(0.05)

    def gv_task(self, pub_node: PublisherNode, stop_event=None):
        """Publisher thread that continuously sends velocity commands."""
        set_parameter = pub_node.set_enable_gv_cmd_vel()

        self.shared_log['log'] += f"Velocity publisher thread started\n"
        self.shared_log['log'] += f"set_gv_cmd_vel_parameter: {set_parameter}\n"

        while not stop_event.is_set():
            twist_msg = Twist()
            twist_msg.linear.x = self._target_linear_x
            twist_msg.linear.y = self._target_linear_y
            twist_msg.angular.z = self._target_angular_z
            pub_node.gv_target_cmd_vel_pub.publish(twist_msg)
            time.sleep(0.05)

        # Return speed to 0 before exiting
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0
        pub_node.gv_target_cmd_vel_pub.publish(twist_msg)
        time.sleep(0.1)

    def convert_key_to_velocity(self, key: str) -> Tuple[float, float, float]:
        """Convert key press to raw target velocity."""
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0

        if key == "w":
            linear_x = self.max_linear_vel_x
        elif key == "s":
            linear_x = -self.max_linear_vel_x
        elif key == "a":
            linear_y = self.max_linear_vel_y
        elif key == "d":
            linear_y = -self.max_linear_vel_y
        elif key == "q":
            angular_z = self.max_angular_vel_z
        elif key == "e":
            angular_z = -self.max_angular_vel_z

        return linear_x, linear_y, angular_z

    def apply_acceleration_limit(
        self, desired: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        """Smooth velocity changes by applying acceleration limits."""
        linear_x, linear_y, angular_z = desired

        now = time.time()
        if self._previous_timestamp > 0.0:
            dt = now - self._previous_timestamp
            dt = max(min(dt, 0.1), 0.01)  # Clamp dt to [0.01, 0.1]
        else:
            dt = 0.01  # First frame
        
        self._previous_timestamp = now

        max_dv = self.max_linear_acc * dt
        max_dw = self.max_angular_acc * dt

        dvx = linear_x - self._target_linear_x
        dvy = linear_y - self._target_linear_y
        dwz = angular_z - self._target_angular_z

        # Apply acceleration limits using clip
        linear_x = self._target_linear_x + np.clip(dvx, -max_dv, max_dv)
        linear_y = self._target_linear_y + np.clip(dvy, -max_dv, max_dv)
        angular_z = self._target_angular_z + np.clip(dwz, -max_dw, max_dw)

        self._target_linear_x = round(linear_x, 4)
        self._target_linear_y = round(linear_y, 4)
        self._target_angular_z = round(angular_z, 4)

        return self._target_linear_x, self._target_linear_y, self._target_angular_z


    def run(self, args=None, stop_event=None):
        """Initialize and run ROS2 node with keyboard and velocity threads."""
        rclpy.init(args=args)
        pub_node = PublisherNode(stop_event)

        keyboard_thread = threading.Thread(
            target=self.keyboard_input_task, args=(stop_event,)
        )
        keyboard_thread.daemon = True
        keyboard_thread.start()

        gv_task_thread = threading.Thread(
            target=self.gv_task, args=(pub_node, stop_event,)
        )
        gv_task_thread.daemon = True
        gv_task_thread.start()

        try:
            while rclpy.ok() and not stop_event.is_set():
                rclpy.spin_once(pub_node, timeout_sec=0.1)
        except KeyboardInterrupt:
            print("Quit by interrupt")
        finally:
            keyboard_thread.join()
            gv_task_thread.join()
            pub_node.destroy_node()
            rclpy.shutdown()

def main(stop_event=None, key_queue=None, shared_log=None):
    try:
        car_movement = CarMovement(
            shared_log=shared_log, 
            key_queue=key_queue
        )
        car_movement.run(stop_event=stop_event)
    except KeyboardInterrupt:
        print("Quit by interrupt")
    except Exception as e:
        print(f"Unexpected exception during spin: {e}")
        traceback.print_exc()

if __name__ == "__main__":
    stop_event = threading.Event()
    main(stop_event=stop_event)