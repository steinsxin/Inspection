import os
import time
import traceback
from typing import Dict, List

import cv2
import numpy as np
import pyrealsense2 as rs

from linuxpy.video import device
from linuxpy.video.device import VideoCapture

from autolife_robot_sdk import GLOBAL_VARS
from autolife_robot_sdk.hardware.hw_api import HWAPI

# Global configuration
GLOBAL_VARS.ACTIVE_ROBOT_VERSION = 'robot_v1_0'
HARDWARE_API = HWAPI()


def list_video_devices(max_retries: int = 3) -> List[device.VideoCapture]:
    """Scan and list available video capture devices.
    
    Args:
        max_retries: Maximum number of retry attempts
        
    Returns:
        List of detected video capture devices
        
    Raises:
        RuntimeError: If no devices are found after retries
    """
    for attempt in range(max_retries):
        try:
            devices = list(device.iter_video_capture_devices())
            if devices:
                return devices
        except Exception as e:
            print(f"Scan attempt {attempt + 1} failed: {e}")
            time.sleep(3)
    
    raise RuntimeError("Failed to detect any video capture devices")


def print_device_info(devices: List[VideoCapture]) -> None:
    """Print information about detected video devices.
    
    Args:
        devices: List of video capture devices
    """
    for idx, dev in enumerate(devices):
        with dev:
            try:
                print(f"Device {idx}: {dev.filename} - Bus: {dev.info.bus_info}")
            except Exception as e:
                traceback.print_exc()
                continue


def test_cameras() -> None:
    """Test all camera modules and save sample images."""
    camera_modules = [
        'mod_camera_head_left',
        'mod_camera_head_right',
        'mod_camera_head_rear',
        'mod_camera_hand_left',
        'mod_camera_hand_right',
        'mod_camera_torso_front',
    ]
    
    HARDWARE_API.initialize(camera_modules)
    print("------------ Camera Test Started ----------")

    # Create directory for test images
    save_dir = "test_images"
    os.makedirs(save_dir, exist_ok=True)

    # Camera names and their corresponding output files
    camera_mapping: Dict[str, str] = {
        "head_left": os.path.join(save_dir, "head_left.jpg"),
        "head_right": os.path.join(save_dir, "head_right.jpg"),
        "head_rear": os.path.join(save_dir, "head_rear.jpg"),
        "hand_left": os.path.join(save_dir, "hand_left.jpg"),
        "hand_right": os.path.join(save_dir, "hand_right.jpg"),
        "torso_front": os.path.join(save_dir, "torso_front.jpg")
    }

    try:
        for camera_name, output_path in camera_mapping.items():
            # Since the first get_camera_image will start the camera and won't get a real image
            image = HARDWARE_API.get_camera_image(camera_name)
            time.sleep(0.1)
            image = HARDWARE_API.get_camera_image(camera_name)
            if isinstance(image, dict):
                color_, depth_ = image['color'], image['depth']
                cv2.imwrite(os.path.join(save_dir, "color.jpg"), color_)
                cv2.imwrite(os.path.join(save_dir, "depth.jpg"), depth_)
            else:
                cv2.imwrite(output_path, image)   
            print(f"Saved image from {camera_name} to {output_path}")
    except Exception as e:
        traceback.print_exc()
        print(f"Camera test failed: {e}")
    finally:
        print("------------ Camera Test Completed ----------")


def realsense_serial_number():
    """
    Print the serial number of connected RealSense devices.
    """
    ctx = rs.context()
    devices = ctx.query_devices()

    if not devices:
        print("No RealSense devices found.")
    else:
        print("Found RealSense devices:")
        for dev in devices:
            serial_number = dev.get_info(rs.camera_info.serial_number)
            name = dev.get_info(rs.camera_info.name)
            print(f"  - Name: {name}, Serial Number: {serial_number}")


if __name__ == "__main__":
    try:
        # Uncomment to list video devices
        devices = list_video_devices()
        print_device_info(devices)
        realsense_serial_number()
        
        test_cameras()
    except Exception as e:
        print(f"Error in main execution: {e}")
        traceback.print_exc()