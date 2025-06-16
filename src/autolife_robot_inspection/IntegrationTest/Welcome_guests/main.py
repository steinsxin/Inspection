import time
import traceback
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from utils import RealSenseCamera, FaceDetector, AudioPlayer, RobotTargetEEPoseDemo
from autolife_robot_sdk.hardware.hw_api import HWAPI


# 全局变量，用于存储最新的彩色图像
realsense_image = None
new_image_available = False
face_detected = False  # 标志位，表示是否检测到人脸
audio_task_complete = True  
action_command_complete = True

def camera_task(camera: RealSenseCamera) -> None:
    """
    Task to continuously capture color images from the RealSense camera.

    :param camera: An instance of RealSenseCamera.
    """
    global realsense_image, new_image_available
    try:
        while True:
            # Get the latest color image
            realsense_image = camera.get_color_image()
            if realsense_image is not None:
                new_image_available = True
            else:
                print("No color image available.")
            time.sleep(0.03)  # Small delay to avoid overwhelming the system
    finally:
        camera.stop()

def face_detector_task(face_detector: FaceDetector) -> None:
    """
    Task to continuously detect faces in the captured images.

    :param face_detector: An instance of FaceDetector.
    """
    global realsense_image, new_image_available, face_detected, audio_task_complete, action_command_complete

    try:
        while True:
            # Check if both audio and action tasks are complete
            if not audio_task_complete or not action_command_complete:
                time.sleep(10)  # Wait for 30 second before checking again
                continue

            # Check if a new image is available
            if new_image_available and realsense_image is not None:
                # Detect faces
                face_detected = face_detector.has_faces(realsense_image)
                if face_detected:
                    print("Detected face")
                else:
                    print("No face detected in the image")
                new_image_available = False  # Reset the flag
            time.sleep(2)  # Small delay to avoid overwhelming the system
    except KeyboardInterrupt:
        print("Face detection task interrupted")
    except Exception as e:
        print(f"Unexpected exception in face detection task: {e}")
        traceback.print_exc()

def audio_task() -> None:
    """
    Task to play audio when a face is detected.
    """
    global face_detected, audio_task_complete

    hw_api = HWAPI()
    hw_api.initialize(['mod_microphone_a311', 'mod_speaker_a311'])
    hw_api.speaker_set_volume(100)
    audio_player = AudioPlayer(hw_api)

    try:
        while True:
            # Check if a face was detected
            if face_detected:
                # greet
                print("Play greet")
                audio_player.play_audio("model/greet.wav")

                # Exhibition Introduction
                print("Play voice")
                audio_player.play_audio("model/introduce.wav")

                # Company Profile
                print("Play company_profile")
                audio_player.play_audio("model/company_profile.wav")

                face_detected = False  # Reset the flag after playing audio
                audio_task_complete = True  # Mark audio task as complete

            time.sleep(1)  # Small delay to avoid overwhelming the system
    except KeyboardInterrupt:
        print("Audio task interrupted")
    except Exception as e:
        print(f"Unexpected exception in audio task: {e}")
        traceback.print_exc()

def action_command() -> None:
    """
    Task to perform robot actions when a face is detected.
    """
    global face_detected, action_command_complete

    rclpy.init()
    robot_target_ee_pose_demo = RobotTargetEEPoseDemo()

    try:
        while True:
            # Check if a face was detected
            if face_detected:
                action_command_complete = False

                robot_target_ee_pose_demo.bow_salute()  # 假设这是你定义的机器人鞠躬动作

                robot_target_ee_pose_demo.replay("pkl/command_hello.pkl")
                time.sleep(1)

                robot_target_ee_pose_demo.control_reset()
                time.sleep(5)

                face_detected = False  # Reset the flag after performing action
                action_command_complete = True  # Mark action command as complete

            time.sleep(1)  # Small delay to avoid overwhelming the system
    except KeyboardInterrupt:
        print("Action command task interrupted")
    except Exception as e:
        print(f"Unexpected exception in action command task: {e}")
        traceback.print_exc()
    finally:
        robot_target_ee_pose_demo.destroy_node()
        rclpy.shutdown()


def main() -> None:
    """
    Main function to start the camera task, face detection task, and audio task in separate threads.
    """
    global realsense_image, new_image_available, face_detected

    # Initialize the RealSense camera
    camera = RealSenseCamera()
    camera.start()

    # Initialize the FaceDetector
    model_file = "model/deploy.prototxt"
    weights_file = "model/res10_300x300_ssd_iter_140000_fp16.caffemodel"
    face_detector = FaceDetector(model_file, weights_file)

    # Create and start the camera task thread
    camera_thread = threading.Thread(target=camera_task, args=(camera,))
    camera_thread.daemon = True  # Set as daemon thread so it exits when main thread exits
    camera_thread.start()

    # Create and start the face detection task thread
    face_detector_thread = threading.Thread(target=face_detector_task, args=(face_detector,))
    face_detector_thread.daemon = True  # Set as daemon thread so it exits when main thread exits
    face_detector_thread.start()

    # Create and start the audio task thread
    audio_thread = threading.Thread(target=audio_task)
    audio_thread.daemon = True  # Set as daemon thread so it exits when main thread exits
    audio_thread.start()

    # Create and start the action command task thread
    action_command_thread = threading.Thread(target=action_command)
    action_command_thread.daemon = True  # Set as daemon thread so it exits when main thread exits
    action_command_thread.start()

    try:
        # Keep the main thread running to allow the threads to execute
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Quit by interrupt")
    except Exception as e:
        print(f"Unexpected exception during execution: {e}")
        traceback.print_exc()
    finally:
        camera.stop()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Quit by interrupt")
    except Exception as e:
        print(f"Unexpected exception during execution: {e}")
        traceback.print_exc()