try:
    from .realsense_camera import RealSenseCamera
except ImportError:
    print("Failed to import RealSenseCamera. Make sure you have the required dependencies installed.")
    RealSenseCamera = None

try:
    from .audio_play import AudioPlayer
except ImportError:
    print("Failed to import AudioPlayer. Make sure you have the required dependencies installed.")
    AudioPlayer = None

try:
    from .face_failure import FaceDetector
except ImportError:
    print("Failed to import FaceDetector. Make sure you have the required dependencies installed.")
    FaceDetector = None

try:
    from .ros_server import RobotTargetEEPoseDemo
except ImportError:
    print("Failed to import FaceDetector. Make sure you have the required dependencies installed.")
    FaceDetector = None