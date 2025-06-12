from contextlib import redirect_stdout
import io

with redirect_stdout(io.StringIO()):
    try:
        from .piper_text_voice import PiperVoice
    except ImportError:
        print("Failed to import PiperVoice. Make sure you have the required dependencies installed.")
        PiperVoice = None

    try:
        from .motor_detector import MotorDetector
    except ImportError:
        print("Failed to import MotorDetector. Make sure you have the required dependencies installed.")
        MotorDetector = None

    try:
        from .camera_detector import CameraDetector
    except ImportError:
        print("Failed to import CameraDetector. Make sure you have the required dependencies installed.")
        CameraDetector = None

    try:
        from .lidar_detector import LidarDetector
    except ImportError:
        print("Failed to import LidarDetector. Make sure you have the required dependencies installed.")
        LidarDetector = None

    try:
        from .imu_detector import ImuDetector
    except ImportError:
        print("Failed to import ImuDetector. Make sure you have the required dependencies installed.")
        ImuDetector = None
        
    try:
        from .battery_detector import BatteryDetector
    except ImportError:
        print("Failed to import BatteryDetector. Make sure you have the required dependencies installed.")
        BatteryDetector = None

    try:
        from .audio_detector import AudioDetector
    except ImportError:
        print("Failed to import AudioDetector. Make sure you have the required dependencies installed.")
        AudioDetector = None

    try:
        from .hardware_detector import HardwareDetector
    except ImportError:
        print("Failed to import HardwareDetector. Make sure you have the required dependencies installed.")
        HardwareDetector = None
        
    try:
        from .internal_detector import InternalDetector
    except ImportError:
        print("Failed to import InternalDetector. Make sure you have the required dependencies installed.")
        InternalDetector = None

    try:
        from .wifi_detector import WifiDetector
    except ImportError:
        print("Failed to import WifiDetector. Make sure you have the required dependencies installed.")
        WifiDetector = None