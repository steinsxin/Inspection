from contextlib import redirect_stdout
import io

with redirect_stdout(io.StringIO()):
    try:
        from .piper_text_voice import PiperVoice
    except ImportError:
        print("Failed to import PiperVoice. Make sure you have the required dependencies installed.")
        PiperVoice = None

    try:
        from .ros_server import ServerSubscriber
    except ImportError:
        print("Failed to import ServerSubscriber. Make sure you have the required dependencies installed.")
        PiperVoice = None