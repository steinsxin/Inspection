from contextlib import redirect_stdout
import io

with redirect_stdout(io.StringIO()):
    try:
        from .piper_text_voice import PiperVoice
    except ImportError:
        print("Failed to import PiperVoice. Make sure you have the required dependencies installed.")
        PiperVoice = None

    try:
        from .publisher_node import PublisherNode
    except ImportError:
        print("Failed to import PublisherNode. Make sure you have the required dependencies installed.")
        PublisherNode = None

    try:
        from .receiver_node import ReceiverNode
    except ImportError:
        print("Failed to import ReceiverNode. Make sure you have the required dependencies installed.")
        ReceiverNode = None