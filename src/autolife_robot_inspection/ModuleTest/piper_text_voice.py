import time
import subprocess
from autolife_robot_sdk.hardware.hw_api import HWAPI

class PiperVoice:
    def __init__(self, hardware_api: HWAPI, model_path: str):
        """Initialize the PiperVoice class.

        Args:
            hardware_api: An instance of HWAPI for hardware operations.
            model_path: Path to the Piper model.
        """
        self.hw_api = hardware_api
        self.model_path = model_path

        """Log level options:
        - quiet:     Disable all log output
        - panic:     Only show critical errors
        - error:     Show errors
        - warning:   Show warnings and errors
        - info:      Show general information
        - verbose:   Show detailed information
        - debug:     Show debug information
        - trace:     Show trace information
        """
        self.loglevel = "quiet"

    def play_voice(self, text: str) -> None:
        """Convert text to speech and play it.

        Args:
            text: The text to be converted to speech.
        """
        # Convert text to speech
        pcm_data, sample_rate, channels, sample_width = self._text_to_voice(text)

        # Calculate total samples and output time
        total_samples = len(pcm_data) // sample_width
        output_time = total_samples / (sample_rate * channels)

        self.hw_api.speaker_play_pcm_data(pcm_data, sample_rate, channels, sample_width)
        time.sleep(output_time)

    def _text_to_voice(
        self, text: str, sample_rate: int = 22050, channels: int = 1, sample_width: int = 2
    ) -> tuple[bytes, int, int, int]:
        """Generate audio data from text using Piper and convert it to PCM format.

        Args:
            text: The text to synthesize.
            sample_rate: Sample rate in Hz, defaults to 22050.
            channels: Number of audio channels, defaults to 1 (mono).
            sample_width: Sample width in bytes, defaults to 2 (16-bit).

        Returns:
            A tuple containing:
                - PCM data
                - Sample rate
                - Number of channels
                - Sample width
        """
        # Generate WAV audio data using Piper
        piper_command = [
            "piper",
            "--model", self.model_path,
            "--output_file", "-"
        ]
        piper_process = subprocess.Popen(
            piper_command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE
        )
        wav_data, _ = piper_process.communicate(text.encode("utf-8"))

        # Convert WAV data to PCM using ffmpeg
        ffmpeg_command = [
            "ffmpeg",
            "-loglevel", self.loglevel,  # Disable log output
            "-i", "-",
            "-f", "s16le",
            "-ar", str(sample_rate),
            "-ac", str(channels),
            "-"
        ]
        ffmpeg_process = subprocess.Popen(
            ffmpeg_command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE
        )
        pcm_data, _ = ffmpeg_process.communicate(wav_data)

        return pcm_data, sample_rate, channels, sample_width