import time
import wave
import traceback
from autolife_robot_sdk.hardware.hw_api import HWAPI


class AudioPlayer:
    """Class to handle audio playback using HWAPI."""

    def __init__(self, hardware_api: HWAPI):
        """
        Initialize the AudioPlayer class.

        Args:
            hardware_api (HWAPI): An instance of HWAPI for hardware operations.
        """
        self.hw_api = hardware_api

    @staticmethod
    def wav_to_pcm(wav_file_path: str) -> tuple:
        """
        Convert a WAV file to PCM data.

        Args:
            wav_file_path (str): Path to the WAV file.

        Returns:
            tuple: A tuple containing PCM data, sample rate, channels, and sample width.
        """
        with wave.open(wav_file_path, 'rb') as wav_file:
            # 获取WAV文件的参数
            sample_rate = wav_file.getframerate()  # 采样率
            channels = wav_file.getnchannels()     # 声道数
            sample_width = wav_file.getsampwidth() # 样本宽度（字节数）
            
            # 读取PCM数据
            pcm_data = wav_file.readframes(wav_file.getnframes())
            
            return pcm_data, sample_rate, channels, sample_width

    def play_audio(self, file_path: str) -> None:
        """
        Play an audio file.

        Args:
            file_path (str): Path to the WAV audio file.
        """
        # Convert WAV file to PCM data
        pcm_data, sample_rate, channels, sample_width = self.wav_to_pcm(file_path)

        # Calculate total samples and output time
        total_samples = len(pcm_data) // sample_width
        output_time = total_samples / (sample_rate * channels)

        # Play PCM data using hardware API
        self.hw_api.speaker_play_pcm_data(pcm_data, sample_rate, channels, sample_width)
        time.sleep(output_time)


def main() -> None:
    """
    Main function to initialize AudioPlayer and play an audio file.
    """
    hw_api = HWAPI()
    hw_api.initialize(['mod_microphone_a311', 'mod_speaker_a311'])
    hw_api.speaker_set_volume(80)
    
    audio_player = AudioPlayer(hw_api)
    audio_player.play_audio("../model/introduce.wav")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Quit by interrupt")
    except Exception as e:
        print(f"Unexpected exception during playback: {e}")
        traceback.print_exc()