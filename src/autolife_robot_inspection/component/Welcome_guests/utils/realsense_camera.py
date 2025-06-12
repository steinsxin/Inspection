import pyrealsense2 as rs
import numpy as np
import time


class RealSenseCamera:
    """Class to handle RealSense camera data capture and provide color image data access."""

    def __init__(self, depth_stream=False, color_stream=True):
        """
        Initialize the RealSenseCamera class.

        Args:
            depth_stream (bool): Enable depth stream. Defaults to False.
            color_stream (bool): Enable color stream. Defaults to True.
        """
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        if depth_stream:
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        if color_stream:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = None

    def start(self):
        """Start the RealSense camera pipeline."""
        self.profile = self.pipeline.start(self.config)
        print("RealSense camera pipeline started.")

    def stop(self):
        """Stop the RealSense camera pipeline."""
        if self.profile:
            self.pipeline.stop()
            print("RealSense camera pipeline stopped.")

    def get_color_image(self):
        """
        Capture and return the latest color image from the RealSense camera.

        Returns:
            numpy.ndarray: The latest color image as a numpy array, or None if no frame is available.
        """
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if color_frame:
            return np.asanyarray(color_frame.get_data())
        else:
            print("Failed to get color frame.")
            return None


def main():
    """Main function to demonstrate RealSense camera usage."""
    camera = RealSenseCamera()
    camera.start()
    try:
        while True:
            # Get the latest color image
            color_image = camera.get_color_image()

            if color_image is not None:
                # Process the color image (example: print shape)
                print(f"Color image shape: {color_image.shape}")
            else:
                print("No color image available.")

    finally:
        camera.stop()


if __name__ == "__main__":
    main()