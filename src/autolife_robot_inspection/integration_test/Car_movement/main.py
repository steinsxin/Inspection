#!/usr/bin/env python3
import os
import sys
import time

def main(args=None) -> None:
    """Main function to initialize and run the ROS2 node."""
    print("Hello Car")
    time.sleep(1)

if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Quit by interrupt")
    except Exception as e:
        print(f"Unexpected exception during spin: {e}")
        traceback.print_exc()