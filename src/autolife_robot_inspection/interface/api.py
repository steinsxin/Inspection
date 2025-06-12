from abc import ABC, abstractmethod
from typing import Any


class DeviceInterface(ABC):
    """
    Base interface for all device controllers.

    This abstract base class defines a common interface for various device controllers,
    ensuring that all implementations provide the necessary methods for logging,
    configuration loading, and loopback functionality. It is designed to be generic
    and applicable to different types of devices, not just cameras.
    """

    @abstractmethod
    def get_log(self) -> Any:
        """
        Retrieves log data from the device controller.

        This method should be implemented to fetch and return log information
        from the device controller. The specific format and content of the log
        data are left to the implementation.

        Returns:
            Any: The log data retrieved from the device controller.
        """
        pass

    @abstractmethod
    def get_keyboard_data(self) -> Any:
        """
        Retrieves keyboard input data from the device controller.

        This method should be implemented to fetch and return keyboard input
        data from the device controller.

        Returns:
            Dict[str, Any]: A dictionary containing keyboard input data.
        """
        pass

    @abstractmethod
    def run(self) -> Any:
        """
        Runs the loopback functionality of the device controller.

        This method should be implemented to execute the loopback process,
        which typically involves capturing and re-transmitting data for testing
        purposes. The specific implementation and return value are left to the
        subclass.

        Returns:
            Any: The result of the loopback operation.
        """
        pass