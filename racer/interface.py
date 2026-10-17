from abc import ABC, abstractmethod

import numpy as np


class CarInterface(ABC):
    """The only way racing logic talks to a car: SimCar today, RosCar in Phase 4."""

    @abstractmethod
    def get_scan(self) -> np.ndarray: ...     # lidar ranges, meters, fixed angle spacing

    @abstractmethod
    def get_pose(self) -> tuple: ...          # (x, y, yaw) in map frame

    @abstractmethod
    def get_speed(self) -> float: ...         # m/s

    @abstractmethod
    def set_drive(self, speed: float, steer: float) -> None: ...
