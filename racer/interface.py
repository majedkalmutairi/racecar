from abc import ABC, abstractmethod

import numpy as np


def beam_angles(cfg):
    """Angle of every lidar beam relative to the heading. Beam 0 is far right."""
    lid = cfg["lidar"]
    return np.linspace(-lid["fov_rad"] / 2, lid["fov_rad"] / 2, lid["n_beams"])


class CarInterface(ABC):
    """The only way racing logic talks to a car: SimCar today, RosCar on the real car."""

    @abstractmethod
    def get_scan(self) -> np.ndarray: ...

    @abstractmethod
    def get_pose(self) -> tuple: ...

    @abstractmethod
    def get_speed(self) -> float: ...

    @abstractmethod
    def set_drive(self, speed: float, steer: float) -> None: ...

    @property
    def crashed(self) -> bool:
        return False
