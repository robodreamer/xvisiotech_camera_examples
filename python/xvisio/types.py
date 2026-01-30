"""Type definitions for xvisio data structures."""

from dataclasses import dataclass
from typing import Tuple
import numpy as np


@dataclass
class Pose:
    """6-DOF pose from SLAM."""
    position: Tuple[float, float, float]
    quat_wxyz: Tuple[float, float, float, float]  # w, x, y, z
    host_timestamp_s: float
    edge_timestamp_us: int
    confidence: float

    @property
    def quaternion(self) -> Tuple[float, float, float, float]:
        """Alias for quat_wxyz (for compatibility)."""
        return self.quat_wxyz


@dataclass
class ImuSample:
    """IMU measurement sample."""
    accel: Tuple[float, float, float]  # m/s^2
    gyro: Tuple[float, float, float]    # rad/s
    host_timestamp_s: float
    edge_timestamp_us: int


@dataclass
class ControllerData:
    """Seer wireless controller data (pose + buttons)."""
    type: str  # "left" or "right"
    position: Tuple[float, float, float]
    quat_wxyz: Tuple[float, float, float, float]  # w, x, y, z
    host_timestamp_s: float
    key_trigger: int
    key_side: int
    rocker_x: int
    rocker_y: int
    key: int


@dataclass
class DeviceInfo:
    """Information about a discovered device."""
    serial_number: str
    model: str

