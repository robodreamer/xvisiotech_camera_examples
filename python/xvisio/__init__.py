"""
ROS-free Python SDK for Xvisio XR-50 devices.

Example usage:
    import xvisio

    with xvisio.open() as dev:
        dev.enable(slam=True, imu=True)
        for pose in dev.poses():
            imu = dev.imu()
            print(pose.position, pose.quat_wxyz, imu.accel)
"""

from ._highlevel import Device, open, discover
from .types import Pose, ImuSample, DeviceInfo

__version__ = "0.1.0"

__all__ = [
    "Device",
    "open",
    "discover",
    "Pose",
    "ImuSample",
    "DeviceInfo",
]

