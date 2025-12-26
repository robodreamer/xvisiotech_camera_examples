"""High-level Python API for Xvisio devices."""

import time
from typing import Optional, Iterator
import numpy as np
from spatialmath.base import q2r, r2q
from .types import Pose, ImuSample, DeviceInfo

def _load_native():
    """Lazy-load the native module.

    Keeps `import xvisio` working even when the extension hasn't been built yet,
    which is helpful for unit tests that mock the backend.
    """
    try:
        from . import _xvisio_impl
    except ImportError as e:
        raise ImportError(
            "Failed to import xvisio native module. "
            "Make sure you've run 'pixi run install' to build the package."
        ) from e
    return _xvisio_impl


def discover() -> list[DeviceInfo]:
    """Discover available Xvisio devices.

    Returns:
        List of DeviceInfo objects for each discovered device.
    """
    _xvisio_impl = _load_native()
    devices = _xvisio_impl.discover_devices()
    return [
        DeviceInfo(
            serial_number=d.serial_number,
            model=d.model,
        )
        for d in devices
    ]


def open(serial_number: Optional[str] = None) -> "Device":
    """Open a device by serial number.

    Args:
        serial_number: Serial number of the device to open.
                      If None or empty, opens the first available device.

    Returns:
        Device object.

    Raises:
        RuntimeError: If no device is found or device cannot be opened.
    """
    _xvisio_impl = _load_native()
    dev = _xvisio_impl.open_device(serial_number or "")
    if dev is None:
        if serial_number:
            raise RuntimeError(f"Device with serial number '{serial_number}' not found")
        else:
            raise RuntimeError("No Xvisio devices found. Make sure device is connected and udev rules are installed.")
    return Device(dev)


class Device:
    """High-level device interface."""

    def __init__(self, impl):
        """Initialize device with native implementation."""
        self._impl = impl
        self._slam_enabled = False
        self._imu_enabled = False

        # Transform state for world-aligned and relative poses
        # Rotation offset to align camera frame with world frame (from ROS2 handler)
        # Standard offset: [[0, 0, 1], [1, 0, 0], [0, 1, 0]]
        # This maps: camera z+ (up) → world x+, camera x+ (forward) → world y+, camera y+ (right) → world z+
        self._R_offset = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=np.float64)
        self._pos_init = None
        self._quat_init = None
        self._initialized = False

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - cleanup."""
        self.close()

    def __repr__(self):
        return f"<Device serial={self.serial_number}>"

    @property
    def serial_number(self) -> str:
        """Get device serial number."""
        return self._impl.serial_number()

    def enable(self, slam: bool = True, imu: bool = False):
        """Enable device streams.

        Args:
            slam: Enable SLAM (pose tracking).
            imu: Enable IMU streaming.
        """
        if slam and not self._slam_enabled:
            if not self._impl.start_slam():
                raise RuntimeError("Failed to start SLAM")
            self._slam_enabled = True

        if imu and not self._imu_enabled:
            if not self._impl.start_imu():
                raise RuntimeError("Failed to start IMU")
            self._imu_enabled = True

    def close(self):
        """Stop all streams and close device."""
        if self._slam_enabled:
            self._impl.stop_slam()
            self._slam_enabled = False
        if self._imu_enabled:
            self._impl.stop_imu()
            self._imu_enabled = False

    def pose(self, prediction_s: float = 0.0) -> Pose:
        """Get current pose.

        Args:
            prediction_s: Prediction time in seconds (default: 0.0).

        Returns:
            Pose object.

        Raises:
            RuntimeError: If SLAM is not enabled or pose cannot be retrieved.
        """
        if not self._slam_enabled:
            raise RuntimeError("SLAM is not enabled. Call enable(slam=True) first.")

        import time
        # SLAM may need time to initialize - retry a few times
        max_retries = 10
        retry_delay = 0.1

        for attempt in range(max_retries):
            try:
                p = self._impl.get_pose(prediction_s)
                return Pose(
                    position=tuple(p.position),
                    quat_wxyz=tuple(p.quaternion),
                    host_timestamp_s=p.host_timestamp_s,
                    edge_timestamp_us=p.edge_timestamp_us,
                    confidence=p.confidence,
                )
            except Exception:
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    continue
                raise RuntimeError(
                    f"Failed to get pose after {max_retries} attempts. "
                    "SLAM may still be initializing - try moving the device or waiting longer."
                )


    def pose_world_aligned(self, prediction_s: float = 0.0) -> Pose:
        """Get current pose aligned with world frame.

        Applies rotation offset to align camera frame with world frame.
        This matches the transform used in the ROS2 teleop handler.

        Args:
            prediction_s: Prediction time in seconds (default: 0.0).

        Returns:
            Pose object with world-aligned orientation.

        Raises:
            RuntimeError: If SLAM is not enabled or pose cannot be retrieved.
        """
        raw_pose = self.pose(prediction_s)

        # Convert position and quaternion to numpy arrays
        pos = np.array(raw_pose.position, dtype=np.float64)
        quat_wxyz = np.array(raw_pose.quat_wxyz, dtype=np.float64)

        # Convert quaternion [w, x, y, z] to [x, y, z, w] for spatialmath q2r(order="xyzs")
        quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]], dtype=np.float64)

        # Convert quaternion to rotation matrix using spatialmath (matching ROS2 handler)
        R = q2r(quat_xyzw, order="xyzs")

        # Apply rotation offset: R_world = R_offset @ R_camera @ R_offset.T
        # Note: The ROS2 handler uses R_offset @ R_init.T @ R_camera @ R_offset.T for relative poses
        # For absolute world-aligned poses, we apply: R_world = R_offset @ R_camera @ R_offset.T
        R_world = self._R_offset @ R @ self._R_offset.T

        # Rotate position: pos_world = R_offset @ pos_camera
        pos_world = self._R_offset @ pos

        # Convert back to quaternion using spatialmath
        quat_xyzw_world = r2q(R_world, order="xyzs")  # Returns [x, y, z, w]
        # Convert back to [w, x, y, z] format
        quat_world = (quat_xyzw_world[3], quat_xyzw_world[0], quat_xyzw_world[1], quat_xyzw_world[2])

        return Pose(
            position=tuple(pos_world),
            quat_wxyz=quat_world,
            host_timestamp_s=raw_pose.host_timestamp_s,
            edge_timestamp_us=raw_pose.edge_timestamp_us,
            confidence=raw_pose.confidence,
        )

    def reset_pose_reference(self):
        """Reset the reference pose for relative pose calculations.

        Call this when you want to start tracking relative to the current pose.
        Useful for teleoperation scenarios where you want delta poses.
        """
        try:
            current_pose = self.pose()
            self._pos_init = np.array(current_pose.position, dtype=np.float64)
            self._quat_init = np.array(current_pose.quat_wxyz, dtype=np.float64)
            self._initialized = True
        except RuntimeError:
            # If pose not available yet, mark as not initialized
            self._initialized = False

    def pose_relative(self, prediction_s: float = 0.0) -> Pose:
        """Get current pose relative to the reference pose.

        Returns pose relative to the last call to reset_pose_reference().
        If reset_pose_reference() hasn't been called, automatically initializes
        on first call.

        This matches the transform used in the ROS2 teleop handler for delta poses.

        Args:
            prediction_s: Prediction time in seconds (default: 0.0).

        Returns:
            Pose object relative to reference pose.

        Raises:
            RuntimeError: If SLAM is not enabled or pose cannot be retrieved.
        """
        raw_pose = self.pose(prediction_s)

        # Auto-initialize on first call if not already initialized
        if not self._initialized:
            self.reset_pose_reference()

        if not self._initialized:
            # If still not initialized, return zero pose
            return Pose(
                position=(0.0, 0.0, 0.0),
                quat_wxyz=(1.0, 0.0, 0.0, 0.0),
                host_timestamp_s=raw_pose.host_timestamp_s,
                edge_timestamp_us=raw_pose.edge_timestamp_us,
                confidence=raw_pose.confidence,
            )

        # Convert to numpy arrays
        pos = np.array(raw_pose.position, dtype=np.float64)
        quat_wxyz = np.array(raw_pose.quat_wxyz, dtype=np.float64)
        pos_init = self._pos_init
        quat_init_wxyz = self._quat_init

        # Convert quaternions [w, x, y, z] to [x, y, z, w] for spatialmath q2r(order="xyzs")
        quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]], dtype=np.float64)
        quat_init_xyzw = np.array([quat_init_wxyz[1], quat_init_wxyz[2], quat_init_wxyz[3], quat_init_wxyz[0]], dtype=np.float64)

        # Convert quaternions to rotation matrices using spatialmath (matching ROS2 handler)
        R = q2r(quat_xyzw, order="xyzs")
        R_init = q2r(quat_init_xyzw, order="xyzs")

        # Apply transform matching ROS2 handler exactly:
        # pos_rel = R_offset @ R_init.T @ (pos - pos_init)
        # R_rel = R_offset @ R_init.T @ R @ R_offset.T
        pos_rel = self._R_offset @ R_init.T @ (pos - pos_init)
        R_rel = self._R_offset @ R_init.T @ R @ self._R_offset.T

        # Convert back to quaternion using spatialmath
        quat_rel_xyzw = r2q(R_rel, order="xyzs")  # Returns [x, y, z, w]
        # Convert back to [w, x, y, z] format
        quat_rel = (quat_rel_xyzw[3], quat_rel_xyzw[0], quat_rel_xyzw[1], quat_rel_xyzw[2])

        return Pose(
            position=tuple(pos_rel),
            quat_wxyz=quat_rel,
            host_timestamp_s=raw_pose.host_timestamp_s,
            edge_timestamp_us=raw_pose.edge_timestamp_us,
            confidence=raw_pose.confidence,
        )

    def pose_at(self, host_timestamp_s: float) -> Pose:
        """Get pose at a specific timestamp.

        Args:
            host_timestamp_s: Host timestamp in seconds.

        Returns:
            Pose object.

        Raises:
            RuntimeError: If SLAM is not enabled or pose cannot be retrieved.
        """
        if not self._slam_enabled:
            raise RuntimeError("SLAM is not enabled. Call enable(slam=True) first.")
        try:
            p = self._impl.get_pose_at(host_timestamp_s)
            return Pose(
                position=tuple(p.position),
                quat_wxyz=tuple(p.quaternion),
                host_timestamp_s=p.host_timestamp_s,
                edge_timestamp_us=p.edge_timestamp_us,
                confidence=p.confidence,
            )
        except Exception as e:
            raise RuntimeError(f"Failed to get pose at timestamp: {e}") from e

    def poses(self, rate_hz: Optional[float] = None) -> Iterator[Pose]:
        """Iterator over pose updates.

        Args:
            rate_hz: Target rate in Hz. If None, yields as fast as possible.

        Yields:
            Pose objects.
        """
        if not self._slam_enabled:
            raise RuntimeError("SLAM is not enabled. Call enable(slam=True) first.")

        sleep_time = 1.0 / rate_hz if rate_hz else 0.0

        while True:
            try:
                yield self.pose()
                if sleep_time > 0:
                    time.sleep(sleep_time)
            except KeyboardInterrupt:
                break
            except Exception as e:
                # Log error but continue
                print(f"Error getting pose: {e}")
                if sleep_time > 0:
                    time.sleep(sleep_time)

    def imu(self, timeout_s: float = 0.5) -> ImuSample:
        """Get current IMU sample.

        Args:
            timeout_s: Maximum time to wait for IMU data to become available (default: 0.5s).
                       This handles the case where IMU was just enabled and callback hasn't fired yet.

        Returns:
            ImuSample object.

        Raises:
            RuntimeError: If IMU is not enabled or sample cannot be retrieved within timeout.
        """
        if not self._imu_enabled:
            raise RuntimeError("IMU is not enabled. Call enable(imu=True) first.")

        import time
        start_time = time.time()
        while time.time() - start_time < timeout_s:
            try:
                imu = self._impl.get_imu()
                return ImuSample(
                    accel=tuple(imu.accel),
                    gyro=tuple(imu.gyro),
                    host_timestamp_s=imu.host_timestamp_s,
                    edge_timestamp_us=imu.edge_timestamp_us,
                )
            except Exception:
                # If get_imu() returns false, wait a bit and retry
                time.sleep(0.01)  # 10ms polling interval
                continue

        raise RuntimeError(f"Failed to get IMU: No data available within {timeout_s}s timeout. "
                          "Make sure IMU is enabled and device is streaming.")

    def imus(self, rate_hz: Optional[float] = None) -> Iterator[ImuSample]:
        """Iterator over IMU samples.

        Args:
            rate_hz: Target rate in Hz. If None, yields as fast as possible.

        Yields:
            ImuSample objects.
        """
        if not self._imu_enabled:
            raise RuntimeError("IMU is not enabled. Call enable(imu=True) first.")

        sleep_time = 1.0 / rate_hz if rate_hz else 0.0

        while True:
            try:
                yield self.imu()
                if sleep_time > 0:
                    time.sleep(sleep_time)
            except KeyboardInterrupt:
                break
            except Exception as e:
                # Log error but continue
                print(f"Error getting IMU: {e}")
                if sleep_time > 0:
                    time.sleep(sleep_time)

