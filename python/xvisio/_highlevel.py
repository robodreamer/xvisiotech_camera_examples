"""High-level Python API for Xvisio devices."""

import time
from typing import Optional, Iterator, Tuple
import numpy as np
from spatialmath.base import q2r, r2q
from .types import Pose, ImuSample, DeviceInfo, ControllerData

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


def discover(timeout_s: float = 2.0, controller_only: bool = False) -> list[DeviceInfo]:
    """Discover available Xvisio devices.

    Returns:
        List of DeviceInfo objects for each discovered device.
    """
    _xvisio_impl = _load_native()
    devices = _xvisio_impl.discover_devices(timeout_s, controller_only)
    return [
        DeviceInfo(
            serial_number=d.serial_number,
            model=d.model,
        )
        for d in devices
    ]


def open(
    serial_number: Optional[str] = None,
    timeout_s: float = 2.0,
    controller_only: bool = False,
) -> "Device":
    """Open a device by serial number.

    Args:
        serial_number: Serial number of the device to open.
                      If None or empty, opens the first available device.
        timeout_s: Maximum time to wait for device discovery (seconds).
        controller_only: If True, only search for Seer wireless controller devices.

    Returns:
        Device object.

    Raises:
        RuntimeError: If no device is found or device cannot be opened.
    """
    _xvisio_impl = _load_native()
    dev = _xvisio_impl.open_device(serial_number or "", timeout_s, controller_only)
    if dev is None:
        if serial_number:
            raise RuntimeError(f"Device with serial number '{serial_number}' not found")
        else:
            raise RuntimeError("No Xvisio devices found. Make sure device is connected and udev rules are installed.")
    return Device(dev)


def discover_controllers(timeout_s: float = 2.0) -> list[DeviceInfo]:
    """Discover Seer wireless controller devices only."""
    return discover(timeout_s=timeout_s, controller_only=True)


def open_controller(port: str = "/dev/ttyUSB0", timeout_s: float = 2.0) -> "Device":
    """Open a Seer controller-only device and start controller streaming.

    Args:
        port: Serial port for controller dongle (e.g. /dev/ttyUSB0).
        timeout_s: Maximum time to wait for controller discovery (seconds).
    """
    dev = open(serial_number=None, timeout_s=timeout_s, controller_only=True)
    dev.enable_controller(port=port)
    return dev


class Device:
    """High-level device interface."""

    def __init__(self, impl):
        """Initialize device with native implementation."""
        self._impl = impl
        self._slam_enabled = False
        self._imu_enabled = False

        # Transform state for world-aligned and relative poses
        # Rotation offset to align camera frame with world frame (from ROS2 handler)
        # Standard offset for XR-50 camera: [[0, 0, 1], [1, 0, 0], [0, 1, 0]]
        # This maps: camera z+ (up) → world x+, camera x+ (forward) → world y+, camera y+ (right) → world z+
        self._R_offset = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=np.float64)
        # Rotation offset for Seer wireless controller (different from camera)
        # Swapped first and last rows from ROS2's R_OFFSET_WIRELESS to fix x/z axis alignment
        self._R_offset_controller = np.array([[0, 0, -1], [1, 0, 0], [0, 1, 0]], dtype=np.float64)
        self._pos_init = None
        self._quat_init = None
        self._initialized = False
        # Controller-specific relative pose state
        self._controller_pos_init = None
        self._controller_R_init = None
        self._controller_initialized = False

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - cleanup."""
        self.close()

    def _compute_relative_transform(
        self,
        pos: np.ndarray,
        R: np.ndarray,
        pos_init: np.ndarray,
        R_init: np.ndarray,
        R_offset: np.ndarray,
        negate_position: bool = False,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Compute relative pose transform (shared by camera and controller).

        This implements the ROS2 teleop handler transform exactly:
            pos_rel = R_offset @ R_init.T @ (pos - pos_init)
            R_rel = R_offset @ R_init.T @ R @ R_offset.T

        Args:
            pos: Current position (raw, not world-aligned)
            R: Current rotation matrix (raw, not world-aligned)
            pos_init: Reference position (raw, from reset)
            R_init: Reference rotation matrix (raw, from reset)
            R_offset: Rotation offset matrix for frame alignment
            negate_position: If True, negate the position result (for controller
                            where rotation offset works but position needs inversion)

        Returns:
            Tuple of (pos_rel, R_rel) - relative position and rotation matrix
        """
        pos_rel = R_offset @ R_init.T @ (pos - pos_init)
        if negate_position:
            pos_rel = -pos_rel
        R_rel = R_offset @ R_init.T @ R @ R_offset.T
        return pos_rel, R_rel

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
        if self._impl.controller_running():
            self._impl.stop_controller()
        if self._slam_enabled:
            self._impl.stop_slam()
            self._slam_enabled = False
        if self._imu_enabled:
            self._impl.stop_imu()
            self._imu_enabled = False

    def enable_controller(self, port: str = "/dev/ttyUSB0") -> None:
        """Start Seer wireless controller on the given serial port.

        Args:
            port: Serial port for controller dongle (e.g. /dev/ttyUSB0).

        Raises:
            RuntimeError: If controller cannot be started.
        """
        if not self._impl.start_controller(port):
            raise RuntimeError(
                f"Failed to start controller on {port}. "
                "Ensure this device supports wireless controller and port is correct."
            )

    def disable_controller(self) -> None:
        """Stop Seer wireless controller."""
        self._impl.stop_controller()

    def controller(self) -> Tuple[Optional[ControllerData], Optional[ControllerData]]:
        """Get latest left and right controller data.

        Returns:
            Tuple (left, right). Either can be None if not yet received.
        """
        left_raw, right_raw = self._impl.get_controller_data()

        def _to_controller_data(c) -> ControllerData:
            kind = "left" if c.type == 0 else "right"
            return ControllerData(
                type=kind,
                position=tuple(c.position),
                quat_wxyz=tuple(c.quaternion),
                host_timestamp_s=c.host_timestamp_s,
                key_trigger=c.key_trigger,
                key_side=c.key_side,
                rocker_x=c.rocker_x,
                rocker_y=c.rocker_y,
                key=c.key,
            )

        left = _to_controller_data(left_raw) if left_raw is not None else None
        right = _to_controller_data(right_raw) if right_raw is not None else None
        return (left, right)

    def controller_left(self) -> Optional[ControllerData]:
        """Get latest left controller data, or None if not yet received."""
        left, _ = self.controller()
        return left

    def controller_right(self) -> Optional[ControllerData]:
        """Get latest right controller data, or None if not yet received."""
        _, right = self.controller()
        return right

    def controller_world_aligned(
        self,
    ) -> Tuple[Optional[ControllerData], Optional[ControllerData]]:
        """Get controller data with world-aligned orientation.

        Applies the Seer controller rotation offset to align with world frame.
        This matches the transform used in the ROS2 teleop handler for wireless controllers.

        Returns:
            Tuple (left, right). Either can be None if not yet received.
        """
        left, right = self.controller()

        def _apply_offset(c: ControllerData) -> ControllerData:
            pos = np.array(c.position, dtype=np.float64)
            quat_wxyz = np.array(c.quat_wxyz, dtype=np.float64)

            # Convert quaternion [w, x, y, z] to [x, y, z, w] for spatialmath
            quat_xyzw = np.array(
                [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]], dtype=np.float64
            )

            # Convert to rotation matrix
            R = q2r(quat_xyzw, order="xyzs")

            # Apply controller rotation offset
            R_world = self._R_offset_controller @ R @ self._R_offset_controller.T
            pos_world = self._R_offset_controller @ pos

            # Convert back to quaternion
            quat_xyzw_world = r2q(R_world, order="xyzs")
            quat_world = (
                quat_xyzw_world[3],
                quat_xyzw_world[0],
                quat_xyzw_world[1],
                quat_xyzw_world[2],
            )

            return ControllerData(
                type=c.type,
                position=tuple(pos_world),
                quat_wxyz=quat_world,
                host_timestamp_s=c.host_timestamp_s,
                key_trigger=c.key_trigger,
                key_side=c.key_side,
                rocker_x=c.rocker_x,
                rocker_y=c.rocker_y,
                key=c.key,
            )

        left_aligned = _apply_offset(left) if left is not None else None
        right_aligned = _apply_offset(right) if right is not None else None
        return (left_aligned, right_aligned)

    def reset_controller_reference(self) -> None:
        """Reset the reference pose for relative controller pose calculations.

        Call this when you want to start tracking controller position relative
        to the current position. Similar to reset_pose_reference() for camera.

        Stores the RAW (not world-aligned) pose as reference, matching the
        ROS2 teleop handler behavior.
        """
        # Use raw controller data (not world-aligned) - same as ROS2 handler
        left, right = self.controller()
        # Use whichever controller is available (prefer right)
        c = right if right is not None else left
        if c is not None:
            self._controller_pos_init = np.array(c.position, dtype=np.float64)
            quat_wxyz = np.array(c.quat_wxyz, dtype=np.float64)
            quat_xyzw = np.array(
                [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]], dtype=np.float64
            )
            self._controller_R_init = q2r(quat_xyzw, order="xyzs")
            self._controller_initialized = True
        else:
            self._controller_initialized = False

    def controller_relative(
        self,
    ) -> Tuple[Optional[ControllerData], Optional[ControllerData]]:
        """Get controller data relative to the reference pose.

        Returns pose relative to the last call to reset_controller_reference().
        If reset_controller_reference() hasn't been called, automatically
        initializes on first call.

        This matches the transform used in the ROS2 teleop handler for delta poses:
        - pos_rel = R_offset @ R_init.T @ (pos - pos_init)
        - R_rel = R_offset @ R_init.T @ R @ R_offset.T

        Returns:
            Tuple (left, right). Either can be None if not yet received.
        """
        # Use raw controller data (not world-aligned) - same as ROS2 handler
        left, right = self.controller()

        # Auto-initialize if not done yet
        if not self._controller_initialized:
            self.reset_controller_reference()
            # After initialization, positions are at origin
            if self._controller_initialized:
                return self._make_zero_controller(left, right)
            return (left, right)

        def _apply_relative(c: ControllerData) -> ControllerData:
            pos = np.array(c.position, dtype=np.float64)
            quat_wxyz = np.array(c.quat_wxyz, dtype=np.float64)
            quat_xyzw = np.array(
                [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]], dtype=np.float64
            )
            R = q2r(quat_xyzw, order="xyzs")

            # Use shared helper for consistent transform (same as camera pose_relative)
            # negate_position=True because controller R_offset works for rotation
            # but produces inverted position (different from camera coordinate system)
            pos_rel, R_rel = self._compute_relative_transform(
                pos=pos,
                R=R,
                pos_init=self._controller_pos_init,
                R_init=self._controller_R_init,
                R_offset=self._R_offset_controller,
                negate_position=True,
            )

            # Convert back to quaternion
            quat_xyzw_rel = r2q(R_rel, order="xyzs")
            quat_rel = (
                quat_xyzw_rel[3],
                quat_xyzw_rel[0],
                quat_xyzw_rel[1],
                quat_xyzw_rel[2],
            )

            return ControllerData(
                type=c.type,
                position=tuple(pos_rel),
                quat_wxyz=quat_rel,
                host_timestamp_s=c.host_timestamp_s,
                key_trigger=c.key_trigger,
                key_side=c.key_side,
                rocker_x=c.rocker_x,
                rocker_y=c.rocker_y,
                key=c.key,
            )

        left_rel = _apply_relative(left) if left is not None else None
        right_rel = _apply_relative(right) if right is not None else None
        return (left_rel, right_rel)

    def _make_zero_controller(
        self, left: Optional[ControllerData], right: Optional[ControllerData]
    ) -> Tuple[Optional[ControllerData], Optional[ControllerData]]:
        """Create controller data at origin (for first frame after reset)."""

        def _zero(c: ControllerData) -> ControllerData:
            return ControllerData(
                type=c.type,
                position=(0.0, 0.0, 0.0),
                quat_wxyz=(1.0, 0.0, 0.0, 0.0),
                host_timestamp_s=c.host_timestamp_s,
                key_trigger=c.key_trigger,
                key_side=c.key_side,
                rocker_x=c.rocker_x,
                rocker_y=c.rocker_y,
                key=c.key,
            )

        return (
            _zero(left) if left is not None else None,
            _zero(right) if right is not None else None,
        )

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

        # Use shared helper for consistent transform (same as controller_relative)
        pos_rel, R_rel = self._compute_relative_transform(
            pos=pos,
            R=R,
            pos_init=pos_init,
            R_init=R_init,
            R_offset=self._R_offset,
        )

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

