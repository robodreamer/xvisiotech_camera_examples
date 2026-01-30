#!/usr/bin/env python3
"""
Visualize Xvisio pose frames in viser.

This example demonstrates:
- Real-time visualization of camera pose using viser
- Relative pose visualization (matching ROS2 teleop handler behavior)
- Trajectory visualization (path history)
- IMU data visualization (acceleration vector)
- GUI controls for resetting pose reference and clearing trajectory
"""

import xvisio
import time
import numpy as np
import argparse
from collections import deque
from typing import Optional, Any, Callable

try:
    import viser
    VISER_AVAILABLE = True
except ImportError:
    VISER_AVAILABLE = False
    print("ERROR: viser not available. Install with: pip install viser")
    print("This example requires viser for visualization.")
    exit(1)


class XvisioPoseVisualizer:
    """Visualize Xvisio device pose in viser."""

    def __init__(
        self,
        port: int = 8080,
        trajectory_length: int = 500,
        on_reset_pose: Optional[Callable[[], None]] = None,
        controller_only_mode: bool = False,
    ):
        """Initialize the visualizer.

        Args:
            port: Port for viser web server
            trajectory_length: Maximum number of poses to keep in trajectory
            on_reset_pose: Callback to reset pose reference on device
            controller_only_mode: If True, always show controller frames (no checkbox).
        """
        self.server = viser.ViserServer(port=port)
        self.scene = self.server.scene
        self.gui = self.server.gui
        self.trajectory_length = trajectory_length
        self._on_reset_pose = on_reset_pose
        self._show_imu_checkbox = None
        self._controller_only_mode = controller_only_mode

        # Trajectory storage (position history)
        self.trajectory = deque(maxlen=trajectory_length)

        # Frame handles
        self.pose_frame: Optional[Any] = None
        self.trajectory_line: Optional[Any] = None
        self.imu_vector: Optional[Any] = None
        self.controller_left_frame: Optional[Any] = None
        self.controller_right_frame: Optional[Any] = None
        self._show_controller_checkbox = None
        self._controller_port_input: Optional[Any] = None

        # Add grid for reference
        self.scene.add_grid(
            "/grid",
            width=2.0,
            height=2.0,
        )

        # Add origin frame (world origin)
        self.scene.add_frame(
            "/origin",
            wxyz=(1.0, 0.0, 0.0, 0.0),  # Identity quaternion
            position=(0.0, 0.0, 0.0),
            axes_length=0.15,
            axes_radius=0.006,
        )

        # Setup GUI controls
        self._setup_gui()

        print(f"Viser server started at http://localhost:{port}")
        print("Open this URL in your browser to see the visualization")

    def _setup_gui(self):
        """Setup GUI controls in viser."""
        with self.gui.add_folder("Controls"):
            # Reset pose reference button
            reset_pose_btn = self.gui.add_button("Reset Pose Reference")

            @reset_pose_btn.on_click
            def _(_):
                if self._on_reset_pose:
                    self._on_reset_pose()
                    print("✓ Pose reference reset")

            # Clear trajectory button
            clear_traj_btn = self.gui.add_button("Clear Trajectory")

            @clear_traj_btn.on_click
            def _(_):
                self.reset_trajectory()
                print("✓ Trajectory cleared")

            # Add separator
            self.gui.add_markdown("---")

            # Toggle IMU vector visualization (off by default)
            self._show_imu_checkbox = self.gui.add_checkbox(
                "Show IMU Vector",
                initial_value=False,
            )

            # Toggle controller visualization (off by default)
            if not self._controller_only_mode:
                self._show_controller_checkbox = self.gui.add_checkbox(
                    "Show Controller",
                    initial_value=False,
                )

            # Info text
            self.gui.add_markdown(
                "**Controls:**\n"
                "- **Reset Pose Reference**: Set current pose as origin\n"
                "- **Clear Trajectory**: Remove path history\n"
                "- **Show IMU Vector**: Display IMU acceleration vector\n"
                "- **Show Controller**: Display Seer controller frames"
            )

    def update_pose(self, pose: xvisio.Pose):
        """Update the pose visualization.

        Args:
            pose: Pose object from xvisio
        """
        position = np.array(pose.position, dtype=np.float64)
        quat_wxyz = np.array(pose.quat_wxyz, dtype=np.float64)

        # Update or create pose frame
        if self.pose_frame is None:
            self.pose_frame = self.scene.add_frame(
                "/camera/pose",
                wxyz=tuple(quat_wxyz),
                position=tuple(position),
                axes_length=0.15,
                axes_radius=0.008,
            )
        else:
            # Update existing frame
            self.pose_frame.wxyz = tuple(quat_wxyz)
            self.pose_frame.position = tuple(position)

        # Add to trajectory
        self.trajectory.append(position.copy())

        # Update trajectory line
        if len(self.trajectory) > 1:
            positions = np.array(list(self.trajectory))
            # Create line segments connecting consecutive points
            # Shape: (n_segments, 2, 3) where each segment has 2 points
            n_segments = len(positions) - 1
            seg_points = np.zeros((n_segments, 2, 3), dtype=np.float64)
            seg_points[:, 0, :] = positions[:-1]
            seg_points[:, 1, :] = positions[1:]

            # Create colors for each segment (cyan)
            colors = np.tile(np.array([0.0, 1.0, 1.0]), (n_segments, 2, 1))

            if self.trajectory_line is None:
                self.trajectory_line = self.scene.add_line_segments(
                    "/trajectory",
                    points=seg_points,
                    colors=colors,
                    line_width=2.0,
                )
            else:
                self.trajectory_line.points = seg_points
                self.trajectory_line.colors = colors

    def update_imu(self, imu: xvisio.ImuSample, pose: xvisio.Pose):
        """Update IMU visualization (acceleration vector).

        Args:
            imu: IMU sample from xvisio
            pose: Current pose (for position)
        """
        if not self.is_imu_enabled():
            self.clear_imu()
            return

        position = np.array(pose.position, dtype=np.float64)
        accel = np.array(imu.accel, dtype=np.float64)

        # Scale acceleration for visualization (normalize to reasonable length)
        accel_magnitude = np.linalg.norm(accel)
        if accel_magnitude > 0.01:  # Only show if significant
            # Scale to 0.1m length for visualization
            scale = 0.1 / accel_magnitude
            accel_scaled = accel * scale

            # Create line segment from position to position + scaled acceleration
            end_point = position + accel_scaled
            # Shape: (1, 2, 3) for one segment with 2 points
            seg_points = np.array([[position, end_point]], dtype=np.float64)
            colors = np.array([[[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]]], dtype=np.float64)  # Red

            if self.imu_vector is None:
                self.imu_vector = self.scene.add_line_segments(
                    "/imu/accel",
                    points=seg_points,
                    colors=colors,
                    line_width=3.0,
                )
            else:
                self.imu_vector.points = seg_points
                self.imu_vector.colors = colors

    def is_imu_enabled(self) -> bool:
        """Return True when IMU visualization is enabled."""
        if self._show_imu_checkbox is None:
            return False
        return bool(self._show_imu_checkbox.value)

    def clear_imu(self):
        """Remove IMU visualization from the scene."""
        if self.imu_vector is not None:
            self.imu_vector.remove()
            self.imu_vector = None

    def is_controller_enabled(self) -> bool:
        """Return True when controller visualization is enabled."""
        if self._controller_only_mode:
            return True
        if self._show_controller_checkbox is None:
            return False
        return bool(self._show_controller_checkbox.value)

    def get_controller_port(self) -> str:
        """Return the controller serial port from GUI."""
        if self._controller_port_input is None:
            return "/dev/ttyUSB0"
        return str(self._controller_port_input.value).strip() or "/dev/ttyUSB0"

    def update_controller(self, left: Optional[xvisio.ControllerData], right: Optional[xvisio.ControllerData]):
        """Update controller frame visualization (left=blue, right=green)."""
        if not self.is_controller_enabled():
            self.clear_controller()
            return
        for data, path, color, frame_attr in [
            (left, "/controller/left", (0.2, 0.4, 1.0), "controller_left_frame"),
            (right, "/controller/right", (0.2, 0.9, 0.3), "controller_right_frame"),
        ]:
            if data is None:
                continue
            pos = np.array(data.position, dtype=np.float64)
            quat = np.array(data.quat_wxyz, dtype=np.float64)
            frame = getattr(self, frame_attr)
            if frame is None:
                frame = self.scene.add_frame(
                    path,
                    wxyz=tuple(quat),
                    position=tuple(pos),
                    axes_length=0.08,
                    axes_radius=0.004,
                )
                setattr(self, frame_attr, frame)
            else:
                frame.wxyz = tuple(quat)
                frame.position = tuple(pos)

    def clear_controller(self):
        """Remove controller frames from the scene."""
        for attr in ("controller_left_frame", "controller_right_frame"):
            frame = getattr(self, attr, None)
            if frame is not None:
                frame.remove()
                setattr(self, attr, None)

    def reset_trajectory(self):
        """Clear the trajectory history."""
        self.trajectory.clear()
        if self.trajectory_line is not None:
            self.trajectory_line.remove()  # Use handle's remove() method
            self.trajectory_line = None


def main():
    parser = argparse.ArgumentParser(description="Visualize Xvisio pose frames in viser")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose output")
    parser.add_argument("-p", "--port", type=int, default=8080, help="Viser server port (default: 8080)")
    parser.add_argument("-r", "--rate", type=float, default=100.0, help="Update rate in Hz (default: 100)")
    parser.add_argument("--controller-port", type=str, default="/dev/ttyUSB0", help="Controller serial port (default: /dev/ttyUSB0)")
    args = parser.parse_args()

    # Calculate sleep time from rate
    sleep_time = 1.0 / args.rate if args.rate > 0 else 0.01

    print("=== Xvisio Pose Visualization Demo ===\n")

    if not VISER_AVAILABLE:
        print("ERROR: viser is not installed.")
        print("Install with: pip install viser")
        return

    # Discover: try camera first, then controller
    print("Discovering devices...")
    devices = xvisio.discover()
    controller_devices = []
    if not devices:
        controller_devices = xvisio.discover_controllers()
        if not controller_devices:
            print("ERROR: No Xvisio devices (camera or controller) found!")
            print("\nTroubleshooting:")
            print("1. Camera: connect XR-50 via USB; run 'sudo ./scripts/setup_host.sh'")
            print("2. Controller: connect Seer dongle (e.g. /dev/ttyUSB0); run udev rules for ttyUSB")
            print("3. If you were just added to plugdev group, log out and log back in")
            return

    use_controller_only = len(devices) == 0 and len(controller_devices) > 0
    if use_controller_only:
        devices = controller_devices
        print("No camera found; using Seer controller only.")
    else:
        print(f"Found {len(devices)} camera device(s):")
        for d in devices:
            print(f"  - Serial: {d.serial_number}, Model: {d.model}")

    print("\nOpening device...")
    try:
        if use_controller_only:
            dev = xvisio.open_controller(port=args.controller_port)

            # Initialize controller pose reference for relative tracking
            print("Waiting for controller data to initialize...")
            time.sleep(0.5)
            dev.reset_controller_reference()
            print("✓ Controller pose reference initialized\n")

            def on_reset_controller():
                dev.reset_controller_reference()
                viz.reset_trajectory()

            viz = XvisioPoseVisualizer(
                port=args.port,
                on_reset_pose=on_reset_controller,
                controller_only_mode=True,
            )

            print("\nVisualization is running (controller mode)!")
            print(f"Open http://localhost:{args.port} in your browser")
            print("Left/right controller frames with world-aligned orientation")
            print("Use 'Reset Pose Reference' button to set current pose as origin")
            print("Press Ctrl+C to stop\n")
            frame_count = 0
            try:
                while True:
                    # Use relative controller pose (world-aligned with offset from reference)
                    left, right = dev.controller_relative()

                    # Use first available pose for single "pose" trajectory
                    if left is not None:
                        from types import SimpleNamespace
                        pose_like = SimpleNamespace(
                            position=left.position,
                            quat_wxyz=left.quat_wxyz,
                            confidence=1.0,
                        )
                        viz.update_pose(pose_like)
                    if right is not None and left is None:
                        from types import SimpleNamespace
                        pose_like = SimpleNamespace(
                            position=right.position,
                            quat_wxyz=right.quat_wxyz,
                            confidence=1.0,
                        )
                        viz.update_pose(pose_like)
                    viz.update_controller(left, right)
                    frame_count += 1
                    if (left or right) and frame_count % max(1, int(args.rate)) == 0:
                        for name, c in [("L", left), ("R", right)]:
                            if c is not None:
                                print(f"  {name}: pos=({c.position[0]:+.3f},{c.position[1]:+.3f},{c.position[2]:+.3f}) "
                                      f"trigger={c.key_trigger} side={c.key_side} rocker=({c.rocker_x},{c.rocker_y}) key={c.key}")
                    time.sleep(sleep_time)
            except KeyboardInterrupt:
                print("\n\nStopped by user")
                print(f"Collected {len(viz.trajectory)} trajectory points")
            dev.close()
            return
        # Camera mode
        with xvisio.open() as dev:
            print(f"Opened device: {dev.serial_number}\n")

            # Enable streams
            print("Enabling SLAM and IMU...")
            dev.enable(slam=True, imu=True)
            print("✓ Streams enabled")

            # Wait for initialization
            print("Waiting for streams to initialize...")
            print("  (SLAM needs time to process initial frames)")
            time.sleep(2.0)
            print("✓ Ready\n")

            # Initialize pose reference for relative tracking
            dev.reset_pose_reference()
            print("✓ Pose reference initialized\n")

            viz = XvisioPoseVisualizer(
                port=args.port,
                on_reset_pose=lambda: (dev.reset_pose_reference(), viz.reset_trajectory()),
            )

            # Optional: open controller if available (for "Show Controller" checkbox)
            controller_dev = None
            try:
                controller_devices_check = xvisio.discover_controllers()
                if controller_devices_check:
                    controller_dev = xvisio.open_controller(port=args.controller_port)
            except Exception:
                controller_dev = None

            print("\nVisualization is running!")
            print(f"Open http://localhost:{args.port} in your browser")
            print(f"Update rate: {args.rate:.0f} Hz (adjust with --rate)")
            print("Use the GUI buttons to reset pose or clear trajectory")
            if controller_dev:
                print("Check 'Show Controller' to display Seer controller pose(s)")
            print("Press Ctrl+C to stop\n")

            frame_count = 0
            try:
                while True:
                    pose = dev.pose_relative()
                    viz.update_pose(pose)

                    if viz.is_imu_enabled():
                        try:
                            imu = dev.imu()
                            viz.update_imu(imu, pose)
                        except RuntimeError:
                            pass
                    else:
                        viz.clear_imu()

                    if controller_dev and viz.is_controller_enabled():
                        left, right = controller_dev.controller()
                        viz.update_controller(left, right)
                    else:
                        viz.clear_controller()

                    frame_count += 1
                    print_interval = max(1, int(args.rate))
                    if args.verbose and frame_count % print_interval == 0:
                        pos = pose.position
                        print(f"[{frame_count:5d}] pos=({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}) "
                              f"conf={pose.confidence:.2f}")
                        if controller_dev and viz.is_controller_enabled():
                            for name, c in [("L", left), ("R", right)]:
                                if c is not None:
                                    print(f"  {name}: trigger={c.key_trigger} side={c.key_side} "
                                          f"rocker=({c.rocker_x},{c.rocker_y}) key={c.key}")

                    time.sleep(sleep_time)

            except KeyboardInterrupt:
                print("\n\nStopped by user")
                print(f"Collected {len(viz.trajectory)} trajectory points")
            finally:
                if controller_dev is not None:
                    controller_dev.close()

    except Exception as e:
        print(f"\nERROR: {e}")
        print("\nTroubleshooting:")
        print("1. Make sure device is connected and powered")
        print("2. Check that udev rules are installed: sudo ./scripts/setup_host.sh")
        print("3. Verify XVSDK is installed: ls /usr/lib/libxvsdk.so")
        raise


if __name__ == "__main__":
    main()
