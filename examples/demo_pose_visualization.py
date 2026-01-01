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
    ):
        """Initialize the visualizer.

        Args:
            port: Port for viser web server
            trajectory_length: Maximum number of poses to keep in trajectory
            on_reset_pose: Callback to reset pose reference on device
        """
        self.server = viser.ViserServer(port=port)
        self.scene = self.server.scene
        self.gui = self.server.gui
        self.trajectory_length = trajectory_length
        self._on_reset_pose = on_reset_pose

        # Trajectory storage (position history)
        self.trajectory = deque(maxlen=trajectory_length)

        # Frame handles
        self.pose_frame: Optional[Any] = None
        self.trajectory_line: Optional[Any] = None
        self.imu_vector: Optional[Any] = None

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

            # Info text
            self.gui.add_markdown(
                "**Controls:**\n"
                "- **Reset Pose Reference**: Set current pose as origin\n"
                "- **Clear Trajectory**: Remove path history"
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
    args = parser.parse_args()

    print("=== Xvisio Pose Visualization Demo ===\n")

    if not VISER_AVAILABLE:
        print("ERROR: viser is not installed.")
        print("Install with: pip install viser")
        return

    # Discover devices
    print("Discovering devices...")
    devices = xvisio.discover()
    if not devices:
        print("ERROR: No Xvisio devices found!")
        print("\nTroubleshooting:")
        print("1. Make sure device is connected via USB")
        print("2. Run 'sudo ./scripts/setup_host.sh' to install udev rules")
        print("3. If you were just added to plugdev group, log out and log back in")
        return

    print(f"Found {len(devices)} device(s):")
    for dev in devices:
        print(f"  - Serial: {dev.serial_number}, Model: {dev.model}")

    print("\nOpening device...")
    try:
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
            # This matches the ROS2 teleop handler behavior:
            # - Store initial position and rotation
            # - All subsequent poses are relative to this initial pose
            # - Apply R_offset to align camera frame with world frame
            dev.reset_pose_reference()
            print("✓ Pose reference initialized\n")

            # Create visualizer with reset callback
            print("Starting visualization...")
            viz = XvisioPoseVisualizer(
                port=args.port,
                on_reset_pose=lambda: (dev.reset_pose_reference(), viz.reset_trajectory()),
            )

            print("\nVisualization is running!")
            print(f"Open http://localhost:{args.port} in your browser")
            print("Use the GUI buttons to reset pose or clear trajectory")
            print("Press Ctrl+C to stop\n")

            frame_count = 0
            try:
                while True:
                    # Get RELATIVE pose (matches ROS2 teleop handler behavior)
                    # This applies:
                    #   pos_rel = R_offset @ R_init.T @ (pos - pos_init)
                    #   rot_rel = R_offset @ R_init.T @ R @ R_offset.T
                    # Where R_offset = [[0,0,1],[1,0,0],[0,1,0]] aligns camera->world
                    pose = dev.pose_relative()

                    # Update pose visualization
                    viz.update_pose(pose)

                    # Get IMU data
                    try:
                        imu = dev.imu()
                        viz.update_imu(imu, pose)
                    except RuntimeError:
                        # IMU might not be available yet
                        pass

                    # Print status (verbose: every second, normal: silent)
                    frame_count += 1
                    if args.verbose and frame_count % 10 == 0:
                        pos = pose.position
                        print(f"[{frame_count:5d}] pos=({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}) "
                              f"conf={pose.confidence:.2f}")

                    time.sleep(0.1)  # ~10 Hz update rate

            except KeyboardInterrupt:
                print("\n\nStopped by user")
                print(f"Collected {len(viz.trajectory)} trajectory points")

    except Exception as e:
        print(f"\nERROR: {e}")
        print("\nTroubleshooting:")
        print("1. Make sure device is connected and powered")
        print("2. Check that udev rules are installed: sudo ./scripts/setup_host.sh")
        print("3. Verify XVSDK is installed: ls /usr/lib/libxvsdk.so")
        raise


if __name__ == "__main__":
    main()
