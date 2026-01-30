#!/usr/bin/env python3
"""
Simple demo showing pose + IMU usage with xvisio.

This script demonstrates the minimal-lines API:
- Open device
- Enable SLAM and IMU
- Print world-aligned pose and IMU data for 10 seconds

Note: Uses pose_world_aligned() by default, which applies rotation offset
to align camera frame with world frame (matching ROS2 teleop handler).
"""

import argparse
import xvisio
import time

def main():
    parser = argparse.ArgumentParser(description="Xvisio pose + IMU demo")
    parser.add_argument(
        "--controller-port",
        type=str,
        default="/dev/ttyUSB0",
        help="Controller serial port (default: /dev/ttyUSB0)",
    )
    args = parser.parse_args()

    print("=== Xvisio Pose + IMU Demo ===\n")

    # Check for devices
    print("Discovering devices...")
    devices = xvisio.discover()
    controller_devices = []
    if not devices:
        controller_devices = xvisio.discover_controllers()
        if not controller_devices:
            print("ERROR: No Xvisio devices found!")
            print("\nTroubleshooting:")
            print("1. Camera: connect XR-50 via USB; run 'sudo ./scripts/setup_host.sh'")
            print("2. Controller: connect Seer dongle (e.g. /dev/ttyUSB0); run udev rules for ttyUSB")
            print("3. If you were just added to plugdev group, log out and log back in")
            return
        print("No camera found; using Seer controller only.")
    else:
        print(f"Found {len(devices)} device(s):")
        for dev in devices:
            print(f"  - Serial: {dev.serial_number}, Model: {dev.model}")

    print("\nOpening device...")
    try:
        if not devices and controller_devices:
            dev = xvisio.open_controller(port=args.controller_port)
            print("Opened controller device\n")
            print("Collecting controller data for 10 seconds (Ctrl+C to stop early)...\n")
            start_time = time.time()
            count = 0
            try:
                while time.time() - start_time < 10.0:
                    left, right = dev.controller()
                    for name, c in [("L", left), ("R", right)]:
                        if c is None:
                            continue
                        pos_str = f"({c.position[0]:.3f},{c.position[1]:.3f},{c.position[2]:.3f})"
                        quat_str = f"({c.quat_wxyz[0]:.3f},{c.quat_wxyz[1]:.3f},{c.quat_wxyz[2]:.3f},{c.quat_wxyz[3]:.3f})"
                        print(
                            f"{name} {pos_str:<30} {quat_str:<35} "
                            f"trigger={c.key_trigger} side={c.key_side} "
                            f"rocker=({c.rocker_x},{c.rocker_y}) key={c.key}"
                        )
                    count += 1
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\n\nInterrupted by user")
            print(f"\n✓ Collected {count} samples")
            dev.close()
            return

        with xvisio.open() as dev:
            print(f"Opened device: {dev.serial_number}\n")

            # Enable streams (one call!)
            print("Enabling SLAM and IMU...")
            dev.enable(slam=True, imu=True)
            print("✓ Streams enabled")

            # Give streams a moment to start (especially IMU callback and SLAM initialization)
            print("Waiting for streams to initialize...")
            print("  (SLAM needs time to process initial frames - this may take a few seconds)")
            time.sleep(2.0)  # Give SLAM more time to initialize with fisheye cameras
            print("✓ Ready\n")

            print("Collecting data for 10 seconds (Ctrl+C to stop early)...\n")
            print("Using world-aligned poses (rotation offset applied for world frame alignment)")
            print(f"{'Time':<8} {'Position (x,y,z)':<30} {'Quat (w,x,y,z)':<35} {'Confidence':<10}")
            print("-" * 100)

            start_time = time.time()
            count = 0

            try:
                while time.time() - start_time < 10.0:
                    # Get world-aligned pose (applies rotation offset to align camera frame with world frame)
                    pose = dev.pose_world_aligned()
                    pos_str = f"({pose.position[0]:.3f},{pose.position[1]:.3f},{pose.position[2]:.3f})"
                    quat_str = f"({pose.quat_wxyz[0]:.3f},{pose.quat_wxyz[1]:.3f},{pose.quat_wxyz[2]:.3f},{pose.quat_wxyz[3]:.3f})"

                    # Get IMU
                    imu = dev.imu()
                    accel_str = f"accel=({imu.accel[0]:.2f},{imu.accel[1]:.2f},{imu.accel[2]:.2f})"
                    gyro_str = f"gyro=({imu.gyro[0]:.2f},{imu.gyro[1]:.2f},{imu.gyro[2]:.2f})"

                    elapsed = time.time() - start_time
                    print(f"{elapsed:6.2f}s  {pos_str:<30} {quat_str:<35} {pose.confidence:.3f}")
                    print(f"         IMU: {accel_str} {gyro_str}")

                    count += 1
                    time.sleep(0.1)  # ~10 Hz

            except KeyboardInterrupt:
                print("\n\nInterrupted by user")

            print(f"\n✓ Collected {count} samples")
            print("Device closed automatically")

    except Exception as e:
        print(f"\nERROR: {e}")
        print("\nTroubleshooting:")
        print("1. Make sure device is connected and powered")
        print("2. Check that udev rules are installed: sudo ./scripts/setup_host.sh")
        print("3. Verify XVSDK is installed: ls /usr/lib/libxvsdk.so")
        raise

if __name__ == "__main__":
    main()
