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

import xvisio
import time

def main():
    print("=== Xvisio Pose + IMU Demo ===\n")

    # Check for devices
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
