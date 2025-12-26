#!/usr/bin/env python3
"""Demo script showing pose transform convenience methods.

This demonstrates:
1. Raw pose (as-is from SDK)
2. World-aligned pose (with rotation offset applied)
3. Relative pose (delta from reference pose)
"""

import time
import xvisio


def main():
    print("=== Xvisio Pose Transforms Demo ===\n")

    # Discover devices
    print("Discovering devices...")
    devices = xvisio.discover()
    if not devices:
        print("ERROR: No devices found")
        print("\nTroubleshooting:")
        print("1. Make sure device is connected and powered")
        print("2. Check that udev rules are installed: sudo ./scripts/setup_host.sh")
        print("3. Verify XVSDK is installed: ls /usr/lib/libxvsdk.so")
        return

    print(f"Found {len(devices)} device(s):")
    for dev in devices:
        print(f"  - {dev.serial_number} ({dev.model})")
    print()

    # Open device
    with xvisio.open() as dev:
        print(f"Opened device: {dev.serial_number}\n")

        # Enable SLAM
        print("Enabling SLAM...")
        dev.enable(slam=True)
        print("✓ SLAM enabled")

        # Wait for initialization
        print("Waiting for SLAM to initialize...")
        time.sleep(2.0)
        print("✓ Ready\n")

        # Get a few raw poses
        print("Collecting raw poses (5 samples)...")
        print(f"{'Time':<8} {'Position (x,y,z)':<30} {'Quat (w,x,y,z)':<35} {'Confidence':<10}")
        print("-" * 100)

        for i in range(5):
            try:
                pose = dev.pose()
                pos_str = f"({pose.position[0]:.3f},{pose.position[1]:.3f},{pose.position[2]:.3f})"
                quat_str = f"({pose.quat_wxyz[0]:.3f},{pose.quat_wxyz[1]:.3f},{pose.quat_wxyz[2]:.3f},{pose.quat_wxyz[3]:.3f})"
                print(f"{i+1:<8} {pos_str:<30} {quat_str:<35} {pose.confidence:.3f}")
                time.sleep(0.2)
            except Exception as e:
                print(f"Error: {e}")
                break

        print("\n" + "=" * 100)
        print("World-aligned poses (rotation offset applied):")
        print(f"{'Time':<8} {'Position (x,y,z)':<30} {'Quat (w,x,y,z)':<35}")
        print("-" * 100)

        for i in range(5):
            try:
                pose = dev.pose_world_aligned()
                pos_str = f"({pose.position[0]:.3f},{pose.position[1]:.3f},{pose.position[2]:.3f})"
                quat_str = f"({pose.quat_wxyz[0]:.3f},{pose.quat_wxyz[1]:.3f},{pose.quat_wxyz[2]:.3f},{pose.quat_wxyz[3]:.3f})"
                print(f"{i+1:<8} {pos_str:<30} {quat_str:<35}")
                time.sleep(0.2)
            except Exception as e:
                print(f"Error: {e}")
                break

        print("\n" + "=" * 100)
        print("Relative poses (delta from reference):")
        print("Setting reference pose...")
        dev.reset_pose_reference()
        print("✓ Reference set\n")

        print(f"{'Time':<8} {'Position (x,y,z)':<30} {'Quat (w,x,y,z)':<35}")
        print("-" * 100)

        for i in range(10):
            try:
                pose = dev.pose_relative()
                pos_str = f"({pose.position[0]:.3f},{pose.position[1]:.3f},{pose.position[2]:.3f})"
                quat_str = f"({pose.quat_wxyz[0]:.3f},{pose.quat_wxyz[1]:.3f},{pose.quat_wxyz[2]:.3f},{pose.quat_wxyz[3]:.3f})"
                print(f"{i+1:<8} {pos_str:<30} {quat_str:<35}")
                time.sleep(0.2)
            except Exception as e:
                print(f"Error: {e}")
                break

        print("\n✓ Demo complete")


if __name__ == "__main__":
    main()

