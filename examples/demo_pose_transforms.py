#!/usr/bin/env python3
"""Demo script showing pose transform convenience methods.

This demonstrates:
1. Raw pose (as-is from SDK)
2. World-aligned pose (with rotation offset applied)
3. Relative pose (delta from reference pose)
"""

import argparse
import time

import xvisio


def main():
    parser = argparse.ArgumentParser(description="Xvisio pose transforms demo")
    parser.add_argument(
        "--controller-port",
        type=str,
        default="/dev/ttyUSB0",
        help="Controller serial port (default: /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--controller-duration",
        type=float,
        default=10.0,
        help="How long to print controller samples in controller-only mode (seconds, default: 10.0)",
    )
    parser.add_argument(
        "--controller-rate",
        type=float,
        default=5.0,
        help="Controller print rate in controller-only mode (Hz, default: 5.0)",
    )
    args = parser.parse_args()

    print("=== Xvisio Pose Transforms Demo ===\n")

    # Discover devices
    print("Discovering devices...")
    devices = xvisio.discover()
    controller_devices = []
    if not devices:
        controller_devices = xvisio.discover_controllers()
        if not controller_devices:
            print("ERROR: No devices found")
            print("\nTroubleshooting:")
            print("1. Camera: connect XR-50 via USB; run 'sudo ./scripts/setup_host.sh'")
            print(
                "2. Controller: connect Seer dongle (e.g. /dev/ttyUSB0); run udev rules for ttyUSB"
            )
            print("3. Verify XVSDK is installed: ls /usr/lib/libxvsdk.so")
            return
        print("No camera found; using Seer controller only.")
    else:
        print(f"Found {len(devices)} device(s):")
        for dev in devices:
            print(f"  - {dev.serial_number} ({dev.model})")
        print()

    # Open device
    if not devices and controller_devices:
        dev = xvisio.open_controller(port=args.controller_port)
        print("Opened controller device\n")
        print(
            f"Controller pose samples ({args.controller_duration:.1f}s at {args.controller_rate:.1f} Hz):"
        )
        print(f"{'Side':<6} {'Position (x,y,z)':<30} {'Quat (w,x,y,z)':<35} {'Buttons':<30}")
        print("-" * 110)
        sleep_s = 1.0 / args.controller_rate if args.controller_rate > 0 else 0.2
        end_time = time.time() + max(0.5, args.controller_duration)
        sample_idx = 0
        left_seen = 0
        right_seen = 0
        while time.time() < end_time:
            left, right = dev.controller()
            sample_idx += 1
            print(f"[sample {sample_idx:03d}]")
            for name, c in [("Left", left), ("Right", right)]:
                if c is None:
                    continue
                if name == "Left":
                    left_seen += 1
                else:
                    right_seen += 1
                pos_str = f"({c.position[0]:.3f},{c.position[1]:.3f},{c.position[2]:.3f})"
                quat_str = f"({c.quat_wxyz[0]:.3f},{c.quat_wxyz[1]:.3f},{c.quat_wxyz[2]:.3f},{c.quat_wxyz[3]:.3f})"
                btn_str = f"trig={c.key_trigger} side={c.key_side} key={c.key} rocker=({c.rocker_x},{c.rocker_y})"
                print(f"{name:<6} {pos_str:<30} {quat_str:<35} {btn_str:<30}")
            time.sleep(sleep_s)
        left_pct = 100.0 * left_seen / sample_idx if sample_idx > 0 else 0.0
        right_pct = 100.0 * right_seen / sample_idx if sample_idx > 0 else 0.0
        print("\nController packet summary:")
        print(f"  Samples polled: {sample_idx}")
        print(f"  Left packets:   {left_seen} ({left_pct:.1f}% of polls)")
        print(f"  Right packets:  {right_seen} ({right_pct:.1f}% of polls)")
        print("\n✓ Controller demo complete")
        dev.close()
        return

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
