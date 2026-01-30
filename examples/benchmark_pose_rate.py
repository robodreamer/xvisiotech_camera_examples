#!/usr/bin/env python3
"""
Benchmark the maximum pose update rate from the Xvisio driver.

This example measures:
- Maximum achievable pose polling rate
- Latency statistics (min, max, mean, std)
- Pose update frequency from the SDK
"""

import xvisio
import time
import numpy as np
import argparse
from collections import deque


def run_benchmark(get_sample, duration_s: float, warmup_s: float = 2.0) -> dict:
    """Run pose rate benchmark.

    Args:
        get_sample: Callable returning (position, edge_ts_us or None)
        duration_s: Duration of benchmark in seconds
        warmup_s: Warmup period before measuring

    Returns:
        Dictionary with benchmark results
    """
    print(f"Warming up for {warmup_s}s...")
    warmup_start = time.perf_counter()
    while time.perf_counter() - warmup_start < warmup_s:
        try:
            get_sample()
        except RuntimeError:
            pass

    print(f"Running benchmark for {duration_s}s...")

    # Track timing
    inter_sample_times = []
    pose_update_times = []  # Host times when we detect a new pose

    # For detecting actual pose updates, we track position changes
    # The SDK may interpolate poses, so edge timestamps change continuously
    # We detect "new" poses by looking for significant position changes
    POSITION_THRESHOLD = 1e-6  # 1 micrometer - detect any real movement

    start_time = time.perf_counter()
    last_time = start_time
    sample_count = 0
    error_count = 0
    duplicate_count = 0

    last_position = None
    last_edge_ts = None
    edge_ts_jumps = []  # Track significant timestamp jumps

    while True:
        current_time = time.perf_counter()
        if current_time - start_time >= duration_s:
            break

        try:
            position, edge_ts = get_sample()
            sample_count += 1

            # Record host-side timing
            inter_sample_times.append(current_time - last_time)
            last_time = current_time

            # Detect actual pose updates by checking if position changed
            current_position = np.array(position)

            if last_position is not None:
                position_change = np.linalg.norm(current_position - last_position)

                if position_change > POSITION_THRESHOLD:
                    # New pose detected
                    pose_update_times.append(current_time)

                    # Also track edge timestamp jump
                    if edge_ts is not None and last_edge_ts is not None:
                        ts_jump = edge_ts - last_edge_ts
                        if ts_jump > 0:  # Ignore negative jumps (wraparound)
                            edge_ts_jumps.append(ts_jump)
                else:
                    duplicate_count += 1
            else:
                # First pose
                pose_update_times.append(current_time)

            last_position = current_position
            if edge_ts is not None:
                last_edge_ts = edge_ts

        except RuntimeError:
            error_count += 1

    end_time = time.perf_counter()
    total_time = end_time - start_time

    # Calculate statistics
    inter_sample_times = np.array(inter_sample_times[1:])  # Skip first

    # Calculate actual pose update rate
    unique_poses = len(pose_update_times)
    if len(pose_update_times) > 1:
        pose_intervals = np.diff(pose_update_times)
        actual_pose_rate = 1.0 / np.mean(pose_intervals) if np.mean(pose_intervals) > 0 else 0
    else:
        actual_pose_rate = 0
        pose_intervals = np.array([])

    # Edge timestamp statistics (in microseconds)
    edge_ts_jumps = np.array(edge_ts_jumps)

    results = {
        "total_samples": sample_count,
        "total_time_s": total_time,
        "polling_rate_hz": sample_count / total_time if total_time > 0 else 0,
        "unique_poses": unique_poses,
        "duplicate_count": duplicate_count,
        "actual_pose_rate_hz": actual_pose_rate,
        "inter_sample_mean_ms": np.mean(inter_sample_times) * 1000 if len(inter_sample_times) > 0 else 0,
        "inter_sample_std_ms": np.std(inter_sample_times) * 1000 if len(inter_sample_times) > 0 else 0,
        "inter_sample_min_ms": np.min(inter_sample_times) * 1000 if len(inter_sample_times) > 0 else 0,
        "inter_sample_max_ms": np.max(inter_sample_times) * 1000 if len(inter_sample_times) > 0 else 0,
        "error_count": error_count,
    }

    if len(pose_intervals) > 0:
        results["pose_interval_mean_ms"] = np.mean(pose_intervals) * 1000
        results["pose_interval_std_ms"] = np.std(pose_intervals) * 1000
        results["pose_interval_min_ms"] = np.min(pose_intervals) * 1000
        results["pose_interval_max_ms"] = np.max(pose_intervals) * 1000

    if len(edge_ts_jumps) > 0:
        results["edge_ts_jump_mean_ms"] = np.mean(edge_ts_jumps) / 1000  # µs to ms
        results["edge_ts_jump_std_ms"] = np.std(edge_ts_jumps) / 1000
        results["edge_ts_jump_min_ms"] = np.min(edge_ts_jumps) / 1000
        results["edge_ts_jump_max_ms"] = np.max(edge_ts_jumps) / 1000

    return results


def print_results(results: dict):
    """Print benchmark results."""
    print("\n" + "=" * 60)
    print("BENCHMARK RESULTS")
    print("=" * 60)

    print(f"\n{'Polling Performance':^60}")
    print("-" * 60)
    print(f"  Total samples polled:     {results['total_samples']:,}")
    print(f"  Total time:               {results['total_time_s']:.2f} s")
    print(f"  Max polling rate:         {results['polling_rate_hz']:,.0f} Hz")
    print(f"  Errors:                   {results['error_count']}")

    print(f"\n{'Actual Pose Updates (position-based detection)':^60}")
    print("-" * 60)
    print(f"  Unique poses received:    {results['unique_poses']:,}")
    print(f"  Duplicate reads:          {results['duplicate_count']:,}")
    print(f"  Actual pose rate:         {results['actual_pose_rate_hz']:.1f} Hz")

    if 'pose_interval_mean_ms' in results:
        print(f"\n{'Pose Update Intervals (host time)':^60}")
        print("-" * 60)
        print(f"  Mean:                     {results['pose_interval_mean_ms']:.2f} ms")
        print(f"  Std Dev:                  {results['pose_interval_std_ms']:.2f} ms")
        print(f"  Min:                      {results['pose_interval_min_ms']:.2f} ms")
        print(f"  Max:                      {results['pose_interval_max_ms']:.2f} ms")

    if 'edge_ts_jump_mean_ms' in results:
        print(f"\n{'Edge Timestamp Intervals (device time)':^60}")
        print("-" * 60)
        print(f"  Mean:                     {results['edge_ts_jump_mean_ms']:.2f} ms")
        print(f"  Std Dev:                  {results['edge_ts_jump_std_ms']:.2f} ms")
        print(f"  Min:                      {results['edge_ts_jump_min_ms']:.2f} ms")
        print(f"  Max:                      {results['edge_ts_jump_max_ms']:.2f} ms")

    print(f"\n{'Host-side Polling Latency (per call)':^60}")
    print("-" * 60)
    print(f"  Mean:                     {results['inter_sample_mean_ms']*1000:.1f} µs")
    print(f"  Std Dev:                  {results['inter_sample_std_ms']*1000:.1f} µs")
    print(f"  Min:                      {results['inter_sample_min_ms']*1000:.1f} µs")
    print(f"  Max:                      {results['inter_sample_max_ms']*1000:.2f} ms")

    print("\n" + "=" * 60)

    # Summary
    print("\nSUMMARY:")
    print(f"  ✓ SDK provides new poses at ~{results['actual_pose_rate_hz']:.0f} Hz")
    print(f"  ✓ getPose() call latency: ~{results['inter_sample_mean_ms']*1000:.0f} µs")

    if results['duplicate_count'] > 0:
        dup_pct = 100 * results['duplicate_count'] / results['total_samples']
        print(f"  ⚠ {dup_pct:.1f}% of reads returned duplicate poses (polling too fast)")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Benchmark Xvisio pose update rate",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                    # Run 10-second benchmark
  %(prog)s -d 30              # Run 30-second benchmark
  %(prog)s -d 60 -w 5         # 60s benchmark with 5s warmup
"""
    )
    parser.add_argument(
        "-d", "--duration",
        type=float,
        default=10.0,
        help="Benchmark duration in seconds (default: 10)"
    )
    parser.add_argument(
        "-w", "--warmup",
        type=float,
        default=2.0,
        help="Warmup duration in seconds (default: 2)"
    )
    parser.add_argument(
        "--controller-port",
        type=str,
        default="/dev/ttyUSB0",
        help="Controller serial port (default: /dev/ttyUSB0)"
    )
    args = parser.parse_args()

    print("=== Xvisio Pose Rate Benchmark ===\n")

    # Discover devices
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
            return
        print("No camera found; using Seer controller only.")
    else:
        print(f"Found {len(devices)} device(s)")
        for dev in devices:
            print(f"  - Serial: {dev.serial_number}, Model: {dev.model}")

    print("\nOpening device...")
    try:
        if not devices and controller_devices:
            dev = xvisio.open_controller(port=args.controller_port)
            print("Opened controller device\n")

            def get_sample():
                left, right = dev.controller()
                sample = left or right
                if sample is None:
                    raise RuntimeError("No controller data available yet")
                return sample.position, None

            results = run_benchmark(get_sample, args.duration, args.warmup)
            print_results(results)
            dev.close()
            return

        with xvisio.open() as dev:
            print(f"Opened device: {dev.serial_number}\n")

            # Enable SLAM only (no IMU for this benchmark)
            print("Enabling SLAM...")
            dev.enable(slam=True, imu=False)
            print("✓ SLAM enabled\n")

            # Run benchmark
            def get_sample():
                pose = dev.pose()
                return pose.position, pose.edge_timestamp_us

            results = run_benchmark(get_sample, args.duration, args.warmup)

            # Print results
            print_results(results)

    except Exception as e:
        print(f"\nERROR: {e}")
        print("\nTroubleshooting:")
        print("1. Make sure device is connected and powered")
        print("2. Check that udev rules are installed: sudo ./scripts/setup_host.sh")
        print("3. Verify XVSDK is installed: ls /usr/lib/libxvsdk.so")
        raise


if __name__ == "__main__":
    main()

