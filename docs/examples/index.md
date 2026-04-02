# Examples

All example scripts are included in the pip package. Use `xvisio-examples --copy` to get a local copy, or run them directly from the `examples/` directory of a source clone.

## Available Examples

### Core

| Example | Script | Description |
|---------|--------|-------------|
| [Pose + IMU](pose_imu.md) | `demo_pose_imu.py` | Stream world-aligned pose and IMU; includes Seer controller fallback |
| [Pose Transforms](pose_transforms.md) | `demo_pose_transforms.py` | Compare raw, world-aligned, and relative poses side by side |

### Visualization

| Example | Script | Description |
|---------|--------|-------------|
| [3D Visualization](visualization.md) | `demo_pose_visualization.py` | Live 3D pose visualization in the browser via viser |

### Benchmarks

| Script | Description |
|--------|-------------|
| `benchmark_pose_rate.py` | Measure achievable pose streaming rate |

## Running Examples

=== "From pip install"

    ```bash
    xvisio-examples --copy
    cd xvisio_examples
    python demo_pose_imu.py
    ```

=== "From source clone"

    ```bash
    python examples/demo_pose_imu.py
    ```

=== "With pixi"

    ```bash
    pixi run demo_pose_imu
    pixi run demo_pose_transforms
    pixi run demo_pose_visualization
    ```
