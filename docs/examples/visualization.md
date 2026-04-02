# 3D Visualization Example

Live 3D pose visualization in the browser using [viser](https://viser.studio).

## Overview

- Streams pose from the device at 30 Hz
- Renders a 3D coordinate frame that moves with the device in real time
- Accessible at `http://localhost:8080` in any browser
- Requires the `xvisio[examples]` extra (`pip install xvisio[examples]`)

## Running

```bash
pip install xvisio[examples]
python examples/demo_pose_visualization.py
# Then open http://localhost:8080
```

With pixi:

```bash
pixi run demo_pose_visualization
```

## Code

```python
#!/usr/bin/env python3
"""Live 3D pose visualization with viser."""
import time
import numpy as np
import xvisio
import viser

def main():
    server = viser.ViserServer()
    print("Open http://localhost:8080 in your browser")

    with xvisio.open() as dev:
        dev.enable(slam=True)
        time.sleep(2.0)   # wait for SLAM initialization

        frame = server.scene.add_frame(
            "/device",
            wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            position=np.array([0.0, 0.0, 0.0]),
            axes_length=0.1,
        )

        for pose in dev.poses(rate_hz=30):
            frame.wxyz     = np.array(pose.quat_wxyz)
            frame.position = np.array(pose.position)

if __name__ == "__main__":
    main()
```

## Explanation

1. **`viser.ViserServer()`** — Starts a lightweight WebGL server on port 8080. Any browser on the same network can connect.
2. **`server.scene.add_frame()`** — Creates a 3D coordinate frame widget (RGB axes) in the scene.
3. **`dev.poses(rate_hz=30)`** — Iterator that yields world-aligned poses at 30 Hz. The loop updates the frame's position and orientation on every yield.
4. The visualization updates live as you move the device. Press `Ctrl+C` to stop.

## Next Steps

- [Pose + IMU Example](pose_imu.md)
- [Pose Transforms Example](pose_transforms.md)
