import os

import pytest


@pytest.mark.skipif(os.environ.get("XVISIO_HARDWARE") != "1", reason="Set XVISIO_HARDWARE=1 to run")
def test_hardware_pose_and_imu_smoke():
    import xvisio

    with xvisio.open() as dev:
        dev.enable(slam=True, imu=True)
        pose = dev.pose()
        imu = dev.imu()

    assert len(pose.position) == 3
    assert len(pose.quat_wxyz) == 4
    assert isinstance(pose.host_timestamp_s, float)

    assert len(imu.accel) == 3
    assert len(imu.gyro) == 3
    assert isinstance(imu.host_timestamp_s, float)
