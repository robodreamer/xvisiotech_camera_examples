import types

import pytest


def _install_mock_backend(monkeypatch):
    import xvisio._highlevel as hl

    class DevInfo:
        def __init__(self, serial_number, model):
            self.serial_number = serial_number
            self.model = model

    class PoseObj:
        def __init__(self):
            self.position = (1.0, 2.0, 3.0)
            # w, x, y, z
            self.quaternion = (1.0, 0.0, 0.0, 0.0)
            self.host_timestamp_s = 123.0
            self.edge_timestamp_us = 456
            self.confidence = 0.9

    class ImuObj:
        def __init__(self):
            self.accel = (0.1, 0.2, 0.3)
            self.gyro = (1.1, 1.2, 1.3)
            self.host_timestamp_s = 124.0
            self.edge_timestamp_us = 0

    class NativeDevice:
        def __init__(self):
            self._slam = False
            self._imu = False

        def serial_number(self):
            return "MOCK123"

        def start_slam(self):
            self._slam = True
            return True

        def stop_slam(self):
            self._slam = False
            return True

        def slam_running(self):
            return self._slam

        def start_imu(self):
            self._imu = True
            return True

        def stop_imu(self):
            self._imu = False
            return True

        def imu_running(self):
            return self._imu

        def get_pose(self, prediction_s=0.0):
            assert isinstance(prediction_s, float)
            return PoseObj()

        def get_pose_at(self, host_timestamp_s):
            assert isinstance(host_timestamp_s, float)
            return PoseObj()

        def get_imu(self):
            return ImuObj()

    mock = types.SimpleNamespace(
        discover_devices=lambda: [DevInfo("MOCK123", "XR-50")],
        open_device=lambda sn="": NativeDevice(),
    )

    monkeypatch.setattr(hl, "_load_native", lambda: mock)


def test_import_without_native_module():
    # Should import even if native module isn't built (lazy import)
    import xvisio

    assert hasattr(xvisio, "open")
    assert hasattr(xvisio, "discover")


def test_discover_and_open(monkeypatch):
    _install_mock_backend(monkeypatch)

    import xvisio

    devices = xvisio.discover()
    assert len(devices) == 1
    assert devices[0].serial_number == "MOCK123"

    dev = xvisio.open()
    assert dev.serial_number == "MOCK123"


def test_enable_pose_imu(monkeypatch):
    _install_mock_backend(monkeypatch)

    import xvisio

    with xvisio.open() as dev:
        dev.enable(slam=True, imu=True)
        pose = dev.pose()
        imu = dev.imu()

        assert pose.position == (1.0, 2.0, 3.0)
        assert pose.quat_wxyz == (1.0, 0.0, 0.0, 0.0)
        assert isinstance(pose.edge_timestamp_us, int)

        assert imu.accel == (0.1, 0.2, 0.3)
        assert imu.gyro == (1.1, 1.2, 1.3)


def test_pose_requires_enable(monkeypatch):
    _install_mock_backend(monkeypatch)

    import xvisio

    dev = xvisio.open()
    with pytest.raises(RuntimeError):
        dev.pose()


def test_imu_requires_enable(monkeypatch):
    _install_mock_backend(monkeypatch)

    import xvisio

    dev = xvisio.open()
    with pytest.raises(RuntimeError):
        dev.imu()
