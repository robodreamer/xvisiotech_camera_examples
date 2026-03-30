import types

import numpy as np


class _ControllerObj:
    def __init__(self, side: str, position, quat_wxyz=(1.0, 0.0, 0.0, 0.0), ts: float = 100.0):
        self.type = 0 if side == "left" else 1
        self.position = tuple(position)
        self.quaternion = tuple(quat_wxyz)
        self.host_timestamp_s = ts
        self.key_trigger = 0
        self.key_side = 0
        self.rocker_x = 0
        self.rocker_y = 0
        self.key = 0


class _NativeControllerDevice:
    def __init__(self):
        self._controller_running = False
        self._left = _ControllerObj("left", (0.0, 0.0, 0.0))
        self._right = _ControllerObj("right", (0.0, 0.0, 0.0))

    def serial_number(self):
        return "CTRL-MOCK-001"

    def start_controller(self, _port):
        self._controller_running = True
        return True

    def stop_controller(self):
        self._controller_running = False
        return True

    def controller_running(self):
        return self._controller_running

    def get_controller_data(self):
        return self._left, self._right

    def set_controller_pose(self, side: str, position, quat_wxyz=(1.0, 0.0, 0.0, 0.0)):
        obj = _ControllerObj(side, position, quat_wxyz)
        if side == "left":
            self._left = obj
        else:
            self._right = obj

    def set_controller_available(self, side: str, available: bool):
        if side == "left":
            self._left = self._left if available else None
        else:
            self._right = self._right if available else None


def _open_with_controller_mock(monkeypatch):
    import xvisio._highlevel as hl

    native = _NativeControllerDevice()
    mock_module = types.SimpleNamespace(
        discover_devices=lambda *args, **kwargs: [],
        open_device=lambda *args, **kwargs: native,
    )
    monkeypatch.setattr(hl, "_load_native", lambda: mock_module)

    import xvisio

    return xvisio.open(), native


def test_side_reset_does_not_disturb_other_side(monkeypatch):
    dev, native = _open_with_controller_mock(monkeypatch)

    assert dev.reset_controller_reference(side="both") is True

    native.set_controller_pose("left", (0.2, -0.1, 0.3))
    native.set_controller_pose("right", (0.8, 0.4, -0.2))
    left_before, right_before = dev.controller_relative()

    assert left_before is not None
    assert right_before is not None

    assert dev.reset_controller_reference(side="left") is True
    left_after, right_after = dev.controller_relative()

    assert left_after is not None
    assert right_after is not None
    assert np.allclose(left_after.position, (0.0, 0.0, 0.0), atol=1e-9)
    assert np.allclose(right_after.position, right_before.position, atol=1e-9)
    assert np.allclose(right_after.quat_wxyz, right_before.quat_wxyz, atol=1e-9)


def test_no_arg_reset_keeps_global_behavior(monkeypatch):
    dev, native = _open_with_controller_mock(monkeypatch)

    native.set_controller_pose("left", (1.0, 2.0, 3.0))
    native.set_controller_pose("right", (-1.0, -2.0, -3.0))

    assert dev.reset_controller_reference() is True
    left_rel, right_rel = dev.controller_relative()

    assert left_rel is not None
    assert right_rel is not None
    assert np.allclose(left_rel.position, (0.0, 0.0, 0.0), atol=1e-9)
    assert np.allclose(right_rel.position, (0.0, 0.0, 0.0), atol=1e-9)


def test_missing_requested_side_returns_false_without_resetting_other(monkeypatch):
    dev, native = _open_with_controller_mock(monkeypatch)

    assert dev.reset_controller_reference(side="right") is True
    native.set_controller_pose("right", (0.3, 0.0, 0.0))
    _, right_before = dev.controller_relative()
    assert right_before is not None

    native.set_controller_available("left", False)
    assert dev.reset_controller_reference(side="left") is False

    _, right_after = dev.controller_relative()
    assert right_after is not None
    assert np.allclose(right_after.position, right_before.position, atol=1e-9)
    assert np.allclose(right_after.quat_wxyz, right_before.quat_wxyz, atol=1e-9)


def test_capability_probe_reports_software_fallback(monkeypatch):
    dev, _ = _open_with_controller_mock(monkeypatch)
    assert dev.supports_individual_reference_reset() is False
