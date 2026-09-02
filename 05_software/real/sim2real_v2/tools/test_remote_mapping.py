from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from input_dev.remote_uart import (  # noqa: E402
    RemoteCommandMapper,
    RemoteControlState,
    RemoteSwitchState,
    SWITCH_HIGH,
    SWITCH_MID,
)


def assert_close(actual: float, expected: float, tol: float = 1e-6) -> None:
    if abs(actual - expected) > tol:
        raise AssertionError(f"expected {expected}, got {actual}")


def test_deadzone() -> None:
    mapper = RemoteCommandMapper(max_vx=0.8, max_vy=0.3, max_yaw=0.5, active_threshold=50)
    state = RemoteControlState(ch1=40, ch2=-49, ch4=50, switches=RemoteSwitchState(ch7=SWITCH_MID), frame_ok=True)
    cmd = mapper.map_command(state)
    assert_close(float(cmd[0]), 0.0)
    assert_close(float(cmd[1]), 0.0)
    assert_close(float(cmd[2]), 0.0)
    if mapper.is_command_active(state):
        raise AssertionError("deadzone values should not be active")


def test_mapping() -> None:
    mapper = RemoteCommandMapper(max_vx=0.8, max_vy=0.3, max_yaw=0.5, active_threshold=50)
    state = RemoteControlState(ch1=330, ch2=-660, ch4=165, switches=RemoteSwitchState(ch7=SWITCH_MID), frame_ok=True)
    cmd = mapper.map_command(state)
    assert_close(float(cmd[0]), -0.8)
    assert_close(float(cmd[1]), 0.075)
    assert_close(float(cmd[2]), 0.25)
    if not mapper.is_command_active(state):
        raise AssertionError("mapped command should be active")


def test_soft_estop_flag() -> None:
    state = RemoteControlState(ch1=0, ch2=0, ch4=0, switches=RemoteSwitchState(ch7=SWITCH_HIGH), frame_ok=True)
    if not state.estop_requested:
        raise AssertionError("switch high should request estop")


if __name__ == "__main__":
    test_deadzone()
    test_mapping()
    test_soft_estop_flag()
    print("remote command mapping tests passed")
