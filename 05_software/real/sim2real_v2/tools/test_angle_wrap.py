"""Offline checks for RS02 multi-turn angle wrapping in startup/control."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from interface.motor_mapping import MotorMapping  # noqa: E402
from startup.pose_initializer import STAND_POSE, _periodic_leg_delta  # noqa: E402


def main() -> int:
    mapper = MotorMapping()

    # Real log example: startup saw rl_knee sim angle 4.648rad while the stand
    # target was -1.8rad. Those are close modulo 2*pi and must not plan a full turn.
    raw_sim = STAND_POSE.copy()
    raw_sim[8] = 4.648097991943359
    raw_real = mapper.sim_to_real(raw_sim)
    canonical = mapper.real_to_sim({(2, 3): raw_real[(2, 3)]})
    delta = _periodic_leg_delta(canonical, STAND_POSE)
    real_target = mapper.sim_to_real(STAND_POSE, current_real_pos={(2, 3): raw_real[(2, 3)]})[(2, 3)]
    real_move = real_target - raw_real[(2, 3)]

    print(f"[WrapCheck] canonical_sim_idx8={canonical[8]:.6f}")
    print(f"[WrapCheck] startup_delta_idx8={delta[8]:.6f}")
    print(f"[WrapCheck] real_move_idx8={real_move:.6f}")

    if abs(float(delta[8])) > 0.25:
        print("[WrapCheck] FAIL: periodic startup delta is too large")
        return 1
    if abs(float(real_move)) > 0.25:
        print("[WrapCheck] FAIL: real target would command a long-path move")
        return 1

    print("[WrapCheck] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
