"""Summarize real-run logs for startup/stand/runtime diagnosis."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics

LEG_JOINTS = (
    "fl_hip_abd", "fl_hip_pitch", "fl_knee",
    "fr_hip_abd", "fr_hip_pitch", "fr_knee",
    "rl_hip_abd", "rl_hip_pitch", "rl_knee",
    "rr_hip_abd", "rr_hip_pitch", "rr_knee",
)


def _f(row: dict[str, str], key: str, default: float = 0.0) -> float:
    try:
        return float(row.get(key, default))
    except Exception:
        return default


def _stats(values: list[float]) -> str:
    if not values:
        return "--"
    return (
        f"mean={statistics.mean(values):.3f} "
        f"std={statistics.pstdev(values):.3f} "
        f"min={min(values):.3f} max={max(values):.3f}"
    )


def _pitch_deg(row: dict[str, str]) -> float:
    gx = _f(row, "pgrav_x")
    gy = _f(row, "pgrav_y")
    gz = _f(row, "pgrav_z")
    return math.degrees(math.atan2(gx, math.sqrt(max(1e-9, gy * gy + gz * gz))))


def summarize(log_dir: Path) -> int:
    state_path = log_dir / "state.csv"
    events_path = log_dir / "events.jsonl"
    if not state_path.exists():
        print(f"[Analyze] missing {state_path}")
        return 1

    if events_path.exists():
        print("[Analyze] key events:")
        for line in events_path.read_text(encoding="utf-8", errors="replace").splitlines():
            try:
                ev = json.loads(line)
            except Exception:
                continue
            if ev.get("kind") in {
                "STARTUP_PLAN",
                "STARTUP_REACHED",
                "STAND_BALANCE_STABLE",
                "RUNTIME_BEGIN",
                "POLICY_TARGET_STALE",
                "POLICY_TIMEOUT",
                "SAFETY_BRAKE",
                "GUARD_STOP",
                "POSE_INIT_FAILED",
            }:
                detail = {k: v for k, v in ev.items() if k not in ("t", "t_rel")}
                print(f"  t={ev.get('t_rel', 0):.2f}s {detail}")

    with state_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    print(f"\n[Analyze] state rows: {len(rows)}")
    global_notes: list[str] = []
    for phase in sorted({r.get("phase", "") for r in rows}):
        phase_rows = [r for r in rows if r.get("phase") == phase]
        if not phase_rows:
            continue
        pitch = [_pitch_deg(r) for r in phase_rows]
        loop = [_f(r, "loop_dt_ms") for r in phase_rows]
        imu = [_f(r, "imu_age_ms") for r in phase_rows]
        print(f"\n[Phase] {phase} n={len(phase_rows)} t={phase_rows[0].get('t_rel')}..{phase_rows[-1].get('t_rel')}")
        print(f"  pitch_deg {_stats(pitch)}")
        print(f"  loop_ms   {_stats(loop)}")
        print(f"  imu_age   {_stats(imu)}")
        if "stand_pitch_corr" in phase_rows[0]:
            stand_pitch = [_f(r, "stand_pitch_deg") for r in phase_rows]
            stand_corr = [_f(r, "stand_pitch_corr") for r in phase_rows]
            stand_enabled = [_f(r, "stand_pitch_comp_enabled") for r in phase_rows]
            print(f"  stand_pitch_deg  {_stats(stand_pitch)}")
            print(f"  stand_pitch_corr {_stats(stand_corr)} enabled_mean={statistics.mean(stand_enabled):.3f}")
        for joint in ("fl_hip_pitch", "fr_hip_pitch", "rl_hip_pitch", "rr_hip_pitch", "fl_knee", "fr_knee", "rl_knee", "rr_knee"):
            pos_key = f"{joint}_pos"
            tgt_key = f"{joint}_tgt"
            tau_key = f"{joint}_tau"
            if pos_key in phase_rows[0] and tgt_key in phase_rows[0]:
                err = [_f(r, pos_key) - _f(r, tgt_key) for r in phase_rows]
                tau = [_f(r, tau_key) for r in phase_rows] if tau_key in phase_rows[0] else []
                print(f"  {joint}_err {_stats(err)} tau {_stats(tau)}")
        high_tau = []
        high_err = []
        for joint in LEG_JOINTS:
            tau_key = f"{joint}_tau"
            pos_key = f"{joint}_pos"
            tgt_key = f"{joint}_tgt"
            if tau_key in phase_rows[0]:
                tau_abs_mean = statistics.mean(abs(_f(r, tau_key)) for r in phase_rows)
                if tau_abs_mean > 6.0:
                    high_tau.append((joint, tau_abs_mean))
            if pos_key in phase_rows[0] and tgt_key in phase_rows[0]:
                err_abs_mean = statistics.mean(abs(_f(r, pos_key) - _f(r, tgt_key)) for r in phase_rows)
                if err_abs_mean > 0.08:
                    high_err.append((joint, err_abs_mean))
        if high_tau:
            text = ", ".join(f"{name}:{value:.2f}Nm" for name, value in sorted(high_tau, key=lambda x: -x[1])[:4])
            print(f"  high_tau_mean {text}")
            global_notes.append(f"{phase}: high mean torque -> {text}")
        if high_err:
            text = ", ".join(f"{name}:{value:.3f}rad" for name, value in sorted(high_err, key=lambda x: -x[1])[:4])
            print(f"  high_err_mean {text}")
            global_notes.append(f"{phase}: high tracking error -> {text}")
    if global_notes:
        print("\n[Analyze] notes:")
        for note in global_notes:
            print(f"  - {note}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log_dir", type=str)
    args = parser.parse_args()
    return summarize(Path(args.log_dir))


if __name__ == "__main__":
    raise SystemExit(main())
