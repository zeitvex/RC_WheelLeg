"""Check whether `sim2real/` is self-contained enough for direct deployment."""

from __future__ import annotations

import importlib
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


REQUIRED_FILES = [
    "config.yaml",
    "deployment_manifest.yaml",
    "main.py",
    "policies/model_rough.pt",
    "policy/policy_runner.py",
    "interface/real_io.py",
    "interface/imu_client.py",
    "interface/motor_driver.py",
    "vendored/drivers/motor_driver.py",
    "vendored/drivers/usb_can_adapter.py",
    "vendored/odin1_imu/odin1_imu.py",
    "vendored/odin1_imu/build/libodin1_imu_bridge.so",
    "mjcf/wheelleg.xml",
]

REQUIRED_IMPORTS = [
    "numpy",
    "yaml",
    "torch",
    "serial",
]

OPTIONAL_IMPORTS = [
    ("pynput", "only needed for CLI keyboard control"),
]


def check() -> int:
    root = Path(__file__).resolve().parents[1]
    issues: list[str] = []
    warnings: list[str] = []

    print(f"[Check] sim2real root: {root}")

    for rel in REQUIRED_FILES:
        path = root / rel
        if path.exists():
            print(f"[Check] file: PASS {rel}")
        else:
            issues.append(f"missing required file: {rel}")

    for module_name in REQUIRED_IMPORTS:
        try:
            importlib.import_module(module_name)
            print(f"[Check] import: PASS {module_name}")
        except Exception as exc:
            issues.append(f"missing python dependency `{module_name}`: {exc}")

    for module_name, note in OPTIONAL_IMPORTS:
        try:
            importlib.import_module(module_name)
            print(f"[Check] optional import: PASS {module_name}")
        except Exception:
            warnings.append(f"optional dependency `{module_name}` not found ({note})")

    index_html = (root / "web" / "static" / "index.html").read_text(encoding="utf-8")
    if "https://unpkg.com/three@" in index_html:
        warnings.append(
            "web 3D viewer depends on remote three.js CDN; CLI/web backend are standalone, "
            "but full offline 3D viewer is not bundled yet"
        )

    if issues:
        print("\n" + "=" * 60)
        print("Standalone deployment check: FAIL")
        for item in issues:
            print(f"- {item}")
    else:
        print("\n" + "=" * 60)
        print("Standalone deployment check: PASS")

    if warnings:
        print("\nWarnings:")
        for item in warnings:
            print(f"- {item}")

    return 1 if issues else 0


def main():
    raise SystemExit(check())


if __name__ == "__main__":
    main()
