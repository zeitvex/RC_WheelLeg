"""Export the current PyTorch actor checkpoint to ONNX and verify parity."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy.policy_runner import load_policy  # noqa: E402


def export_onnx(pt_path: Path, onnx_path: Path, opset: int = 14) -> None:
    device = torch.device("cpu")
    model = load_policy(pt_path, device)
    if getattr(model, "backend", "torch") != "torch":
        raise ValueError(f"export source must be a .pt policy, got {pt_path}")

    obs_dim = int(model.expected_obs_dim)
    dummy = torch.randn(1, obs_dim, dtype=torch.float32, device=device)
    onnx_path.parent.mkdir(parents=True, exist_ok=True)

    torch.onnx.export(
        model,
        dummy,
        str(onnx_path),
        export_params=True,
        opset_version=opset,
        do_constant_folding=True,
        input_names=["obs"],
        output_names=["action"],
        dynamic_axes={"obs": {0: "batch_size"}, "action": {0: "batch_size"}},
    )

    try:
        import onnxruntime as ort
    except ImportError:
        print("[Export] onnxruntime not installed; export done but parity check skipped.")
        return

    opts = ort.SessionOptions()
    opts.intra_op_num_threads = 1
    opts.inter_op_num_threads = 1
    opts.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    session = ort.InferenceSession(str(onnx_path), sess_options=opts, providers=["CPUExecutionProvider"])
    with torch.no_grad():
        torch_out = model(dummy).detach().cpu().numpy()
    ort_out = session.run([session.get_outputs()[0].name], {session.get_inputs()[0].name: dummy.cpu().numpy()})[0]
    max_diff = float(np.max(np.abs(torch_out - ort_out)))
    mean_diff = float(np.mean(np.abs(torch_out - ort_out)))
    print(f"[Export] ONNX parity max_diff={max_diff:.8f}, mean_diff={mean_diff:.8f}")
    if max_diff > 1e-4:
        raise RuntimeError(f"ONNX parity check failed: max_diff={max_diff:.8f}")


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--pt", default=str(root / "policies" / "model_rough.pt"), help="Source .pt checkpoint")
    parser.add_argument("--onnx", default=None, help="Destination .onnx path; default replaces .pt suffix")
    parser.add_argument("--opset", type=int, default=14)
    args = parser.parse_args()

    pt_path = Path(args.pt)
    onnx_path = Path(args.onnx) if args.onnx else pt_path.with_suffix(".onnx")
    if not pt_path.exists():
        print(f"[Export] missing source policy: {pt_path}")
        return 1

    export_onnx(pt_path, onnx_path, args.opset)
    print(f"[Export] wrote {onnx_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
