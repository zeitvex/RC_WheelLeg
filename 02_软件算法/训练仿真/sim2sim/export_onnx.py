import argparse
import torch
import torch.nn as nn
import numpy as np
from pathlib import Path
import onnxruntime as ort


PROJECT_ROOT = Path(__file__).resolve().parents[1]

class PolicyMLP(nn.Module):
    def __init__(self, obs_dim=53, action_dim=16):
        super().__init__()
        self.register_buffer("obs_mean", torch.zeros(obs_dim))
        self.register_buffer("obs_std", torch.ones(obs_dim))
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 512), nn.ELU(),
            nn.Linear(512, 256), nn.ELU(),
            nn.Linear(256, 128), nn.ELU(),
            nn.Linear(128, action_dim),
        )

    def forward(self, x):
        x = (x - self.obs_mean) / torch.clamp(self.obs_std, min=1e-6)
        return self.net(x)

def load_policy(model_path, device):
    ckpt = torch.load(model_path, map_location=device, weights_only=False)
    state_dict = ckpt["actor_state_dict"]
    weight_key = "mlp.0.weight" if "mlp.0.weight" in state_dict else "net.0.weight"
    obs_dim = state_dict[weight_key].shape[1]

    output_key = "mlp.6.weight" if "mlp.6.weight" in state_dict else "net.6.weight"
    action_dim = state_dict[output_key].shape[0]

    model = PolicyMLP(obs_dim=obs_dim, action_dim=action_dim)
    my_sd = {}
    for k, v in state_dict.items():
        if k.startswith("mlp."):
            my_sd[k.replace("mlp.", "net.")] = v
        elif k.startswith("net."):
            my_sd[k] = v
        elif k == "obs_normalizer._mean":
            my_sd["obs_mean"] = v.squeeze()
        elif k == "obs_normalizer._var":
            my_sd["obs_std"] = torch.sqrt(v.squeeze() + 1e-5)

    model.load_state_dict(my_sd, strict=False)
    model.eval()
    model.to(device)
    return model, obs_dim

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--pt-path",
        "--pt_path",
        dest="pt_path",
        type=Path,
        default=PROJECT_ROOT / "model_rough.pt",
        help="PyTorch checkpoint to export (default: ../model_rough.pt).",
    )
    args = parser.parse_args()

    pt_path = args.pt_path.expanduser().resolve()
    if not pt_path.exists():
        print(f"File not found: {pt_path}")
        return

    device = torch.device("cpu")
    print(f"Loading {pt_path}...")
    model, obs_dim = load_policy(pt_path, device)

    onnx_path = pt_path.with_suffix(".onnx")

    dummy_input = torch.randn(1, obs_dim, device=device)

    print(f"Exporting to {onnx_path}...")
    torch.onnx.export(
        model,
        dummy_input,
        str(onnx_path),
        export_params=True,
        opset_version=14,
        do_constant_folding=True,
        input_names=["obs"],
        output_names=["action"],
        dynamic_axes={"obs": {0: "batch_size"}, "action": {0: "batch_size"}}
    )

    print("Verifying ONNX export...")
    try:
        session = ort.InferenceSession(str(onnx_path))
        with torch.no_grad():
            pt_out = model(dummy_input).numpy()
        onnx_out = session.run(["action"], {"obs": dummy_input.numpy()})[0]

        max_diff = np.max(np.abs(pt_out - onnx_out))
        mean_diff = np.mean(np.abs(pt_out - onnx_out))
        print(f"ONNX vs PyTorch - max_diff: {max_diff:.6f}, mean_diff: {mean_diff:.6f}")

        if max_diff < 1e-4:
            print("ONNX export verified OK.")
        else:
            print("WARNING: ONNX export has significant divergence from PyTorch model.")
    except ImportError:
        print("onnxruntime not installed. Skipping verification. Install with: pip install onnxruntime")

if __name__ == "__main__":
    main()
