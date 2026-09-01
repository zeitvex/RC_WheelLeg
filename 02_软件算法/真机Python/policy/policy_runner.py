from pathlib import Path

import numpy as np
import torch
import torch.nn as nn


class PolicyMLP(nn.Module):
    def __init__(self, obs_dim: int, action_dim: int):
        super().__init__()
        self.register_buffer("obs_mean", torch.zeros(obs_dim))
        self.register_buffer("obs_std", torch.ones(obs_dim))
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 512),
            nn.ELU(),
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, action_dim),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = (x - self.obs_mean) / torch.clamp(self.obs_std, min=1e-6)
        return self.net(x)


def load_policy(model_path: Path, device: torch.device) -> PolicyMLP:
    checkpoint = torch.load(model_path, map_location=device, weights_only=False)
    state_dict = checkpoint["actor_state_dict"]

    input_key = "mlp.0.weight" if "mlp.0.weight" in state_dict else "net.0.weight"
    output_key = "mlp.6.weight" if "mlp.6.weight" in state_dict else "net.6.weight"
    obs_dim = int(state_dict[input_key].shape[1])
    action_dim = int(state_dict[output_key].shape[0])

    model = PolicyMLP(obs_dim=obs_dim, action_dim=action_dim)
    remapped_state_dict: dict[str, torch.Tensor] = {}
    for key, value in state_dict.items():
        if key.startswith("mlp."):
            remapped_state_dict[key.replace("mlp.", "net.")] = value
        elif key.startswith("net."):
            remapped_state_dict[key] = value
        elif key == "obs_normalizer._mean":
            remapped_state_dict["obs_mean"] = value.squeeze()
        elif key == "obs_normalizer._var":
            remapped_state_dict["obs_std"] = torch.sqrt(value.squeeze() + 1e-5)

    model.load_state_dict(remapped_state_dict, strict=False)
    model.eval()
    model.to(device)
    model.expected_obs_dim = obs_dim
    model.expected_action_dim = action_dim
    return model


class PolicyRunner:
    BASE_OBS_DIM = 53
    DEFAULT_STAND_POSE = np.array(
        [
            0.0, 0.9, -1.8,
            0.0, 0.9, -1.8,
            0.0, 0.9, -1.8,
            0.0, 0.9, -1.8,
            0.0, 0.0, 0.0, 0.0,
        ],
        dtype=np.float32,
    )

    def __init__(
        self,
        policy_path: Path,
        device: torch.device | None = None,
        enable_zero_cmd_suppression: bool = True,
        hold_zero_command_pose: bool = True,
        command_release_s: float = 0.35,
        action_scale: np.ndarray | None = None,
        zero_cmd_use_yaw_rate: bool = True,
    ):
        self.device = device or torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy_path = Path(policy_path)
        self.enable_zero_cmd_suppression = bool(enable_zero_cmd_suppression)
        self.hold_zero_command_pose = bool(hold_zero_command_pose)
        self.command_release_s = max(float(command_release_s), 1e-3)
        print(f"[PolicyRunner] device={self.device}, policy={self.policy_path}")
        self.policy = load_policy(self.policy_path, self.device)
        if self.policy.expected_obs_dim != self.BASE_OBS_DIM:
            raise ValueError(
                f"Unsupported policy obs dim {self.policy.expected_obs_dim}. "
                f"Current sim2real only supports {self.BASE_OBS_DIM}-D actor observations."
            )

        self.default_dof_pos = self.DEFAULT_STAND_POSE.copy()
        self.last_actions = np.zeros(16, dtype=np.float32)

        self.action_scale = np.asarray(
            action_scale
            if action_scale is not None
            else [
                0.125, 0.25, 0.25,
                0.125, 0.25, 0.25,
                0.125, 0.25, 0.25,
                0.125, 0.25, 0.25,
                5.0, 5.0, 5.0, 5.0,
            ],
            dtype=np.float32,
        )
        if self.action_scale.shape != (16,):
            raise ValueError(f"action_scale must be shape (16,), got {self.action_scale.shape}")

        self.zero_cmd_lin_thresh = 0.05
        self.zero_cmd_yaw_thresh = 0.05
        self.zero_yaw_rate_thresh = 0.10
        self.zero_cmd_use_yaw_rate = bool(zero_cmd_use_yaw_rate)
        self._command_release_alpha = 0.0
        print(
            f"[PolicyRunner] obs_dim={self.policy.expected_obs_dim}, "
            f"base_obs_dim={self.BASE_OBS_DIM}, history=1, "
            f"action_dim={self.policy.expected_action_dim}, "
            f"zero_cmd_suppression={self.enable_zero_cmd_suppression}, "
            f"hold_zero_command_pose={self.hold_zero_command_pose}"
        )

    def reset(self, prime_obs: np.ndarray | None = None) -> None:
        self.last_actions = np.zeros(16, dtype=np.float32)
        self._command_release_alpha = 0.0

    def _is_zero_command(self, command: np.ndarray, base_ang_vel: np.ndarray) -> bool:
        cmd_is_zero = (
            np.linalg.norm(command[:2]) < self.zero_cmd_lin_thresh
            and abs(command[2]) < self.zero_cmd_yaw_thresh
        )
        if not self.zero_cmd_use_yaw_rate:
            return cmd_is_zero
        return cmd_is_zero and abs(base_ang_vel[2]) < self.zero_yaw_rate_thresh

    def command_activation_metrics(self, command: np.ndarray) -> tuple[float, float]:
        command = np.asarray(command, dtype=np.float32)
        planar = float(np.linalg.norm(command[:2]))
        yaw = float(abs(command[2]))
        return planar, yaw

    def is_command_active(self, command: np.ndarray) -> bool:
        planar, yaw = self.command_activation_metrics(command)
        return planar >= self.zero_cmd_lin_thresh or yaw >= self.zero_cmd_yaw_thresh

    def step(self, obs: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        obs = np.asarray(obs, dtype=np.float32)
        expected_obs_dim = int(self.policy.expected_obs_dim)
        if obs.shape[0] != expected_obs_dim:
            raise ValueError(
                f"Observation dim mismatch: got {obs.shape[0]}, expected {expected_obs_dim}."
            )

        obs_tensor = torch.tensor(obs, dtype=torch.float32, device=self.device).unsqueeze(0)
        with torch.no_grad():
            raw_actions = self.policy(obs_tensor).squeeze(0).cpu().numpy()

        raw_actions = np.clip(raw_actions, -10.0, 10.0).astype(np.float32)
        command = obs[6:9]
        base_ang_vel = obs[0:3] / 0.25
        zero_command = self._is_zero_command(command, base_ang_vel)
        if zero_command:
            self._command_release_alpha = 0.0
            if self.hold_zero_command_pose:
                raw_actions[:] = 0.0
            elif self.enable_zero_cmd_suppression:
                raw_actions[12:16] = 0.0
                raw_actions[:12] *= 0.5
        else:
            self._command_release_alpha = min(1.0, self._command_release_alpha + 0.02 / self.command_release_s)
            raw_actions *= self._command_release_alpha

        self.last_actions = raw_actions.copy()
        scaled_actions = raw_actions * self.action_scale
        return scaled_actions, raw_actions
