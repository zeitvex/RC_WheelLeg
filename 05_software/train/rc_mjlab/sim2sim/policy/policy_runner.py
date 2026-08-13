import torch
import torch.nn as nn
import numpy as np
from collections import deque
from pathlib import Path
from pynput import keyboard

# ============================================================
# Policy Model
# ============================================================
class PolicyMLP(nn.Module):
    def __init__(self, obs_dim=318, action_dim=16):
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
        x = (x - self.obs_mean) / self.obs_std
        return self.net(x)


def load_policy(model_path, device):
    ckpt = torch.load(model_path, map_location=device, weights_only=False)
    state_dict = ckpt["actor_state_dict"]
    
    # Check mlp.0.weight to determine obs_dim
    weight_key = "mlp.0.weight" if "mlp.0.weight" in state_dict else "net.0.weight"
    obs_dim = state_dict[weight_key].shape[1]
    
    model = PolicyMLP(obs_dim=obs_dim, action_dim=16)
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
    return model


# ============================================================
# Policy Runner with Smooth Dual-Policy Blending Transition
# ============================================================
class PolicyRunner:
    """Handles multi-policy execution and smooth linear blending transitions."""
    def __init__(self, policy_path, device, history_length=6, obs_dim=53):
        self.device = device
        self.history_length = history_length
        self.obs_dim = obs_dim

        # Automatically resolve rough and crawl policy paths
        if isinstance(policy_path, dict):
            self.policy_paths = policy_path
        else:
            p_path = Path(policy_path)
            policy_dir = p_path.parent
            crawl_path = policy_dir / "logs" / "crawl.pt"
            if not crawl_path.exists():
                crawl_path = policy_dir / "crawl.pt"
            self.policy_paths = {
                "rough": p_path,
                "crawl": crawl_path if crawl_path.exists() else p_path
            }

        # Load both policy networks
        self.policies = {}
        for name, path in self.policy_paths.items():
            print(f"[PolicyRunner] Loading {name} policy from: {path}")
            if Path(path).exists():
                self.policies[name] = load_policy(path, device)
            else:
                print(f"[PolicyRunner] WARNING: {name} policy file not found! Falling back to rough.")
                self.policies[name] = load_policy(self.policy_paths["rough"], device)

        # Default DOF positions for each policy
        self.default_dof_poses = {
            "rough": np.array([0.0, 0.9, -1.8] * 4 + [0.0] * 4, dtype=np.float32),
            "crawl": np.array([
                0.4, 1.65, -2.55,   # FL (Left)
                -0.4, 1.65, -2.55,  # FR (Right)
                0.4, 1.65, -2.55,   # RL (Left)
                -0.4, 1.65, -2.55,  # RR (Right)
                0.0, 0.0, 0.0, 0.0  # Wheels
            ], dtype=np.float32)
        }

        # Separate observation history buffers for each policy
        self.obs_histories = {
            name: deque(maxlen=self.history_length) for name in self.policies.keys()
        }

        # Separate previous actions for each policy to maintain correct history stacking
        self.last_actions_dict = {
            name: np.zeros(16, dtype=np.float32) for name in self.policies.keys()
        }

        # Blending and state transition variables
        self.current_policy_name = "rough"
        self.transition_in_progress = False
        self.transition_old_name = None
        self.transition_step = 0
        self.transition_steps = 50  # 1.0s at 50Hz (N=50 steps)

        # Action scale settings (HIP=0.125, KNEE/THIGH=0.25, WHEEL=5.0)
        self.HIP_SCALE = 0.125
        self.LEG_POS_SCALE = 0.25
        self.WHEEL_VEL_SCALE = 5.0
        self.action_scale = np.array([
            self.HIP_SCALE, self.LEG_POS_SCALE, self.LEG_POS_SCALE,  # FL
            self.HIP_SCALE, self.LEG_POS_SCALE, self.LEG_POS_SCALE,  # FR
            self.HIP_SCALE, self.LEG_POS_SCALE, self.LEG_POS_SCALE,  # RL
            self.HIP_SCALE, self.LEG_POS_SCALE, self.LEG_POS_SCALE,  # RR
            self.WHEEL_VEL_SCALE, self.WHEEL_VEL_SCALE, self.WHEEL_VEL_SCALE, self.WHEEL_VEL_SCALE
        ], dtype=np.float32)

        # Background keyboard listener for seamless switcher keys ('1' and '2')
        self.listener = keyboard.Listener(on_press=self._on_press)
        self.listener.start()
        print("[PolicyRunner] Background Keyboard Switcher active: Press '1' for ROUGH, '2' for CRAWL")

    def _on_press(self, key):
        try:
            if hasattr(key, 'char') and key.char is not None:
                if key.char == '1':
                    self.trigger_transition("rough")
                elif key.char == '2':
                    self.trigger_transition("crawl")
        except Exception as e:
            print(f"[PolicyRunner] Error in key listener: {e}")

    def trigger_transition(self, target_name):
        if target_name not in self.policies:
            return
        if target_name == self.current_policy_name and not self.transition_in_progress:
            return
        
        # If already transitioning, override target or complete the current one
        self.transition_old_name = self.current_policy_name
        self.current_policy_name = target_name
        self.transition_in_progress = True
        self.transition_step = 0
        print(f"\n[PolicyRunner] Smooth Transition: {self.transition_old_name.upper()} -> {self.current_policy_name.upper()} over 1.0s...")

    @property
    def default_dof_pos(self) -> np.ndarray:
        """Dynamic property returning the blended default pose during transition."""
        if self.transition_in_progress:
            alpha = self.transition_step / self.transition_steps
            return (1.0 - alpha) * self.default_dof_poses[self.transition_old_name] + alpha * self.default_dof_poses[self.current_policy_name]
        return self.default_dof_poses[self.current_policy_name]

    @property
    def last_actions(self) -> np.ndarray:
        """Dynamic property returning the blended previous raw actions for logging/main.py."""
        if self.transition_in_progress:
            alpha = self.transition_step / self.transition_steps
            return (1.0 - alpha) * self.last_actions_dict[self.transition_old_name] + alpha * self.last_actions_dict[self.current_policy_name]
        return self.last_actions_dict[self.current_policy_name]

    def reset(self):
        for name in self.obs_histories.keys():
            self.obs_histories[name].clear()
            self.last_actions_dict[name] = np.zeros(16, dtype=np.float32)
        self.transition_in_progress = False
        self.transition_old_name = None
        self.transition_step = 0

    def step(self, current_obs_53d):
        """
        Receives raw 53D observation (computed in main.py using blended default_dof_pos),
        re-aligns observation for each policy dynamically, runs inference, and blends output actions.
        
        This implementation uses a local state snapshot to prevent multi-threaded race conditions
        with the keyboard background listener thread.
        """
        # 1. Take atomic snapshot of state variables
        in_progress = self.transition_in_progress
        current_name = self.current_policy_name
        old_name = self.transition_old_name
        step_idx = self.transition_step
        total_steps = self.transition_steps

        # 2. Compute blended default dof pos using snapshot variables
        if in_progress:
            alpha = step_idx / total_steps
            default_dof_pos_blended = (1.0 - alpha) * self.default_dof_poses[old_name] + alpha * self.default_dof_poses[current_name]
        else:
            default_dof_pos_blended = self.default_dof_poses[current_name]

        # Update histories for BOTH policies with their respective mathematically aligned observations
        for name in self.policies.keys():
            # Align relative joint positions: obs[9:21] is joint_pos_rel
            obs_policy = current_obs_53d.copy()
            
            # Math: q_rel_policy = q_rel_blended + q_default_blended - q_default_policy
            # Since current_obs_53d was computed as (q - q_default_blended), this recovers (q - q_default_policy)
            obs_policy[9:21] = obs_policy[9:21] + default_dof_pos_blended[:12] - self.default_dof_poses[name][:12]
            
            # Inject policy-specific previous actions
            obs_policy[37:53] = self.last_actions_dict[name]

            # Populate queue
            if len(self.obs_histories[name]) == 0:
                for _ in range(self.history_length):
                    self.obs_histories[name].append(obs_policy.copy())
            else:
                self.obs_histories[name].append(obs_policy.copy())

        # Decide which policies to run using snapshot variables
        active_policies = [current_name]
        if in_progress:
            active_policies.append(old_name)

        raw_actions_out = {}
        for name in active_policies:
            # Flatten observation history
            obs_history_array = np.array(self.obs_histories[name])
            term_dims = [3, 3, 3, 12, 12, 4, 16]
            term_histories = np.split(obs_history_array, np.cumsum(term_dims)[:-1], axis=1)
            flat_obs = np.concatenate([h.flatten() for h in term_histories])
            
            obs_tensor = torch.tensor(flat_obs, device=self.device, dtype=torch.float32).unsqueeze(0)
            
            with torch.no_grad():
                raw_actions = self.policies[name](obs_tensor).squeeze(0).cpu().numpy()
            
            raw_actions = np.clip(raw_actions, -100.0, 100.0)
            self.last_actions_dict[name] = raw_actions.copy()
            raw_actions_out[name] = raw_actions

        # Blend actions if transitioning using snapshot variables
        if in_progress:
            alpha = step_idx / total_steps
            raw_actions_blended = (1.0 - alpha) * raw_actions_out[old_name] + alpha * raw_actions_out[current_name]
            
            # Step transition state machine on the class state safely
            self.transition_step += 1
            if self.transition_step >= self.transition_steps:
                self.transition_in_progress = False
                print(f"[PolicyRunner] Switch to {self.current_policy_name.upper()} complete!")
        else:
            raw_actions_blended = raw_actions_out[current_name]

        # Scale blended actions to physical outputs
        scaled_actions = raw_actions_blended * self.action_scale

        return scaled_actions, raw_actions_blended
