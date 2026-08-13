"""Wrapper to adapt mjlab HimManagerBasedRlEnv for HIMLoco RSL-RL.

Handles:
- Observation history stacking (actor obs get history, critic gets single step)
- Termination privileged observation extraction
- 7-value step return: (obs, privileged_obs, rewards, dones, infos,
                        termination_ids, termination_privileged_obs)
"""

import torch

from him_mjlab.envs.him_manager_based_rl_env import HimManagerBasedRlEnv
from ..env.vec_env import VecEnv


class HimMjlabVecEnvWrapper(VecEnv):
    """Wraps mjlab HimManagerBasedRlEnv for HIMLoco RSL-RL.

    Key features:
    - Converts dict observations to flat tensors
    - Maintains history buffers for actor observations
    - Tracks terminated envs and their pre-reset privileged observations
    - Returns 7-value step required by HIMOnPolicyRunner
    """

    def __init__(
        self,
        env: HimManagerBasedRlEnv,
        history_length: int = 5,
        privileged_history_length: int = 0,
    ):
        if not isinstance(env.unwrapped, HimManagerBasedRlEnv):
            raise ValueError(
                "The environment must be HimManagerBasedRlEnv. "
                f"Got: {type(env)}"
            )

        self.env = env
        self.reorder_indices = [0, 4, 8, 12, 1, 5, 9, 13, 2, 6, 10, 14, 3, 7, 11, 15]

        self.num_envs = self.unwrapped.num_envs
        self.device = self.unwrapped.device
        self.max_episode_length = self.unwrapped.max_episode_length
        self.num_actions = self.unwrapped.action_manager.total_action_dim

        # Single-step observation dimensions from observation manager groups
        self.num_one_step_obs = self.unwrapped.observation_manager.group_obs_dim["policy"][0]
        self.history_length = history_length
        self.num_obs = self.num_one_step_obs * (self.history_length + 1)

        # History buffer for policy observations
        self.obs_history_buf = torch.zeros(self.num_envs, self.num_obs, device=self.device)

        # Termination tracking
        self._termination_ids = torch.tensor([], dtype=torch.long, device=self.device)

        # Privileged observation setup if critic group exists
        if "critic" in self.unwrapped.observation_manager.group_obs_dim:
            self.num_one_step_privileged_obs = (
                self.unwrapped.observation_manager.group_obs_dim["critic"][0]
            )
            self.privileged_history_length = privileged_history_length
            self.num_privileged_obs = self.num_one_step_privileged_obs * (
                self.privileged_history_length + 1
            )
            self.privileged_obs_history_buf = torch.zeros(
                self.num_envs, self.num_privileged_obs, device=self.device
            )
            self._termination_privileged_obs = torch.zeros(
                0, self.num_privileged_obs, device=self.device
            )
        else:
            self.num_one_step_privileged_obs = None
            self.num_privileged_obs = None
            self.privileged_obs_history_buf = None
            self._termination_privileged_obs = None

        # Reset at start since HIM runner does not call reset
        self.env.reset()

    @property
    def cfg(self):
        return self.env.cfg

    @property
    def render_mode(self):
        return self.env.render_mode

    @property
    def observation_space(self):
        return self.env.observation_space

    @property
    def action_space(self):
        return self.env.action_space

    @classmethod
    def class_name(cls) -> str:
        return cls.__name__

    @property
    def unwrapped(self):
        return self.env.unwrapped

    @property
    def episode_length_buf(self) -> torch.Tensor:
        return self.unwrapped.episode_length_buf

    @episode_length_buf.setter
    def episode_length_buf(self, value: torch.Tensor):
        self.unwrapped.episode_length_buf = value

    def seed(self, seed: int = -1) -> int:
        return self.env.seed(seed)

    def reset(self):
        obs_dict, _ = self.env.reset()
        policy_obs = obs_dict.get("policy", obs_dict[next(iter(obs_dict))])
        return policy_obs, {"observations": obs_dict}

    def get_observations(self) -> torch.Tensor:
        return self.obs_history_buf

    def get_privileged_observations(self) -> torch.Tensor | None:
        return self.privileged_obs_history_buf

    def compute_termination_observations(
        self, env_ids: torch.Tensor, obs_before_reset: torch.Tensor
    ) -> torch.Tensor | None:
        if len(env_ids) == 0:
            return torch.zeros(
                0, self.num_one_step_privileged_obs, device=self.device
            )
        return obs_before_reset[env_ids]

    def step(self, actions: torch.Tensor):
        """Execute one time-step and return 7 values for HIMLoco.

        Returns:
            obs: History-stacked policy observations
            privileged_obs: Privileged/critic observations
            rewards: Rewards
            dones: Done flags (terminated | truncated)
            infos: Additional info dict
            termination_ids: Indices of envs that terminated this step
            termination_privileged_obs: Pre-reset privileged obs for terminated envs
        """
        # Reorder actions from policy (Leg-by-Leg) to environment (Group)
        actions_reordered = actions[:, self.reorder_indices]

        # Step the environment (6-value return)
        obs_dict, obs_before_reset, rewards, terminated, truncated, infos = \
            self.env.step(actions_reordered)

        dones = (terminated | truncated).to(dtype=torch.long)

        if not self.unwrapped.cfg.is_finite_horizon:
            infos["time_outs"] = truncated

        # Extract policy observations from dict
        current_obs = obs_dict.get("policy")
        if current_obs is None:
            first_key = next(iter(obs_dict.keys()))
            current_obs = obs_dict[first_key]

        # Update policy observation history buffer
        if self.history_length > 0:
            self.obs_history_buf = torch.cat(
                (
                    current_obs[:, :self.num_one_step_obs],
                    self.obs_history_buf[:, :-self.num_one_step_obs],
                ),
                dim=-1,
            )
        else:
            self.obs_history_buf = current_obs

        # Track terminated environments
        self._termination_ids = torch.nonzero(dones, as_tuple=False).squeeze(-1)

        # Update privileged observation history buffer
        if "critic" in obs_dict and self.privileged_obs_history_buf is not None:
            current_privileged_obs = obs_dict["critic"]
            termination_observation = obs_before_reset.get("critic")
            if termination_observation is None:
                termination_observation = obs_before_reset.get("policy")
            self._termination_privileged_obs = self.compute_termination_observations(
                self._termination_ids, termination_observation
            )
            if self.privileged_history_length > 0:
                self.privileged_obs_history_buf = torch.cat(
                    (
                        current_privileged_obs[:, :self.num_one_step_privileged_obs],
                        self.privileged_obs_history_buf[
                            :, :-self.num_one_step_privileged_obs
                        ],
                    ),
                    dim=-1,
                )
            else:
                self.privileged_obs_history_buf = current_privileged_obs

        # NaN/Inf guard
        if torch.isnan(self.obs_history_buf).any() or torch.isinf(self.obs_history_buf).any():
            raise ValueError("NaN/Inf detected in obs_history_buf!")
        if torch.isnan(rewards).any() or torch.isinf(rewards).any():
            raise ValueError("NaN/Inf detected in rewards!")

        return (
            self.obs_history_buf,
            self.privileged_obs_history_buf,
            rewards,
            dones,
            infos,
            self._termination_ids,
            self._termination_privileged_obs,
        )

    def close(self):
        return self.env.close()

    def __getattr__(self, name):
        return getattr(self.env, name)