"""Continuous low-frequency external wrench disturbance for mjlab."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import torch

from mjlab.entity import Entity
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.utils.lab_api.math import sample_uniform

if TYPE_CHECKING:
    from mjlab.envs import ManagerBasedRlEnv


class apply_continuous_disturbance:
    """Apply smooth, continuous low-frequency force and torque disturbances to bodies.

    Achieved by sampling random target forces and torques periodically, and interpolating
    towards them via a 1st-order low-pass filter (exponential smoothing).
    """

    def __init__(self, cfg, env: ManagerBasedRlEnv):
        self._asset: Entity = env.scene[cfg.params["asset_cfg"].name]
        self._body_ids = cfg.params["asset_cfg"].body_ids
        self._num_envs = env.num_envs
        self._device = env.device
        self._step_dt = env.step_dt

        self._num_bodies = (
            len(self._body_ids)
            if isinstance(self._body_ids, list)
            else self._asset.num_bodies
        )

        # Disturbance states: shape (num_envs, num_bodies, 3)
        self._current_force = torch.zeros(self._num_envs, self._num_bodies, 3, device=self._device)
        self._current_torque = torch.zeros(self._num_envs, self._num_bodies, 3, device=self._device)
        self._target_force = torch.zeros(self._num_envs, self._num_bodies, 3, device=self._device)
        self._target_torque = torch.zeros(self._num_envs, self._num_bodies, 3, device=self._device)

        # Periodic resampling timers: shape (num_envs,)
        self._timer = torch.zeros(self._num_envs, device=self._device)

    def __call__(
        self,
        env: ManagerBasedRlEnv,
        env_ids: torch.Tensor | None,
        force_range: tuple[float, float],
        torque_range: tuple[float, float],
        resample_time_range: tuple[float, float],
        time_constant: float,
        asset_cfg: SceneEntityCfg,
    ) -> None:
        """Tick disturbance state: interpolate towards targets, periodically resample targets.

        Use with mode="step".
        """
        del env, env_ids, asset_cfg  # Unused, step events act on all envs.
        dt = self._step_dt

        # Decrement timers
        self._timer -= dt

        # Identify envs that need resampling
        resample = self._timer <= 0
        if resample.any():
            resample_ids = resample.nonzero(as_tuple=False).squeeze(-1)
            n = len(resample_ids)

            # Sample new targets
            size = (n, self._num_bodies, 3)
            self._target_force[resample_ids] = sample_uniform(*force_range, size, self._device)
            self._target_torque[resample_ids] = sample_uniform(*torque_range, size, self._device)

            # Sample new timer durations
            t_low, t_high = resample_time_range
            self._timer[resample_ids] = (
                torch.rand(n, device=self._device) * (t_high - t_low) + t_low
            )

        # Exponential smoothing step: x_new = (1 - alpha) * x + alpha * x_target
        alpha = 1.0 - math.exp(-dt / time_constant)

        self._current_force = (1.0 - alpha) * self._current_force + alpha * self._target_force
        self._current_torque = (1.0 - alpha) * self._current_torque + alpha * self._target_torque

        # Apply smooth wrenches to simulation
        all_env_ids = torch.arange(self._num_envs, device=self._device)
        self._asset.write_external_wrench_to_sim(
            self._current_force, self._current_torque, env_ids=all_env_ids, body_ids=self._body_ids
        )

    def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
        if env_ids is None:
            env_ids = slice(None)

        # Reset states
        self._current_force[env_ids] = 0.0
        self._current_torque[env_ids] = 0.0
        self._target_force[env_ids] = 0.0
        self._target_torque[env_ids] = 0.0
        self._timer[env_ids] = 0.0

        if isinstance(env_ids, slice):
            reset_ids = torch.arange(self._num_envs, device=self._device)[env_ids]
        else:
            reset_ids = env_ids

        if len(reset_ids) > 0:
            zeros = torch.zeros((len(reset_ids), self._num_bodies, 3), device=self._device)
            self._asset.write_external_wrench_to_sim(
                zeros, zeros, env_ids=reset_ids, body_ids=self._body_ids
            )
