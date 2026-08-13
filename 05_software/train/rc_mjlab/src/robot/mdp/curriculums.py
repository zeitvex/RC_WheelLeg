"""Adaptive command curriculum based on tracking reward performance."""

from __future__ import annotations

from typing import TYPE_CHECKING, cast
import torch

from mjlab.managers.curriculum_manager import CurriculumTermCfg
from mjlab.entity import Entity
from mjlab.managers.scene_entity_config import SceneEntityCfg

if TYPE_CHECKING:
    from mjlab.envs import ManagerBasedRlEnv


class adaptive_command_vel:
    """Expand velocity command ranges dynamically when tracking performance exceeds threshold.

    Uses the reward manager's per-step reward buffer directly to compute the mean raw
    tracking reward (in [0, 1] for exponential-based rewards).
    """

    def __init__(self, cfg: CurriculumTermCfg, env: ManagerBasedRlEnv):
        from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg

        p = cfg.params
        self._command_name: str = p["command_name"]
        self._reward_name: str = p["reward_name"]
        self._upper_threshold: float = p.get("upper_threshold", 0.8)
        self._lower_threshold: float = p.get("lower_threshold", 0.4)
        self._delta: float = p.get("delta", 0.2)
        self._max_lin_vel_x: tuple = tuple(p.get("max_lin_vel_x", (-1.0, 1.0)))
        self._max_lin_vel_y: tuple = tuple(p.get("max_lin_vel_y", (-0.5, 0.5)))
        self._max_ang_vel_z: tuple = tuple(p.get("max_ang_vel_z", (-1.0, 1.0)))
        self._min_range: float = 0.3

        command_term = env.command_manager.get_term(self._command_name)
        self._cfg = cast(UniformVelocityCommandCfg, command_term.cfg)

        # Locate the index of the target reward term
        self._reward_idx = list(env.reward_manager._term_names).index(self._reward_name)
        self._reward_weight = env.reward_manager.get_term_cfg(self._reward_name).weight

        # Exponential moving average (EMA) smoothing to prevent oscillations
        self._ema = 0.5
        self._running_mean = 0.0

    def __call__(
        self,
        env: ManagerBasedRlEnv,
        env_ids: torch.Tensor,
        **kwargs,
    ) -> dict[str, torch.Tensor]:
        # Retrieve raw step rewards from step_reward buffer
        # step_reward[:, idx] = raw * weight (already scaled by dt)
        step_rewards = env.reward_manager._step_reward[:, self._reward_idx]
        # raw = step_reward / weight -> [0, 1] for exponential rewards
        mean_raw = (torch.mean(step_rewards) / self._reward_weight).item()

        # Apply EMA smoothing to prevent jitter
        self._running_mean = self._ema * mean_raw + (1 - self._ema) * self._running_mean

        if self._running_mean > self._upper_threshold:
            self._expand()
        elif self._running_mean < self._lower_threshold:
            self._shrink()

        return self._log()

    def _expand(self):
        lo, hi = self._cfg.ranges.lin_vel_x
        self._cfg.ranges.lin_vel_x = (
            max(lo - self._delta, self._max_lin_vel_x[0]),
            min(hi + self._delta, self._max_lin_vel_x[1]),
        )
        lo, hi = self._cfg.ranges.lin_vel_y
        self._cfg.ranges.lin_vel_y = (
            max(lo - self._delta * 0.5, self._max_lin_vel_y[0]),
            min(hi + self._delta * 0.5, self._max_lin_vel_y[1]),
        )
        lo, hi = self._cfg.ranges.ang_vel_z
        self._cfg.ranges.ang_vel_z = (
            max(lo - self._delta, self._max_ang_vel_z[0]),
            min(hi + self._delta, self._max_ang_vel_z[1]),
        )

    def _shrink(self):
        lo, hi = self._cfg.ranges.lin_vel_x
        new_lo = min(lo + self._delta, -self._min_range)
        new_hi = max(hi - self._delta, self._min_range)
        if new_hi - new_lo >= 2 * self._min_range:
            self._cfg.ranges.lin_vel_x = (new_lo, new_hi)

    def _log(self) -> dict[str, torch.Tensor]:
        return {
            "lin_vel_x_min": torch.tensor(self._cfg.ranges.lin_vel_x[0]),
            "lin_vel_x_max": torch.tensor(self._cfg.ranges.lin_vel_x[1]),
            "ang_vel_z_max": torch.tensor(self._cfg.ranges.ang_vel_z[1]),
        }


def terrain_levels_vel_strict(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor,
    command_name: str,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> dict[str, torch.Tensor]:
    """Velocity-based terrain curriculum aligned with standard legged_gym logic.

    Upgrade:   robot travels beyond half the terrain tile width (> 4m).
    Downgrade: actual distance < 50% of commanded target distance.

    This matches DreamWaQ / HIMLoco / LocoLeggedWheel curriculum behaviour:
    - Promotion is easy (any traversal past 4m qualifies).
    - Demotion requires consistently failing to cover half the expected distance.
    """
    asset: Entity = env.scene[asset_cfg.name]

    terrain = env.scene.terrain
    assert terrain is not None
    terrain_generator = terrain.cfg.terrain_generator
    assert terrain_generator is not None

    command = env.command_manager.get_command(command_name)
    assert command is not None

    # Horizontal displacement from episode start
    distance = torch.norm(
        asset.data.root_link_pos_w[env_ids, :2] - env.scene.env_origins[env_ids, :2],
        dim=1,
    )

    cmd_speed = torch.norm(command[env_ids, :2], dim=1)

    # Upgrade: crossed half the tile width
    move_up = distance > terrain_generator.size[0] / 2

    # Downgrade: traveled less than 33% of commanded target distance.
    # Absolute threshold ≈ cmd_speed × 10m, identical to DreamWaQ / HIMLoco / LocoLeggedWheel
    # which use 50% × 20s episode = 10m. Adjusted for the longer 30s episode here.
    move_down = (distance < cmd_speed * env.max_episode_length_s * 0.33) & ~move_up



    terrain.update_env_origins(env_ids, move_up, move_down)

    levels = terrain.terrain_levels.float()
    result: dict[str, torch.Tensor] = {
        "mean": torch.mean(levels),
        "max": torch.max(levels),
    }

    sub_terrain_names = list(terrain_generator.sub_terrains.keys())
    terrain_origins = terrain.terrain_origins
    assert terrain_origins is not None
    num_cols = terrain_origins.shape[1]
    if num_cols == len(sub_terrain_names):
        types = terrain.terrain_types
        for i, name in enumerate(sub_terrain_names):
            mask = types == i
            if mask.any():
                result[name] = torch.mean(levels[mask])

    return result

