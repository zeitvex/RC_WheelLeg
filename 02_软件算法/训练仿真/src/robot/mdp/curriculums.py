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


class command_axis_levels_vel:
    """Linearly expand one command axis over training steps."""

    def __init__(self, cfg: CurriculumTermCfg, env: ManagerBasedRlEnv):
        from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg

        p = cfg.params
        self._command_name: str = p.get("command_name", "twist")
        self._reward_name: str = p["reward_term_name"]
        self._axis: str = p["axis"]
        self._range_multiplier: tuple[float, float] = tuple(p.get("range_multiplier", (0.4, 1.0)))
        self._initial_range_param: tuple[float, float] | None = (
            tuple(p["initial_range"]) if "initial_range" in p else None
        )
        self._ema_alpha: float = p.get("ema_alpha", 0.5)
        self._warmup_steps: int = p.get("warmup_steps", 0)
        self._ramp_steps: int = p.get("ramp_steps", 800 * 24)

        command_term = env.command_manager.get_term(self._command_name)
        self._cfg = cast(UniformVelocityCommandCfg, command_term.cfg)
        self._reward_idx = list(env.reward_manager._term_names).index(self._reward_name)
        self._reward_weight = env.reward_manager.get_term_cfg(self._reward_name).weight
        self._running_mean = 0.0

        self._full_range = self._get_range()
        self._min_range = self._initial_range_param or self._scaled_range(self._range_multiplier[0])
        self._set_range(self._min_range)

    def __call__(self, env: ManagerBasedRlEnv, env_ids: torch.Tensor, **kwargs) -> dict[str, torch.Tensor]:
        if len(env_ids) > 0:
            episode_sums = env.reward_manager._episode_sums[self._reward_name][env_ids]
            mean_raw = torch.mean(episode_sums / env.cfg.episode_length_s / self._reward_weight).item()
            self._running_mean = self._ema_alpha * mean_raw + (1.0 - self._ema_alpha) * self._running_mean
        self._set_range(self._current_max_range(env.common_step_counter))

        lo, hi = self._get_range()
        return {
            f"{self._axis}_range_min": torch.tensor(lo),
            f"{self._axis}_range_max": torch.tensor(hi),
            f"{self._axis}_tracking_ema": torch.tensor(self._running_mean),
            f"{self._axis}_warmup_active": torch.tensor(float(env.common_step_counter < self._warmup_steps)),
            f"{self._axis}_range_cap": torch.tensor(self._current_multiplier(env.common_step_counter)),
        }

    def _get_range(self) -> tuple[float, float]:
        if self._axis == "x":
            return tuple(self._cfg.ranges.lin_vel_x)
        if self._axis == "y":
            return tuple(self._cfg.ranges.lin_vel_y)
        if self._axis == "yaw":
            return tuple(self._cfg.ranges.ang_vel_z)
        raise ValueError(f"Unknown command curriculum axis: {self._axis}")

    def _set_range(self, value: tuple[float, float]) -> None:
        if self._axis == "x":
            self._cfg.ranges.lin_vel_x = value
        elif self._axis == "y":
            self._cfg.ranges.lin_vel_y = value
        elif self._axis == "yaw":
            self._cfg.ranges.ang_vel_z = value
        else:
            raise ValueError(f"Unknown command curriculum axis: {self._axis}")

    def _scaled_range(self, multiplier: float) -> tuple[float, float]:
        lo, hi = self._full_range
        return (lo * multiplier, hi * multiplier)

    def _current_multiplier(self, step: int) -> float:
        start, end = self._range_multiplier
        if self._ramp_steps <= 0:
            return end
        progress = max(0.0, min(1.0, (step - self._warmup_steps) / self._ramp_steps))
        return start + (end - start) * progress

    def _current_max_range(self, step: int) -> tuple[float, float]:
        if self._initial_range_param is None:
            return self._scaled_range(self._current_multiplier(step))
        progress = self._current_progress(step)
        lo = self._min_range[0] + (self._full_range[0] - self._min_range[0]) * progress
        hi = self._min_range[1] + (self._full_range[1] - self._min_range[1]) * progress
        return (lo, hi)

    def _current_progress(self, step: int) -> float:
        if self._ramp_steps <= 0:
            return 1.0
        return max(0.0, min(1.0, (step - self._warmup_steps) / self._ramp_steps))


class command_levels_adaptive:
    """Adaptive command range curriculum based on average tracking performance."""

    def __init__(self, cfg: CurriculumTermCfg, env: ManagerBasedRlEnv):
        p = cfg.params
        self._command_name: str = p.get("command_name", "twist")
        self._reward_name: str = p["reward_term_name"]
        self._axis: str = p["axis"]
        self._delta_command: float = p.get("delta_command", 0.05)
        self._target_ratio: float = p.get("target_ratio", 0.8)
        self._ema_alpha: float = p.get("ema_alpha", 0.5)

        command_term = env.command_manager.get_term(self._command_name)
        self._cfg = command_term.cfg
        self._reward_weight = env.reward_manager.get_term_cfg(self._reward_name).weight
        self._running_mean = 0.0

        # Read the full range configured in the environment configuration
        self._full_range = self._get_range()

        # Set the command range to initial range at the start
        self._initial_range = list(p["initial_range"])  # e.g. [-0.5, 0.5]
        self._current_range = list(self._initial_range)
        self._set_range(self._current_range)

    def __call__(self, env: ManagerBasedRlEnv, env_ids: torch.Tensor, **kwargs) -> dict[str, torch.Tensor]:
        if len(env_ids) > 0:
            episode_sums = env.reward_manager._episode_sums[self._reward_name][env_ids]
            mean_raw = torch.mean(episode_sums / env.cfg.episode_length_s / self._reward_weight).item()
            self._running_mean = self._ema_alpha * mean_raw + (1.0 - self._ema_alpha) * self._running_mean

        # Check performance at the end of every episode (or every max_episode_length_s)
        episode_length_steps = int(env.cfg.episode_length_s / env.step_dt)
        if env.common_step_counter > 0 and env.common_step_counter % episode_length_steps == 0:
            # If performance exceeds target ratio (e.g., 0.8), widen the range
            if self._running_mean > self._target_ratio:
                # Widen the range
                lo, hi = self._current_range
                new_lo = max(self._full_range[0], lo - self._delta_command)
                new_hi = min(self._full_range[1], hi + self._delta_command)
                self._current_range = [new_lo, new_hi]
                self._set_range(self._current_range)

        lo, hi = self._current_range
        return {
            f"{self._axis}_range_min": torch.tensor(lo),
            f"{self._axis}_range_max": torch.tensor(hi),
            f"{self._axis}_tracking_ema": torch.tensor(self._running_mean),
            f"{self._axis}_target_ratio": torch.tensor(self._target_ratio),
        }

    def _get_range(self) -> tuple[float, float]:
        if self._axis == "x":
            return tuple(self._cfg.ranges.lin_vel_x)
        if self._axis == "y":
            return tuple(self._cfg.ranges.lin_vel_y)
        if self._axis == "yaw":
            return tuple(self._cfg.ranges.ang_vel_z)
        raise ValueError(f"Unknown command curriculum axis: {self._axis}")

    def _set_range(self, value: list[float]) -> None:
        if self._axis == "x":
            self._cfg.ranges.lin_vel_x = tuple(value)
        elif self._axis == "y":
            self._cfg.ranges.lin_vel_y = tuple(value)
        elif self._axis == "yaw":
            self._cfg.ranges.ang_vel_z = tuple(value)
        else:
            raise ValueError(f"Unknown command curriculum axis: {self._axis}")


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


def terrain_levels_ramp_strict(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor,
    command_name: str,
    ramp_steps: int = 50 * 24,
    move_up_expected_distance_ratio: float = 0.60,
    move_down_distance_ratio: float = 0.50,
    min_command_speed: float = 0.15,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> dict[str, torch.Tensor]:
    """Terrain curriculum active from start, with strict promotion and time-ramped max level."""
    asset: Entity = env.scene[asset_cfg.name]
    terrain = env.scene.terrain
    assert terrain is not None
    terrain_generator = terrain.cfg.terrain_generator
    assert terrain_generator is not None
    assert terrain.terrain_origins is not None
    assert terrain.env_origins is not None

    command = env.command_manager.get_command(command_name)
    assert command is not None

    distance = torch.norm(
        asset.data.root_link_pos_w[env_ids, :2] - env.scene.env_origins[env_ids, :2],
        dim=1,
    )
    cmd_speed = torch.norm(command[env_ids, :2], dim=1)
    active_command = cmd_speed >= min_command_speed

    expected_distance = cmd_speed * env.max_episode_length_s
    move_up = (distance > expected_distance * move_up_expected_distance_ratio) & active_command
    move_down = (
        (distance < expected_distance * move_down_distance_ratio)
        & active_command
        & ~move_up
    )

    terrain.terrain_levels[env_ids] += 1 * move_up - 1 * move_down

    max_level = max(int(terrain.max_terrain_level) - 1, 0)
    if ramp_steps <= 0:
        level_cap = max_level
    else:
        progress = max(0.0, min(1.0, env.common_step_counter / ramp_steps))
        level_cap = int(round(progress * max_level))
    terrain.terrain_levels[env_ids] = torch.clamp(
        terrain.terrain_levels[env_ids],
        min=0,
        max=min(level_cap, max_level),
    )

    terrain.env_origins[env_ids] = terrain.terrain_origins[
        terrain.terrain_levels[env_ids], terrain.terrain_types[env_ids]
    ]

    levels = terrain.terrain_levels.float()
    result: dict[str, torch.Tensor] = {
        "mean": torch.mean(levels),
        "max": torch.max(levels),
        "level_cap": torch.tensor(float(level_cap), device=env.device),
    }

    sub_terrain_names = list(terrain_generator.sub_terrains.keys())
    num_cols = terrain.terrain_origins.shape[1]
    if num_cols == len(sub_terrain_names):
        types = terrain.terrain_types
        for i, name in enumerate(sub_terrain_names):
            mask = types == i
            if mask.any():
                result[name] = torch.mean(levels[mask])

    return result


def terrain_levels_flat_warmup(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor,
    command_name: str,
    warmup_steps: int = 4800,
    flat_terrain_name: str = "flat",
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> dict[str, torch.Tensor]:
    """Keep all reset envs on flat level 0 before enabling terrain curriculum."""
    terrain = env.scene.terrain
    assert terrain is not None
    terrain_generator = terrain.cfg.terrain_generator
    assert terrain_generator is not None
    assert terrain.terrain_origins is not None
    assert terrain.env_origins is not None

    sub_terrain_names = list(terrain_generator.sub_terrains.keys())
    flat_type = sub_terrain_names.index(flat_terrain_name) if flat_terrain_name in sub_terrain_names else 0

    if env.common_step_counter < warmup_steps:
        terrain.terrain_levels[env_ids] = 0
        terrain.terrain_types[env_ids] = flat_type
        terrain.env_origins[env_ids] = terrain.terrain_origins[0, flat_type]
        if not hasattr(terrain, "_flat_warmup_env_released"):
            terrain._flat_warmup_env_released = torch.zeros(
                env.num_envs, dtype=torch.bool, device=env.device
            )
        terrain._flat_warmup_env_released[env_ids] = False

        levels = terrain.terrain_levels.float()
        result: dict[str, torch.Tensor] = {
            "mean": torch.mean(levels),
            "max": torch.max(levels),
            "warmup_active": torch.ones((), device=env.device),
        }
        for i, name in enumerate(sub_terrain_names):
            mask = terrain.terrain_types == i
            if mask.any():
                result[name] = torch.mean(levels[mask])
        return result

    if not hasattr(terrain, "_flat_warmup_released"):
        terrain._flat_warmup_released = True
        terrain._flat_warmup_release_counts = 0
    if not hasattr(terrain, "_flat_warmup_env_released"):
        terrain._flat_warmup_env_released = torch.zeros(
            env.num_envs, dtype=torch.bool, device=env.device
        )

    newly_released = env_ids[~terrain._flat_warmup_env_released[env_ids]]
    if len(newly_released) > 0:
        proportions = torch.tensor(
            [sub.proportion for sub in terrain_generator.sub_terrains.values()],
            device=env.device,
            dtype=torch.float,
        )
        proportions = proportions / torch.clamp(proportions.sum(), min=1.0e-6)
        terrain.terrain_types[newly_released] = torch.multinomial(
            proportions, len(newly_released), replacement=True
        )
        terrain.terrain_levels[newly_released] = 0
        terrain.env_origins[newly_released] = terrain.terrain_origins[
            terrain.terrain_levels[newly_released], terrain.terrain_types[newly_released]
        ]
        terrain._flat_warmup_env_released[newly_released] = True
        terrain._flat_warmup_release_counts += len(newly_released)

    result = terrain_levels_vel_strict(env, env_ids, command_name, asset_cfg=asset_cfg)
    result["warmup_active"] = torch.zeros((), device=env.device)
    result["released_envs"] = torch.tensor(float(getattr(terrain, "_flat_warmup_release_counts", 0)), device=env.device)
    return result


def terrain_levels_obstacle_release(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor,
    command_name: str,
    release_schedule: tuple[tuple[int, tuple[str, ...]], ...],
    initial_terrain_names: tuple[str, ...] = ("flat", "random_rough", "perlin_noise", "sloped_terrain"),
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> dict[str, torch.Tensor]:
    """Release obstacle terrain types gradually while keeping standard level progression."""
    terrain = env.scene.terrain
    assert terrain is not None
    terrain_generator = terrain.cfg.terrain_generator
    assert terrain_generator is not None
    assert terrain.terrain_origins is not None
    assert terrain.env_origins is not None

    sub_terrain_names = list(terrain_generator.sub_terrains.keys())
    allowed_names = list(initial_terrain_names)
    for step, names in release_schedule:
        if env.common_step_counter >= step:
            allowed_names.extend(names)
    allowed_type_ids = [
        sub_terrain_names.index(name) for name in allowed_names if name in sub_terrain_names
    ]
    if not allowed_type_ids:
        allowed_type_ids = [0]

    if not hasattr(terrain, "_obstacle_release_env_allowed"):
        terrain._obstacle_release_env_allowed = torch.zeros(
            env.num_envs, dtype=torch.bool, device=env.device
        )
        terrain._last_allowed_count = len(allowed_type_ids)

    # If new terrains were released, force all envs to eventually resample upon their next reset
    if len(allowed_type_ids) > terrain._last_allowed_count:
        terrain._obstacle_release_env_allowed.fill_(False)
        terrain._last_allowed_count = len(allowed_type_ids)

    allowed_tensor = torch.tensor(allowed_type_ids, dtype=torch.long, device=env.device)
    current_allowed = torch.isin(terrain.terrain_types[env_ids], allowed_tensor)
    need_resample = env_ids[~terrain._obstacle_release_env_allowed[env_ids] | ~current_allowed]

    if len(need_resample) > 0:
        proportions = torch.tensor(
            [terrain_generator.sub_terrains[sub_terrain_names[i]].proportion for i in allowed_type_ids],
            device=env.device,
            dtype=torch.float,
        )
        proportions = proportions / torch.clamp(proportions.sum(), min=1.0e-6)
        sampled = allowed_tensor[torch.multinomial(proportions, len(need_resample), replacement=True)]

        # Check which envs actually changed terrain type
        changed_mask = terrain.terrain_types[need_resample] != sampled
        changed_envs = need_resample[changed_mask]

        terrain.terrain_types[need_resample] = sampled
        # Only reset the level to 0 if the terrain type was actually changed
        if len(changed_envs) > 0:
            terrain.terrain_levels[changed_envs] = 0

        terrain.env_origins[need_resample] = terrain.terrain_origins[
            terrain.terrain_levels[need_resample], terrain.terrain_types[need_resample]
        ]
        terrain._obstacle_release_env_allowed[need_resample] = True

    result = terrain_levels_vel_strict(env, env_ids, command_name, asset_cfg=asset_cfg)
    result["allowed_types"] = torch.tensor(float(len(allowed_type_ids)), device=env.device)
    for name in ("pyramid_stairs", "pyramid_stairs_inv", "random_grid", "rc_wall"):
        result[f"{name}_released"] = torch.tensor(float(name in allowed_names), device=env.device)
    return result
