"""MDP Reward functions for unitree locomotion training."""

from __future__ import annotations

from typing import TYPE_CHECKING
import torch

from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.entity import Entity
from mjlab.utils.lab_api.string import resolve_matching_names_values

if TYPE_CHECKING:
    from mjlab.envs import ManagerBasedRlEnv


def _finite(value: torch.Tensor, nan: float = 0.0, posinf: float = 0.0, neginf: float = 0.0) -> torch.Tensor:
    """Keep diagnostic metrics finite when a terminating env has invalid physics state."""
    return torch.nan_to_num(value, nan=nan, posinf=posinf, neginf=neginf)


def track_linear_velocity(
    env: ManagerBasedRlEnv,
    std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Reward tracking commanded horizontal xy velocity."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset: Entity = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    actual = asset.data.root_link_lin_vel_b
    xy_error = torch.sum(torch.square(command[:, :2] - actual[:, :2]), dim=1)
    reward = torch.exp(-xy_error / std**2)
    reward *= torch.clamp(-asset.data.projected_gravity_b[:, 2], 0.0, 0.7) / 0.7
    return reward


def track_linear_velocity_x(
    env: ManagerBasedRlEnv,
    std: float,
    command_name: str,
    gravity_z_power: float | None = None,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """LocoLeggedWheel-style independent x velocity tracking reward."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset: Entity = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    x_error = torch.square(command[:, 0] - asset.data.root_link_lin_vel_b[:, 0])
    reward = torch.exp(-x_error / std**2)
    if gravity_z_power is not None:
        reward *= torch.clamp(-asset.data.projected_gravity_b[:, 2], min=0.0) ** gravity_z_power
    else:
        reward *= -asset.data.projected_gravity_b[:, 2]
    return reward


def track_linear_velocity_y(
    env: ManagerBasedRlEnv,
    std: float,
    command_name: str,
    gravity_z_power: float | None = None,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """LocoLeggedWheel-style independent y velocity tracking reward."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset: Entity = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    y_error = torch.square(command[:, 1] - asset.data.root_link_lin_vel_b[:, 1])
    reward = torch.exp(-y_error / std**2)
    if gravity_z_power is not None:
        reward *= torch.clamp(-asset.data.projected_gravity_b[:, 2], min=0.0) ** gravity_z_power
    else:
        reward *= -asset.data.projected_gravity_b[:, 2]
    return reward


def track_angular_velocity_z(
    env: ManagerBasedRlEnv,
    std: float,
    command_name: str,
    gravity_z_power: float | None = None,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """LocoLeggedWheel-style independent yaw velocity tracking reward."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset: Entity = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    z_error = torch.square(command[:, 2] - asset.data.root_link_ang_vel_b[:, 2])
    reward = torch.exp(-z_error / std**2)
    if gravity_z_power is not None:
        reward *= torch.clamp(-asset.data.projected_gravity_b[:, 2], min=0.0) ** gravity_z_power
    else:
        reward *= -asset.data.projected_gravity_b[:, 2]
    return reward


def track_angular_velocity(
    env: ManagerBasedRlEnv,
    std: float,
    command_name: str,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Reward tracking commanded yaw rate."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset: Entity = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    actual = asset.data.root_link_ang_vel_b
    z_error = torch.square(command[:, 2] - actual[:, 2])
    reward = torch.exp(-z_error / std**2)
    reward *= torch.clamp(-asset.data.projected_gravity_b[:, 2], 0.0, 0.7) / 0.7
    return reward


def stair_lateral_yaw_drift_l2(
    env: ManagerBasedRlEnv,
    terrain_names: tuple[str, ...] = ("pyramid_stairs", "pyramid_stairs_inv"),
    y_scale: float = 1.0,
    yaw_scale: float = 1.0,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Penalize sideways velocity and yaw-rate drift only on stair terrains."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset: Entity = env.scene[asset_cfg.name]

    terrain = getattr(env.scene, "terrain", None)
    terrain_types = getattr(terrain, "terrain_types", None)
    terrain_cfg = getattr(terrain, "cfg", None)
    terrain_generator = getattr(terrain_cfg, "terrain_generator", None)
    if terrain_types is None or terrain_generator is None:
        return torch.zeros(env.num_envs, device=env.device)

    sub_terrain_names = list(terrain_generator.sub_terrains.keys())
    mask = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    for name in terrain_names:
        if name in sub_terrain_names:
            mask |= terrain_types == sub_terrain_names.index(name)

    y_vel = asset.data.root_link_lin_vel_b[:, 1]
    yaw_vel = asset.data.root_link_ang_vel_b[:, 2]
    penalty = y_scale * torch.square(y_vel) + yaw_scale * torch.square(yaw_vel)
    return _finite(torch.where(mask, penalty, torch.zeros_like(penalty)))


def base_height_l2(
    env: ManagerBasedRlEnv,
    target_height: float = 0.36,
    sensor_cfg: SceneEntityCfg | None = None,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Penalize base height deviation from target height, relative to terrain or origin."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset: Entity = env.scene[asset_cfg.name]
    
    has_sensor = False
    if sensor_cfg is not None:
        try:
            _ = env.scene[sensor_cfg.name]
            has_sensor = True
        except KeyError:
            has_sensor = False
            
    if has_sensor:
        from mjlab.envs.mdp.observations import height_scan
        # Get height scanner readings (each ray's height relative to base frame Z)
        heights = height_scan(env, sensor_cfg.name, offset=0.0)
        # Clean any invalid values
        heights = torch.nan_to_num(heights, nan=target_height, posinf=target_height, neginf=target_height)
        
        # Filter out rays that represent extreme misses/cliffs (e.g. height < 0 or height > 1.5)
        # The robot target height is ~0.36m, so the scanner should measure something between 0.1m and 0.8m usually.
        valid_mask = (heights > 0.0) & (heights < 1.5)
        
        # Calculate mean height above terrain ignoring misses
        sum_heights = torch.sum(torch.where(valid_mask, heights, torch.zeros_like(heights)), dim=1)
        count_valid = torch.sum(valid_mask.float(), dim=1)
        measured_height = torch.where(count_valid > 0, sum_heights / torch.clamp(count_valid, min=1.0), torch.full_like(sum_heights, target_height))
        
        error = measured_height - target_height
    else:
        if hasattr(env.scene, "env_origins") and env.scene.env_origins is not None:
            env_origins_z = env.scene.env_origins[:, 2]
        else:
            env_origins_z = torch.zeros(env.num_envs, device=env.device)
        
        root_z = asset.data.root_link_pos_w[:, 2] - env_origins_z
        error = root_z - target_height
        
    reward = torch.square(error)
    reward *= torch.clamp(-asset.data.projected_gravity_b[:, 2], 0.0, 0.7) / 0.7
    return reward


def safe_height_scan(env: ManagerBasedRlEnv, sensor_name: str) -> torch.Tensor:
    """Safely fetch height scanner outputs, replacing NaNs/infs with finite values."""
    from mjlab.envs.mdp.observations import height_scan
    result = height_scan(env, sensor_name)
    return torch.nan_to_num(result, nan=0.0, posinf=5.0, neginf=-5.0)


def safe_base_lin_vel(env: ManagerBasedRlEnv) -> torch.Tensor:
    """Safely fetch base linear velocity, replacing NaNs/infs with finite values."""
    from mjlab.envs.mdp.observations import base_lin_vel
    result = base_lin_vel(env)
    return torch.nan_to_num(result, nan=0.0, posinf=100.0, neginf=-100.0)


def safe_foot_air_time(env: ManagerBasedRlEnv, sensor_name: str) -> torch.Tensor:
    """Safely fetch foot air time, replacing NaNs with zeros."""
    from mjlab.tasks.velocity.mdp.observations import foot_air_time
    result = foot_air_time(env, sensor_name)
    return torch.nan_to_num(result, nan=0.0)


def safe_foot_contact(env: ManagerBasedRlEnv, sensor_name: str) -> torch.Tensor:
    """Safely fetch foot contact flags, replacing NaNs with zeros."""
    from mjlab.tasks.velocity.mdp.observations import foot_contact
    result = foot_contact(env, sensor_name)
    return torch.nan_to_num(result, nan=0.0)


def safe_foot_contact_forces(env: ManagerBasedRlEnv, sensor_name: str) -> torch.Tensor:
    """Safely fetch foot contact forces, replacing NaNs with zeros."""
    from mjlab.tasks.velocity.mdp.observations import foot_contact_forces
    result = foot_contact_forces(env, sensor_name)
    return torch.nan_to_num(result, nan=0.0)


def feet_clearance(
    env: ManagerBasedRlEnv,
    target_height: float,
    command_name: str = "twist",
    command_threshold: float = 0.1,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Penalize deviation from the target foot clearance height."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot", body_names=(".*_wheel_Link",))
    asset: Entity = env.scene[asset_cfg.name]
    
    foot_z = asset.data.body_com_pos_w[:, asset_cfg.body_ids, 2]
    foot_vel_xy = asset.data.body_link_vel_w[:, asset_cfg.body_ids, :2]
    vel_norm = torch.norm(foot_vel_xy, dim=-1)
    delta = torch.abs(foot_z - target_height)
    cost = torch.sum(delta * vel_norm, dim=1)
    
    cmd = env.command_manager.get_command(command_name)
    linear_norm = torch.norm(cmd[:, :2], dim=1)
    angular_norm = torch.abs(cmd[:, 2])
    total_command = linear_norm + angular_norm
    active = (total_command > command_threshold).float()
    
    reward = cost * active
    return reward


def soft_landing(
    env: ManagerBasedRlEnv,
    sensor_name: str,
    command_name: str = "twist",
    command_threshold: float = 0.05,
) -> torch.Tensor:
    """Penalize high impact forces on the first landing contact."""
    from mjlab.sensor import ContactSensor
    contact_sensor: ContactSensor = env.scene[sensor_name]
    forces = contact_sensor.data.force
    force_magnitude = torch.norm(forces, dim=-1)
    first_contact = contact_sensor.compute_first_contact(dt=env.step_dt)
    landing_impact = force_magnitude * first_contact.float()
    cost = torch.sum(landing_impact, dim=1)
    
    cmd = env.command_manager.get_command(command_name)
    linear_norm = torch.norm(cmd[:, :2], dim=1)
    angular_norm = torch.abs(cmd[:, 2])
    total_command = linear_norm + angular_norm
    active = (total_command > command_threshold).float()
    
    reward = cost * active
    return reward


def _total_command_magnitude(env: ManagerBasedRlEnv, command_name: str) -> torch.Tensor:
    """Retrieve absolute horizontal command speed."""
    command = env.command_manager.get_command(command_name)
    linear_norm = torch.norm(command[:, :2], dim=1)
    angular_norm = torch.abs(command[:, 2])
    return linear_norm + angular_norm


def wheel_roll_tracking(
    env: ManagerBasedRlEnv,
    command_name: str,
    wheel_radius: float,
    wheel_track: float,
    std: float,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Reward wheel angular velocity matching the commanded planar motion."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot", joint_names=(".*_wheel_joint",))
    asset: Entity = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)

    wheel_vel = asset.data.joint_vel[:, asset_cfg.joint_ids]
    # Assumes order: fl, fr, rl, rr. Side signs: fl=1, fr=-1, rl=1, rr=-1
    side_signs = torch.tensor(
        [1.0, -1.0, 1.0, -1.0],
        device=env.device,
        dtype=wheel_vel.dtype,
    ).unsqueeze(0)

    lin_vel_x = command[:, 0:1]
    ang_vel_z = command[:, 2:3]
    target_wheel_vel = (
        lin_vel_x + 0.5 * wheel_track * side_signs * ang_vel_z
    ) / wheel_radius

    err = torch.mean(torch.square(wheel_vel - target_wheel_vel), dim=1)
    reward = torch.exp(-err / (std**2))
    return reward


def adaptive_leg_motion_penalty(
    env: ManagerBasedRlEnv,
    command_name: str,
    sensor_name: str,
    command_threshold: float = 0.05,
    tilt_relax_start: float = 0.08,
    tilt_relax_end: float = 0.30,
    contact_target: float = 0.85,
    min_penalty_scale: float = 0.2,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Penalize leg motion less when tilt or wheel contact suggests recovery."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot", joint_names=(
            ".*_hip_abduction_joint",
            ".*_hip_pitch_joint",
            ".*_knee_joint",
        ))
    asset: Entity = env.scene[asset_cfg.name]
    leg_vel = asset.data.joint_vel[:, asset_cfg.joint_ids]
    base_cost = torch.mean(torch.square(leg_vel), dim=1)
    active = (_total_command_magnitude(env, command_name) > command_threshold).float()

    tilt_xy = torch.norm(asset.data.projected_gravity_b[:, :2], dim=1)
    tilt_relax = torch.clamp(
        (tilt_xy - tilt_relax_start) / max(tilt_relax_end - tilt_relax_start, 1.0e-6),
        min=0.0,
        max=1.0,
    )

    from mjlab.sensor import ContactSensor
    sensor: ContactSensor = env.scene[sensor_name]
    in_contact = (sensor.data.found > 0).float()
    contact_fraction = torch.mean(in_contact, dim=1)
    if contact_fraction.ndim > 1:
        contact_fraction = torch.mean(contact_fraction, dim=1)
    contact_relax = torch.clamp(
        (contact_target - contact_fraction) / max(contact_target, 1.0e-6),
        min=0.0,
        max=1.0,
    )

    relax = torch.maximum(tilt_relax, contact_relax)
    penalty_scale = 1.0 - (1.0 - min_penalty_scale) * relax
    reward = base_cost * active * penalty_scale
    return reward


def contact_fraction_reward(
    env: ManagerBasedRlEnv,
    sensor_name: str,
) -> torch.Tensor:
    """Reward persistent wheel-ground contact."""
    from mjlab.sensor import ContactSensor
    sensor: ContactSensor = env.scene[sensor_name]
    in_contact = (sensor.data.found > 0).float()
    reward = torch.mean(in_contact, dim=1)
    return reward


def stand_still(
    env,
    command_name: str,
    command_threshold: float = 0.1,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot", joint_names=(
            ".*_hip_abduction_joint", ".*_hip_pitch_joint", ".*_knee_joint",
        ))
    asset = env.scene[asset_cfg.name]
    diff = asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    cost = torch.mean(torch.abs(diff), dim=1)
    command = env.command_manager.get_command(command_name)
    linear_norm = torch.norm(command[:, :2], dim=1)
    angular_norm = torch.abs(command[:, 2])
    inactive = (linear_norm + angular_norm < command_threshold).float()
    reward = cost * inactive
    reward *= torch.clamp(-asset.data.projected_gravity_b[:, 2], 0.0, 0.7) / 0.7
    return reward

def hip_deviation(
    env: ManagerBasedRlEnv,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Penalize hip abduction joints deviating from default."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot", joint_names=(".*_hip_abduction_joint",))
    asset: Entity = env.scene[asset_cfg.name]
    q = asset.data.joint_pos[:, asset_cfg.joint_ids]
    q0 = asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    reward = torch.sum(torch.square(q - q0), dim=1)
    return reward


def joint_deviation_l2(
    env: ManagerBasedRlEnv,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Penalize all specified joints deviating from default."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot", joint_names=(".*_hip_abduction_joint", ".*_hip_pitch_joint", ".*_knee_joint"))
    asset: Entity = env.scene[asset_cfg.name]
    q = asset.data.joint_pos[:, asset_cfg.joint_ids]
    q0 = asset.data.default_joint_pos[:, asset_cfg.joint_ids]
    reward = torch.sum(torch.square(q - q0), dim=1)
    return reward


def flat_orientation_l2(
    env: ManagerBasedRlEnv,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Penalize non-flat base orientation (roll/pitch via projected gravity)."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset: Entity = env.scene[asset_cfg.name]
    reward = torch.sum(torch.square(asset.data.projected_gravity_b[:, :2]), dim=1)
    return reward


def lin_vel_z_l2(
    env: ManagerBasedRlEnv,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Penalize vertical (z) base linear velocity."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset: Entity = env.scene[asset_cfg.name]
    reward = torch.square(asset.data.root_link_lin_vel_b[:, 2])
    reward *= torch.clamp(-asset.data.projected_gravity_b[:, 2], 0.0, 0.7) / 0.7
    return reward


def ang_vel_xy_l2(
    env: ManagerBasedRlEnv,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Penalize xy-axis base angular velocity using the go2w kernel."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset: Entity = env.scene[asset_cfg.name]
    reward = torch.sum(torch.square(asset.data.root_link_ang_vel_b[:, :2]), dim=1)
    reward *= torch.clamp(-asset.data.projected_gravity_b[:, 2], 0.0, 0.7) / 0.7
    return reward


class variable_posture:
    """Reward posture tracking with speed-dependent tolerance."""

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRlEnv):
        asset: Entity = env.scene[cfg.params["asset_cfg"].name]
        default_joint_pos = asset.data.default_joint_pos
        self.default_joint_pos = default_joint_pos

        _, joint_names = asset.find_joints(cfg.params["asset_cfg"].joint_names)

        _, _, std_standing = resolve_matching_names_values(
            data=cfg.params["std_standing"], list_of_strings=joint_names,
        )
        self.std_standing = torch.tensor(std_standing, device=env.device, dtype=torch.float32)

        _, _, std_walking = resolve_matching_names_values(
            data=cfg.params["std_walking"], list_of_strings=joint_names,
        )
        self.std_walking = torch.tensor(std_walking, device=env.device, dtype=torch.float32)

        _, _, std_running = resolve_matching_names_values(
            data=cfg.params["std_running"], list_of_strings=joint_names,
        )
        self.std_running = torch.tensor(std_running, device=env.device, dtype=torch.float32)

    def __call__(
        self,
        env: ManagerBasedRlEnv,
        std_standing: dict,
        std_walking: dict,
        std_running: dict,
        asset_cfg: SceneEntityCfg,
        command_name: str,
        walking_threshold: float = 0.5,
        running_threshold: float = 1.5,
    ) -> torch.Tensor:
        del std_standing, std_walking, std_running
        asset: Entity = env.scene[asset_cfg.name]
        total_speed = _total_command_magnitude(env, command_name)
        
        # 提取身体倾斜度 (重力在 XY 平面的投影)
        tilt_xy = torch.norm(asset.data.projected_gravity_b[:, :2], dim=1)
        
        # 如果身体倾斜 > 0.15 (约 8.6 度)，说明正在爬墙、越障或已摔倒
        is_tilting = (tilt_xy > 0.15).float()
        is_running_cmd = (total_speed >= running_threshold).float()
        
        # 【核心越障修改】只要速度极快，或者正在倾斜攀爬，直接赋予最大幅度的动作容忍权 (running_mask)
        running_mask = torch.clamp(is_running_cmd + is_tilting, 0.0, 1.0)
        
        # 剩下的平稳且不倾斜的状态，再去根据低速指令判断是 walking 还是 standing
        not_running_mask = 1.0 - running_mask
        is_walking_cmd = (total_speed >= walking_threshold).float()
        
        walking_mask = not_running_mask * is_walking_cmd
        standing_mask = not_running_mask * (1.0 - is_walking_cmd)

        std = (
            self.std_standing * standing_mask.unsqueeze(1)
            + self.std_walking * walking_mask.unsqueeze(1)
            + self.std_running * running_mask.unsqueeze(1)
        )

        current_joint_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
        desired_joint_pos = self.default_joint_pos[:, asset_cfg.joint_ids]
        error_squared = torch.square(current_joint_pos - desired_joint_pos)
        reward = torch.exp(-torch.mean(error_squared / (std**2), dim=1))
        return reward


def get_mode_id(env: ManagerBasedRlEnv) -> torch.Tensor:
    """Retrieve current integer mode_id tensor from command_manager.

    Safely falls back to zeros (walk mode) if command manager or mode is not initialized.
    """
    if not hasattr(env, "command_manager") or env.command_manager is None:
        return torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
    term = env.command_manager.get_term("mode")
    if term is None or not hasattr(term, "get_mode_id"):
        return torch.zeros(env.num_envs, dtype=torch.long, device=env.device)
    return term.get_mode_id()


def crouch_height_reward(
    env: ManagerBasedRlEnv,
    target_height: float = 0.22,
    std: float = 0.05,
    mode_command_name: str = "mode",
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Reward the robot for keeping body low under target height when in crouch mode (mode_id == 2).

    Uses a single-sided penalty so that going lower than target_height receives full reward.
    """
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset: Entity = env.scene[asset_cfg.name]

    if hasattr(env.scene, "env_origins") and env.scene.env_origins is not None:
        env_origins_z = env.scene.env_origins[:, 2]
    else:
        env_origins_z = torch.zeros(env.num_envs, device=env.device)

    root_z = asset.data.root_link_pos_w[:, 2] - env_origins_z
    error = torch.square(torch.clamp(root_z - target_height, min=0.0))
    reward = torch.exp(-error / std ** 2)

    mode_id = get_mode_id(env)
    active = (mode_id == 2).float()
    reward = reward * active
    return reward


class mode_conditioned_posture:
    """Conditioned posture reward selecting targets/tolerances by active mode_id."""

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRlEnv) -> None:
        asset = env.scene[cfg.params["asset_cfg"].name]
        default_joint_pos = asset.data.default_joint_pos
        assert default_joint_pos is not None
        self.default_joint_pos = default_joint_pos

        _, joint_names = asset.find_joints(cfg.params["asset_cfg"].joint_names)
        self.joint_ids = asset.find_joints(cfg.params["asset_cfg"].joint_names)[0]

        _, _, std_walk = resolve_matching_names_values(
            data=cfg.params["std_walk"], list_of_strings=joint_names,
        )
        self.std_walk = torch.tensor(std_walk, device=env.device, dtype=torch.float32)

        if "crouch_pos" in cfg.params:
            _, _, crouch_pos_vals = resolve_matching_names_values(
                data=cfg.params["crouch_pos"], list_of_strings=joint_names,
            )
            self.crouch_pos_target = torch.tensor(
                crouch_pos_vals, device=env.device, dtype=torch.float32
            ).unsqueeze(0)
        else:
            self.crouch_pos_target = None

        climb_scale = cfg.params.get("climb_std_scale", 3.0)
        crouch_scale = cfg.params.get("crouch_std_scale", 4.0)
        self.std_climb = self.std_walk * climb_scale
        self.std_crouch = self.std_walk * crouch_scale

    def __call__(
        self,
        env: ManagerBasedRlEnv,
        std_walk: dict,
        asset_cfg: SceneEntityCfg,
        climb_std_scale: float = 3.0,
        crouch_std_scale: float = 4.0,
        crouch_pos: dict | None = None,
    ) -> torch.Tensor:
        del std_walk, crouch_pos

        asset = env.scene[asset_cfg.name]
        mode_id = get_mode_id(env).float()

        walk_mask = (mode_id == 0).float().unsqueeze(1)
        climb_mask = (mode_id == 1).float().unsqueeze(1)
        crouch_mask = (mode_id == 2).float().unsqueeze(1)

        std = (
            self.std_walk * walk_mask
            + self.std_climb * climb_mask
            + self.std_crouch * crouch_mask
        )

        current_pos = asset.data.joint_pos[:, self.joint_ids]
        base_desired_pos = self.default_joint_pos[:, self.joint_ids]

        if self.crouch_pos_target is not None:
            desired_pos = (
                base_desired_pos * (walk_mask + climb_mask)
                + self.crouch_pos_target * crouch_mask
            )
        else:
            desired_pos = base_desired_pos

        error_sq = torch.square(current_pos - desired_pos)
        reward = torch.exp(-torch.mean(error_sq / (std ** 2 + 1e-8), dim=1))
        return reward


def crawl_height_reward(
    env: ManagerBasedRlEnv,
    target_height: float = 0.22,
    std: float = 0.05,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Reward the robot for staying below target crawling height (0.22m).

    Uses a single-sided penalty: error is 0 if root_z <= target_height.
    """
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset: Entity = env.scene[asset_cfg.name]
    
    if hasattr(env.scene, "env_origins") and env.scene.env_origins is not None:
        env_origins_z = env.scene.env_origins[:, 2]
    else:
        env_origins_z = torch.zeros(env.num_envs, device=env.device)
        
    root_z = asset.data.root_link_pos_w[:, 2] - env_origins_z
    error = torch.square(torch.clamp(root_z - target_height, min=0.0))
    reward = torch.exp(-error / std ** 2)
    return reward


def _get_terrain_levels(env: ManagerBasedRlEnv) -> torch.Tensor | None:
    """Safely fetch current active terrain levels from environment."""
    terrain = getattr(env.scene, "terrain", None)
    if terrain is not None and hasattr(terrain, "terrain_levels"):
        return terrain.terrain_levels
    if hasattr(env, "terrain_levels"):
        return env.terrain_levels
    return None


def terrain_level_bonus(env: ManagerBasedRlEnv) -> torch.Tensor:
    """Positive step bonus normalized to [0, 1] by max terrain level (num_rows-1 = 9).

    Normalizing prevents the raw level integer (0~9) from dominating the reward
    signal at high difficulty, which would otherwise incentivize level-rushing
    over quality locomotion.
    """
    levels = _get_terrain_levels(env)
    if levels is None:
        return torch.zeros(env.num_envs, device=env.device)
    return levels.float() / 9.0


def action_rate_curriculum_l2(env: ManagerBasedRlEnv) -> torch.Tensor:
    """Apply action change rate penalty with curriculum-driven scaling.

    Penalizes rate changes fully at level 0 (flat) for optimal smoothness,
    decaying to 10% penalty at levels 8+ to facilitate explosive dynamic clearing maneuvers.
    """
    action_rate = torch.sum(torch.square(env.action_manager.action - env.action_manager.prev_action), dim=1)
    levels = _get_terrain_levels(env)
    if levels is None:
        reward = action_rate
    else:
        decay = 1.0 - 0.9 * torch.clamp(levels.float() / 8.0, 0.0, 1.0)
        reward = action_rate * decay
        
    return reward


def leg_symmetry(
    env: ManagerBasedRlEnv,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Penalize asymmetry between left and right legs (pitch and knee joints) robustly."""
    asset: Entity = env.scene["robot"]
    joint_names = asset.data.joint_names
    
    def find_idx(side, jtype):
        for i, name in enumerate(joint_names):
            name_lower = name.lower()
            if side in name_lower and jtype in name_lower:
                return i
        # Fallback if not found to avoid crash, though it should find it
        return 0
        
    fl_p = find_idx("fl_", "pitch")
    fr_p = find_idx("fr_", "pitch")
    rl_p = find_idx("rl_", "pitch")
    rr_p = find_idx("rr_", "pitch")
    
    fl_k = find_idx("fl_", "knee")
    fr_k = find_idx("fr_", "knee")
    rl_k = find_idx("rl_", "knee")
    rr_k = find_idx("rr_", "knee")
    
    q = asset.data.joint_pos
    cost = torch.square(q[:, fl_p] - q[:, fr_p]) + \
           torch.square(q[:, rl_p] - q[:, rr_p]) + \
           torch.square(q[:, fl_k] - q[:, fr_k]) + \
           torch.square(q[:, rl_k] - q[:, rr_k])
    return cost

def feet_contact_without_cmd(env, command_name: str, sensor_name: str) -> torch.Tensor:
    from mjlab.sensor import ContactSensor
    contact_sensor = env.scene[sensor_name]
    contact = contact_sensor.data.found > 0
    reward = torch.sum(contact, dim=-1).float()
    cmd = env.command_manager.get_command(command_name)
    linear_norm = torch.norm(cmd[:, :2], dim=1)
    angular_norm = torch.abs(cmd[:, 2])
    reward *= (linear_norm + angular_norm) < 0.1
    asset = env.scene["robot"]
    reward *= torch.clamp(-asset.data.projected_gravity_b[:, 2], 0.0, 0.7) / 0.7
    return reward

def joint_pos_penalty(
    env,
    command_name: str,
    stand_still_scale: float,
    velocity_threshold: float,
    command_threshold: float,
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset = env.scene[asset_cfg.name]
    cmd_val = env.command_manager.get_command(command_name)
    cmd = torch.linalg.norm(cmd_val[:, :2], dim=1) + torch.abs(cmd_val[:, 2])
    body_vel = torch.linalg.norm(asset.data.root_link_lin_vel_b[:, :2], dim=1)
    running_reward = torch.linalg.norm(
        (asset.data.joint_pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]), dim=1
    )
    reward = torch.where(
        torch.logical_or(cmd > command_threshold, body_vel > velocity_threshold),
        running_reward,
        stand_still_scale * running_reward,
    )
    reward *= torch.clamp(-asset.data.projected_gravity_b[:, 2], 0.0, 0.7) / 0.7
    return reward

def joint_mirror(env, mirror_joints: list[list[str]], asset_cfg: SceneEntityCfg | None = None) -> torch.Tensor:
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset = env.scene[asset_cfg.name]
    if not hasattr(env, "joint_mirror_joints_cache") or env.joint_mirror_joints_cache is None:
        env.joint_mirror_joints_cache = []
        for joint_pair in mirror_joints:
            p0 = asset.find_joints(joint_pair[0])[0]
            p1 = asset.find_joints(joint_pair[1])[0]
            env.joint_mirror_joints_cache.append([p0, p1])
    reward = torch.zeros(env.num_envs, device=env.device)
    for joint_pair in env.joint_mirror_joints_cache:
        diff = torch.sum(
            torch.square(asset.data.joint_pos[:, joint_pair[0]] - asset.data.joint_pos[:, joint_pair[1]]),
            dim=-1,
        )
        reward += diff
    reward *= 1 / len(mirror_joints) if len(mirror_joints) > 0 else 0
    reward *= torch.clamp(-asset.data.projected_gravity_b[:, 2], 0.0, 0.7) / 0.7
    return reward


def undesired_contacts(
    env: ManagerBasedRlEnv,
    sensor_name: str,
    threshold: float = 1.0,
) -> torch.Tensor:
    """Penalize non-wheel contacts above a force threshold."""
    from mjlab.sensor import ContactSensor

    sensor: ContactSensor = env.scene[sensor_name]
    data = sensor.data
    if data.force_history is not None:
        force_mag = torch.norm(data.force_history, dim=-1)
        is_contact = torch.max(force_mag, dim=2)[0] > threshold
    else:
        force_mag = torch.norm(data.force, dim=-1)
        is_contact = force_mag > threshold
    reward = torch.sum(is_contact, dim=1).float()
    asset = env.scene["robot"]
    reward *= torch.clamp(-asset.data.projected_gravity_b[:, 2], 0.0, 0.7) / 0.7
    return reward


def contact_forces(
    env: ManagerBasedRlEnv,
    sensor_name: str,
    threshold: float = 100.0,
) -> torch.Tensor:
    """Penalize foot contact forces above threshold."""
    from mjlab.sensor import ContactSensor

    sensor: ContactSensor = env.scene[sensor_name]
    data = sensor.data
    if data.force_history is not None:
        force_mag = torch.norm(data.force_history, dim=-1)
        peak_force = torch.max(force_mag, dim=2)[0]
    else:
        peak_force = torch.norm(data.force, dim=-1)
    reward = torch.sum(torch.clamp(peak_force - threshold, min=0.0), dim=1)
    asset = env.scene["robot"]
    reward *= torch.clamp(-asset.data.projected_gravity_b[:, 2], 0.0, 0.7) / 0.7
    return reward



def upward(env, asset_cfg=None):
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg('robot')
    asset = env.scene[asset_cfg.name]
    reward = torch.square(1 - asset.data.projected_gravity_b[:, 2])
    return reward


def joint_power(env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg | None = None) -> torch.Tensor:
    """Penalty for total joint mechanical power: sum(|tau * dq|)."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset: Entity = env.scene[asset_cfg.name]
    return torch.sum(torch.abs(asset.data.qfrc_actuator * asset.data.joint_vel), dim=1)

def upright_roll_only(env, asset_cfg=None):
    if asset_cfg is None:
        from mjlab.envs.manager_based_rl_env import SceneEntityCfg
        asset_cfg = SceneEntityCfg('robot')
    asset = env.scene[asset_cfg.name]
    reward = torch.square(asset.data.projected_gravity_b[:, 1])
    return reward

def track_linear_velocity_l1(env, std, command_name):
    asset = env.scene['robot']
    cmd = env.command_manager.get_command(command_name)
    lin_vel_error = torch.linalg.norm(asset.data.root_link_lin_vel_b[:, :2] - cmd[:, :2], dim=1)
    reward = torch.exp(-lin_vel_error / std)
    return reward

def pitch_control_penalty(env, max_pitch_rad: float = 0.50, asset_cfg=None) -> torch.Tensor:
    """Penalize excessive pitch orientation beyond a safe threshold.
    
    Allows pitch angles up to max_pitch_rad (e.g. 29 degrees) for climbing obstacles,
    but quadratically penalizes any pitch angle exceeding this threshold (e.g. during flinging).
    """
    import math
    if asset_cfg is None:
        from mjlab.envs.manager_based_rl_env import SceneEntityCfg
        asset_cfg = SceneEntityCfg('robot')
    asset = env.scene[asset_cfg.name]
    g_x = asset.data.projected_gravity_b[:, 0]
    g_x_threshold = math.sin(max_pitch_rad)
    excessive_pitch = torch.clamp(torch.abs(g_x) - g_x_threshold, min=0.0)
    reward = torch.square(excessive_pitch)
    return reward


def tracking_lin_vel_error(env, command_name: str = "twist", asset_cfg=None) -> torch.Tensor:
    """Current-step xy velocity tracking error."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset = env.scene[asset_cfg.name]
    cmd = env.command_manager.get_command(command_name)
    return _finite(torch.linalg.norm(asset.data.root_link_lin_vel_b[:, :2] - cmd[:, :2], dim=1))


def tracking_yaw_vel_error(env, command_name: str = "twist", asset_cfg=None) -> torch.Tensor:
    """Current-step yaw velocity tracking error."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset = env.scene[asset_cfg.name]
    cmd = env.command_manager.get_command(command_name)
    return _finite(torch.abs(asset.data.root_link_ang_vel_b[:, 2] - cmd[:, 2]))


def tracking_lin_vel_x_error(env, command_name: str = "twist", asset_cfg=None) -> torch.Tensor:
    """Current-step x velocity tracking error."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset = env.scene[asset_cfg.name]
    cmd = env.command_manager.get_command(command_name)
    return _finite(torch.abs(asset.data.root_link_lin_vel_b[:, 0] - cmd[:, 0]))


def tracking_lin_vel_y_error(env, command_name: str = "twist", asset_cfg=None) -> torch.Tensor:
    """Current-step y velocity tracking error."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset = env.scene[asset_cfg.name]
    cmd = env.command_manager.get_command(command_name)
    return _finite(torch.abs(asset.data.root_link_lin_vel_b[:, 1] - cmd[:, 1]))


def tracking_lin_vel_along_command_error(
    env,
    command_name: str = "twist",
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Error of velocity projected onto the commanded xy direction."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset = env.scene[asset_cfg.name]
    cmd = env.command_manager.get_command(command_name)
    cmd_xy = cmd[:, :2]
    cmd_speed = torch.linalg.norm(cmd_xy, dim=1)
    direction = cmd_xy / torch.clamp(cmd_speed.unsqueeze(1), min=1.0e-6)
    actual_along = torch.sum(asset.data.root_link_lin_vel_b[:, :2] * direction, dim=1)
    return _finite(torch.abs(actual_along - cmd_speed))


def actual_lin_vel_orthogonal_command_mean(
    env,
    command_name: str = "twist",
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Absolute velocity component perpendicular to the commanded xy direction."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset = env.scene[asset_cfg.name]
    cmd = env.command_manager.get_command(command_name)
    cmd_xy = cmd[:, :2]
    cmd_speed = torch.linalg.norm(cmd_xy, dim=1)
    direction = cmd_xy / torch.clamp(cmd_speed.unsqueeze(1), min=1.0e-6)
    actual = asset.data.root_link_lin_vel_b[:, :2]
    actual_along = torch.sum(actual * direction, dim=1, keepdim=True) * direction
    orthogonal = actual - actual_along
    return _finite(torch.linalg.norm(orthogonal, dim=1))


def command_lin_vel_mean(env, command_name: str = "twist") -> torch.Tensor:
    """Current commanded xy speed magnitude."""
    cmd = env.command_manager.get_command(command_name)
    return torch.linalg.norm(cmd[:, :2], dim=1)


def command_yaw_vel_abs_mean(env, command_name: str = "twist") -> torch.Tensor:
    """Current commanded yaw speed magnitude."""
    cmd = env.command_manager.get_command(command_name)
    return torch.abs(cmd[:, 2])


def actual_lin_vel_mean(env, asset_cfg=None) -> torch.Tensor:
    """Current actual xy speed magnitude."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset = env.scene[asset_cfg.name]
    return _finite(torch.linalg.norm(asset.data.root_link_lin_vel_b[:, :2], dim=1))


def tracking_lin_vel_error_band_mean(
    env,
    command_name: str = "twist",
    min_speed: float = 0.0,
    max_speed: float = 10.0,
    asset_cfg=None,
) -> torch.Tensor:
    """Broadcast the masked mean xy error for a command speed band."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset = env.scene[asset_cfg.name]
    cmd = env.command_manager.get_command(command_name)
    speed = torch.linalg.norm(cmd[:, :2], dim=1)
    err = _finite(torch.linalg.norm(asset.data.root_link_lin_vel_b[:, :2] - cmd[:, :2], dim=1))
    active = torch.logical_and(speed >= min_speed, speed < max_speed)
    denom = active.float().sum().clamp_min(1.0)
    mean_err = torch.sum(torch.where(active, err, torch.zeros_like(err))) / denom
    return torch.full_like(err, mean_err)


def tracking_lin_vel_axis_error_band_mean(
    env,
    axis: int,
    command_name: str = "twist",
    min_speed: float = 0.0,
    max_speed: float = 10.0,
    asset_cfg=None,
) -> torch.Tensor:
    """Broadcast the masked mean x/y error for a command speed band."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset = env.scene[asset_cfg.name]
    cmd = env.command_manager.get_command(command_name)
    speed = torch.linalg.norm(cmd[:, :2], dim=1)
    err = _finite(torch.abs(asset.data.root_link_lin_vel_b[:, axis] - cmd[:, axis]))
    active = torch.logical_and(speed >= min_speed, speed < max_speed)
    denom = active.float().sum().clamp_min(1.0)
    mean_err = torch.sum(torch.where(active, err, torch.zeros_like(err))) / denom
    return torch.full_like(err, mean_err)


def command_band_active(
    env,
    command_name: str = "twist",
    min_speed: float = 0.0,
    max_speed: float = 10.0,
) -> torch.Tensor:
    """Fraction helper for command speed bands."""
    cmd = env.command_manager.get_command(command_name)
    speed = torch.linalg.norm(cmd[:, :2], dim=1)
    return torch.logical_and(speed >= min_speed, speed < max_speed).float()


def wheel_raw_action_abs_mean(env, action_name: str = "wheel_joint_vel") -> torch.Tensor:
    """Mean absolute raw wheel action."""
    action = env.action_manager.get_term(action_name).raw_action
    return torch.mean(torch.abs(action), dim=1)


def wheel_target_vel_abs_mean(env, action_name: str = "wheel_joint_vel") -> torch.Tensor:
    """Mean absolute processed wheel velocity target."""
    term = env.action_manager.get_term(action_name)
    target = getattr(term, "_processed_actions")
    return torch.mean(torch.abs(target), dim=1)


def wheel_actual_vel_abs_mean(env, asset_cfg: SceneEntityCfg | None = None) -> torch.Tensor:
    """Mean absolute actual wheel joint velocity."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot", joint_names=(".*_wheel_joint",))
    asset = env.scene[asset_cfg.name]
    joint_ids = asset.find_joints(asset_cfg.joint_names)[0]
    return _finite(torch.mean(torch.abs(asset.data.joint_vel[:, joint_ids]), dim=1))


def wheel_target_actual_vel_error_mean(
    env,
    action_name: str = "wheel_joint_vel",
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Mean absolute error between processed wheel target and actual wheel velocity."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot", joint_names=(".*_wheel_joint",))
    term = env.action_manager.get_term(action_name)
    target = getattr(term, "_processed_actions")
    asset = env.scene[asset_cfg.name]
    joint_ids = asset.find_joints(asset_cfg.joint_names)[0]
    actual = asset.data.joint_vel[:, joint_ids]
    return _finite(torch.mean(torch.abs(target - actual), dim=1))


def wheel_actual_to_target_vel_ratio_mean(
    env,
    action_name: str = "wheel_joint_vel",
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Mean |actual wheel velocity| / |target wheel velocity|, clipped for readable logs."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot", joint_names=(".*_wheel_joint",))
    term = env.action_manager.get_term(action_name)
    target = getattr(term, "_processed_actions")
    asset = env.scene[asset_cfg.name]
    joint_ids = asset.find_joints(asset_cfg.joint_names)[0]
    actual = asset.data.joint_vel[:, joint_ids]
    ratio = torch.abs(actual) / torch.clamp(torch.abs(target), min=0.1)
    return _finite(torch.mean(torch.clamp(ratio, max=3.0), dim=1))


def wheel_target_actual_sign_agreement(
    env,
    action_name: str = "wheel_joint_vel",
    asset_cfg: SceneEntityCfg | None = None,
) -> torch.Tensor:
    """Fraction of wheel targets and actual velocities with matching sign."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot", joint_names=(".*_wheel_joint",))
    term = env.action_manager.get_term(action_name)
    target = getattr(term, "_processed_actions")
    asset = env.scene[asset_cfg.name]
    joint_ids = asset.find_joints(asset_cfg.joint_names)[0]
    actual = asset.data.joint_vel[:, joint_ids]
    active = torch.abs(target) > 0.1
    same_sign = torch.sign(target) == torch.sign(actual)
    return torch.sum((active & same_sign).float(), dim=1) / torch.clamp(torch.sum(active.float(), dim=1), min=1.0)


def upright_metric(env, asset_cfg=None) -> torch.Tensor:
    """1 means upright, 0 means fully inverted according to projected gravity."""
    if asset_cfg is None:
        asset_cfg = SceneEntityCfg("robot")
    asset = env.scene[asset_cfg.name]
    return torch.clamp(-asset.data.projected_gravity_b[:, 2], 0.0, 1.0)


def base_ground_contact_metric(env, sensor_name: str) -> torch.Tensor:
    """Per-step base contact flag for diagnosing reset-biased metrics."""
    sensor = env.scene[sensor_name]
    contact = sensor.data.found > 0
    while contact.ndim > 1:
        contact = torch.any(contact, dim=-1)
    return contact.float()
