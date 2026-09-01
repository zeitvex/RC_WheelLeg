"""Robot Environment Configurations for RL Locomotion Training.

This module defines the Manager-Based RL Environment Configurations for unitree robots
equipped with actuated wheels and leg joints. It handles sensors, actuators, command
generators, observation/critic terms, event randomizations, rewards, and terminations
for flat ground, rough terrains, and crawling tasks.
"""

import math
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp import dr as envs_dr
from mjlab.sim import SimulationCfg, MujocoCfg
from mjlab.managers.action_manager import ActionTermCfg
from mjlab.managers.command_manager import CommandTermCfg
from mjlab.managers.curriculum_manager import CurriculumTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.metrics_manager import MetricsTermCfg
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.scene import SceneCfg
from mjlab.sensor import (
    ContactMatch,
    ContactSensorCfg,
    ObjRef,
    RayCastSensorCfg,
    GridPatternCfg,
)
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity import mdp as velocity_mdp
from mjlab.terrains import (
    TerrainEntityCfg,
    TerrainGeneratorCfg,
    BoxFlatTerrainCfg,
    BoxPyramidStairsTerrainCfg,
    BoxInvertedPyramidStairsTerrainCfg,
    BoxRandomGridTerrainCfg,
    HfRandomUniformTerrainCfg,
    HfPerlinNoiseTerrainCfg,
    HfPyramidSlopedTerrainCfg,
)
from ..terrains import RCWallTerrainCfg, RCLowBarTerrainCfg
from mjlab.utils.noise import UniformNoiseCfg as Unoise
from mjlab.viewer import ViewerConfig

from ..robot_cfg import get_robot_cfg, get_robot_crawl_cfg, LEG_POS_SCALE, WHEEL_VEL_SCALE
from ..mdp.lowpass_actions import JointPositionDelayedLowPassActionCfg, JointVelocityDelayedLowPassActionCfg
from ..mdp.disturbances import apply_continuous_disturbance
from ..mdp.only_positive_rewards import enable_only_positive_rewards
from ..mdp.rewards import (
    track_linear_velocity,
    track_linear_velocity_l1,
    track_linear_velocity_x,
    track_linear_velocity_y,
    track_angular_velocity,
    track_angular_velocity_z,
    stair_lateral_yaw_drift_l2,
    base_height_l2,
    safe_base_lin_vel,
    safe_foot_contact,
    safe_height_scan,
    wheel_roll_tracking,
    adaptive_leg_motion_penalty,
    leg_symmetry,
    contact_fraction_reward,
    stand_still,
    hip_deviation,
    joint_deviation_l2,
    flat_orientation_l2,
    lin_vel_z_l2,
    crawl_height_reward,
    terrain_level_bonus,
    action_rate_curriculum_l2,
    variable_posture,
    joint_pos_penalty,
    joint_mirror,
    feet_contact_without_cmd,
    upright_roll_only,
    upward,
    joint_power,
    ang_vel_xy_l2,
    undesired_contacts,
    contact_forces,
    tracking_lin_vel_error,
    tracking_yaw_vel_error,
    tracking_lin_vel_x_error,
    tracking_lin_vel_y_error,
    tracking_lin_vel_along_command_error,
    actual_lin_vel_orthogonal_command_mean,
    command_lin_vel_mean,
    command_yaw_vel_abs_mean,
    actual_lin_vel_mean,
    tracking_lin_vel_error_band_mean,
    tracking_lin_vel_axis_error_band_mean,
    command_band_active,
    wheel_raw_action_abs_mean,
    wheel_target_vel_abs_mean,
    wheel_actual_vel_abs_mean,
    wheel_target_actual_vel_error_mean,
    wheel_actual_to_target_vel_ratio_mean,
    wheel_target_actual_sign_agreement,
    upright_metric,
    base_ground_contact_metric,
)
from ..mdp.curriculums import (
    command_axis_levels_vel,
    command_levels_adaptive,
    terrain_levels_obstacle_release,
    terrain_levels_ramp_strict,
    terrain_levels_vel_strict,
)
from ..mdp.commands import UniformThresholdVelocityCommandCfg

# Constant Definitions
WHEEL_NAMES = ("fl", "fr", "rl", "rr")


def _make_base_env_cfg() -> ManagerBasedRlEnvCfg:
    """Create the base environment configuration containing sensors, commands, and default policies."""
    
    # ------------------
    # Sensors Definition
    # ------------------
    feet_ground_cfg = ContactSensorCfg(
        name="feet_ground_contact",
        primary=ContactMatch(
            mode="body",
            pattern=tuple(f"{w}_wheel_Link" for w in WHEEL_NAMES),
            entity="robot",
        ),
        secondary=ContactMatch(mode="body", pattern="terrain"),
        fields=("found", "force"),
        reduce="none",
        num_slots=1,
        history_length=3,
        track_air_time=True,
    )

    base_ground_cfg = ContactSensorCfg(
        name="base_ground_contact",
        primary=ContactMatch(mode="body", pattern="base_link", entity="robot"),
        secondary=ContactMatch(mode="body", pattern="terrain"),
        fields=("found",),
        reduce="none",
        num_slots=1,
        history_length=4,
    )

    body_collision_cfg = ContactSensorCfg(
        name="body_collision",
        primary=ContactMatch(
            mode="body",
            pattern=(".*_hip_abduction_Link", ".*_hip_pitch_Link", ".*_knee_Link"),
            entity="robot",
        ),
        secondary=ContactMatch(mode="body", pattern="terrain"),
        fields=("found", "force"),
        reduce="none",
        num_slots=1,
        history_length=4,
    )

    terrain_scan = RayCastSensorCfg(
        name="height_scanner",
        frame=ObjRef(type="body", name="base_link", entity="robot"),
        pattern=GridPatternCfg(resolution=0.08, size=(1.6, 1.0)),
        ray_alignment="yaw",
        max_distance=5.0,
        exclude_parent_body=True,
        include_geom_groups=(0,),
        debug_vis=False,
    )

    # ------------------
    # Observations Setup
    # ------------------
    actor_terms = {
        "base_ang_vel": ObservationTermCfg(
            func=envs_mdp.base_ang_vel, scale=0.25,
            noise=Unoise(n_min=-0.2, n_max=0.2),
        ),
        "projected_gravity": ObservationTermCfg(
            func=velocity_mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
        ),
        "command": ObservationTermCfg(
            func=velocity_mdp.generated_commands,
            params={"command_name": "twist"},
        ),
        "joint_pos": ObservationTermCfg(
            func=envs_mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=(
                ".*_hip_abduction_joint", ".*_hip_pitch_joint", ".*_knee_joint",
            ))},
            noise=Unoise(n_min=-0.01, n_max=0.01),
        ),
        "joint_vel": ObservationTermCfg(
            func=envs_mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=(
                ".*_hip_abduction_joint", ".*_hip_pitch_joint", ".*_knee_joint",
            ))},
            scale=0.05, noise=Unoise(n_min=-1.5, n_max=1.5),
        ),
        "wheel_vel": ObservationTermCfg(
            func=envs_mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=(".*_wheel_joint",))},
            scale=0.05, noise=Unoise(n_min=-1.0, n_max=1.0),
        ),
        "actions": ObservationTermCfg(func=velocity_mdp.last_action),
    }

    critic_terms = {
        **actor_terms,
        "base_lin_vel": ObservationTermCfg(func=safe_base_lin_vel, scale=2.0),
        "foot_contact": ObservationTermCfg(
            func=safe_foot_contact, params={"sensor_name": "feet_ground_contact"},
        ),
        "height_scan": ObservationTermCfg(
            func=safe_height_scan, params={"sensor_name": "height_scanner"},
            clip=(-1.0, 1.0),
        ),
    }

    observations = {
        "actor": ObservationGroupCfg(
            terms=actor_terms, concatenate_terms=True,
            enable_corruption=True,
        ),
        "critic": ObservationGroupCfg(
            terms=critic_terms, concatenate_terms=True, enable_corruption=False,
        ),
    }

    # ------------------
    # Actions & Commands
    # ------------------
    actions: dict[str, ActionTermCfg] = {
        "leg_joint_pos": JointPositionDelayedLowPassActionCfg(
            entity_name="robot",
            actuator_names=(".*_hip_abduction_joint", ".*_hip_pitch_joint", ".*_knee_joint"),
            scale={".*_hip_abduction_joint": 0.125, "^(?!.*_hip_abduction_joint).*": 0.25}, use_default_offset=True,
            control_frequency=50.0, cut_off_frequency=5.0,
            min_delay=0, max_delay=2,
        ),
        "wheel_joint_vel": JointVelocityDelayedLowPassActionCfg(
            entity_name="robot", actuator_names=(".*_wheel_joint",),
            scale=5.0, offset=0.0, use_default_offset=False,
            control_frequency=50.0, cut_off_frequency=15.0,
            min_delay=0, max_delay=2,
        ),
    }

    commands: dict[str, CommandTermCfg] = {
        "twist": UniformThresholdVelocityCommandCfg(
            entity_name="robot", resampling_time_range=(10.0, 10.0),
            rel_standing_envs=0.15, rel_heading_envs=1.0, heading_command=True,
            heading_control_stiffness=0.6,
            rel_forward_envs=0.40,
            ranges=UniformThresholdVelocityCommandCfg.Ranges(
                lin_vel_x=(-1.0, 1.0), lin_vel_y=(-0.5, 0.5),
                ang_vel_z=(-1.0, 1.0), heading=(-math.pi, math.pi),
            ),
        )
    }

    # ------------------
    # Domain Randomization (Events)
    # ------------------
    events = {
        "reset_scene": EventTermCfg(func=envs_mdp.reset_scene_to_default, mode="reset"),
        "reset_base": EventTermCfg(
            func=envs_mdp.reset_root_state_uniform, mode="reset",
            params={
                "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-math.pi, math.pi)},
                "velocity_range": {
                    "x": (-0.5, 0.5),
                    "y": (-0.5, 0.5),
                    "z": (-0.5, 0.5),
                    "roll": (-0.5, 0.5),
                    "pitch": (-0.5, 0.5),
                    "yaw": (-0.5, 0.5),
                },
                "asset_cfg": SceneEntityCfg("robot"),
            },
        ),
        "push_robot": EventTermCfg(
            func=envs_mdp.push_by_setting_velocity, mode="interval",
            interval_range_s=(10.0, 15.0),
            params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}, "asset_cfg": SceneEntityCfg("robot")},
        ),
        "base_com": EventTermCfg(
            func=envs_dr.body_com_offset, mode="startup",
            params={"asset_cfg": SceneEntityCfg("robot", body_names=("base_link",)),
                    "operation": "add", "ranges": {0: (-0.05, 0.05), 1: (-0.05, 0.05), 2: (-0.05, 0.05)}},
        ),
        "body_friction": EventTermCfg(
            func=envs_dr.geom_friction, mode="startup",
            params={"asset_cfg": SceneEntityCfg("robot", geom_names=(".*",)), "operation": "abs", "ranges": (0.3, 1.0)},
        ),
        "actuator_stiffness": EventTermCfg(
            func=envs_dr.joint_stiffness, mode="startup",
            params={"asset_cfg": SceneEntityCfg("robot"), "ranges": (0.9, 1.1), "operation": "scale", "distribution": "log_uniform"},
        ),
        "actuator_damping": EventTermCfg(
            func=envs_dr.joint_damping, mode="startup",
            params={"asset_cfg": SceneEntityCfg("robot"), "ranges": (0.9, 1.1), "operation": "scale", "distribution": "log_uniform"},
        ),
        "body_mass_base": EventTermCfg(
            func=envs_dr.body_mass, mode="startup",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=("base_link",)),
                "operation": "add",
                "ranges": (-1.0, 3.0),
            },
        ),
    }

    # ------------------
    # Rewards Setup
    # ------------------
    rewards = {
        "track_lin_vel": RewardTermCfg(func=track_linear_velocity, weight=2.5, params={"std": 0.5, "command_name": "twist"}),
        "track_ang_vel": RewardTermCfg(func=track_angular_velocity, weight=2.5, params={"std": 0.5, "command_name": "twist"}),
        "upright": RewardTermCfg(func=velocity_mdp.upright, weight=1.0, params={"std": 0.5, "asset_cfg": SceneEntityCfg("robot", body_names=("base_link",))}),
        "base_height_l2": RewardTermCfg(func=base_height_l2, weight=-2.0, params={"target_height": 0.36}),
        "body_ang_vel": RewardTermCfg(func=velocity_mdp.body_angular_velocity_penalty, weight=-0.1, params={"asset_cfg": SceneEntityCfg("robot", body_names=("base_link",))}),
        "is_terminated": RewardTermCfg(func=envs_mdp.is_terminated, weight=-200.0),
        "joint_torques": RewardTermCfg(func=envs_mdp.joint_torques_l2, weight=-2.0e-4),
        "joint_acc": RewardTermCfg(func=envs_mdp.joint_acc_l2, weight=-2.5e-7),
        "action_rate": RewardTermCfg(func=envs_mdp.action_rate_l2, weight=-0.01),
        "joint_pos_limits": RewardTermCfg(func=envs_mdp.joint_pos_limits, weight=-10.0),
        "wheel_roll_tracking": RewardTermCfg(func=wheel_roll_tracking, weight=2.0, params={"command_name": "twist", "wheel_radius": 0.10, "wheel_track": 0.32, "std": 3.0, "asset_cfg": SceneEntityCfg("robot", joint_names=(".*_wheel_joint",))}),
        "wheel_contact_bonus": RewardTermCfg(func=contact_fraction_reward, weight=0.5, params={"sensor_name": "feet_ground_contact"}),
        "feet_air_time": RewardTermCfg(func=velocity_mdp.feet_air_time, weight=0.5, params={"sensor_name": "feet_ground_contact", "threshold_min": 0.1, "threshold_max": 0.5, "command_name": "twist", "command_threshold": 0.1}),
        "leg_motion_penalty": RewardTermCfg(func=adaptive_leg_motion_penalty, weight=-0.02, params={"command_name": "twist", "sensor_name": "feet_ground_contact", "command_threshold": 0.05, "tilt_relax_start": 0.08, "tilt_relax_end": 0.30, "contact_target": 0.85, "min_penalty_scale": 0.2, "asset_cfg": SceneEntityCfg("robot", joint_names=(".*_hip_abduction_joint", ".*_hip_pitch_joint", ".*_knee_joint"))}),
        "stand_still": RewardTermCfg(func=stand_still, weight=-0.2, params={"command_name": "twist", "command_threshold": 0.1}),
        "body_collision": RewardTermCfg(func=velocity_mdp.self_collision_cost, weight=-1.0, params={"sensor_name": "body_collision"}),
    }

    # ------------------
    # Terminations
    # ------------------
    terminations = {
        "time_out": TerminationTermCfg(func=envs_mdp.time_out, time_out=True),
        "bad_orientation": TerminationTermCfg(func=envs_mdp.bad_orientation, params={"limit_angle": 1.0}),
        "base_ground_contact": TerminationTermCfg(func=velocity_mdp.illegal_contact, params={"sensor_name": "base_ground_contact"}),
        "nan_detection": TerminationTermCfg(func=envs_mdp.nan_detection),
    }

    curriculum: dict[str, CurriculumTermCfg] = {}

    metrics = {"mean_leg_action_acc": MetricsTermCfg(func=velocity_mdp.mean_action_acc)}

    return ManagerBasedRlEnvCfg(
        scene=SceneCfg(
            num_envs=2048, env_spacing=2.5,
            terrain=TerrainEntityCfg(terrain_type="generator", terrain_generator=TerrainGeneratorCfg(
                size=(8.0, 8.0), border_width=20.0, num_rows=10, num_cols=20,
                sub_terrains={"flat": BoxFlatTerrainCfg(proportion=1.0)},
            )),
            sensors=(feet_ground_cfg, base_ground_cfg, body_collision_cfg, terrain_scan),
        ),
        commands=commands, actions=actions, observations=observations,
        rewards=rewards, terminations=terminations, events=events,
        metrics=metrics, curriculum=curriculum, decimation=4, episode_length_s=20.0,
        sim=SimulationCfg(mujoco=MujocoCfg(timestep=0.005, impratio=100, cone="elliptic")),
        viewer=ViewerConfig(body_name="base_link", distance=3.0, elevation=-20.0, azimuth=45.0),
    )


def flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
    """Flat ground training and evaluation configuration."""
    cfg = _make_base_env_cfg()
    cfg.scene.entities = {"robot": get_robot_cfg()}
    if play:
        cfg.episode_length_s = int(1e9)
        cfg.observations["actor"].enable_corruption = False
        cfg.events.pop("push_robot", None)
        cfg.curriculum = {}
    return cfg


def rough_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
    """Rough terrains configuration for general wheel-legged navigation."""
    enable_only_positive_rewards()

    cfg = _make_base_env_cfg()
    cfg.scene.entities = {"robot": get_robot_cfg()}

    # ------------------
    # Terrain Generator & Curriculum
    # ------------------
    cfg.scene.terrain = TerrainEntityCfg(
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            size=(8.0, 8.0), border_width=20.0, num_rows=10, num_cols=20, curriculum=True,
            sub_terrains={
                "flat": BoxFlatTerrainCfg(proportion=0.15, size=(8.0, 8.0)),
                "pyramid_stairs": BoxPyramidStairsTerrainCfg(proportion=0.05, step_height_range=(0.0, 0.20), step_width=0.30, size=(8.0, 8.0)),
                "pyramid_stairs_inv": BoxInvertedPyramidStairsTerrainCfg(proportion=0.35, step_height_range=(0.0, 0.20), step_width=0.30, size=(8.0, 8.0)),
                "random_grid": BoxRandomGridTerrainCfg(proportion=0.27, grid_width=0.45, grid_height_range=(0.0, 0.20), size=(8.0, 8.0)),
                "random_rough": HfRandomUniformTerrainCfg(proportion=0.01, noise_range=(0.0, 0.06), noise_step=0.01, horizontal_scale=0.20, downsampled_scale=0.20, border_width=0.25, base_thickness_ratio=100.0, size=(8.0, 8.0)),
                "perlin_noise": HfPerlinNoiseTerrainCfg(proportion=0.01, height_range=(0.0, 0.06), octaves=2, persistence=0.4, lacunarity=2.0, horizontal_scale=0.20, resolution=0.20, border_width=0.50, base_thickness_ratio=100.0, size=(8.0, 8.0)),
                "rc_wall": RCWallTerrainCfg(
                    proportion=0.15,
                    wall_height_range=(0.10, 0.35),
                    wall_centers_x=(2.1, 3.2, 4.3, 5.4, 6.5),
                    size=(8.0, 8.0),
                ),
                "sloped_terrain": HfPyramidSlopedTerrainCfg(proportion=0.01, slope_range=(0.052, 0.325), platform_width=2.0, border_width=0.25, base_thickness_ratio=100.0, horizontal_scale=0.20, size=(8.0, 8.0)),
            },
        ),
        max_init_terrain_level=5,
    )

    # Keep the custom terrain set, but align command/curriculum behavior with go2w rough.
    cfg.curriculum.pop("command_vel", None)
    cfg.curriculum["terrain_levels"] = CurriculumTermCfg(
        func=terrain_levels_obstacle_release,
        params={
            "command_name": "twist",
            "initial_terrain_names": ("flat", "random_rough", "perlin_noise", "sloped_terrain", "pyramid_stairs"),
            "release_schedule": (
                (200 * 24, ("random_grid",)),
                (500 * 24, ("pyramid_stairs_inv",)),
                (700 * 24, ("rc_wall",)),
            ),
        },
    )
    cfg.curriculum["command_x_levels"] = CurriculumTermCfg(
        func=command_levels_adaptive,
        params={
            "command_name": "twist",
            "reward_term_name": "track_lin_vel_x_exp",
            "axis": "x",
            "initial_range": (-0.5, 0.5),
            "delta_command": 0.05,
            "target_ratio": 0.8,
            "ema_alpha": 0.5,
        },
    )
    cfg.curriculum["command_y_levels"] = CurriculumTermCfg(
        func=command_levels_adaptive,
        params={
            "command_name": "twist",
            "reward_term_name": "track_lin_vel_y_exp",
            "axis": "y",
            "initial_range": (-0.5, 0.5),
            "delta_command": 0.05,
            "target_ratio": 0.8,
            "ema_alpha": 0.5,
        },
    )
    cfg.curriculum["command_yaw_levels"] = CurriculumTermCfg(
        func=command_levels_adaptive,
        params={
            "command_name": "twist",
            "reward_term_name": "track_ang_vel_z_exp",
            "axis": "yaw",
            "initial_range": (-0.5, 0.5),
            "delta_command": 0.05,
            "target_ratio": 0.8,
            "ema_alpha": 0.5,
        },
    )

    cfg.commands["twist"].heading_command = True
    cfg.commands["twist"].rel_heading_envs = 1.0
    cfg.commands["twist"].heading_control_stiffness = 0.5
    cfg.commands["twist"].ranges.heading = (-math.pi, math.pi)
    cfg.commands["twist"].rel_standing_envs = 0.02
    cfg.commands["twist"].rel_forward_envs = 0.30
    cfg.commands["twist"].rel_lateral_envs = 0.20
    cfg.commands["twist"].rel_yaw_envs = 0.20
    cfg.commands["twist"].ranges.lin_vel_x = (-1.0, 1.0)
    cfg.commands["twist"].ranges.lin_vel_y = (-1.0, 1.0)
    cfg.commands["twist"].ranges.ang_vel_z = (-1.0, 1.0)

    # ------------------
    # Startup & Reset Randomizations
    # ------------------
    cfg.events.pop("joint_friction", None)
    cfg.events["reset_joints"] = EventTermCfg(func=envs_mdp.reset_joints_by_offset, mode="reset", params={"position_range": (0.0, 0.0), "velocity_range": (0.0, 0.0), "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",))})
    
    cfg.events["reset_base"] = EventTermCfg(
        func=envs_mdp.reset_root_state_uniform, mode="reset",
        params={
            "pose_range": {"z": (0.42, 0.42), "yaw": (-math.pi, math.pi)},
            "velocity_range": {"x": (-0.2, 0.2), "y": (-0.1, 0.1), "yaw": (-0.2, 0.2)},
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )
    cfg.events["push_robot"] = EventTermCfg(
        func=envs_mdp.push_by_setting_velocity, mode="interval",
        interval_range_s=(5.0, 10.0),
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}, "asset_cfg": SceneEntityCfg("robot")},
    )

    # ------------------
    # Rewards Integration
    # ------------------
    cfg.rewards.pop("track_lin_vel", None)
    cfg.rewards.pop("track_ang_vel", None)
    cfg.rewards["track_lin_vel_x_exp"] = RewardTermCfg(
        func=track_linear_velocity_x,
        weight=1.0,
        params={"std": 0.25, "command_name": "twist"},
    )
    cfg.rewards["track_lin_vel_y_exp"] = RewardTermCfg(
        func=track_linear_velocity_y,
        weight=1.0,
        params={"std": 0.25, "command_name": "twist"},
    )
    cfg.rewards["track_ang_vel_z_exp"] = RewardTermCfg(
        func=track_angular_velocity_z,
        weight=1.0,
        params={"std": 0.25, "command_name": "twist"},
    )
    cfg.rewards["stair_lateral_yaw_drift"] = RewardTermCfg(
        func=stair_lateral_yaw_drift_l2,
        weight=-1.0,
        params={
            "terrain_names": ("pyramid_stairs", "pyramid_stairs_inv", "random_grid"),
            "y_scale": 1.0,
            "yaw_scale": 1.0,
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )
    
    cfg.rewards["lin_vel_z"] = RewardTermCfg(func=lin_vel_z_l2, weight=-2.0)
    cfg.rewards["ang_vel_xy"] = RewardTermCfg(func=ang_vel_xy_l2, weight=-0.05, params={"asset_cfg": SceneEntityCfg("robot")})

    cfg.rewards.pop("upright", None)
    cfg.rewards.pop("roll_penalty", None)

    # 🌟 限制俯仰角死区（Pitch Dead-zone）：允许正常爬坡时有最大 29 度（0.50 rad）的仰角，但严厉惩罚超过该仰角的“前轮悬空暴冲/后翻”
    # 动态课程奖励与动作惩罚衰减
    cfg.rewards.pop("terrain_level_bonus", None)
    cfg.rewards.pop("action_rate_curriculum", None)
    cfg.rewards["action_rate"].weight = -0.01

    cfg.rewards["joint_torques"].weight = -2.5e-5
    cfg.rewards["joint_power"] = RewardTermCfg(func=joint_power, weight=-2.0e-5)
    cfg.rewards.pop("joint_acc", None)
    cfg.rewards["leg_joint_acc_l2"] = RewardTermCfg(func=envs_mdp.joint_acc_l2, weight=-2.5e-7, params={"asset_cfg": SceneEntityCfg("robot", joint_names=(".*_hip_abduction_joint", ".*_hip_pitch_joint", ".*_knee_joint"))})
    cfg.rewards["wheel_joint_acc_l2"] = RewardTermCfg(func=envs_mdp.joint_acc_l2, weight=-2.5e-9, params={"asset_cfg": SceneEntityCfg("robot", joint_names=(".*_wheel_joint",))})


    cfg.rewards["joint_pos_limits"].weight = -5.0
    cfg.rewards.pop("leg_motion_penalty", None)
    cfg.rewards["is_terminated"].weight = 0.0
    cfg.rewards.pop("leg_symmetry", None)
    cfg.rewards["joint_mirror"] = RewardTermCfg(
        func=joint_mirror, 
        weight=-0.05, 
        params={
            "mirror_joints": [
                ["fl_(hip_pitch|knee)_joint", "rr_(hip_pitch|knee)_joint"],
                ["fr_(hip_pitch|knee)_joint", "rl_(hip_pitch|knee)_joint"]
            ], 
            "asset_cfg": SceneEntityCfg("robot")
        }
    )

    # 移除 variable_posture 及其产生的静止奖励陷阱，换用极轻微的偏离惩罚
    cfg.rewards.pop("stand_still", None)
    cfg.rewards["stand_still"] = RewardTermCfg(func=stand_still, weight=-2.0, params={"command_name": "twist", "command_threshold": 0.1})

    cfg.rewards.pop("hip_deviation", None)
    cfg.rewards.pop("variable_posture", None)

    cfg.rewards.pop("joint_deviation_l2", None)

    # 针对 ab 关节施加较严厉的惩罚，防止在 yaw 时乱撇腿
    cfg.rewards["joint_pos_penalty_ab"] = RewardTermCfg(
        func=joint_pos_penalty,
        weight=-1.0,
        params={
            "stand_still_scale": 5.0,
            "velocity_threshold": 0.5,
            "command_threshold": 0.1,
            "asset_cfg": SceneEntityCfg("robot", joint_names=(".*_hip_abduction_joint",)),
            "command_name": "twist"
        }
    )

    # 针对 pitch 和 knee 关节施加较宽松的惩罚，保留跨越障碍的抬腿自由度
    cfg.rewards["joint_pos_penalty_sagittal"] = RewardTermCfg(
        func=joint_pos_penalty,
        weight=-0.3,
        params={
            "stand_still_scale": 5.0,
            "velocity_threshold": 0.5,
            "command_threshold": 0.1,
            "asset_cfg": SceneEntityCfg("robot", joint_names=(".*_hip_pitch_joint", ".*_knee_joint")),
            "command_name": "twist"
        }
    )

    # 🌟 强力约束同侧外展关节平行对称，消除转向时的前后剪刀式摆动
    cfg.rewards["abduction_mirror"] = RewardTermCfg(
        func=joint_mirror,
        weight=-0.5,  # 施加合理惩罚，限制前后腿同侧外展关节反向运动
        params={
            "mirror_joints": [
                ["fl_hip_abduction_joint", "rl_hip_abduction_joint"],
                ["fr_hip_abduction_joint", "rr_hip_abduction_joint"]
            ],
            "asset_cfg": SceneEntityCfg("robot")
        }
    )

    cfg.rewards["feet_contact_without_cmd"] = RewardTermCfg(
        func=feet_contact_without_cmd,
        weight=0.1,
        params={"command_name": "twist", "sensor_name": "feet_ground_contact"}
    )
    cfg.rewards["feet_air_time"].weight = 0.15
    cfg.rewards["upward"] = RewardTermCfg(func=upward, weight=0.5)
    
    cfg.rewards["base_height_l2"].weight = 0.0
    cfg.rewards["base_height_l2"].params["target_height"] = 0.42
    cfg.rewards["base_height_l2"].params["sensor_cfg"] = SceneEntityCfg("height_scanner")

    # 恢复机身碰撞惩罚为-1.0，逼迫机器人高抬腿跨越障碍，防止拖地
    cfg.rewards.pop("body_collision", None)
    cfg.rewards["undesired_contacts"] = RewardTermCfg(func=undesired_contacts, weight=-1.0, params={"sensor_name": "body_collision", "threshold": 1.0})
    cfg.rewards["contact_forces"] = RewardTermCfg(func=contact_forces, weight=-1.5e-4, params={"sensor_name": "feet_ground_contact", "threshold": 100.0})

    # 🌟 严厉惩罚机身/胸部碰撞（防止硬撞高墙），逼迫机器人学会用前轮触墙并主动抬腿攀爬的“触觉反射”
    # 彻底移除机身俯仰约束，允许机器人抬头爬高?    cfg.rewards.pop("flat_orientation", None)

    # Remove non-applicable rewards
    for key in ("wheel_roll_tracking", "wheel_contact_bonus", "body_ang_vel", "terrain_level_bonus", "action_rate_curriculum"):
        cfg.rewards.pop(key, None)

    cfg.episode_length_s = 20.0
    cfg.sim = SimulationCfg(contact_sensor_maxmatch=128, mujoco=MujocoCfg(timestep=0.005, impratio=100, cone="elliptic", ccd_iterations=80))

    # 移除 orientation 终止，允许机器人翻倒以学习回复
    cfg.seed = 42
    if cfg.scene.terrain is not None:
        cfg.scene.terrain.num_envs = 2048
        cfg.scene.terrain.env_spacing = 2.5

    cfg.metrics.update(
        {
            "tracking_lin_vel_error": MetricsTermCfg(func=tracking_lin_vel_error, params={"command_name": "twist"}),
            "tracking_lin_vel_x_error": MetricsTermCfg(func=tracking_lin_vel_x_error, params={"command_name": "twist"}),
            "tracking_lin_vel_y_error": MetricsTermCfg(func=tracking_lin_vel_y_error, params={"command_name": "twist"}),
            "tracking_lin_vel_along_cmd_error": MetricsTermCfg(
                func=tracking_lin_vel_along_command_error, params={"command_name": "twist"}
            ),
            "actual_lin_vel_orthogonal_cmd": MetricsTermCfg(
                func=actual_lin_vel_orthogonal_command_mean, params={"command_name": "twist"}
            ),
            "tracking_yaw_vel_error": MetricsTermCfg(func=tracking_yaw_vel_error, params={"command_name": "twist"}),
            "cmd_lin_vel": MetricsTermCfg(func=command_lin_vel_mean, params={"command_name": "twist"}),
            "cmd_yaw_vel": MetricsTermCfg(func=command_yaw_vel_abs_mean, params={"command_name": "twist"}),
            "actual_lin_vel": MetricsTermCfg(func=actual_lin_vel_mean),
            "tracking_lin_vel_error_cmd_0_03": MetricsTermCfg(
                func=tracking_lin_vel_error_band_mean,
                params={"command_name": "twist", "min_speed": 0.0, "max_speed": 0.3},
            ),
            "tracking_lin_vel_error_cmd_03_07": MetricsTermCfg(
                func=tracking_lin_vel_error_band_mean,
                params={"command_name": "twist", "min_speed": 0.3, "max_speed": 0.7},
            ),
            "tracking_lin_vel_error_cmd_07_up": MetricsTermCfg(
                func=tracking_lin_vel_error_band_mean,
                params={"command_name": "twist", "min_speed": 0.7, "max_speed": 10.0},
            ),
            "tracking_lin_vel_x_error_cmd_0_03": MetricsTermCfg(
                func=tracking_lin_vel_axis_error_band_mean,
                params={"axis": 0, "command_name": "twist", "min_speed": 0.0, "max_speed": 0.3},
            ),
            "tracking_lin_vel_x_error_cmd_03_07": MetricsTermCfg(
                func=tracking_lin_vel_axis_error_band_mean,
                params={"axis": 0, "command_name": "twist", "min_speed": 0.3, "max_speed": 0.7},
            ),
            "tracking_lin_vel_x_error_cmd_07_up": MetricsTermCfg(
                func=tracking_lin_vel_axis_error_band_mean,
                params={"axis": 0, "command_name": "twist", "min_speed": 0.7, "max_speed": 10.0},
            ),
            "tracking_lin_vel_y_error_cmd_0_03": MetricsTermCfg(
                func=tracking_lin_vel_axis_error_band_mean,
                params={"axis": 1, "command_name": "twist", "min_speed": 0.0, "max_speed": 0.3},
            ),
            "tracking_lin_vel_y_error_cmd_03_07": MetricsTermCfg(
                func=tracking_lin_vel_axis_error_band_mean,
                params={"axis": 1, "command_name": "twist", "min_speed": 0.3, "max_speed": 0.7},
            ),
            "tracking_lin_vel_y_error_cmd_07_up": MetricsTermCfg(
                func=tracking_lin_vel_axis_error_band_mean,
                params={"axis": 1, "command_name": "twist", "min_speed": 0.7, "max_speed": 10.0},
            ),
            "cmd_band_0_03": MetricsTermCfg(
                func=command_band_active, params={"command_name": "twist", "min_speed": 0.0, "max_speed": 0.3}
            ),
            "cmd_band_03_07": MetricsTermCfg(
                func=command_band_active, params={"command_name": "twist", "min_speed": 0.3, "max_speed": 0.7}
            ),
            "cmd_band_07_up": MetricsTermCfg(
                func=command_band_active, params={"command_name": "twist", "min_speed": 0.7, "max_speed": 10.0}
            ),
            "wheel_raw_action_abs": MetricsTermCfg(func=wheel_raw_action_abs_mean),
            "wheel_target_vel_abs": MetricsTermCfg(func=wheel_target_vel_abs_mean),
            "wheel_actual_vel_abs": MetricsTermCfg(func=wheel_actual_vel_abs_mean),
            "wheel_target_actual_vel_error": MetricsTermCfg(func=wheel_target_actual_vel_error_mean),
            "wheel_actual_to_target_vel_ratio": MetricsTermCfg(func=wheel_actual_to_target_vel_ratio_mean),
            "wheel_target_actual_sign_agreement": MetricsTermCfg(func=wheel_target_actual_sign_agreement),
            "upright": MetricsTermCfg(func=upright_metric),
            "base_ground_contact_rate": MetricsTermCfg(
                func=base_ground_contact_metric, params={"sensor_name": "base_ground_contact"}
            ),
        }
    )

    if play:
        cfg.episode_length_s = int(1e9)
        cfg.observations["actor"].enable_corruption = False
        cfg.events.pop("push_robot", None)
        cfg.curriculum = {}
        if cfg.scene.terrain is not None and cfg.scene.terrain.terrain_generator is not None:
            cfg.scene.terrain.terrain_generator.curriculum = False
            cfg.scene.terrain.terrain_generator.num_cols = 5
            cfg.scene.terrain.terrain_generator.num_rows = 5
            cfg.scene.terrain.terrain_generator.border_width = 10.0

    return cfg


def crawl_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
    """Crawling and ducking configuration for climbing obstacles under low clearing heights."""
    enable_only_positive_rewards()

    cfg = _make_base_env_cfg()
    cfg.scene.entities = {"robot": get_robot_crawl_cfg()}

    # ------------------
    # Terrain Definition
    # ------------------
    cfg.scene.terrain = TerrainEntityCfg(
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            size=(8.0, 8.0), border_width=20.0, num_rows=10, num_cols=20, curriculum=True,
            sub_terrains={
                "flat": BoxFlatTerrainCfg(proportion=0.25),
                "rc_low_bar": RCLowBarTerrainCfg(proportion=0.35, clearance_range=(0.24, 0.32)),
                "random_grid": BoxRandomGridTerrainCfg(proportion=0.20, grid_width=0.45, grid_height_range=(0.0, 0.05)),
                "perlin_noise": HfPerlinNoiseTerrainCfg(proportion=0.20, height_range=(0.0, 0.05), octaves=2, persistence=0.4, lacunarity=2.0, horizontal_scale=0.20, resolution=0.20, border_width=0.50, base_thickness_ratio=100.0),
            },
        ),
        max_init_terrain_level=0,
    )

    # Disable command vel curriculum and setup base command ranges
    cfg.curriculum.pop("command_vel", None)
    cfg.curriculum["terrain_levels"] = CurriculumTermCfg(func=velocity_mdp.terrain_levels_vel, params={"command_name": "twist"})

    cfg.commands["twist"].heading_command = False
    cfg.commands["twist"].rel_heading_envs = 0.0
    cfg.commands["twist"].ranges.heading = None
    cfg.commands["twist"].rel_standing_envs = 0.05   # 减少静止比例（匍匐需持续运动�?    cfg.commands["twist"].rel_forward_envs = 0.40    # 40% 纯前向（低杆地形全是直穿�?
    # ------------------
    # Events & Reset
    # ------------------
    cfg.events["joint_friction"] = EventTermCfg(func=envs_dr.joint_friction, mode="startup", params={"asset_cfg": SceneEntityCfg("robot"), "ranges": (0.7, 1.3), "operation": "scale"})
    cfg.events["reset_joints"] = EventTermCfg(func=envs_mdp.reset_joints_by_offset, mode="reset", params={"position_range": (0.0, 0.1), "velocity_range": (0.0, 0.0), "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",))})
    
    cfg.events["reset_base"] = EventTermCfg(
        func=envs_mdp.reset_root_state_uniform, mode="reset",
        params={
            "pose_range": {"z": (0.00, 0.04), "yaw": (-math.pi, math.pi)},
            "velocity_range": {"x": (-0.1, 0.1), "y": (-0.05, 0.05), "yaw": (-0.1, 0.1)},
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )
    cfg.events["push_robot"] = EventTermCfg(
        func=envs_mdp.push_by_setting_velocity, mode="interval",
        interval_range_s=(5.0, 10.0),
        params={"velocity_range": {"x": (-0.3, 0.3), "y": (-0.3, 0.3)}, "asset_cfg": SceneEntityCfg("robot")},
    )

    # ------------------
    # Rewards Integration
    # ------------------
    cfg.rewards["track_lin_vel"].weight = 3.0
    cfg.rewards["track_lin_vel"].params["std"] = 0.5
    cfg.rewards["track_ang_vel"].weight = 1.5
    cfg.rewards["track_ang_vel"].params["std"] = 0.5
    
    cfg.rewards["lin_vel_z"] = RewardTermCfg(func=lin_vel_z_l2, weight=-1.0)
    cfg.rewards["ang_vel_xy"] = RewardTermCfg(func=velocity_mdp.body_angular_velocity_penalty, weight=-0.05, params={"asset_cfg": SceneEntityCfg("robot", body_names=("base_link",))})
    
    cfg.rewards["upright"].weight = 1.0
    cfg.rewards["upright"].params["std"] = 0.5
    cfg.rewards["action_rate"].weight = -0.001
    cfg.rewards["joint_torques"].weight = -1e-4
    cfg.rewards["joint_acc"].weight = 0.0
    cfg.rewards["joint_pos_limits"].weight = -1.0
    cfg.rewards["leg_motion_penalty"].weight = -5.0
    cfg.rewards["is_terminated"].weight = -50.0

    # Under-crawling height reward: maximum bonus when body stays under 0.22m
    cfg.rewards.pop("base_height_l2", None)
    cfg.rewards["crawl_height_reward"] = RewardTermCfg(
        func=crawl_height_reward,
        weight=1.5,
        params={"target_height": 0.22, "std": 0.05}
    )

    cfg.rewards.pop("stand_still", None)
    
    cfg.rewards.pop("hip_deviation", None)
    cfg.rewards["leg_joint_deviation"] = RewardTermCfg(
        func=joint_deviation_l2,
        weight=-15.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=(".*_hip_abduction_joint", ".*_hip_pitch_joint", ".*_knee_joint"))},
    )
    
    # 🌟 强力引入轮子滚动跟踪奖励，引导机器人完全依靠轮子的正向滚动进行平地/低矮处的推进
    cfg.rewards["wheel_roll_tracking"] = RewardTermCfg(
        func=wheel_roll_tracking, 
        weight=4.0, 
        params={
            "command_name": "twist", 
            "wheel_radius": 0.10, 
            "wheel_track": 0.32, 
            "std": 3.0, 
            "asset_cfg": SceneEntityCfg("robot", joint_names=(".*_wheel_joint",))
        }
    )

    if "body_collision" in cfg.rewards:
        cfg.rewards["body_collision"].weight = -0.1

    cfg.rewards["flat_orientation"] = RewardTermCfg(func=flat_orientation_l2, weight=-1.0)

    for key in ("wheel_roll_tracking", "feet_air_time", "wheel_contact_bonus", "body_ang_vel"):
        cfg.rewards.pop(key, None)

    cfg.episode_length_s = 30.0
    cfg.sim = SimulationCfg(contact_sensor_maxmatch=128, mujoco=MujocoCfg(timestep=0.005, impratio=100, cone="elliptic", ccd_iterations=80))

    # Loosen orientation bad threshold to 80 degrees for steep crawling tilts
    cfg.terminations["bad_orientation"].params["limit_angle"] = math.radians(80.0)
    
    # Remove base ground contact termination to facilitate crawl under bars
    cfg.terminations.pop("base_ground_contact", None)

    if play:
        cfg.episode_length_s = int(1e9)
        cfg.observations["actor"].enable_corruption = False
        cfg.events.pop("push_robot", None)
        cfg.curriculum = {}
        if cfg.scene.terrain is not None and cfg.scene.terrain.terrain_generator is not None:
            cfg.scene.terrain.terrain_generator.curriculum = False
            cfg.scene.terrain.terrain_generator.num_cols = 5
            cfg.scene.terrain.terrain_generator.num_rows = 5
            cfg.scene.terrain.terrain_generator.border_width = 10.0

    return cfg
