from __future__ import annotations

import torch
from dataclasses import dataclass
from typing import TYPE_CHECKING

from mjlab.tasks.velocity.mdp.velocity_command import UniformVelocityCommand, UniformVelocityCommandCfg

if TYPE_CHECKING:
    from mjlab.envs.manager_based_rl_env import ManagerBasedRlEnv


class UniformThresholdVelocityCommand(UniformVelocityCommand):
    """带死区过滤、高速侧向解耦以及地形自适应速度约束的速度命令生成器。
    
    在楼梯和垂直短墙等障碍地形上，自动将 y 轴（横移）和 z 轴（偏航）控制约束置 0.0，
    强制机器人心无旁骛笔直冲锋越障，有效杜绝打滑和偏航翻滚跌落；而在平地、斜坡等普通地形上
    放开多向混合采样，训练机动转向能力。
    """
    cfg: UniformThresholdVelocityCommandCfg

    def __init__(self, cfg: UniformThresholdVelocityCommandCfg, env: ManagerBasedRlEnv):
        super().__init__(cfg, env)
        self.is_lateral_env = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.is_yaw_env = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self.was_climbing = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        # 缓存地形类型的索引（台阶、反向台阶、垂直短墙），实现高容错动态查找
        self._climbing_indices = []
        self._flat_index = -1
        terrain = getattr(self._env.scene, "terrain", None)
        if terrain is not None and getattr(terrain.cfg, "terrain_generator", None) is not None:
            sub_terrain_names = list(terrain.cfg.terrain_generator.sub_terrains.keys())
            for name in ["pyramid_stairs", "pyramid_stairs_inv", "rc_wall"]:
                if name in sub_terrain_names:
                    self._climbing_indices.append(sub_terrain_names.index(name))
            if "flat" in sub_terrain_names:
                self._flat_index = sub_terrain_names.index("flat")

    def _resample_command(self, env_ids: torch.Tensor) -> None:
        # 1. 调用基类的标准采样
        super()._resample_command(env_ids)
        
        # 2. 基础死区过滤：低于 0.2 m/s 时强制归零，区分静止和运动
        cmd_xy_norm = torch.norm(self.vel_command_b[env_ids, :2], dim=1)
        small_cmd_mask = cmd_xy_norm < 0.2
        small_cmd_ids = env_ids[small_cmd_mask]
        if len(small_cmd_ids) > 0:
            self.vel_command_b[small_cmd_ids, :] = 0.0
            self.vel_command_w[small_cmd_ids, :] = 0.0

        # 重置并采样单 Y（只横移）与单 Z（只原地转向）指令分布
        self.is_lateral_env[env_ids] = False
        self.is_yaw_env[env_ids] = False

        # 对没有设为前进且没有静止的激活环境进行单轴采样
        active_non_fwd_mask = (~self.is_forward_env[env_ids]) & (~self.is_standing_env[env_ids])
        active_non_fwd_ids = env_ids[active_non_fwd_mask]

        if len(active_non_fwd_ids) > 0:
            r = torch.empty(len(active_non_fwd_ids), device=self.device)
            # 采样单 Y 占比
            self.is_lateral_env[active_non_fwd_ids] = r.uniform_(0.0, 1.0) <= self.cfg.rel_lateral_envs
            lat_ids = active_non_fwd_ids[self.is_lateral_env[active_non_fwd_ids]]
            if len(lat_ids) > 0:
                self.vel_command_b[lat_ids, 0] = 0.0
                y_signs = self.vel_command_b[lat_ids, 1].sign()
                y_signs[y_signs == 0] = 1.0
                self.vel_command_b[lat_ids, 1] = y_signs * self.vel_command_b[lat_ids, 1].abs().clamp(min=0.3)
                self.vel_command_b[lat_ids, 2] = 0.0

            # 对不是 forward 也不是 lateral 的环境采样单 Z 占比
            non_lat_mask = ~self.is_lateral_env[active_non_fwd_ids]
            non_lat_ids = active_non_fwd_ids[non_lat_mask]

            if len(non_lat_ids) > 0:
                r_yaw = torch.empty(len(non_lat_ids), device=self.device)
                self.is_yaw_env[non_lat_ids] = r_yaw.uniform_(0.0, 1.0) <= self.cfg.rel_yaw_envs
                yaw_ids = non_lat_ids[self.is_yaw_env[non_lat_ids]]
                if len(yaw_ids) > 0:
                    self.vel_command_b[yaw_ids, 0] = 0.0
                    self.vel_command_b[yaw_ids, 1] = 0.0
                    z_signs = self.vel_command_b[yaw_ids, 2].sign()
                    z_signs[z_signs == 0] = 1.0
                    self.vel_command_b[yaw_ids, 2] = z_signs * self.vel_command_b[yaw_ids, 2].abs().clamp(min=0.3)

        # 3. 地形自适应重采样限制：若在爬行地形，强制纯前进方向且速度 >= 0.3 m/s
        terrain = getattr(self._env.scene, "terrain", None)
        terrain_types = getattr(terrain, "terrain_types", None)
        if terrain_types is not None and len(self._climbing_indices) > 0:
            is_climbing = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
            for idx in self._climbing_indices:
                is_climbing |= (terrain_types == idx)
                
            climbing_env_ids = env_ids[is_climbing[env_ids]]
            if len(climbing_env_ids) > 0:
                # 强制 x 方向为正（向前越障），y 和 z 轴指令清零
                self.vel_command_b[climbing_env_ids, 0] = self.vel_command_b[climbing_env_ids, 0].abs().clamp(min=0.3)
                self.vel_command_b[climbing_env_ids, 1] = 0.0
                self.vel_command_b[climbing_env_ids, 2] = 0.0
                self.is_heading_env[climbing_env_ids] = True

        # 4. 在平地（flat）地形上生成 35% 纯 Y 和 35% 纯 Z 指令，排除静止环境
        if terrain_types is not None and self._flat_index != -1:
            is_flat = terrain_types == self._flat_index
            flat_env_ids = env_ids[is_flat[env_ids]]
            if len(flat_env_ids) > 0:
                active_flat_mask = ~self.is_standing_env[flat_env_ids]
                active_flat_ids = flat_env_ids[active_flat_mask]
                if len(active_flat_ids) > 0:
                    r = torch.empty(len(active_flat_ids), device=self.device).uniform_(0.0, 1.0)
                    y_only_mask = r < 0.35
                    z_only_mask = (r >= 0.35) & (r < 0.70)

                    y_only_ids = active_flat_ids[y_only_mask]
                    if len(y_only_ids) > 0:
                        self.vel_command_b[y_only_ids, 0] = 0.0  # x = 0
                        self.vel_command_b[y_only_ids, 2] = 0.0  # yaw = 0
                        if self.cfg.heading_command:
                            self.heading_target[y_only_ids] = self.robot.data.heading_w[y_only_ids]
                            self.is_heading_env[y_only_ids] = True

                    z_only_ids = active_flat_ids[z_only_mask]
                    if len(z_only_ids) > 0:
                        self.vel_command_b[z_only_ids, 0] = 0.0  # x = 0
                        self.vel_command_b[z_only_ids, 1] = 0.0  # y = 0
                        if self.cfg.heading_command:
                            yaw_delta = torch.empty(len(z_only_ids), device=self.device).uniform_(
                                -self.cfg.yaw_only_heading_range, self.cfg.yaw_only_heading_range
                            )
                            self.heading_target[z_only_ids] = self.robot.data.heading_w[z_only_ids] + yaw_delta
                            self.is_heading_env[z_only_ids] = True

    def _update_command(self) -> None:
        # 调用基类的每步更新
        super()._update_command()
        
        # 4. 每步更新时强力约束：若处于台阶/短墙爬高地形，强制 y 轴横移持续为 0.0，允许温和的偏航纠偏对齐台阶
        terrain = getattr(self._env.scene, "terrain", None)
        terrain_types = getattr(terrain, "terrain_types", None)
        if terrain_types is not None and len(self._climbing_indices) > 0:
            is_climbing = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
            for idx in self._climbing_indices:
                is_climbing |= (terrain_types == idx)

            # 【出台阶实时重采样】：检查从爬坡状态刚刚离开环境的机器人，进行即时指令重采样
            left_climbing_mask = self.was_climbing & ~is_climbing
            if left_climbing_mask.any():
                left_climbing_ids = torch.where(left_climbing_mask)[0]
                self._resample_command(left_climbing_ids)

            # 【入台阶实时截断】：对于正在爬楼梯/翻越短墙的环境，强行截断其横向横移指令，并进行温和偏航对齐
            climbing_env_ids = is_climbing.nonzero(as_tuple=False).flatten()
            if len(climbing_env_ids) > 0:
                self.vel_command_b[climbing_env_ids, 1] = 0.0
                # 🌟 允许微弱的偏航纠偏，将偏航指令限制在温和的 [-0.3, 0.3] 区间，防止过度甩尾，但保证能修正航向
                self.vel_command_b[climbing_env_ids, 2] = torch.clip(
                    self.cfg.heading_control_stiffness * self.heading_error[climbing_env_ids],
                    min=-0.3,
                    max=0.3
                )
            self.was_climbing = is_climbing

        # 5. 高速侧向解耦（适用于平地/斜坡等混合路面）：当前进速度 >= 0.8 m/s 时，清空侧向指令，防止高速甩尾甩飞
        high_speed_mask = self.vel_command_b[:, 0].abs() >= 0.8
        high_speed_ids = high_speed_mask.nonzero(as_tuple=False).flatten()
        if len(high_speed_ids) > 0:
            self.vel_command_b[high_speed_ids, 1] = 0.0


@dataclass(kw_only=True)
class UniformThresholdVelocityCommandCfg(UniformVelocityCommandCfg):
    class_type: type = UniformThresholdVelocityCommand
    yaw_only_heading_range: float = 1.0
    rel_lateral_envs: float = 0.0
    rel_yaw_envs: float = 0.0
