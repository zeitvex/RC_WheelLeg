"""RobStride 电机驱动包装。

职责：
- 封装 ik_real 中 RobStrideDriver 的 enable/disable/clear/control_mit 调用
- **真实的丢包检测**：旧版用「value=0 启发式」会误判（电机回机械零位时也是 0）。
  新方案：
    1. 调用 process_messages 前快照所有电机的 (pos, vel, torque)
    2. 调用后比较：状态变了 → 这一帧有新反馈；状态完全没变 → 累计 stale_count
    3. stale_count 超过阈值才沿用上一帧（方法论 3.4.2）
  仍然不完美（电机长时间静止确实会有连续多帧 state 不变），但比 0 启发式可靠。
- 通过 driver_factory 由调用方注入：远程 Linux 主机用 RobStrideDriver，
  本地 Windows 调试可用 Mock。
"""
from dataclasses import dataclass
import threading
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from interface.motor_mapping import MotorMapping


@dataclass
class MotorReading:
    position: float
    velocity: float
    torque: float = 0.0
    fresh: bool = False  # True 表示本帧驱动板有新反馈


class HardwareIO:
    """统一的电机+IMU总线接口（不含策略），主控调用这一层。

    Args:
        driver_factory: () -> (drv1, drv2)，由调用方注入；返回的对象需要满足：
            connect()/disconnect()/disable(name)/enable(name)/clear_warnings(name)
            add_motor(name, mid, model)/process_messages()
            control_mit(name, q, dq, kp, kd, tau)
            .motors: dict[name -> motor], motor.state.position / .velocity / .torque
        config: yaml 解析后的字典
    """

    def __init__(self, driver_factory: Callable[[str, str, bool], Tuple[object, object]],
                 motor_model: str, can1_port: str, can2_port: str, debug: bool = False,
                 stale_frames_to_holdover: int = 2):
        self.mapper = MotorMapping()
        drv1, drv2 = driver_factory(can1_port, can2_port, debug)
        self.driver_can1 = drv1
        self.driver_can2 = drv2
        self.motor_model = motor_model
        self.stale_frames_to_holdover = stale_frames_to_holdover

        # 上一帧反馈（按 (bus, can_id) 索引），用于丢包兜底
        self._last_pos: Dict[Tuple[int, int], float] = {}
        self._last_vel: Dict[Tuple[int, int], float] = {}
        self._last_torque: Dict[Tuple[int, int], float] = {}
        # 每个电机连续多少帧没收到新反馈
        self._stale_counts: Dict[Tuple[int, int], int] = {}
        # 第一次必须读到才能解锁，避免初始化时直接用零位发送大力矩
        self._initialized = False

        self.lock = threading.Lock()

        # 累计诊断
        self.holdover_total = 0  # 累计被沿用上一帧的次数

    # ---- 总线管理 ----
    def connect(self):
        self.driver_can1.connect()
        self.driver_can2.connect()
        for jk in self.mapper.SIM_JOINT_ORDER:
            leg, joint = jk
            bus, mid = self.mapper.CAN_ID_MAP[jk]
            name = f"{leg}_{joint}"
            drv = self.driver_can1 if bus == 1 else self.driver_can2
            drv.add_motor(name, mid, self.motor_model)
            self._stale_counts[(bus, mid)] = 0

    def disconnect(self):
        try:
            self.driver_can1.disconnect()
        finally:
            self.driver_can2.disconnect()

    def enable_all(self):
        for drv in (self.driver_can1, self.driver_can2):
            for name in drv.motors:
                drv.clear_warnings(name)
                drv.enable(name)

    def disable_all(self):
        for drv in (self.driver_can1, self.driver_can2):
            for name in drv.motors:
                drv.disable(name)

    # ---- 状态读取 ----
    def _snapshot_state(self) -> Dict[Tuple[int, int], Tuple[float, float, float, int]]:
        """快照所有电机的 (pos, vel, torque, update_count)，process_messages 前后比较即可判 fresh。"""
        snap: Dict[Tuple[int, int], Tuple[float, float, float, int]] = {}
        for drv_idx, drv in enumerate((self.driver_can1, self.driver_can2)):
            bus = drv_idx + 1
            for name, motor in drv.motors.items():
                parts = name.split("_", 1)
                if len(parts) != 2:
                    continue
                key = (parts[0], parts[1])
                if key not in self.mapper.CAN_ID_MAP:
                    continue
                _, mid = self.mapper.CAN_ID_MAP[key]
                s = motor.state
                snap[(bus, mid)] = (s.position, s.velocity, s.torque, getattr(s, "update_count", 0))
        return snap

    def read_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, Dict[str, object]]:
        """返回 (sim_joint_pos[16], sim_joint_vel[16], sim_joint_torque[16], debug_info)。"""
        with self.lock:
            # 1) 抓取上一次的状态作为「pre」快照（基线）
            pre = self._snapshot_state()

            # 2) 拉取本帧反馈
            self.driver_can1.process_messages()
            self.driver_can2.process_messages()

            # 3) 抓取「post」快照
            post = self._snapshot_state()

            # 4) 比较：state 元组变了 → 本帧有新反馈，stale_count 清零；否则 stale_count++
            per_motor_fresh: Dict[Tuple[int, int], bool] = {}
            for key in post:
                fresh = (pre.get(key) != post[key])
                per_motor_fresh[key] = fresh
                if fresh:
                    self._stale_counts[key] = 0
                else:
                    self._stale_counts[key] += 1

            # 5) 取出本帧 pos/vel；若该电机连续多帧没刷新，沿用上一帧（方法论 3.4.2）
            real_pos: Dict[Tuple[int, int], float] = {}
            real_vel: Dict[Tuple[int, int], float] = {}
            real_torque: Dict[Tuple[int, int], float] = {}
            holdover_this_frame = 0
            for key, (pos, vel, tor, _) in post.items():
                if (not per_motor_fresh[key]) and self._stale_counts[key] >= self.stale_frames_to_holdover:
                    # 长时间不刷新视作丢包：沿用上一帧
                    if key in self._last_pos:
                        real_pos[key] = self._last_pos[key]
                        real_vel[key] = self._last_vel[key]
                        real_torque[key] = self._last_torque[key]
                        holdover_this_frame += 1
                    else:
                        real_pos[key] = pos
                        real_vel[key] = vel
                        real_torque[key] = tor
                else:
                    real_pos[key] = pos
                    real_vel[key] = vel
                    real_torque[key] = tor

            self.holdover_total += holdover_this_frame
            # 缓存本帧（即便部分是 holdover 也缓存）
            self._last_pos = real_pos.copy()
            self._last_vel = real_vel.copy()
            self._last_torque = real_torque.copy()
            if not self._initialized:
                self._initialized = True

            cur_pos = self.mapper.real_to_sim(real_pos)
            cur_vel = self.mapper.real_vel_to_sim(real_vel)
            cur_torque = self.mapper.real_vel_to_sim(real_torque)

            # 诊断信息
            stale_max = max(self._stale_counts.values()) if self._stale_counts else 0
            n_stale_motors = sum(1 for c in self._stale_counts.values()
                                 if c >= self.stale_frames_to_holdover)
            # 按 SIM_JOINT_ORDER 排列的每个电机连续丢帧数
            per_motor_stale = [
                self._stale_counts.get(self.mapper.CAN_ID_MAP[jk], 99)
                for jk in self.mapper.SIM_JOINT_ORDER
            ]
            return cur_pos, cur_vel, cur_torque, {
                "holdover_this_frame": holdover_this_frame,
                "stale_max": stale_max,
                "n_stale_motors": n_stale_motors,
                "fresh_count": sum(1 for v in per_motor_fresh.values() if v),
                "per_motor_stale": per_motor_stale,
            }
        
    def passive_poll(self):
        """发送全 0 (0刚度0阻尼0力矩) 的 MIT 指令给所有电机。
        目的：在 ENABLED 状态下，不产生力矩地索要反馈（因为 RobStride 在 MIT 模式下必须有指令才反馈）。"""
        with self.lock:
            for jk in self.mapper.SIM_JOINT_ORDER:
                bus, mid = self.mapper.CAN_ID_MAP[jk]
                name = f"{jk[0]}_{jk[1]}"
                drv = self.driver_can1 if bus == 1 else self.driver_can2
                if name in drv.motors:
                    drv.control_mit(name, 0.0, 0.0, 0.0, 0.0, 0.0)

    # ---- 控制下发 ----
    def send_control(self, target_angles: np.ndarray, kp_leg: float, kd_leg: float,
                     kd_wheel: float):
        """与 sim2sim 的 PD 模型对齐：
        - 腿: position 控制，目标角度由 target_angles[:12] 给出，kp/kd 来自配置
        - 轮: velocity 控制，目标速度由 target_angles[12:] 给出，kd 阻尼
        """
        with self.lock:
            if target_angles.shape != (16,):
                raise ValueError("target_angles must be (16,)")

            real_targets = self.mapper.sim_to_real(target_angles.astype(np.float32))
            
            # 轮毂速度目标暂且用 0，如果 target_angles 里包含了速度，就在 policy 那里处理，
            # 这里的 target_angles 是 pose 目标，轮毂作为连续旋转关节其实位置控制没有意义。
            # 为了兼容旧代码，这里构造一个 16 维的 velocity array，只有后 4 个是目标（如果当作速度的话）。
            vel_targets = np.zeros(16, dtype=np.float32)
            vel_targets[12:] = target_angles[12:].astype(np.float32)
            real_wheel = self.mapper.sim_vel_to_real(vel_targets)

            for jk in self.mapper.SIM_JOINT_ORDER:
                leg, joint = jk
                bus, mid = self.mapper.CAN_ID_MAP[jk]
                name = f"{leg}_{joint}"
                drv = self.driver_can1 if bus == 1 else self.driver_can2
                if name not in drv.motors:
                    continue

                if joint == "wheel":
                    v = real_wheel[(bus, mid)]
                    drv.control_mit(name, 0.0, v, 0.0, kd_wheel, 0.0)
                else:
                    q = real_targets[(bus, mid)]
                    drv.control_mit(name, q, 0.0, kp_leg, kd_leg, 0.0)

    def damping_brake(self, kd_leg: float, kd_wheel: float):
        """急停模式：所有关节卸载刚度，仅保留阻尼。
        对应 270_SimToReal 方法论 97.11 Level 2 "刹车"。
        """
        with self.lock:
            for jk in self.mapper.SIM_JOINT_ORDER:
                leg, joint = jk
                bus, _ = self.mapper.CAN_ID_MAP[jk]
                name = f"{leg}_{joint}"
                drv = self.driver_can1 if bus == 1 else self.driver_can2
                if name not in drv.motors:
                    continue
                kd = kd_wheel if joint == "wheel" else kd_leg
                drv.control_mit(name, 0.0, 0.0, 0.0, kd, 0.0)

    def wait_feedback_ready(self, max_attempts: int = 20,
                            poll_interval: float = 0.05) -> Tuple[bool, list]:
        """enable 后调用：尝试 max_attempts 次读总线，等所有 16 个电机
        都至少给出一帧反馈。
        返回 (all_ready, missing_motors)；missing_motors 是 (bus, mid, name) 列表。
        """
        import time
        seen: Dict[Tuple[int, int], bool] = {
            self.mapper.CAN_ID_MAP[jk]: False for jk in self.mapper.SIM_JOINT_ORDER
        }
        # 用第一次读到的 (pos, vel, torque) 三元组的"非零"或"已变化"作为反馈到达的判据。
        # 启动瞬间所有 motor.state 默认全 0，要么收到反馈让其变化，要么收到反馈但值确实是 0。
        # 退化情况下电机静止时 vel=0 且 pos=机械零位也=0，那种情况只能等多帧确认。
        snap_prev = self._snapshot_state()
        for attempt in range(max_attempts):
            with self.lock:
                self.driver_can1.process_messages()
                self.driver_can2.process_messages()
                snap_cur = self._snapshot_state()
            for key, fields_cur in snap_cur.items():
                if seen[key]:
                    continue
                fields_prev = snap_prev.get(key)
                # 任一字段不为 0 → 一定有反馈（因为初始值都是 0）
                if any(v != 0.0 for v in fields_cur):
                    seen[key] = True
                # 与上一次快照不同 → 一定有反馈（即便都很小）
                elif fields_prev is not None and fields_cur != fields_prev:
                    seen[key] = True
            snap_prev = snap_cur
            if all(seen.values()):
                return True, []
            time.sleep(poll_interval)

        # 超时：列出仍未反馈的电机
        missing = []
        rev_can = {v: k for k, v in self.mapper.CAN_ID_MAP.items()}
        for key, ok in seen.items():
            if not ok:
                leg, joint = rev_can[key]
                missing.append((key[0], key[1], f"{leg}_{joint}"))
        return False, missing

    def read_measured_pose(self) -> np.ndarray:
        """返回 (16,) 当前实测 sim 坐标系下的关节位置。
        会先 process_messages 一次保证拿到本帧。
        """
        self.driver_can1.process_messages()
        self.driver_can2.process_messages()
        real_pos: Dict[Tuple[int, int], float] = {}
        for drv_idx, drv in enumerate((self.driver_can1, self.driver_can2)):
            bus = drv_idx + 1
            for name, motor in drv.motors.items():
                parts = name.split("_", 1)
                if len(parts) != 2:
                    continue
                key = (parts[0], parts[1])
                if key not in self.mapper.CAN_ID_MAP:
                    continue
                _, mid = self.mapper.CAN_ID_MAP[key]
                real_pos[(bus, mid)] = motor.state.position
        return self.mapper.real_to_sim(real_pos)
