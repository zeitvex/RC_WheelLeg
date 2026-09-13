#!/usr/bin/env python3
"""
轨迹插值模块 - 用于平滑关节角度过渡，避免突变和冲击

支持多种插值方法：
- linear: 线性插值
- cubic: 三次多项式（速度连续）
- quintic: 五次多项式（速度和加速度连续，最平滑）
- cosine: 余弦 S 曲线

使用示例：
    interp = TrajectoryInterpolator(method='quintic', transition_time=0.5)
    
    # 设置新目标
    interp.set_target({'joint1': 1.5, 'joint2': 0.8})
    
    # 每个控制周期调用
    smooth_q = interp.update(dt=0.01, current_q={'joint1': 0.5, 'joint2': 0.3})
"""

import time
from typing import Dict, Optional
import numpy as np


class TrajectoryInterpolator:
    def __init__(self, method: str = 'quintic', transition_time: float = 0.5):
        """
        初始化轨迹插值器
        
        Args:
            method: 插值方法 ('linear', 'cubic', 'quintic', 'cosine')
            transition_time: 过渡时间（秒）
        """
        self.method = method
        self.transition_time = transition_time
        
        self.q_start: Dict[str, float] = {}
        self.q_target: Dict[str, float] = {}
        self.q_current: Dict[str, float] = {}
        
        self.transition_start_time: Optional[float] = None
        self.is_transitioning = False
        
        self._interpolation_funcs = {
            'linear': self._linear,
            'cubic': self._cubic,
            'quintic': self._quintic,
            'cosine': self._cosine,
        }
        
        if method not in self._interpolation_funcs:
            raise ValueError(f"Unknown interpolation method: {method}. "
                           f"Available: {list(self._interpolation_funcs.keys())}")
    
    def set_target(self, q_target: Dict[str, float], force_restart: bool = False):
        """
        设置新的目标角度，开始新的过渡
        
        Args:
            q_target: 目标关节角度字典 {joint_name: angle}
            force_restart: 是否强制重新开始过渡（即使已经在过渡中）
        """
        if not self.q_current:
            self.q_current = q_target.copy()
            self.q_target = q_target.copy()
            self.q_start = q_target.copy()
            self.is_transitioning = False
            return
        
        if not force_restart and self.is_transitioning:
            self.q_target = q_target.copy()
            return
        
        self.q_start = self.q_current.copy()
        self.q_target = q_target.copy()
        self.transition_start_time = time.time()
        self.is_transitioning = True
    
    def update(self, dt: float, current_q: Optional[Dict[str, float]] = None) -> Dict[str, float]:
        """
        更新插值状态，返回当前应该下发的平滑角度
        
        Args:
            dt: 时间步长（秒）
            current_q: 可选的当前实际角度（用于初始化或同步）
        
        Returns:
            平滑后的关节角度字典
        """
        if current_q is not None and not self.q_current:
            self.q_current = current_q.copy()
            self.q_start = current_q.copy()
            self.q_target = current_q.copy()
            return self.q_current.copy()
        
        if not self.is_transitioning:
            return self.q_target.copy()
        
        elapsed = time.time() - self.transition_start_time
        
        if elapsed >= self.transition_time:
            self.q_current = self.q_target.copy()
            self.is_transitioning = False
            return self.q_current.copy()
        
        s = elapsed / self.transition_time
        alpha = self._interpolation_funcs[self.method](s)
        
        self.q_current = {}
        for joint_name in self.q_target:
            start_val = self.q_start.get(joint_name, 0.0)
            target_val = self.q_target[joint_name]
            self.q_current[joint_name] = start_val + (target_val - start_val) * alpha
        
        return self.q_current.copy()
    
    def reset(self, q_init: Optional[Dict[str, float]] = None):
        """
        重置插值器状态
        
        Args:
            q_init: 初始角度，如果为 None 则清空所有状态
        """
        if q_init is None:
            self.q_start = {}
            self.q_target = {}
            self.q_current = {}
        else:
            self.q_start = q_init.copy()
            self.q_target = q_init.copy()
            self.q_current = q_init.copy()
        
        self.transition_start_time = None
        self.is_transitioning = False
    
    def is_done(self) -> bool:
        """返回是否已完成当前过渡"""
        return not self.is_transitioning
    
    def set_transition_time(self, t: float):
        """动态修改过渡时间"""
        self.transition_time = max(0.01, t)
    
    def set_method(self, method: str):
        """动态修改插值方法"""
        if method not in self._interpolation_funcs:
            raise ValueError(f"Unknown method: {method}")
        self.method = method
    
    @staticmethod
    def _linear(s: float) -> float:
        """线性插值：alpha = s"""
        return np.clip(s, 0.0, 1.0)
    
    @staticmethod
    def _cubic(s: float) -> float:
        """三次多项式：alpha = 3s² - 2s³"""
        s = np.clip(s, 0.0, 1.0)
        return 3.0 * s**2 - 2.0 * s**3
    
    @staticmethod
    def _quintic(s: float) -> float:
        """五次多项式：alpha = 10s³ - 15s⁴ + 6s⁵"""
        s = np.clip(s, 0.0, 1.0)
        return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
    
    @staticmethod
    def _cosine(s: float) -> float:
        """余弦 S 曲线：alpha = (1 - cos(πs)) / 2"""
        s = np.clip(s, 0.0, 1.0)
        return (1.0 - np.cos(np.pi * s)) / 2.0


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    
    print("轨迹插值模块测试")
    print("=" * 60)
    
    methods = ['linear', 'cubic', 'quintic', 'cosine']
    colors = ['blue', 'green', 'red', 'purple']
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    for method, color in zip(methods, colors):
        interp = TrajectoryInterpolator(method=method, transition_time=1.0)
        
        interp.reset({'joint1': 0.0})
        interp.set_target({'joint1': 1.0})
        
        times = []
        positions = []
        velocities = []
        
        t = 0.0
        dt = 0.01
        last_pos = 0.0
        
        while t <= 1.0:
            q = interp.update(dt)
            pos = q['joint1']
            vel = (pos - last_pos) / dt if t > 0 else 0.0
            
            times.append(t)
            positions.append(pos)
            velocities.append(vel)
            
            last_pos = pos
            t += dt
        
        ax1.plot(times, positions, label=method, color=color, linewidth=2)
        ax2.plot(times, velocities, label=method, color=color, linewidth=2)
    
    ax1.set_xlabel('时间 (s)')
    ax1.set_ylabel('位置 (rad)')
    ax1.set_title('不同插值方法的位置曲线')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('速度 (rad/s)')
    ax2.set_title('不同插值方法的速度曲线')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/home/rc2/work/rcwork/trajectory_interpolation_comparison.png', dpi=150)
    print("已保存对比图到: trajectory_interpolation_comparison.png")
    
    print("\n测试完成")
    print("=" * 60)
    print("推荐使用:")
    print("  - quintic: 最平滑，速度和加速度连续")
    print("  - cosine: 平滑且计算简单")
    print("  - cubic: 速度连续，比 quintic 稍快")
    print("  - linear: 最简单但速度会突变")
