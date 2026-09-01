import time
import numpy as np
from pathlib import Path
from tools.math_utils import get_body_velocity

class SimpleLogger:
    def __init__(self, log_dir="."):
        self.log_file = Path(log_dir) / f"sim2sim_log_{int(time.time())}.txt"
        self.data = []
        print(f"[Logger] Will write diagnostic data to: {self.log_file}")
        
    def update(self, sim_time, m, d, command):
        true_vel_body = get_body_velocity(m, d)
        # 记录 仿真时间、控制指令、真实的机体线速度
        self.data.append(
            f"{sim_time:.4f}, {command[0]:.4f}, {command[1]:.4f}, {command[2]:.4f}, "
            f"{true_vel_body[0]:.4f}, {true_vel_body[1]:.4f}, {true_vel_body[2]:.4f}"
        )
        
    def save(self):
        print(f"\n[Logger] Saving {len(self.data)} records to {self.log_file}...")
        with open(self.log_file, "w") as f:
            f.write("time, cmd_vx, cmd_vy, cmd_yaw, true_vx, true_vy, true_vz\n")
            for row in self.data:
                f.write(row + "\n")
        print("[Logger] Save complete.")
