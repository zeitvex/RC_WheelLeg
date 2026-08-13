import numpy as np
import mujoco
import matplotlib.pyplot as plt
from .math_utils import get_body_velocity

class DisturbanceTester:
    def __init__(self, interval=3.0, duration=0.02, force_mag=2000.0):
        self.interval = interval
        self.duration = duration
        self.force_mag = force_mag
        self.time_log = []
        self.base_vel_log = []
        self.vel_log = []
        self.force_input_log = []
        
        # Define sensors to plot
        self.SENSOR_NAMES_TO_PLOT = {
            "FR": ["FR_hip_torque", "FR_thigh_torque", "FR_calf_torque"],
            "FL": ["FL_hip_torque", "FL_thigh_torque", "FL_calf_torque"],
            "RR": ["RR_hip_torque", "RR_thigh_torque", "RR_calf_torque"],
            "RL": ["RL_hip_torque", "RL_thigh_torque", "RL_calf_torque"],
            "Wheels": ["FR_wheel_torque", "FL_wheel_torque", "RR_wheel_torque", "RL_wheel_torque"]
        }
        self.sensor_logs = {name: [] for group in self.SENSOR_NAMES_TO_PLOT.values() for name in group}

    def update(self, current_time, m, d, estimated_vel):
        cycle_time = current_time % self.interval
        is_pushing = cycle_time < self.duration
        applied_force = np.zeros(6)
        if is_pushing:
            applied_force[0] = -self.force_mag

        # Apply external force
        base_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if base_body_id == -1:
            base_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "trunk")
        if base_body_id != -1:
            d.xfrc_applied[base_body_id] = applied_force

        # Record data
        self.time_log.append(current_time)
        true_vel_body = get_body_velocity(m, d)
        self.base_vel_log.append(true_vel_body)
        self.force_input_log.append(applied_force[0])
        self.vel_log.append(estimated_vel)

        # Record sensor data
        for name in self.sensor_logs.keys():
            sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid != -1:
                adr = m.sensor_adr[sid]
                val = d.sensordata[adr]
                self.sensor_logs[name].append(val)
            else:
                self.sensor_logs[name].append(0.0)

    def plot_results(self):
        print("Generating diagnostic plots...")
        if len(self.time_log) == 0:
            print("No data recorded. Skipping plot.")
            return

        time_arr = np.array(self.time_log)
        vel_arr = np.array(self.base_vel_log)
        force_arr = np.array(self.force_input_log)

        fig, axes = plt.subplots(4, 2, figsize=(16, 18), sharex=True)

        # 1. External Force
        axes[0, 0].plot(time_arr, force_arr, 'r-', linewidth=1.5)
        axes[0, 0].set_title("External Push Force (N)")
        axes[0, 0].set_ylabel("Force")
        axes[0, 0].grid(True)

        # 2. Velocity
        axes[0, 1].plot(time_arr, vel_arr[:, 0], label='True Vx')
        axes[0, 1].plot(time_arr, vel_arr[:, 1], label='True Vy')
        est_vel_arr = np.array(self.vel_log)
        if est_vel_arr.shape[1] >= 2:
            axes[0, 1].plot(time_arr, est_vel_arr[:, 0], '--', label='Est Vx')
            axes[0, 1].plot(time_arr, est_vel_arr[:, 1], '--', label='Est Vy')
        axes[0, 1].set_title("Base Velocity (m/s)")
        axes[0, 1].legend()
        axes[0, 1].grid(True)

        # 3. Legs
        plot_config = [
            ("FR", axes[1, 1]),
            ("FL", axes[1, 0]),
            ("RR", axes[2, 1]),
            ("RL", axes[2, 0])
        ]
        for group_name, ax in plot_config:
            sensor_names = self.SENSOR_NAMES_TO_PLOT[group_name]
            labels = ["Hip", "Thigh", "Calf"]
            for i, s_name in enumerate(sensor_names):
                if s_name in self.sensor_logs:
                    data = self.sensor_logs[s_name]
                    ax.plot(time_arr, data, label=labels[i], linewidth=1)
            ax.set_title(f"{group_name} Leg Torques")
            ax.set_ylabel("Torque (Nm)")
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)

        # 4. Wheels
        ax_wheel = axes[3, 0]
        wheel_sensors = self.SENSOR_NAMES_TO_PLOT["Wheels"]
        for w_name in wheel_sensors:
            if w_name in self.sensor_logs:
                data = self.sensor_logs[w_name]
                short_label = w_name.replace("_wheel_torque", "")
                ax_wheel.plot(time_arr, data, label=short_label, linewidth=1)
        ax_wheel.set_title("Wheel Torques")
        ax_wheel.set_ylabel("Torque (Nm)")
        ax_wheel.set_xlabel("Time (s)")
        ax_wheel.legend()
        ax_wheel.grid(True, alpha=0.3)

        axes[3, 1].axis('off')
        plt.tight_layout()
        plt.show()
