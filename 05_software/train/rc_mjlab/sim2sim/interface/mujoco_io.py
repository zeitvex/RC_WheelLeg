import os
import mujoco
import numpy as np
from pathlib import Path
from tools.math_utils import get_gravity_orientation

class LowPassFilter:
    def __init__(self, cutoff_freq, dt, dim):
        self.alpha = dt / (dt + 1.0 / (2.0 * np.pi * cutoff_freq))
        self.y_prev = None

    def filter(self, x):
        if self.y_prev is None:
            self.y_prev = x.copy()
        y = self.alpha * x + (1.0 - self.alpha) * self.y_prev
        self.y_prev = y.copy()
        return y

class MuJoCoIO:
    """Handles MuJoCo simulation environment initialization and IO."""
    def __init__(self, terrain_xml_path, robot_xml_path, hfield_dir):
        # Create temp XML
        temp_xml = self._create_sim2sim_xml(terrain_xml_path, robot_xml_path, hfield_dir)
        
        print("[MuJoCoIO] Loading MuJoCo model...")
        spec = mujoco.MjSpec.from_file(str(temp_xml))
        
        # Override actuators to match mjlab exactly
        self._rebuild_actuators(spec)
        
        self.m = spec.compile()
        # Boost headlight
        self.m.vis.headlight.ambient[:] = [0.6, 0.6, 0.6]
        self.m.vis.headlight.diffuse[:] = [0.8, 0.8, 0.8]
        self.d = mujoco.MjData(self.m)
        
        # Joint definitions
        self.leg_joint_names = [
            "fl_hip_abduction_joint", "fl_hip_pitch_joint", "fl_knee_joint",
            "fr_hip_abduction_joint", "fr_hip_pitch_joint", "fr_knee_joint",
            "rl_hip_abduction_joint", "rl_hip_pitch_joint", "rl_knee_joint",
            "rr_hip_abduction_joint", "rr_hip_pitch_joint", "rr_knee_joint",
        ]
        self.wheel_joint_names = [
            "fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint",
        ]
        self.all_joint_names = self.leg_joint_names + self.wheel_joint_names
        
        self.qpos_ids = np.array([
            self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in self.all_joint_names
        ])
        self.qvel_ids = np.array([
            self.m.jnt_dofadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in self.all_joint_names
        ])
        self.ctrl_ids = np.array([
            mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
            for n in self.all_joint_names
        ])
        
        sim_dt = self.m.opt.timestep
        self.control_dt = sim_dt * int(round(0.02 / sim_dt))
        
        self.lpf_legs = LowPassFilter(cutoff_freq=5.0, dt=self.control_dt, dim=12)
        self.lpf_wheels = LowPassFilter(cutoff_freq=15.0, dt=self.control_dt, dim=4)

    def _create_sim2sim_xml(self, terrain_xml_path, robot_xml_path, hfield_dir):
        with open(terrain_xml_path, "r", encoding="utf-8") as f:
            content = f.read()

        robot_xml_abs = str(robot_xml_path.absolute()).replace("\\", "/")
        content = content.replace(
            '<include file="go2w.xml"/>', f'<include file="{robot_xml_abs}"/>'
        )

        hfield_1 = str((hfield_dir / "height_field.png").absolute()).replace("\\", "/")
        hfield_2 = str((hfield_dir / "unitree_hfield.png").absolute()).replace("\\", "/")
        content = content.replace("../height_field.png", hfield_1)
        content = content.replace("../unitree_hfield.png", hfield_2)

        out_xml_path = robot_xml_path.parent / "sim2sim_temp.xml"
        with open(out_xml_path, "w", encoding="utf-8") as f:
            f.write(content)
            
        os.chdir(str(robot_xml_path.parent))
        return out_xml_path

    def _rebuild_actuators(self, spec):
        actuators_to_delete = list(spec.actuators)
        for act in actuators_to_delete:
            spec.delete(act)
        
        KP_LEG, KD_LEG = 40.0, 1.0
        KD_WHEEL = 0.5
        EFFORT_LIMIT = 17.0
        
        leg_jnames = [
            "fl_hip_abduction_joint", "fr_hip_abduction_joint",
            "rl_hip_abduction_joint", "rr_hip_abduction_joint",
            "fl_hip_pitch_joint", "fr_hip_pitch_joint",
            "rl_hip_pitch_joint", "rr_hip_pitch_joint",
            "fl_knee_joint", "fr_knee_joint",
            "rl_knee_joint", "rr_knee_joint",
        ]
        wheel_jnames = ["fl_wheel_joint", "fr_wheel_joint", "rl_wheel_joint", "rr_wheel_joint"]
        
        for jname in leg_jnames:
            act = spec.add_actuator(name=jname, target=jname)
            act.trntype = mujoco.mjtTrn.mjTRN_JOINT
            act.dyntype = mujoco.mjtDyn.mjDYN_NONE
            act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            act.gainprm[0] = KP_LEG
            act.biasprm[1] = -KP_LEG
            act.biasprm[2] = -KD_LEG
            act.forcelimited = True
            act.forcerange[:] = [-EFFORT_LIMIT, EFFORT_LIMIT]
            act.inheritrange = 0.0
            act.ctrllimited = False
            
        for jname in wheel_jnames:
            act = spec.add_actuator(name=jname, target=jname)
            act.trntype = mujoco.mjtTrn.mjTRN_JOINT
            act.dyntype = mujoco.mjtDyn.mjDYN_NONE
            act.gaintype = mujoco.mjtGain.mjGAIN_FIXED
            act.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            act.gainprm[0] = KD_WHEEL
            act.biasprm[2] = -KD_WHEEL
            act.forcelimited = True
            act.forcerange[:] = [-EFFORT_LIMIT, EFFORT_LIMIT]
            act.inheritrange = 0.0
            act.ctrllimited = False
            
        l = spec.worldbody.add_light()
        l.pos[:] = [3.7, -9.0, 4.0]
        l.dir[:] = [0.0, 0.0, -1.0]
        l.diffuse[:] = [0.8, 0.8, 0.8]
        l.specular[:] = [0.3, 0.3, 0.3]

    def reset_robot(self, default_dof_pos):
        print("[MuJoCoIO] Dropping robot to floor...")
        self.d.qpos[:3] = [3.7, -9.0, 0.6]
        self.d.qpos[self.qpos_ids] = default_dof_pos
        self.d.ctrl[self.ctrl_ids[:12]] = default_dof_pos[:12]
        self.d.ctrl[self.ctrl_ids[12:]] = 0.0
        for _ in range(500):
            mujoco.mj_step(self.m, self.d)

    def get_obs_53d(self, command, default_dof_pos, last_actions_raw):
        quat_wxyz = self.d.qpos[3:7].copy()
        ang_vel_body = self.d.qvel[3:6].copy()
        base_ang_vel = (ang_vel_body * 0.25).astype(np.float32)
        projected_gravity = get_gravity_orientation(quat_wxyz)
        
        dof_pos = self.d.qpos[self.qpos_ids]
        dof_vel = self.d.qvel[self.qvel_ids]
        
        joint_pos_rel = (dof_pos[:12] - default_dof_pos[:12]).astype(np.float32)
        joint_vel_leg = (dof_vel[:12] * 0.05).astype(np.float32)
        wheel_vel = (dof_vel[12:] * 0.05).astype(np.float32)
        
        obs = np.concatenate([
            base_ang_vel,        # 3
            projected_gravity,   # 3
            command,             # 3
            joint_pos_rel,       # 12
            joint_vel_leg,       # 12
            wheel_vel,           # 4
            last_actions_raw,    # 16
        ])
        return obs

    def send_actions(self, scaled_actions, default_dof_pos):
        act = scaled_actions + default_dof_pos
        act = np.clip(act, -100, 100)
        
        # Apply Low Pass Filter
        act[:12] = self.lpf_legs.filter(act[:12])
        act[12:] = self.lpf_wheels.filter(act[12:])
        
        for i in range(min(len(act), len(self.d.ctrl))):
            self.d.ctrl[self.ctrl_ids[i]] = act[i]
