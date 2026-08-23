"""
Sim2Sim: Deploy rc_mjlab policy in MuJoCo with RC_MAP terrain.

Actuator setup matches mjlab training exactly:
  - Legs: <position> actuator (kp=40, kd=1) — d.ctrl = target_position
  - Wheels: <velocity> actuator (kd=0.5) — d.ctrl = target_velocity

The original XML's <general> actuators are overridden in Python to match
the mjlab BuiltinPositionActuator / BuiltinVelocityActuator setup.
"""
import os
import time
import math
import torch
import torch.nn as nn
import mujoco
import mujoco.viewer
import numpy as np
import pygame
from pathlib import Path


# ============================================================
# Policy Model
# ============================================================
class PolicyMLP(nn.Module):
    def __init__(self, obs_dim=318, action_dim=16):
        super().__init__()
        self.register_buffer("obs_mean", torch.zeros(obs_dim))
        self.register_buffer("obs_std", torch.ones(obs_dim))
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 512), nn.ELU(),
            nn.Linear(512, 256), nn.ELU(),
            nn.Linear(256, 128), nn.ELU(),
            nn.Linear(128, action_dim),
        )

    def forward(self, x):
        x = (x - self.obs_mean) / self.obs_std
        return self.net(x)


def load_policy(model_path, device):
    if str(model_path).endswith('.onnx'):
        import onnxruntime as ort
        session = ort.InferenceSession(str(model_path))
        class OnnxWrapper:
            def __init__(self, session):
                self.session = session
            def __call__(self, x):
                inputs = {self.session.get_inputs()[0].name: x.cpu().numpy()}
                out = self.session.run(None, inputs)[0]
                return torch.tensor(out, device=x.device)
        return OnnxWrapper(session)


    ckpt = torch.load(model_path, map_location=device, weights_only=False)
    state_dict = ckpt["actor_state_dict"]
    model = PolicyMLP()
    my_sd = {}
    for k, v in state_dict.items():
        if k.startswith("mlp."):
            my_sd[k.replace("mlp.", "net.")] = v
        elif k == "obs_normalizer._mean":
            my_sd["obs_mean"] = v.squeeze()
        elif k == "obs_normalizer._var":
            my_sd["obs_std"] = torch.sqrt(v.squeeze() + 1e-5)
    model.load_state_dict(my_sd, strict=False)
    model.eval()
    model.to(device)
    return model


# ============================================================
# Math Utilities
# ============================================================
def get_gravity_orientation(quat_wxyz):
    """
    Compute projected gravity in body frame from quaternion [w,x,y,z].
    Proven formula from DreamWaQ-sim2sim reference (easy_math.py).
    """
    qw, qx, qy, qz = quat_wxyz
    gx = 2.0 * (-qz * qx + qw * qy)
    gy = -2.0 * (qz * qy + qw * qx)
    gz = 1.0 - 2.0 * (qw * qw + qz * qz)
    return np.array([gx, gy, gz], dtype=np.float32)


def quat_rotate_inverse(quat_wxyz, v):
    """
    Rotate vector v from world frame to body frame.
    quat_wxyz: [w, x, y, z] (as stored in MuJoCo qpos[3:7])
    v: [3] world-frame vector
    Same formula as go2w_sim2sim/lab2mujoco.py world2self (no conjugate).
    """
    q_w = quat_wxyz[0]
    q_vec = quat_wxyz[1:]
    a = v * (2.0 * q_w * q_w - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a - b + c


# ============================================================
# XML Preparation
# ============================================================
def create_sim2sim_xml(terrain_xml_path, robot_xml_path, hfield_dir, out_xml_path):
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

    with open(out_xml_path, "w", encoding="utf-8") as f:
        f.write(content)


class LowPassFilter:
    def __init__(self, cutoff_freq, dt, dim):
        self.alpha = dt / (dt + 1.0 / (2.0 * math.pi * cutoff_freq))
        self.y_prev = np.zeros(dim, dtype=np.float64)

    def filter(self, x):
        y = self.alpha * x + (1.0 - self.alpha) * self.y_prev
        self.y_prev = y.copy()
        return y


# ============================================================
# Main
# ============================================================
def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    project_root = Path(__file__).parent.parent.absolute()
    # Local terrain directory to make rc_mjlab independent
    terrain_dir = Path(__file__).parent / "terrain"
    terrain_xml = terrain_dir / "scene_terrain.xml"
    robot_xml = Path(__file__).parent.parent / "mjcf" / "wheelleg.xml"
    policy_path = Path(__file__).parent.parent / "model_6800.onnx"
    if not policy_path.exists():
        policy_path = Path(__file__).parent.parent / "model_rough.pt"
    hfield_dir = terrain_dir

    temp_xml = project_root / "mjcf" / "sim2sim_temp.xml"
    create_sim2sim_xml(terrain_xml, robot_xml, hfield_dir, temp_xml)

    print("Loading MuJoCo model...")
    os.chdir(str(project_root / "mjcf"))
    
    # Build model using MjSpec (same as mjlab training) to avoid
    # broken <include> which renames joints and drops actuators.
    spec = mujoco.MjSpec.from_file(str(temp_xml))
    
    # Delete existing XML actuators (mjlab's get_spec() does this)
    actuators_to_delete = list(spec.actuators)
    for act in actuators_to_delete:
        spec.delete(act)
    
    # Rebuild actuators matching mjlab training config exactly
    KP_LEG = 40.0
    KD_LEG = 1.0
    KD_WHEEL = 0.5
    EFFORT_LIMIT = 17.0
    
    leg_joint_names = [
        "fl_hip_abduction_joint", "fr_hip_abduction_joint",
        "rl_hip_abduction_joint", "rr_hip_abduction_joint",
        "fl_hip_pitch_joint", "fr_hip_pitch_joint",
        "rl_hip_pitch_joint", "rr_hip_pitch_joint",
        "fl_knee_joint", "fr_knee_joint",
        "rl_knee_joint", "rr_knee_joint",
    ]
    wheel_joint_names = [
        "fl_wheel_joint", "fr_wheel_joint",
        "rl_wheel_joint", "rr_wheel_joint",
    ]
    
    # Add position actuators for legs (kp=40, kd=1)
    for jname in leg_joint_names:
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
    
    # Add velocity actuators for wheels (kd=0.5)
    for jname in wheel_joint_names:
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
    
    # Add an explicit light over the spawn area so it's guaranteed to be bright
    l = spec.worldbody.add_light()
    l.pos[:] = [3.7, -9.0, 4.0]
    l.dir[:] = [0.0, 0.0, -1.0]
    l.diffuse[:] = [0.8, 0.8, 0.8]
    l.specular[:] = [0.3, 0.3, 0.3]
    
    m = spec.compile()
    
    # Boost global headlight to ensure no dark corners when camera moves
    m.vis.headlight.ambient[:] = [0.6, 0.6, 0.6]
    m.vis.headlight.diffuse[:] = [0.8, 0.8, 0.8]
    
    d = mujoco.MjData(m)
    
    # Verify actuators
    print(f"Model: {m.njnt} joints, {m.nu} actuators")
    for i in range(m.nu):
        print(f"  actuator[{i}] = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i)}")

    print(f"Loading Policy from {policy_path}...")
    policy = load_policy(policy_path, device)
    print("Policy loaded successfully.")

    # ------------------------------------------------------------------
    # Joint Configuration — names match the XML exactly (MjSpec preserves them)
    # ------------------------------------------------------------------
    leg_joint_names = [
        "fl_hip_abduction_joint", "fl_hip_pitch_joint", "fl_knee_joint",
        "fr_hip_abduction_joint", "fr_hip_pitch_joint", "fr_knee_joint",
        "rl_hip_abduction_joint", "rl_hip_pitch_joint", "rl_knee_joint",
        "rr_hip_abduction_joint", "rr_hip_pitch_joint", "rr_knee_joint",
    ]
    wheel_joint_names = [
        "fl_wheel_joint",
        "fr_wheel_joint",
        "rl_wheel_joint",
        "rr_wheel_joint",
    ]
    # ------------------------------------------------------------------
    all_joint_names = leg_joint_names + wheel_joint_names

    qpos_ids = np.array([
        m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
        for n in all_joint_names
    ])
    qvel_ids = np.array([
        m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
        for n in all_joint_names
    ])
    ctrl_ids = np.array([
        mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
        for n in all_joint_names
    ])

    # Verify all IDs are valid
    for i, name in enumerate(all_joint_names):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert jid >= 0, f"Joint '{name}' not found!"
        assert ctrl_ids[i] >= 0, f"Actuator '{name}' not found!"
    print(f"All {len(all_joint_names)} joints and actuators verified.")

    # Default joint positions
    default_dof_pos = np.array([
        0.0, 0.9, -1.8,   # FL
        0.0, 0.9, -1.8,   # FR
        0.0, 0.9, -1.8,   # RL
        0.0, 0.9, -1.8,   # RR
        0.0, 0.0, 0.0, 0.0,       # wheel
    ], dtype=np.float64)

    # Action scales
    HIP_SCALE = 0.125
    LEG_POS_SCALE = 0.25
    WHEEL_VEL_SCALE = 5.0
    action_scale = np.array([
        HIP_SCALE, LEG_POS_SCALE, LEG_POS_SCALE,  # FL
        HIP_SCALE, LEG_POS_SCALE, LEG_POS_SCALE,  # FR
        HIP_SCALE, LEG_POS_SCALE, LEG_POS_SCALE,  # RL
        HIP_SCALE, LEG_POS_SCALE, LEG_POS_SCALE,  # RR
        WHEEL_VEL_SCALE, WHEEL_VEL_SCALE, WHEEL_VEL_SCALE, WHEEL_VEL_SCALE
    ], dtype=np.float64)

    # ------------------------------------------------------------------
    # Physics & Control
    # ------------------------------------------------------------------
    sim_dt = m.opt.timestep
    decimation = int(round(0.02 / sim_dt))  # 50Hz control
    control_dt = sim_dt * decimation

    lpf_legs = LowPassFilter(cutoff_freq=5.0, dt=control_dt, dim=12)
    lpf_wheels = LowPassFilter(cutoff_freq=15.0, dt=control_dt, dim=4)

    # History buffer
    history_length = 6
    obs_dim = 53
    obs_history = np.zeros((history_length, obs_dim), dtype=np.float32)

    last_actions = np.zeros(16, dtype=np.float32)
    command = np.array([0.0, 0.0, 0.0], dtype=np.float32)

    # ------------------------------------------------------------------
    # Pygame UI
    # ------------------------------------------------------------------
    pygame.init()
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Go2W Sim2Sim Control")
    font = pygame.font.Font(pygame.font.get_default_font(), 24)

    def get_obs():
        """Build 53-dim observation matching IsaacLab env_cfgs.py actor_terms order."""
        quat_wxyz = d.qpos[3:7].copy()
        # MuJoCo d.qvel[3:6] for free joints is ALREADY in body frame
        # (unlike cvel which is in world frame). No rotation needed.
        ang_vel_body = d.qvel[3:6].copy()

        # Body-frame angular velocity * scale
        base_ang_vel = (ang_vel_body * 0.25).astype(np.float32)

        # Projected gravity
        projected_gravity = get_gravity_orientation(quat_wxyz)

        # Joint states
        dof_pos = d.qpos[qpos_ids]
        dof_vel = d.qvel[qvel_ids]

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
            last_actions,        # 16
        ])  # total = 53
        return obs

    # ------------------------------------------------------------------
    # Initialize: set default pose and let robot settle
    # ------------------------------------------------------------------
    print("Dropping robot to floor...")
    d.qpos[:3] = [3.7, -9.0, 0.6]
    d.qpos[qpos_ids] = default_dof_pos
    
    # Set ctrl to default targets so the PD controller holds the pose
    d.ctrl[ctrl_ids[:12]] = default_dof_pos[:12]  # leg position targets
    d.ctrl[ctrl_ids[12:]] = 0.0                   # wheel velocity targets = 0
    for _ in range(500):
        mujoco.mj_step(m, d)

    # Fill history buffer
    init_obs = get_obs()
    for i in range(history_length):
        obs_history[i] = init_obs

    print("Starting control loop...")

    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            step_start = time.time()
            
            # --- Pygame UI ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    viewer.close()
                    pygame.quit()
                    return

            keys = pygame.key.get_pressed()
            cmd_vx, cmd_vy, cmd_wz = 0.0, 0.0, 0.0
            if keys[pygame.K_UP]:    cmd_vx = 1.0
            if keys[pygame.K_DOWN]:  cmd_vx = -1.0
            if keys[pygame.K_LEFT]:  cmd_vy = 0.5
            if keys[pygame.K_RIGHT]: cmd_vy = -0.5
            if keys[pygame.K_a]:     cmd_wz = 1.0
            if keys[pygame.K_d]:     cmd_wz = -1.0
            command[0] = cmd_vx
            command[1] = cmd_vy
            command[2] = cmd_wz

            screen.fill((30, 30, 30))
            screen.blit(font.render("Go2W Sim2Sim Control", True, (255, 255, 255)), (20, 20))
            screen.blit(font.render(f"VX (UP/DOWN): {cmd_vx:.1f}", True, (0, 255, 0)), (20, 60))
            screen.blit(font.render(f"VY (LEFT/RIGHT): {cmd_vy:.1f}", True, (0, 255, 0)), (20, 100))
            screen.blit(font.render(f"WZ (A/D): {cmd_wz:.1f}", True, (0, 255, 0)), (20, 140))
            screen.blit(font.render(f"Time: {d.time:.1f}s", True, (200, 200, 200)), (20, 200))
            pygame.display.flip()

            # --- Policy inference at control frequency ---
            obs = get_obs()
            # update history buffer
            obs_history = np.roll(obs_history, -1, axis=0)
            obs_history[-1] = obs

            # mjlab observation layout: flatten history per-term, then concatenate
            term_dims = [3, 3, 3, 12, 12, 4, 16]
            term_histories = np.split(obs_history, np.cumsum(term_dims)[:-1], axis=1)
            flat_obs = np.concatenate([h.flatten() for h in term_histories])
            
            # policy inference
            pi_input = torch.tensor(flat_obs, dtype=torch.float32, device=device).unsqueeze(0)

            with torch.no_grad():
                actions = policy(pi_input).squeeze(0).cpu().numpy()

            actions = np.clip(actions, -100.0, 100.0)
            last_actions[:] = actions

            # Scale and filter
            scaled = actions * action_scale
            leg_targets = lpf_legs.filter(scaled[:12])
            wheel_targets = lpf_wheels.filter(scaled[12:])

            # Apply to MuJoCo ctrl:
            # Legs: position targets (offset by default pose)
            d.ctrl[ctrl_ids[:12]] = leg_targets + default_dof_pos[:12]
            # Wheels: velocity targets
            d.ctrl[ctrl_ids[12:]] = wheel_targets

            # Step simulation (decimation steps per control step)
            for _ in range(decimation):
                mujoco.mj_step(m, d)

            viewer.sync()

            elapsed = time.time() - step_start
            sleep_time = control_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    pygame.quit()


if __name__ == "__main__":
    main()
