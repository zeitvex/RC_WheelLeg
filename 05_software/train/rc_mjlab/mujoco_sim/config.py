"""Robot constants and control parameters."""

import numpy as np
from pathlib import Path

# Paths
REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_XML = REPO_ROOT / "mjcf" / "scene.xml"
MJCF_PATH = REPO_ROOT / "mjcf" / "wheelleg.xml"

# Robot geometry
WHEEL_RADIUS = 0.10  # m
WHEEL_TRACK = 0.32   # m (left-right distance)
ROBOT_MASS = 12.3    # kg
MAX_TORQUE = 17.0    # Nm per joint
MAX_JOINT_VEL = 13.0 # rad/s

# Leg link lengths (from MJCF)
L_THIGH = 0.25  # m
L_CALF = 0.20   # m (to wheel center)

# Leg names and joint ordering
LEG_NAMES = ("fl", "fr", "rl", "rr")
LEG_JOINTS = ("hip_abduction_joint", "hip_pitch_joint", "knee_joint")
WHEEL_JOINT = "wheel_joint"

# Default standing pose (from go2w_sim2sim: [0, 0.8, -1.5])
DEFAULT_JOINT_ANGLES = {
    "hip_abduction": 0.0,
    "hip_pitch": 0.93,
    "knee": -1.65,
}

# Actuator modes (MJCF native):
#   Leg joints: position PD (kp=120, kd=8), ctrl = target angle
#   Wheel joints: velocity (gain=0.5), ctrl = target velocity (rad/s)

# Control rates
SIM_DT = 0.002       # 500 Hz (from scene.xml)
CTRL_DT = 0.004      # 250 Hz control loop
CTRL_DECIMATION = int(CTRL_DT / SIM_DT)

# Wheel drive
WHEEL_VEL_MAX = 10.0  # rad/s max wheel command

# Body pose control gains (for height/roll/pitch compensation)
KP_HEIGHT = 3.0    # rad/m error → joint angle correction
KP_ROLL = 0.5      # compensation gain
KP_PITCH = 0.5     # compensation gain

# Gait parameters
GAIT_FREQ = 2.5     # Hz
GAIT_DUTY = 0.6     # stance fraction
SWING_HEIGHT = 0.06  # m

# Trot phase offsets: FL/RR in phase, FR/RL in phase
PHASE_OFFSETS = {"fl": 0.0, "fr": 0.5, "rl": 0.5, "rr": 0.0}
