"""Robot constants and control parameters."""

from pathlib import Path

# Paths
REPO_ROOT = Path(__file__).resolve().parents[1]
SCENE_XML = REPO_ROOT / "mjcf" / "scene.xml"
MJCF_PATH = REPO_ROOT / "mjcf" / "wheelleg.xml"

# Robot geometry
WHEEL_RADIUS = 0.10  # m
WHEEL_TRACK = 0.32   # m, left-right distance
ROBOT_MASS = 12.3    # kg
MAX_TORQUE = 17.0    # Nm per joint
MAX_JOINT_VEL = 13.0 # rad/s

# Leg link lengths from MJCF, measured to wheel center.
L_THIGH = 0.25
L_CALF = 0.20

# Leg names and joint ordering
LEG_NAMES = ("fl", "fr", "rl", "rr")
LEG_JOINTS = ("hip_abduction_joint", "hip_pitch_joint", "knee_joint")
WHEEL_JOINT = "wheel_joint"

# Default standing pose aligned with the soft wheel-X height table.
# height ~= 0.37m, wheel x-offset ~= 0, peak/RMS leg torque balanced.
DEFAULT_JOINT_ANGLES = {
    "hip_abduction": 0.0,
    "hip_pitch": 0.666,
    "knee": -1.546,
}

# Actuator modes, configured at runtime:
#   Leg joints: position PD, ctrl = target angle
#   Wheel joints: velocity, ctrl = target velocity in rad/s

# Control rates
SIM_DT = 0.002
CTRL_DT = 0.02
CTRL_DECIMATION = int(CTRL_DT / SIM_DT)

# Wheel drive
WHEEL_VEL_MAX = 10.0

# Body pose control gains for height/roll/pitch compensation.
KP_HEIGHT = 3.0
KP_ROLL = 0.5
KP_PITCH = 0.5

# Calibrated height-to-joint-angle table.
# Constraint: avoid large wheel-center X offset from the hip/leg. This is a
# soft support-geometry guardrail, not a strict x=0 requirement.
# The optimizer also considers peak motor torque and RMS torque, so one hot
# motor is not hidden by a low average across all motors.
# Format: (height_m, hip_pitch_rad, knee_rad)
HEIGHT_TABLE = [
    (0.17, 0.914, -2.628),
    (0.19, 0.926, -2.528),
    (0.21, 0.924, -2.428),
    (0.23, 0.912, -2.328),
    (0.25, 0.892, -2.226),
    (0.27, 0.864, -2.122),
    (0.29, 0.834, -2.014),
    (0.31, 0.798, -1.906),
    (0.33, 0.758, -1.792),
    (0.35, 0.714, -1.672),
    (0.37, 0.666, -1.546),
    (0.39, 0.612, -1.412),
    (0.41, 0.552, -1.266),
    (0.43, 0.484, -1.104),
    (0.45, 0.404, -0.918),
]

# Gait parameters
GAIT_FREQ = 2.5
GAIT_DUTY = 0.6
SWING_HEIGHT = 0.06

# Trot phase offsets: FL/RR in phase, FR/RL in phase
PHASE_OFFSETS = {"fl": 0.0, "fr": 0.5, "rl": 0.5, "rr": 0.0}
