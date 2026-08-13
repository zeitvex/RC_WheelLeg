from pathlib import Path

import mujoco

from mjlab.actuator import BuiltinPositionActuatorCfg, BuiltinVelocityActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

_PROJECT_ROOT = Path(__file__).parent.parent.parent
ROBOT_XML = _PROJECT_ROOT / "mjcf" / "wheelleg.xml"

LEG_JOINT_PATTERNS = (
    ".*_hip_abduction_joint",
    ".*_hip_pitch_joint",
    ".*_knee_joint",
)
WHEEL_JOINT_PATTERNS = (".*_wheel_joint",)
ALL_JOINT_PATTERNS = LEG_JOINT_PATTERNS + WHEEL_JOINT_PATTERNS


def get_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(ROBOT_XML))
    actuators_to_delete = list(spec.actuators)
    for act in actuators_to_delete:
        spec.delete(act)
    return spec


# All 16 actuators have identical physical specs: effort limit = 17 Nm, max velocity = 13 rad/s.
# Leg joints: Position PD control (referenced from HIMLoco Go2W: kp = 40, kd = 1).
STIFFNESS_LEG = 40.0
DAMPING_LEG = 1.0
EFFORT_LEG = 17.0

# Wheel joints: Velocity control with damping = 0.5 and effort limit = 17 Nm.
DAMPING_WHEEL = 0.5
EFFORT_WHEEL = 17.0

# Maximum joint speed for all actuators (rad/s)
MAX_JOINT_VEL = 13.0

LEG_ACTUATOR_CFG = BuiltinPositionActuatorCfg(
    target_names_expr=LEG_JOINT_PATTERNS,
    stiffness=STIFFNESS_LEG,
    damping=DAMPING_LEG,
    effort_limit=EFFORT_LEG,
)

WHEEL_ACTUATOR_CFG = BuiltinVelocityActuatorCfg(
    target_names_expr=WHEEL_JOINT_PATTERNS,
    damping=DAMPING_WHEEL,
    effort_limit=EFFORT_WHEEL,
)

INIT_STATE = EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.40),
    joint_pos={
        ".*_hip_abduction_joint": 0.0,
        ".*_hip_pitch_joint": 0.9,
        ".*_knee_joint": -1.8,
        ".*_wheel_joint": 0.0,
    },
    joint_vel={".*": 0.0},
)

COLLISION_CFG = CollisionCfg(
    geom_names_expr=(".*",),
    contype=0,
    conaffinity=1,
    condim={".*_wheel_Link.*": 6, ".*": 1},
    priority={".*_wheel_Link.*": 1},
    friction={".*_wheel_Link.*": (0.8, 0.05, 0.01)},
)

ARTICULATION_CFG = EntityArticulationInfoCfg(
    actuators=(LEG_ACTUATOR_CFG, WHEEL_ACTUATOR_CFG),
    soft_joint_pos_limit_factor=0.95,
)


def get_robot_cfg() -> EntityCfg:
    return EntityCfg(
        init_state=INIT_STATE,
        collisions=(COLLISION_CFG,),
        spec_fn=get_spec,
        articulation=ARTICULATION_CFG,
    )


def get_robot_crawl_cfg() -> EntityCfg:
    crawl_init_state = EntityCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.20),
        joint_pos={
            "(fl|rl)_hip_abduction_joint": 0.4,
            "(fr|rr)_hip_abduction_joint": -0.4,
            ".*_hip_pitch_joint": 1.65,
            ".*_knee_joint": -2.55,
            ".*_wheel_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    )
    return EntityCfg(
        init_state=crawl_init_state,
        collisions=(COLLISION_CFG,),
        spec_fn=get_spec,
        articulation=ARTICULATION_CFG,
    )


# Action scale: Leg target position is ±0.25 rad, wheel target velocity is ±10.0 rad/s.
# (Policy output range ±1 maps to wheel speeds ±10 rad/s, staying well within the max velocity of 13 rad/s).
LEG_POS_SCALE = 0.25
WHEEL_VEL_SCALE = 10.0
