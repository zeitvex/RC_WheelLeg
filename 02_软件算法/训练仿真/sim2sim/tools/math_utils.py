import numpy as np

def get_gravity_orientation(quat_wxyz):
    """Compute projected gravity in body frame from quaternion [w,x,y,z]."""
    qw, qx, qy, qz = quat_wxyz
    gx = 2.0 * (-qz * qx + qw * qy)
    gy = -2.0 * (qz * qy + qw * qx)
    gz = 1.0 - 2.0 * (qw * qw + qz * qz)
    return np.array([gx, gy, gz], dtype=np.float32)

def quat_rotate_inverse(quat_wxyz, v):
    """Rotate vector v from world frame to body frame."""
    q_w = quat_wxyz[0]
    q_vec = quat_wxyz[1:]
    a = v * (2.0 * q_w * q_w - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a - b + c

def get_body_velocity(m, d):
    """Calculate body frame linear velocity (vx, vy, vz) from world velocity."""
    v_world = d.qvel[0:3]
    q = d.qpos[3:7]  # MuJoCo freejoint quaternion is at indices 3:7 (x,y,z, w,x,y,z)
    w, x, y, z = q
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if norm < 1e-6:
        return np.array([0.0, 0.0, 0.0], dtype=np.float32)
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    R = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
    ])
    v_body = R.T @ v_world
    return v_body.astype(np.float32)
