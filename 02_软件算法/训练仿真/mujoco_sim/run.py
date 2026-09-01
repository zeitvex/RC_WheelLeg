"""Main entry point: wheeled-legged robot simulation.

Controls:
  Mode: wheel (default) - differential drive + posture hold
        trot - quadruped gait with wheel assist
        mpc  - convex MPC locomotion (torque control)

  Keyboard (in MuJoCo viewer):
    W/S: vel_x ±0.1
    A/D: yaw_rate ±0.2
    Q/E: height ±0.02
    1: wheel mode
    2: trot mode
    3: MPC mode
    4: prone toggle
    Z: reset commands
"""

import time
import numpy as np
import mujoco.viewer as mjv

from robot import Robot
from controller import Controller
from gui import GUI
from config import CTRL_DECIMATION


def main():
    robot = Robot()
    robot.reset()
    ctrl = Controller(robot)
    gui = GUI(ctrl)

    step = 0
    sim_steps_per_ctrl = CTRL_DECIMATION

    def key_callback(keycode):
        """Called from MuJoCo render thread - only modify ctrl directly, not tkinter."""
        try:
            c = chr(keycode).lower()
        except (ValueError, OverflowError):
            return
        if c == 'w':
            ctrl.vel_x = min(ctrl.vel_x + 0.1, 1.5)
        elif c == 's':
            ctrl.vel_x = max(ctrl.vel_x - 0.1, -1.5)
        elif c == 'a':
            ctrl.yaw_rate = min(ctrl.yaw_rate + 0.2, 2.0)
        elif c == 'd':
            ctrl.yaw_rate = max(ctrl.yaw_rate - 0.2, -2.0)
        elif c == 'q':
            ctrl.height = min(ctrl.height + 0.02, 0.45)
        elif c == 'e':
            ctrl.height = max(ctrl.height - 0.02, 0.17)
        elif c == '1':
            ctrl.mode = "wheel"; ctrl.prone = False
        elif c == '2':
            ctrl.mode = "trot"; ctrl.prone = False
        elif c == '3':
            ctrl.mode = "mpc"; ctrl.prone = False
        elif c == '4':
            ctrl.prone = not ctrl.prone
        elif c == 'z':
            ctrl.vel_x = 0.0; ctrl.vel_y = 0.0; ctrl.yaw_rate = 0.0

    with mjv.launch_passive(robot.model, robot.data, key_callback=key_callback) as viewer:
        viewer.cam.distance = 2.5
        viewer.cam.elevation = -20
        viewer.cam.azimuth = 135

        last_time = robot.data.time

        while viewer.is_running() and not gui.closed:
            t_start = time.perf_counter()

            # Detect viewer reset (Backspace) - time jumps back to 0
            if robot.data.time < last_time:
                robot.reset()
            last_time = robot.data.time

            # Get state and compute control
            state = robot.get_state()
            leg_targets, wheel_targets = ctrl.compute(state, robot.dt * sim_steps_per_ctrl)

            # Apply control and step simulation
            # MPC mode sets ctrl directly via set_ctrl_mit, skip set_ctrl
            if ctrl.mode != "mpc":
                robot.set_ctrl(leg_targets, wheel_targets)
            for _ in range(sim_steps_per_ctrl):
                robot.step()

            viewer.sync()
            step += 1

            # Update GUI every 25 steps (~10 Hz)
            if step % 25 == 0:
                state = robot.get_state()
                gui.update_status(state, step)
                if not gui.tick():
                    break

            # Real-time sync
            elapsed = time.perf_counter() - t_start
            target_dt = robot.dt * sim_steps_per_ctrl
            if elapsed < target_dt:
                time.sleep(target_dt - elapsed)


if __name__ == "__main__":
    main()
