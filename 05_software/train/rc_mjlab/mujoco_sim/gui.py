"""GUI control panel for the wheeled-legged robot."""

import tkinter as tk
from tkinter import ttk


class GUI:
    """Tkinter control panel: sliders + gait buttons + status display."""

    def __init__(self, controller):
        self.ctrl = controller
        self.root = tk.Tk()
        self.root.title("WheelLeg Control")
        self.root.geometry("400x500")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._closed = False

        self._build()

    def _build(self):
        # Mode buttons
        mf = ttk.LabelFrame(self.root, text="Mode")
        mf.pack(fill="x", padx=8, pady=4)
        for mode in ("wheel", "trot", "mpc"):
            ttk.Button(mf, text=mode.upper(),
                       command=lambda m=mode: self._set_mode(m)
                       ).pack(side="left", padx=4, expand=True)
        ttk.Button(mf, text="PRONE/STAND",
                   command=self._toggle_prone).pack(side="left", padx=4, expand=True)

        # Command sliders
        cf = ttk.LabelFrame(self.root, text="Commands")
        cf.pack(fill="x", padx=8, pady=4)

        self.vel_x_var = tk.DoubleVar(value=0.0)
        self.vel_y_var = tk.DoubleVar(value=0.0)
        self.yaw_var = tk.DoubleVar(value=0.0)
        self.height_var = tk.DoubleVar(value=self.ctrl.height)

        self._slider(cf, "Vel X", self.vel_x_var, -1.5, 1.5)
        self._slider(cf, "Vel Y*", self.vel_y_var, -0.5, 0.5)
        self._slider(cf, "Yaw", self.yaw_var, -2.0, 2.0)
        self._slider(cf, "Height", self.height_var, 0.17, 0.45)

        ttk.Label(cf, text="* Vel Y: trot mode only (diff-drive can't sidestep)",
                  font=("", 8)).pack(anchor="w", padx=8)

        ttk.Button(cf, text="Reset", command=self._reset).pack(pady=4)

        # Status display
        sf = ttk.LabelFrame(self.root, text="Status")
        sf.pack(fill="both", expand=True, padx=8, pady=4)
        self.status_text = tk.Text(sf, height=12, width=45, font=("Consolas", 9))
        self.status_text.pack(fill="both", expand=True, padx=4, pady=4)

    def _slider(self, parent, label, var, lo, hi):
        f = ttk.Frame(parent)
        f.pack(fill="x", padx=4, pady=2)
        ttk.Label(f, text=label, width=7).pack(side="left")
        ttk.Scale(f, from_=lo, to=hi, variable=var,
                  command=lambda *_: self._sync()).pack(side="left", fill="x", expand=True)
        lbl = ttk.Label(f, text="0.00", width=6)
        lbl.pack(side="left")
        var.trace_add("write", lambda *_, v=var, l=lbl: l.config(text=f"{v.get():.2f}"))

    def _set_mode(self, mode):
        self.ctrl.mode = mode
        self.ctrl.prone = False

    def _toggle_prone(self):
        self.ctrl.prone = not self.ctrl.prone

    def _sync(self):
        self.ctrl.vel_x = self.vel_x_var.get()
        self.ctrl.vel_y = self.vel_y_var.get()
        self.ctrl.yaw_rate = self.yaw_var.get()
        self.ctrl.height = self.height_var.get()

    def _reset(self):
        self.vel_x_var.set(0.0)
        self.vel_y_var.set(0.0)
        self.yaw_var.set(0.0)
        self._sync()

    def _on_close(self):
        self._closed = True
        self.root.destroy()

    @property
    def closed(self):
        return self._closed

    def update_status(self, state, step):
        """Update status text with current robot state."""
        txt = (
            f"Mode: {self.ctrl.mode}  Step: {step}\n"
            f"Pos:  x={state.pos[0]:.3f} y={state.pos[1]:.3f} z={state.pos[2]:.3f}\n"
            f"RPY:  r={np.degrees(state.rpy[0]):.1f}° p={np.degrees(state.rpy[1]):.1f}° "
            f"y={np.degrees(state.rpy[2]):.1f}°\n"
            f"Vel:  vx={state.lin_vel[0]:.3f} vy={state.lin_vel[1]:.3f} vz={state.lin_vel[2]:.3f}\n"
            f"Cmd:  vx={self.ctrl.vel_x:.2f} yaw={self.ctrl.yaw_rate:.2f} h={self.ctrl.height:.3f}\n"
            f"─────────────────────────────────\n"
        )
        # Joint angles (compact)
        for i, leg in enumerate(("FL", "FR", "RL", "RR")):
            q = state.joint_pos[i*4:i*4+3]
            w = state.joint_vel[i*4+3]
            txt += f"{leg}: [{q[0]:+.2f} {q[1]:+.2f} {q[2]:+.2f}] w={w:+.1f}\n"

        self.status_text.delete("1.0", tk.END)
        self.status_text.insert(tk.END, txt)

    def tick(self):
        """Process GUI events. Returns False if window closed."""
        if self._closed:
            return False
        try:
            self.root.update_idletasks()
            self.root.update()
            return True
        except tk.TclError:
            self._closed = True
            return False


# Need numpy for degrees conversion in update_status
import numpy as np
