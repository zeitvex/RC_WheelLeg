from pynput import keyboard

class KeyboardCommandController:
    """Provides smoothed velocity commands via keyboard input."""
    def __init__(self, max_x_vel=2.0, max_yaw_vel=2.0, acc_step=0.05, dec_step=0.1):
        self.current_cmd = [0.0, 0.0, 0.0]
        self.max_x_vel = max_x_vel
        self.max_yaw_vel = max_yaw_vel
        self.acc_step = acc_step
        self.dec_step = dec_step
        self.pressed_keys = set()
        
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )

    def start(self):
        self.listener.start()
        print("[InputDev] Keyboard control active: UP(fwd) DOWN(bwd) LEFT(yawL) RIGHT(yawR)")

    def stop(self):
        self.listener.stop()

    def on_press(self, key):
        self.pressed_keys.add(key)

    def on_release(self, key):
        if key in self.pressed_keys:
            self.pressed_keys.remove(key)

    def get_command(self):
        target_vx = 0.0
        target_dyaw = 0.0
        
        if keyboard.Key.up in self.pressed_keys:
            target_vx += self.max_x_vel
        if keyboard.Key.down in self.pressed_keys:
            target_vx -= self.max_x_vel
        if keyboard.Key.left in self.pressed_keys:
            target_dyaw += self.max_yaw_vel
        if keyboard.Key.right in self.pressed_keys:
            target_dyaw -= self.max_yaw_vel

        # Smooth X velocity
        step_x = self.acc_step if target_vx != 0 else self.dec_step
        if self.current_cmd[0] < target_vx:
            self.current_cmd[0] = min(self.current_cmd[0] + step_x, target_vx)
        elif self.current_cmd[0] > target_vx:
            self.current_cmd[0] = max(self.current_cmd[0] - step_x, target_vx)

        # Smooth Yaw velocity
        step_yaw = self.acc_step * 2.0 if target_dyaw != 0 else self.dec_step * 2.0
        if self.current_cmd[2] < target_dyaw:
            self.current_cmd[2] = min(self.current_cmd[2] + step_yaw, target_dyaw)
        elif self.current_cmd[2] > target_dyaw:
            self.current_cmd[2] = max(self.current_cmd[2] - step_yaw, target_dyaw)

        return self.current_cmd.copy()
