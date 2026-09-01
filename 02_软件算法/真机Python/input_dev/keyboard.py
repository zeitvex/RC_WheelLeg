"""键盘控制器 — 兼容 sim2sim/input_dev/keyboard.py 的接口与平滑参数。"""
import numpy as np

try:
    from pynput import keyboard
    PYNPUT_AVAILABLE = True
except ImportError:
    PYNPUT_AVAILABLE = False
    keyboard = None  # type: ignore


class KeyboardCommandController:
    """方向键 + AD 键的键盘指令源。
    指令: [vx, vy, yaw_rate]，平滑加减速；空格触发急停标志。
    """

    def __init__(self,
                 max_x_vel: float = 0.8,
                 max_y_vel: float = 0.3,
                 max_yaw_vel: float = 0.5,
                 acc_step: float = 0.05,
                 dec_step: float = 0.1):
        if not PYNPUT_AVAILABLE:
            raise RuntimeError("pynput 不可用，无法使用键盘控制；改用其他输入源。")

        self.current_cmd = np.zeros(3, dtype=np.float32)
        self.max_x_vel = max_x_vel
        self.max_y_vel = max_y_vel
        self.max_yaw_vel = max_yaw_vel
        self.acc_step = acc_step
        self.dec_step = dec_step

        self._pressed = set()
        self._estop = False
        self.listener = keyboard.Listener(
            on_press=self._on_press, on_release=self._on_release
        )

    def start(self):
        self.listener.start()
        print("[Keyboard] 启动。↑↓ 前后, ←→ 转向, A/D 横移, SPACE 急停")

    def stop(self):
        try:
            self.listener.stop()
        except Exception:
            pass

    def _on_press(self, key):
        self._pressed.add(key)
        if key == keyboard.Key.space:
            self._estop = True

    def _on_release(self, key):
        self._pressed.discard(key)

    def is_estop_triggered(self) -> bool:
        return self._estop

    def reset_estop(self):
        self._estop = False

    def get_command(self) -> np.ndarray:
        target = np.zeros(3, dtype=np.float32)
        if keyboard.Key.up in self._pressed:
            target[0] += self.max_x_vel
        if keyboard.Key.down in self._pressed:
            target[0] -= self.max_x_vel
        if keyboard.Key.left in self._pressed:
            target[2] += self.max_yaw_vel
        if keyboard.Key.right in self._pressed:
            target[2] -= self.max_yaw_vel
        try:
            if keyboard.KeyCode.from_char('a') in self._pressed:
                target[1] += self.max_y_vel
            if keyboard.KeyCode.from_char('d') in self._pressed:
                target[1] -= self.max_y_vel
        except Exception:
            pass

        for i, max_v in enumerate((self.max_x_vel, self.max_y_vel, self.max_yaw_vel)):
            step = self.acc_step if target[i] != 0 else self.dec_step
            if i == 2:
                step *= 2.0
            if self.current_cmd[i] < target[i]:
                self.current_cmd[i] = min(self.current_cmd[i] + step, target[i])
            else:
                self.current_cmd[i] = max(self.current_cmd[i] - step, target[i])
        return self.current_cmd.copy()
