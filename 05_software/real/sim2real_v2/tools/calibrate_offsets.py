"""零位偏移标定向导。

用途：把机器人摆到 sim2sim/训练侧的 stand 默认姿态（人工摆好），
跑这个脚本，它会读 16 个电机的当前位置，反算每个电机的 ZERO_OFFSET。

关键公式（与 motor_mapping.py 一致）：
    real = sign * sim + offset
当 sim = stand_default 时：
    offset = real - sign * stand_default

⚠️ 使用前置条件：
1. 已运行过 motor_driver_direction_test 类的脚本，确认每个电机的 sign 是对的；
   sign 错的话本工具会算出错误的 offset 看起来很对，但发动作时机器人会反向冲撞
2. 机器人物理上摆到 stand 姿态：四条腿微弯曲、轮子接地、机身水平
3. 电机已 enable 并清除告警

输出：把打印出来的 ZERO_OFFSET_MAP 字段直接覆盖 motor_mapping.py 中的对应字典。
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from interface.motor_mapping import MotorMapping  # noqa: E402
from policy.policy_runner import PolicyRunner  # noqa: E402


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--can1-port", default="/dev/can1")
    parser.add_argument("--can2-port", default="/dev/can2")
    parser.add_argument("--motor-model", default="rs-02")
    parser.add_argument("--samples", type=int, default=100,
                        help="平均采样帧数（去抖动）")
    parser.add_argument("--target-pose", default="stand", choices=["stand", "crawl"],
                        help="标定时机器人摆的物理姿态")
    parser.add_argument("--no-enable", action="store_true",
                        help="不主动 enable 电机（仅读取，适合手动转关节标定）")
    args = parser.parse_args()

    # 真机驱动注入（路径优先级与 main.py 一致：vendored/drivers > /home/rc2/...）
    sim2real_root = Path(__file__).resolve().parents[1]
    for path in (sim2real_root / "vendored",
                 "/home/rc2/work/rcwork/control",
                 "/home/rc2/work/rcwork"):
        sp = str(path)
        if sp not in sys.path and Path(path).exists():
            sys.path.append(sp)
    from drivers.motor_driver import RobStrideDriver  # type: ignore

    mapper = MotorMapping()
    drv1 = RobStrideDriver(args.can1_port, debug=False)
    drv2 = RobStrideDriver(args.can2_port, debug=False)
    drv1.connect()
    drv2.connect()

    for jk in mapper.SIM_JOINT_ORDER:
        leg, joint = jk
        bus, mid = mapper.CAN_ID_MAP[jk]
        name = f"{leg}_{joint}"
        (drv1 if bus == 1 else drv2).add_motor(name, mid, args.motor_model)

    if not args.no_enable:
        print("[Calib] Enable 电机以读取状态...（已就位则可加 --no-enable 跳过）")
        for drv in (drv1, drv2):
            for name in drv.motors:
                drv.clear_warnings(name)
                drv.enable(name)
        time.sleep(0.5)

    # 选择标定姿态
    if args.target_pose == "stand":
        sim_pose = PolicyRunner.DEFAULT_STAND_POSE.copy()  # [0,0.9,-1.8] x4 + zeros
    else:
        sim_pose = np.array([
            0.4, 1.65, -2.55, -0.4, 1.65, -2.55,
            0.4, 1.65, -2.55, -0.4, 1.65, -2.55,
            0.0, 0.0, 0.0, 0.0,
        ], dtype=np.float32)

    print(f"\n[Calib] 请把机器人物理摆成 {args.target_pose.upper()} 姿态：")
    if args.target_pose == "stand":
        print("  四条腿髋外展=0, 髋俯仰=0.9rad(~52°), 膝=-1.8rad(~-103°), 轮接地")
    else:
        print("  内收外展 ±0.4rad, 髋俯仰=1.65rad, 膝=-2.55rad（深蹲下趴）")
    print("  轮子可以保持任意角度，offset 强制为 0")
    print("  按回车开始采样...")
    try:
        input()
    except EOFError:
        pass

    print(f"\n[Calib] 开始采样 {args.samples} 帧并平均...")
    pos_acc = np.zeros(16, dtype=np.float64)
    valid = 0
    for i in range(args.samples):
        drv1.process_messages()
        drv2.process_messages()
        real_pos = {}
        for drv_idx, drv in enumerate((drv1, drv2)):
            bus = drv_idx + 1
            for name, motor in drv.motors.items():
                parts = name.split("_", 1)
                if len(parts) != 2:
                    continue
                key = (parts[0], parts[1])
                if key not in mapper.CAN_ID_MAP:
                    continue
                _, mid = mapper.CAN_ID_MAP[key]
                real_pos[(bus, mid)] = motor.state.position
        if len(real_pos) == 16:
            ordered = np.array([real_pos[mapper.CAN_ID_MAP[jk]]
                                for jk in mapper.SIM_JOINT_ORDER], dtype=np.float64)
            pos_acc += ordered
            valid += 1
        time.sleep(0.02)

    if valid < args.samples * 0.5:
        print(f"[Calib] 警告: 只收到 {valid}/{args.samples} 帧反馈，标定可能不可靠")
    real_avg = pos_acc / max(valid, 1)

    # 反算 offset：offset = real - sign * sim
    sign = mapper._sign
    offsets = real_avg - sign * sim_pose

    # 轮子 offset 强制 0
    for i, jk in enumerate(mapper.SIM_JOINT_ORDER):
        if jk[1] == "wheel":
            offsets[i] = 0.0

    # 打印结果（按 motor_mapping.py 的字典格式）
    print("\n" + "=" * 64)
    print(f"[Calib] 标定完成（{valid} 帧平均）")
    print("=" * 64)
    print("把以下字典覆盖 sim2real/interface/motor_mapping.py 中的 ZERO_OFFSET_MAP:")
    print()
    print("    ZERO_OFFSET_MAP = {")
    for i, jk in enumerate(mapper.SIM_JOINT_ORDER):
        leg, joint = jk
        cur = offsets[i]
        old = mapper.ZERO_OFFSET_MAP[jk]
        delta = cur - old
        marker = " *" if abs(delta) > 0.01 else ""
        print(f'        ("{leg}", "{joint:13s}"): {cur:>+8.4f},  '
              f'# old={old:+.4f} delta={delta:+.4f}{marker}')
    print("    }")
    print("\n标记 * 的项与现表偏离 > 0.01 rad，请重点核对该关节的 sign 是否正确。\n")

    # Disable
    if not args.no_enable:
        for drv in (drv1, drv2):
            for name in drv.motors:
                drv.disable(name)
    drv1.disconnect()
    drv2.disconnect()


if __name__ == "__main__":
    main()
