import time
import torch
import numpy as np
import mujoco.viewer
from pathlib import Path

# 导入我们的模块化组件
from input_dev.keyboard import KeyboardCommandController
from policy.policy_runner import PolicyRunner
from interface.mujoco_io import MuJoCoIO
from tools.logger import SimpleLogger

def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # 路径设置 (兼容 rc_mjlab/sim2sim 目录结构)
    project_root = Path(__file__).parent.parent.absolute()
    terrain_dir = Path(__file__).parent / "terrain"
    terrain_xml = terrain_dir / "scene_terrain.xml"
    robot_xml = project_root / "mjcf" / "wheelleg.xml"
    policy_path = {
        "rough": project_root / "model_rough.pt",
        "crawl": project_root / "model_crawl.pt"
    }

    # 1. 初始化 MuJoCo IO 接口
    print("\n[Main] Initializing MuJoCo Environment...")
    io = MuJoCoIO(terrain_xml, robot_xml, terrain_dir)
    
    # 2. 初始化 Policy 推理层
    print("\n[Main] Initializing Policy Runner...")
    runner = PolicyRunner(policy_path, device)
    
    # 3. 初始化键盘控制器 (带平滑加减速)
    print("\n[Main] Initializing Input Controller...")
    kb = KeyboardCommandController(max_x_vel=1.0, max_yaw_vel=1.0)
    kb.start()
    
    # 4. 初始化数据日志记录器
    print("\n[Main] Initializing Data Logger...")
    logger = SimpleLogger(log_dir=str(project_root / "sim2sim"))

    # 机器人复位
    io.reset_robot(runner.default_dof_pos)
    runner.reset()

    # 时序控制计算
    control_dt = io.control_dt  # 通常是 0.02s (50Hz)
    sim_steps_per_control = int(round(control_dt / io.m.opt.timestep)) # 通常是 10
    
    next_exec_time = time.perf_counter()
    viewer_counter = 0

    print(f"\n[Main] Starting Control Loop (Control DT: {control_dt:.3f}s)")
    
    try:
        with mujoco.viewer.launch_passive(io.m, io.d) as viewer:
            # 初始相机视角
            viewer.cam.distance = 5.0
            viewer.cam.elevation = -20.0
            viewer.cam.azimuth = 45.0
            
            while viewer.is_running():
                # [1] 获取用户指令
                command = kb.get_command()
                
                # [2] 读取环境观测值
                obs = io.get_obs_53d(command, runner.default_dof_pos, runner.last_actions)
                
                # [3] 神经网络推理 (包含历史堆叠处理)
                scaled_actions, raw_actions = runner.step(obs)
                
                # [4] 下发动作到仿真器 (包含低通滤波)
                io.send_actions(scaled_actions, runner.default_dof_pos)
                
                # [5] 记录日志
                logger.update(io.d.time, io.m, io.d, command)
                
                # [6] 推进物理仿真
                for _ in range(sim_steps_per_control):
                    mujoco.mj_step(io.m, io.d)
                
                # [7] 降低渲染频率以节省性能 (25Hz 渲染)
                viewer_counter += 1
                if viewer_counter >= 2:
                    base_id = mujoco.mj_name2id(io.m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
                    if base_id != -1:
                        viewer.cam.lookat[:] = io.d.xpos[base_id]
                    viewer.sync()
                    viewer_counter = 0

                # [8] 高精度时序锁帧 (完全对标真机 RTOS 逻辑)
                next_exec_time += control_dt
                now = time.perf_counter()
                sleep_time = next_exec_time - now
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < -control_dt:
                    # 如果发生严重掉帧，重置时钟，避免疯狂快进
                    next_exec_time = now
    finally:
        print("\n[Main] Shutting down...")
        kb.stop()
        logger.save()  # 保存数据到 txt

if __name__ == "__main__":
    main()
