#!/usr/bin/env python3
"""
nav_sim2sim.py: 机器人高精度绝对惯导 2D 交互导航系统 & 实时遥测仪表盘。

特性：
  - 动态 XML 地图解析引擎：自动读取并解析 scene_terrain.xml 中的所有 geom 障碍物，实现 100% 地图画面与物理环境精准对应。
  - 极佳的可视化表现：支持旋转 box 顶点多边形解算、柱状 cylinder 绘制以及 hfield 区域半透明渲染，支持鼠标滚轮缩放与右键拖拽。
    - 鼠标手绘轨迹巡航 (Path Drawing)：长按左键即可在地图上画出任意曲线，系统智能离散化为航点，控制机器人顺畅驶完轨迹。
    - 纯坐标触发自适应限高爬行：不依赖高度传感器，依据全局 IMU 坐标范围自动下蹲过杆。
  - 关键点记录与指令微调：支持 R 键打点、TXT 文件导出以及命令行（如 "go 3.5 -6.0"）直接发令控制。
  - 全界面中文本地化。
"""

import os
import sys
import time
import math
import torch
import numpy as np
import pygame
import xml.etree.ElementTree as ET
from pathlib import Path

# 插入局部路径，确保模块导入顺畅
sys.path.append(str(Path(__file__).parent.absolute()))

from interface.mujoco_io import MuJoCoIO
from policy.policy_runner import PolicyRunner
import mujoco
import mujoco.viewer

# ==============================================================================
# HSL / RGB 科技感深色调配色系统
# ==============================================================================
COLOR_BG = (10, 15, 30)             # Slate 950 深邃星空蓝
COLOR_GRID = (22, 29, 48)           # 浅灰色网格线
COLOR_AXIS = (38, 50, 78)           # 主轴线
COLOR_HUD_BG = (18, 24, 42)         # 半透明磨砂控制面板背景
COLOR_HUD_BORDER = (38, 52, 84)     # 边框线
COLOR_TEXT_LIGHT = (248, 250, 252)  # 高对比度白色文字
COLOR_TEXT_MUTED = (148, 163, 184)  # 灰色辅助说明文字
COLOR_EMERALD = (16, 185, 129)      # 翡翠绿：自动模式 / 正常状态
COLOR_ROSE = (244, 63, 94)          # 玫瑰红：手动模式 / 警告状态
COLOR_AMBER = (245, 158, 11)        # 琥珀黄：高墙 / 爬坡提示
COLOR_GOLD = (234, 179, 8)          # 亮金色：导航目标点
COLOR_CYAN = (6, 182, 212)          # 霓虹青：机器人本体与行进轨迹
COLOR_PURPLE = (139, 92, 246)       # 科技紫：关键点标记与 hfield 区域
COLOR_CONSOLE_BG = (7, 10, 19)      # 控制台终端深黑色背景

# ==============================================================================
# 动态 XML 地图解析引擎
# ==============================================================================
def parse_xml_obstacles(xml_path):
    """
    解析 scene_terrain.xml，动态提取所有的 geom 障碍物、平台、限高柱。
    支持提取：pos, size, type, quat, rgba, name, 碰撞屏蔽(contype/conaffinity)。
    """
    obstacles = []
    hfields = {}
    
    if not os.path.exists(xml_path):
        print(f"[XML 解析器] 警告：未找到 XML 地图文件: {xml_path}")
        return obstacles

    try:
        tree = ET.parse(xml_path)
        root = tree.getroot()
        
        # 1. 扫描并缓存所有 hfield 几何参数
        for hf in root.iter('hfield'):
            hf_name = hf.get('name')
            hf_size_str = hf.get('size')
            if hf_name and hf_size_str:
                try:
                    hfields[hf_name] = [float(x) for x in hf_size_str.split()]
                except ValueError:
                    continue
                    
        # 2. 解析所有的 geom 碰撞体
        for geom in root.iter('geom'):
            g_name = geom.get('name', 'geom')
            # 过滤地面
            if g_name == 'floor':
                continue
                
            g_type = geom.get('type')
            pos_str = geom.get('pos')
            size_str = geom.get('size')
            quat_str = geom.get('quat', '1 0 0 0')
            rgba_str = geom.get('rgba', '0.7 0.7 0.7 1')
            contype = geom.get('contype', '1')
            conaffinity = geom.get('conaffinity', '1')
            hfield_ref = geom.get('hfield')
            
            if not pos_str or not g_type:
                continue
                
            try:
                pos = [float(x) for x in pos_str.split()]
                quat = [float(x) for x in quat_str.split()]
                rgba = [float(x) for x in rgba_str.split()]
                
                # 如果是 hfield，则从缓存的 hfield 大小中提取尺寸
                if g_type == 'hfield' and hfield_ref in hfields:
                    size = hfields[hfield_ref]
                else:
                    size = [float(x) for x in size_str.split()] if size_str else []
            except ValueError:
                continue
                
            # contype="0" 且 conaffinity="0" 表示仅渲染不发生碰撞的指示标
            collidable = (contype != '0') and (conaffinity != '0')
            
            obstacles.append({
                'name': g_name,
                'type': g_type,
                'pos': pos,
                'size': size,
                'quat': quat,
                'rgba': rgba,
                'collidable': collidable
            })
            
        print(f"[XML 解析器] 成功加载地图：共解析出 {len(obstacles)} 个几何元素，已完成 100% 位置对应。")
    except Exception as e:
        print(f"[XML 解析器] 解析 XML 时发生错误: {e}")
        
    return obstacles


# ==============================================================================
# 数学计算工具与姿态解算
# ==============================================================================
def quat_to_euler(quat_wxyz):
    """将 MuJoCo 的 wxyz 四元数转换为机身欧拉角 Roll, Pitch, Yaw (弧度)。"""
    qw, qx, qy, qz = quat_wxyz
    
    # 横滚角 Roll (x轴自转)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # 俯仰角 Pitch (y轴摆动)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(np.clip(sinp, -1.0, 1.0))
    
    # 偏航角 Yaw (z轴航向)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


# ==============================================================================
# 自动巡航控制器
# ==============================================================================
class NavController:
    def __init__(self, io):
        self.io = io
        self.mode = "MANUAL"  # "MANUAL" (手动键盘) 或 "AUTO" (自动导航)
        
        # 导航目标点
        self.target_x = None
        self.target_y = None
        self.target_z = 0.0
        
        # 预设巡航任务状态
        self.mission_name = "无"
        self.waypoints = []   # list of (x, y, speed, policy)
        self.waypoint_idx = 0
        self.drawn_path = []  # 记录当前整条规划路径，用于 2D 高亮线段绘制
        
        # 关键点记录表
        self.recorded_keypoints = []
        
        # APF 动态人工势场避障列表（将在主程序启动时根据 XML 解析结果动态填充）
        self.obstacles = []

    def start_mission(self, name, waypoints):
        self.mode = "AUTO"
        self.mission_name = name
        self.waypoints = waypoints
        self.waypoint_idx = 0
        self.drawn_path = [(wp[0], wp[1]) for wp in waypoints]
        if waypoints:
            self.target_x, self.target_y = waypoints[0][0], waypoints[0][1]
            print(f"[导航系统] 启动巡航任务：{name}。前往第 1 个航点: ({self.target_x:.2f}, {self.target_y:.2f})")
        else:
            self.target_x, self.target_y = None, None

    def cancel_mission(self):
        self.mode = "MANUAL"
        self.mission_name = "无"
        self.waypoints = []
        self.waypoint_idx = 0
        self.target_x, self.target_y = None, None
        self.drawn_path = []
        print("[导航系统] 自动巡航已取消，机器人切换回手动控制模式。")

    def update_navigation(self, rx, ry, ryaw, runner, dt=0.02):
        """
        核心控制计算逻辑：基于 PD 路径寻迹、自适应转向锁定与边界感知避障势场。
        """
        if self.mode != "AUTO" or self.target_x is None or self.target_y is None:
            return 0.0, 0.0, 0.0

        # ----------------------------------------------------------------------
        # 正常的 PD 路径跟随算法
        # ----------------------------------------------------------------------
        dx = self.target_x - rx
        dy = self.target_y - ry
        dist = math.sqrt(dx*dx + dy*dy)
        
        # 判定抵达航点
        if dist < 0.15:
            if len(self.waypoints) > 0 and self.waypoint_idx < len(self.waypoints) - 1:
                self.waypoint_idx += 1
                next_wp = self.waypoints[self.waypoint_idx]
                self.target_x, self.target_y = next_wp[0], next_wp[1]
                print(f"[导航系统] 顺利抵达航点 WP {self.waypoint_idx}。下一个航点: ({self.target_x:.2f}, {self.target_y:.2f})")
                return 0.0, 0.0, 0.0
            else:
                print("[导航系统] 自动巡航顺利走完！已平稳停留在终点。")
                self.mode = "MANUAL"
                self.mission_name = "无"
                self.target_x, self.target_y = None, None
                return 0.0, 0.0, 0.0

        # 提取当前子航点的运行速度限制
        speed_limit = 0.5
        if self.waypoints and self.waypoint_idx < len(self.waypoints):
            wp = self.waypoints[self.waypoint_idx]
            if len(wp) >= 3:
                speed_limit = wp[2]

        # ----------------------------------------------------------------------
        # 根据当前绝对地理坐标动态分配具体速度上限 (全局峰值限制为 1.2 m/s)
        # ----------------------------------------------------------------------
        dynamic_speed_limit = 0.6  # 默认平地过渡速度
        
        # 1. 高越障墙冲越区 (拉满至峰值 1.2 m/s 以获取最大过墙动能)
        if 1.0 <= rx <= 2.6 and -8.5 <= ry <= -5.0:
            dynamic_speed_limit = 1.2
        # 2. 限高低姿爬行区 (下蹲慢行，安全速度 0.3 m/s)
        elif 4.8 <= rx <= 6.4 and -9.5 <= ry <= -8.5:
            dynamic_speed_limit = 0.3
        # 3. 蛇形绕障避险区 (精密控弯，平稳速度 0.4 m/s)
        elif 0.5 <= rx <= 3.5 and -14.0 <= ry <= -9.5:
            dynamic_speed_limit = 0.4
        # 4. 台阶独木桥跨越区 (攀爬跨越，精细速度 0.4 m/s)
        elif 1.0 <= rx <= 4.5 and -5.0 <= ry <= 0.0:
            dynamic_speed_limit = 0.4
            
        # 综合考虑航点限速、地理限速与 1.2 m/s 全局硬上限
        speed_limit = min(speed_limit, dynamic_speed_limit, 1.2)

        # ----------------------------------------------------------------------
        # 直接路径寻迹 (已按需彻底移除人工势场避障功能)
        # ----------------------------------------------------------------------
        # 最终目标航向角 (直接朝着目标点行驶)
        target_yaw = math.atan2(dy, dx)
        
        # 航向偏差角度规范化至 [-pi, pi]
        yaw_err = (target_yaw - ryaw + math.pi) % (2.0 * math.pi) - math.pi
        
        # 自适应转向 PD 控制
        Kp_yaw = 1.8
        cmd_wz = Kp_yaw * yaw_err
        cmd_wz = np.clip(cmd_wz, -1.0, 1.0)
        
        # 转向优先自锁机制：当航向误差过大时（>45度），停止向前行进，全力原地自转对准
        if abs(yaw_err) > 0.8:
            cmd_vx = 0.0
        else:
            if "高墙" in self.mission_name or "charge" in self.mission_name.lower():
                # 冲越复杂障碍物（如高越障墙）时，不随距离缩减速度，保持全速冲越！
                cmd_vx = speed_limit
            else:
                Kp_dist = 0.8
                cmd_vx = Kp_dist * dist * math.cos(yaw_err)
            cmd_vx = np.clip(cmd_vx, -0.2, speed_limit)
            
        return float(cmd_vx), 0.0, float(cmd_wz)


# ==============================================================================
# UI 组件：现代化圆角高光按钮
# ==============================================================================
class PygameButton:
    def __init__(self, rect, text, bg_color, hover_color, text_color):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.bg_color = bg_color
        self.hover_color = hover_color
        self.text_color = text_color
        
    def draw(self, surface, font):
        mouse_pos = pygame.mouse.get_pos()
        color = self.hover_color if self.rect.collidepoint(mouse_pos) else self.bg_color
        pygame.draw.rect(surface, color, self.rect, border_radius=6)
        pygame.draw.rect(surface, COLOR_HUD_BORDER, self.rect, width=1, border_radius=6)
        
        text_surf = font.render(self.text, True, self.text_color)
        text_rect = text_surf.get_rect(center=self.rect.center)
        surface.blit(text_surf, text_rect)
        
    def is_clicked(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            return self.rect.collidepoint(event.pos)
        return False


# ==============================================================================
# 主循环与渲染主程序
# ==============================================================================
def main():
    # 检测计算卡与渲染配置
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"[主程序] 正在使用计算设备: {device}")

    # 路径解析
    project_root = Path(__file__).parent.parent.absolute()
    terrain_dir = Path(__file__).parent / "terrain"
    terrain_xml = terrain_dir / "scene_terrain.xml"
    robot_xml = project_root / "mjcf" / "wheelleg.xml"
    rough_onnx = project_root / "model_6800.onnx"
    crawl_onnx = project_root / "model_crawl.onnx"
    policy_path = {
        "rough": rough_onnx if rough_onnx.exists() else project_root / "model_rough.pt",
        "crawl": crawl_onnx if crawl_onnx.exists() else project_root / "model_crawl.pt"
    }

    # 1. 解析 XML 地图障碍物，实现 100% 可视化精准对应
    print("\n[主程序] 解析 scene_terrain.xml 实景物理元素...")
    xml_obstacles = parse_xml_obstacles(terrain_xml)

    # 2. 初始化 MuJoCo 物理接口与神经网络策略
    print("\n[主程序] 正在编译 MuJoCo 物理环境模型...")
    io = MuJoCoIO(terrain_xml, robot_xml, terrain_dir)
    
    print("\n[主程序] 载入神经网络策略驱动模型...")
    runner = PolicyRunner(policy_path, device)

    # 初始化机器人姿态
    io.reset_robot(runner.default_dof_pos)
    runner.reset()

    # 高频时序同步
    control_dt = io.control_dt  # 0.02s
    sim_steps_per_control = int(round(control_dt / io.m.opt.timestep)) # 10

    # 初始化自建巡航控制器
    nav = NavController(io)
    
    # 动态把从 XML 解析出来的碰撞体积 geom，自动注册装配到导航避障势场列表中（剔除翻越的高墙）
    for obs in xml_obstacles:
        # 1. 排除出发区
        if obs['name'] == 'spawn_zone' or not obs['collidable']:
            continue
        # 2. 排除位于 Y = -7.0 处的 30cm 高越障墙，实现直接加速跨越冲锋
        if abs(obs['pos'][1] - (-7.0)) < 0.1:
            continue
            
        # 根据几何形状计算其近似避障边界外切半径 o_rad
        cx, cy = obs['pos'][0], obs['pos'][1]
        if obs['type'] == 'cylinder':
            o_rad = obs['size'][0]
        elif obs['type'] == 'box':
            o_rad = math.sqrt(obs['size'][0]**2 + obs['size'][1]**2)
        else:
            o_rad = 0.3 # 默认缺省值
            
        nav.obstacles.append((cx, cy, o_rad, obs['name']))
        
    print(f"[避障系统] 已成功动态注册 {len(nav.obstacles)} 个避障势场障碍体。高越障墙已做冲越通行处理。")

    # ==========================================================================
    # Pygame 图形化仪表盘窗口设置
    # ==========================================================================
    pygame.init()
    pygame.font.init()
    
    WINDOW_WIDTH, WINDOW_HEIGHT = 1024, 768
    MAP_WIDTH = 680  # 稍微缩减地图面幅，给右侧 HUD 面板腾出充足排版空间
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("Robot High-Precision Absolute Navigation & Real-time Telemetry Dashboard")
    
    # 中文兼容与高解析度字体加载系统，直接加载 Windows 系统字体文件，杜绝方框（Tofu 乱码）现象
    def get_chinese_font(size, bold=False):
        # 优先读取系统物理路径下的字体文件，彻底避开 pygame.font.SysFont Silently-Fail Bug
        font_paths = [
            "C:/Windows/Fonts/msyhbd.ttc" if bold else "C:/Windows/Fonts/msyh.ttc",
            "C:/Windows/Fonts/msyh.ttc",
            "C:/Windows/Fonts/simhei.ttf",
            "C:/Windows/Fonts/simsun.ttc",
        ]
        for path in font_paths:
            if os.path.exists(path):
                try:
                    return pygame.font.Font(path, size)
                except Exception:
                    continue
        
        # 备用 SysFont 尝试
        for font_name in ["microsoftyahei", "微软雅黑", "simhei", "黑体", "simsun", "宋体"]:
            try:
                return pygame.font.SysFont(font_name, size, bold=bold)
            except Exception:
                continue
                
        # 最终安全降级
        return pygame.font.Font(None, size)

    font_large = get_chinese_font(22, bold=True)
    font_medium = get_chinese_font(16, bold=True)
    font_small = get_chinese_font(13, bold=False)
    font_mono = get_chinese_font(13, bold=False)

    # 视口状态量
    zoom = 30.0          # 物理单位到屏幕像素倍数 (像素/米)
    pan_x = 0.0          # pan平移偏移量
    pan_y = 0.0
    is_dragging = False
    drag_start_x = 0
    drag_start_y = 0
    auto_center = True
    sim_speed_factor = 1.0  # 仿真物理流速倍率

    # 轨迹面包屑历史缓存
    robot_trail = []
    max_trail_len = 250

    # 手绘轨迹变量
    draw_mode = False
    is_drawing_path = False
    drawn_points = []

    # HUD 侧边交互按钮排版（垂直布局与位置微调，彻底消除重叠，完美对齐分界线）
    # Column 1 X: 700, Column 2 X: 860. 宽度 140 像素
    btn_slalom = PygameButton((700, 307, 140, 26), "S形绕杆", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)
    btn_crawl = PygameButton((860, 307, 140, 26), "限高下蹲", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)
    btn_gravel = PygameButton((700, 338, 140, 26), "砂砾碎石", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)
    btn_wall = PygameButton((860, 338, 140, 26), "高墙越障", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)
    btn_stairs = PygameButton((700, 369, 140, 26), "台阶攀爬", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)
    btn_bridge = PygameButton((860, 369, 140, 26), "斜坡木桥", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)
    
    # 终极任务：大满贯 (科技紫圆角大按钮)
    btn_grand = PygameButton((700, 400, 300, 26), "障碍赛大满贯 (总任务)", (139, 92, 246), (124, 58, 237), COLOR_TEXT_LIGHT)
    
    # 系统与视图控制按钮
    btn_clear_trg = PygameButton((700, 467, 140, 26), "清除与停止", (244, 63, 94, 100), (225, 29, 72), COLOR_TEXT_LIGHT)
    btn_center_toggle = PygameButton((860, 467, 140, 26), "视角居中:开", (16, 185, 129), (5, 150, 105), COLOR_TEXT_LIGHT)
    btn_draw_mode = PygameButton((700, 498, 140, 26), "手绘模式:关", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)
    btn_speed_down = PygameButton((860, 498, 44, 26), "倍速-", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)
    btn_speed_normal = PygameButton((908, 498, 44, 26), "标准", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)
    btn_speed_up = PygameButton((956, 498, 44, 26), "倍速+", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)
    
    # 坐标直接微调控制发令键
    btn_x_plus = PygameButton((700, 565, 65, 24), "X+", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)
    btn_x_minus = PygameButton((775, 565, 65, 24), "X-", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)
    btn_y_plus = PygameButton((860, 565, 65, 24), "Y+", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)
    btn_y_minus = PygameButton((935, 565, 65, 24), "Y-", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)

    # 关键点记录按键
    btn_rec_key = PygameButton((700, 628, 95, 24), "保存位置", (139, 92, 246), (124, 58, 237), COLOR_TEXT_LIGHT)
    btn_exp_key = PygameButton((805, 628, 95, 24), "导出文件", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)
    btn_clear_key = PygameButton((910, 628, 90, 24), "清空位置", (30, 41, 59), (51, 65, 85), COLOR_TEXT_LIGHT)

    # 动态指令命令行输入窗口
    console_active = False
    console_text = ""
    console_rect = pygame.Rect(700, 708, 300, 22)

    # 绝对坐标变换函数闭包
    def world_to_screen(wx, wy):
        sx = int(MAP_WIDTH / 2.0 + wx * zoom + pan_x)
        sy = int(WINDOW_HEIGHT / 2.0 - wy * zoom + pan_y)
        return sx, sy

    def screen_to_world(sx, sy):
        wx = (sx - MAP_WIDTH / 2.0 - pan_x) / zoom
        wy = -(sy - WINDOW_HEIGHT / 2.0 - pan_y) / zoom
        return wx, wy

    # 锁频同步计数
    next_exec_time = time.perf_counter()
    clock = pygame.time.Clock()
    viewer_counter = 0

    print(f"\n[主程序] 高频控制与 2D 可视化仪表盘已启动 (采样步频: {control_dt:.3f}s)")
    
    # 拉起 MuJoCo 被动渲染视口
    with mujoco.viewer.launch_passive(io.m, io.d) as viewer:
        viewer.cam.distance = 5.0
        viewer.cam.elevation = -20.0
        viewer.cam.azimuth = 45.0
        
        running = True
        while running and viewer.is_running():
            step_start_time = time.perf_counter()
            
            # ------------------------------------------------------------------
            # 读取高精度绝对全局惯导系统 (IMU simulated readings)
            # ------------------------------------------------------------------
            rx = io.d.qpos[0]
            ry = io.d.qpos[1]
            rz = io.d.qpos[2]
            
            # 读取四元数
            quat = io.d.qpos[3:7].copy()
            roll_rad, pitch_rad, yaw_rad = quat_to_euler(quat)
            roll_deg = math.degrees(roll_rad)
            pitch_deg = math.degrees(pitch_rad)
            yaw_deg = math.degrees(yaw_rad)
            
            # 读取线速度与角速度
            vx_world = io.d.qvel[0]
            vy_world = io.d.qvel[1]
            linear_speed = math.sqrt(vx_world*vx_world + vy_world*vy_world)
            wz_rate = io.d.qvel[5]

            # 网格地图行迹更新
            if not robot_trail or math.dist((rx, ry), robot_trail[-1]) > 0.05:
                robot_trail.append((rx, ry))
                if len(robot_trail) > max_trail_len:
                    robot_trail.pop(0)

            # 居中锁定机身
            if auto_center:
                pan_x = -rx * zoom
                pan_y = ry * zoom

            # ------------------------------------------------------------------
            # 统一自适应高度与神经网络控制策略切换决策系统
            # ------------------------------------------------------------------
            # 默认维持常规站立策略
            desired_policy = "rough"
            
            # 1. 自动导航模式下，且当前航点的期望策略为 crawl (触发限高下蹲)
            if nav.mode == "AUTO" and nav.waypoints and nav.waypoint_idx < len(nav.waypoints):
                wp = nav.waypoints[nav.waypoint_idx]
                if len(wp) >= 4 and wp[3] == "crawl":
                    # 计算指向目标的方位角与偏角偏差，确保在对正后立即切换下蹲
                    dx = nav.target_x - rx
                    dy = nav.target_y - ry
                    target_yaw = math.atan2(dy, dx)
                    yaw_err = (target_yaw - yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
                    
                    # 只要车头对齐（偏角小于 10 度 / 0.18 弧度），立即在行进中提前下蹲准备过杆
                    if abs(yaw_err) < 0.18:
                        desired_policy = "crawl"
            
            # 2. 绝对物理位置安全兜底：踏入限高栏区间 (Y in [-9.5, -8.5] 且 X in [4.8, 6.4]) 强制下蹲
            if 4.8 <= rx <= 6.4 and -9.5 <= ry <= -8.5:
                desired_policy = "crawl"
                
            # 执行高度状态的敏捷切换
            if runner.current_policy_name != desired_policy and not runner.transition_in_progress:
                runner.trigger_transition(desired_policy)

            # ------------------------------------------------------------------
            # Pygame 键鼠事件与拖拽交互响应
            # ------------------------------------------------------------------
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
                # 滚轮缩放事件
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 4:    # 向上滚：放大
                        zoom = min(120.0, zoom * 1.1)
                    elif event.button == 5:  # 向下滚：缩小
                        zoom = max(8.0, zoom / 1.1)
                        
                    # 鼠标左键点击地图设置目标点
                    elif event.button == 1:
                        if event.pos[0] < MAP_WIDTH:
                            wx, wy = screen_to_world(event.pos[0], event.pos[1])
                            if draw_mode:
                                is_drawing_path = True
                                drawn_points = [(wx, wy)]
                                nav.cancel_mission()  # 清除以往规划
                            else:
                                # 正常的单点巡航目标发令
                                nav.mode = "AUTO"
                                nav.target_x = wx
                                nav.target_y = wy
                                nav.target_z = rz
                                nav.drawn_path = [(wx, wy)] # 生成一段直线指示
                                print(f"[导航系统] 用户手动指派新目标坐标: ({wx:.2f}, {wy:.2f})")
                            console_active = False
                        elif console_rect.collidepoint(event.pos):
                            console_active = True
                        else:
                            console_active = False
                            
                    # 鼠标右键拖拽地图平移
                    elif event.button == 3:
                        if event.pos[0] < MAP_WIDTH:
                            is_dragging = True
                            auto_center = False
                            drag_start_x, drag_start_y = event.pos
                            
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 3:
                        is_dragging = False
                    elif event.button == 1:
                        if is_drawing_path:
                            is_drawing_path = False
                            # 用户拖拽完鼠标，将所有坐标降采样过滤转化为离散巡航航点
                            if len(drawn_points) > 1:
                                wps = []
                                for pt in drawn_points:
                                    if not wps or math.dist(pt, (wps[-1][0], wps[-1][1])) >= 0.25:
                                        wps.append((pt[0], pt[1], 0.5, "rough"))
                                if wps:
                                    nav.start_mission("手绘自定义轨迹", wps)
                                    print(f"[导航系统] 成功加载手绘轨迹，已装配成 {len(wps)} 个精密航点。")
                            else:
                                nav.cancel_mission()
                        
                elif event.type == pygame.MOUSEMOTION:
                    if is_dragging:
                        dx_pix = event.pos[0] - drag_start_x
                        dy_pix = event.pos[1] - drag_start_y
                        pan_x += dx_pix
                        pan_y += dy_pix
                        drag_start_x, drag_start_y = event.pos
                    elif is_drawing_path:
                        if event.pos[0] < MAP_WIDTH:
                            wx, wy = screen_to_world(event.pos[0], event.pos[1])
                            # 间距大于 0.15m 时记录画线点，保证线条采样平滑
                            if not drawn_points or math.dist((wx, wy), drawn_points[-1]) > 0.15:
                                drawn_points.append((wx, wy))

                # 键盘输入事件
                elif event.type == pygame.KEYDOWN:
                    if console_active:
                        if event.key == pygame.K_RETURN:
                            cmd = console_text.strip().lower()
                            print(f"[终端控制] 捕获命令行输入: {cmd}")
                            if cmd.startswith("go "):
                                try:
                                    parts = cmd.split()
                                    cx = float(parts[1])
                                    cy = float(parts[2])
                                    nav.mode = "AUTO"
                                    nav.target_x = cx
                                    nav.target_y = cy
                                    print(f"[终端控制] 执行导航，目的地坐标: ({cx}, {cy})")
                                except:
                                    print("[终端控制] 指令语法错误！例：go <X坐标> <Y坐标>")
                            elif cmd == "clear" or cmd == "c":
                                nav.cancel_mission()
                            elif cmd == "record" or cmd == "r":
                                key_id = len(nav.recorded_keypoints) + 1
                                nav.recorded_keypoints.append({
                                    'id': key_id, 'x': rx, 'y': ry, 'z': rz, 'yaw': yaw_deg
                                })
                                print(f"[关键点] 已记录 WP{key_id}：({rx:.2f}, {ry:.2f})")
                            elif cmd == "crawl":
                                runner.trigger_transition("crawl")
                            elif cmd == "rough":
                                runner.trigger_transition("rough")
                            elif cmd == "slalom":
                                # 包含三处必经红色点并且留出宽裕避让余量的精准S形绕杆路径
                                slalom_wps = [
                                    (2.9, -9.5, 0.4, "rough"),     # 1. 安全过渡点，引导机器人向西南切入
                                    (1.8, -9.95, 0.4, "rough"),    # 2. 触发红点 1，与 Pole 1 保持 0.55m 超大安全间距
                                    (0.9, -10.0, 0.4, "rough"),    # 3. 绕过 Pole 1 左侧的保护航点，拉开间距
                                    (0.8, -10.8, 0.4, "rough"),    # 4. 左侧平滑过渡点，为对准 Pole 1-2 缝隙做准备
                                    (0.9, -11.0, 0.4, "rough"),    # 5. 缝隙入口前置对齐点
                                    (2.7, -11.0, 0.4, "rough"),    # 6. 横穿 Pole 1-2 缝隙，与上下两杆均保持 0.50m 绝对中心间距
                                    (2.8, -11.8, 0.4, "rough"),    # 7. 右侧平滑过渡点，为对准 Pole 2-3 缝隙做准备
                                    (2.7, -12.0, 0.4, "rough"),    # 8. 缝隙入口前置对齐点
                                    (0.9, -12.0, 0.4, "rough"),    # 9. 横穿 Pole 2-3 缝隙，与上下两杆均保持 0.50m 绝对中心间距
                                    (0.9, -12.75, 0.4, "rough"),   # 10. 左侧平滑过渡点，为侧向触发红点 3 做准备
                                    (1.42, -12.82, 0.4, "rough"),  # 11. 触发红点 3，与 Pole 3 保持 0.50m 超大安全间距
                                    (2.3, -13.3, 0.4, "rough"),    # 12. 向下切入 Pole 3-4 缝隙的正下方对齐点
                                    (2.3, -11.5, 0.4, "rough"),    # 13. 笔直向北穿过 Pole 3-4 缝隙，与两杆各留 0.50m 最大对称间距
                                    (3.32, -12.44, 0.4, "rough"),  # 14. 触发红点 2，与 Pole 4 保持 0.52m 超大安全间距
                                    (3.7, -9.0, 0.6, "rough")      # 15. 安全返回起点
                                ]
                                nav.start_mission("S形绕行任务", slalom_wps)
                            elif cmd == "charge":
                                # 包含前置对位与直线冲刺的高墙全速越障路径
                                charge_wps = [
                                    (3.3, -9.0, 0.5, "rough"),   # 1. 移出起步区，向西对准 X = 3.3 轴线中途过渡
                                    (1.8, -9.0, 0.5, "rough"),   # 2. 轴线对齐入口，将 X 轴拉正到 1.8
                                    (1.8, -8.0, 0.4, "rough"),   # 3. 垂直逼近高墙第一步
                                    (1.8, -7.5, 0.4, "rough"),   # 4. 垂直逼近高墙第二步（距墙仅 0.5m，车身彻底正对）
                                    (1.8, -5.8, 1.2, "rough"),   # 5. 全速冲锋！越过高墙并在 Y = -5.8 处的安全平地上平稳落地（绝不碰台阶）
                                    (3.3, -5.8, 0.5, "rough"),   # 6. 安全东移折回，对齐到 X = 3.3 安全返航通道
                                    (3.3, -9.0, 0.5, "rough"),   # 7. 沿 X = 3.3 安全线直行南下，完全避开高墙
                                    (3.7, -9.0, 0.6, "rough")    # 8. 顺畅平准移入起点
                                ]
                                nav.start_mission("高墙加速冲锋", charge_wps)
                            elif cmd == "bridge":
                                bridge_wps = [
                                    (3.3, -9.0, 0.5, "rough"),   # 1. 移出起步区，向西对准 X = 3.3 避开高墙通道
                                    (3.3, -1.6, 0.5, "rough"),   # 2. 沿 X = 3.3 笔直向北，完全避开所有高墙与台阶结构
                                    (1.8, -1.6, 0.5, "rough"),   # 3. 水平移入木桥 A 斜坡起点的平地
                                    (1.8, -1.38, 0.4, "rough"),  # 4. 对齐木桥 A 坡底起点
                                    (1.8, 0.0, 0.4, "rough"),    # 5. 稳定爬上木桥 A 斜坡登上平台
                                    (3.2, 0.0, 0.4, "rough"),    # 6. 沿独木桥 Y=0 轴线精细循迹
                                    (4.85, 0.0, 0.4, "rough"),   # 7. 沿独木桥 Y=0 轴线继续前进
                                    (5.7, -0.5, 0.4, "rough"),   # 8. 驶入东侧木桥转弯缓冲段
                                    (5.7, -2.25, 0.4, "rough"),  # 9. 驶上中间木桥平台
                                    (5.7, -3.5, 0.4, "rough"),   # 10. 到达木桥 B 斜坡起爬平整区
                                    (5.7, -5.0, 0.4, "rough"),   # 11. 稳健行进下斜坡 B 
                                    (5.7, -7.0, 0.4, "rough"),   # 12. 平稳落地脱离木桥 B，在平地上放平车身
                                    (3.3, -7.0, 0.5, "rough"),   # 13. 横向西移，重新汇入 X=3.3 安全返航通道
                                    (3.3, -9.0, 0.5, "rough"),   # 14. 沿 X=3.3 安全走廊直行南下
                                    (3.7, -9.0, 0.6, "rough")    # 15. 顺畅平准移入起点
                                ]
                                nav.start_mission("斜坡木桥任务", bridge_wps)
                            elif cmd == "gravel":
                                gravel_wps = [
                                    (3.7, -12.5, 0.5, "rough"),  # 1. 沿 X=3.7 轴线一路笔直向南，在 flat 地面上完全避开限高门柱
                                    (4.84, -12.5, 0.4, "rough"), # 2. 横向东进，从西侧入口完美切入 1号砂砾平台 tip
                                    (5.84, -12.0, 0.4, "rough"), # 3. 横跨转移至 2号砂砾平台
                                    (5.84, -10.2, 0.4, "rough"), # 4. 笔直向北走下平台，平稳落于北侧平地 tip
                                    (3.3, -10.2, 0.5, "rough"),  # 5. 西向横移，重新汇入 X=3.3 安全返航通道
                                    (3.3, -9.0, 0.5, "rough"),   # 6. 沿着 X=3.3 直行北上
                                    (3.7, -9.0, 0.6, "rough")    # 7. 顺畅平准移入起点
                                ]
                                nav.start_mission("砂砾碎石任务", gravel_wps)
                            elif cmd == "grand":
                                grand_wps = [
                                    # 1. S形绕杆
                                    (2.9, -9.5, 0.4, "rough"),
                                    (1.8, -9.95, 0.4, "rough"),
                                    (0.9, -10.0, 0.4, "rough"),
                                    (0.8, -10.8, 0.4, "rough"),
                                    (0.9, -11.0, 0.4, "rough"),
                                    (2.7, -11.0, 0.4, "rough"),
                                    (2.8, -11.8, 0.4, "rough"),
                                    (2.7, -12.0, 0.4, "rough"),
                                    (0.9, -12.0, 0.4, "rough"),
                                    (0.9, -12.75, 0.4, "rough"),
                                    (1.42, -12.82, 0.4, "rough"),
                                    (2.3, -13.3, 0.4, "rough"),
                                    (2.3, -11.5, 0.4, "rough"),
                                    (3.32, -12.44, 0.4, "rough"),
                                    (3.7, -9.0, 0.5, "rough"),
                                    # 2. 限高下蹲
                                    (5.7, -10.2, 0.5, "rough"),
                                    (5.7, -9.0, 0.3, "crawl"),
                                    (5.7, -7.8, 0.3, "rough"),
                                    (5.7, -7.0, 0.5, "rough"),
                                    (3.7, -9.0, 0.5, "rough"),
                                    # 3. 砂砾碎石
                                    (3.7, -12.5, 0.5, "rough"),
                                    (4.84, -12.5, 0.4, "rough"),
                                    (5.84, -12.0, 0.4, "rough"),
                                    (5.84, -10.2, 0.4, "rough"),
                                    (3.3, -10.2, 0.5, "rough"),
                                    (3.3, -9.0, 0.5, "rough"),
                                    (3.7, -9.0, 0.5, "rough"),
                                    # 4. 高墙越障
                                    (3.3, -9.0, 0.5, "rough"),
                                    (1.8, -9.0, 0.5, "rough"),
                                    (1.8, -8.0, 0.4, "rough"),
                                    (1.8, -7.5, 0.4, "rough"),
                                    (1.8, -5.8, 1.2, "rough"),
                                    # 5. 台阶攀爬
                                    (1.8, -3.5, 0.4, "rough"),
                                    (1.8, -1.6, 0.4, "rough"),
                                    # 6. 斜坡木桥
                                    (1.8, -1.38, 0.4, "rough"),
                                    (1.8, 0.0, 0.4, "rough"),
                                    (3.2, 0.0, 0.4, "rough"),
                                    (4.85, 0.0, 0.4, "rough"),
                                    (5.7, -0.5, 0.4, "rough"),
                                    (5.7, -2.25, 0.4, "rough"),
                                    (5.7, -3.5, 0.4, "rough"),
                                    (5.7, -5.0, 0.4, "rough"),
                                    (5.7, -7.0, 0.4, "rough"),
                                    (3.3, -7.0, 0.5, "rough"),   # 横向西移，重新汇入 X=3.3 安全通道
                                    (3.3, -9.0, 0.5, "rough"),   # 沿 X=3.3 通道南下返回
                                    (3.7, -9.0, 0.6, "rough")    # 顺畅平准移入起点
                                ]
                                nav.start_mission("障碍赛大满贯", grand_wps)
                            console_text = ""
                        elif event.key == pygame.K_BACKSPACE:
                            console_text = console_text[:-1]
                        else:
                            console_text += event.unicode
                    else:
                        # 正常打点快捷键
                        if event.key == pygame.K_r:
                            key_id = len(nav.recorded_keypoints) + 1
                            nav.recorded_keypoints.append({
                                'id': key_id, 'x': rx, 'y': ry, 'z': rz, 'yaw': yaw_deg
                            })
                            print(f"[关键点] 快捷键记录 WP{key_id}：({rx:.2f}, {ry:.2f})")
                        elif event.key == pygame.K_SPACE:
                            # 紧急刹车 / 停止
                            nav.cancel_mission()
                
                # 动态交互按钮响应 (防止与终端输入冲突)
                if not console_active:
                    if btn_slalom.is_clicked(event):
                        # 包含三处必经红色点并且留出宽裕避让余量的精准S形绕杆路径
                        slalom_wps = [
                            (2.9, -9.5, 0.4, "rough"),     # 1. 安全过渡点，引导机器人向西南切入
                            (1.8, -9.95, 0.4, "rough"),    # 2. 触发红点 1，与 Pole 1 保持 0.55m 超大安全间距
                            (0.9, -10.0, 0.4, "rough"),    # 3. 绕过 Pole 1 左侧的保护航点，拉开间距
                            (0.8, -10.8, 0.4, "rough"),    # 4. 左侧平滑过渡点，为对准 Pole 1-2 缝隙做准备
                            (0.9, -11.0, 0.4, "rough"),    # 5. 缝隙入口前置对齐点
                            (2.7, -11.0, 0.4, "rough"),    # 6. 横穿 Pole 1-2 缝隙，与上下两杆均保持 0.50m 绝对中心间距
                            (2.8, -11.8, 0.4, "rough"),    # 7. 右侧平滑过渡点，为对准 Pole 2-3 缝隙做准备
                            (2.7, -12.0, 0.4, "rough"),    # 8. 缝隙入口前置对齐点
                            (0.9, -12.0, 0.4, "rough"),    # 9. 横穿 Pole 2-3 缝隙，与上下两杆均保持 0.50m 绝对中心间距
                            (0.9, -12.75, 0.4, "rough"),   # 10. 左侧平滑过渡点，为侧向触发红点 3 做准备
                            (1.42, -12.82, 0.4, "rough"),  # 11. 触发红点 3，与 Pole 3 保持 0.50m 超大安全间距
                            (2.3, -13.3, 0.4, "rough"),    # 12. 向下切入 Pole 3-4 缝隙的正下方对齐点
                            (2.3, -11.5, 0.4, "rough"),    # 13. 笔直向北穿过 Pole 3-4 缝隙，与两杆各留 0.50m 最大对称间距
                            (3.32, -12.44, 0.4, "rough"),  # 14. 触发红点 2，与 Pole 4 保持 0.52m 超大安全间距
                            (3.7, -9.0, 0.6, "rough")      # 15. 安全返回起点
                        ]
                        nav.start_mission("S形绕行演练", slalom_wps)
                    elif btn_wall.is_clicked(event):
                        # 包含前置对位与直线冲刺的高墙全速越障路径
                        charge_wps = [
                            (3.3, -9.0, 0.5, "rough"),   # 1. 移出起步区，向西对准 X = 3.3 轴线中途过渡
                            (1.8, -9.0, 0.5, "rough"),   # 2. 轴线对齐入口，将 X 轴拉正到 1.8
                            (1.8, -8.0, 0.4, "rough"),   # 3. 垂直逼近高墙第一步
                            (1.8, -7.5, 0.4, "rough"),   # 4. 垂直逼近高墙第二步（距墙仅 0.5m，车身彻底正对）
                            (1.8, -5.8, 1.2, "rough"),   # 5. 全速冲锋！越过高墙并在 Y = -5.8 处的安全平地上平稳落地（绝不碰台阶）
                            (3.3, -5.8, 0.5, "rough"),   # 6. 安全东移折回，对齐到 X = 3.3 安全返航通道
                            (3.3, -9.0, 0.5, "rough"),   # 7. 沿 X = 3.3 安全线直行南下，完全避开高墙
                            (3.7, -9.0, 0.6, "rough")    # 8. 顺畅平准移入起点
                        ]
                        nav.start_mission("高墙加速越障", charge_wps)
                    elif btn_crawl.is_clicked(event):
                        # 正确的 Y 轴方向对准跨越限高门
                        crawl_wps = [
                            (5.7, -10.2, 0.5, "rough"),  # 首先对准限高门中心轴线前方
                            (5.7, -9.0, 0.3, "crawl"),   # 弯腰通过限高门中心
                            (5.7, -7.8, 0.3, "rough"),   # 只要过了门槛，直接恢复为常规高度
                            (3.7, -9.0, 0.5, "rough")    # 顺畅返回起点
                        ]
                        nav.start_mission("限高爬行任务", crawl_wps)
                    elif btn_stairs.is_clicked(event):
                        # 阶梯攀爬：利用 X=3.3 通道绕过高墙边缘，水平对齐轴线登顶，并完整从另一侧下台阶穿过
                        stairs_wps = [
                            (3.3, -9.0, 0.5, "rough"),   # 1. 移出起步区，向西对准 X = 3.3 避开高墙通道
                            (3.3, -5.8, 0.5, "rough"),   # 2. 沿 X = 3.3 笔直向北，完全绕开高墙右侧边缘（留出超大空间）
                            (1.8, -5.8, 0.5, "rough"),   # 3. 水平移入台阶前方的平地/空地中心，完美对齐轴线
                            (1.8, -3.5, 0.4, "rough"),   # 4. 直线攀爬登上顶部平台
                            (1.8, -1.6, 0.4, "rough"),   # 5. 直线走下另一侧台阶，到 Y=-1.6m，后腿完全过台阶，前腿也不上桥，在平地平稳落地
                            (3.3, -1.6, 0.5, "rough"),   # 6. 横向东移，重新对齐到 X = 3.3 安全返航通道
                            (3.3, -9.0, 0.5, "rough"),   # 7. 沿 X = 3.3 安全线直行南下，完全避开高墙与台阶
                            (3.7, -9.0, 0.6, "rough")    # 8. 顺畅平准移入起点
                        ]
                        nav.start_mission("阶梯攀爬登顶", stairs_wps)
                    elif btn_bridge.is_clicked(event):
                        bridge_wps = [
                            (3.3, -9.0, 0.5, "rough"),   # 1. 移出起步区，向西对准 X = 3.3 避开高墙通道
                            (3.3, -1.6, 0.5, "rough"),   # 2. 沿 X = 3.3 笔直向北，完全绕开所有高墙与台阶结构
                            (1.8, -1.6, 0.5, "rough"),   # 3. 水平移入木桥 A 斜坡起点的平地
                            (1.8, -1.38, 0.4, "rough"),  # 4. 对齐木桥 A 坡底起点
                            (1.8, 0.0, 0.4, "rough"),    # 5. 稳定爬上木桥 A 斜坡登上平台
                            (3.2, 0.0, 0.4, "rough"),    # 6. 沿独木桥 Y=0 轴线精细循迹
                            (4.85, 0.0, 0.4, "rough"),   # 7. 沿独木桥 Y=0 轴线继续前进
                            (5.7, -0.5, 0.4, "rough"),   # 8. 驶入东侧木桥转弯缓冲段
                            (5.7, -2.25, 0.4, "rough"),  # 9. 驶上中间木桥平台
                            (5.7, -3.5, 0.4, "rough"),   # 10. 到达木桥 B 斜坡起爬平整区
                            (5.7, -5.0, 0.4, "rough"),   # 11. 稳健行进下斜坡 B 
                            (5.7, -7.0, 0.4, "rough"),   # 12. 平稳落地脱离木桥 B，在平地上放平车身
                            (3.3, -7.0, 0.5, "rough"),   # 13. 横向西移，重新汇入 X=3.3 安全返航通道
                            (3.3, -9.0, 0.5, "rough"),   # 14. 沿 X=3.3 安全走廊直行南下
                            (3.7, -9.0, 0.6, "rough")    # 15. 顺畅平准移入起点
                        ]
                        nav.start_mission("斜坡木桥任务", bridge_wps)
                    elif btn_gravel.is_clicked(event):
                        gravel_wps = [
                            (3.7, -12.5, 0.5, "rough"),  # 1. 沿 X=3.7 轴线一路笔直向南，在 flat 地面上完全避开限高门柱
                            (4.84, -12.5, 0.4, "rough"), # 2. 横向东进，从西侧入口完美切入 1号砂砾平台 tip
                            (5.84, -12.0, 0.4, "rough"), # 3. 横跨转移至 2号砂砾平台
                            (5.84, -10.2, 0.4, "rough"), # 4. 笔直向北走下平台，平稳落于北侧平地 tip
                            (3.3, -10.2, 0.5, "rough"),  # 5. 西向横移，重新汇入 X=3.3 安全返航通道
                            (3.3, -9.0, 0.5, "rough"),   # 6. 沿着 X=3.3 直行北上
                            (3.7, -9.0, 0.6, "rough")    # 7. 顺畅平准移入起点
                        ]
                        nav.start_mission("砂砾碎石任务", gravel_wps)
                    elif btn_grand.is_clicked(event):
                        grand_wps = [
                            # 1. S形绕杆
                            (2.9, -9.5, 0.4, "rough"),
                            (1.8, -9.95, 0.4, "rough"),
                            (0.9, -10.0, 0.4, "rough"),
                            (0.8, -10.8, 0.4, "rough"),
                            (0.9, -11.0, 0.4, "rough"),
                            (2.7, -11.0, 0.4, "rough"),
                            (2.8, -11.8, 0.4, "rough"),
                            (2.7, -12.0, 0.4, "rough"),
                            (0.9, -12.0, 0.4, "rough"),
                            (0.9, -12.75, 0.4, "rough"),
                            (1.42, -12.82, 0.4, "rough"),
                            (2.3, -13.3, 0.4, "rough"),
                            (2.3, -11.5, 0.4, "rough"),
                            (3.32, -12.44, 0.4, "rough"),
                            (3.7, -9.0, 0.5, "rough"),
                            # 2. 限高下蹲
                            (5.7, -10.2, 0.5, "rough"),
                            (5.7, -9.0, 0.3, "crawl"),
                            (5.7, -7.8, 0.3, "rough"),
                            (5.7, -7.0, 0.5, "rough"),
                            (3.7, -9.0, 0.5, "rough"),
                            # 3. 砂砾碎石
                            (3.7, -12.5, 0.5, "rough"),
                            (4.84, -12.5, 0.4, "rough"),
                            (5.84, -12.0, 0.4, "rough"),
                            (5.84, -10.2, 0.4, "rough"),
                            (3.3, -10.2, 0.5, "rough"),
                            (3.3, -9.0, 0.5, "rough"),
                            (3.7, -9.0, 0.5, "rough"),
                            # 4. 高墙越障
                            (3.3, -9.0, 0.5, "rough"),
                            (1.8, -9.0, 0.5, "rough"),
                            (1.8, -8.0, 0.4, "rough"),
                            (1.8, -7.5, 0.4, "rough"),
                            (1.8, -5.8, 1.2, "rough"),
                            # 5. 台阶攀爬
                            (1.8, -3.5, 0.4, "rough"),
                            (1.8, -1.6, 0.4, "rough"),
                            # 6. 斜坡木桥
                            (1.8, -1.38, 0.4, "rough"),
                            (1.8, 0.0, 0.4, "rough"),
                            (3.2, 0.0, 0.4, "rough"),
                            (4.85, 0.0, 0.4, "rough"),
                            (5.7, -0.5, 0.4, "rough"),
                            (5.7, -2.25, 0.4, "rough"),
                            (5.7, -3.5, 0.4, "rough"),
                            (5.7, -5.0, 0.4, "rough"),
                            (5.7, -7.0, 0.4, "rough"),
                            (3.3, -7.0, 0.5, "rough"),   # 横向西移，重新汇入 X=3.3 安全通道
                            (3.3, -9.0, 0.5, "rough"),   # 沿 X=3.3 通道南下返回
                            (3.7, -9.0, 0.6, "rough")    # 顺畅平准移入起点
                        ]
                        nav.start_mission("障碍赛大满贯", grand_wps)
                    elif btn_clear_trg.is_clicked(event):
                        nav.cancel_mission()
                    elif btn_center_toggle.is_clicked(event):
                        auto_center = not auto_center
                        btn_center_toggle.text = "视角居中:开" if auto_center else "视角居中:关"
                        btn_center_toggle.bg_color = (16, 185, 129) if auto_center else (244, 63, 94)
                    elif btn_draw_mode.is_clicked(event):
                        draw_mode = not draw_mode
                        btn_draw_mode.text = "手绘模式:开" if draw_mode else "手绘模式:关"
                        btn_draw_mode.bg_color = (16, 185, 129) if draw_mode else (30, 41, 59)
                    elif btn_speed_down.is_clicked(event):
                        # 物理减速
                        if sim_speed_factor > 1.0:
                            if sim_speed_factor >= 5.0: sim_speed_factor = 3.0
                            elif sim_speed_factor >= 3.0: sim_speed_factor = 2.0
                            elif sim_speed_factor >= 2.0: sim_speed_factor = 1.5
                            else: sim_speed_factor = 1.0
                        else:
                            if sim_speed_factor >= 1.0: sim_speed_factor = 0.8
                            elif sim_speed_factor >= 0.8: sim_speed_factor = 0.5
                            elif sim_speed_factor >= 0.5: sim_speed_factor = 0.2
                        print(f"[仿真控制] 物理减速。当前加速倍率: {sim_speed_factor:.1f}x")
                    elif btn_speed_normal.is_clicked(event):
                        sim_speed_factor = 1.0
                        print(f"[仿真控制] 恢复标准时间流速 (1x)")
                    elif btn_speed_up.is_clicked(event):
                        # 物理加速
                        if sim_speed_factor < 1.0:
                            if sim_speed_factor <= 0.2: sim_speed_factor = 0.5
                            elif sim_speed_factor <= 0.5: sim_speed_factor = 0.8
                            else: sim_speed_factor = 1.0
                        else:
                            if sim_speed_factor <= 1.0: sim_speed_factor = 1.5
                            elif sim_speed_factor <= 1.5: sim_speed_factor = 2.0
                            elif sim_speed_factor <= 2.0: sim_speed_factor = 3.0
                            elif sim_speed_factor <= 3.0: sim_speed_factor = 5.0
                        print(f"[仿真控制] 物理加速。当前加速倍率: {sim_speed_factor:.1f}x")
                    elif btn_x_plus.is_clicked(event):
                        nav.mode = "AUTO"
                        nav.target_x = (nav.target_x if nav.target_x is not None else rx) + 0.5
                        nav.target_y = nav.target_y if nav.target_y is not None else ry
                    elif btn_x_minus.is_clicked(event):
                        nav.mode = "AUTO"
                        nav.target_x = (nav.target_x if nav.target_x is not None else rx) - 0.5
                        nav.target_y = nav.target_y if nav.target_y is not None else ry
                    elif btn_y_plus.is_clicked(event):
                        nav.mode = "AUTO"
                        nav.target_x = nav.target_x if nav.target_x is not None else rx
                        nav.target_y = (nav.target_y if nav.target_y is not None else ry) + 0.5
                    elif btn_y_minus.is_clicked(event):
                        nav.mode = "AUTO"
                        nav.target_x = nav.target_x if nav.target_x is not None else rx
                        nav.target_y = (nav.target_y if nav.target_y is not None else ry) - 0.5
                    elif btn_rec_key.is_clicked(event):
                        key_id = len(nav.recorded_keypoints) + 1
                        nav.recorded_keypoints.append({
                            'id': key_id, 'x': rx, 'y': ry, 'z': rz, 'yaw': yaw_deg
                        })
                        print(f"[关键点] 已记录 WP{key_id}：({rx:.2f}, {ry:.2f})")
                    elif btn_exp_key.is_clicked(event):
                        try:
                            exp_path = Path(__file__).parent / "recorded_keypoints.txt"
                            with open(exp_path, "w", encoding="utf-8") as f:
                                f.write("序号, X坐标(米), Y坐标(米), Z高度(米), 机身航向角(度)\n")
                                for kp in nav.recorded_keypoints:
                                    f.write(f"WP{kp['id']}, {kp['x']:.4f}, {kp['y']:.4f}, {kp['z']:.4f}, {kp['yaw']:.2f}\n")
                            print(f"[关键点] 数据文件已成功导出至: {exp_path}")
                        except Exception as ex:
                            print(f"[关键点] 导出出错: {ex}")
                    elif btn_clear_key.is_clicked(event):
                        nav.recorded_keypoints = []
                        print("[关键点] 所有已记录的关键点已清空。")

            # ------------------------------------------------------------------
            # 自动路径寻寻航与物理指令步进 (50Hz 控制频率)
            # ------------------------------------------------------------------
            if nav.mode == "AUTO":
                cmd_vx, cmd_vy, cmd_wz = nav.update_navigation(rx, ry, yaw_rad, runner, dt=control_dt)
                command_input = np.array([cmd_vx, cmd_vy, cmd_wz], dtype=np.float32)
            else:
                # 读取键盘输入控制：方向键控制 x (前后) 和 yaw (旋转)，A/D 键控制 y (左右横移)
                keys = pygame.key.get_pressed()
                cmd_vx, cmd_vy, cmd_wz = 0.0, 0.0, 0.0
                
                # 方向键控制 x (前后，峰值限制为 1.2m/s)
                if keys[pygame.K_UP]:         cmd_vx = 1.2
                elif keys[pygame.K_DOWN]:     cmd_vx = -1.2
                
                # A/D 键控制 y (左右横移)
                if keys[pygame.K_a]:          cmd_vy = 0.8
                elif keys[pygame.K_d]:        cmd_vy = -0.8
                
                # 方向键控制 yaw (旋转，峰值限制为 1.2r/s)
                if keys[pygame.K_LEFT]:       cmd_wz = 1.2
                elif keys[pygame.K_RIGHT]:    cmd_wz = -1.2
                
                command_input = np.array([cmd_vx, cmd_vy, cmd_wz], dtype=np.float32)

            # 融合观测数据
            obs = io.get_obs_53d(command_input, runner.default_dof_pos, runner.last_actions)
            
            # 策略网络推理
            scaled_actions, raw_actions = runner.step(obs)
            
            # 向仿真器下发关节控制指令
            io.send_actions(scaled_actions, runner.default_dof_pos)
            
            # 推进物理步进
            for _ in range(sim_steps_per_control):
                mujoco.mj_step(io.m, io.d)

            # 控制视口镜头跟随
            viewer_counter += 1
            if viewer_counter >= 2:
                base_id = mujoco.mj_name2id(io.m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
                if base_id != -1:
                    viewer.cam.lookat[:] = io.d.xpos[base_id]
                viewer.sync()
                viewer_counter = 0

            # ------------------------------------------------------------------
            # 画面绘制：左侧网格物理元素实景图 (100% 实景对应解析绘制)
            # ------------------------------------------------------------------
            pygame.draw.rect(screen, COLOR_BG, (0, 0, MAP_WIDTH, WINDOW_HEIGHT))
            
            # 计算当前网格绘制边界
            grid_min_w, grid_min_h = screen_to_world(0, WINDOW_HEIGHT)
            grid_max_w, grid_max_h = screen_to_world(MAP_WIDTH, 0)
            
            start_grid_x = int(math.floor(grid_min_w))
            end_grid_x = int(math.ceil(grid_max_w))
            start_grid_y = int(math.floor(grid_min_h))
            end_grid_y = int(math.ceil(grid_max_h))
            
            # 绘制纵轴网格线
            for gx in range(start_grid_x, end_grid_x + 1):
                sx, _ = world_to_screen(gx, 0)
                color = COLOR_AXIS if gx == 0 else COLOR_GRID
                width = 2 if gx == 0 else 1
                pygame.draw.line(screen, color, (sx, 0), (sx, WINDOW_HEIGHT), width)
                if gx % 2 == 0 and 0 < sx < MAP_WIDTH - 20:
                    lbl = font_mono.render(f"{gx}m", True, COLOR_TEXT_MUTED)
                    screen.blit(lbl, (sx + 3, WINDOW_HEIGHT - 20))

            # 绘制横轴网格线
            for gy in range(start_grid_y, end_grid_y + 1):
                _, sy = world_to_screen(0, gy)
                color = COLOR_AXIS if gy == 0 else COLOR_GRID
                width = 2 if gy == 0 else 1
                pygame.draw.line(screen, color, (0, sy), (MAP_WIDTH, sy), width)
                if gy % 2 == 0 and 0 < sy < WINDOW_HEIGHT - 20:
                    lbl = font_mono.render(f"{gy}m", True, COLOR_TEXT_MUTED)
                    screen.blit(lbl, (5, sy + 3))

            # ------------------------------------------------------------------
            # 🌟 动态地图绘制引擎 —— 解析并精确渲染 scene_terrain.xml 所有实景元素
            # ------------------------------------------------------------------
            for geom in xml_obstacles:
                g_type = geom['type']
                pos = geom['pos']
                size = geom['size']
                quat = geom['quat']
                rgba = geom['rgba']
                name = geom['name']
                
                # 转换色彩至 0-255 并增加发光度
                color = (int(rgba[0]*255), int(rgba[1]*255), int(rgba[2]*255))
                cx, cy = pos[0], pos[1]
                
                # 1. 矩形盒子渲染（支持任意偏航角 Z轴自转 顶点多边形解算）
                if g_type == 'box':
                    qw, qx, qy, qz = quat
                    siny_cosp = 2.0 * (qw * qz + qx * qy)
                    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                    yaw = math.atan2(siny_cosp, cosy_cosp)
                    
                    sx, sy = size[0], size[1]
                    local_corners = [
                        (-sx, -sy), (sx, -sy), (sx, sy), (-sx, sy)
                    ]
                    screen_corners = []
                    for lx, ly in local_corners:
                        # 空间旋转
                        wx = cx + lx * math.cos(yaw) - ly * math.sin(yaw)
                        wy = cy + lx * math.sin(yaw) + ly * math.cos(yaw)
                        screen_corners.append(world_to_screen(wx, wy))
                    
                    # 填充半透明色，勾画发光边界
                    pygame.draw.polygon(screen, color, screen_corners)
                    border_color = (min(color[0]+40, 255), min(color[1]+40, 255), min(color[2]+40, 255))
                    pygame.draw.polygon(screen, border_color, screen_corners, width=1)
                    
                    # 如果是高越障墙，渲染额外标识
                    if abs(cy - (-7.0)) < 0.1:
                        wall_lbl = font_small.render("Obstacle Wall", True, COLOR_TEXT_LIGHT)
                        screen.blit(wall_lbl, (screen_corners[0][0] + 5, screen_corners[0][1] - 18))
                
                # 2. 圆柱体渲染
                elif g_type == 'cylinder':
                    sx, sy = world_to_screen(cx, cy)
                    rad = max(int(size[0] * zoom), 3)
                    pygame.draw.circle(screen, color, (sx, sy), rad)
                    border_color = (min(color[0]+40, 255), min(color[1]+40, 255), min(color[2]+40, 255))
                    pygame.draw.circle(screen, border_color, (sx, sy), rad, width=1)
                    
                    # 如果是绕杆立柱，加注底图安全边界环
                    if "Slalom" in name:
                        pygame.draw.circle(screen, (6, 182, 212, 30), (sx, sy), int(0.1 * zoom))
                        pygame.draw.circle(screen, COLOR_CYAN, (sx, sy), int(0.1 * zoom), width=1)
                
                # 3. 半透明 hfield 高原区域渲染
                elif g_type == 'hfield':
                    sx = size[0] if len(size) > 0 else 1.0
                    sy = size[1] if len(size) > 1 else 1.0
                    x1, y1 = world_to_screen(cx - sx, cy + sy)
                    x2, y2 = world_to_screen(cx + sx, cy - sy)
                    w = x2 - x1
                    h = y2 - y1
                    if w > 0 and h > 0:
                        surf = pygame.Surface((w, h), pygame.SRCALPHA)
                        surf.fill((139, 92, 246, 35))  # 紫色高原提示
                        pygame.draw.rect(surf, COLOR_PURPLE, (0, 0, w, h), width=1)
                        screen.blit(surf, (x1, y1))
                        hf_lbl = font_mono.render("HField Platform", True, COLOR_PURPLE)
                        screen.blit(hf_lbl, (x1 + 6, y1 + 4))

            # ------------------------------------------------------------------
            # 规划与运动行迹线绘制
            # ------------------------------------------------------------------
            # 绘制机器人面包屑行迹线
            if len(robot_trail) > 1:
                trail_points = [world_to_screen(pt[0], pt[1]) for pt in robot_trail]
                pygame.draw.lines(screen, COLOR_CYAN, False, trail_points, 2)

            # 绘制正在实时手划的绿色轨迹
            if is_drawing_path and len(drawn_points) > 1:
                draw_pts = [world_to_screen(pt[0], pt[1]) for pt in drawn_points]
                pygame.draw.lines(screen, COLOR_EMERALD, False, draw_pts, 3)

            # 绘制已加载的自动路径寻迹线
            if len(nav.drawn_path) > 1:
                path_pts = [world_to_screen(pt[0], pt[1]) for pt in nav.drawn_path]
                pygame.draw.lines(screen, COLOR_EMERALD, False, path_pts, 2)
                # 渲染每一个航点节点小球
                for pt in nav.drawn_path:
                    sx, sy = world_to_screen(pt[0], pt[1])
                    pygame.draw.circle(screen, COLOR_CYAN, (sx, sy), 4)

            # 绘制关键点数据打点
            for kp in nav.recorded_keypoints:
                kx, ky = world_to_screen(kp['x'], kp['y'])
                pygame.draw.circle(screen, COLOR_PURPLE, (kx, ky), 7)
                pygame.draw.circle(screen, COLOR_TEXT_LIGHT, (kx, ky), 7, width=1)
                kp_lbl = font_mono.render(f"WP{kp['id']}", True, COLOR_TEXT_LIGHT)
                screen.blit(kp_lbl, (kx - 8, ky - 18))

            # 绘制当前正在行进指向的目标点
            if nav.target_x is not None and nav.target_y is not None:
                tx, ty = world_to_screen(nav.target_x, nav.target_y)
                pulse_rad = int(8 + 4 * math.sin(time.time() * 8.0))
                pygame.draw.circle(screen, COLOR_GOLD, (tx, ty), pulse_rad, width=2)
                pygame.draw.circle(screen, COLOR_GOLD, (tx, ty), 3)
                pygame.draw.line(screen, COLOR_GOLD, (tx - pulse_rad - 4, ty), (tx + pulse_rad + 4, ty), 1)
                pygame.draw.line(screen, COLOR_GOLD, (tx, ty - pulse_rad - 4), (tx, ty + pulse_rad + 4), 1)
                trg_lbl = font_mono.render(f"Target: ({nav.target_x:.2f}, {nav.target_y:.2f})", True, COLOR_GOLD)
                screen.blit(trg_lbl, (tx + 12, ty - 6))

            # ------------------------------------------------------------------
            # 渲染机器人本体图标 (多边形轮骨，指示航向)
            # ------------------------------------------------------------------
            rx_pix, ry_pix = world_to_screen(rx, ry)
            robot_rad = max(int(0.25 * zoom), 6)
            
            # 画机身盘
            pygame.draw.circle(screen, COLOR_CYAN, (rx_pix, ry_pix), robot_rad)
            pygame.draw.circle(screen, COLOR_TEXT_LIGHT, (rx_pix, ry_pix), robot_rad, width=2)
            
            # 航向箭头鼻线
            nose_wx = rx + 0.35 * math.cos(yaw_rad)
            nose_wy = ry + 0.35 * math.sin(yaw_rad)
            nose_x_pix, nose_y_pix = world_to_screen(nose_wx, nose_wy)
            pygame.draw.line(screen, COLOR_EMERALD, (rx_pix, ry_pix), (nose_x_pix, nose_y_pix), 3)
            pygame.draw.circle(screen, COLOR_EMERALD, (nose_x_pix, nose_y_pix), 3)

            # 四轮足点映射
            wheel_offset_w = 0.2
            for angle_offset in [-0.7, 0.7, -2.4, 2.4]:
                w_angle = yaw_rad + angle_offset
                w_wx = rx + wheel_offset_w * math.cos(w_angle)
                w_wy = ry + wheel_offset_w * math.sin(w_angle)
                w_x_pix, w_y_pix = world_to_screen(w_wx, w_wy)
                pygame.draw.rect(screen, (0, 0, 0), (w_x_pix - 4, w_y_pix - 4, 8, 8), border_radius=4)

            # ------------------------------------------------------------------
            # 渲染右侧 HUD 磨砂玻璃科技控制面板
            # ------------------------------------------------------------------
            pygame.draw.line(screen, COLOR_HUD_BORDER, (MAP_WIDTH, 0), (MAP_WIDTH, WINDOW_HEIGHT), 2)
            pygame.draw.rect(screen, COLOR_HUD_BG, (MAP_WIDTH, 0, WINDOW_WIDTH - MAP_WIDTH, WINDOW_HEIGHT))
            
            # 面板标题
            screen.blit(font_large.render("数据与控制面板", True, COLOR_TEXT_LIGHT), (MAP_WIDTH + 20, 15))
            pygame.draw.line(screen, COLOR_HUD_BORDER, (MAP_WIDTH + 20, 42), (WINDOW_WIDTH - 20, 42), 1)

            # 自动模式状态指示
            y_offset = 55
            screen.blit(font_medium.render("当前状态：", True, COLOR_TEXT_MUTED), (MAP_WIDTH + 20, y_offset))
            if nav.mode == "AUTO":
                pygame.draw.rect(screen, (16, 185, 129, 40), (MAP_WIDTH + 110, y_offset - 2, 85, 22), border_radius=4)
                screen.blit(font_medium.render("自动巡航", True, COLOR_EMERALD), (MAP_WIDTH + 120, y_offset))
            else:
                pygame.draw.rect(screen, (244, 63, 94, 40), (MAP_WIDTH + 110, y_offset - 2, 85, 22), border_radius=4)
                screen.blit(font_medium.render("手动接管", True, COLOR_ROSE), (MAP_WIDTH + 120, y_offset))

            # 当前驱动神经网络控制策略
            y_offset = 85
            screen.blit(font_medium.render("当前高度：", True, COLOR_TEXT_MUTED), (MAP_WIDTH + 20, y_offset))
            p_name = runner.current_policy_name.upper()
            p_color = COLOR_CYAN if p_name == "CRAWL" else COLOR_AMBER
            cn_pname = "低姿爬行" if p_name == "CRAWL" else "常规站立"
            screen.blit(font_medium.render(cn_pname, True, p_color), (MAP_WIDTH + 110, y_offset))

            # (已移除自动脱困及卡死报警提醒)

            # 当前仿真时间倍速
            y_offset = 115
            screen.blit(font_medium.render("仿真倍速：", True, COLOR_TEXT_MUTED), (MAP_WIDTH + 20, y_offset))
            speed_display_str = f"{sim_speed_factor:.1f}x"
            screen.blit(font_medium.render(speed_display_str, True, COLOR_EMERALD), (MAP_WIDTH + 110, y_offset))

            # 全局惯导 telemetry 数据读取
            y_offset = 145
            screen.blit(font_small.render("实时全局位置(m)：", True, COLOR_TEXT_MUTED), (MAP_WIDTH + 20, y_offset))
            y_offset += 18
            pygame.draw.rect(screen, COLOR_CONSOLE_BG, (MAP_WIDTH + 20, y_offset, 285, 45), border_radius=4)
            pos_text = f"X: {rx:6.3f} m  Y: {ry:6.3f} m  Z: {rz:5.3f} m"
            screen.blit(font_mono.render(pos_text, True, COLOR_EMERALD), (MAP_WIDTH + 30, y_offset + 5))
            speed_text = f"速度: {linear_speed:4.2f} m/s 转向: {wz_rate:5.2f} r/s"
            screen.blit(font_mono.render(speed_text, True, COLOR_EMERALD), (MAP_WIDTH + 30, y_offset + 22))

            # Euler 机身姿态
            y_offset = 218
            screen.blit(font_small.render("机身姿态角(deg)：", True, COLOR_TEXT_MUTED), (MAP_WIDTH + 20, y_offset))
            y_offset += 18
            pygame.draw.rect(screen, COLOR_CONSOLE_BG, (MAP_WIDTH + 20, y_offset, 285, 30), border_radius=4)
            euler_text = f"横滚: {roll_deg:5.1f}°  俯仰: {pitch_deg:5.1f}°  航向: {yaw_deg:5.1f}°"
            screen.blit(font_medium.render(euler_text, True, COLOR_EMERALD), (MAP_WIDTH + 25, y_offset + 6))

            # PRESET routines title
            y_offset = 280
            screen.blit(font_large.render("预设任务列表", True, COLOR_TEXT_LIGHT), (MAP_WIDTH + 20, y_offset))
            pygame.draw.line(screen, COLOR_HUD_BORDER, (MAP_WIDTH + 20, y_offset + 25), (WINDOW_WIDTH - 20, y_offset + 25), 1)

            # 绘制所有功能按钮
            btn_slalom.draw(screen, font_small)
            btn_crawl.draw(screen, font_small)
            btn_gravel.draw(screen, font_small)
            btn_wall.draw(screen, font_small)
            btn_stairs.draw(screen, font_small)
            btn_bridge.draw(screen, font_small)
            btn_grand.draw(screen, font_small)

            # 系统与视图控制
            y_offset = 440
            screen.blit(font_large.render("系统与视图控制", True, COLOR_TEXT_LIGHT), (MAP_WIDTH + 20, y_offset))
            pygame.draw.line(screen, COLOR_HUD_BORDER, (MAP_WIDTH + 20, y_offset + 22), (WINDOW_WIDTH - 20, y_offset + 22), 1)

            btn_clear_trg.draw(screen, font_small)
            btn_center_toggle.draw(screen, font_small)
            btn_draw_mode.draw(screen, font_small)
            btn_speed_down.draw(screen, font_small)
            btn_speed_normal.draw(screen, font_small)
            btn_speed_up.draw(screen, font_small)

            # 发令命令
            y_offset = 542
            screen.blit(font_medium.render("目标微调", True, COLOR_TEXT_LIGHT), (MAP_WIDTH + 20, y_offset))
            pygame.draw.line(screen, COLOR_HUD_BORDER, (MAP_WIDTH + 20, y_offset + 18), (WINDOW_WIDTH - 20, y_offset + 18), 1)
            
            btn_x_plus.draw(screen, font_small)
            btn_x_minus.draw(screen, font_small)
            btn_y_plus.draw(screen, font_small)
            btn_y_minus.draw(screen, font_small)

            # 打点打点打点
            y_offset = 605
            screen.blit(font_medium.render("关键点记录 (R键)", True, COLOR_TEXT_LIGHT), (MAP_WIDTH + 20, y_offset))
            pygame.draw.line(screen, COLOR_HUD_BORDER, (MAP_WIDTH + 20, y_offset + 18), (WINDOW_WIDTH - 20, y_offset + 18), 1)

            # 渲染最新记录位置
            if nav.recorded_keypoints:
                kp = nav.recorded_keypoints[-1]
                kp_str = f"最新记录: WP{kp['id']}: X:{kp['x']:5.2f} Y:{kp['y']:5.2f} YAW:{kp['yaw']:4.0f}°"
                screen.blit(font_mono.render(kp_str, True, COLOR_PURPLE), (MAP_WIDTH + 20, 661))
            else:
                screen.blit(font_small.render("最新记录: 暂无记录位置 (按R键保存)", True, COLOR_TEXT_MUTED), (MAP_WIDTH + 20, 661))

            btn_rec_key.draw(screen, font_small)
            btn_exp_key.draw(screen, font_small)
            btn_clear_key.draw(screen, font_small)

            # 命令行
            y_offset = 685
            screen.blit(font_medium.render("命令行输入", True, COLOR_TEXT_LIGHT), (MAP_WIDTH + 20, y_offset))
            pygame.draw.line(screen, COLOR_HUD_BORDER, (MAP_WIDTH + 20, y_offset + 18), (WINDOW_WIDTH - 20, y_offset + 18), 1)
            
            # 指令框
            con_color = COLOR_EMERALD if console_active else COLOR_HUD_BORDER
            pygame.draw.rect(screen, COLOR_CONSOLE_BG, console_rect, border_radius=4)
            pygame.draw.rect(screen, con_color, console_rect, width=1, border_radius=4)
            
            cursor = "_" if (int(time.time() * 2.0) % 2 == 0 and console_active) else ""
            con_str = f" > {console_text}{cursor}"
            screen.blit(font_mono.render(con_str, True, COLOR_EMERALD), (console_rect.x + 5, console_rect.y + 4))

            # 地图横条
            pygame.draw.rect(screen, COLOR_CONSOLE_BG, (0, WINDOW_HEIGHT - 25, MAP_WIDTH, 25))
            pygame.draw.line(screen, COLOR_HUD_BORDER, (0, WINDOW_HEIGHT - 25), (MAP_WIDTH, WINDOW_HEIGHT - 25), 1)
            status_str = f"Target: X: {nav.target_x if nav.target_x is not None else 0.0:.2f} m, Y: {nav.target_y if nav.target_y is not None else 0.0:.2f} m | Mission: {nav.mission_name}"
            screen.blit(font_mono.render(status_str, True, COLOR_TEXT_MUTED), (10, WINDOW_HEIGHT - 20))

            pygame.display.flip()

            # 50Hz 物理锁帧同步计算
            dt_real = control_dt / sim_speed_factor
            next_exec_time += dt_real
            now = time.perf_counter()
            sleep_time = next_exec_time - now
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -dt_real:
                next_exec_time = now
                
            clock.tick(int(50 * sim_speed_factor))

    print("\n[主程序] 退出网格主循环。正在关闭物理引擎环境...")
    pygame.quit()


if __name__ == "__main__":
    main()
