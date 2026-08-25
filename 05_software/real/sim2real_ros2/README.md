# ROS 2 最终比赛 Sim2Real

本目录归档 `last_not_slalom_1050` 真机工程，对应 RC_WheelLeg 在 RoboCon 仿生足式障碍赛使用的最终 ROS 2 部署栈。`1050` 是比赛得分，不是模型编号；比赛 Rough 策略为 `model_6800.onnx`。

该里程碑计划标记为 `v0.9.0`。训练架构和策略来源见 `v0.6.0`，比赛 Rough 模型首次归档见 `v0.8.0`，导航打点与路线演进见 `v0.8.1`。

## 系统闭环

```text
Odin IMU / Odom ──> hardware bridge ──> RuntimeState
                                              |
导航 / 遥控 / 屏幕 ──> cmd mux ──> policy runtime (50 Hz)
                                              |
                                      RuntimeTarget
                                              |
                              hardware bridge / CAN (200 Hz)
```

核心约束：

- 53 维策略观测、16 维动作输出。
- Rough：`model_6800`，优先 TensorRT，失败时回退 ONNX Runtime。
- Wall：`model_84`，同样保留 TensorRT 与 ONNX 两种文件。
- Crawl：比赛配置使用解析 IK，不加载 Crawl RL 权重。
- 默认站姿：髋俯仰 `0.550`、膝关节 `-1.125`。
- 默认命令源：`NAV`；默认定位模式：`relocal`。

## 目录

```text
sim2real_ros2/
├─ src/
│  ├─ sim2real_interfaces/  # RuntimeState / RuntimeTarget 消息
│  ├─ sim2real_common/      # 部署契约、滤波、平衡和安全监控
│  ├─ sim2real_hw/          # SocketCAN、IMU 和 200 Hz 电机热路径
│  ├─ sim2real_runtime/     # 策略、命令仲裁、导航、Web API
│  ├─ sim2real_nav2/        # Nav2 配置入口
│  ├─ sim2real_bringup/     # 统一参数和启动文件
│  └─ odin_ros_driver/      # Odin ROS 驱动（Apache-2.0）
├─ policies/                # 比赛实际使用的 Rough / Wall 模型
├─ map/                     # 比赛路线和抽样 PCD
├─ screen/                  # Orin 800×600 触控面板
├─ docs/                    # 架构、遥控、Web 和迁移说明
├─ Dockerfile
└─ start_sim2real.sh
```

## 构建与运行

目标环境是 Ubuntu 22.04、ROS 2 Humble 和 Jetson Orin。系统依赖和 Docker 流程见 [`DEPLOYMENT_GUIDE.md`](DEPLOYMENT_GUIDE.md)。

```bash
cd 05_software/real/sim2real_ros2
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
./start_sim2real.sh
```

运行参数和模型/路线均使用工作区根目录相对路径，因此应从本目录启动。常用启动覆盖：

```bash
# 纯里程计模式，不等待 Odin 重定位地图
./start_sim2real.sh localization_mode:=odom \
  odin_config_file:=src/odin_ros_driver/config/control_command_odom.yaml

# 禁止驱动，仅做软件链路检查
./start_sim2real.sh launch_driver:=false launch_remote:=false
```

## 必须补充的部署资产

最终源目录配置引用了 Odin `map/1hao.bin`，但工作区备份中不存在这个文件；全盘检索也未找到同名文件。为避免用来源不明的 `.bin` 冒充比赛地图，本仓库不伪造该资产。

使用 `relocal` 前必须：

1. 从比赛 Orin 或 Odin 建图备份取得真实 `1hao.bin`。
2. 修改 `src/odin_ros_driver/config/control_command_relocal.yaml` 中的 `relocalization_map_abs_path` 为目标机绝对路径。
3. 核对文件哈希并在发布说明中补充来源。

缺少该文件时请使用 `localization_mode:=odom`，不要宣称重定位闭环已复现。地图和路线边界见 [`map/README.md`](map/README.md)。

## 归档边界

已保留：

- 最终六个 ROS 2 包、Odin 驱动源码、比赛设备标定参数和预编译 SDK 静态库。
- 最终 Rough/Wall ONNX 与比赛机 TensorRT engine。
- 五份最终工程路线、1 号场地抽样 PCD、屏幕 UI 和启动脚本。
- Odin 驱动 Apache-2.0 许可证。

未保留：

- 嵌套 `.git`、`__pycache__`、日志、备份、构建/安装目录。
- 未被比赛配置引用的候选模型与候选 TensorRT engine。
- 开发计划、任务草稿、重复地图工具和运行时轨迹。
- 原备份中大小为 0 的浏览器静态页面；HTTP JSON API 和屏幕 UI 源码仍保留。

TensorRT engine 与 JetPack、TensorRT 版本及 GPU 架构有关；其他机器应从同名 ONNX 重新生成，不应默认复用比赛 engine。模型哈希见 [`policies/README.md`](policies/README.md)。

## 安全与开源状态

- 真机运行前必须架空轮组验证 CAN 映射、方向、零位、急停和限幅。
- `deployment_contract.hpp` 是电机映射和动作缩放真值源；参考 YAML 不会自动修改 C++ 契约。
- 自研 ROS 包的 `package.xml` 仍保留原工程的 `Proprietary` 字段。迁移到 GitHub 公共开源前，需要由项目负责人选择许可证并统一修改；本次整理不代替权利人作许可证决定。
- 当前 Windows 环境只能做静态检查，不能证明 ROS 2、SocketCAN、Odin SDK 或 TensorRT 真机运行成功。
