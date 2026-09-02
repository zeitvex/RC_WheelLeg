# 使用 ROS2 官方 Humble 基础镜像
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

# 安装 C++ 编译依赖、SocketCAN 调试工具及 Eigen 等核心库
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    can-utils \
    libyaml-cpp-dev \
    libeigen3-dev \
    libusb-1.0-0-dev \
    libpcl-dev \
    libopencv-dev \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-cv-bridge \
    ros-humble-pcl-conversions \
    wget \
    tar \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ============================================================================
# ONNX Runtime — 架构自适应，aarch64 启用 CUDA GPU 加速
# ============================================================================
#   - Orin Nano (aarch64): pip 安装 onnxruntime-gpu（含 CUDA EP）
#   - x86_64 开发机: 下载 CPU-only 预编译包（GPU 不可用）
WORKDIR /opt
RUN ARCH=$(uname -m) && \
    if [ "$ARCH" = "aarch64" ]; then \
        echo "[ONNX] Installing CUDA-enabled ONNX Runtime for Jetson Orin..." && \
        pip3 install --no-cache-dir onnxruntime-gpu && \
        SITE_PKGS=$(python3 -c "import site; print(site.getsitepackages()[0])") && \
        mkdir -p onnxruntime/include onnxruntime/lib && \
        cp -r "$SITE_PKGS/onnxruntime/include/"* onnxruntime/include/ && \
        cp "$SITE_PKGS/onnxruntime/capi/libonnxruntime.so"* onnxruntime/lib/ && \
        echo "[ONNX] CUDA ONNX Runtime installed."; \
    else \
        echo "[ONNX] Installing CPU-only ONNX Runtime for x86_64 dev..." && \
        wget -q https://github.com/microsoft/onnxruntime/releases/download/v1.16.3/onnxruntime-linux-x64-1.16.3.tgz && \
        tar -zxf onnxruntime-linux-x64-1.16.3.tgz && \
        mv onnxruntime-linux-x64-1.16.3 onnxruntime && \
        rm onnxruntime-linux-x64-1.16.3.tgz; \
    fi

ENV ONNXRUNTIME_DIR=/opt/onnxruntime

# 创建工作空间，将所有 C++ 源码包拷入
WORKDIR /sim2real_ws/src
COPY src/sim2real_bringup sim2real_bringup
COPY src/sim2real_common sim2real_common
COPY src/sim2real_hw sim2real_hw
COPY src/sim2real_interfaces sim2real_interfaces
COPY src/sim2real_runtime sim2real_runtime
COPY src/odin_ros_driver odin_ros_driver
COPY src/sim2real_nav2 sim2real_nav2

# 拷贝策略文件与运行脚本
WORKDIR /sim2real_ws
COPY policies policies
COPY start_sim2real.sh start_sim2real.sh
RUN chmod +x start_sim2real.sh

# 编译 ROS2 工作空间
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 拷贝 Docker 入口脚本并设置
COPY docker_entrypoint.sh /docker_entrypoint.sh
RUN chmod +x /docker_entrypoint.sh

ENTRYPOINT ["/docker_entrypoint.sh"]
CMD ["./start_sim2real.sh"]
