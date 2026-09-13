#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Define color codes for pretty output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}====================================================${NC}"
echo -e "${GREEN}      Starting Sim2Real Locomotion ROS2 Stack       ${NC}"
echo -e "${YELLOW}====================================================${NC}"

# 1. Source ROS2 Humble environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "[System] Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
else
    echo -e "${RED}[Error] ROS2 Humble not found. Please install ROS2 Humble first.${NC}"
    exit 1
fi

# 2. Check if local workspace is compiled and source it
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

if [ -f "install/setup.bash" ]; then
    echo -e "[Workspace] Sourcing local workspace..."
    source install/setup.bash
elif [ -f "../../install/setup.bash" ]; then
    echo -e "[Workspace] Sourcing parent install/setup.bash..."
    source ../../install/setup.bash
else
    echo -e "${YELLOW}[Warning] install/setup.bash not found. Attempting to build the workspace first...${NC}"
    if command -v colcon &> /dev/null; then
        echo -e "[Build] Running colcon build..."
        colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
        source install/setup.bash
    else
        echo -e "${RED}[Error] 'colcon' tool not found. Please compile the workspace manually before running.${NC}"
        exit 1
    fi
fi

# 3. Check for SocketCAN interfaces (in non-dry-run mode)
# Reading dry_run parameter from yaml config
if [ -f "src/sim2real_bringup/config/runtime.yaml" ]; then
    # Use sed for portability (busybox-compatible, avoids GNU grep -oP dependency)
    DRY_RUN=$(sed -n 's/^[[:space:]]*dry_run:[[:space:]]*//p' src/sim2real_bringup/config/runtime.yaml | head -n 1 || echo "true")
    # Trim trailing whitespace/newlines
    DRY_RUN=$(echo "$DRY_RUN" | tr -d '[:space:]')
else
    DRY_RUN="true"
fi

if [ "$DRY_RUN" = "false" ]; then
    echo -e "[Network] Checking CAN interfaces..."
    if ip link show can0 &> /dev/null && ip link show can1 &> /dev/null; then
        echo -e "[Network] can0 and can1 interfaces detected."
    else
        echo -e "${YELLOW}[Warning] CAN interfaces (can0/can1) not fully active.${NC}"
        echo -e "To configure CAN interfaces, run:"
        echo -e "  sudo ip link set can0 up type can bitrate 1000000"
        echo -e "  sudo ip link set can1 up type can bitrate 1000000"
    fi
else
    echo -e "${YELLOW}[Dry-Run] Running in Dry-Run mode. SocketCAN will not be accessed.${NC}"
fi

# 4. Run the ROS2 Launch file
echo -e "${GREEN}[Launch] Starting sim2real launch file...${NC}"
ros2 launch sim2real_bringup sim2real_system.launch.py "$@"

