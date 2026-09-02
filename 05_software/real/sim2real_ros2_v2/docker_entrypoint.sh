#!/bin/bash
set -e

# Source ROS2 Humble environment
source /opt/ros/humble/setup.bash

# Source workspace install setup if compiled
if [ -f "/sim2real_ws/install/setup.bash" ]; then
    source /sim2real_ws/install/setup.bash
fi

exec "$@"
