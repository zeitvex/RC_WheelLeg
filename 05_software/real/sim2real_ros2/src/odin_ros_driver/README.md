# Odin 驱动依赖占位

`real/sim2real_ros2` 原始快照中的 `src/odin_ros_driver` 为空目录，但启动文件、Dockerfile 和 `sim2real_bringup` 已经引用该包。

因此 `v0.10.0` 记录的是 ROS 2/C++ 迁移初版，不能仅凭本目录宣称 Odin 驱动可独立构建。兼容的 Odin ROS 2 驱动源码从后续版本开始随工程归档。
