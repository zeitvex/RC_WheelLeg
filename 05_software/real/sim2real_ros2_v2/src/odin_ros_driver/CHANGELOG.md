v0.10.5 2026_0525
1. 修改 recorddata 数据格式，新增 device_id、algorithm_version 字段
2. 修复 SLAM 模式下下载地图失败的问题(USB2.0)
3. 修复重定位模式下上传地图失败的问题(USB2.0)

v0.10.4 2026_0522
1. 修复 USB2.0 心跳超时导致软断开的问题
2. control_command.yaml 新增 custom_init_pose_search_radius 和 custom_init_pose_max_rot_deg 参数