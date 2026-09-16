# 项目文档

本目录用于保存 16DOF 轮足项目自身的技术文档和使用说明。

当前文档结构：

```text
01_doc/
├─ architecture/
│  └─ early_software_stack.md  # 第一代训练—仿真—真机闭环
├─ training_evolution.md       # v0.4～v0.6 训练架构演进
└─ version_history.md          # 全项目 Tag 与里程碑
```

具体运行说明放在对应工程目录内，避免在顶层重复并逐渐失真：训练见 `05_software/train/rc_mjlab/`，真机部署见 `05_software/real/`。
