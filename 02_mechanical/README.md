# 16DOF 串联轮足机械

本目录保存山东华宇工学院 16DOF 串联轮足机器人的机械设计资料。

机器人由 12 个腿部关节和 4 个驱动轮组成，共 16 个执行器。源资料说明其使用 SolidWorks 26 版本完成设计、装配和 URDF 导出准备。

## 目录结构

```text
02_mechanical/
├─ CAD/
│  └─ solidworks/
│     ├─ WEEKDOG.SLDASM       # 整机装配体入口
│     ├─ 轮子完整.SLDASM        # 轮组装配体
│     ├─ 轮毂垫片.SLDPRT        # 独立零件
│     └─ 26版sw单件/            # 单件与子装配体
├─ STEP/
│  ├─ WEEKDOG.STEP            # 整机通用交换文件
│  └─ *.STEP                  # 身体、关节、腿部和轮组
└─ source_notes.txt           # 原始资料说明
```

## 文件统计

- SolidWorks 装配体：4 个 `*.SLDASM`
- SolidWorks 零件：26 个 `*.SLDPRT`
- STEP 文件：10 个

## 使用方式

- 需要继续编辑设计时，从 `CAD/solidworks/WEEKDOG.SLDASM` 打开整机。
- 只需查看、测量或导入其他 CAD 软件时，使用 `STEP/WEEKDOG.STEP`。
- 子部件调试可以使用身体、髋关节、大腿、轮小腿和轮组 STEP。
- 为避免装配引用丢失，不要单独移动或重命名 `CAD/solidworks` 内部文件。

## 当前缺项

- 尚未提供关键零件二维 PDF 工程图。
- 尚未形成 BOM、装配步骤和干涉检查报告。
- 尚未核对 CAD 参数与训练使用的 URDF/MJCF 是否完全一致。
