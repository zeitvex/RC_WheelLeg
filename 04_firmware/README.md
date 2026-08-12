# 嵌入式固件

本目录保存运行在 MCU 上的固件源码和构建工程。Keil 工程属于 firmware；编译生成的 `.hex`、`.bin`、`.axf` 等属于构建产物。

```text
04_firmware/
└─ stm32/
   └─ dji_a_board/
      ├─ RCPROyuan/
      └─ rc_/
```

两个目录均包含 STM32F427、FreeRTOS、RS02 电机控制和 Keil MDK 工程。当前保留原名，待确认中期检查实际烧录版本后再决定是否改为 `baseline`、`midterm` 等语义名称。

工程入口均为：

```text
Project/RoboMentors_Board.uvprojx
```

根目录 `.gitignore` 排除 Keil 编译输出和用户状态文件。预编译库 `Inc/RoboMentors_Board1_3.lib` 暂时保留以维持原工程依赖，其再分发条件仍需确认。
