## 工程文件框架总览（RCPRO）

- 工程类型：STM32F4（大疆 RoboMentors A 型板）四足机器人控制工程  
- 工具链：STM32CubeMX 初始化 + Keil MDK 工程（Project 目录）  
- 主要语言：C / C++（以 C 为主）  

---

## 顶层目录结构

- Inc/  
  - 工程公共头文件与 HAL、CMSIS、FreeRTOS 相关头文件
- User/  
  - 用户自定义代码（算法、驱动、任务、启动文件等）
- Project/  
  - Keil 工程文件、编译输出中间文件、调试配置
- .vscode/  
  - VS Code 配置（智能提示、调试配置等）
- 文档与说明文件  
  - README.md、DBUS.md、can.md、app_can.md、姿态.md、辅助通道.md 等  
  - RS02使用说明书251112.pdf、遥控器相关 md 文档
- 其他工具与脚本  
  - keilkilll.bat 等

---

## Inc 目录结构

- Inc/hal_inc/  
  - STM32 HAL 库头文件（GPIO、RCC、CAN、UART、SPI、TIM、RTC 等）  
  - 由 STM32CubeMX 生成或随库提供
- Inc/m3_inc/  
  - CMSIS 相关头文件（内核 core_cmX、stm32f4xx.h、系统时钟 system_stm32f4xx.h 等）  
  - 提供底层寄存器与内核抽象
- Inc/os_inc/  
  - FreeRTOS 核心头文件（FreeRTOS.h、task.h、queue.h、semphr.h 等）  
  - CMSIS-OS 适配层 cmsis_os.h
- Inc 根目录下关键头文件：  
  - main.h：主函数相关声明  
  - gpio.h、dma.h、adc.h、tim.h、spi.h、usart.h、rtc.h：外设初始化接口  
  - can.h：CAN 相关句柄与接口声明  
  - FreeRTOSConfig.h：FreeRTOS 配置  
  - stm32f4xx_hal_conf.h：HAL 外设使能配置  
  - stm32f4xx_it.h / stm32f4xx_it.c：中断向量与中断服务函数

---

## User 目录结构

- User/algorithm/  
  - pid.c / pid.h  
    - PID 控制算法实现（位置/速度等）  
  - ramp.c / ramp.h  
    - 斜坡函数、平滑过渡控制（例如速度/位置渐变）

- User/driver/  
  - Movement.c / Movement.h  
    - 四足运动控制与步态生成（站立、行走等高层运动逻辑）  
  - leg.c / leg.h  
    - 机械腿几何建模与逆运动学计算  
  - lingzu_motor.c / lingzu_motor.h  
    - 灵足 RS02 关节电机 CAN 通讯与控制接口  
  - can_device.c / can_device.h  
    - CAN 收发封装、设备层抽象  
  - uart_device.c / uart_device.h  
    - UART 封装（遥控器、调试串口等）  
  - mpu6500_reg.h / ist8310_reg.h  
    - IMU、磁力计寄存器定义  
  - keyboard.h、calibrate.h 等  
    - 键盘控制、校准相关接口

- User/app/  
  - lingzu_task.c / lingzu_task.h  
    - 主要控制任务入口：四足行走、站立、模式切换等  
  - uart_task.c / uart_task.h  
    - 串口任务（接收/解析/发送数据）  
  - study_task.c / study_task.h  
    - 学习/实验用任务逻辑  
  - chassis_task.h、gimbal_task.h、shoot_custom.c、execute_task.h 等  
    - 源于 RM 工程的任务接口，可复用/裁剪

- User 其他文件  
  - startup.c / startup.h  
    - 启动相关代码（复位后初始化、任务启动等）  
  - rm_hal_lib.h  
    - RoboMentors 板级相关封装  
  - sys.h  
    - 系统级通用定义与工具

---

## Project 目录结构（Keil 工程）

- Project/RoboMentors_Board/  
  - RoboMentors_Board.uvprojx / .uvoptx / .uvguix.*  
    - Keil 工程、编译配置、调试配置文件  
  - *.o / *.d / *.crf  
    - 中间目标文件与依赖信息（编译产物）  
  - RoboMentors_Board.axf / .hex / .map / .htm  
    - 最终固件、链接映射与编译报告

- Project/DebugConfig/  
  - RoboMentors_Board_STM32F427IIHx*.dbgconf  
    - 调试器与目标芯片调试配置

- Project/output/  
  - RoboMentors_Board.map  
    - 链接映射文件（地址分布、段信息等）

- Project 根目录其他文件  
  - JLinkLog.txt / JLinkSettings.ini  
    - J-Link 调试相关日志与配置  
  - images.jpg  
    - 可能用于说明或调试记录

---

## 文档与辅助说明

- README.md  
  - 工程整体说明（来源、用途等）  
- DBUS.md / can.md / app_can.md  
  - 遥控器 DBUS、CAN 通信相关说明  
- move.c.md、姿态.md、辅助通道.md  
  - 与运动控制、姿态解算、辅助通道配置相关的说明文档  
- # RadioLink AT9S Pro 遥控器使用说明书（详细版）.md  
  - 遥控器详细使用说明
- RS02使用说明书251112.pdf  
  - RS02 关节电机官方说明书

---

## 建议阅读顺序（熟悉工程）

- 1）阅读顶层 README.md 与遥控器 / RS02 说明文档  
- 2）查看 User/driver/ 中的 lingzu_motor.c、leg.c、Movement.c  
- 3）查看 User/app/lingzu_task.c，理解任务入口与模式切换  
- 4）根据需要再深入 Inc/、Project/ 与其他说明 md 文档

