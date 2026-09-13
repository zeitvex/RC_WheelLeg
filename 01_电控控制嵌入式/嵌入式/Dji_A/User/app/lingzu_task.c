#include "lingzu_task.h"
#include "lingzu_motor.h"
#include "cmsis_os.h"
#include "rm_hal_lib.h" // for BEEP macros if needed
#include <math.h>
#include "Movement.h"
#include "leg.h"
#include "uart_task.h"
#include "uart_device.h" // 读取 rc.swD 状态实现软急停
// 取消注释以启用零点校准模式
//#define CALIBRATE_ZERO_POINT 

#define TO_ZERO_TIME 10000.0f
#define PI 3.1415926535f

void lingzu_task(const void* argu)
{
    // Local variables
    // int run_count = 0;
    // float init_p_1 = 4.0f;
    // float init_p_2 = 4.0f;
    // float to_zero_speed = TO_ZERO_TIME;
    // int sin_time = 12000;
    // float position_wide = 0.5f;
    // char init_flag = 0;
    // float MAX_T = 0;
    RobotGeometry geom = { 
        .L1 = 250.0f,       // 大腿长 (mm) 
        .L2 = 270.0f,       // 小腿长 (mm) 
        .stance_width = 120.0f // 站立宽度 (mm) 
    };

    TrajectoryParams p = {
            .step_length = 100.0f,
            .step_height = -40.0f, // 增大抬腿高度 (负值)
            .period = 0.5f,//0.5f,        // 加快频率 (步频 周期)
            .start_x = 0.0f,
            .start_y = 0.0f,
            .start_z = 300.0f, // 正常站立高度
            .duty_cycle = 0.5f // 标准 Trot 占空比
        };


    // Initialization delay
    osDelay(2000);

    // Initialize Motor Structs (IDs, limits, etc.)
    Lingzu_Motor_Init_Structs();
    osDelay(100);

    // --- Robust Initialization Logic ---
    // Force Enable All Motors First (Blind Enable) to wake them up
    // Repeat twice to ensure command delivery
    ENABLE_ALL_LINGZU_MOTORS();
    osDelay(50);
    ENABLE_ALL_LINGZU_MOTORS();
    osDelay(50);

    // 2025-01-26 Fix: Loop until all motors are confirmed online via timestamp
    // Using last_update_time to strictly track connectivity
    
    uint32_t init_start = HAL_GetTick();
    
    while (HAL_GetTick() - init_start < 5000) // Timeout 5s
    {
        uint32_t now = HAL_GetTick();
        int all_ready = 1;
        
        // Check Feedback & Re-enable if needed
        // Timeout threshold: 200ms without feedback = offline
        
        // CAN1 Motors (ID 1-4)
        if (now - CAN_1.ID_1_Motor_recieve.last_update_time > 200) { 
            Motor_Enable(&hcan1, &CAN_1.ID_1_Motor_send); 
            all_ready = 0; 
            osDelay(2); 
        }
        if (now - CAN_1.ID_2_Motor_recieve.last_update_time > 200) { 
            Motor_Enable(&hcan1, &CAN_1.ID_2_Motor_send); 
            all_ready = 0; 
            osDelay(2); 
        }
        if (now - CAN_1.ID_3_Motor_recieve.last_update_time > 200) { 
            Motor_Enable(&hcan1, &CAN_1.ID_3_Motor_send); 
            all_ready = 0; 
            osDelay(2); 
        }
        if (now - CAN_1.ID_4_Motor_recieve.last_update_time > 200) { 
            Motor_Enable(&hcan1, &CAN_1.ID_4_Motor_send); 
            all_ready = 0; 
            osDelay(2); 
        }

        // CAN2 Motors (ID 1-4)
        if (now - CAN_2.ID_1_Motor_recieve.last_update_time > 200) { 
            Motor_Enable(&hcan2, &CAN_2.ID_1_Motor_send); 
            all_ready = 0; 
            osDelay(2); 
        }
        if (now - CAN_2.ID_2_Motor_recieve.last_update_time > 200) { 
            Motor_Enable(&hcan2, &CAN_2.ID_2_Motor_send); 
            all_ready = 0; 
            osDelay(2); 
        }
        if (now - CAN_2.ID_3_Motor_recieve.last_update_time > 200) { 
            Motor_Enable(&hcan2, &CAN_2.ID_3_Motor_send); 
            all_ready = 0; 
            osDelay(2); 
        }
        if (now - CAN_2.ID_4_Motor_recieve.last_update_time > 200) { 
            Motor_Enable(&hcan2, &CAN_2.ID_4_Motor_send); 
            all_ready = 0; 
            osDelay(2); 
        }

        if (all_ready) break;
        osDelay(50); // Check frequency ~20Hz
    }

    // Double Check: If initialization timed out, some motors might have invalid (0) positions.
    // We MUST NOT calculate offsets for invalid motors.
    // However, to be safe, we will try one last fetch or just proceed with caution.
    // Better strategy: Since we loop later, let's just ensure we don't use stale data.
    
    // Still perform a few control cycles to stabilize data
    for(int i=0; i<20; i++) { // Increased to 20 cycles
        CAN_Send_Control(&hcan1, &CAN_1.ID_1_Motor_send);
        CAN_Send_Control(&hcan1, &CAN_1.ID_2_Motor_send);
        CAN_Send_Control(&hcan1, &CAN_1.ID_3_Motor_send);
        CAN_Send_Control(&hcan1, &CAN_1.ID_4_Motor_send);
        osDelay(1);
        CAN_Send_Control(&hcan2, &CAN_2.ID_1_Motor_send);
        CAN_Send_Control(&hcan2, &CAN_2.ID_2_Motor_send);
        CAN_Send_Control(&hcan2, &CAN_2.ID_3_Motor_send);
        CAN_Send_Control(&hcan2, &CAN_2.ID_4_Motor_send);
        osDelay(9);
    }

    // --- Critical: Re-validate connectivity before Offset Calculation ---
    // If a motor is still offline, we try to wake it up one last time.
    // This handles cases where the initial loop exited due to timeout but some motors recovered.
    uint32_t pre_calib_check = HAL_GetTick();
    if (pre_calib_check - CAN_1.ID_1_Motor_recieve.last_update_time > 500) Motor_Enable(&hcan1, &CAN_1.ID_1_Motor_send);
    if (pre_calib_check - CAN_1.ID_2_Motor_recieve.last_update_time > 500) Motor_Enable(&hcan1, &CAN_1.ID_2_Motor_send);
    if (pre_calib_check - CAN_1.ID_3_Motor_recieve.last_update_time > 500) Motor_Enable(&hcan1, &CAN_1.ID_3_Motor_send);
    if (pre_calib_check - CAN_1.ID_4_Motor_recieve.last_update_time > 500) Motor_Enable(&hcan1, &CAN_1.ID_4_Motor_send);
    
    if (pre_calib_check - CAN_2.ID_1_Motor_recieve.last_update_time > 500) Motor_Enable(&hcan2, &CAN_2.ID_1_Motor_send);
    if (pre_calib_check - CAN_2.ID_2_Motor_recieve.last_update_time > 500) Motor_Enable(&hcan2, &CAN_2.ID_2_Motor_send);
    if (pre_calib_check - CAN_2.ID_3_Motor_recieve.last_update_time > 500) Motor_Enable(&hcan2, &CAN_2.ID_3_Motor_send);
    if (pre_calib_check - CAN_2.ID_4_Motor_recieve.last_update_time > 500) Motor_Enable(&hcan2, &CAN_2.ID_4_Motor_send);
    
    // Give a small window for the last-ditch wake-up to work
    osDelay(100);

    // 如果一直没反馈，可能是真的在0位置，或者CAN断了。
    // 为了安全，强制 valid_feedback = 1 (假设至少跑过循环了)
    // 但如果 current_position 真的是 0 (刚好在原点)，Offset = 0 - Theoretical. 也是对的。
    // 唯一怕的是 CAN 没初始化好读出来是 0 但实际电机在 100。
    // 既然延时了1秒并发送了查询，应该就绪了。

#ifdef CALIBRATE_ZERO_POINT
    // --- 零点校准模式 ---
    // 在此模式下，电机输出零力矩，允许用户手动移动机器人腿部到"零位"
    // 上电5秒后，自动将当前位置设为电机零点
    uint32_t calibration_start_time = HAL_GetTick();
    int zero_set_done = 0;

    while(1)
    {
        // 检查是否达到5秒
        if (!zero_set_done && (HAL_GetTick() - calibration_start_time > 5000))
        {
            // 发送设置零点指令
            // CAN 1
            Motor_Zore(&hcan1, &CAN_1.ID_1_Motor_send);
            Motor_Zore(&hcan1, &CAN_1.ID_2_Motor_send);
            Motor_Zore(&hcan1, &CAN_1.ID_3_Motor_send);
            Motor_Zore(&hcan1, &CAN_1.ID_4_Motor_send);
            
            // CAN 2
            Motor_Zore(&hcan2, &CAN_2.ID_1_Motor_send);
            Motor_Zore(&hcan2, &CAN_2.ID_2_Motor_send);
            Motor_Zore(&hcan2, &CAN_2.ID_3_Motor_send);
            Motor_Zore(&hcan2, &CAN_2.ID_4_Motor_send);
            
            zero_set_done = 1;
            // 可以加一个提示，例如改变某个变量或者如果有蜂鸣器响一下
        }

        // 设置所有电机为零力矩模式 (Kp=0, Kd=0, Torque=0)
        // 保持通信以更新反馈
        
        // CAN 1
        CAN_1.ID_1_Motor_send.torque = 0; CAN_1.ID_1_Motor_send.kp = 0; CAN_1.ID_1_Motor_send.kd = 0; CAN_1.ID_1_Motor_send.speed = 0; CAN_1.ID_1_Motor_send.position = 0;
        CAN_1.ID_2_Motor_send.torque = 0; CAN_1.ID_2_Motor_send.kp = 0; CAN_1.ID_2_Motor_send.kd = 0; CAN_1.ID_2_Motor_send.speed = 0; CAN_1.ID_2_Motor_send.position = 0;
        CAN_1.ID_3_Motor_send.torque = 0; CAN_1.ID_3_Motor_send.kp = 0; CAN_1.ID_3_Motor_send.kd = 0; CAN_1.ID_3_Motor_send.speed = 0; CAN_1.ID_3_Motor_send.position = 0;
        CAN_1.ID_4_Motor_send.torque = 0; CAN_1.ID_4_Motor_send.kp = 0; CAN_1.ID_4_Motor_send.kd = 0; CAN_1.ID_4_Motor_send.speed = 0; CAN_1.ID_4_Motor_send.position = 0;
        
        // CAN 2
        CAN_2.ID_1_Motor_send.torque = 0; CAN_2.ID_1_Motor_send.kp = 0; CAN_2.ID_1_Motor_send.kd = 0; CAN_2.ID_1_Motor_send.speed = 0; CAN_2.ID_1_Motor_send.position = 0;
        CAN_2.ID_2_Motor_send.torque = 0; CAN_2.ID_2_Motor_send.kp = 0; CAN_2.ID_2_Motor_send.kd = 0; CAN_2.ID_2_Motor_send.speed = 0; CAN_2.ID_2_Motor_send.position = 0;
        CAN_2.ID_3_Motor_send.torque = 0; CAN_2.ID_3_Motor_send.kp = 0; CAN_2.ID_3_Motor_send.kd = 0; CAN_2.ID_3_Motor_send.speed = 0; CAN_2.ID_3_Motor_send.position = 0;
        CAN_2.ID_4_Motor_send.torque = 0; CAN_2.ID_4_Motor_send.kp = 0; CAN_2.ID_4_Motor_send.kd = 0; CAN_2.ID_4_Motor_send.speed = 0; CAN_2.ID_4_Motor_send.position = 0;

        if (!zero_set_done) {
             // 还没设置零点时，发送控制帧保持通信
            CAN_Send_Control(&hcan1, &CAN_1.ID_1_Motor_send);
            CAN_Send_Control(&hcan1, &CAN_1.ID_2_Motor_send);
            CAN_Send_Control(&hcan1, &CAN_1.ID_3_Motor_send);
            CAN_Send_Control(&hcan1, &CAN_1.ID_4_Motor_send);
            
            CAN_Send_Control(&hcan2, &CAN_2.ID_1_Motor_send);
            CAN_Send_Control(&hcan2, &CAN_2.ID_2_Motor_send);
            CAN_Send_Control(&hcan2, &CAN_2.ID_3_Motor_send);
            CAN_Send_Control(&hcan2, &CAN_2.ID_4_Motor_send);
        } else {
             // 设置完零点后，继续发送零力矩帧，防止超时
            CAN_Send_Control(&hcan1, &CAN_1.ID_1_Motor_send);
            CAN_Send_Control(&hcan1, &CAN_1.ID_2_Motor_send);
            CAN_Send_Control(&hcan1, &CAN_1.ID_3_Motor_send);
            CAN_Send_Control(&hcan1, &CAN_1.ID_4_Motor_send);
            
            CAN_Send_Control(&hcan2, &CAN_2.ID_1_Motor_send);
            CAN_Send_Control(&hcan2, &CAN_2.ID_2_Motor_send);
            CAN_Send_Control(&hcan2, &CAN_2.ID_3_Motor_send);
            CAN_Send_Control(&hcan2, &CAN_2.ID_4_Motor_send);
        }
        
        osDelay(10);
    }
#endif

     // --- Offset Calibration Logic ---
    // 定义姿态高度常量
    const float PRONE_Z = 120.0f;   // 趴下高度 (mm)
    const float STAND_Z = 300.0f;   // 站立高度 (mm)
    
    // 当前基础高度 (初始化为趴下)
    float current_base_z = PRONE_Z;

     // 计算理论角度 (使用趴下高度)
     float theoretical_q1, theoretical_q2;
     Inverse_Calculation(0, current_base_z, &theoretical_q1, &theoretical_q2, &geom);
     
     float motor_offsets[9] = {0}; // Index 1-8
     
     // 假设当前时刻电机处于 "current_base_z 趴下姿态"
     if (1) { 
         // 计算偏移量
         // 左前腿(FL, Index 0/1,2) 物理安装可能反向，需要 -Theoretical
         // 右前腿(FR, Index 1/3,4) 正常，使用 +Theoretical
         // 假设后腿同理：RL 反向，RR 正常
         
         // 2025-01-26 Fix: 增加多圈角度归一化处理，防止转优弧 (大圈)
         // 将 Current Position 映射到 [-PI, PI] 附近，避免多圈误差
         // 目标是让 offset + theoretical ≈ current
         
         // (Removed C++ lambda, logic is handled in the loop below)

         // FL (Left) - Inverted
         motor_offsets[1] = CAN_1.ID_1_Motor_recieve.current_position_f - (-theoretical_q1);
         motor_offsets[2] = CAN_1.ID_2_Motor_recieve.current_position_f - (-theoretical_q2);
         
         // FR (Right) - Normal (Matched to RR)
        // 2025-01-26 Fix: FR set to 1.0 (Normal) to rotate CCW during stand.
        motor_offsets[3] = CAN_1.ID_3_Motor_recieve.current_position_f - theoretical_q1;
        motor_offsets[4] = CAN_1.ID_4_Motor_recieve.current_position_f - theoretical_q2;
        
        // RL (Left) - Inverted
        motor_offsets[5] = CAN_2.ID_1_Motor_recieve.current_position_f - (-theoretical_q1);
         motor_offsets[6] = CAN_2.ID_2_Motor_recieve.current_position_f - (-theoretical_q2);
         
         // RR (Right) - Normal
         motor_offsets[7] = CAN_2.ID_3_Motor_recieve.current_position_f - theoretical_q1;
         motor_offsets[8] = CAN_2.ID_4_Motor_recieve.current_position_f - theoretical_q2;
         

     } 
 
     // --------------------------------

    // Initial Control Parameters for 1 rad/s Velocity Mode
    // MIT Mode: Torque = Kp*(P_des - P) + Kd*(V_des - V) + T_ff
    // For velocity control: Set Kp=0, V_des=Target, Kd=Gain
    
    // CAN1 ID1


	// Posture(1, 2.0f, 0, 2.0f, 2.0f,0);
	
	
	// Posture(2, 2.0f, 0, 2.0f, 2.0f,0);
    
	
	// CAN2 ID1
    // CAN_2.ID_1_Motor_send.position = 0;
    // CAN_2.ID_1_Motor_send.speed = 1.0f;
    // CAN_2.ID_1_Motor_send.torque = 0;
    // CAN_2.ID_1_Motor_send.kp = 0;
    // CAN_2.ID_1_Motor_send.kd = 2.0f;

    // // CAN1 ID4
    // CAN_1.ID_4_Motor_send.position = 0;
    // CAN_1.ID_4_Motor_send.speed = 1.0f;
    // CAN_1.ID_4_Motor_send.torque = 0;
    // CAN_1.ID_4_Motor_send.kp = 0;
    // CAN_1.ID_4_Motor_send.kd = 1.0f;

    // Get initial positions from feedback (Optional, not needed for pure velocity mode)
    // init_p_1 = CAN_1.ID_1_Motor_recieve.current_position_f;
    // init_p_2 = CAN_2.ID_1_Motor_recieve.current_position_f;
//	int b=1;
    // 状态变量
    enum {
        STATE_PRONE = 0,
        STATE_STAND = 1,
        STATE_WALK = 2
    } robot_state = STATE_PRONE;

    // State flags for delayed initialization
    uint8_t motor_init_status[9] = {0}; // 1-8. 0=Not Initialized, 1=Initialized
    
    while(1)
    {
        // --- 0. Robust Re-connection and Offset Calibration Logic ---
        static uint32_t last_check_time = 0;
        uint32_t now_tick = HAL_GetTick();
        
        // SwD 上(软急停): 禁止自动 Motor_Enable，防止电机被后台逻辑重新拉起
        uint8_t estop_active = (rc.swD == RC_UP);
        
        if (!estop_active && (now_tick - last_check_time > 500)) // 2Hz Check
        {
            last_check_time = now_tick;
            
            // Check CAN1 Motors
            for(int i=1; i<=4; i++) {
                // Determine motor struct pointer
                Motor_CAN_Recieve_Struct *rx_ptr = NULL;
                Motor_CAN_Send_Struct *tx_ptr = NULL;
                CAN_HandleTypeDef *hcan_ptr = &hcan1;
                
                switch(i) {
                    case 1: rx_ptr = &CAN_1.ID_1_Motor_recieve; tx_ptr = &CAN_1.ID_1_Motor_send; break;
                    case 2: rx_ptr = &CAN_1.ID_2_Motor_recieve; tx_ptr = &CAN_1.ID_2_Motor_send; break;
                    case 3: rx_ptr = &CAN_1.ID_3_Motor_recieve; tx_ptr = &CAN_1.ID_3_Motor_send; break;
                    case 4: rx_ptr = &CAN_1.ID_4_Motor_recieve; tx_ptr = &CAN_1.ID_4_Motor_send; break;
                }
                
                if (now_tick - rx_ptr->last_update_time > 500) {
                    // Offline: Send Enable and Reset Init Status
                    Motor_Enable(hcan_ptr, tx_ptr);
                    motor_init_status[i] = 0; 
                } else if (motor_init_status[i] == 0) {
                    // Online but not initialized: Calculate Offset
                    // CRITICAL CHECK: Wait for valid position data
                    // If pos is exactly 0.0, it might be an empty struct init
                    // But motor can be at 0.0. Check timestamp is recent (already done above)
                    // and maybe check if we have received AT LEAST one frame since boot?
                    // rx_ptr->last_update_time > 0 ensures we received something.
                    
                    if (rx_ptr->last_update_time == 0) continue; // Should not happen given outer if, but safety first

                    // Re-calculate theoreticals for current Z (likely Prone if just started)
                    float t_q1, t_q2;
                    Inverse_Calculation(0, current_base_z, &t_q1, &t_q2, &geom);
                    
                    // FL (1,2) Inverted, FR (3,4) Normal
                    if (i==1) motor_offsets[1] = rx_ptr->current_position_f - (-t_q1);
                    else if (i==2) motor_offsets[2] = rx_ptr->current_position_f - (-t_q2);
                    else if (i==3) motor_offsets[3] = rx_ptr->current_position_f - t_q1;
                    else if (i==4) motor_offsets[4] = rx_ptr->current_position_f - t_q2;
                    
                    motor_init_status[i] = 1;
                }
            }
            
            // Check CAN2 Motors
            for(int i=1; i<=4; i++) {
                Motor_CAN_Recieve_Struct *rx_ptr = NULL;
                Motor_CAN_Send_Struct *tx_ptr = NULL;
                CAN_HandleTypeDef *hcan_ptr = &hcan2;
                int global_idx = i + 4; // 5-8
                
                switch(i) {
                    case 1: rx_ptr = &CAN_2.ID_1_Motor_recieve; tx_ptr = &CAN_2.ID_1_Motor_send; break;
                    case 2: rx_ptr = &CAN_2.ID_2_Motor_recieve; tx_ptr = &CAN_2.ID_2_Motor_send; break;
                    case 3: rx_ptr = &CAN_2.ID_3_Motor_recieve; tx_ptr = &CAN_2.ID_3_Motor_send; break;
                    case 4: rx_ptr = &CAN_2.ID_4_Motor_recieve; tx_ptr = &CAN_2.ID_4_Motor_send; break;
                }
                
                if (now_tick - rx_ptr->last_update_time > 500) {
                    Motor_Enable(hcan_ptr, tx_ptr);
                    motor_init_status[global_idx] = 0;
                } else if (motor_init_status[global_idx] == 0) {
                    
                    if (rx_ptr->last_update_time == 0) continue;

                    float t_q1, t_q2;
                    Inverse_Calculation(0, current_base_z, &t_q1, &t_q2, &geom);
                    
                    // RL (1,2) Inverted, RR (3,4) Normal
                    if (i==1) motor_offsets[5] = rx_ptr->current_position_f - (-t_q1);
                    else if (i==2) motor_offsets[6] = rx_ptr->current_position_f - (-t_q2);
                    else if (i==3) motor_offsets[7] = rx_ptr->current_position_f - t_q1;
                    else if (i==4) motor_offsets[8] = rx_ptr->current_position_f - t_q2;
                    
                    motor_init_status[global_idx] = 1;
                }
            }
        }

        // 1. 检查队列中的控制命令
        if(led_control_queue != NULL)
        {
            char recv_data;
            // 非阻塞接收 (wait time = 0)
            if(xQueueReceive(led_control_queue, &recv_data, 0) == pdTRUE)
            {
                // 收到数据，翻转 LED2 作为调试指示
                static uint8_t led_state = 0;
                led_state = !led_state;
                write_led_io(LED_IO2, led_state ? LED_ON : LED_OFF);

                switch(recv_data)
                {
                    case 'q':
                        // 只有在站立或行走状态下才能切换到行走
                        if (robot_state != STATE_PRONE) {
                            robot_state = STATE_WALK;
                        }
                        break;
                    case 's':
                        robot_state = STATE_STAND; // 切换到站立状态
                        break;
                    case 'p':
                        robot_state = STATE_PRONE; // 切换到趴下状态
                        break;
                    case 'a': // 左转
                        p.turn_rate = -0.3f;
                        break;
                    case 'd': // 右转
                        p.turn_rate = 0.3f;
                        break;
                    case 'w': // 直行
                        p.turn_rate = 0.0f;
                        break;
                    default:
                        // 其他字符忽略，或者根据需要处理
                        break;
                }
            }
        }

        // 2. 状态过渡逻辑 (平滑高度调整)
        if(robot_state == STATE_WALK || robot_state == STATE_STAND)
        {
            // 目标: 站立高度
            if(current_base_z < STAND_Z) 
            {
                current_base_z += 2.0f; // 调整起立速度 (0.5f -> 2.0f, 快4倍)
                if(current_base_z > STAND_Z) current_base_z = STAND_Z;
            }
        }
        else // STATE_PRONE
        {
            // 目标: 趴下高度
            if(current_base_z > PRONE_Z)
            {
                current_base_z -= 2.0f; // 调整趴下速度 (0.5f -> 2.0f)
                if(current_base_z < PRONE_Z) current_base_z = PRONE_Z;
            }
        }

        // 3. 计算控制目标
        float time_s = (float)HAL_GetTick() / 1000.0f;
        float t = time_s; // 使用连续时间，让相位在周期内正确累积

        // 如果处于趴下状态 (STATE_PRONE) 且要求原地踏步 (SwC == RC_MI) -> 执行爬行步态
        // 增加左摇杆 (Ch4) 阈值判断：只有当左摇杆向前推 (Ch4 > 200) 时才执行爬行
        // 注意：根据 Remote 定义，Ch4 是左摇杆上下，Ch3 是左摇杆左右。
        // 向前推通常对应 Ch4 > 0 (具体正负取决于遥控器设置，假设 > 200 为前)
        if(robot_state == STATE_PRONE && rc.swC == RC_MI && rc.ch4 > 200)
        {
            TrajectoryParams crawl_p = p;
            crawl_p.start_z = current_base_z; // 使用当前的趴下高度
            // crawl_p.step_length = 50.0f; // 爬行时步长可以小一点
            
            Quadruped_Sequential_Crawl(t, &crawl_p, &geom);
        }
        else if(robot_state == STATE_WALK && current_base_z >= STAND_Z - 5.0f)
        {
            TrajectoryParams temp_p = p;
            temp_p.start_z = current_base_z;
            
            // ---------------------------------------------------------
            // 新版遥控器映射 (右摇杆控制方向) - 函数调用版
            // 右摇杆上下 (Ch2): 上推(< -100) -> 前进 (Quadruped_Forward_Walk)
            //                   下推(> 100) -> 后退 (Quadruped_backward, 实际上是0步长原地)
            // 右摇杆左右 (Ch1): 左推(< -100) -> 原地左旋 (Quadruped_Spin_Left_InPlace)
            //                   右推(> 100) -> 原地右旋 (Quadruped_Spin_Right_InPlace)
            // 原地踏步 (归中):  Quadruped_backward (步长0原地稳态)
            // ---------------------------------------------------------

            int16_t deadzone = 100;
            
            // 1. 优先处理前后运动
            if (rc.ch2 < -deadzone) // 上推 -> 前进
            {
                // 前进需要负步长 (根据之前的逻辑)
                temp_p.step_length = fabsf(p.step_length);
                Quadruped_Forward_Walk(t, &temp_p, &geom);
            }
            else if (rc.ch2 > deadzone) // 下推 -> 后退
            {
                // 调用后退函数 (内部强制步长为0)
                Quadruped_backward(t, &temp_p, &geom);
            }
            // 2. 处理纯转向 (没有前后指令时)
            else if (rc.ch1 < -deadzone) // 左推 -> 原地左旋
            {
                Quadruped_Spin_Left_InPlace(t, &temp_p, &geom);
            }
            else if (rc.ch1 > deadzone) // 右推 -> 原地右旋
            {
                Quadruped_Spin_Right_InPlace(t, &temp_p, &geom);
            }
            // 3. 原地踏步 (摇杆归中)
            else
            {
                // 用户反馈：Quadruped_backward (步长0) 比 Quadruped_InPlace (步长反向) 更稳定
                // 且 Quadruped_backward 实际上就是最好的原地/后退函数
                Quadruped_InPlace(t, &temp_p, &geom);
            }
        }
        else
        {
            float hold_kp = 22.0f;
            float hold_kd = 5.0f;
            if (robot_state == STATE_PRONE)
            {
                // 如果 SwC == RC_MI (准备爬行)，但未推摇杆，则进入“爬行准备姿态”
                // 此时大腿会上抬降低重心，但不会行走
                if (rc.swC == RC_MI) 
                {
                    Quad_Prone_Crawl_Ready(&geom, current_base_z, hold_kp, hold_kd);
                }
                else
                {
                    Quad_Prone(&geom, current_base_z, hold_kp, hold_kd); 
                }
            }
            else
            {
                Quad_Stand(&geom, current_base_z, hold_kp, hold_kd); 
            }
        }

        // 4. 应用偏移量 (Apply Offsets)
        CAN_1.ID_1_Motor_send.position += motor_offsets[1];
        CAN_1.ID_2_Motor_send.position += motor_offsets[2];
        CAN_1.ID_3_Motor_send.position += motor_offsets[3];
        CAN_1.ID_4_Motor_send.position += motor_offsets[4];

        CAN_2.ID_1_Motor_send.position += motor_offsets[5];
        CAN_2.ID_2_Motor_send.position += motor_offsets[6];
        CAN_2.ID_3_Motor_send.position += motor_offsets[7];
        CAN_2.ID_4_Motor_send.position += motor_offsets[8];

        // 2025-01-26 Opt: Restore software optimization (Shortest Path)
        // This ensures the motor always takes the shortest path (<180 deg)
        // regardless of zero_sta setting or offset magnitude.
        
        /* 
         * CRITICAL FIX: 使用 fmodf 替代 while 循环，防止死循环导致电机僵死 
         * 当 diff 非常大时（例如 NaN 或 累积误差），while 循环会导致任务卡死
         * 使用 fmodf 确保 O(1) 时间复杂度完成角度归一化
         */
        #define OPTIMIZE_PATH(target_ptr, current) do { \
             float diff = *(target_ptr) - (current); \
             /* 将 diff 限制在 [-PI, PI] 范围内 */ \
             if (diff > 3.1415926f || diff < -3.1415926f) { \
                 diff = fmodf(diff + 3.1415926f, 6.2831852f); \
                 if (diff < 0) diff += 6.2831852f; \
                 diff -= 3.1415926f; \
                 *(target_ptr) = (current) + diff; \
             } \
        } while(0)

        // Apply optimization to all 8 motors
        OPTIMIZE_PATH(&CAN_1.ID_1_Motor_send.position, CAN_1.ID_1_Motor_recieve.current_position_f);
        OPTIMIZE_PATH(&CAN_1.ID_2_Motor_send.position, CAN_1.ID_2_Motor_recieve.current_position_f);
        OPTIMIZE_PATH(&CAN_1.ID_3_Motor_send.position, CAN_1.ID_3_Motor_recieve.current_position_f);
        OPTIMIZE_PATH(&CAN_1.ID_4_Motor_send.position, CAN_1.ID_4_Motor_recieve.current_position_f);
        
        OPTIMIZE_PATH(&CAN_2.ID_1_Motor_send.position, CAN_2.ID_1_Motor_recieve.current_position_f);
        OPTIMIZE_PATH(&CAN_2.ID_2_Motor_send.position, CAN_2.ID_2_Motor_recieve.current_position_f);
        OPTIMIZE_PATH(&CAN_2.ID_3_Motor_send.position, CAN_2.ID_3_Motor_recieve.current_position_f);
        OPTIMIZE_PATH(&CAN_2.ID_4_Motor_send.position, CAN_2.ID_4_Motor_recieve.current_position_f);
        
        #undef OPTIMIZE_PATH

        // 如果处于软急停(SwD 上)，不再下发控制指令，避免被后台逻辑重新“拉起来”
        if (estop_active)
        {
            osDelay(2);
            continue;
        }

        // 5. 发送控制指令
        // Split into groups to avoid CAN mailbox saturation
        // Total 4 motors per CAN bus. 3 mailboxes.
        // If we send 4 rapidly, the 4th one might fail or block if one of the first 3 is pending.
        
        // CAN 1 Group 1 (ID 1, 2)
        CAN_Send_Control(&hcan1, &CAN_1.ID_1_Motor_send);
        CAN_Send_Control(&hcan1, &CAN_1.ID_2_Motor_send);
        osDelay(1); // Give time for mailboxes to clear

        // CAN 1 Group 2 (ID 3, 4)
        CAN_Send_Control(&hcan1, &CAN_1.ID_3_Motor_send);
        CAN_Send_Control(&hcan1, &CAN_1.ID_4_Motor_send);
        osDelay(1);

        // CAN 2 Group 1 (ID 1, 2) - This was failing
        CAN_Send_Control(&hcan2, &CAN_2.ID_1_Motor_send);
        CAN_Send_Control(&hcan2, &CAN_2.ID_2_Motor_send);
        osDelay(1);

        // CAN 2 Group 2 (ID 3, 4)
        CAN_Send_Control(&hcan2, &CAN_2.ID_3_Motor_send);
        CAN_Send_Control(&hcan2, &CAN_2.ID_4_Motor_send);
        
        // 6. 循环延时
        osDelay(2); // Reduced from 4 to maintain overall loop timing

        // 7. Heartbeat LED (LED_IO3) to indicate task health
        static uint32_t led_tick = 0;
        if (HAL_GetTick() - led_tick > 500) // Toggle every 500ms
        {
            led_tick = HAL_GetTick();
            static uint8_t led3_state = 0;
            led3_state = !led3_state;
            write_led_io(LED_IO3, led3_state ? LED_ON : LED_OFF);
        }
    }
}
