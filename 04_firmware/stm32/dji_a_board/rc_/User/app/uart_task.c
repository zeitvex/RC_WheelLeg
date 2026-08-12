#include "uart_task.h"
#include "uart_device.h"
#include "cmsis_os.h"
#include "rm_hal_lib.h"
#include "lingzu_motor.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

QueueHandle_t led_control_queue;

static void uart8_send_motor_feedback(uint8_t can_bus, uint8_t motor_id, Motor_CAN_Recieve_Struct *motor)
{
    char msg[160];
    uint32_t age_ms = HAL_GetTick() - motor->last_update_time;

    int pos_i = (int)motor->current_position_f;
    int pos_f = (int)(fabs(motor->current_position_f - pos_i) * 10000);
    int spd_i = (int)motor->current_speed_f;
    int spd_f = (int)(fabs(motor->current_speed_f - spd_i) * 10000);
    int tor_i = (int)motor->current_torque_f;
    int tor_f = (int)(fabs(motor->current_torque_f - tor_i) * 10000);
    int tmp_i = (int)motor->current_temp_f;
    int tmp_f = (int)(fabs(motor->current_temp_f - tmp_i) * 10);

    int len = snprintf(msg, sizeof(msg),
                       "CAN%u,ID%u,POS:%s%d.%04d,SPD:%s%d.%04d,TOR:%s%d.%04d,TMP:%s%d.%01d,FAULT:0x%02X,AGE:%lu\r\n",
                       can_bus,
                       motor_id,
                       (motor->current_position_f < 0 && pos_i == 0) ? "-" : "", pos_i, pos_f,
                       (motor->current_speed_f < 0 && spd_i == 0) ? "-" : "", spd_i, spd_f,
                       (motor->current_torque_f < 0 && tor_i == 0) ? "-" : "", tor_i, tor_f,
                       (motor->current_temp_f < 0 && tmp_i == 0) ? "-" : "", tmp_i, tmp_f,
                       motor->fault_message,
                       (unsigned long)age_ms);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart8, (uint8_t *)msg, (uint16_t)len, 20);
    }
}

static void uart8_send_all_motor_feedback(void)
{
    uart8_send_motor_feedback(1, 1, &CAN_1.ID_1_Motor_recieve);
    uart8_send_motor_feedback(1, 2, &CAN_1.ID_2_Motor_recieve);
    uart8_send_motor_feedback(1, 3, &CAN_1.ID_3_Motor_recieve);
    uart8_send_motor_feedback(1, 4, &CAN_1.ID_4_Motor_recieve);
    uart8_send_motor_feedback(2, 1, &CAN_2.ID_1_Motor_recieve);
    uart8_send_motor_feedback(2, 2, &CAN_2.ID_2_Motor_recieve);
    uart8_send_motor_feedback(2, 3, &CAN_2.ID_3_Motor_recieve);
    uart8_send_motor_feedback(2, 4, &CAN_2.ID_4_Motor_recieve);
}

/**
 * 输入: 无
 * 输出: 无
 * 作用: 根据遥控器 SwD 开关状态控制灵足电机使能/失能
 *       约定: SwD 上=软急停(失能全部电机), SwD 下=恢复行走(使能全部电机)
 */
static void RC_SwD_Motor_StartStop_Update(void)
{
    static uint8_t last_swD = 0;

    if (rc.swD == 0)
    {
        return;
    }

    if (rc.swD != last_swD)
        {
            if (rc.swD == RC_UP)
            {
                LINGZU_All_Motors_Limp();   // User/driver/lingzu_motor.c
                DISABLE_ALL_LINGZU_MOTORS(); // User/driver/lingzu_motor.c
            }
            else if (rc.swD == RC_DN)
            {
                ENABLE_ALL_LINGZU_MOTORS(); // User/driver/lingzu_motor.c
            }
            last_swD = rc.swD;
        }
}

void uart_task(const void* argu)
{
    /* 创建队列，深度为10，每个单元大小为char */
    led_control_queue = xQueueCreate(10, sizeof(char));
    
    while(1)
    {
        /* 
         * SwC (rc.sw3) 控制 站立/趴下
         * RC_DN (2): 站立 (Stand) -> 发送 's'
         * RC_UP (1) / RC_MI (3): 趴下 (Prone) -> 发送 'p'
         * 
         * 调试信息：如果 rc.sw3 一直是 0，说明遥控器没连上或没解析到数据。
         * 为了验证是否接收到数据，如果接收到任何非零数据，闪烁 LED1。
         */
        
        // 强制闪烁逻辑：只要进入了 while(1) 循环，LED1 就以 1Hz 闪烁
        // 如果遥控器有数据，则改为快闪
        static int debug_cnt = 0;
        debug_cnt++;
        
        // 只要任一通道有数据，就认为连接正常
        // 简化逻辑：只检查 SwA (十通) 是否有数据变化，或者检查基本通道
        // 这里我们检查 SwA 是否非0 (说明已解析)，或者 ch1 (右摇杆) 是否有值
        // 为了稳健，只要 rc 结构体非全0即可。这里沿用之前的风格，检查 SwA。
        if (rc.swA != 0) {
             // 接收到有效遥控数据
             // 调试逻辑：如果 SwA 处于“下”位 (RC_DN)，则常亮 LED1
             // 这样用户可以测试 SwA 是否对应“站立”指令
             if (rc.swA == RC_DN) {
                 write_led_io(LED_IO1, LED_ON);
             } else {
                 // 否则快闪，表示连接正常但处于趴下模式 (SwA 在上或中)
                 write_led_io(LED_IO1, (debug_cnt % 2) ? LED_ON : LED_OFF);
             }
        } else {
             // 未接收到数据：慢闪
             write_led_io(LED_IO1, (debug_cnt / 25) % 2 ? LED_ON : LED_OFF);
        }

        static char last_sent_cmd = 0;
        char current_cmd = 'p';

        // 逻辑：仅使用 SwA (十通) 控制
        // SwA 下 (RC_DN) -> 站立/行走
        // SwA 上/中 -> 趴下
        if (rc.swA == RC_DN)
        {
            if(rc.ch4 > 200)
            {
                current_cmd = 'q'; // Walk
            }
            else
            {
                current_cmd = 's'; // Stand (Stop walking)
            }
        }
        else
        {
            current_cmd = 'p'; // Prone
        }

        // 仅在状态改变时发送命令，防止队列溢出
        // 添加超时重发机制 (每1秒重发一次) 以防止丢包
        static int resend_cnt = 0;
        resend_cnt++;
        
        if (current_cmd != last_sent_cmd || resend_cnt > 50) 
        {
            xQueueSend(led_control_queue, &current_cmd, 0);
            last_sent_cmd = current_cmd;
            resend_cnt = 0;
        }

        // 根据 SwD 状态控制电机启停
        RC_SwD_Motor_StartStop_Update();

        static uint32_t last_uart8_send_tick = 0;
        uint32_t now_tick = HAL_GetTick();
        if (now_tick - last_uart8_send_tick >= 100)
        {
            last_uart8_send_tick = now_tick;
            uart8_send_all_motor_feedback();
        }
        
        /* 延时 20ms，控制发送频率 */
        osDelay(20);
    }
}
