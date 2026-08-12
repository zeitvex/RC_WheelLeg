/**
 * @file  lingzu_task.c
 * @brief 主控制任务 — 适配直接电机角 IK
 *
 * 改动:
 *   - IK 直接输出无符号电机角 (motor_thigh, motor_shank)
 *   - 偏移量校准: IK(0, PRONE_Z) 应输出 ≈ (0, 0)
 *     offset = encoder_reading - IK_output*sign
 *   - 废除 PostureConfig 相关调用
 *   - Quad_Prone_Crawl_Ready 合并为 Quad_Prone
 */

#include "lingzu_task.h"
#include "lingzu_motor.h"
#include "cmsis_os.h"
#include "rm_hal_lib.h"
#include <math.h>
#include "Movement.h"
#include "gait_config.h"
#include "uart_task.h"
#include "uart_device.h"

/* #define CALIBRATE_ZERO_POINT */

#define MOTOR_COUNT 8
#define PI 3.1415926535f

typedef struct {
    CAN_HandleTypeDef     *hcan;
    Motor_CAN_Send_Struct    *tx;
    Motor_CAN_Recieve_Struct *rx;
    float  sign;         /* -1(左) 或 +1(右) */
    uint8_t is_thigh;    /* 1=大腿, 0=小腿 */
} MotorHandle;

static MotorHandle motors[MOTOR_COUNT];

static void motors_init_handles(void)
{
    motors[0] = (MotorHandle){ &hcan1, &CAN_1.ID_1_Motor_send, &CAN_1.ID_1_Motor_recieve, -1.0f, 1 }; /* FL thigh */
    motors[1] = (MotorHandle){ &hcan1, &CAN_1.ID_2_Motor_send, &CAN_1.ID_2_Motor_recieve, -1.0f, 0 }; /* FL shank */
    motors[2] = (MotorHandle){ &hcan1, &CAN_1.ID_3_Motor_send, &CAN_1.ID_3_Motor_recieve,  1.0f, 1 }; /* FR thigh */
    motors[3] = (MotorHandle){ &hcan1, &CAN_1.ID_4_Motor_send, &CAN_1.ID_4_Motor_recieve,  1.0f, 0 }; /* FR shank */
    motors[4] = (MotorHandle){ &hcan2, &CAN_2.ID_1_Motor_send, &CAN_2.ID_1_Motor_recieve, -1.0f, 1 }; /* RL thigh */
    motors[5] = (MotorHandle){ &hcan2, &CAN_2.ID_2_Motor_send, &CAN_2.ID_2_Motor_recieve, -1.0f, 0 }; /* RL shank */
    motors[6] = (MotorHandle){ &hcan2, &CAN_2.ID_3_Motor_send, &CAN_2.ID_3_Motor_recieve,  1.0f, 1 }; /* RR thigh */
    motors[7] = (MotorHandle){ &hcan2, &CAN_2.ID_4_Motor_send, &CAN_2.ID_4_Motor_recieve,  1.0f, 0 }; /* RR shank */
}

static void motors_enable_all(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        Motor_Enable(motors[i].hcan, motors[i].tx);
        osDelay(2);
    }
}

static void motors_send_control_all(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        CAN_Send_Control(motors[i].hcan, motors[i].tx);
        if ((i & 1) == 1) osDelay(1);
    }
}



static void optimize_path(float *target, float current)
{
    float diff = *target - current;
    if (diff > PI || diff < -PI) {
        diff = fmodf(diff + PI, 2.0f * PI);
        if (diff < 0.0f) diff += 2.0f * PI;
        diff -= PI;
        *target = current + diff;
    }
}

/* ============================================================
 *  主任务
 * ============================================================ */
void lingzu_task(const void* argu)
{
    RobotGeometry geom = DEFAULT_GEOMETRY;

    GaitParams gait = {
        .step_length = 120.0f,
        .step_height = 35.0f,
        .period      = 0.7f,
        .start_z     = STAND_Z,
        .duty_cycle  = 0.5f,
        .turn_rate   = 0.0f
    };

    /* ---- 初始化 ---- */
    osDelay(2000);
    Lingzu_Motor_Init_Structs();
    osDelay(100);
    motors_init_handles();

    motors_enable_all();
    osDelay(50);
    motors_enable_all();
    osDelay(50);

    /* 等待电机上线 */
    uint32_t init_start = HAL_GetTick();
    while (HAL_GetTick() - init_start < 5000) {
        uint32_t now = HAL_GetTick();
        int all_ready = 1;
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (now - motors[i].rx->last_update_time > 200) {
                Motor_Enable(motors[i].hcan, motors[i].tx);
                all_ready = 0;
                osDelay(2);
            }
        }
        if (all_ready) break;
        osDelay(50);
    }

    for (int k = 0; k < 20; k++) {
        motors_send_control_all();
        osDelay(10);
    }

    {
        uint32_t now = HAL_GetTick();
        for (int i = 0; i < MOTOR_COUNT; i++) {
            if (now - motors[i].rx->last_update_time > 500)
                Motor_Enable(motors[i].hcan, motors[i].tx);
        }
    }
    osDelay(100);

#ifdef CALIBRATE_ZERO_POINT
    uint32_t cal_start = HAL_GetTick();
    int zero_done = 0;
    while(1) {
        if (!zero_done && (HAL_GetTick() - cal_start > 5000)) {
            for (int i = 0; i < MOTOR_COUNT; i++)
                Motor_Zore(motors[i].hcan, motors[i].tx);
            zero_done = 1;
        }
        for (int i = 0; i < MOTOR_COUNT; i++) {
            motors[i].tx->torque = 0; motors[i].tx->kp = 0;
            motors[i].tx->kd = 0; motors[i].tx->speed = 0;
            motors[i].tx->position = 0;
        }
        motors_send_control_all();
        osDelay(10);
    }
#endif

    /* ---- 偏移量校准（已废弃） ---- */
    float current_base_z = PRONE_Z;

    {
        /* 不再自动校准偏移量，完全依赖电机真实的物理零点 */
        float vmc_offsets[8] = {0};
        VMC_UpdateEncoderOffsets(vmc_offsets);
    }

    /* ---- 状态机 ---- */
    enum { STATE_PRONE = 0, STATE_STAND = 1, STATE_WALK = 2 } robot_state = STATE_PRONE;

    while(1)
    {
        /* ======= 0. 后台重连 ======= */
        static uint32_t last_check = 0;
        uint32_t now_tick = HAL_GetTick();
        uint8_t estop = (rc.swD == RC_UP);

        if (!estop && (now_tick - last_check > 500))
        {
            last_check = now_tick;

            for (int i = 0; i < MOTOR_COUNT; i++) {
                if (now_tick - motors[i].rx->last_update_time > 500) {
                    Motor_Enable(motors[i].hcan, motors[i].tx);
                }
            }
        }

        /* ======= 1. 队列命令 ======= */
        if (led_control_queue != NULL)
        {
            char cmd;
            if (xQueueReceive(led_control_queue, &cmd, 0) == pdTRUE)
            {
                static uint8_t led_st = 0;
                led_st = !led_st;
                write_led_io(LED_IO2, led_st ? LED_ON : LED_OFF);

                switch(cmd) {
                    case 'q': if (robot_state != STATE_PRONE) robot_state = STATE_WALK; break;
                    case 's': robot_state = STATE_STAND; break;
                    case 'p': robot_state = STATE_PRONE; break;
                    case 'a': gait.turn_rate = -0.3f; break;
                    case 'd': gait.turn_rate =  0.3f; break;
                    case 'w': gait.turn_rate =  0.0f; break;
                    default: break;
                }
            }
        }

        /* ======= 2. 高度过渡 ======= */
        if (robot_state == STATE_WALK || robot_state == STATE_STAND) {
            if (current_base_z < STAND_Z) {
                current_base_z += 2.0f;
                if (current_base_z > STAND_Z) current_base_z = STAND_Z;
            }
        } else {
            if (current_base_z > PRONE_Z) {
                current_base_z -= 2.0f;
                if (current_base_z < PRONE_Z) current_base_z = PRONE_Z;
            }
        }

        /* ======= 3. 运动控制 ======= */
        float time_s = (float)HAL_GetTick() / 1000.0f;

        if (robot_state == STATE_PRONE && rc.swC == RC_MI && rc.ch4 > 200)
        {
            GaitParams crawl_p = gait;
            crawl_p.start_z = current_base_z;
            crawl_p.duty_cycle = 0.25f; /* 匍匐步态应当为真正的 4 节拍爬行, 腾空相占 0.25, 支撑相占 0.75 保证 3 腿支撑 */
            Quadruped_Crawl(time_s, &crawl_p, &geom);
        }
        else if (robot_state == STATE_WALK && current_base_z >= STAND_Z - 5.0f)
        {
            GaitParams temp = gait;
            temp.start_z = current_base_z;
            int16_t deadzone = 100;

            if (rc.ch2 < -deadzone) {
                temp.step_length = fabsf(gait.step_length);
                Quadruped_Forward(time_s, &temp, &geom);
            } else if (rc.ch2 > deadzone) {
                Quadruped_Backward(time_s, &temp, &geom);
            } else if (rc.ch1 < -deadzone) {
                Quadruped_SpinLeft(time_s, &temp, &geom);
            } else if (rc.ch1 > deadzone) {
                Quadruped_SpinRight(time_s, &temp, &geom);
            } else {
                Quadruped_InPlace(time_s, &temp, &geom);
            }
        }
        else
        {
            float hold_kp = 180.0f, hold_kd = 6.0f;
            if (robot_state == STATE_PRONE) {
                Quad_Prone(&geom, current_base_z, hold_kp, hold_kd);
            } else {
                Quad_Stand(&geom, current_base_z, hold_kp, hold_kd);
            }
        }


        /* ======= 5. 最短路径优化 ======= */
        for (int i = 0; i < MOTOR_COUNT; i++) {
            optimize_path(&motors[i].tx->position, motors[i].rx->current_position_f);
        }

        /* ======= 6. 急停检查 ======= */
        if (estop) {
            osDelay(2);
            continue;
        }

        /* ======= 7. 发送控制指令 ======= */
        motors_send_control_all();
        osDelay(2);

        /* ======= 8. 心跳 LED ======= */
        static uint32_t led_tick = 0;
        if (HAL_GetTick() - led_tick > 500) {
            led_tick = HAL_GetTick();
            static uint8_t led3 = 0;
            led3 = !led3;
            write_led_io(LED_IO3, led3 ? LED_ON : LED_OFF);
        }
    }
}
