#ifndef __LINGZU_MOTOR_H__
#define __LINGZU_MOTOR_H__

#include "stm32f4xx_hal.h"
#include "can.h" // For CAN_HandleTypeDef

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -50.0f
#define V_MAX 50.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -60.0f
#define T_MAX 60.0f

#define KP_SOFTSTOP 200.0f
#define KD_SOFTSTOP 3.0f

typedef struct
{
	uint8_t id;
	uint8_t mode;
	uint16_t exdata;
	uint8_t res;

	float position;
	float speed;
	float kp;
	float kd;
	float torque;

	//位置限制
	float max_position;
	float min_position;

} Motor_CAN_Send_Struct;

typedef struct
{
	uint8_t master_id;
	uint8_t motor_id;
	uint8_t fault_message;
	uint8_t motor_state;
	uint8_t mode;

	uint16_t current_position; // [0~65535] (-4π~4π)
	uint16_t current_speed;    // [0~65535] (-15rad/s~15rad/s)
	uint16_t current_torque;   // [0~65535] (-120Nm~120Nm)
	uint16_t current_temp;     // Temp * 10

	float current_position_f;
	float current_speed_f;
	float current_torque_f;
	float current_temp_f;
    
    uint32_t last_update_time; // Timestamp of last valid feedback

} Motor_CAN_Recieve_Struct;

typedef struct
{
	Motor_CAN_Send_Struct ID_1_Motor_send, ID_2_Motor_send, ID_3_Motor_send, ID_4_Motor_send;
	Motor_CAN_Recieve_Struct ID_1_Motor_recieve, ID_2_Motor_recieve, ID_3_Motor_recieve, ID_4_Motor_recieve;
} Can_Bus_Data_Struct;

typedef enum
{
	RS02_MOTOR_1 = 1,
	RS02_MOTOR_2 = 2,
	RS02_MOTOR_3 = 3,
	RS02_MOTOR_4 = 4,
	RS02_MOTOR_5 = 5,
	RS02_MOTOR_6 = 6,
	RS02_MOTOR_7 = 7,
	RS02_MOTOR_8 = 8
} rs02_motor_index_e;

extern Motor_CAN_Recieve_Struct Motor_Recieve_Single_CAN1;
extern Motor_CAN_Recieve_Struct Motor_Recieve_Single_CAN2;
extern Can_Bus_Data_Struct CAN_1, CAN_2;

// Functions
void Lingzu_Motor_Init_Structs(void);
void Motor_Enable(CAN_HandleTypeDef *hcan, Motor_CAN_Send_Struct *Motor_Data);
void Motor_Disable(CAN_HandleTypeDef *hcan, Motor_CAN_Send_Struct *Motor_Data);
void Motor_Zore(CAN_HandleTypeDef *hcan, Motor_CAN_Send_Struct *Motor_Data);
void CAN_Send_Control(CAN_HandleTypeDef *hcan, Motor_CAN_Send_Struct *Motor_Data);
void ENABLE_ALL_LINGZU_MOTORS(void);
void DISABLE_ALL_LINGZU_MOTORS(void);
void ZERO_ALL_LINGZU_MOTORS(void);

void LINGZU_All_Motors_Limp(void);

/**
 * 输入: motor_index(uint8_t), position(float), speed(float), kp(float), kd(float), torque(float)
 * 输出: 无
 * 作用: 设置指定编号RS02电机的目标角度及控制参数
 */
void rs02_set_target_rad(uint8_t motor_index, float position, float speed, float kp, float kd, float torque);

/**
 * 输入: motor_index(uint8_t)
 * 输出: 当前角度(float, rad)
 * 作用: 读取指定编号RS02电机的当前关节角度
 */
float rs02_get_position_rad(uint8_t motor_index);

/**
 * 输入: motor_index(uint8_t), target_position(float)
 * 输出: 角度误差(float, rad)
 * 作用: 计算目标角度与当前角度的误差
 */
float rs02_get_error_rad(uint8_t motor_index, float target_position);

/**
 * 输入: position(float), speed(float), kp(float), kd(float), torque(float)
 * 输出: 无
 * 作用: 控制1号RS02电机目标角度及控制参数
 */
void rs02_m1_control(float position, float speed, float kp, float kd, float torque);
float rs02_m1_get_position(void);
float rs02_m1_get_error(float target_position);

void rs02_m2_control(float position, float speed, float kp, float kd, float torque);
float rs02_m2_get_position(void);
float rs02_m2_get_error(float target_position);

void rs02_m3_control(float position, float speed, float kp, float kd, float torque);
float rs02_m3_get_position(void);
float rs02_m3_get_error(float target_position);

void rs02_m4_control(float position, float speed, float kp, float kd, float torque);
float rs02_m4_get_position(void);
float rs02_m4_get_error(float target_position);

void rs02_m5_control(float position, float speed, float kp, float kd, float torque);
float rs02_m5_get_position(void);
float rs02_m5_get_error(float target_position);

void rs02_m6_control(float position, float speed, float kp, float kd, float torque);
float rs02_m6_get_position(void);
float rs02_m6_get_error(float target_position);

void rs02_m7_control(float position, float speed, float kp, float kd, float torque);
float rs02_m7_get_position(void);
float rs02_m7_get_error(float target_position);

void rs02_m8_control(float position, float speed, float kp, float kd, float torque);
float rs02_m8_get_position(void);
float rs02_m8_get_error(float target_position);

// Utils
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float_mit(int x_int, float limit, int bits);
int float_to_uint_mit(float x, float limit, int bits);
int float_to_uint_mit_param(float x, float limit);

#endif
