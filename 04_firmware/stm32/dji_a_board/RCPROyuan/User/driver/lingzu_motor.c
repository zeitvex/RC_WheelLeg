#include "lingzu_motor.h"
#include "cmsis_os.h"
#include "can.h"
#include <stdio.h> // For potential debug
#include <string.h> // For memset if needed
#include <math.h>

#ifndef isnan
#define isnan(x) ((x) != (x))
#endif

#define PI 3.1415926535f
#define RS02_SAFE_POS_RAD (200.0f * PI / 180.0f)

Can_Bus_Data_Struct CAN_1;
Can_Bus_Data_Struct CAN_2;
Motor_CAN_Recieve_Struct Motor_Recieve_Single_CAN1;
Motor_CAN_Recieve_Struct Motor_Recieve_Single_CAN2;

// Private variables for sending
static CanTxMsgTypeDef txMsg_CAN;

static Motor_CAN_Send_Struct* rs02_get_send_struct(uint8_t motor_index)
{
	switch (motor_index)
	{
		case RS02_MOTOR_1: return &CAN_1.ID_1_Motor_send;
		case RS02_MOTOR_2: return &CAN_1.ID_2_Motor_send;
		case RS02_MOTOR_3: return &CAN_1.ID_3_Motor_send;
		case RS02_MOTOR_4: return &CAN_1.ID_4_Motor_send;
		case RS02_MOTOR_5: return &CAN_2.ID_1_Motor_send;
		case RS02_MOTOR_6: return &CAN_2.ID_2_Motor_send;
		case RS02_MOTOR_7: return &CAN_2.ID_3_Motor_send;
		case RS02_MOTOR_8: return &CAN_2.ID_4_Motor_send;
		default: return NULL;
	}
}

static Motor_CAN_Recieve_Struct* rs02_get_recv_struct(uint8_t motor_index)
{
	switch (motor_index)
	{
		case RS02_MOTOR_1: return &CAN_1.ID_1_Motor_recieve;
		case RS02_MOTOR_2: return &CAN_1.ID_2_Motor_recieve;
		case RS02_MOTOR_3: return &CAN_1.ID_3_Motor_recieve;
		case RS02_MOTOR_4: return &CAN_1.ID_4_Motor_recieve;
		case RS02_MOTOR_5: return &CAN_2.ID_1_Motor_recieve;
		case RS02_MOTOR_6: return &CAN_2.ID_2_Motor_recieve;
		case RS02_MOTOR_7: return &CAN_2.ID_3_Motor_recieve;
		case RS02_MOTOR_8: return &CAN_2.ID_4_Motor_recieve;
		default: return NULL;
	}
}

void rs02_set_target_rad(uint8_t motor_index, float position, float speed, float kp, float kd, float torque)
{
	Motor_CAN_Send_Struct *send_struct = rs02_get_send_struct(motor_index);
	if (send_struct == NULL)
	{
		return;
	}

	// motor_index 取值范围为 1~8，对应 1~8 号关节电机
	if (motor_index == 0 || motor_index > 8)
	{
		return;
	}

	// 对目标位置做绝对角度限幅，防止指令过大导致潜在疯转
	float p = position;
	if (p > RS02_SAFE_POS_RAD)
	{
		p = RS02_SAFE_POS_RAD;
	}
	else if (p < -RS02_SAFE_POS_RAD)
	{
		p = -RS02_SAFE_POS_RAD;
	}

	send_struct->position = p;
	send_struct->speed = speed;
	send_struct->kp = kp;
	send_struct->kd = kd;
	send_struct->torque = torque;
}

float rs02_get_position_rad(uint8_t motor_index)
{
	Motor_CAN_Recieve_Struct *recv_struct = rs02_get_recv_struct(motor_index);
	if (recv_struct == NULL)
	{
		return 0.0f;
	}
	return recv_struct->current_position_f;
}

float rs02_get_velocity_rad(uint8_t motor_index)
{
	Motor_CAN_Recieve_Struct *recv_struct = rs02_get_recv_struct(motor_index);
	if (recv_struct == NULL)
	{
		return 0.0f;
	}
	return recv_struct->current_speed_f;
}

float rs02_get_error_rad(uint8_t motor_index, float target_position)
{
	float current = rs02_get_position_rad(motor_index);
	return target_position - current;
}

/* Per-motor wrapper functions removed in refactoring.
 * Use rs02_set_target_rad(motor_index, ...) directly. */

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x = x_max;
    else if(x < x_min) x = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

// 适配 Python 驱动的映射算法: int(((x / limit) + 1.0) * 32767.0)
// x_max 对应 limit. x_min 对应 -limit.
int float_to_uint_mit(float x, float limit, int bits)
{
    float max = limit;
    float min = -limit;
    if(x > max) x = max;
    else if(x < min) x = min;
    
    // Python Logic: ((x / limit) + 1.0) * 32767.0
    // Maps [-limit, limit] to [0, 65534] approx
    // Using 32767.0 as scale factor
    return (int)(((x / limit) + 1.0f) * 32767.0f);
}

// Kp/Kd 映射: int((val / limit) * 65535.0)
// Maps [0, limit] to [0, 65535]
int float_to_uint_mit_param(float x, float limit)
{
    if (x > limit) x = limit;
    if (x < 0) x = 0;
    return (int)((x / limit) * 65535.0f);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

// 适配 Python 驱动的反向映射 (仅用于参考，接收时可能用不上，接收用 uint_to_float 即可)
// 实际上 Python 接收解析是 struct.unpack，得到的是 uint16。
// 并没有展示 uint -> float 的逻辑，但通常是对称的。
// 假设接收也是同样的逻辑，或者直接用 float_to_uint 的逆运算。
// 但根据 constants.py, Python 并没有做 float conversion regarding feedback except printing?
// robstride_driver.py line 295: unpacks to u16.
// line 298: p_limit etc.
// The code cuts off at line 301. It presumably converts u16 back to float.
// Let's assume symmetric mapping.
float uint_to_float_mit(int x_int, float limit, int bits)
{
    // y = ((x/L) + 1) * 32767
    // y/32767 = x/L + 1
    // x/L = y/32767 - 1
    // x = (y/32767 - 1) * L
    return ((float)x_int / 32767.0f - 1.0f) * limit;
}

// Kp/Kd/Torque(Received?)
// Feedback T is signed int16? Python unpacks >HHHH, but names it t_i16?
// If unpacks as H, it is u16.
// If the feedback torque is indeed signed int16 in the motor firmware, struct.unpack should use 'h'.
// If Python uses 'H', it treats it as unsigned.
// Let's assume standard mapping for now.

// 通讯类型 3: 电机使能运行
void Motor_Enable(CAN_HandleTypeDef *hcan, Motor_CAN_Send_Struct *Motor_Data)
{
    hcan->pTxMsg = &txMsg_CAN;

    hcan->pTxMsg->StdId = 0;
    // bit28~24: 0x3 (通讯类型)
    // bit23~8: bit15~8 用标识主CAN_ID (0xFD) -> Extra Data
    // bit7~0: 目标电机CAN_ID
    
    // Python: _send_command(ENABLE, host_id, motor.id)
    // Extra Data = host_id
    // Device ID = motor.id
    
    uint32_t cmd_type = 0x3;
    uint32_t host_id = 0xFD; // Master ID
    uint32_t motor_id = Motor_Data->id;

    hcan->pTxMsg->ExtId = (cmd_type << 24) | (host_id << 8) | motor_id;
    hcan->pTxMsg->IDE = CAN_ID_EXT;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->DLC = 8;

    for(int i=0; i<8; i++)
    {
        hcan->pTxMsg->Data[i] = 0;
    }
    
    // 2025-01-26 Mod: Set zero_sta = 1 (Data[0])
    // 0: 0-2PI (Default, may take long path)
    // 1: -PI~PI (Shortest path/优弧)
    hcan->pTxMsg->Data[0] = 1; 

    // Retry mechanism for reliable transmission
    uint8_t retry = 0;
    while(HAL_CAN_Transmit(hcan, 2) != HAL_OK && retry < 5)
    {
        retry++;
        osDelay(1);
    }
}

// 通讯类型 4: 电机停止运行
void Motor_Disable(CAN_HandleTypeDef *hcan, Motor_CAN_Send_Struct *Motor_Data)
{
    hcan->pTxMsg = &txMsg_CAN;

    hcan->pTxMsg->StdId = 0;
    // bit28~24: 0x4 (通讯类型)
    // bit23~8: Extra Data (Host ID)
    // bit7~0: 目标电机CAN_ID
    uint32_t cmd_type = 0x4;
    uint32_t host_id = 0xFD;
    uint32_t motor_id = Motor_Data->id;

    hcan->pTxMsg->ExtId = (cmd_type << 24) | (host_id << 8) | motor_id;
    hcan->pTxMsg->IDE = CAN_ID_EXT;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->DLC = 8;

    for(int i=0; i<8; i++)
    {
        hcan->pTxMsg->Data[i] = 0;
    }
    // 正常停止时Data清0
    
    uint8_t retry = 0;
    while(HAL_CAN_Transmit(hcan, 2) != HAL_OK && retry < 5)
    {
        retry++;
        osDelay(1);
    }
}

// 通讯类型 6: 设置电机机械零位
// 注意: 此指令仅将当前位置重置为0，不会修改 zero_sta 标志位。
// 若需修改位置范围(0~2PI 或 -PI~PI)，请使用上位机修改 zero_sta 并保存。
void Motor_Zore(CAN_HandleTypeDef *hcan, Motor_CAN_Send_Struct *Motor_Data)
{
    hcan->pTxMsg = &txMsg_CAN;

    hcan->pTxMsg->StdId = 0;
    // bit28~24: 0x6 (通讯类型)
    // bit23~8: Extra Data (Host ID)
    // bit7~0: 目标电机CAN_ID
    uint32_t cmd_type = 0x6;
    uint32_t host_id = 0xFD;
    uint32_t motor_id = Motor_Data->id;

    hcan->pTxMsg->ExtId = (cmd_type << 24) | (host_id << 8) | motor_id;
    hcan->pTxMsg->IDE = CAN_ID_EXT;
    hcan->pTxMsg->RTR = CAN_RTR_DATA;
    hcan->pTxMsg->DLC = 8;

    for(int i=0; i<8; i++)
    {
        hcan->pTxMsg->Data[i] = 0;
    }
    hcan->pTxMsg->Data[0] = 1; // 触发设置零点动作 (并非设置 zero_sta=1)

    uint8_t retry = 0;
    while(HAL_CAN_Transmit(hcan, 2) != HAL_OK && retry < 5)
    {
        retry++;
        osDelay(1);
    }
}

// 通讯类型 1: 遥控模式电机控制指令
void CAN_Send_Control(CAN_HandleTypeDef *hcan, Motor_CAN_Send_Struct *Motor_Data)
{
    // Safety check: Do not send if position is NaN
    if (isnan(Motor_Data->position))
    {
        return;
    }

    CanTxMsgTypeDef txMsg_Control;
    hcan->pTxMsg = &txMsg_Control;

    txMsg_Control.StdId = 0;
    txMsg_Control.IDE = CAN_ID_EXT;
    txMsg_Control.RTR = CAN_RTR_DATA;
    txMsg_Control.DLC = 8;

    // bit28~24: 0x1 (指令)
    // bit23~8: Extra Data (Torque)
    // Bit7~0: 目标电机CAN_ID
    
    uint32_t cmd_type = 0x1;
    
    // ExtId力矩 (Mapped T_MIN ~ T_MAX -> 0~65535)
    // Python Logic: t_u16 = int(((torque / t_limit) + 1.0) * 32767.0)
    uint16_t torque_ext_id = float_to_uint_mit(Motor_Data->torque, T_MAX, 16); // T_MAX corresponds to limit
    
    txMsg_Control.ExtId = (cmd_type << 24) | (torque_ext_id << 8) | Motor_Data->id;

    // Data Structure: Big Endian [Pos, Vel, Kp, Kd]
    
    // Byte0~1: 角度 (Mapped)
    // Python Logic: p_u16 = int(((position / p_limit) + 1.0) * 32767.0)
    uint16_t pos_int = float_to_uint_mit(Motor_Data->position, P_MAX, 16);
    txMsg_Control.Data[0] = pos_int >> 8;   // 高字节在前 (Big Endian)
    txMsg_Control.Data[1] = pos_int & 0xFF; // 低字节在后

    // Byte2~3: 速度 (Mapped)
    // Python Logic: v_u16 = int(((velocity / v_limit) + 1.0) * 32767.0)
    uint16_t spd_int = float_to_uint_mit(Motor_Data->speed, V_MAX, 16);
    txMsg_Control.Data[2] = spd_int >> 8;
    txMsg_Control.Data[3] = spd_int & 0xFF;

    // Byte4~5: Kp (Mapped 0~Limit -> 0~65535)
    uint16_t kp_int = float_to_uint_mit_param(Motor_Data->kp, KP_MAX);
    txMsg_Control.Data[4] = kp_int >> 8;
    txMsg_Control.Data[5] = kp_int & 0xFF;

    // Byte6~7: Kd (Mapped 0~Limit -> 0~65535)
    uint16_t kd_int = float_to_uint_mit_param(Motor_Data->kd, KD_MAX);
    txMsg_Control.Data[6] = kd_int >> 8;
    txMsg_Control.Data[7] = kd_int & 0xFF;

    // 2025-01-26 Fix: Add retry logic for CAN Mailbox Full
    // If HAL_CAN_Transmit returns HAL_BUSY, we should retry.
    uint8_t retry = 0;
    HAL_StatusTypeDef status;
    
    // Explicitly check if mailbox is full (for old HAL lib, Transmit is blocking if timeout != 0)
    // But if timeout occurs, it returns HAL_TIMEOUT.
    // If all mailboxes are busy, it waits until timeout.
    // We should try a few times.
    
    do {
        status = HAL_CAN_Transmit(hcan, 2); // 2ms timeout per attempt
        if (status == HAL_OK) break;
        
        retry++;
        // If we are stuck in a busy loop, maybe we should yield or delay slightly more?
        // But we want to send ASAP.
    } while (retry < 5);
}

void Lingzu_Motor_Init_Structs(void)
{
    // Initialize CAN 1 Motor Structs
	CAN_1.ID_1_Motor_send.id=1;
	CAN_1.ID_1_Motor_send.res=0x04;
	CAN_1.ID_1_Motor_send.max_position=2;
	CAN_1.ID_1_Motor_send.min_position=-2;

	CAN_1.ID_2_Motor_send.id=2;
	CAN_1.ID_2_Motor_send.res=0x04;
	CAN_1.ID_2_Motor_send.max_position=2;
	CAN_1.ID_2_Motor_send.min_position=-2;

	CAN_1.ID_3_Motor_send.id=3;
	CAN_1.ID_3_Motor_send.res=0x04;
	CAN_1.ID_3_Motor_send.max_position=2;
	CAN_1.ID_3_Motor_send.min_position=-2;

	CAN_1.ID_4_Motor_send.id=4;
	CAN_1.ID_4_Motor_send.res=0x04;

    // Initialize CAN 2 Motor Structs
	CAN_2.ID_1_Motor_send.id=1;
	CAN_2.ID_1_Motor_send.res=0x04;
	CAN_2.ID_1_Motor_send.max_position=2;
	CAN_2.ID_1_Motor_send.min_position=-2;

	CAN_2.ID_2_Motor_send.id=2;
	CAN_2.ID_2_Motor_send.res=0x04;
	CAN_2.ID_2_Motor_send.max_position=2;
	CAN_2.ID_2_Motor_send.min_position=-2;

	CAN_2.ID_3_Motor_send.id=3;
	CAN_2.ID_3_Motor_send.res=0x04;
	CAN_2.ID_3_Motor_send.max_position=2;
	CAN_2.ID_3_Motor_send.min_position=-2;

	CAN_2.ID_4_Motor_send.id=4;
	CAN_2.ID_4_Motor_send.res=0x04;
	CAN_2.ID_4_Motor_send.max_position=2;
	CAN_2.ID_4_Motor_send.min_position=-2;
}

void ENABLE_ALL_LINGZU_MOTORS(void)
{
	Motor_Enable(&hcan1, &CAN_1.ID_1_Motor_send);
	osDelay(2);
	Motor_Enable(&hcan2, &CAN_2.ID_1_Motor_send);
	osDelay(2);

	Motor_Enable(&hcan1, &CAN_1.ID_2_Motor_send);
	osDelay(2);
	Motor_Enable(&hcan2, &CAN_2.ID_2_Motor_send);
	osDelay(2);

	Motor_Enable(&hcan1, &CAN_1.ID_3_Motor_send);
	osDelay(2);
	Motor_Enable(&hcan2, &CAN_2.ID_3_Motor_send);
	osDelay(2);

	Motor_Enable(&hcan1, &CAN_1.ID_4_Motor_send);
	osDelay(2);
	Motor_Enable(&hcan2, &CAN_2.ID_4_Motor_send);
	osDelay(2);
}

void DISABLE_ALL_LINGZU_MOTORS(void)
{
	Motor_Disable(&hcan1, &CAN_1.ID_1_Motor_send);
	osDelay(1);
	Motor_Disable(&hcan2, &CAN_2.ID_1_Motor_send);
	osDelay(1);

	Motor_Disable(&hcan1, &CAN_1.ID_2_Motor_send);
	osDelay(1);
	Motor_Disable(&hcan2, &CAN_2.ID_2_Motor_send);
	osDelay(1);

	Motor_Disable(&hcan1, &CAN_1.ID_3_Motor_send);
	osDelay(1);
	Motor_Disable(&hcan2, &CAN_2.ID_3_Motor_send);
	osDelay(1);

	Motor_Disable(&hcan1, &CAN_1.ID_4_Motor_send);
	osDelay(1);
	Motor_Disable(&hcan2, &CAN_2.ID_4_Motor_send);
	osDelay(1);
}

void ZERO_ALL_LINGZU_MOTORS(void)
{
	Motor_Zore(&hcan1, &CAN_1.ID_1_Motor_send);
	osDelay(1);
	Motor_Zore(&hcan2, &CAN_2.ID_1_Motor_send);
	osDelay(1);

	Motor_Zore(&hcan1, &CAN_1.ID_2_Motor_send);
	osDelay(1);
	Motor_Zore(&hcan2, &CAN_2.ID_2_Motor_send);
	osDelay(1);

	Motor_Zore(&hcan1, &CAN_1.ID_3_Motor_send);
	osDelay(1);
	Motor_Zore(&hcan2, &CAN_2.ID_3_Motor_send);
	osDelay(1);

	Motor_Zore(&hcan1, &CAN_1.ID_4_Motor_send);
	osDelay(1);
	Motor_Zore(&hcan2, &CAN_2.ID_4_Motor_send);
	osDelay(1);
}

void LINGZU_All_Motors_Limp(void)
{
    CAN_1.ID_1_Motor_send.torque = 0; CAN_1.ID_1_Motor_send.kp = 0; CAN_1.ID_1_Motor_send.kd = 0; CAN_1.ID_1_Motor_send.speed = 0;
    CAN_1.ID_2_Motor_send.torque = 0; CAN_1.ID_2_Motor_send.kp = 0; CAN_1.ID_2_Motor_send.kd = 0; CAN_1.ID_2_Motor_send.speed = 0;
    CAN_1.ID_3_Motor_send.torque = 0; CAN_1.ID_3_Motor_send.kp = 0; CAN_1.ID_3_Motor_send.kd = 0; CAN_1.ID_3_Motor_send.speed = 0;
    CAN_1.ID_4_Motor_send.torque = 0; CAN_1.ID_4_Motor_send.kp = 0; CAN_1.ID_4_Motor_send.kd = 0; CAN_1.ID_4_Motor_send.speed = 0;

    CAN_2.ID_1_Motor_send.torque = 0; CAN_2.ID_1_Motor_send.kp = 0; CAN_2.ID_1_Motor_send.kd = 0; CAN_2.ID_1_Motor_send.speed = 0;
    CAN_2.ID_2_Motor_send.torque = 0; CAN_2.ID_2_Motor_send.kp = 0; CAN_2.ID_2_Motor_send.kd = 0; CAN_2.ID_2_Motor_send.speed = 0;
    CAN_2.ID_3_Motor_send.torque = 0; CAN_2.ID_3_Motor_send.kp = 0; CAN_2.ID_3_Motor_send.kd = 0; CAN_2.ID_3_Motor_send.speed = 0;
    CAN_2.ID_4_Motor_send.torque = 0; CAN_2.ID_4_Motor_send.kp = 0; CAN_2.ID_4_Motor_send.kd = 0; CAN_2.ID_4_Motor_send.speed = 0;

    CAN_Send_Control(&hcan1, &CAN_1.ID_1_Motor_send);
    CAN_Send_Control(&hcan1, &CAN_1.ID_2_Motor_send);
    CAN_Send_Control(&hcan1, &CAN_1.ID_3_Motor_send);
    CAN_Send_Control(&hcan1, &CAN_1.ID_4_Motor_send);

    CAN_Send_Control(&hcan2, &CAN_2.ID_1_Motor_send);
    CAN_Send_Control(&hcan2, &CAN_2.ID_2_Motor_send);
    CAN_Send_Control(&hcan2, &CAN_2.ID_3_Motor_send);
    CAN_Send_Control(&hcan2, &CAN_2.ID_4_Motor_send);
}

/* ============================================================
 *  CAN 接收回调 (原 can_device.c, 合并至此)
 * ============================================================ */

/* 速度低通滤波系数: 新值权重 0.25, 保留 75% 历史
 * 等效时间常数 ≈ 3 × 控制周期 (2ms) = 6ms
 * 足以抑制 CAN 量化噪声, 同时对步态响应无明显滞后 */
#define SPEED_LPF_ALPHA 0.25f

static void parse_motor_feedback(Motor_CAN_Recieve_Struct *out,
                                  uint32_t actual_id, uint8_t data[])
{
    uint32_t motor_id  = (actual_id >> 8) & 0xFF;
    uint32_t master_id = actual_id & 0xFF;
    uint8_t  status    = (actual_id >> 16) & 0xFF;

    out->master_id = master_id;
    out->motor_id  = motor_id;

    uint16_t pos_int = (data[0] << 8) | data[1];
    out->current_position_f = uint_to_float_mit(pos_int, P_MAX, 16);

    /* 速度: 一阶低通滤波, 抑制量化噪声 */
    uint16_t spd_int = (data[2] << 8) | data[3];
    float new_speed = uint_to_float_mit(spd_int, V_MAX, 16);
    out->current_speed_f = SPEED_LPF_ALPHA * new_speed
                         + (1.0f - SPEED_LPF_ALPHA) * out->current_speed_f;

    uint16_t tor_int = (data[4] << 8) | data[5];
    out->current_torque_f = uint_to_float_mit(tor_int, T_MAX, 16);

    uint16_t temp_int = (data[6] << 8) | data[7];
    out->current_temp_f = (float)temp_int * 0.1f;

    out->last_update_time = HAL_GetTick();
    out->fault_message = status;
}

void can1_recv_callback(uint32_t recv_id, uint8_t data[])
{
    uint32_t actual_id = recv_id;
    if (hcan1.pRxMsg != NULL && hcan1.pRxMsg->IDE == CAN_ID_EXT)
        actual_id = hcan1.pRxMsg->ExtId;

    if (((actual_id >> 24) & 0x1F) != 0x2) return;

    parse_motor_feedback(&Motor_Recieve_Single_CAN1, actual_id, data);

    switch (Motor_Recieve_Single_CAN1.motor_id) {
        case 1: CAN_1.ID_1_Motor_recieve = Motor_Recieve_Single_CAN1; break;
        case 2: CAN_1.ID_2_Motor_recieve = Motor_Recieve_Single_CAN1; break;
        case 3: CAN_1.ID_3_Motor_recieve = Motor_Recieve_Single_CAN1; break;
        case 4: CAN_1.ID_4_Motor_recieve = Motor_Recieve_Single_CAN1; break;
    }
}

void can2_recv_callback(uint32_t recv_id, uint8_t data[])
{
    uint32_t actual_id = recv_id;
    if (hcan2.pRxMsg != NULL && hcan2.pRxMsg->IDE == CAN_ID_EXT)
        actual_id = hcan2.pRxMsg->ExtId;

    if (((actual_id >> 24) & 0x1F) != 0x2) return;

    parse_motor_feedback(&Motor_Recieve_Single_CAN2, actual_id, data);

    switch (Motor_Recieve_Single_CAN2.motor_id) {
        case 1: CAN_2.ID_1_Motor_recieve = Motor_Recieve_Single_CAN2; break;
        case 2: CAN_2.ID_2_Motor_recieve = Motor_Recieve_Single_CAN2; break;
        case 3: CAN_2.ID_3_Motor_recieve = Motor_Recieve_Single_CAN2; break;
        case 4: CAN_2.ID_4_Motor_recieve = Motor_Recieve_Single_CAN2; break;
    }
}

/* CAN 总线滤波器初始化 */
static CanRxMsgTypeDef Rx1Message;
static CanRxMsgTypeDef Rx2Message;

void can_manual_init(void)
{
    CAN_FilterConfTypeDef f;
    f.FilterActivation = ENABLE;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh = 0; f.FilterIdLow = 0;
    f.FilterMaskIdHigh = 0; f.FilterMaskIdLow = 0;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.BankNumber = 14;

    f.FilterNumber = 0;
    HAL_CAN_ConfigFilter(&hcan1, &f);
    if (hcan1.pRxMsg == NULL) hcan1.pRxMsg = &Rx1Message;
    HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);

    f.FilterNumber = 14;
    f.FilterActivation = DISABLE;
    HAL_CAN_ConfigFilter(&hcan2, &f);
    f.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&hcan2, &f);
    if (hcan2.pRxMsg == NULL) hcan2.pRxMsg = &Rx2Message;
    HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
}
