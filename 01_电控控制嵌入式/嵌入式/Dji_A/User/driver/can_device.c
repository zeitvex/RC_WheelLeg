/****************************************************************************
 *  RoboMentors www.robomentors.com.
 *	wechat:superzz8080
 *	基于RoboMaster二次开发
 ***************************************************************************/

#include "can_device.h"
#include "sys.h"
#include "lingzu_motor.h"
#include <math.h>

#define PI 3.1415926535f

/* 云台电机 */
moto_measure_t moto_pit;
/* 底盘电机 */
moto_measure_t moto_chassis[4];

/**
  * @brief     CAN1 中断回调函数，在程序初始化时注册
  * @param     recv_id: CAN1 接收到的数据 ID
  * @param     data: 接收到的 CAN1 数据指针
  */
void can1_recv_callback(uint32_t recv_id, uint8_t data[])
{
      uint32_t actual_id = recv_id;
      if (hcan1.pRxMsg != NULL && hcan1.pRxMsg->IDE == CAN_ID_EXT) {
          actual_id = hcan1.pRxMsg->ExtId;
      }

      // 协议反馈帧 ID: Bit28~24=0x2, Bit23~8=extra_data, Bit7~0=master_id
      // extra_data: High Byte=Status, Low Byte=Motor ID
      uint32_t cmd_type = (actual_id >> 24) & 0x1F;
      
      if (cmd_type == 0x2) 
      {
          uint32_t feedback_motor_id = (actual_id >> 8) & 0xFF;
          uint32_t feedback_master_id = actual_id & 0xFF;
          uint8_t status_byte = (actual_id >> 16) & 0xFF;

          // if (feedback_master_id == 0xFD) // 2025-01-26 Fix: Relax Master ID check to support mixed configurations
          {
              Motor_Recieve_Single_CAN1.master_id = feedback_master_id;
              Motor_Recieve_Single_CAN1.motor_id = feedback_motor_id;
              
              // 解析 Data (Big Endian)
              // Byte0~1: 角度 (Mapped Symmetric)
              uint16_t pos_int = (data[0] << 8) | data[1]; 
              Motor_Recieve_Single_CAN1.current_position_f = uint_to_float_mit(pos_int, P_MAX, 16);
              
              // Byte2~3: 速度 (Mapped Symmetric)
              uint16_t spd_int = (data[2] << 8) | data[3];
              Motor_Recieve_Single_CAN1.current_speed_f = uint_to_float_mit(spd_int, V_MAX, 16);
              
              // Byte4~5: 力矩 (Mapped Symmetric)
              uint16_t tor_int = (data[4] << 8) | data[5];
              Motor_Recieve_Single_CAN1.current_torque_f = uint_to_float_mit(tor_int, T_MAX, 16);
              
              // Byte6~7: 温度 (Unsigned Short, scaled by 0.1)
              uint16_t temp_int = (data[6] << 8) | data[7];
              Motor_Recieve_Single_CAN1.current_temp_f = (float)temp_int * 0.1f;
              
              // Timestamp
              Motor_Recieve_Single_CAN1.last_update_time = HAL_GetTick();

              // Status from ID
              Motor_Recieve_Single_CAN1.fault_message = status_byte;

              switch (feedback_motor_id)
          {
              case 1: CAN_1.ID_1_Motor_recieve = Motor_Recieve_Single_CAN1; break;
              case 2: CAN_1.ID_2_Motor_recieve = Motor_Recieve_Single_CAN1; break;
              case 3: CAN_1.ID_3_Motor_recieve = Motor_Recieve_Single_CAN1; break;
              case 4: CAN_1.ID_4_Motor_recieve = Motor_Recieve_Single_CAN1; break;
          }
      }
      }

  switch (actual_id)
  {
    case CAN_3508_M1_ID:
    {
      moto_chassis[0].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[0], data) : \
      encoder_data_handle(&moto_chassis[0], data);
    }
    break;
    case CAN_3508_M2_ID:
    {
      moto_chassis[1].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[1], data) : \
      encoder_data_handle(&moto_chassis[1], data);
    }
    break;
    case CAN_3508_M3_ID:
    {
      moto_chassis[2].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[2], data) : \
      encoder_data_handle(&moto_chassis[2], data);
    }
    break;
    case CAN_3508_M4_ID:
    {
      moto_chassis[3].msg_cnt++ <= 50 ? get_moto_offset(&moto_chassis[3], data) : \
      encoder_data_handle(&moto_chassis[3], data);
    }
    break;
    case CAN_PIT_MOTOR_ID:
    {
      encoder_data_handle(&moto_pit, data);
    }
    break;
    default:
    {
    }
    break;
  }
}
  
/**
  * @brief     CAN2 中断回调函数，在程序初始化时注册
  * @param     recv_id: CAN2 接收到的数据 ID
  * @param     data: 接收到的 CAN2 数据指针
  */
void can2_recv_callback(uint32_t recv_id, uint8_t data[])
{
      uint32_t actual_id = recv_id;
      if (hcan2.pRxMsg != NULL && hcan2.pRxMsg->IDE == CAN_ID_EXT) {
          actual_id = hcan2.pRxMsg->ExtId;
      }

      // 协议反馈帧 ID: Bit28~24=0x2, Bit23~8=extra_data, Bit7~0=master_id
      uint32_t cmd_type = (actual_id >> 24) & 0x1F;
      
      if (cmd_type == 0x2) 
      {
          uint32_t feedback_motor_id = (actual_id >> 8) & 0xFF;
          uint32_t feedback_master_id = actual_id & 0xFF;
          uint8_t status_byte = (actual_id >> 16) & 0xFF;

          // if (feedback_master_id == 0xFD) // 2025-01-26 Fix: Relax Master ID check
          {
          Motor_Recieve_Single_CAN2.master_id = feedback_master_id;
          Motor_Recieve_Single_CAN2.motor_id = feedback_motor_id;
          
          // 解析 Data (Big Endian)
          // Byte0~1: 角度 (Mapped Symmetric)
          uint16_t pos_int = (data[0] << 8) | data[1]; 
          Motor_Recieve_Single_CAN2.current_position_f = uint_to_float_mit(pos_int, P_MAX, 16);
          
          // Byte2~3: 速度 (Mapped Symmetric)
          uint16_t spd_int = (data[2] << 8) | data[3];
          Motor_Recieve_Single_CAN2.current_speed_f = uint_to_float_mit(spd_int, V_MAX, 16);
          
          // Byte4~5: 力矩 (Mapped Symmetric)
          uint16_t tor_int = (data[4] << 8) | data[5];
          Motor_Recieve_Single_CAN2.current_torque_f = uint_to_float_mit(tor_int, T_MAX, 16);
          
          // Byte6~7: 温度 (Unsigned Short, scaled by 0.1)
          uint16_t temp_int = (data[6] << 8) | data[7];
          Motor_Recieve_Single_CAN2.current_temp_f = (float)temp_int * 0.1f;
          
          // Timestamp
          Motor_Recieve_Single_CAN2.last_update_time = HAL_GetTick();

          // Status from ID
          Motor_Recieve_Single_CAN2.fault_message = status_byte;

          switch (feedback_motor_id)
          {
              case 1: CAN_2.ID_1_Motor_recieve = Motor_Recieve_Single_CAN2; break;
              case 2: CAN_2.ID_2_Motor_recieve = Motor_Recieve_Single_CAN2; break;
              case 3: CAN_2.ID_3_Motor_recieve = Motor_Recieve_Single_CAN2; break;
              case 4: CAN_2.ID_4_Motor_recieve = Motor_Recieve_Single_CAN2; break;
          }
      }
  }

  switch (actual_id)
  {
//    case CAN_GIMBAL_ZGYRO_ID;
		{
		
		}
//		break;
    
    default:
    {
    }
    break;
  }
}

/**
  * @brief     获得电机初始偏差
  * @param     ptr: 电机参数 moto_measure_t 结构体指针
  * @param     data: 接收到的电机 CAN 数据指针
  */
static void get_moto_offset(moto_measure_t *ptr, uint8_t data[])
{
  ptr->ecd        = (uint16_t)(data[0] << 8 | data[1]);
  ptr->offset_ecd = ptr->ecd;
}

/**
  * @brief     计算电机的转速rmp 圈数round_cnt 
  *            总编码器数值total_ecd 总旋转的角度total_angle
  * @param     ptr: 电机参数 moto_measure_t 结构体指针
  * @param     data: 接收到的电机 CAN 数据指针
  */
static void encoder_data_handle(moto_measure_t *ptr, uint8_t data[])
{
  int32_t temp_sum = 0;
  
  ptr->last_ecd      = ptr->ecd;
  ptr->ecd           = (uint16_t)(data[0] << 8 | data[1]);

  ptr->speed_rpm     = (int16_t)(data[2] << 8 | data[3]);

  if (ptr->ecd - ptr->last_ecd > 5000)
  {
    ptr->round_cnt--;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd - 8192;
  }
  else if (ptr->ecd - ptr->last_ecd < -5000)
  {
    ptr->round_cnt++;
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd + 8192;
  }
  else
  {
    ptr->ecd_raw_rate = ptr->ecd - ptr->last_ecd;
  }

  ptr->total_ecd = ptr->round_cnt * 8192 + ptr->ecd - ptr->offset_ecd;
  ptr->total_angle = ptr->total_ecd * 360 / 8192;
  
  
  ptr->rate_buf[ptr->buf_cut++] = ptr->ecd_raw_rate;
  if (ptr->buf_cut >= FILTER_BUF)
    ptr->buf_cut = 0;
  for (uint8_t i = 0; i < FILTER_BUF; i++)
  {
    temp_sum += ptr->rate_buf[i];
  }
  ptr->filter_rate = (int32_t)(temp_sum/FILTER_BUF);
}

/**
  * @brief     发送底盘电机电流数据到电调
  */
void send_chassis_moto_current(int16_t current[])
{
  static uint8_t data[8];
  
  data[0] = current[0] >> 8;
  data[1] = current[0];
  data[2] = current[1] >> 8;
  data[3] = current[1];
  data[4] = current[2] >> 8;
  data[5] = current[2];
  data[6] = current[3] >> 8;
  data[7] = current[3];
  
  write_can(CHASSIS_CAN, CAN_CHASSIS_ID, data);
}
void send_chassis_moto_zero_current(void)
{
  static uint8_t data[8];
  
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
  
  write_can(CHASSIS_CAN, CAN_CHASSIS_ID, data);
}

/**
  * @brief     发送云台电机电流数据到电调
  */
 int16_t trigger_moto_current;
void send_gimbal_moto_current(int16_t yaw_current, int16_t pit_current)
{
  static uint8_t data[8];
  int16_t trigger_current = trigger_moto_current;
  
  data[0] = -yaw_current >> 8;
  data[1] = -yaw_current;
  data[2] = pit_current >> 8;
  data[3] = pit_current;
  data[4] = trigger_current >> 8;
  data[5] = trigger_current;
  data[6] = 0;
  data[7] = 0;
  
  write_can(GIMBAL_CAN, CAN_GIMBAL_ID, data);
}
void send_gimbal_moto_zero_current(void)
{
  static uint8_t data[8];
  
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
  
  write_can(GIMBAL_CAN, CAN_GIMBAL_ID, data);
}



void set_test_motor_current_left(int16_t test_moto_current[])
{
	static uint8_t data[8];
  
	data[0] = test_moto_current[0] >> 8; 
	data[1] = test_moto_current[0];
	data[2] = test_moto_current[1] >> 8; 
	data[3] = test_moto_current[1];
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
	write_can(CHASSIS_CAN, CAN_3508_M2_ID, data);
}

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static CanRxMsgTypeDef Rx1Message;
static CanRxMsgTypeDef Rx2Message;

void can_manual_init(void)
{
    CAN_FilterConfTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter_st.BankNumber = 14;

    // CAN1
    can_filter_st.FilterNumber = 0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    
    if (hcan1.pRxMsg == NULL) hcan1.pRxMsg = &Rx1Message;
    HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);

    // CAN2
    can_filter_st.FilterNumber = 14;
    can_filter_st.FilterActivation = DISABLE; // Disable first
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    
    can_filter_st.FilterActivation = ENABLE; // Then Enable
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    
    if (hcan2.pRxMsg == NULL) hcan2.pRxMsg = &Rx2Message;
    HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
}
