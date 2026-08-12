#include "main.h"
#include "can.h"
#include "app_can.h"
motor_measure_t m3508[8];
motor_measure_t m2006[8];
static CAN_TxHeaderTypeDef  can1_tx_message;
static uint8_t              can1_send_data[8];
static CAN_TxHeaderTypeDef  can2_tx_message;
static uint8_t              can2_send_data[8];

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    can_filter_st.FilterBank = 0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收3508电机通过CAN发过来的信息
  * @Param
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(motor_measure_t *ptr, uint8_t Data[])
{
	  ptr->last_angle = ptr->angle;
    ptr->angle = (uint16_t)(Data[0] << 8 | Data[1]) ;
    ptr->speed  = (int16_t)(Data[2] << 8 | Data[3]);
    ptr->troque = (Data[4] << 8 | Data[5]);
    ptr->temperate = Data[6];

    if(ptr->angle - ptr->last_angle > 4096)
        ptr->round_cnt --;
    else if (ptr->angle - ptr->last_angle < -4096) 
        ptr->round_cnt ++;

    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
    ptr->real_angle = ptr->total_angle / 22.75555556f;
}

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx1_header;
    CAN_RxHeaderTypeDef rx2_header;
    uint8_t rx1_data[8];
    uint8_t rx2_data[8];
    /*	if(hcan==(&hcan1))另一种方式*/
    if(hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx1_header, rx1_data);
			      switch (rx1_header.StdId)//3508接收
            {
                case 0x201:
                case 0x202:
                case 0x203:
                case 0x204:
                case 0x205:
                case 0x206:
                case 0x207:
                case 0x208:
                {
                    static u8 i = 0;
                    static u8 offset_sign[8] = {0}; //八个电机上电角度标志位
                    //get motor id
                    i = rx1_header.StdId - 0x201;
                    get_moto_measure(&m3508[i], rx1_data);//获取电机信息

                    if(offset_sign[i] == 0)  //只记一次上电角度
                    {
                        get_moto_offset(&m3508[i]);//获取上电角度
                        offset_sign[i]++;
                    }
                }
                default:
                {
                    break;
                }
            }
        }
    if(hcan->Instance == CAN2)
    {
        HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx2_header, rx2_data);
        switch (rx2_header.StdId)
        {
            case 0x201:
            case 0x202:
            case 0x203:
            case 0x204:
            case 0x205:
            case 0x206:
            case 0x207:
            case 0x208:
            {
                static u8 i = 0;
                static u8 offset_sign[8] = {0}; //八个电机上电角度标志位
                //get motor id
                i = rx2_header.StdId - 0x201;
                get_moto_measure(&m2006[i],rx2_data);//获取电机信息

                if(offset_sign[i] == 0)  //只记一次上电角度
                {
                    get_moto_offset(&m2006[i]);//获取上电角度
                    offset_sign[i]++;
                }
                break;
            }
            default:
            {
                break;
            }
        }
    }
}
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void can1_cmd1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can1_tx_message.StdId = 0x200;
    can1_tx_message.IDE = CAN_ID_STD;
    can1_tx_message.RTR = CAN_RTR_DATA;
    can1_tx_message.DLC = 0x08;
    can1_send_data[0] = motor1 >> 8;
    can1_send_data[1] = motor1;
    can1_send_data[2] = motor2 >> 8;
    can1_send_data[3] = motor2;
    can1_send_data[4] = motor3 >> 8;
    can1_send_data[5] = motor3;
    can1_send_data[6] = motor4 >> 8;
    can1_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &can1_tx_message, can1_send_data, &send_mail_box);
}
void can1_cmd2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can1_tx_message.StdId = 0x1FF;
    can1_tx_message.IDE = CAN_ID_STD;
    can1_tx_message.RTR = CAN_RTR_DATA;
    can1_tx_message.DLC = 0x08;
    can1_send_data[0] = motor1 >> 8;
    can1_send_data[1] = motor1;
    can1_send_data[2] = motor2 >> 8;
    can1_send_data[3] = motor2;
    can1_send_data[4] = motor3 >> 8;
    can1_send_data[5] = motor3;
    can1_send_data[6] = motor4 >> 8;
    can1_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &can1_tx_message, can1_send_data, &send_mail_box);
}

/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 2006电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 2006电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 2006电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 2006电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void can2_cmd1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can2_tx_message.StdId = 0x200;
    can2_tx_message.IDE = CAN_ID_STD;
    can2_tx_message.RTR = CAN_RTR_DATA;
    can2_tx_message.DLC = 0x08;
    can2_send_data[0] = (motor1 >> 8);
    can2_send_data[1] = motor1;
    can2_send_data[2] = (motor2 >> 8);
    can2_send_data[3] = motor2;
    can2_send_data[4] = (motor3 >> 8);
    can2_send_data[5] = motor3;
    can2_send_data[6] = (motor4 >> 8);
    can2_send_data[7] = motor4;
    HAL_CAN_AddTxMessage(&hcan2, &can2_tx_message, can2_send_data, &send_mail_box);
}
void can2_cmd2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can2_tx_message.StdId = 0x1FF;
    can2_tx_message.IDE = CAN_ID_STD;
    can2_tx_message.RTR = CAN_RTR_DATA;
    can2_tx_message.DLC = 0x08;
    can2_send_data[0] = (motor1 >> 8);
    can2_send_data[1] = motor1;
    can2_send_data[2] = (motor2 >> 8);
    can2_send_data[3] = motor2;
    can2_send_data[4] = (motor3 >> 8);
    can2_send_data[5] = motor3;
    can2_send_data[6] = (motor4 >> 8);
    can2_send_data[7] = motor4;
    HAL_CAN_AddTxMessage(&hcan2, &can2_tx_message, can2_send_data, &send_mail_box);
}
/*this function should be called after system+can init */
void get_moto_offset(motor_measure_t *ptr)
{
    ptr->offset_angle = ptr->angle;
    ptr->round_cnt = 0;//防止角度差8192即一圈
}
