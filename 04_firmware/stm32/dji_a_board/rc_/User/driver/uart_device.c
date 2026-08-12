/****************************************************************************
 *  RoboMentors www.robomentors.com.
 *	wechat:superzz8080
 *	基于RoboMaster二次开发
 ***************************************************************************/
 
#include "uart_device.h"
#include "sys.h"

#include "stdlib.h"
#include "string.h"

/* 解析后的遥控器数据 */
rc_type_t rc;
/* 接收到的遥控器原始数据 */
uint8_t   dbus_recv[DBUS_FRAME_SIZE];

/**
  * @brief     遥控器中断回调函数，在设置 UART 接收时注册
  */
void dbus_uart_callback(void)
{
  remote_data_handle(&rc, dbus_recv);
}

/**
  * @brief     解析遥控器数据
  * @param     rc: 解析后的遥控器数据结构体指针
  * @param     buff: 串口接收到的遥控器原始数据指针
  */
static void remote_data_handle(rc_type_t *rc, uint8_t *buff)
{
  /* SBUS Start Byte Check */
  // if (buff[0] != 0x0F) // DJI SBUS uses 0x00? Standard SBUS uses 0x0F.
  // 暂时移除起始字节检查，防止协议差异导致丢包
  // {
  //   memset(rc, 0, sizeof(rc_type_t));
  //   return;
  // }

  /* Parse SBUS Channels (11 bits per channel) */
  int16_t ch[16];
  ch[0]  = ((buff[1]       | buff[2]<<8)                    & 0x07FF);
  
  /* 
   * CRITICAL FIX: 过滤全0或极低值的噪声数据
   * 正常 SBUS 通道值范围通常在 200 到 1800 之间。
   * 如果接收到 0 或者非常小的值，说明是无效帧（可能是全0噪声）。
   * 直接丢弃该帧，保留上一次的有效 rc 状态。
   */
  if (ch[0] < 100) 
  {
      return;
  }

  ch[1]  = ((buff[2]>>3    | buff[3]<<5)                    & 0x07FF);
  ch[2]  = ((buff[3]>>6    | buff[4]<<2 | buff[5]<<10)      & 0x07FF);
  ch[3]  = ((buff[5]>>1    | buff[6]<<7)                    & 0x07FF);
  ch[4]  = ((buff[6]>>4    | buff[7]<<4)                    & 0x07FF);
  // ch[5]  = ((buff[7]>>7    | buff[8]<<1 | buff[9]<<9)       & 0x07FF);
  ch[6]  = ((buff[9]>>2    | buff[10]<<6)                   & 0x07FF);
  ch[7]  = ((buff[10]>>5   | buff[11]<<3)                   & 0x07FF);
  
  /* Corrected SBUS Parsing for Ch9-Ch12 (Indices 8-11) */
  /* Pattern repeats every 11 bytes (8 channels) */
  /* ch[8] corresponds to ch[0] pattern, shifted by 11 bytes */
  ch[8]  = ((buff[12]      | buff[13]<<8)                   & 0x07FF);
  ch[9]  = ((buff[13]>>3   | buff[14]<<5)                   & 0x07FF);
  // ch[10] = ((buff[14]>>6   | buff[15]<<2 | buff[16]<<10)    & 0x07FF);
  // ch[11] = ((buff[16]>>1   | buff[17]<<7)                   & 0x07FF);

  /* Map Channels to rc structure
     RadioLink AT9S Pro (SBUS) mapping:
     Ch1: Aileron (Right LR) -> rc->ch1
     Ch2: Elevator (Right UD) -> rc->ch2
     Ch3: Throttle (Left UD) -> rc->ch4 (Note: rc->ch4 is Left UD in current struct comments)
     Ch4: Rudder (Left LR) -> rc->ch3 (Note: rc->ch3 is Left LR in current struct comments)
  */
  
  /* Normalize to -660 ~ 660 range (SBUS range approx 200~1800, center 1024) */
  /* Scaling factor: 660 / (1800-1024) approx 0.85 */
  
  rc->ch1 = (ch[0] - SBUS_RC_MID) * 660 / 800;
  rc->ch2 = (ch[1] - SBUS_RC_MID) * 660 / 800;
  rc->ch3 = (ch[3] - SBUS_RC_MID) * 660 / 800; // Map SBUS Ch4 (Rudder) to rc->ch3
  rc->ch4 = (ch[2] - SBUS_RC_MID) * 660 / 800; // Map SBUS Ch3 (Throttle) to rc->ch4

  /* 防止遥控器零点有偏差 */
  if(abs(rc->ch1) < 50) rc->ch1 = 0;
  if(abs(rc->ch2) < 50) rc->ch2 = 0;
  if(abs(rc->ch3) < 50) rc->ch3 = 0;
  if(abs(rc->ch4) < 50) rc->ch4 = 0;

  /* SwC (Ch8) */
  if (ch[7] < 500) rc->swC = RC_DN;
  else if (ch[7] > 1500) rc->swC = RC_UP;
  else rc->swC = RC_MI;

  /* SwA (Ch10) */
  if (ch[9] < 500) rc->swA = RC_DN;
  else if (ch[9] > 1500) rc->swA = RC_UP;
  else rc->swA = RC_MI;
  
  /* SwD (Ch7) */
  if (ch[6] < 500) rc->swD = RC_DN;
  else if (ch[6] > 1500) rc->swD = RC_UP;
  else rc->swD = RC_MI;

  /* SwG (Ch5) */
  if (ch[4] < 500) rc->swG = RC_DN;
  else if (ch[4] > 1500) rc->swG = RC_UP;
  else rc->swG = RC_MI;

  /* 
   * CRITICAL FIX: 如果遥控器还没开，接收到的数据可能是全0
   * 这种情况下，ch[4] = 0 -> rc->sw1 = RC_DN
   * 这会导致系统认为开关处于“向下”状态（趴下模式）。
   * 但如果此时 LED1 快闪，说明确实收到了数据帧。
   * 
   * 问题：为什么快闪（有数据）但没反应？
   * 可能是遥控器通道映射不对，或者开关值反了。
   */

  /* 遥控器异常值处理，函数直接返回 */
  // 暂时放宽限制，防止轻微越界导致丢失控制
  if ((abs(rc->ch1) > 800) || \
      (abs(rc->ch2) > 800) || \
      (abs(rc->ch3) > 800) || \
      (abs(rc->ch4) > 800))
  {
    return ;
  }

  /* Clear Mouse and Keyboard (Not available in SBUS) - Removed
  memset(&rc->mouse, 0, sizeof(rc->mouse));
  memset(&rc->kb, 0, sizeof(rc->kb));
  */
}
