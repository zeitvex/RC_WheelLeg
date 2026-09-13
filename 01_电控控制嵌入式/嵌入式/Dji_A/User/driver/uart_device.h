/****************************************************************************
 *  RoboMentors www.robomentors.com.
 *	wechat:superzz8080
 *	基于RoboMaster二次开发
 ***************************************************************************/
 
#ifndef __UART_DEVICE_H__
#define __UART_DEVICE_H__

#include "rm_hal_lib.h"

#define SBUS_RC_MIN 200
#define SBUS_RC_MID 1024
#define SBUS_RC_MAX 1800

/**
  * @brief     解析后的遥控器数据结构体
  */
typedef struct 
{
  /* 遥控器的通道数据，数值范围：-660 ~ 660 */
  int16_t ch1;   //右侧左右
  int16_t ch2;   //右侧上下
  int16_t ch3;   //左侧左右
  int16_t ch4;   //左侧上下
  
  /* 遥控器的拨杆数据，上中下分别为：1、3、2 */
  uint8_t sw1;   //SwF (Ch5) - Left Switch
  uint8_t sw2;   //SwG? (Ch6) - Right Switch (Undefined in doc)
  uint8_t sw3;   //SwC (Ch7)
  uint8_t sw4;   //VrB/SwD? (Ch8) - Knob/Switch
  
  /* 新增辅助通道映射 */
  uint8_t swA;   //SwA (Ch10)
  uint8_t swB;   //SwB (Ch9)
  uint8_t swC;   //SwC (Ch7) - Alias for sw3
  uint8_t swD;   //SwD (Ch7)
  uint8_t swF;   //SwF (Ch5) - Alias for sw1
  uint8_t swG;   //SwG (Ch5)
  uint8_t swH;   //SwH (Ch7)
  
  /* PC 鼠标数据 */
  struct
  {
    /* 鼠标移动相关 */
    int16_t x;   //鼠标平移
    int16_t y;   //鼠标上下
    /* 鼠标按键相关，1为按下，0为松开 */
    uint8_t l;   //左侧按键
    uint8_t r;   //右侧按键
  }mouse;
  
  /* PC 键盘按键数据 */
  union 
  {
    uint16_t key_code;
    struct 
    {
      uint16_t W:1;
      uint16_t S:1;
      uint16_t A:1;
      uint16_t D:1;
      uint16_t SHIFT:1;
      uint16_t CTRL:1;
      uint16_t Q:1;
      uint16_t E:1;
      uint16_t R:1;
      uint16_t F:1;
      uint16_t G:1;
      uint16_t Z:1;
      uint16_t X:1;
      uint16_t C:1;
      uint16_t V:1;
      uint16_t B:1;
    }bit;
  }kb;
  
  /* 遥控器左侧拨轮数据 */
  int16_t wheel;
} rc_type_t;

/**
  * @brief     遥控器拨杆数据枚举
  */
enum
{
  RC_UP = 1,
  RC_MI = 3,
  RC_DN = 2,
};

/**
  * @brief     解析遥控器数据
  * @param     rc: 解析后的遥控器数据结构体指针
  * @param     buff: 串口接收到的遥控器原始数据指针
  */
static void remote_data_handle(rc_type_t *rc, uint8_t *buff);
/**
  * @brief     遥控器中断回调函数，在设置 UART 接收时注册
  */
void dbus_uart_callback(void);

extern rc_type_t rc;
extern uint8_t   dbus_recv[];

#endif
