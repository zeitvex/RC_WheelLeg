/****************************************************************************
 *  RoboMentors www.robomentors.com.
 *	wechat:superzz8080
 *	基于RoboMaster二次开发
 ***************************************************************************/

#ifndef __RM_LIB_H__
#define __RM_LIB_H__

#include "stm32f4xx_hal.h"

/**
  * @brief     IMU 数据结构体
  */
typedef struct
{
  float acc_x;   //m/s^2
  float acc_y;   //m/s^2
  float acc_z;   //m/s^2
  float gyro_x;  //degree/s
  float gyro_y;  //degree/s
  float gyro_z;  //degree/s
  float angle_x; //degree
  float angle_y; //degree
  float angle_z; //degree
} imu_t;

/**
  * @brief     UART 配置类型枚举
  */
typedef enum
{
  WORD_LEN_8B = 0,
  WORD_LEN_9B,
  STOP_BITS_1,
  STOP_BITS_2,
  PARITY_NONE,
  PARITY_EVEN,
  PARITY_ODD,
} uart_config_e;

/**
  * @brief     数字 IO 引脚方向枚举
  */
typedef enum
{
  IO_INPUT  = 1,
  IO_OUTPUT = 2,
} digital_io_e;

/**
  * @brief     LED IO 状态
  */
typedef enum
{
  LED_ON  = 0,
  LED_OFF = 1,
} led_io_e;

//CAN 设备端口定义
#define USER_CAN1         1  //CAN1
#define USER_CAN2         2  //CAN2
#define CHASSIS_CAN       1  //底盘电机使用CAN1
#define GIMBAL_CAN        1  //云台电机使用CAN1
#define TRIGGER_CAN       1  //拨弹电机使用CAN1
//CAN 设备相关函数
/**
  * @brief     CAN 设备初始化
  */
void can_device_init(void);
/**
  * @brief     发送 CAN 数据
  * @param     can_id: CAN 设备 ID，只有 CAN1 或者 CAN2
  * @param     send_id: 发送数据 ID
  * @param     send_data: 发送数据指针，大小为 8 位
  */
void write_can(uint8_t can_id, uint32_t send_id, uint8_t send_data[]);
/**
  * @brief     注册 CAN 接收回调函数
  * @param     can_id: CAN 设备 ID
  * @param     callback: 接收回调函数指针
  */
void can_recv_callback_register(uint8_t can_id, void (* callback)(uint32_t recv_id, uint8_t data[]));
/**
  * @brief     开启 CAN 接收数据中断
  */
void can_receive_start(void);

//UART 设备端口定义
#define DBUS_UART         1 //遥控器接收机串口
#define USER_UART1        1 //用户串口1，遥控器接收机使用
#define USER_UART2        2 //用户串口2
#define USER_UART3        3 //用户串口3
#define USER_UART4        4 //用户串口4
#define USER_UART5        5 //用户串口5，用于和官方上位机通信
//UART 设备相关函数
/**
  * @brief     UART 设备初始化
  * @param     uart_id: UART 设备 ID
  * @param     baud_rate: 波特率
  * @param     word_len: 字长，数据类型为 uart_config_e
  * @param     stop_bits: 停止位，数据类型为 uart_config_e
  * @param     parity: 校验位，数据类型为 uart_config_e
  */
void uart_init(uint8_t uart_id, uint32_t baud_rate, uart_config_e word_len, \
               uart_config_e stop_bits, uart_config_e parity);
/**
  * @brief     发送 UART 数据
  * @param     uart_id: UART ID
  * @param     send_data: 发送数据指针
  * @param     size: 发送数据的长度
  */
void write_uart(uint8_t uart_id, uint8_t *send_data, uint16_t size);
/**
  * @brief     注册 UART 接收回调函数
  * @param     uart_id: UART ID
  * @param     callback: 接收回调函数指针
  */
void uart_recv_callback_register(uint8_t uart_id, void (* callback)(void));
/**
  * @brief     开启 UART 接收数据中断
  * @param     uart_id: UART ID
  * @param     recv_data: 要接收数据的指针
  * @param     size: 要接收的数据的长度
  */
void uart_receive_start(uint8_t uart_id, uint8_t *recv_data, uint16_t size);


//数字 IO 接口定义
#define DIGI_IO1          1
#define DIGI_IO2          2
#define DIGI_IO3          3
#define DIGI_IO4          4
#define DIGI_IO5          5
#define DIGI_IO6          6
#define DIGI_IO7          7
#define DIGI_IO8          8
#define DIGI_IO9          9
//数字 IO 相关函数
/**
  * @brief     设置数字 IO 方向
  * @param     io_id: 数字 IO 的 ID
  * @param     io_type: IO 方向，数据类型为 digital_io_e
  */
void set_digital_io_dir(uint8_t io_id, digital_io_e io_type);
/**
  * @brief     写数字 IO
  * @param     io_id: 数字 IO 的 ID
  * @param     value: 写入的数据值，只能为0或1，否则设置不成功
  */
void write_digital_io(uint8_t io_id, uint8_t value);
/**
  * @brief     读数字 IO
  * @param     io_id: 数字 IO 的 ID
  * @param     value: 读取数据值的指针，读取的数据值为0或1
  */
void read_digital_io(uint8_t io_id, uint8_t *value);

//LED IO 接口定义
#define LED_IO1           1   //LED1 IO
#define LED_IO2           2   //LED2 IO
#define LED_IO3           3   //LED3 IO
#define LED_IO4           4   //LED4 IO
#define LED_IO5           5   //LED5 IO
#define LED_IO6           6   //LED6 IO
#define LED_IO7           7   //LED7 IO
#define LED_IO8           8   //LED8 IO
#define LED_G             9   //绿色 LED IO
#define LED_R             10  //红色 LED IO
#define LASER_IO          11  //激光 IO
//LED IO 相关函数
/**
  * @brief     写 LED IO
  * @param     led_id: LED IO 的 ID
  * @param     value: 写入的数据值，数据类型为 led_io_e，状态为亮或灭
  */
void write_led_io(uint8_t led_id, led_io_e value);

//PWM IO 接口定义
#define PWM_GROUP1        1   //第一组 PWM IO
#define PWM_IO1           1
#define PWM_IO2           2
#define PWM_IO3           3
#define PWM_IO4           4

#define PWM_GROUP2        2   //第二组 PWM IO
#define PWM_IO5           5
#define PWM_IO6           6
#define PWM_IO7           7
#define PWM_IO8           8

#define PWM_GROUP3        3   //第三组 PWM IO
#define PWM_IO9           9
#define PWM_IO10          10
#define PWM_IO11          11
#define PWM_IO12          12

#define PWM_GROUP4        4   //第四组 PWM IO
#define PWM_IO13          13
#define PWM_IO14          14
#define PWM_IO15          15
#define PWM_IO16          16
//PWM IO 相关函数
/**
  * @brief     开启 PWM 输出
  * @param     pwm_id: PWM IO 的 ID
  */
void start_pwm_output(uint8_t pwm_id);
/**
  * @brief     设置 PWM 组对应参数
  * @param     pwm_group: PWM 组别，目前有 4 组PWM
  * @param     period:  每组 PWM 对应周期时间，单位是微秒(us)
  */
void set_pwm_group_param(uint8_t pwm_group, uint32_t period);
/**
  * @brief     设置 PWM IO 参数
  * @param     pwm_id: PWM IO 的 ID
  * @param     pulse: 配置 PWM 的高电平时间，单位是微秒(us)
  */
void set_pwm_param(uint8_t pwm_id, uint32_t pulse);


//蜂鸣器 IO
/* 板子上只有一个蜂鸣器 */
#define BEEP1_IO        1    //蜂鸣器1 IO

#define BEEP_FREQ       3000 //默认蜂鸣器频率
#define BEEP_ON         1    //打开蜂鸣器
#define BEEP_OFF        0    //关闭蜂鸣器
//蜂鸣器 IO 相关函数
/**
  * @brief     设置蜂鸣器 IO 参数
  * @param     beep_id: 蜂鸣器的 ID
  * @param     freq: 配置蜂鸣器响的声音的频率(Hz)
  * @param     ctrl: 蜂鸣器开启控制，参数可以为BEEP_ON/BEEP_OFF
  */
void set_beep_param(uint8_t beep_id, uint32_t freq, uint8_t ctrl);


//按键 IO 接口定义
#define KEY_IO1          1   //用户按键1
#define KEY_IO2          2   //用户按键2
#define KEY_IO3          3   //用户按键3
#define KEY_IO4          4   //用户按键4
//按键 IO 相关函数
/**
  * @brief     读按键 IO
  * @param     key_id: 按键 IO 的 ID
  * @param     value: 读取数据值的指针，读取的数据值为0或1，按键没有按下时为1，按下为0
  */
void read_key_io(uint8_t key_id, uint8_t *value);


//ADC IO 接口定义
#define ADC_IO1          1   //ADC接口1
//ADC IO 相关函数
/**
  * @brief     读 ADC IO 数据
  * @param     adc_id: ADC IO 的 ID
  * @param     value: 读取数据值的指针，读取的数据值为 uint32_t 类型的正整数，
  *            数据范围为 0 ~ 65535
  *            目前只有一个 ADC 接口，用于拓展版上的霍尔传感器
  */
void read_adc_io(uint8_t adc_id, uint32_t *value);


//数码管接口定义
#define DIG_TUBE1        1
#define DIG_TUBE2        2
#define DIG_TUBE3        3
#define DIG_TUBE4        4
#define DIG_TUBE5        5
#define DIG_TUBE6        6
#define DIG_TUBE7        7
#define DIG_TUBE8        8

#define TUBE_ALL_ON      255  //数码管全亮
#define TUBE_ALL_OFF     0    //数码管全灭
//数码管接口相关函数
/**
  * @brief     数码管初始化
  * @usage     在使用数码管显示前使用，配置数码管各个控制引脚的功能，
  * @attention 因为数码管控制接口和用户接口复用，在使用数码管时 9 个用户数字 IO 不能再被使用
  */
void digital_tube_init(void);
/**
  * @brief     刷新所有数码管
  * @usage     数码管需要显示新的数据时使用
  */
void refresh_digital_tube(void);
/**
  * @brief     第几个数码管显示具体数字
  * @param     pos: 从左到右第几个数码管
  * @param     code: 数码管需要显示的数据，数据类型为 uint8_t 用 10 进制表示的范围为 0 ~ 255
  *                  8位数据由高到低分别对应每段的：A B C D E F G P
  *                  数码管的段和数据代码的对应位置1即可点亮，例如显示数字1，数据代码为96
  *                  每位的具体位置见说明书，其中 P 为小数点
  */
void switch_display_num(uint8_t pos, uint8_t code);



//其他外设接口相关函数
//flash
/**
  * @brief     写 FLASH 设备
  * @param     write_data: 写入数据的指针
  * @param     len: 写入数据的长度
  */
void write_flash(uint8_t *write_data, uint32_t len);
/**
  * @brief     读 FLASH 设备
  * @param     read_data: 读取数据的指针
  * @param     len: 读取数据的长度
  */
void read_flash(uint8_t *read_data, uint32_t len);

//imu
/**
  * @brief     读取 IMU 数据
  * @param     imu_data: 接收 IMU 数据的结构体指针
  * @usage     需要在循环任务中调用，用来刷新 IMU 数据
  */
void get_imu_data(imu_t *imu_data);

#endif
