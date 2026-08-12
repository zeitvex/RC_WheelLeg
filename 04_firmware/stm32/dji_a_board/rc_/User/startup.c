/****************************************************************************
 *  RoboMentors www.robomentors.com.
 *	wechat:superzz8080
 *	基于RoboMaster二次开发
 ***************************************************************************/

#include "startup.h"
#include "sys.h"
#include "study_task.h"
#include "uart_device.h"
#include "can_device.h"
#include "lingzu_motor.h"
#include "uart_task.h"
//#include "chassis_task.h"
//#include "gimbal_task.h"


//运行在定义的任务函数执行前，可以用来初始化任务中用到的 IO 端口，配置、开启外界硬件设备，注册硬件设备的接收回调函数
void init_setup(void){
  
	//关闭所有LED灯
  write_led_io(LED_IO1, LED_OFF);
  write_led_io(LED_IO2, LED_OFF);
  write_led_io(LED_IO3, LED_OFF);
  write_led_io(LED_IO4, LED_OFF);
  write_led_io(LED_IO5, LED_OFF);
  write_led_io(LED_IO6, LED_OFF);
  write_led_io(LED_IO7, LED_OFF);
  write_led_io(LED_IO8, LED_OFF);
	
	digital_tube_init();
	
  HAL_Delay(2000);

  //初始化遥控器接收串口
  uart_init(DBUS_UART, 100000, WORD_LEN_8B, STOP_BITS_2, PARITY_EVEN);
	
  //注册遥控器接收数据回调函数
  uart_recv_callback_register(DBUS_UART, dbus_uart_callback);
	
  //开启遥控器接收
  uart_receive_start(DBUS_UART, dbus_recv, DBUS_FRAME_SIZE);

	//初始化 CAN 设备
  can_device_init();
  can_manual_init();
  
  // 初始化灵足电机数据结构
  Lingzu_Motor_Init_Structs();

	//注册CAN的回调函数
  can_recv_callback_register(USER_CAN1, can1_recv_callback);
  can_recv_callback_register(USER_CAN2, can2_recv_callback);
	
	//开启CAN接收数据中断
  can_receive_start();
}

//开启相关任务函数
extern TaskHandle_t task1_t;
extern TaskHandle_t task2_t;
extern TaskHandle_t task3_t;
extern TaskHandle_t task4_t;
extern TaskHandle_t task5_t;

void sys_start_task(void){
	
#ifdef USER_TASK1
    osThreadDef(ostask1, USER_TASK1, osPriorityAboveNormal, 0, 1024);
    task1_t = osThreadCreate(osThread(ostask1), NULL);
#endif

#ifdef USER_TASK2
    osThreadDef(ostask2, USER_TASK2, osPriorityAboveNormal, 0, 128);
    task2_t = osThreadCreate(osThread(ostask2), NULL);
#endif

#ifdef USER_TASK3
    osThreadDef(ostask3, USER_TASK3, osPriorityHigh, 0, 128);
    task3_t = osThreadCreate(osThread(ostask3), NULL);
#endif

#ifdef USER_TASK4
    osThreadDef(ostask4, USER_TASK4, osPriorityNormal, 0, 512);
    task4_t = osThreadCreate(osThread(ostask4), NULL);
#endif

#ifdef USER_TASK5
    osThreadDef(ostask5, USER_TASK5, osPriorityNormal, 0, 128);
    task5_t = osThreadCreate(osThread(ostask5), NULL);
#endif
}
