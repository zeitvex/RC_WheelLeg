/****************************************************************************
 *  RoboMentors www.robomentors.com.
 *	wechat:superzz8080
 *	RoboMasterο
 ***************************************************************************/


//ʵ飺GPIOЧ
#include "study_task.h"
#include "uart_task.h"
#include "rm_hal_lib.h"
#include "cmsis_os.h"

void study_task(const void*argu){
	
	// 默认 LED 常亮
	write_led_io(LED_IO1, LED_ON);

	while(1){
		
		if(led_control_queue != NULL)
		{
			// char recv_data;
			// 等待队列消息 (这里只作为LED控制，不控制电机)
			// 注意：如果 lingzu_task 也从同一个队列读取，这里可能会抢占数据。
			// 既然用户说“不要在 study_task 实现”，我们暂时保留LED逻辑（用户未明确删除LED逻辑），
			// 或者，如果用户意图是完全移出，我们应该注释掉读取队列的代码，避免竞争。
			// 但根据“can为1ID为1的电机正转”这个新需求，我们主要关注 lingzu_task。
			// 为了安全起见，我们使用 xQueuePeek 或者让 study_task 暂时不消耗数据，或者假设 lingzu_task 是主要消费者。
			// 鉴于用户之前的描述是“将此数据转化成字符‘q’的形式...传输到study_task中...led熄灭”，
			// 现在的指令是“关于代码实现不要在 study_task... 在 lingzu_task”，可能是指“电机控制逻辑”。
			// 如果两个任务都读队列，肯定会冲突。
			// 既然 lingzu_task 要控制电机，它必须读队列。
			// 那么 study_task 就不应该读队列了，否则 lingzu_task 收不到。
			// 所以这里我们注释掉队列接收，或者移除。
			
			// 暂时移除队列接收逻辑，避免与 lingzu_task 竞争
			/* 
			if(xQueueReceive(led_control_queue, &recv_data, portMAX_DELAY) == pdTRUE)
			{
				switch(recv_data)
				{
					case 'q':
						write_led_io(LED_IO1, LED_OFF);
						break;
					default:
						write_led_io(LED_IO1, LED_ON);
						break;
				}
			}
			*/
			osDelay(100);
		}
		else
		{
			// 如果队列尚未创建，稍作延时
			osDelay(10);
		}
	}
	
}
