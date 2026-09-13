/****************************************************************************
 *  RoboMentors www.robomentors.com.
 *	wechat:superzz8080
 *	基于RoboMaster二次开发
 ***************************************************************************/
 
#include "ramp.h"

/* yaw 轴云台控制斜坡 */
ramp_t yaw_ramp = RAMP_GEN_DAFAULT;
/* pitch 轴云台控制斜坡 */
ramp_t pit_ramp = RAMP_GEN_DAFAULT;

/**
  * @brief     斜坡控制结构体初始化
  * @param[in] ramp: 斜坡数据结构体指针
  * @param[in] scale: 控制数据变化斜率
  */
void ramp_init(ramp_t *ramp, int32_t scale)
{
  ramp->count = 0;
  ramp->scale = scale;
}
/**
  * @brief     斜坡控制计算函数
  * @param[in] ramp: 斜坡数据结构体指针
  * @retval    斜坡控制计算输出
  */
float ramp_calc(ramp_t *ramp)
{
  if (ramp->scale <= 0)
    return 0;
  
  if (ramp->count++ >= ramp->scale)
    ramp->count = ramp->scale;
  
  ramp->out = ramp->count / ((float)ramp->scale);
  return ramp->out;
}
