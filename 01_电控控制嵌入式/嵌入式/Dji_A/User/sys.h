/****************************************************************************
 *  RoboMentors www.robomentors.com.
 *	wechat:superzz8080
 *	基于RoboMaster二次开发
 ***************************************************************************/

#ifndef __SYS_H__
#define __SYS_H__

#include "rm_hal_lib.h"

/*************************课程模块设置*******************************/
///* 可用的课程模块选项有 */
//enum
//{
//  ALONE_MOTO_TEST,    //单独电机测试
//  ALONE_CHASSIS,      //单独底盘机构
//  ALONE_GIMBAL,       //单独云台机构
//  ALONE_SHOOT,        //单独发射机构
//  COMPLETE_STRUCTURE, //整车结构模式
//};
///* 将上面课程模块使用如下的宏定义使能 */
//#define INFANTRY_STRUCT COMPLETE_STRUCTURE
/* 4轮车，如果使用3轮车注释掉此宏定义 */
#define FOUR_WHEEL

/*************************底盘速度设置*******************************/

/* 遥控器模式下的底盘最大速度限制 */
/* 底盘平移速度 */
#define CHASSIS_RC_MOVE_RATIO_X 1.0f
/* 底盘前进速度 */
#define CHASSIS_RC_MOVE_RATIO_Y 1.0f
/* 底盘旋转速度，只在底盘开环模式下使用 */
#define CHASSIS_RC_MOVE_RATIO_R 1.0f

/* 鼠标键盘模式下的底盘最大速度限制 */
/* 底盘平移速度 */
#define CHASSIS_PC_MOVE_RATIO_X 1.0f
/* 底盘前进速度 */
#define CHASSIS_PC_MOVE_RATIO_Y 1.0f
/* 底盘旋转速度，只在底盘开环模式下使用 */
#define CHASSIS_PC_MOVE_RATIO_R 5.0f

/*************************云台速度设置*******************************/

/* 遥控器模式下的云台速度限制 */
/* 云台pitch轴速度 */
#define GIMBAL_RC_MOVE_RATIO_PIT 1.0f
/* 云台yaw轴速度 */
#define GIMBAL_RC_MOVE_RATIO_YAW 1.0f
/* 鼠标键盘模式下的云台速度限制 */
/* 云台pitch轴速度 */
#define GIMBAL_PC_MOVE_RATIO_PIT 1.2f
/* 云台yaw轴速度 */
#define GIMBAL_PC_MOVE_RATIO_YAW 1.0f


/*************************发射速度设置*******************************/
#define SHOT_FRIC_WHEEL_SPEED    1300 //最大为2500


/*************************发射频率设置*******************************/
#define TRIGGER_MOTOR_SPEED      -5000 //






/*************************下面是系统接口设置，请勿改动**************************/


/* 遥控器的最大行程 */
#define RC_MAX_VALUE      660.0f

#define RC_RATIO          0.002f
#define KB_RATIO          0.02f

/* 单发拨弹的编码器行程 */
#define DEGREE_60_TO_ENCODER  -49146


/********** 4轮底盘信息 **********/
/* 底盘轮距(mm) */
#define WHEELTRACK     353
/* 底盘轴距(mm) */
#define WHEELBASE      370
/* 云台偏移(mm) */
#define GIMBAL_OFFSET  0


/* 底盘轮子周长(mm) */
#define PERIMETER      478

/******** 底盘电机使用3508 *******/
/* 3508底盘电机减速比 */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)
/* 单个电机速度极限，单位是分钟每转 */
#define MAX_WHEEL_RPM        8500   //8347rpm = 3500mm/s


/******** 底盘最大速度设置 *******/
/* 底盘移动最大速度，单位是毫米每秒 */
#define MAX_CHASSIS_VX_SPEED 3300
#define MAX_CHASSIS_VY_SPEED 3300
/* 底盘旋转最大速度，单位是度每秒 */
#define MAX_CHASSIS_VR_SPEED 300

/* yaw轴最大转角 */
#define YAW_ANGLE_MAX        80
/* yaw轴最小转角 */
#define YAW_ANGLE_MIN        -80
/* pitch轴最大仰角 */
#define PIT_ANGLE_MAX        20
/* pitch轴最大俯角 */
#define PIT_ANGLE_MIN        -20



/* 串口数据相关 */
#define MAX_DMA_COUNT        200
#define DBUS_FRAME_SIZE      25

/* 常用的一些物理系数 */
/* 角度转弧度系数 */
#define RADIAN_COEF          57.3f

/* 极值限制函数宏定义 */
#define VAL_LIMIT(val, min, max)\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\

#endif
