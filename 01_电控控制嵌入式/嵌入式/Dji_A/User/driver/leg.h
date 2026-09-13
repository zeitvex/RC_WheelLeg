#ifndef __LEG_H__
#define __LEG_H__
//#include "lingzu_motor.h"
#include "cmsis_os.h"
//#include "can.h"
#include <stdio.h> // For potential debug
#include <string.h> // For memset if needed
#include <math.h> // For memset if needed

typedef struct {
	float step_length; //步长
	float step_height; //步高
	float period; //周期
	float start_x; //初始x坐标
	float start_y; //初始y坐标
	float start_z; //初始z坐标
	float turn_rate; // 转向率 (-1.0 左转 ~ 1.0 右转)
	float duty_cycle; // 占空比 (摆动相占比, 0.0~1.0)
} TrajectoryParams;

typedef struct {
	float L1; // 大腿长
	float L2; // 小腿长
	float stance_width; // 站立宽度
} RobotGeometry;

void Inverse_Calculation(float X,float Y,float *range1, float *range2, RobotGeometry *geom);
void Trajectory1(float t, TrajectoryParams *params, float *x, float *y, float *z);
void Trajectory2(float t, TrajectoryParams *params, float *x, float *y, float *z);
void Trajectory3(float t, TrajectoryParams *params, float *x, float *y, float *z);
void Trajectory4(float t, TrajectoryParams *params, float *x, float *y, float *z);



#endif
