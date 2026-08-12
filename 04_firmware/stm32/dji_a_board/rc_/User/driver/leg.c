#include "lingzu_motor.h"
#include "cmsis_os.h"
#include "can.h"
#include <stdio.h> // For potential debug
#include <string.h> // For memset if needed
#include <math.h> // For memset if needed
#include "leg.h"
//���䴮������⣨�Ȳ�ƽ�棩
#define M_PI 3.14159265358979323846f

void Inverse_Calculation(float X,float Y,float *range1, float *range2, RobotGeometry *geom){
	float L = sqrtf(X*X + Y*Y);
	float L1 = geom ? geom->L1 : 250.0f;
	float L2 = geom ? geom->L2 : 250.0f;
	
	// Clamp input to acosf to prevent NaN
	float cos_theta1 = (L1*L1 + L2*L2 - L*L)/(2*L1*L2);
	if (cos_theta1 > 1.0f) cos_theta1 = 1.0f;
	else if (cos_theta1 < -1.0f) cos_theta1 = -1.0f;
	
	float theta1 = 3.1415f - acosf(cos_theta1);
	float phi1 = atan2f(-X, Y);
	
	float cos_phi2 = (L*L + L1*L1 - L2*L2)/(2*L*L1);
	if (cos_phi2 > 1.0f) cos_phi2 = 1.0f;
	else if (cos_phi2 < -1.0f) cos_phi2 = -1.0f;
	
	float phi2 = acosf(cos_phi2);
	float theta2 = phi2 + phi1;
	// 修改前腿大腿电机方向：由负号改为正号
	// 原逻辑: *range1 = -theta1*6.33f; (导致伸腿时电机向后转)
	// 新逻辑: *range1 = theta1*1.0f;  (伸腿时电机向前转)
    // 
    // 用户反馈：大腿转的角度过大（接近360度，应为30度左右）
    // 原因分析：
    // 1. 角度过大：代码中曾乘以 6.33 (减速比)，但电机驱动表现为 1:1 映射。
    //    因此移除 6.33 因子。
    // 2. 方向问题：需确保伸腿方向正确。
    
    // 【同步带传动可能的关节耦合补偿】
    // 如果您的机器人是同轴驱动（Hip和Knee电机都固定在机身上，通过皮带传动），
    // 那么当大腿（Hip）转动时，会带动小腿（Knee）皮带轮，导致小腿相对于大腿发生转动。
    // 通常需要补偿：Knee_Motor_Angle = Theta_Knee - Theta_Hip (或 +，取决于绕线方向)
    // 如果发现大腿运动时小腿被动跟随（无法保持相对角度），请取消下方注释并调整符号。
    
    // 2025-01-26 Opt: 启用耦合补偿 (针对同步带结构)
    float coupling_factor = -1.0f; // 请根据实际情况修改符号 (通常为 -1.0 或 1.0)
    theta2 += theta1 * coupling_factor;

	*range1 = theta1 * 1.0f;
	*range2 = theta2 * 1.0f;
}

// 新增前腿逆解函数（适用于前腿伸直向前的零位）
// 假设：X轴正方向为前，Y轴正方向为上（或根据实际安装调整）
// 此处逻辑：如果前腿零位是向前伸直（与后腿向后伸直相反），则需要对计算出的角度进行偏移或镜像
// 通常前腿向前伸直意味着关节角度与后腿相差 180度 (PI) 或者坐标系定义不同
void Inverse_Calculation_Front(float X,float Y,float *range1, float *range2, RobotGeometry *geom){
	// 复用后腿的计算逻辑，但输出时加上偏移
    // 假设前腿安装方式使得其零位在“前方”，而标准算法零位在“后方”
    // 则前腿的实际角度 = 标准计算角度 + PI (或 -PI)
    // 这里的具体符号取决于电机正转方向
    
    float q1, q2;
    Inverse_Calculation(X, Y, &q1, &q2, geom);
    
    // 如果前腿向前伸直是零位，而算法算出的是向后伸直的角度
    // 则需要根据电机转向调整。
    // 简单假设：前腿的运动范围与后腿镜像
    
    // 方案A：直接在输出角度上加偏移 (需要根据减速比 6.33 换算)
    // *range1 = q1 + (3.14159f * 6.33f); 
    // *range2 = q2 + (3.14159f * 6.33f);
    
    // 方案B：保持与 Inverse_Calculation 一致，由上层调用者决定是否加偏移
    // 鉴于目前只修改了校准方式，运动学算法暂不改变，
    // 而是在 Posture 调用时，针对前腿 ID 加上偏移量。
    
    *range1 = q1;
    *range2 = q2;
}



/**
* 1. 单腿轨迹1
* 单腿在x轴上的运动轨迹，y轴上的运动轨迹为0，z轴上的运动轨迹为sin函数
*/
void Trajectory1(float t, TrajectoryParams *params, float *x, float *y, float *z) 
{
	if (t < 0 || t > params->period) {
			*x = params->start_x;
			*y = params->start_y;
			*z = params->start_z;
			return;
		}
	float tau = t / params->period;
	*x = params->start_x + params->step_length * (tau - sin(2 * M_PI * tau) / (2 * M_PI));
	*y = params->start_y;
	*z = params->start_z + params->step_height * (1 - cos(2 * M_PI * tau)) / 2;
}
/**
* 2. 单腿轨迹2
* 单腿在x轴上的运动轨迹，y轴上的运动轨迹为0，z轴上的运动轨迹为sin函数的平方
*/
void Trajectory2(float t, TrajectoryParams *params, float *x, float *y, float *z) {
	if (t < 0 || t > params->period) {
			*x = params->start_x;
			*y = params->start_y;
			*z = params->start_z;
			return;
		}
	float tau = t / params->period;
	*x = params->start_x + params->step_length * tau;
	*y = params->start_y;
	*z = params->start_z + params->step_height * sin(M_PI * tau) * sin(M_PI * tau);
}
/**
* 3. 单腿轨迹3
* 单腿在x轴上的运动轨迹，y轴上的运动轨迹为0，z轴上的运动轨迹为sin函数的立方
*/
void Trajectory3(float t, TrajectoryParams *params, float *x, float *y, float *z) {
	if (t < 0 || t > params->period) {
			*x = params->start_x;
			*y = params->start_y;
			*z = params->start_z;
			return;
		}
	float tau = t / params->period;
	*x = params->start_x + params->step_length * (3 * tau * tau - 2 * tau * tau * tau);
	*y = params->start_y;
	*z = params->start_z + params->step_height * (3 * tau * tau - 2 * tau * tau * tau);
}

/**
* 4. 单腿轨迹4
* 单腿在x轴上的运动轨迹，y轴上的运动轨迹为0，z轴上的运动轨迹为一个三次函数
* 该函数在0到1之间有一个周期，周期为params->period
* 函数的最大值为params->step_height，最小值为0
* 函数的导数在0和1之间为0
*/
/* P3: �յ� (start_x + step_length, start_y, start_z)
*/
void Trajectory4(float t, TrajectoryParams *params, float *x, float *y, float *z) {
		if (t < 0 || t > params->period) {
				*x = params->start_x;
				*y = params->start_y;
				*z = params->start_z;
				return;
			}
float tau = t / params->period;
float tau2 = tau * tau;
float tau3 = tau2 * tau;
float one_minus_tau = 1.0f - tau;
float one_minus_tau2 = one_minus_tau * one_minus_tau;
float one_minus_tau3 = one_minus_tau2 * one_minus_tau;
//���α��������߹�ʽ: B(t) = P0*(1-t)^3 + 3*P1*(1-t)^2*t + 3*P2*(1-t)*t^2 + P3*t^3
float b = 3.0f * one_minus_tau2 * tau;
float c = 3.0f * one_minus_tau * tau2;
float d = tau3;
	*x = params->start_x + one_minus_tau3 * 0.0f +
		b * (params->step_length / 3.0f) +
		c * (2.0f * params->step_length / 3.0f) +
		d * params->step_length;
	*y = params->start_y;
	*z = params->start_z + one_minus_tau3 * 0.0f +
		b * (params->step_height / 2.0f) +
		c * (params->step_height / 2.0f) +
		d * 0.0f;
}
