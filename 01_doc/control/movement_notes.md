#include "main.h"
#include "Init.h"

PID_motor m3508set[8] = {0};
PID_motor m2006set[8] = {0};
u8 Bound_Flag_Point[2]={1,1};
/**
* @name 	   motor_pid_set
* @brief  	 电机pid初始化
* @param     DJ：内环pid：1000、6、0  外环0.1、0、0.006
* @param     mode：1使用外环
* @param     deadband：10
*/
void motor_pid_set(void)
{
    static u8 i[2];
    static float PID[6] = {1000.0f, 6.0f, 0.0f, 0.1f, 0.0f, 0.006f};
    static float max_out[2] = {16000.0f, 400.0f};
    static float max_idout[2] = {2000.0f, 100.0f};

    for(i[0] = 0; i[0] < 4; i[0]++)
    {
        PID_motoinit(&Motor_PID[i[0]], 1, PID, max_out, max_idout, 0);//3508_1
        PID_motoinit(&Motor_PID[i[0] + 4], 1, PID, max_out, max_idout, 0); //3508_2
        PID_motoinit(&Motor_PID[i[0] + 8], 1, PID, max_out, max_idout, 0); //2006_1
        PID_motoinit(&Motor_PID[i[0] + 12], 1, PID, max_out, max_idout, 0); //2006_2
    }

    for(i[1] = 0; i[1] < 4; i[1]++)
    {
        m3508set[i[1]].setpos = 0;
        m3508set[i[1]].setspeed = 0;
        m3508set[i[1] + 4].setpos = 0;
        m3508set[i[1] + 4].setspeed = 0;

        m2006set[i[1]].setpos = 0;
        m2006set[i[1]].setspeed = 0;
        m2006set[i[1] + 4].setpos = 0;
        m2006set[i[1] + 4].setspeed = 0;
    }
}

/**
* @name 	    MOTOR_PID_CHANGE
* @brief
* @param    	Mid为0为正常
							Mid为1柔性
							Mid 为2 速度环加P
* @param
* @retval
*/
void MOTOR_PID_CHANGE(u8 Mid)
{
    static float OUT_KP[2]= {0.1,0.03};
    static float NUM_max[2]= {16000,8000};
    static float IN_KP[2]= {1000,1400};
    if(Mid==0)
    {
        Motor_PID[0].out_Kp=OUT_KP[0];
        Motor_PID[1].out_Kp=OUT_KP[0];
        Motor_PID[2].out_Kp=OUT_KP[0];
        Motor_PID[3].out_Kp=OUT_KP[0];
        Motor_PID[4].out_Kp=OUT_KP[0];
        Motor_PID[5].out_Kp=OUT_KP[0];
        Motor_PID[6].out_Kp=OUT_KP[0];
        Motor_PID[7].out_Kp=OUT_KP[0];
        Motor_PID[0].in_Kp =IN_KP[0];
        Motor_PID[1].in_Kp =IN_KP[0];
        Motor_PID[2].in_Kp =IN_KP[0];
        Motor_PID[3].in_Kp =IN_KP[0];
        Motor_PID[4].in_Kp =IN_KP[0];
        Motor_PID[5].in_Kp =IN_KP[0];
        Motor_PID[6].in_Kp =IN_KP[0];
        Motor_PID[7].in_Kp =IN_KP[0];
        Motor_PID[0].in_max_out=NUM_max[0];
        Motor_PID[1].in_max_out=NUM_max[0];
        Motor_PID[2].in_max_out=NUM_max[0];
        Motor_PID[3].in_max_out=NUM_max[0];
        Motor_PID[4].in_max_out=NUM_max[0];
        Motor_PID[5].in_max_out=NUM_max[0];
        Motor_PID[6].in_max_out=NUM_max[0];
        Motor_PID[7].in_max_out=NUM_max[0];
    }
    else if(Mid==1)
    {
        Motor_PID[0].out_Kp=OUT_KP[1];
        Motor_PID[1].out_Kp=OUT_KP[1];
        Motor_PID[2].out_Kp=OUT_KP[1];
        Motor_PID[3].out_Kp=OUT_KP[1];
        Motor_PID[4].out_Kp=OUT_KP[1];
        Motor_PID[5].out_Kp=OUT_KP[1];
        Motor_PID[6].out_Kp=OUT_KP[1];
        Motor_PID[7].out_Kp=OUT_KP[1];
        Motor_PID[0].in_Kp =IN_KP[0];
        Motor_PID[1].in_Kp =IN_KP[0];
        Motor_PID[2].in_Kp =IN_KP[0];
        Motor_PID[3].in_Kp =IN_KP[0];
        Motor_PID[4].in_Kp =IN_KP[0];
        Motor_PID[5].in_Kp =IN_KP[0];
        Motor_PID[6].in_Kp =IN_KP[0];
        Motor_PID[7].in_Kp =IN_KP[0];
        Motor_PID[0].in_max_out=NUM_max[1];
        Motor_PID[1].in_max_out=NUM_max[1];
        Motor_PID[2].in_max_out=NUM_max[1];
        Motor_PID[3].in_max_out=NUM_max[1];
        Motor_PID[4].in_max_out=NUM_max[1];
        Motor_PID[5].in_max_out=NUM_max[1];
        Motor_PID[6].in_max_out=NUM_max[1];
        Motor_PID[7].in_max_out=NUM_max[1];
    }
    else if(Mid==2)
    {
        Motor_PID[0].out_Kp=OUT_KP[0];
        Motor_PID[1].out_Kp=OUT_KP[0];
        Motor_PID[2].out_Kp=OUT_KP[0];
        Motor_PID[3].out_Kp=OUT_KP[0];
        Motor_PID[4].out_Kp=OUT_KP[0];
        Motor_PID[5].out_Kp=OUT_KP[0];
        Motor_PID[6].out_Kp=OUT_KP[0];
        Motor_PID[7].out_Kp=OUT_KP[0];
        Motor_PID[0].in_Kp =IN_KP[1];
        Motor_PID[1].in_Kp =IN_KP[1];
        Motor_PID[2].in_Kp =IN_KP[1];
        Motor_PID[3].in_Kp =IN_KP[1];
        Motor_PID[4].in_Kp =IN_KP[1];
        Motor_PID[5].in_Kp =IN_KP[1];
        Motor_PID[6].in_Kp =IN_KP[1];
        Motor_PID[7].in_Kp =IN_KP[1];
        Motor_PID[0].in_max_out=NUM_max[0];
        Motor_PID[1].in_max_out=NUM_max[0];
        Motor_PID[2].in_max_out=NUM_max[0];
        Motor_PID[3].in_max_out=NUM_max[0];
        Motor_PID[4].in_max_out=NUM_max[0];
        Motor_PID[5].in_max_out=NUM_max[0];
        Motor_PID[6].in_max_out=NUM_max[0];
        Motor_PID[7].in_max_out=NUM_max[0];
    }

}
/**
* @name 	    Stop_flat_ground(float Height_body)
* @brief      stop_flag_speed=0初始速度较小，
* @param    	stop_flag_speed=1速度较大电流大
* @param
* @retval    control.s[3]==2 为平地步态下的位置  control.s[3]==3为上斜坡下的位置 control.s[3]==1为下斜坡的位置
*/
void Stop_flat_ground(float Height_body)
{	
//		static float step=170;
 //   static float Slope_Err[2]= {60,60}; //100 240
 //   static float Slope_step[2]= {170,170}; //100,170
    Speed=0.00f;
    /*************对刚体坐标E点坐标初始化**************/
    if(control.s[3]==2&&(control.s[5]==2||control.s[5]==3||control.s[5]==1)&&control.s[4]==2)//平地归零
    {    MOTOR_PID_CHANGE(0);
				
        Xstart[0]=0;
        Xstart[1]=0;
        Xstart[2]=0;
        Xstart[3]=0;
        Xend[0]=Xstart[0];
        Xend[1]=Xstart[1];
        Xend[2]=Xstart[2];
        Xend[3]=Xstart[3];

        Hight[0]=0;
        Hight[3]=Hight[2]=Hight[1]=Hight[0];

        Zstart[0]=-Height_body;  //can2 id 1,2  腿1
        Zstart[1]=-Height_body;  //can2 id 3,4  腿2
        Zstart[2]=-Height_body+20;  //can1 id 1,2  腿3
        Zstart[3]=-Height_body+20;  //can1 id 3,4  腿4
        M3508_stop_postion(10,0.5,TS);
    }
    else if(control.s[3]==3)//上坡落点
    {
        MOTOR_PID_CHANGE(0);
        Xstart[0]=0;
        Xstart[1]=0;
        Xstart[2]=0;
        Xstart[3]=0;

        Xend[0]=Xstart[0];
        Xend[1]=Xstart[1];
        Xend[2]=Xstart[2];
        Xend[3]=Xstart[3];

        Hight[0]=0;
        Hight[3]=Hight[2]=Hight[1]=Hight[0];

        Zstart[0]=-Height_body+50;  //can2 id 1,2  腿1
        Zstart[1]=-Height_body+50;  //can2 id 3,4  腿2
        Zstart[2]=-Height_body;  //can1 id 1,2  腿3
        Zstart[3]=-Height_body;  //can1 id 3,4  腿4

        M3508_stop_postion(15,0.5,TS);
    }
    else if(control.s[3]==1)//下坡落点
    {
        MOTOR_PID_CHANGE(0);
        Xstart[0]=0;
        Xstart[1]=0;
        Xstart[2]=0;
        Xstart[3]=0;

        Xend[0]=Xstart[0];
        Xend[1]=Xstart[1];
        Xend[2]=Xstart[2];
        Xend[3]=Xstart[3];

        Hight[0]=0;
        Hight[3]=Hight[2]=Hight[1]=Hight[0];

        Zstart[0]=-Height_body;  //can2 id 1,2  腿1
        Zstart[1]=-Height_body;  //can2 id 3,4  腿2
        Zstart[2]=-Height_body+50;  //can1 id 1,2  腿3
        Zstart[3]=-Height_body+50;  //can1 id 3,4  腿4

        M3508_stop_postion(15,0.5,TS);
    }
//    else if(control.s[3]==2&&control.s[5]==3)//跷跷板出发位置
//    {
//        MOTOR_PID_CHANGE(0);

//        Xstart[0]=-(Slope_step[0]+Slope_Err[0])/2;
//        Xstart[1]=(Slope_step[0]-Slope_Err[0])/2;
//        Xstart[2]=(Slope_step[1]-Slope_Err[1])/2;
//        Xstart[3]=-(Slope_step[1]+Slope_Err[1])/2;

//        Xend[0]=Xstart[0] ; //+step为前进   腿1
//        Xend[1]=Xstart[1] ;                     //腿2
//        Xend[2]=Xstart[2] ;       //腿3
//        Xend[3]=Xstart[3] ;                     //腿4
//        Hight[0]=0;
//        Hight[3]=Hight[2]=Hight[1]=Hight[0];


//        Zstart[0]=-Height_body+15;     //can2 id 1,2 腿1
//        Zstart[1]=-Height_body+15;     //can2 id 3,4 腿2
//        Zstart[2]=-Height_body+20;  //can1 id 1,2 腿3
//        Zstart[3]=-Height_body+20;  //can1 id 3,4 腿4
//   
//        M3508_stop_postion(20,0.5,TS);
//    }
}



void M3508_stop_postion(float setspeed,float arr,float Ts)
{
    static float M3508_Angle[16]= {0} ;
    static float M2006_Angle[16]= {0} ;

    /******************生成E,点坐标********************************/
    Leg_cycloid(0,&link[0],arr,Ts,Hight[0],Zstart[0],Xstart[0],Xend[0]);
    Leg_cycloid(1,&link[1],arr,Ts,Hight[1],Zstart[1],Xstart[1],Xend[1]);

    Leg_cycloid(2,&link[2],arr,Ts,Hight[2],Zstart[2],Xstart[2],Xend[2]);
    Leg_cycloid(3,&link[3],arr,Ts,Hight[3],Zstart[3],Xstart[3],Xend[3]);


    Inverse_leg(&leg[0],&link[0]);
    Inverse_leg(&leg[1],&link[1]);	 //E3,E4的运动学逆解
    Inverse_leg(&leg[2],&link[2]);
    Inverse_leg(&leg[3],&link[3]);	 //E3,E4的运动学逆解

    M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //后 can2 ID1,2 腿1
    M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

    M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //后 can2 ID3,4 腿3
    M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

    M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //前 can1 ID1,2 腿2
    M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

    M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //前 can1 ID3,4 腿4
    M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

    m2006set[0].setspeed=setspeed;
    m2006set[1].setspeed=setspeed;
    m2006set[2].setspeed=setspeed;
    m2006set[3].setspeed=setspeed;

    m3508set[0].setspeed=setspeed;
    m3508set[1].setspeed=setspeed;
    m3508set[2].setspeed=setspeed;
    m3508set[3].setspeed=setspeed;

    m2006set[0].setpos = M2006_Angle[0];
    m2006set[1].setpos = M2006_Angle[1];
    m2006set[2].setpos = M2006_Angle[2];
    m2006set[3].setpos = M2006_Angle[3];

    m3508set[0].setpos = M3508_Angle[0];
    m3508set[1].setpos = M3508_Angle[1];
    m3508set[2].setpos = M3508_Angle[2];
    m3508set[3].setpos = M3508_Angle[3];
}
/**
* @name 	    trot_flat_ground
* @brief  		对角步态
* @param    	speed 迈步速度
* @param      T[0]=T[3]=0 T[2]=T[1]=0.5  顺序位 左后右前->左前有后
* @retval    control.s[3]==2 为平地步态下的位置  control.s[3]==3为上斜坡下的位置 control.s[3]==1为下斜坡的位置
*/

void trot_flat_ground(float step,float length_hight,float Height_body,float speed)
{
    MOTOR_PID_CHANGE(0);
    static float Deviation_Err=5;//8  平地误差
    static float Deviation_Err_s5=0;//8  翘翘板误差
    static float Slope_Err[2]= {60,60}; //60 60  跷跷板滞后位置
    static float Slope_step[2]= {140,140}; //170,170 跷跷板步长
    /*************对刚体坐标E点坐标初始化**************/

    if(control.s[5]==3)//跷跷板前进轨迹
    {
        Speed=0.0035;
			if(control.s[1]==4)
			{
				Xstart[0]=-(Slope_Err[0]+Slope_step[0]+38)/2.0f;
				Xstart[1]=-(Slope_Err[0]+Slope_step[0]-30)/2.0f;
        Xstart[2]=-(Slope_Err[1]+Slope_step[1]+38)/2.0f;
        Xstart[3]=-(Slope_Err[1]+Slope_step[1]-30)/2.0f;

        Xend[0]=Xstart[0] + Slope_step[0] +38 ; //+step为前进   腿1
        Xend[1]=Xstart[1] + Slope_step[0] -30  ;                     //腿2
        Xend[2]=Xstart[2] + Slope_step[1] +38 ;       //腿3
        Xend[3]=Xstart[3] + Slope_step[1] -30  ;                     //腿4

			}
			else if(control.s[1]==3)
			{
				Xstart[0]=-(Slope_Err[0]+Slope_step[0]-38)/2.0f;
				Xstart[1]=-(Slope_Err[0]+Slope_step[0]+48)/2.0f;
        Xstart[2]=-(Slope_Err[1]+Slope_step[1]-38)/2.0f;
        Xstart[3]=-(Slope_Err[1]+Slope_step[1]+48)/2.0f;

        Xend[0]=Xstart[0] + Slope_step[0] -38 ; //+step为前进   腿1
        Xend[1]=Xstart[1] + Slope_step[0] +48  ;                     //腿2
        Xend[2]=Xstart[2] + Slope_step[1] -38 ;       //腿3
        Xend[3]=Xstart[3] + Slope_step[1] +48  ;                     //腿4

			}
			else
			{
				Xstart[0]=-(Slope_Err[0]+Slope_step[0]-Deviation_Err_s5)/2.0f;
				Xstart[1]=-(Slope_Err[0]+Slope_step[0])/2.0f;
        Xstart[2]=-(Slope_Err[1]+Slope_step[1]-Deviation_Err_s5)/2.0f;
        Xstart[3]=-(Slope_Err[1]+Slope_step[1])/2.0f;

        Xend[0]=Xstart[0] + Slope_step[0] -Deviation_Err_s5 ; //+step为前进   腿1
        Xend[1]=Xstart[1] + Slope_step[0]-0  ;                     //腿2
        Xend[2]=Xstart[2] + Slope_step[1] -Deviation_Err_s5 ;       //腿3
        Xend[3]=Xstart[3] + Slope_step[1]-0  ;                     //腿4
			}
        Hight[0]=length_hight-20;
        Hight[1]=length_hight-20;
        Hight[2]=length_hight-20;
        Hight[3]=length_hight-20;

        Zstart[0]=-Height_body+15;     //can2 id 1,2 腿1
        Zstart[1]=-Height_body+15;     //can2 id 3,4 腿2
        Zstart[2]=-Height_body+20;  //can1 id 1,2 腿3
        Zstart[3]=-Height_body+20;  //can1 id 3,4 腿4
    }

     if(control.s[3]==2&&(control.s[5]==2||control.s[5]==1)&&control.s[4]==2)//正常出发轨迹
    {
        Speed=speed;
			if(control.s[1]==4)
			{
        Xstart[2]=Xstart[0]=-(130-Deviation_Err)/2.0f;
        Xstart[3]=Xstart[1]=-100/2.0f;
        Xend[0]=Xstart[0] + 130 -Deviation_Err ; //+step为前进   腿1
        Xend[1]=Xstart[1] + 100  ;                     //腿2
        Xend[2]=Xstart[2] + 130 -Deviation_Err ;       //腿3
        Xend[3]=Xstart[3] + 100  ; 
				}
			else if(control.s[1]==3)
			{
        Xstart[2]=Xstart[0]=-(100-Deviation_Err)/2.0f;
        Xstart[3]=Xstart[1]=-130/2.0f;
        Xend[0]=Xstart[0] + 100 -Deviation_Err ; //+step为前进   腿1
        Xend[1]=Xstart[1] + 130  ;                     //腿2
        Xend[2]=Xstart[2] + 100 -Deviation_Err ;       //腿3
        Xend[3]=Xstart[3] + 130  ; 
			}
			else 
			{
        Xstart[2]=Xstart[0]=-(100-Deviation_Err)/2.0f;
        Xstart[3]=Xstart[1]=-100/2.0f;
        Xend[0]=Xstart[0] + 100 -Deviation_Err ; //+step为前进   腿1
        Xend[1]=Xstart[1] + 100  ;                     //腿2
        Xend[2]=Xstart[2] + 100 -Deviation_Err ;       //腿3
        Xend[3]=Xstart[3] + 100  ;                     //腿4
			}
        Hight[0]=length_hight+0;
        Hight[1]=length_hight+0;
        Hight[2]=length_hight+0;
        Hight[3]=length_hight+0;

        Zstart[0]=-Height_body;     //can2 id 1,2 腿1
        Zstart[1]=-Height_body;     //can2 id 3,4 腿2
        Zstart[2]=-Height_body+20;  //can1 id 1,2 腿3
        Zstart[3]=-Height_body+20;  //can1 id 3,4 腿4
    }
    else if(control.s[3]==3&&control.s[5]==2)//上坡调整轨迹 步长为50
    {
        Speed=speed;
//			if(control.s[1]==4)
//			{
//			
//        Xstart[0]=-(85)/2.0f;
//        Xstart[1]=-50/2.0f;
//        Xstart[2]=-(85+50)/2.0f;
//        Xstart[3]=-(50+50)/2.0f;

//        Xend[0]=Xstart[0] + 85  ; //+step为前进   腿1
//        Xend[1]=Xstart[1] + 50  ;                     //腿2
//        Xend[2]=Xstart[2] + 85  ;       //腿3
//        Xend[3]=Xstart[3] + 50  ;                     //腿4

//			}
//			else if(control.s[1]==3)
//			{
//			
//        Xstart[0]=-(55)/2.0f;
//        Xstart[1]=-80/2.0f;
//        Xstart[2]=-(55+50)/2.0f;
//        Xstart[3]=-(80+50)/2.0f;

//        Xend[0]=Xstart[0] + 55  ; //+step为前进   腿1
//        Xend[1]=Xstart[1] + 80  ;                     //腿2
//        Xend[2]=Xstart[2] + 55  ;       //腿3
//        Xend[3]=Xstart[3] + 80  ;                     //腿4

//			}
			
			
        Xstart[0]=-(30)/2.0f;
        Xstart[1]=-30/2.0f;
        Xstart[2]=-(0+30)/2.0f;
        Xstart[3]=-(0+30)/2.0f;

        Xend[0]=Xstart[0] + 30  ; //+step为前进   腿1
        Xend[1]=Xstart[1] + 30  ;                     //腿2
        Xend[2]=Xstart[2] + 30  ;       //腿3
        Xend[3]=Xstart[3] + 30  ;                     //腿4
			
        Hight[0]=length_hight-30;
        Hight[1]=length_hight-30;
        Hight[2]=length_hight-10;
        Hight[3]=length_hight-10;

        Zstart[0]=-Height_body+40;     //can2 id 1,2 腿1
        Zstart[1]=-Height_body+40;     //can2 id 3,4 腿2
        Zstart[2]=-Height_body-10;  //can1 id 1,2 腿3
        Zstart[3]=-Height_body-10;  //can1 id 3,4 腿4
    }
    else if(control.s[3]==1&&control.s[5]==2)
    {
        Speed=0.0032;

        Xstart[0]=-(75-Deviation_Err)/2.0f;
        Xstart[1]=-75/2.0f;
        Xstart[2]=-(75-Deviation_Err)/2.0f;
        Xstart[3]=-(75)/2.0f;

        Xend[0]=Xstart[0] + 75 -Deviation_Err ; //+step为前进   腿1
        Xend[1]=Xstart[1] + 75  ;                     //腿2
        Xend[2]=Xstart[2] + 75 -Deviation_Err ;       //腿3
        Xend[3]=Xstart[3] + 75  ;                     //腿4
                //腿4

        Hight[0]=length_hight-0;
        Hight[1]=length_hight-0;
        Hight[2]=length_hight-30;
        Hight[3]=length_hight-30;

        Zstart[0]=-Height_body-10;     //can2 id 1,2 腿1
        Zstart[1]=-Height_body-10;     //can2 id 3,4 腿2
        Zstart[2]=-Height_body+40;  //can1 id 1,2 腿3
        Zstart[3]=-Height_body+40;  //can1 id 3,4 腿4
    }

    M3508_trot_postion(0.5,TS);
}

/**
* @name 	    trot_flat_ground
* @brief  		对角步态
* @param    	speed 迈步速度
* @param      T[0]=T[3]=0 T[2]=T[1]=0.5  顺序位 左后右前->左前有后
* @retval    control.s[3]==2 为平地步态下的位置  control.s[3]==3为上斜坡下的位置 control.s[3]==1为下斜坡的位置
*/
void trot_behind_flat_ground(float step,float length_hight,float Height_body,float speed)
{
    MOTOR_PID_CHANGE(0);

    Speed=speed;
    if(control.s[3]==2)//平地正常
    {
        /*************对刚体坐标E点坐标初始化**************/
        Xstart[2]=Xstart[0]=step/2.0f;
        Xstart[3]=Xstart[1]=step/2.0f;

        Xend[0]=Xstart[0] - step ; //-step 为后退
        Xend[1]=Xstart[1] - step ;
        Xend[2]=Xstart[2] - step ;
        Xend[3]=Xstart[3] - step ;

        Hight[3]=length_hight;
        Hight[2]=length_hight;
        Hight[1]=length_hight;
        Hight[0]=length_hight;


        Zstart[0]=-Height_body+15;  //can2 id 1,2 腿1
        Zstart[1]=-Height_body+15;  //can2 id 3,4 腿2
        Zstart[2]=-Height_body;  //can1 id 1,2 腿3
        Zstart[3]=-Height_body;  //can1 id 3,4 腿4
    }
    if(control.s[3]==3)//上坡
    {
				Xstart[0]=(50)/2.0f;
        Xstart[1]=50/2.0f;
        Xstart[2]=(50)/2.0f;
        Xstart[3]=(50)/2.0f;
        /*************对刚体坐标E点坐标初始化**************/
//        Xstart[0]=step/2.0f;
//        Xstart[1]=step/2.0f;
//        Xstart[2]=(step)/2.0f;
//        Xstart[3]=(step)/2.0f;

        Xend[0]=Xstart[0] - 50 ; //-step 为后退
        Xend[1]=Xstart[1] - 50 ;
        Xend[2]=Xstart[2] - 50 ;
        Xend[3]=Xstart[3] - 50 ;

        Hight[0]=length_hight-40;
        Hight[1]=length_hight-40;
        Hight[2]=length_hight-40;
        Hight[3]=length_hight-40;

        Zstart[0]=-Height_body+40;  //can2 id 1,2 腿1
        Zstart[1]=-Height_body+40;  //can2 id 3,4 腿2
        Zstart[2]=-Height_body-10;  //can1 id 1,2 腿3
        Zstart[3]=-Height_body-10;  //can1 id 3,4 腿4
    }
    M3508_trot_postion(0.5,TS);
}
/**
* @name 	    trot_flat_Right_ground
* @brief  		对角步态右转
* @param    	speed 迈步速度
* @param      T[0]=T[3]=0 T[2]=T[1]=0.5  顺序位 左后右前->左前有后
* @retval
*/
void trot_flat_Right_ground(float step,float length_hight,float Height_body,float speed)
{
 //   static float Deviation_Err=30;//8
    MOTOR_PID_CHANGE(0);
    /*************对刚体坐标E点坐标初始化**************/
    static float Deviation_Err_s5=50;//8
    static float Slope_Err[2]= {60,60}; //100 240
    static float Slope_step[2]= {150,150}; //100,170
    /*************对刚体坐标E点坐标初始化**************/
//    if(control.s[5]==1)
//    {
//        Speed=0.0025;

//				Xstart[0]=-(Slope_Err[0]+Slope_step[0])/2.0f;
//				Xstart[1]=-(Slope_Err[0]+Slope_step[0]-Deviation_Err_s5)/2.0f;
//        Xstart[2]=-(Slope_Err[1]+Slope_step[1])/2.0f;
//        Xstart[3]=-(Slope_Err[1]+Slope_step[1]-Deviation_Err_s5)/2.0f;

//        Xend[0]=Xstart[0] + Slope_step[0]  ; //+step为前进   腿1
//        Xend[1]=Xstart[1] + Slope_step[0] -Deviation_Err_s5 ;                     //腿2
//        Xend[2]=Xstart[2] + Slope_step[1]  ;       //腿3
//        Xend[3]=Xstart[3] + Slope_step[1] -Deviation_Err_s5 ;                     //腿4

//        Hight[0]=length_hight-0;
//        Hight[1]=length_hight-0;
//        Hight[2]=length_hight-20;
//        Hight[3]=length_hight-20;

//        Zstart[0]=-Height_body+25;     //can2 id 1,2 腿1
//        Zstart[1]=-Height_body+25;     //can2 id 3,4 腿2
//        Zstart[2]=-Height_body+20;  //can1 id 1,2 腿3
//        Zstart[3]=-Height_body+20;  //can1 id 3,4 腿4
//    }
    if(control.s[3]==2&&control.s[5]==2)
    {
        Speed=speed;
        Xstart[0]=-step/2.0f;
        Xstart[1]=step/2.0f;
        Xstart[2]=-step/2.0f;
        Xstart[3]=step/2.0f;

        Xend[0]=Xstart[0] + step ;
        Xend[1]=Xstart[1] - step ;
        Xend[2]=Xstart[2] + step ;
        Xend[3]=Xstart[3] - step ;


        Hight[3]=length_hight;
        Hight[2]=length_hight;
        Hight[1]=length_hight;
        Hight[0]=length_hight;

        Zstart[0]=-Height_body;  //can2 id 1,2
        Zstart[1]=-Height_body;  //can2 id 3,4
        Zstart[2]=-Height_body;  //can1 id 1,2
        Zstart[3]=-Height_body;  //can1 id 3,4
    }
    if(control.s[3]==3&&control.s[5]==2)
    {
        Speed=speed;
				Xstart[0]=(30)/2.0f;
        Xstart[1]=30/2.0f;
        Xstart[2]=-(30)/2.0f;
        Xstart[3]=-(30)/2.0f;

        Xend[0]=Xstart[0] + 30 ;
        Xend[1]=Xstart[1] - 30 ;
        Xend[2]=Xstart[2] + 30 ;
        Xend[3]=Xstart[3] - 30 ;

        Hight[3]=length_hight-40;
        Hight[2]=length_hight-40;
        Hight[1]=length_hight-40;
        Hight[0]=length_hight-40;

        Zstart[0]=-Height_body+50;  //can2 id 1,2
        Zstart[1]=-Height_body+50;  //can2 id 3,4
        Zstart[2]=-Height_body;  //can1 id 1,2
        Zstart[3]=-Height_body;  //can1 id 3,4
    }
    if(control.s[3]==1&&control.s[5]==2)
    {
        Speed=speed;
				Xstart[0]=(20)/2.0f;
        Xstart[1]=20/2.0f;
        Xstart[2]=-(20)/2.0f;
        Xstart[3]=-(20)/2.0f;

        Xend[0]=Xstart[0] + step ;
        Xend[1]=Xstart[1] - step ;
        Xend[2]=Xstart[2] + step ;
        Xend[3]=Xstart[3] - step ;

        Hight[3]=length_hight-40;
        Hight[2]=length_hight-40;
        Hight[1]=length_hight-40;
        Hight[0]=length_hight-40;

        Zstart[0]=-Height_body;  //can2 id 1,2
        Zstart[1]=-Height_body;  //can2 id 3,4
        Zstart[2]=-Height_body+50;  //can1 id 1,2
        Zstart[3]=-Height_body+50;  //can1 id 3,4
    }
    M3508_trot_postion(0.5,TS);

}
/**
* @name 	    trot_flat_Left_ground
* @brief  		对角步态左转
* @param    	speed 迈步速度
* @param      T[0]=T[3]=0 T[2]=T[1]=0.5  顺序位 左后右前->左前有后
* @retval
*/
void trot_flat_Left_ground(float step,float length_hight,float Height_body,float speed)
{
//    static float Deviation_Err=-30;//8
//    static float Slope_Err[2]= {100,240}; //100 240
//    static float Slope_step[2]= {170,120}; //100,170
    MOTOR_PID_CHANGE(0);
    /*************对刚体坐标E点坐标初始化**************/
    static float Deviation_Err_s5=50;//8
    static float Slope_Err[2]= {60,60}; //100 240
    static float Slope_step[2]= {150,150}; //100,170
    /*************对刚体坐标E点坐标初始化**************/
//    if(control.s[5]==1)
//    {
//        Speed=0.0025;

//				Xstart[0]=-(Slope_Err[0]+Slope_step[0]-Deviation_Err_s5)/2.0f;
//				Xstart[1]=-(Slope_Err[0]+Slope_step[0])/2.0f;
//        Xstart[2]=-(Slope_Err[1]+Slope_step[1]-Deviation_Err_s5)/2.0f;
//        Xstart[3]=-(Slope_Err[1]+Slope_step[1])/2.0f;

//        Xend[0]=Xstart[0] + Slope_step[0] -Deviation_Err_s5 ; //+step为前进   腿1
//        Xend[1]=Xstart[1] + Slope_step[0]  ;                     //腿2
//        Xend[2]=Xstart[2] + Slope_step[1] -Deviation_Err_s5 ;       //腿3
//        Xend[3]=Xstart[3] + Slope_step[1]  ;                     //腿4

//        Hight[0]=length_hight-0;
//        Hight[1]=length_hight-0;
//        Hight[2]=length_hight-20;
//        Hight[3]=length_hight-20;

//        Zstart[0]=-Height_body+25;     //can2 id 1,2 腿1
//        Zstart[1]=-Height_body+25;     //can2 id 3,4 腿2
//        Zstart[2]=-Height_body+20;  //can1 id 1,2 腿3
//        Zstart[3]=-Height_body+20;  //can1 id 3,4 腿4
//    }

    if(control.s[3]==2&&control.s[5]==2)
    {
			Speed =speed;
        /*************对刚体坐标E点坐标初始化**************/
        Xstart[0]=step/2.0f;
        Xstart[1]=-step/2.0f;
        Xstart[2]=step/2.0f;
        Xstart[3]=-step/2.0f;

        Xend[0]=Xstart[0] - step ;
        Xend[1]=Xstart[1] + step ;
        Xend[2]=Xstart[2] - step ;
        Xend[3]=Xstart[3] + step ;
        Zstart[0]=-Height_body;  //can2 id 1,2
        Zstart[1]=-Height_body;  //can2 id 3,4
        Zstart[2]=-Height_body;  //can1 id 1,2
        Zstart[3]=-Height_body;  //can1 id 3,4
        Hight[3]=Hight[2]=Hight[1]=Hight[0]=length_hight;

    }
    if(control.s[3]==3&&control.s[5]==2)
    {
			Speed =speed;
        /*************对刚体坐标E点坐标初始化**************/
				Xstart[0]=(30)/2.0f;
        Xstart[1]=-30/2.0f;
        Xstart[2]=(50)/2.0f;
        Xstart[3]=-(50)/2.0f;

        Xend[0]=Xstart[0] - 30 ;
        Xend[1]=Xstart[1] + 30 ;
        Xend[2]=Xstart[2] - 50 ;
        Xend[3]=Xstart[3] + 50 ;
        Zstart[0]=-Height_body+50;  //can2 id 1,2
        Zstart[1]=-Height_body+50;  //can2 id 3,4
        Zstart[2]=-Height_body;  //can1 id 1,2
        Zstart[3]=-Height_body;  //can1 id 3,4

        Hight[0]=length_hight-40;
        Hight[1]=length_hight-40;
        Hight[2]=length_hight-40;
        Hight[3]=length_hight-40;
    }
    if(control.s[3]==1&&control.s[5]==2)
    {
			Speed =speed;
        /*************对刚体坐标E点坐标初始化**************/
				Xstart[0]=(30)/2.0f;
        Xstart[1]=30/2.0f;
        Xstart[2]=-(30)/2.0f;
        Xstart[3]=-(30)/2.0f;

        Xend[0]=Xstart[0] - step ;
        Xend[1]=Xstart[1] + step ;
        Xend[2]=Xstart[2] - step ;
        Xend[3]=Xstart[3] + step ;
        Zstart[0]=-Height_body;  //can2 id 1,2
        Zstart[1]=-Height_body;  //can2 id 3,4
        Zstart[2]=-Height_body+50;  //can1 id 1,2
        Zstart[3]=-Height_body+50;  //can1 id 3,4

        Hight[0]=length_hight-40;
        Hight[1]=length_hight-40;
        Hight[2]=length_hight-40;
        Hight[3]=length_hight-40;
    }
    M3508_trot_postion(0.5,TS);
}

/*
* @name 	    M3508_postion
* @brief  		对角步态电机位置
* @param    	trot :arr占空比一般取0.5
* @param 			Walk :arr占空比一般取0.25
* @param      Bound:arr占空比一般取0.1
* @retval
*/
void M3508_trot_postion(float arr,float Ts)
{
    static float M3508_Angle[16]= {0} ;
    static float M2006_Angle[16]= {0} ;
    /******************生成E,点坐标********************************/
    Leg_cycloid(0,&link[0],arr,Ts,Hight[0],Zstart[0],Xstart[0],Xend[0]);
    Leg_cycloid(1,&link[1],arr,Ts,Hight[1],Zstart[1],Xstart[1],Xend[1]);
    Leg_cycloid(2,&link[2],arr,Ts,Hight[2],Zstart[2],Xstart[2],Xend[2]);
    Leg_cycloid(3,&link[3],arr,Ts,Hight[3],Zstart[3],Xstart[3],Xend[3]);

    Inverse_leg(&leg[0],&link[0]);
    Inverse_leg(&leg[1],&link[1]);	 //E3,E4的运动学逆解
    Inverse_leg(&leg[2],&link[2]);
    Inverse_leg(&leg[3],&link[3]);	 //E3,E4的运动学逆解

    M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //后 can2 ID1,2 腿1
    M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

    M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //后 can2 ID3,4 腿3
    M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

    M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //前 can1 ID1,2 腿2
    M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

    M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //前 can1 ID3,4 腿4
    M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

    if(link[0].point_E.x==Xend[0]||link[0].point_E.x==Xstart[0])
    {
        m2006set[0].setspeed  = 0;
        m2006set[1].setspeed  = 0;
//        LED0=0;
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);
    }
    else
    {
        m2006set[0].setspeed  = 150;
        m2006set[1].setspeed  = 150;
//        LED0=1;
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
    }
    if(link[1].point_E.x==Xend[1]||link[1].point_E.x==Xstart[1])
    {
        m2006set[2].setspeed  = 0;
        m2006set[3].setspeed  = 0;
//        LED0=0;
		
HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);
    }
    else
    {
        m2006set[2].setspeed  = 150;
        m2006set[3].setspeed  = 150;
//        LED0=1;
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
    }
    if(link[2].point_E.x==Xend[2]||link[2].point_E.x==Xstart[2])
    {
        m3508set[0].setspeed = 0;
        m3508set[1].setspeed = 0;
//        LED0=0;
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);
    }
    else
    {
        m3508set[0].setspeed = 150;
        m3508set[1].setspeed = 150;
//        LED0=1;//
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
    }
    if(link[3].point_E.x==Xend[3]||link[3].point_E.x==Xstart[3])
    {
        m3508set[2].setspeed = 0;
        m3508set[3].setspeed = 0;
//        LED0=0;
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);
    }
    else
    {
        m3508set[2].setspeed = 150;
        m3508set[3].setspeed = 150;
//        LED0=1;
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
    }

    m2006set[0].setpos = M2006_Angle[0];
    m2006set[1].setpos = M2006_Angle[1];
    m2006set[2].setpos = M2006_Angle[2];
    m2006set[3].setpos = M2006_Angle[3];

    m3508set[0].setpos = M3508_Angle[0];
    m3508set[1].setpos = M3508_Angle[1];
    m3508set[2].setpos = M3508_Angle[2];
    m3508set[3].setpos = M3508_Angle[3];
}


/**
* @name 	    BOUND_Inclined_Plane_flat_ground
* @brief  		斜向上跳跃
* @param    	speed 迈步速度
* @param      T[0]=T[3]=0 T[2]=T[1]=0.5  顺序位 左后右前->左前有后
* @retval
*/
void BOUND_Inclined_Plane_flat_ground(float Height_body_start,float Height_body_end)
{
    static float M2006_Angle[4]= {0};
    static float M3508_Angle[4]= {0};
    static float point_Speed[6]= {0,15,150,150,150,20};
    static u8 bound_flag_point2=1;
    LCD_Num(50, 240,bound_flag_point2,1, 16);
    /*******************第一个点***********************/
    if(Bound_Flag_Point[0]==1)
    {
        MOTOR_PID_CHANGE(1);
        Leg_Point(0,&link[0],-Height_body_start,-50);
        Leg_Point(1,&link[1],-Height_body_start,-50);
        Leg_Point(2,&link[2],-Height_body_start,-85);
        Leg_Point(3,&link[3],-Height_body_start,-85);//85

        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //后 can2 ID1,2 腿1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //后 can2 ID3,4 腿3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //前 can1 ID1,2 腿2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //前 can1 ID3,4 腿4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[1];
        m2006set[1].setspeed  = point_Speed[1];
        m2006set[2].setspeed  = point_Speed[1];
        m2006set[3].setspeed  = point_Speed[1];
        m3508set[0].setspeed  = point_Speed[1];
        m3508set[1].setspeed  = point_Speed[1];
        m3508set[2].setspeed  = point_Speed[1];
        m3508set[3].setspeed  = point_Speed[1];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];

        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if(control.s[0]==1)Bound_Flag_Point[0]=2;

    }
		else if(control.s[2]==3)
		{bound_flag_point2=1;}
    /**********************第二个点*********************************/
    else if(Bound_Flag_Point[0]==2&&(control.s[2]==2||control.s[2]==1))
    {
        MOTOR_PID_CHANGE(0);//速度P加大
        if(control.s[2]==3)bound_flag_point2=1;

        Leg_Point(0,&link[0],-Height_body_end,-140);
        Leg_Point(1,&link[1],-Height_body_end,-140);//双目桥
        Leg_Point(2,&link[2],-Height_body_end,-140);
        Leg_Point(3,&link[3],-Height_body_end,-140);

        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //后 can2 ID1,2 腿1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //后 can2 ID3,4 腿3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //前 can1 ID1,2 腿2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //前 can1 ID3,4 腿4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[2];
        m2006set[1].setspeed  = point_Speed[2];
        m2006set[2].setspeed  = point_Speed[2];
        m2006set[3].setspeed  = point_Speed[2];
        m3508set[0].setspeed  = point_Speed[2];
        m3508set[1].setspeed  = point_Speed[2];
        m3508set[2].setspeed  = point_Speed[2];
        m3508set[3].setspeed  = point_Speed[2];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m2006set[0].setpos-m2006[0].real_angle)<15)&&(fabs(m2006set[1].setpos-m2006[1].real_angle)<15)&&(control.s[2]==2||control.s[2]==1))Bound_Flag_Point[0]=3;
    }
    else if(Bound_Flag_Point[0]==3&&(control.s[2]==2||control.s[2]==1))
    {
        /*****************3点************************/
        MOTOR_PID_CHANGE(0);
        if(control.s[2]==3)bound_flag_point2=1;


        Leg_Point(0,&link[0],-Height_body_start,0);
        Leg_Point(1,&link[1],-Height_body_start,0);
        Leg_Point(2,&link[2],-Height_body_start,0);
        Leg_Point(3,&link[3],-Height_body_start,0);
        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //后 can2 ID1,2 腿1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //后 can2 ID3,4 腿3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //前 can1 ID1,2 腿2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //前 can1 ID3,4 腿4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[3];
        m2006set[1].setspeed  = point_Speed[3];
        m2006set[2].setspeed  = point_Speed[3];
        m2006set[3].setspeed  = point_Speed[3];
        m3508set[0].setspeed  = point_Speed[3];
        m3508set[1].setspeed  = point_Speed[3];
        m3508set[2].setspeed  = point_Speed[3];
        m3508set[3].setspeed  = point_Speed[3];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m3508set[1].setpos-m3508[1].real_angle)<20)&&(fabs(m3508set[0].setpos-m3508[0].real_angle)<20)&&(control.s[2]==2||control.s[2]==1))Bound_Flag_Point[0]=4;

    }
    /**********************第4个点*********************************/
    else if(Bound_Flag_Point[0]==4&&(control.s[2]==2||control.s[2]==1))
    {
        MOTOR_PID_CHANGE(1);
        if(control.s[2]==3)bound_flag_point2=1;

        Leg_Point(0,&link[0],-Height_body_start-50,120);//-60,120
        Leg_Point(1,&link[1],-Height_body_start-50,120);
        Leg_Point(2,&link[2],-Height_body_start-50,120);
        Leg_Point(3,&link[3],-Height_body_start-50,120);
        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //后 can2 ID1,2 腿1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //后 can2 ID3,4 腿3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //前 can1 ID1,2 腿2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //前 can1 ID3,4 腿4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[4];
        m2006set[1].setspeed  = point_Speed[4];
        m2006set[2].setspeed  = point_Speed[4];
        m2006set[3].setspeed  = point_Speed[4];
        m3508set[0].setspeed  = point_Speed[4];
        m3508set[1].setspeed  = point_Speed[4];
        m3508set[2].setspeed  = point_Speed[4];
        m3508set[3].setspeed  = point_Speed[4];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m2006set[1].setpos-m2006[1].real_angle)<100)&&(fabs(m2006set[0].setpos-m2006[0].real_angle)<100)&&(control.s[2]==2||control.s[2]==1))
        {
            Bound_Flag_Point[0]=5;
        }
    }
    else if(Bound_Flag_Point[0]==5&&(control.s[2]==2||control.s[2]==1))
    {
        MOTOR_PID_CHANGE(1);
        if(control.s[2]==3)
            bound_flag_point2=1;

        Leg_Point(0,&link[0],-Height_body_start-50,-20);
        Leg_Point(1,&link[1],-Height_body_start-50,-20);
        Leg_Point(2,&link[2],-Height_body_start-50,-20);
        Leg_Point(3,&link[3],-Height_body_start-50,-20);
        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //后 can2 ID1,2 腿1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;
        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //后 can2 ID3,4 腿2
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //前 can1 ID1,2 腿3
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //前 can1 ID3,4 腿4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[5];
        m2006set[1].setspeed  = point_Speed[5];
        m2006set[2].setspeed  = point_Speed[5];
        m2006set[3].setspeed  = point_Speed[5];
        m3508set[0].setspeed  = point_Speed[5];
        m3508set[1].setspeed  = point_Speed[5];
        m3508set[2].setspeed  = point_Speed[5];
        m3508set[3].setspeed  = point_Speed[5];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m2006set[1].setpos-m2006[1].real_angle)<50)&&(fabs(m2006set[0].setpos-m2006[0].real_angle)<50)&&(control.s[2]==2||control.s[2]==1)) {
            Bound_Flag_Point[0]=6;
        }
    }
		else if(Bound_Flag_Point[0]==6)
		{if(control.s[2]==3)Bound_Flag_Point[0]=1;}

}


/**
* @name 	    BOUND_Inclined_Plane_flat_ground
* @brief  		平跳
* @param    	point_Speed 每个点期望速度
* @param
* @retval
*/
void BOUND_Plane_flat_ground(float Height_body_start,float Height_body_end)
{
    static float M2006_Angle[4]= {0};
    static float M3508_Angle[4]= {0};
    static float point_Speed[6]= {0,15,150,150,150,20};
    static u8 bound_flag_point2=1;
    LCD_Num(50, 220,bound_flag_point2,1, 16);

    /*******************第一个点***********************/
    if(Bound_Flag_Point[1]==1)
    {
        MOTOR_PID_CHANGE(0);
        Leg_Point(0,&link[0],-Height_body_start,-50);
        Leg_Point(1,&link[1],-Height_body_start,-50);
        Leg_Point(2,&link[2],-Height_body_start,-50);
        Leg_Point(3,&link[3],-Height_body_start,-50);

        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //后 can2 ID1,2 腿1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //后 can2 ID3,4 腿3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //前 can1 ID1,2 腿2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //前 can1 ID3,4 腿4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[1];
        m2006set[1].setspeed  = point_Speed[1];
        m2006set[2].setspeed  = point_Speed[1];
        m2006set[3].setspeed  = point_Speed[1];
        m3508set[0].setspeed  = point_Speed[1];
        m3508set[1].setspeed  = point_Speed[1];
        m3508set[2].setspeed  = point_Speed[1];
        m3508set[3].setspeed  = point_Speed[1];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];

        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if(control.s[0]==1)Bound_Flag_Point[1]=2;

    }
		else if(control.s[2]==3)
		{bound_flag_point2=1;}
    /**********************第二个点*********************************/
    else if(Bound_Flag_Point[1]==2&&(control.s[2]==2||control.s[2]==1))
    {
        MOTOR_PID_CHANGE(0);//速度P加大
        if(control.s[2]==3)bound_flag_point2=1;

        Leg_Point(0,&link[0],-Height_body_end,-140);
        Leg_Point(1,&link[1],-Height_body_end,-140);
        Leg_Point(2,&link[2],-Height_body_end,-140);
        Leg_Point(3,&link[3],-Height_body_end,-140);

        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //后 can2 ID1,2 腿1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //后 can2 ID3,4 腿3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //前 can1 ID1,2 腿2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //前 can1 ID3,4 腿4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[2];
        m2006set[1].setspeed  = point_Speed[2];
        m2006set[2].setspeed  = point_Speed[2];
        m2006set[3].setspeed  = point_Speed[2];
        m3508set[0].setspeed  = point_Speed[2];
        m3508set[1].setspeed  = point_Speed[2];
        m3508set[2].setspeed  = point_Speed[2];
        m3508set[3].setspeed  = point_Speed[2];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m2006set[0].setpos-m2006[0].real_angle)<10)&&(fabs(m2006set[1].setpos-m2006[1].real_angle)<10)&&(control.s[2]==2||control.s[2]==1))Bound_Flag_Point[1]=3;
    }
    else if(Bound_Flag_Point[1]==3&&(control.s[2]==2||control.s[2]==1))
    {
        /*****************3点************************/
        MOTOR_PID_CHANGE(0);
        if(control.s[2]==3)bound_flag_point2=1;


        Leg_Point(0,&link[0],-Height_body_start,0);
        Leg_Point(1,&link[1],-Height_body_start,0);
        Leg_Point(2,&link[2],-Height_body_start,0);
        Leg_Point(3,&link[3],-Height_body_start,0);
        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //后 can2 ID1,2 腿1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //后 can2 ID3,4 腿3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //前 can1 ID1,2 腿2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //前 can1 ID3,4 腿4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[3];
        m2006set[1].setspeed  = point_Speed[3];
        m2006set[2].setspeed  = point_Speed[3];
        m2006set[3].setspeed  = point_Speed[3];
        m3508set[0].setspeed  = point_Speed[3];
        m3508set[1].setspeed  = point_Speed[3];
        m3508set[2].setspeed  = point_Speed[3];
        m3508set[3].setspeed  = point_Speed[3];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m3508set[1].setpos-m3508[1].real_angle)<20)&&(fabs(m3508set[0].setpos-m3508[0].real_angle)<20)&&(control.s[2]==2||control.s[2]==1))Bound_Flag_Point[1]=4;

    }
    /**********************第4个点*********************************/
    else if(Bound_Flag_Point[1]==4&&(control.s[2]==2||control.s[2]==1))
    {
        MOTOR_PID_CHANGE(1);
        if(control.s[2]==3)bound_flag_point2=1;

        Leg_Point(0,&link[0],-Height_body_start-60,120);
        Leg_Point(1,&link[1],-Height_body_start-60,120);
        Leg_Point(2,&link[2],-Height_body_start-60,120);
        Leg_Point(3,&link[3],-Height_body_start-60,120);
        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //后 can2 ID1,2 腿1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;

        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //后 can2 ID3,4 腿3
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //前 can1 ID1,2 腿2
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //前 can1 ID3,4 腿4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[4];
        m2006set[1].setspeed  = point_Speed[4];
        m2006set[2].setspeed  = point_Speed[4];
        m2006set[3].setspeed  = point_Speed[4];
        m3508set[0].setspeed  = point_Speed[4];
        m3508set[1].setspeed  = point_Speed[4];
        m3508set[2].setspeed  = point_Speed[4];
        m3508set[3].setspeed  = point_Speed[4];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m2006set[1].setpos-m2006[1].real_angle)<20)&&(fabs(m2006set[0].setpos-m2006[0].real_angle)<20)&&(control.s[2]==2||control.s[2]==1)) {
            Bound_Flag_Point[1]=5;
        }
    }
    else if(Bound_Flag_Point[1]==5&&(control.s[2]==2||control.s[2]==1))
    {
        MOTOR_PID_CHANGE(1);
        if(bound_flag_point2==5&&control.s[2]==3)bound_flag_point2=1;

        Leg_Point(0,&link[0],-Height_body_start-60,-30);
        Leg_Point(1,&link[1],-Height_body_start-60,-30);
        Leg_Point(2,&link[2],-Height_body_start-60,-30);
        Leg_Point(3,&link[3],-Height_body_start-60,-30);
        Inverse_leg(&leg[0],&link[0]);
        Inverse_leg(&leg[1],&link[1]);
        Inverse_leg(&leg[2],&link[2]);
        Inverse_leg(&leg[3],&link[3]);

        M2006_Angle[0]=(RESET_ANGLE+leg[0].real_angle[0])*M3508Reduction_Ratio;	 //后 can2 ID1,2 腿1
        M2006_Angle[1]=(-RESET_ANGLE2-leg[0].real_angle[1])*M3508Reduction_Ratio;
        M2006_Angle[2]=(-RESET_ANGLE-leg[1].real_angle[0])*M3508Reduction_Ratio;	   //后 can2 ID3,4 腿2
        M2006_Angle[3]=(RESET_ANGLE2+leg[1].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[0]=(RESET_ANGLE+leg[2].real_angle[0])*M3508Reduction_Ratio;   //前 can1 ID1,2 腿3
        M3508_Angle[1]=(-RESET_ANGLE2-leg[2].real_angle[1])*M3508Reduction_Ratio;

        M3508_Angle[2]=(-RESET_ANGLE-leg[3].real_angle[0])*M3508Reduction_Ratio;	   //前 can1 ID3,4 腿4
        M3508_Angle[3]=(RESET_ANGLE2+leg[3].real_angle[1])*M3508Reduction_Ratio;

        m2006set[0].setspeed  = point_Speed[5];
        m2006set[1].setspeed  = point_Speed[5];
        m2006set[2].setspeed  = point_Speed[5];
        m2006set[3].setspeed  = point_Speed[5];
        m3508set[0].setspeed  = point_Speed[5];
        m3508set[1].setspeed  = point_Speed[5];
        m3508set[2].setspeed  = point_Speed[5];
        m3508set[3].setspeed  = point_Speed[5];

        m2006set[0].setpos = M2006_Angle[0];
        m2006set[1].setpos = M2006_Angle[1];
        m2006set[2].setpos = M2006_Angle[2];
        m2006set[3].setpos = M2006_Angle[3];
        m3508set[0].setpos = M3508_Angle[0];
        m3508set[1].setpos = M3508_Angle[1];
        m3508set[2].setpos = M3508_Angle[2];
        m3508set[3].setpos = M3508_Angle[3];
        if((fabs(m2006set[1].setpos-m2006[1].real_angle)<50)&&(fabs(m2006set[0].setpos-m2006[0].real_angle)<50)&&(control.s[2]==2||control.s[2]==1)) {
            Bound_Flag_Point[1]=6;
        }
    }
    else if(Bound_Flag_Point[1]==6&&(control.s[2]==2||control.s[2]==1))
    

{if(control.s[2]==3)Bound_Flag_Point[1]=1;}

}



