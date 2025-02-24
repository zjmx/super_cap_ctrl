#ifndef __PID_H
#define __PID_H

#include "struct_typedef.h"
#include "main.h"

#define PI 3.1415926358979f
#define gimbal_balance 0//云台机械平衡点

//创建PID结构体
typedef struct
{
	//PID控制参数
	fp32 kp;
	fp32 ki;
	fp32 kd;
	
	//误差
	fp32 dbuf[3];  //微分项 0最新 1上一次 2上上次
  fp32 error[3]; //误差项 0最新 1上一次 2上上次
	
	//PID限幅
	fp32 max_pout;
	fp32 max_iout;
	fp32 max_dout;
	fp32 max_out;
	
	//目标量与当前量
	fp32 set;
	fp32 fdb;
	
	//PID输出
	fp32 out;
	fp32 pout;
	fp32 iout;
	fp32 dout;

	//双环PID参数
	float angle_ref;
	float angle_fdb;
	float speed_ref;
	float speed_fdb;

	//直立环参数
	fp32 Angle_pitch;
	fp32 gyro_y;

}pid_type_def;

void LimitMax(fp32 *input,fp32 max);
fp32 rad_format_MF4010(fp32 rad);
void PID_init(pid_type_def *pid,const fp32 PID[3],fp32 max_pout,fp32 max_iout,fp32 max_out);
fp32 PID_calc(pid_type_def *pid,fp32 ref,fp32 set);
fp32 PID_calc_angle(pid_type_def *pid,fp32 fdb,fp32 set);//当前值，目标值
fp32 PID_calc_application(pid_type_def *pid,fp32 ref,fp32 set,fp32 speed);//当前值，目标值


fp32 PI_postion(pid_type_def *pid,fp32 pitch,fp32 gyro_y);//PI角度环

#endif
