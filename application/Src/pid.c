#include "pid.h"


//用于6020,1:1减速比的过零处理
fp32 rad_format(fp32 rad)
{
	if(rad>180)
	{
		while(rad>180)
		{
			rad=rad-2*180;
		}
	}
	else if(rad<-180)
	{
		while(rad<-180)
		{
			rad=rad+2*180;
		}
	}
	return rad;
}

//pid初始化
void PID_init(pid_type_def *pid,const fp32 PID[3],fp32 max_pout,fp32 max_iout,fp32 max_out)
{
	pid->kp=PID[0];
	pid->ki=PID[1];
	pid->kd=PID[2];
	pid->max_pout=max_pout;
	pid->max_iout=max_iout;
	pid->max_out=max_out;
	pid->dbuf[0]=pid->dbuf[1]=pid->dbuf[2]=0.0f;
  pid->error[0]=pid->error[1]=pid->error[2]=pid->pout=pid->iout=pid->dout=pid->out=0.0f;
}

//位置式pid
fp32 PID_calc(pid_type_def *pid,fp32 fdb,fp32 set)//当前值，目标值
{
	pid->fdb=fdb;//当前值
	pid->set=set;//目标值
	pid->error[1]=pid->error[0];
	pid->error[0]=(set-fdb);//计算误差
	
	pid->pout=pid->kp*pid->error[0];
	LimitMax(&pid->iout,pid->max_pout);//pid比例限幅
	
	pid->iout+=pid->ki*pid->error[0];
	LimitMax(&pid->iout,pid->max_iout);//pid积分限幅
	
	pid->dout=pid->kd*(pid->error[0]-pid->error[1]);
	
	pid->out=pid->pout+pid->iout+pid->dout;
	
	LimitMax(&pid->out,pid->max_out);//pid微分限幅
	
	return pid->out;
}

//带零点处理的位置式pid  
fp32 PID_calc_angle(pid_type_def *pid,fp32 fdb,fp32 set)//当前值，目标值
{
	pid->fdb=fdb;//当前值
	pid->set=set;//目标值
	pid->error[1]=pid->error[0];
	pid->error[0]=rad_format(set-fdb);//计算误差
	
	pid->pout=pid->kp*pid->error[0];
	pid->iout+=pid->ki*pid->error[0];
	
	LimitMax(&pid->iout,pid->max_iout);//pid积分限幅
	
	pid->dout=pid->kd*(pid->error[0]-pid->error[1]);
	
	pid->out=pid->pout+pid->iout+pid->dout;
	
	LimitMax(&pid->out,pid->max_out);//pid微分限幅
	
	return pid->out;
}

//PI
//平衡车学来的控制方法
//传入角度值和角速度值
fp32 PI_postion(pid_type_def *pid,fp32 pitch,fp32 gyro_y)
{
	pid->fdb=pitch;
	pid->set=gimbal_balance;//机械中值
	pid->error[1]=pid->error[0];
	pid->error[0]=pid->set-pid->fdb;
	
	pid->pout=pid->kp*pid->error[0];
	pid->dout=pid->kd*gyro_y;

	//目标值减去机械中值（不一定为0）
	pid->out=pid->pout+pid->dout;

	return pid->out;
}


