#include "pid.h"

PID_TypeDef Pid[8];
CascadePID double_pid[2];
void Pid_config(int i,float pid_p,float pid_i,float pid_d)
{
	Pid[i].kp=pid_p;
	Pid[i].ki=pid_i;
	Pid[i].kd=pid_d;
	Pid[i].MaxOutput=13000;
	Pid[i].IntegralLimit=1000;
	Pid[i].DeadBand=10;
	Pid[i].name=inner;
}
void Pid_config_follow(int i,float pid_p,float pid_i,float pid_d)
{
	Pid[i].kp=pid_p;
	Pid[i].ki=pid_i;
	Pid[i].kd=pid_d;
	Pid[i].MaxOutput=2000;
	Pid[i].IntegralLimit=1500;
	Pid[i].DeadBand=0;
	Pid[i].name=inner;
}
void Pid_config_friction(int i,float pid_p,float pid_i,float pid_d)
{
	Pid[i].kp=pid_p;
	Pid[i].ki=pid_i;
	Pid[i].kd=pid_d;
	Pid[i].MaxOutput=13000;
	Pid[i].IntegralLimit=5000;
	Pid[i].DeadBand=10;
	Pid[i].name=inner;
}
void Pid_config_dial(int i,float pid_p,float pid_i,float pid_d)
{
	Pid[i].kp=pid_p;
	Pid[i].ki=pid_i;
	Pid[i].kd=pid_d;
	Pid[i].MaxOutput=13000;
	Pid[i].IntegralLimit=5000;
	Pid[i].DeadBand=10;
	Pid[i].name=inner;
}
void DoublePid_Config(int i,float in_p,float in_i,float in_d,float out_p,float out_i,float out_d)
{
	double_pid[i].inner.kp=in_p;
	double_pid[i].inner.ki=in_i;
	double_pid[i].inner.kd=in_d;
  double_pid[i].inner.MaxOutput=28000;
	double_pid[i].inner.IntegralLimit=20000;
	double_pid[i].inner.DeadBand=0;
	double_pid[i].inner.outer_deadband=0;
	double_pid[i].inner.name=inner;
	double_pid[i].outer.kp=out_p;
	double_pid[i].outer.ki=out_i;
	double_pid[i].outer.kd=out_d;
	double_pid[i].outer.MaxOutput=28000;
	double_pid[i].outer.IntegralLimit=20000;
	double_pid[i].outer.DeadBand=0;
	double_pid[i].outer.outer_deadband=0;
	double_pid[i].outer.name=outer;
}

float	pid_calculate(PID_TypeDef* pid, float measure,float target)
{
	pid->measure = measure;						//实际
	pid->last_err  = pid->err;	//更新前一次误差
	pid->target=target;
	pid->err = pid->target - pid->measure;		 //计算当前误差
	/*云台yaw轴的过零处理*/
	if(pid->name==outer)
	{
		if(pid->err>=90.0f)
		{
			pid->err=pid->err-360.0f;
		}
		if(pid->err<=-90)
		{
			pid->err=pid->err+360.0f;
		}
	}
	pid->last_output = pid->output;
	if((ABS(pid->err) > pid->DeadBand))		//是否进入死区，如果进入则直接跳过，返回上一次的output结果
	{   
		pid->pout = pid->kp * pid->err;			
		pid->iout += (pid->ki * pid->err);			//注意是加等于
		pid->dout =  pid->kd * (pid->err - pid->last_err);    
		//积分是否超出限制
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit; 
		//pid输出和
		pid->output = pid->pout + pid->iout + pid->dout;
		//限制输出的大小
		if(pid->output>pid->MaxOutput)         
		{
			pid->output = pid->MaxOutput;
		}
		if(pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
	}
	return pid->output;
}
//串级pid计算
float PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb)
{
	pid_calculate(&pid->outer,angleFdb,angleRef);//计算外环(角度环)
	pid_calculate(&pid->inner,speedFdb,pid->outer.output);//计算内环(速度环)pid->outer.output
	pid->output=pid->inner.output;
	return pid->inner.output;
}
