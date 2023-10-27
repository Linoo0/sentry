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
	pid->measure = measure;						//ʵ��
	pid->last_err  = pid->err;	//����ǰһ�����
	pid->target=target;
	pid->err = pid->target - pid->measure;		 //���㵱ǰ���
	/*��̨yaw��Ĺ��㴦��*/
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
	if((ABS(pid->err) > pid->DeadBand))		//�Ƿ�������������������ֱ��������������һ�ε�output���
	{   
		pid->pout = pid->kp * pid->err;			
		pid->iout += (pid->ki * pid->err);			//ע���Ǽӵ���
		pid->dout =  pid->kd * (pid->err - pid->last_err);    
		//�����Ƿ񳬳�����
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit; 
		//pid�����
		pid->output = pid->pout + pid->iout + pid->dout;
		//��������Ĵ�С
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
//����pid����
float PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb)
{
	pid_calculate(&pid->outer,angleFdb,angleRef);//�����⻷(�ǶȻ�)
	pid_calculate(&pid->inner,speedFdb,pid->outer.output);//�����ڻ�(�ٶȻ�)pid->outer.output
	pid->output=pid->inner.output;
	return pid->inner.output;
}
