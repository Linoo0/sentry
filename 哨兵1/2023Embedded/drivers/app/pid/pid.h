#ifndef __PID_H
#define __PID_H
#include "stm32f4xx.h"
#define ABS(x)		((x>0)? (x): (-x))
typedef struct _PID_TypeDef
{
	float target;							//目标值
	float kp;								//比例系数
	float ki;								//积分系数
	float kd;								//微分系数
	
	float   measure;						//测量值
	float   err;							//误差
	float   last_err;      	      			//上次误差
	float   last_last_err;
	
	float pout;								//比例项
	float iout;								//积分项
	float dout;								//微分项
	
	float output;							//本次输出
	float last_output;						//上次输出
	
	float MaxOutput;						//输出限幅
	float IntegralLimit;					//积分限幅
	float DeadBand;						    //死区（绝对值）
	
	float outer_deadband;
	uint8_t name;
}PID_TypeDef;
enum pid_name
{
	inner  =0,
	outer  =1,

};
//串级pid
typedef struct _CascadePID
{
	PID_TypeDef inner;//内环
	PID_TypeDef outer;//外环
	float output;//串级输出，等于inner.output
}CascadePID;

void Pid_config(int i,float pid_p,float pid_i,float pid_d);
float	pid_calculate(PID_TypeDef* pid, float measure,float target);
float PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb);
void DoublePid_Config(int i,float in_p,float in_i,float in_d,float out_p,float out_i,float out_d);
void Pid_config_follow(int i,float pid_p,float pid_i,float pid_d);
void Pid_config_friction(int i,float pid_p,float pid_i,float pid_d);
void Pid_config_dial(int i,float pid_p,float pid_i,float pid_d);
#endif
