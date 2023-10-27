#ifndef __PID_H
#define __PID_H
#include "stm32f4xx.h"
#define ABS(x)		((x>0)? (x): (-x))
typedef struct _PID_TypeDef
{
	float target;							//Ŀ��ֵ
	float kp;								//����ϵ��
	float ki;								//����ϵ��
	float kd;								//΢��ϵ��
	
	float   measure;						//����ֵ
	float   err;							//���
	float   last_err;      	      			//�ϴ����
	float   last_last_err;
	
	float pout;								//������
	float iout;								//������
	float dout;								//΢����
	
	float output;							//�������
	float last_output;						//�ϴ����
	
	float MaxOutput;						//����޷�
	float IntegralLimit;					//�����޷�
	float DeadBand;						    //����������ֵ��
	
	float outer_deadband;
	uint8_t name;
}PID_TypeDef;
enum pid_name
{
	inner  =0,
	outer  =1,

};
//����pid
typedef struct _CascadePID
{
	PID_TypeDef inner;//�ڻ�
	PID_TypeDef outer;//�⻷
	float output;//�������������inner.output
}CascadePID;

void Pid_config(int i,float pid_p,float pid_i,float pid_d);
float	pid_calculate(PID_TypeDef* pid, float measure,float target);
float PID_CascadeCalc(CascadePID *pid,float angleRef,float angleFdb,float speedFdb);
void DoublePid_Config(int i,float in_p,float in_i,float in_d,float out_p,float out_i,float out_d);
void Pid_config_follow(int i,float pid_p,float pid_i,float pid_d);
void Pid_config_friction(int i,float pid_p,float pid_i,float pid_d);
void Pid_config_dial(int i,float pid_p,float pid_i,float pid_d);
#endif
