#include "pid_config.h"
#include "pid.h"
#include "can_receive.h"
void pid_parameter_setting(void)
{
	/*���̵������*/
	Pid_config(motor_1,9  ,0.01 ,0);            //���1����
	Pid_config(motor_2,7  ,0.005,0);            //���2����
	Pid_config(motor_3,6.8,0.01 ,0);            //���3����
	Pid_config(motor_4,7  ,0.01 ,0);            //���4����
	
	Pid_config_follow(follow,5,0,0);//���̸���pid����
	
	/*��̨������ã���NUC*/
	DoublePid_Config(holder_Yaw  ,580,0.05,0,  10,0,0);       //yaw��������
	DoublePid_Config(holder_Pitch, 355,0.08,0,   14,0,0);        //pitch��������
	
	/*Ħ���ֵ������*/
	Pid_config_friction(friction_left ,3,0.02,0);
	Pid_config_friction(friction_right,3,0.02,0);
	
	/*���̵������*/
	Pid_config_dial(dial,3.5,0,0);
}




























