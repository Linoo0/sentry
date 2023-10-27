#include "pid_config.h"
#include "pid.h"
#include "can_receive.h"
void pid_parameter_setting(void)
{
	/*底盘电机控制*/
	Pid_config(motor_1,9  ,0.01 ,0);            //电机1配置
	Pid_config(motor_2,7  ,0.005,0);            //电机2配置
	Pid_config(motor_3,6.8,0.01 ,0);            //电机3配置
	Pid_config(motor_4,7  ,0.01 ,0);            //电机4配置
	
	Pid_config_follow(follow,5,0,0);//底盘跟随pid配置
	
	/*云台电机配置，带NUC*/
	DoublePid_Config(holder_Yaw  ,350,0.05,0,  10,0,0);       //580
	DoublePid_Config(holder_Pitch, 200,0.08,0,   14,0,0);        //355
	
	/*摩擦轮电机配置*/
	Pid_config_friction(friction_left ,3,0.02,0);
	Pid_config_friction(friction_right,3,0.02,0);
	
	/*拨盘电机配置*/
	Pid_config_dial(dial,3.5,0,0);
}

/*裸云台
DoublePid_Config(holder_Yaw  ,300,0,0,  11,0,0); 
DoublePid_Config(holder_Pitch, 260,0.08,0,   14,0,0); 
*/


























