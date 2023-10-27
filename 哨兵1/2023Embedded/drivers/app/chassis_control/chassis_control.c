#include "chassis_control.h"
#include "pid.h"
#include "math.h"
#define chassis_mode 2
extern uint16_t actual_ch0;
extern uint16_t actual_ch1;
extern uint16_t actual_ch2;
extern uint16_t actual_ch3;
float angle_correction;
float Vx=0;
float Vy=0;
float W=0;
float Vx_gyro=0;
float Vy_gyro=0;
float W_gyro=0;
float W_follow=0;
extern uint8_t offline_contrl_sign;
extern PID_TypeDef Pid[8];
extern RC_Ctl_t RC_CtrlData;
extern MOTOR_FEEDBACK_ MOTOR_FEEDBACK[4];
signed short int m_1=0, m_2=0, m_3=0, m_4=0;
float expect_speed_1,expect_speed_2,expect_speed_3,expect_speed_4;
extern float Eular[3];
extern CascadePID double_pid[1];
float PI=3.141592;
float Find_Y_AnglePNY(void);
void chassis_follow(void);
void wheel_control(float Vx, float Vy, float W)//麦轮解算
{
	  expect_speed_1 = -(Vy - Vx + W*(wheel_axlespacing +wheel_spacing));
    expect_speed_2 =  (Vy + Vx - W*(wheel_axlespacing +wheel_spacing));
    expect_speed_3 =  (Vy - Vx - W*(wheel_axlespacing +wheel_spacing));
    expect_speed_4 = -(Vy + Vx + W*(wheel_axlespacing +wheel_spacing));
	
	  m_1 = pid_calculate( &Pid[motor_1], MOTOR_FEEDBACK[motor_1].speed_rpm, expect_speed_1);
	  m_2 = pid_calculate( &Pid[motor_2], MOTOR_FEEDBACK[motor_2].speed_rpm, expect_speed_2);
	  m_3 = pid_calculate( &Pid[motor_3], MOTOR_FEEDBACK[motor_3].speed_rpm, expect_speed_3);
	  m_4 = pid_calculate( &Pid[motor_4], MOTOR_FEEDBACK[motor_4].speed_rpm, expect_speed_4);
		Motor_Set_Current(m_1,m_2,m_3,m_4); 
}
/*普通底盘单独控制*/
#if chassis_mode==1
void chassis_control(void)
{
	if(RC_CtrlData.rc.s2==3||offline_contrl_sign==1||RC_CtrlData.rc.s1==3)
	{
		m_1 = pid_calculate( &Pid[motor_1], MOTOR_FEEDBACK[motor_1].speed_rpm, 0);
	  m_2 = pid_calculate( &Pid[motor_2], MOTOR_FEEDBACK[motor_2].speed_rpm, 0);
	  m_3 = pid_calculate( &Pid[motor_3], MOTOR_FEEDBACK[motor_3].speed_rpm, 0);
	  m_4 = pid_calculate( &Pid[motor_4], MOTOR_FEEDBACK[motor_4].speed_rpm, 0);
		Motor_Set_Current(m_1,m_2,m_3,m_4); 
	}
	else if(RC_CtrlData.rc.s2==1)
	{
		if(RC_CtrlData.rc.s1==1)
		{
				m_1 = pid_calculate( &Pid[motor_1], MOTOR_FEEDBACK[motor_1].speed_rpm, 2000);
				m_2 = pid_calculate( &Pid[motor_2], MOTOR_FEEDBACK[motor_2].speed_rpm, 2000);
				m_3 = pid_calculate( &Pid[motor_3], MOTOR_FEEDBACK[motor_3].speed_rpm, 2000);
				m_4 = pid_calculate( &Pid[motor_4], MOTOR_FEEDBACK[motor_4].speed_rpm, 2000);
				Motor_Set_Current(m_1,m_2,m_3,m_4); 
//			m_1 = pid_calculate( &Pid[motor_1], MOTOR_FEEDBACK[motor_1].speed_rpm, 2000);
//			Motor_Set_Current(m_1,0,0,0);
		}
		else if(RC_CtrlData.rc.s1==2)
		{
			m_2 = pid_calculate( &Pid[motor_2], MOTOR_FEEDBACK[motor_2].speed_rpm, 2000);
			Motor_Set_Current(0,m_2,0,0);
		}
	}
	else if(RC_CtrlData.rc.s2==2)
	{
		if(RC_CtrlData.rc.s1==1)
		{
			m_3 = pid_calculate( &Pid[motor_3], MOTOR_FEEDBACK[motor_3].speed_rpm, 2000);
			Motor_Set_Current(0,0,m_3,0);
		}
		else if(RC_CtrlData.rc.s1==2)
		{
			m_4 = pid_calculate( &Pid[motor_4], MOTOR_FEEDBACK[motor_4].speed_rpm, 2000);
			Motor_Set_Current(0,0,0,m_4);
		}
	}
}
#elif chassis_mode==2
void chassis_control(void)
{
	if(RC_CtrlData.rc.s1==1||offline_contrl_sign==1||RC_CtrlData.rc.s1==2)
	{
		Motor_Set_Current(0,0,0,0);
	}
	else if(RC_CtrlData.rc.s1==3)
	{
		if(RC_CtrlData.rc.ch3>=1300)
		{
			Vy=-2000;
		}
		else if(RC_CtrlData.rc.ch3<=748)
		{
			Vy=2000;
		}
		else
		{
			Vy=0;
		}
		if(RC_CtrlData.rc.ch2>=1300)
		{
			Vx=-2000;
		}
		else if(RC_CtrlData.rc.ch2<=748)
		{
			Vx=2000;
		}
		else 
		{
			Vx=0;
		}
		if(RC_CtrlData.rc.s2==1)
		{
			W=0;
		}
		else if(RC_CtrlData.rc.s2==2)
		{
			if(Vx!=0||Vy!=0||Vx!=0||Vy!=0)
			{
				W=1800;
			}
			else if(Vx==0&&Vy==0&&Vx==0&&Vy==0)
			{
				W=3000;
			}
		}
		chassis_gyro();
		chassis_follow();
		if(RC_CtrlData.rc.s2==3||RC_CtrlData.rc.s2==1)
		{
			wheel_control(Vx,Vy,W);
		}
		else if(RC_CtrlData.rc.s2==2)
		{
			wheel_control(Vx_gyro,Vy_gyro,W);
		}
	}
}
#endif
/*小陀螺全向移动函数*/
extern HOLDER_FEEDBACK_ HOLDER_FEEDBACK[2];
float Eular_Yaw;
void chassis_gyro(void)
{
	float relative_angle;
	relative_angle=Find_Y_AnglePNY();
	float sin_yaw = 0.0f, cos_yaw = 0.0f;
	relative_angle=relative_angle/8192.0f*360.0f *PI/180.0f;
  sin_yaw = sin(relative_angle);
	cos_yaw = cos(relative_angle);
	Vx_gyro =  cos_yaw * Vx - sin_yaw * Vy;
	Vy_gyro =  sin_yaw * Vx + cos_yaw * Vy;
}
/*底盘跟随*/
void chassis_follow(void)
{
	
	if(RC_CtrlData.rc.s1==1||offline_contrl_sign==1||RC_CtrlData.rc.s2==2)
	{
		W_follow=0;
	}
	else if(RC_CtrlData.rc.s1==3&&RC_CtrlData.rc.s2==3)
	{
		
		float chassis_relative_angle;
		chassis_relative_angle = Find_Y_AnglePNY();
		W=pid_calculate(&Pid[follow],0,chassis_relative_angle);
	}
}

/*yaw轴相对角度获取*/
float Find_Y_AnglePNY(void)
{
	  float mintemp1;
    float temp1 = HOLDER_FEEDBACK[holder_Yaw].angle_value - chassis_positive_direction;
    float temp2 = HOLDER_FEEDBACK[holder_Yaw].angle_value - chassis_positive_direction;
    //float mintemp1;
    if(temp1 > 4096)
        temp1 -= 8192;
    else if(temp1 < -4096)
        temp1 += 8192;
    if(temp2 > 4096)
        temp2 -= 8192;
    else if(temp2 < -4096)
        temp2 += 8192;
    mintemp1 = (ABS((int32_t)temp1) < ABS((int32_t)temp2) ? temp1 : temp2);
		if(fabs(mintemp1)<0)
			 mintemp1=0;
    return mintemp1;
}












