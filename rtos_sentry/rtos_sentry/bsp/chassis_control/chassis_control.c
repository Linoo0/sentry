#include "chassis_control.h"
#include "PTZ_Communication.h"
#include "pid.h"
#include "math.h"
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
//#if ROBOT_MODE==1
void chassis_control(void)
{
	if(RC_CtrlData.rc.s1==1||offline_contrl_sign==1||RC_CtrlData.rc.s1==2)
	{
		Motor_Set_Current(0,0,0,0);
	}
	else if(RC_CtrlData.rc.s1==3)
	{
		if(RC_CtrlData.rc.ch3>=1224)
		{
			Vy=1000+(RC_CtrlData.rc.ch3-1224)*6;
		}
		else if(RC_CtrlData.rc.ch3<=824)
		{
			Vy=-1000-(824-RC_CtrlData.rc.ch3)*6;
		}
		else
		{
			Vy=0;
		}
		if(RC_CtrlData.rc.ch2>=1224)
		{
			Vx=1000+(RC_CtrlData.rc.ch2-1224)*6;
		}
		else if(RC_CtrlData.rc.ch2<=824)
		{
			Vx=-1000-(824-RC_CtrlData.rc.ch2)*6;
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
				W=1500;
			}
			else if(Vx==0&&Vy==0&&Vx==0&&Vy==0)
			{
				W=2300;
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
//#elif ROBOT_MODE==2
float Vx_t=0;
float Vy_t=0;
static uint8_t stopgyro_sign=0,stopgyro_sign_num=0,stopgyro_sign_sign=0;
static uint8_t tanslation_sign_num_L=0,tanslation_sign_num_R=0,tanslation_sign=0;
static uint8_t tanslation_first=0,tanslation_sign_num_L_stop=0,stop_tanslation=0;
static uint8_t gyro_mode=0,tanslation_mode=0;
static uint8_t tanslation_GO_STOP=0,tanslation_GO_STOP_sign=0;
 
 void gyro_reset(void)
{
	stopgyro_sign=0;stopgyro_sign_num=0;stopgyro_sign_sign=0;
	
}

void translation_reset(void)
{
	 tanslation_sign_num_L=0;tanslation_sign_num_R=0;tanslation_sign=0;stop_tanslation=0;tanslation_GO_STOP=0;tanslation_GO_STOP_sign=0;
}
extern uint8_t Injury_sign;
void chassis_control_sentry(void)
{
	if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1&&RC_CtrlData.rc.s2==1&&Injury_sign==1) //原地模式受打击自转
	{
		//translation_reset();
		gyro_mode=1;
//		tanslation_mode=0;
//		tanslation_sign_num_L_stop=0;
//		tanslation_first=1;
		stopgyro_sign=1;
		sentry_gyro_mode();
	}
	else if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1&&RC_CtrlData.rc.s2==1&&Injury_sign==0)
	{
		/*小陀螺停止*/
		if(gyro_mode==1)
		{
			if(stopgyro_sign==1)
			{
				stopgyro_sign_sign=1;
				stopgyro_sign_num=0;
			}
			if(stopgyro_sign_num<2&&stopgyro_sign_sign==1)
			{
				sentry_gyro_mode();
				stopgyro_sign=0;
			}
			if(stopgyro_sign_num>=2)
			{
				W=0;
				wheel_control(0,0,0);
				stopgyro_sign_sign=0;
			}
		}
		if(gyro_mode==0)
		{
			wheel_control(0,0,0);
		}
	}
  else if(RC_CtrlData.rc.s1==3&&offline_contrl_sign!=1)
	{
		chassis_adjust();
		float chassis_relative_angle;
		chassis_relative_angle = Find_Y_AnglePNY();
		W=pid_calculate(&Pid[follow],0,chassis_relative_angle);
		wheel_control(Vx,Vy,W);
	}
	else if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1&&RC_CtrlData.rc.s2==3) //停止
	{
//		tanslation_first=1;
//		/*小陀螺停止*/
//		if(gyro_mode==1)
//		{
//			if(stopgyro_sign==1)
//			{
//				stopgyro_sign_sign=1;
//				stopgyro_sign_num=0;
//			}
//			if(stopgyro_sign_num<2&&stopgyro_sign_sign==1)
//			{
//				sentry_gyro_mode();
//				stopgyro_sign=0;
//			}
//			if(stopgyro_sign_num>=2)
//			{
//				W=0;
//				wheel_control(0,0,0);
//				stopgyro_sign_sign=0;
//			}
//		}
//    /*平移停止*/
//		if(tanslation_mode==1)
//		{
//			if(tanslation_sign==1)
//			{
//				stop_tanslation=1;
//			}			
//			if(tanslation_sign_num_L_stop==1)
//			{
//				translation_reset();
//				Vx_t=0;
//				Vy_t=0;
//				wheel_control(0,0,0);
//				tanslation_sign=0;
//				tanslation_sign_num_L=0;
//				tanslation_sign_num_R=0;
//			}
//			if(tanslation_sign_num_L_stop==0&&stop_tanslation==1)
//			{
//				sentry_translation_mode();
//			}
//		}
//		if(tanslation_mode==0&&gyro_mode==0)
//		{
			wheel_control(0,0,0);
//		}
	}
	else if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1&&RC_CtrlData.rc.s2==2) //原地模式
	{
//		gyro_mode=1;
//		stopgyro_sign=1;
		
		sentry_gyro_mode();
//		float chassis_relative_angle;
//		chassis_relative_angle = Find_Y_AnglePNY();
//		W=pid_calculate(&Pid[follow],0,chassis_relative_angle);
//		wheel_control(0,0,W);
		
		/*平移模式代码*/
//		gyro_reset();
//		gyro_mode=0;
//		tanslation_mode=1;
//		tanslation_sign_num_L_stop=0;
//		tanslation_sign=1;
//		sentry_translation_mode();
	}
	else
	{
		gyro_reset();
		translation_reset();
		Motor_Set_Current(0,0,0,0);
	}
}
/*小陀螺停止标志位*/
void gyro_stop(void)
{
	if(stopgyro_sign_sign==1)
	{
		stopgyro_sign_num++;
	}
}
/*小陀螺模式1：固定小陀螺*/
void sentry_gyro_mode(void)
{
	W=2300;
	Vy=-10;      //2500小陀螺补偿   10   -3
	Vx=3;
	chassis_gyro();
	wheel_control(Vx_gyro,Vy_gyro,W);
}
void sentry_gyro_mode_1(void)
{
	W=1800;
	Vy=-7;      //1800小陀螺补偿   7   3
	Vx=3;
	chassis_gyro();
	wheel_control(Vx_gyro,Vy_gyro,W);
}

/*平移模式云台左限位操作*/
void translation_L(void)
{
	if(tanslation_sign==1)
	{
		if(tanslation_first==1)
		{
			tanslation_sign_num_L=1;
			tanslation_first=0;
		}
	}
	if(stop_tanslation==1)
	{
		stop_tanslation=0;
		tanslation_sign_num_L_stop=1;
	}
	else 
	{
		tanslation_sign_num_L_stop=0;
	}
}
/*平移模式云台右限位操作*/
void translation_R(void)
{
	if(tanslation_sign==1)
	{
		if(tanslation_sign_num_L==1)
		{
			tanslation_sign_num_L=2;
		}
		if(tanslation_sign_num_L==2)
		{
			if(tanslation_sign_num_R==0)
			{
				tanslation_sign_num_R=1;
			}
			else if(tanslation_sign_num_R==1)
			{
				tanslation_sign_num_R=0;
			}
		}
	}
}
/*哨兵模式底盘平移模式*/
void sentry_translation_mode(void)
{
	if(tanslation_sign_num_L==1)
	{
		Vx_t=0;
		Vy_t=1500;
	}
	if(tanslation_sign_num_R==0&&tanslation_sign_num_L==2&&tanslation_GO_STOP==0)
	{
		Vx_t=0;
		Vy_t=1500;
		tanslation_GO_STOP_sign=0;
	}
	else if(tanslation_sign_num_R==0&&tanslation_sign_num_L==2&&tanslation_GO_STOP==1)
	{
		Vx_t=0;
		Vy_t=-1500;
		tanslation_GO_STOP_sign=1;
	}
	if(tanslation_sign_num_R==1&&tanslation_sign_num_L==2)
	{
		Vx_t=0;
		Vy_t=0;
	}
	wheel_control(Vx_t,Vy_t,0);
}
/*平移模式停止与移动标志位操作*/
void Translation_G_S(void)
{
	if(tanslation_sign_num_R==0&&tanslation_sign_num_L==2)
	{
		if(tanslation_GO_STOP_sign==0)
		{
			tanslation_GO_STOP=1;
		}
		else if(tanslation_GO_STOP_sign==1)
		{
			tanslation_GO_STOP=0;
		}
  }
}
/*哨兵模式底盘控制*/
void chassis_adjust(void)
{
	if(RC_CtrlData.rc.ch3>=1224)
	{
		Vy=1500;
	}
	else if(RC_CtrlData.rc.ch3<=824)
	{
		Vy= -1500;
	}
	else
	{
		Vy=0;
	}
	if(RC_CtrlData.rc.ch2>=1224)
	{
		Vx=1500;
	}
	else if(RC_CtrlData.rc.ch2<=824)
	{
		Vx=-1500;
	}
	else 
	{
		Vx=0;
	}
	
}

//#endif
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
/*底盘跟随转45度*/
float Find_Y_AnglePNY_atuo_mode(void)
{
	  float mintemp1;
    float temp1 = HOLDER_FEEDBACK[holder_Yaw].angle_value - 5458;
    float temp2 = HOLDER_FEEDBACK[holder_Yaw].angle_value - 5458;
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

/*三角函数小陀螺速度计算*/
 uint16_t gyro_speed_num=150;
void gyro_speed_calculation(void)
{
	gyro_speed_num++;
	W=400*sin(0.03*gyro_speed_num)+2500;
	if(gyro_speed_num>=6666)
	{
		gyro_speed_num=150;
	}
}
/*小陀螺模式2：三角函数小陀螺*/
 uint8_t gyro_mode_2_num;
void sentry_gyro_mode_2(void)
{
	if(gyro_mode_2_num>=8)
	{
	  gyro_speed_calculation();
		gyro_mode_2_num=0;
	}
	Vy=-10;
	chassis_gyro();
	wheel_control(Vx_gyro,Vy_gyro,W);
	gyro_mode_2_num++;
}
extern uint8_t PTZ_sentry_mode;
void PTZ_chassis_control(void)
{
	if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1&&RC_CtrlData.rc.s2==1)  //遥控器切换哨兵模式
	{
		PTZ_Control();
		if(PTZ_sentry_mode==0)
		{
			float chassis_relative_angle;
			chassis_relative_angle = Find_Y_AnglePNY();
			W=pid_calculate(&Pid[follow],0,chassis_relative_angle);
			wheel_control(Vx,Vy,W);
		}
		else if(PTZ_sentry_mode==1)
		{
			if(Injury_sign==1)
			{
				gyro_mode=1;
				stopgyro_sign=1;
				sentry_gyro_mode();
			}
			else if(Injury_sign==0)
			{
				/*小陀螺停止*/
				if(gyro_mode==1)
				{
					if(stopgyro_sign==1)
					{
						stopgyro_sign_sign=1;
						stopgyro_sign_num=0;
					}
					if(stopgyro_sign_num<2&&stopgyro_sign_sign==1)
					{
						sentry_gyro_mode();
						stopgyro_sign=0;
					}
					if(stopgyro_sign_num>=2)
					{
						W=0;
						wheel_control(0,0,0);
						stopgyro_sign_sign=0;
					}
				}
				if(gyro_mode==0)
				{
					wheel_control(0,0,0);
				}
			}
		}
	}
	else if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1&&RC_CtrlData.rc.s2==2)
	{
		PTZ_Control();
		if(PTZ_sentry_mode==0)
		{
			float chassis_relative_angle;
			chassis_relative_angle = Find_Y_AnglePNY();
			W=pid_calculate(&Pid[follow],0,chassis_relative_angle);
			wheel_control(Vx,Vy,W);
		}
		else if(PTZ_sentry_mode==1)
		{
			if(((1<<10)&REF_Event())==0) //前哨战被击毁
			{
				sentry_gyro_mode_1();
			}
			else
			{
				wheel_control(0,0,0);
			}
		}
	}
  else if(RC_CtrlData.rc.s1==3&&offline_contrl_sign!=1)  //底盘修正控制
	{
		chassis_adjust();
		float chassis_relative_angle;
		chassis_relative_angle = Find_Y_AnglePNY();
		W=pid_calculate(&Pid[follow],0,chassis_relative_angle);
		wheel_control(Vx,Vy,W);
	}
	else if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1&&RC_CtrlData.rc.s2==3) //停止
	{
		float chassis_relative_angle;
		chassis_relative_angle = Find_Y_AnglePNY();
		W=pid_calculate(&Pid[follow],0,chassis_relative_angle);
		wheel_control(0,0,W);
	}
	else
	{
		gyro_reset();
		Motor_Set_Current(0,0,0,0);
	}
}

uint8_t sentry_go_sign=0;
extern uint8_t start;
void chassis_control_sentry_gogogo(void)
{
	REF_STATE();
	if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1&&RC_CtrlData.rc.s2==1&&Injury_sign==1) //原地模式受打击自转
	{
		gyro_mode=1;
		stopgyro_sign=1;
		sentry_gyro_mode();
	}
	else if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1&&RC_CtrlData.rc.s2==1&&Injury_sign==0)
	{
		
		/*小陀螺停止*/
		if(gyro_mode==1)
		{
			if(stopgyro_sign==1)
			{
				stopgyro_sign_sign=1;
				stopgyro_sign_num=0;
			}
			if(stopgyro_sign_num<2&&stopgyro_sign_sign==1)
			{
				sentry_gyro_mode();
				stopgyro_sign=0;
			}
			if(stopgyro_sign_num>=2)
			{
				W=0;
				wheel_control(0,0,0);
				stopgyro_sign_sign=0;
			}
		}
		if(gyro_mode==0)
		{
			wheel_control(0,0,0);
		}
	}
  else if(RC_CtrlData.rc.s1==3&&offline_contrl_sign!=1)
	{
		chassis_adjust();
		float chassis_relative_angle;
		chassis_relative_angle = Find_Y_AnglePNY();
		W=pid_calculate(&Pid[follow],0,chassis_relative_angle);
		wheel_control(Vx,Vy,W);
	}
	else if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1&&RC_CtrlData.rc.s2==3) //停止
	{
		  
			wheel_control(0,0,0);
	}
	else if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1&&RC_CtrlData.rc.s2==2) //原地模式
	{
		if(start==1)
		{
			sentry_start_gogogo();
			if(sentry_go_sign==1)
			{
				sentry_gyro_mode();
//				float chassis_relative_angle;
//				chassis_relative_angle = Find_Y_AnglePNY();
//				W=pid_calculate(&Pid[follow],0,chassis_relative_angle);
//				wheel_control(0,0,W);
				
			}
		}
		else
		{
			sentry_go_sign=0;
			wheel_control(0,0,0);
		}
	}
	else
	{
		gyro_reset();
		translation_reset();
		Motor_Set_Current(0,0,0,0);
	}
}

uint16_t go_num;
void sentry_start_gogogo(void)
{
	if(sentry_go_sign==0)
	{
		Vx=0;
		Vy=-1000;
		go_num++;
		if(go_num>=333*3)
		{
			go_num=0;
			Vx=Vy=0;
			sentry_go_sign=1;
		}
		wheel_control(Vx,Vy,0);
	}
}






