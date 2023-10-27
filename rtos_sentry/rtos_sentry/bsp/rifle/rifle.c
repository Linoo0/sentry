#include "rifle.h"
#include "can_receive.h"
#include "bsp_rc.h"
/*6000:speed 24*/
signed short int r_1=0,r_2=0,d_1=0;
extern PID_TypeDef Pid[8];
extern RC_Ctl_t RC_CtrlData;
extern uint8_t offline_contrl_sign;
extern FRICTION_WHEEL_ FRICTION_WHEEL[2];
extern DIAL_MOTOR_ DIAL_MOTOR;
extern signed short int h_1,h_2;
uint8_t launch_sign;
//#if ROBOT_MODE==1
void rifle_control(void)
{
	if(offline_contrl_sign==1||RC_CtrlData.rc.s1==1)
	{
		d_1=0;
		r_1=0,r_2=0,
		Friction_Set_Current(0,0);
		CAN2_Set_Current(0,0,0,0);
	}
	else if(RC_CtrlData.rc.s1==3||RC_CtrlData.rc.s2==1)
	{
		r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , 0);
		r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm, 0);
		Friction_Set_Current(r_1,r_2);
	}
	else if(RC_CtrlData.rc.s1==2&&RC_CtrlData.rc.s2==3)
	{
		r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , -7000);
		r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm,  7000);
		if(RC_CtrlData.rc.ch3<=500)
		{
		  d_1=pid_calculate( &Pid[dial], DIAL_MOTOR.speed_rpm,  3000);
		}
		else if(RC_CtrlData.rc.ch3>=1500)
		{
			d_1=pid_calculate( &Pid[dial], DIAL_MOTOR.speed_rpm, -3000);
		}
		else
		{
			d_1=0;
		}
		Friction_Set_Current(r_1,r_2);
		CAN2_Set_Current(0,h_2,d_1,0); 
	}
}
//#elif ROBOT_MODE==2
extern uint8_t visual_sign;
extern uint8_t PTZ_sentry_mode;
extern uint8_t PTZ_rifle_sign;
void PTZ_rifle_control(void)
{
	//Bounce_Amount_Judgment();
	launch_sign=1;
	if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1)
	{
		if(launch_sign==1)
		{
			r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , -7000);
			r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm,  7000);
			Friction_Set_Current(r_1,r_2);
			if(((visual_sign==1&&RC_CtrlData.rc.s2==1)||(visual_sign==1&&RC_CtrlData.rc.s2==2))&&PTZ_sentry_mode==1)
			{
				uint16_t Cooling_Limit; //枪口热量上限
				uint16_t Real_Heat;     //枪口实时热量
				uint16_t Real_Heat_2;
				Cooling_Limit=REF_Report_Shoot_CoolingLimit()-40;
				Real_Heat=REF_Report_Shooter_Heat();
				Real_Heat_2=REF_Report_Shooter_Heat_2();
				if(Cooling_Limit==0||(Real_Heat<Cooling_Limit&&Real_Heat_2<Cooling_Limit))
				{
					d_1=pid_calculate( &Pid[dial], DIAL_MOTOR.speed_rpm,  4500);
				}
				else if(Real_Heat>=Cooling_Limit||Real_Heat_2>=Cooling_Limit)
				{
					d_1=pid_calculate( &Pid[dial], DIAL_MOTOR.speed_rpm,  0);
				}
			}
			else if((visual_sign==0||PTZ_sentry_mode==0)&&PTZ_rifle_sign==0)
			{
				d_1=0;
			}
			else if(PTZ_sentry_mode==0&&PTZ_rifle_sign==1)
			{
				d_1=pid_calculate( &Pid[dial], DIAL_MOTOR.speed_rpm,  -3000);
			}
			CAN2_Set_Current(0,h_2,d_1,0); 
  	}
		else if(launch_sign==0)
		{
			d_1=0;
			CAN2_Set_Current(0,h_2,0,0);
			r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , 0);
			r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm, 0);
			Friction_Set_Current(0,0);
		}
	}
	else if(RC_CtrlData.rc.s1==3&&offline_contrl_sign!=1)
	{
		d_1=0;
		CAN2_Set_Current(0,h_2,0,0);
		r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , 0);
		r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm, 0);
		Friction_Set_Current(r_1,r_2);
	}
	else
	{
		d_1=0;
		r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , 0);
		r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm, 0);
		Friction_Set_Current(0,0);
		CAN2_Set_Current(0,0,0,0); 
	}
}
void rifle_control_sentry(void)
{
	//Bounce_Amount_Judgment();
	launch_sign=1;
	if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1)
	{
		if(launch_sign==1)
		{
			r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , -7000);
			r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm,  7000);
			Friction_Set_Current(r_1,r_2);
			if((visual_sign==1&&RC_CtrlData.rc.s2==1)||(visual_sign==1&&RC_CtrlData.rc.s2==2))
			{
				uint16_t Cooling_Limit; //枪口热量上限
				uint16_t Real_Heat;     //枪口实时热量
				uint16_t Real_Heat_2;
				Cooling_Limit=REF_Report_Shoot_CoolingLimit()-40;
				Real_Heat=REF_Report_Shooter_Heat();
				Real_Heat_2=REF_Report_Shooter_Heat_2();
				if(Cooling_Limit==0||(Real_Heat<Cooling_Limit&&Real_Heat_2<Cooling_Limit))
				{
					d_1=pid_calculate( &Pid[dial], DIAL_MOTOR.speed_rpm,  4500);
				}
				else if(Real_Heat>=Cooling_Limit||Real_Heat_2>=Cooling_Limit)
				{
					d_1=pid_calculate( &Pid[dial], DIAL_MOTOR.speed_rpm,  0);
				}
			}
			else if(visual_sign==0||PTZ_sentry_mode==0)
			{
				d_1=0;
			}
			CAN2_Set_Current(0,h_2,d_1,0); 
  	}
		else if(launch_sign==0)
		{
			d_1=0;
			CAN2_Set_Current(0,h_2,0,0);
			r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , 0);
			r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm, 0);
			Friction_Set_Current(0,0);
		}
	}
	else if(RC_CtrlData.rc.s1==3&&offline_contrl_sign!=1)
	{
		d_1=0;
		CAN2_Set_Current(0,h_2,0,0);
		r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , 0);
		r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm, 0);
		Friction_Set_Current(r_1,r_2);
	}
	else
	{
		d_1=0;
		r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , 0);
		r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm, 0);
		Friction_Set_Current(0,0);
		CAN2_Set_Current(0,0,0,0); 
	}
}
extern uint8_t sentry_go_sign;
void rifle_control_gogogo(void)
{
	//Bounce_Amount_Judgment();
	launch_sign=1;
	if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1)
	{
		if(launch_sign==1)
		{
			r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , -7000);
			r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm,  7000);
			Friction_Set_Current(r_1,r_2);
			if(((visual_sign==1&&RC_CtrlData.rc.s2==1)||(visual_sign==1&&RC_CtrlData.rc.s2==2))&&sentry_go_sign==1)
			{
				uint16_t Cooling_Limit; //枪口热量上限
				uint16_t Real_Heat;     //枪口实时热量
				uint16_t Real_Heat_2;
				Cooling_Limit=REF_Report_Shoot_CoolingLimit()-40;
				Real_Heat=REF_Report_Shooter_Heat();
				Real_Heat_2=REF_Report_Shooter_Heat_2();
				if(Cooling_Limit==0||(Real_Heat<Cooling_Limit&&Real_Heat_2<Cooling_Limit))
				{
					d_1=pid_calculate( &Pid[dial], DIAL_MOTOR.speed_rpm,  4500);
				}
				else if(Real_Heat>=Cooling_Limit||Real_Heat_2>=Cooling_Limit)
				{
					d_1=pid_calculate( &Pid[dial], DIAL_MOTOR.speed_rpm,  0);
				}
			}
			else if(visual_sign==0||PTZ_sentry_mode==0)
			{
				d_1=0;
			}
			CAN2_Set_Current(0,h_2,d_1,0); 
  	}
		else if(launch_sign==0)
		{
			d_1=0;
			CAN2_Set_Current(0,h_2,0,0);
			r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , 0);
			r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm, 0);
			Friction_Set_Current(0,0);
		}
	}
	else if(RC_CtrlData.rc.s1==3&&offline_contrl_sign!=1)
	{
		d_1=0;
		CAN2_Set_Current(0,h_2,0,0);
		r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , 0);
		r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm, 0);
		Friction_Set_Current(r_1,r_2);
	}
	else
	{
		d_1=0;
		r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , 0);
		r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm, 0);
		Friction_Set_Current(0,0);
		CAN2_Set_Current(0,0,0,0); 
	}
}
//#endif

void Bounce_Amount_Judgment(void)//剩余子弹判断
{
	if(REF_Bullet()==0)
	{
		launch_sign=1;
	}
	else
	{
		if(REF_Bullet()<=750-350)
		{
			launch_sign=0;
		}
		else
		{
			launch_sign=1;
		}
	}
}



