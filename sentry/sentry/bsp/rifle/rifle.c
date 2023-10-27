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
void rifle_control(void)
{
	if(offline_contrl_sign==1||RC_CtrlData.rc.s1==1)
	{
		d_1=0;
		r_1=0,r_2=0,
		Friction_Set_Current(0,0);	
	}
	else if(RC_CtrlData.rc.s1==3||RC_CtrlData.rc.s2==1)
	{
		r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , 0);
		r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm, 0);
		Friction_Set_Current(r_1,r_2);
	}
	else if(RC_CtrlData.rc.s1==2&&RC_CtrlData.rc.s2==3)
	{
		r_1=pid_calculate( &Pid[left] , FRICTION_WHEEL[left].speed_rpm , -6000);
		r_2=pid_calculate( &Pid[right], FRICTION_WHEEL[right].speed_rpm,  6000);
		if(RC_CtrlData.rc.ch3<=500)
		{
		  d_1=pid_calculate( &Pid[dial], DIAL_MOTOR.speed_rpm,  3000);
		}
		else if(RC_CtrlData.rc.ch3>=1500)
		{
			d_1=pid_calculate( &Pid[dial], DIAL_MOTOR.speed_rpm,    -500);
		}
		else
		{
			d_1=0;
		}
		Friction_Set_Current(r_1,r_2);
	}
}
