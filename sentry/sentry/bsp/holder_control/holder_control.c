#include "holder_control.h"
#include "visual communication.h"
#define holder_mode 1
extern RC_Ctl_t RC_CtrlData;
extern uint8_t offline_contrl_sign;
signed short int h_1=0,h_2=0;
extern CascadePID double_pid[3];
extern 	int16_t Gyo[3];
extern float Eular[3];
extern Visual_Data_ Visual_Data;
#if holder_mode==1
/*调pid用的云台控制程序*/
void holder_control(void)//陀螺仪云台
{
	if(RC_CtrlData.rc.s2==3 || offline_contrl_sign==1)//掉线保护
	{
		Holder_Set_Current_yaw(0);
		Holder_Set_Current_pitch(0);
		h_1=0;
		h_2=0;
	}
	/*yaw轴模式*/
	if(RC_CtrlData.rc.s1==1&&RC_CtrlData.rc.s2==1)                             
	{
		h_1=-PID_CascadeCalc(&double_pid[holder_Yaw],0.0f,Eular[2],(Gyo[2]/10));
		Holder_Set_Current_yaw(h_1);
	}
	else if(RC_CtrlData.rc.s1==3&&RC_CtrlData.rc.s2==1)
	{
		h_1=-PID_CascadeCalc(&double_pid[holder_Yaw],-20.0f,Eular[2],(Gyo[2]/10));
    Holder_Set_Current_yaw(h_1);
	}
	else if(RC_CtrlData.rc.s1==2&&RC_CtrlData.rc.s2==1)
	{
		h_1=-PID_CascadeCalc(&double_pid[holder_Yaw],-40.0f,Eular[2],(Gyo[2]/10));
		Holder_Set_Current_yaw(h_1);
	}
	/*pitch轴模式*/
	if(RC_CtrlData.rc.s1==1&&RC_CtrlData.rc.s2==2)
	{
		h_2=-PID_CascadeCalc(&double_pid[holder_Pitch],20.0f,Eular[0],(Gyo[1]/10));
		Holder_Set_Current_pitch(h_2);
	}
	else if(RC_CtrlData.rc.s1==3&&RC_CtrlData.rc.s2==2)
	{
		h_2=-PID_CascadeCalc(&double_pid[holder_Pitch],10.0f,Eular[0],(Gyo[1]/10));
		Holder_Set_Current_pitch(h_2);
	}
	else if(RC_CtrlData.rc.s1==2&&RC_CtrlData.rc.s2==2)
	{
		h_2=-PID_CascadeCalc(&double_pid[holder_Pitch],0.0f,Eular[0],(Gyo[1]/10));
		Holder_Set_Current_pitch(h_2);
	}
}
#elif holder_mode==2
float holder_yaw=0.0f;
float holder_pitch=0.0f;
void holder_control(void)
{
	if(RC_CtrlData.rc.s1==1|| offline_contrl_sign==1)
	{
		Holder_Set_Current_yaw(0);
		h_1=0;
		h_2=0;
	}
	else if(RC_CtrlData.rc.s1==3||RC_CtrlData.rc.s1==2)
	{
		visual_HolderControl();
		if(RC_CtrlData.rc.ch0>=1300)
		{
			holder_yaw=holder_yaw+0.3f;
		}
		else if(RC_CtrlData.rc.ch0<=748)
		{
			holder_yaw=holder_yaw-0.3f;
		}
		if(RC_CtrlData.rc.ch1>=1300)
		{
			holder_pitch=holder_pitch+0.2f;
		}
		else if(RC_CtrlData.rc.ch1<=748)
		{
			holder_pitch=holder_pitch-0.2f;
		}
		if(holder_yaw>=180.0f)
		{
			holder_yaw=-180.0f;
		}
		else if(holder_yaw<=-180.0f)
		{
			holder_yaw=180.0f;
		}
		if(holder_pitch>=30)
		{
			holder_pitch=30;
		}
		else if(holder_pitch<=-15)
		{
			holder_pitch=-15;
		}
		h_1=PID_CascadeCalc(&double_pid[holder_Yaw]  ,holder_yaw  , Eular[2], (Gyo[2]/10));
		h_2=PID_CascadeCalc(&double_pid[holder_Pitch],holder_pitch, Eular[1], (Gyo[0]/10));
		Holder_Set_Current_yaw(h_1);
  }
}
void visual_HolderControl(void)
{
	if(RC_CtrlData.rc.s1==2)
	{
		holder_pitch=holder_pitch+Visual_Data.Visual_Pitch;
		holder_yaw  =holder_yaw  +Visual_Data.Visual_Yaw  ;
		Visual_Data.Visual_Pitch=Visual_Data.Visual_Yaw=0;    //重置视觉传回来的值，以防疯车
	}
}

#endif



