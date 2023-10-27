#include "holder_control.h"
#include "visual communication.h"
#include "rifle.h"
#include "chassis_control.h"
extern RC_Ctl_t RC_CtrlData;
extern uint8_t offline_contrl_sign;
signed short int h_1=0,h_2=0;
extern CascadePID double_pid[3];
extern 	int16_t Gyo[3];
extern float Eular[3];
extern Visual_Data_ Visual_Data;
extern signed short int d_1;
float holder_yaw;
float holder_pitch=0.0f;
extern uint8_t visual_sign;
//#if ROBOT_MODE==1
void holder_control(void)
{
	if(RC_CtrlData.rc.s1==1|| offline_contrl_sign==1)
	{
		Holder_Set_Current_yaw(0);
		h_1=0;
		h_2=0;
		holder_yaw=Eular[2];
	}
	else if(RC_CtrlData.rc.s1==3||RC_CtrlData.rc.s1==2)
	{
		visual_HolderControl();
		if(RC_CtrlData.rc.ch0>=1300)
		{
			holder_yaw=holder_yaw-0.3f;
		}
		else if(RC_CtrlData.rc.ch0<=748)
		{
			holder_yaw=holder_yaw+0.3f;
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
		CAN2_Set_Current(0,h_2,d_1,0); 
  }
}
//#elif ROBOT_MODE==2
uint8_t yaw_limit_sign=0,pitch_limit_sign=0;
uint8_t turn_back_sign=0;
float yaw_limit_L=0.0f;
float yaw_limit_R=-0.0f;
float yaw_limit_L_set=0.0f ;//0.0f;
float yaw_limit_R_set=-140.0f;//-140.0f;
float holder_yaw_adjust=0.0f;  //yaw轴初始位置离线设置
float holder_yaw_set;
float holder_yaw_set_S;
float yaw_limit_L_L;
float yaw_limit_R_R;
static uint8_t limit_set_sign=0;
void holder_control_sentry(void)
{
	/*上电后遥控器控制初始位置与角度*/
	if(RC_CtrlData.rc.s1==3&&offline_contrl_sign!=1)
	{
		if(RC_CtrlData.rc.ch0>=1300)
		{
			holder_yaw_adjust=holder_yaw_adjust-0.1f;
		}
		else if(RC_CtrlData.rc.ch0<=748)
		{
			holder_yaw_adjust=holder_yaw_adjust+0.1f;
		}
//		if(RC_CtrlData.rc.ch1>=1300)
//		{
//			holder_pitch=holder_pitch+0.2f;
//		}
//		else if(RC_CtrlData.rc.ch1<=748)
//		{
//			holder_pitch=holder_pitch-0.2f;
//		}
		if(holder_yaw_adjust>=180.0f)
		{
			holder_yaw_adjust=-180.0f;
		}
		else if(holder_yaw_adjust<=-180.0f)
		{
			holder_yaw_adjust=180.0f;
		}
		if(holder_pitch>=30)
		{
			holder_pitch=30;
		}
		else if(holder_pitch<=-15)
		{
			holder_pitch=-15;
		}
		limit_set_sign=1;
		holder_yaw=holder_yaw_adjust;
		holder_yaw_set=holder_yaw;
		holder_yaw_set_S=holder_yaw_set;
		holder_pitch=-5.0f;
		h_1=PID_CascadeCalc(&double_pid[holder_Yaw]  ,holder_yaw  , Eular[2], (Gyo[2]/10));
		h_2=PID_CascadeCalc(&double_pid[holder_Pitch],holder_pitch, Eular[1], (Gyo[0]/10));
		Holder_Set_Current_yaw(h_1);
		CAN2_Set_Current(0,h_2,d_1,0); 
	}
	else if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1)//yaw 80 -80  pitch -8  15
	{
		holder_limit_set();
		if(visual_sign==0)
		{
			/*限位设置*/
			if(yaw_limit_L<0&&yaw_limit_R>0)
			{
				yaw_limit_L_L=holder_yaw_set_S+yaw_limit_L_set;//85.0f;
				yaw_limit_R_R=holder_yaw_set_S+yaw_limit_R_set;//85.0f;
				
				if(holder_yaw_set>=yaw_limit_L_L&&yaw_limit_sign==0)
				{
					yaw_limit_sign=1;
					//turn_back_sign++;//回头
					gyro_stop();
					translation_L();
				}
				else if(holder_yaw_set<=yaw_limit_R_R&&yaw_limit_sign==1)
				{
					yaw_limit_sign=0;
//					translation_R();
//					Translation_G_S();
			  }
		}
			else 
			{
				if(holder_yaw>=yaw_limit_L&&yaw_limit_sign==0)
				{
					yaw_limit_sign=1;
					//turn_back_sign++;//回头
					gyro_stop();
//					translation_L();
				}
				else if(holder_yaw<=yaw_limit_R&&yaw_limit_sign==1)
				{
					yaw_limit_sign=0;
//					translation_R();
//					Translation_G_S();
				}
			}
			/*pitch轴限制*/
//			if(holder_pitch>=15)
//			{
//				pitch_limit_sign=1;
//			}
//			else if(holder_pitch<=-8)
//			{
//				pitch_limit_sign=0;
//			}
			/*如果需要修改扫描速度两处都要更改*/
			if(turn_back_sign<=3)
			{
				/*如果需要角度翻转*/
				if(yaw_limit_L<0&&yaw_limit_R>0)
				{
					if(yaw_limit_sign==1)
					{
						holder_yaw_set=holder_yaw_set-0.25f;
					}
					else if(yaw_limit_sign==0)
					{
						holder_yaw_set=holder_yaw_set+0.25f;
					}
					if(holder_yaw_set>=180.0f)
					{
						holder_yaw=holder_yaw_set-360.0f;
					}
					else if(holder_yaw_set<=-180.0f)
					{
						holder_yaw=holder_yaw_set+360.0f;
					}
					else 
					{
						holder_yaw=holder_yaw_set;
					}
				}
				/*正常模式*/
				else
				{
					if(yaw_limit_sign==1)
					{
						holder_yaw=holder_yaw-0.25f;
					}
					else if(yaw_limit_sign==0)
					{
						holder_yaw=holder_yaw+0.25f;
					}
				}
				/*pitch轴转动*/
//		  	if(pitch_limit_sign==1)
//		  	{
//		  		holder_pitch=holder_pitch-0.1f;
//		  	} 
//		  	else if(pitch_limit_sign==0)
//			  {
//		  		holder_pitch=holder_pitch+0.1f;
//		  	}
		   }
			/*回头*/
			else if(turn_back_sign>2)
			{
				holder_yaw=holder_yaw+0.3f;
				holder_yaw_set=holder_yaw;
				if((holder_yaw>yaw_limit_R&&holder_yaw<yaw_limit_L)||(holder_yaw>yaw_limit_R_R&&holder_yaw<yaw_limit_L_L))
				{
					turn_back_sign=0;
				}
			}
			holder_pitch=-5.0f;
	  }
		/*视觉识别模式*/
		else if(visual_sign==1)
		{
			visual_HolderControl();
			holder_yaw_set=holder_yaw;
		}
		if(holder_yaw>=180.0f)
		{
			holder_yaw=-180.0f;
		}
		else if(holder_yaw<=-180.0f)
		{
			holder_yaw=180.0f;
		}
		if(holder_pitch>=10)
		{
			holder_pitch=10;
		}
		else if(holder_pitch<=-15)
		{
			holder_pitch=-15;
		}
		h_1=PID_CascadeCalc(&double_pid[holder_Yaw]  ,holder_yaw  , Eular[2], (Gyo[2]/10));
		h_2=PID_CascadeCalc(&double_pid[holder_Pitch],holder_pitch, Eular[1], (Gyo[0]/10));
		Holder_Set_Current_yaw(h_1);
		CAN2_Set_Current(0,h_2,d_1,0); 
	}
	else
	{
		holder_yaw_adjust=Eular[2];
		Holder_Set_Current_yaw(0);
		CAN2_Set_Current(0,0,0,0); 
		holder_yaw=Eular[2];
	}
}
/*角度设置防止反转*/
void holder_limit_set(void)
{
	if(limit_set_sign==1)
	{
		limit_set_sign=0;
		yaw_limit_L=holder_yaw_adjust+yaw_limit_L_set;
		yaw_limit_R=holder_yaw_adjust+yaw_limit_R_set;
		if(yaw_limit_L>=180.0f)
		{
			yaw_limit_L=yaw_limit_L-360.0f;
		}
		else if(yaw_limit_L<=-180.0f)
		{
			yaw_limit_L=yaw_limit_L+360.0f;
		}
		if(yaw_limit_R>=180.0f)
		{
			yaw_limit_R=yaw_limit_R-360.0f;
		}
		else if(yaw_limit_R<=-180.0f)
		{
			yaw_limit_R=yaw_limit_R+360.0f;
		}
  }
}
//#endif
extern uint8_t visual_contrl;
void visual_HolderControl(void)
{
	if(RC_CtrlData.rc.s1==2)
	{
		holder_pitch=holder_pitch+Visual_Data.Visual_Pitch;
		holder_yaw  =holder_yaw  +Visual_Data.Visual_Yaw  ;
		Visual_Data.Visual_Pitch=Visual_Data.Visual_Yaw=0;    //重置视觉传回来的值，以防疯车
	}
}
extern uint8_t PTZ_sentry_mode;
extern uint8_t PTZ_sentry_mode_adnormal;
void PTZ_holder_control(void)
{
	/*上电后遥控器控制初始位置与角度*/
	if(RC_CtrlData.rc.s1==3&&offline_contrl_sign!=1)   //云台位置修正
	{
		if(RC_CtrlData.rc.ch0>=1300)
		{
			holder_yaw_adjust=holder_yaw_adjust-0.1f;
		}
		else if(RC_CtrlData.rc.ch0<=748)
		{
			holder_yaw_adjust=holder_yaw_adjust+0.1f;
		}
//		if(RC_CtrlData.rc.ch1>=1300)
//		{
//			holder_pitch=holder_pitch+0.2f;
//		}
//		else if(RC_CtrlData.rc.ch1<=748)
//		{
//			holder_pitch=holder_pitch-0.2f;
//		}
		if(holder_yaw_adjust>=180.0f)
		{
			holder_yaw_adjust=-180.0f;
		}
		else if(holder_yaw_adjust<=-180.0f)
		{
			holder_yaw_adjust=180.0f;
		}
		if(holder_pitch>=30)
		{
			holder_pitch=30;
		}
		else if(holder_pitch<=-15)
		{
			holder_pitch=-15;
		}
		limit_set_sign=1;
		holder_yaw=holder_yaw_adjust;
		holder_yaw_set=holder_yaw;
		holder_yaw_set_S=holder_yaw_set;
		holder_pitch=0.0f;
		h_1=PID_CascadeCalc(&double_pid[holder_Yaw]  ,holder_yaw  , Eular[2], (Gyo[2]/10));
		h_2=PID_CascadeCalc(&double_pid[holder_Pitch],holder_pitch, Eular[1], (Gyo[0]/10));
		Holder_Set_Current_yaw(h_1);
		CAN2_Set_Current(0,h_2,d_1,0); 
	}
	else if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1)//yaw 80 -80  pitch -8  15
	{
		if(PTZ_sentry_mode==1)
		{
			holder_limit_set();
			if(visual_sign==0)
			{
				/*限位设置*/
				if(yaw_limit_L<0&&yaw_limit_R>0)
				{
					yaw_limit_L_L=holder_yaw_set_S+yaw_limit_L_set;//0.0f;   //记得上面也要改
					yaw_limit_R_R=holder_yaw_set_S+yaw_limit_R_set;//-140.0f;
					
					if(holder_yaw_set>=yaw_limit_L_L&&yaw_limit_sign==0)
					{
						yaw_limit_sign=1;
						turn_back_sign++;//回头
						gyro_stop();
						translation_L();
					}
					else if(holder_yaw_set<=yaw_limit_R_R&&yaw_limit_sign==1)
					{
						yaw_limit_sign=0;
					}
				}
				else 
				{
					if(holder_yaw>=yaw_limit_L&&yaw_limit_sign==0)
					{
						yaw_limit_sign=1;
						turn_back_sign++;//回头
						gyro_stop();
					}
					else if(holder_yaw<=yaw_limit_R&&yaw_limit_sign==1)
					{
						yaw_limit_sign=0;
					}
				}
				/*pitch轴限制*/
				if(PTZ_sentry_mode_adnormal==0)
				{
					if(holder_pitch>=6)
					{
						pitch_limit_sign=1;
					}
					else if(holder_pitch<=-14)
					{
						pitch_limit_sign=0;
					}
				}
				else if(PTZ_sentry_mode_adnormal==1)
				{
					if(holder_pitch>=20)
					{
						pitch_limit_sign=1;
					}
					else if(holder_pitch<=-10)
					{
						pitch_limit_sign=0;
					}
				}
				/*如果需要修改扫描速度两处都要更改*/
				if(turn_back_sign<=2)
				{
					/*如果需要角度翻转*/
					if(yaw_limit_L<0&&yaw_limit_R>0)
					{
						if(yaw_limit_sign==1)
						{
							holder_yaw_set=holder_yaw_set-0.2f;
						}
						else if(yaw_limit_sign==0)
						{
							holder_yaw_set=holder_yaw_set+0.2f;
						}
						if(holder_yaw_set>=180.0f)
						{
							holder_yaw=holder_yaw_set-360.0f;
						}
						else if(holder_yaw_set<=-180.0f)
						{
							holder_yaw=holder_yaw_set+360.0f;
						}
						else 
						{
							holder_yaw=holder_yaw_set;
						}
					}
					/*正常模式*/
					else
					{
						if(yaw_limit_sign==1)
						{
							holder_yaw=holder_yaw-0.2f;
						}
						else if(yaw_limit_sign==0)
						{
							holder_yaw=holder_yaw+0.2f;
						}
					}
				 }
				/*回头*/
				else if(turn_back_sign>2)
				{
					//holder_pitch=-5.0f;
					holder_yaw=holder_yaw+0.3f;
					holder_yaw_set=holder_yaw;
					if((holder_yaw>yaw_limit_R&&holder_yaw<yaw_limit_L)||(holder_yaw>yaw_limit_R_R&&holder_yaw<yaw_limit_L_L))
					{
						turn_back_sign=0;
					}
				}
				//holder_pitch=-5.0f;
				/*pitch轴转动*/
				if(pitch_limit_sign==1)
				{
					holder_pitch=holder_pitch-0.1f;
				} 
				else if(pitch_limit_sign==0)
				{
					holder_pitch=holder_pitch+0.1f;
				}
			}
			
			/*视觉识别模式*/
			else if(visual_sign==1)
			{
				visual_HolderControl();
				holder_yaw_set=holder_yaw;
			}
		}
		else if(PTZ_sentry_mode==0)
		{
			holder_yaw=holder_yaw_adjust;
			holder_pitch=0.0f;
		}
		if(holder_yaw>=180.0f)
		{
			holder_yaw=-180.0f;
		}
		else if(holder_yaw<=-180.0f)
		{
			holder_yaw=180.0f;
		}
		if(PTZ_sentry_mode_adnormal==0)
		{
			if(holder_pitch>=10)
			{
				holder_pitch=10;
			}
			else if(holder_pitch<=-15)
			{
				holder_pitch=-15;
			}
  	}
		else if(PTZ_sentry_mode_adnormal==1)
		{
			if(holder_pitch>=30)
			{
				holder_pitch=30;
			}
			else if(holder_pitch<=-15)
			{
				holder_pitch=-15;
			}
		}
		h_1=PID_CascadeCalc(&double_pid[holder_Yaw]  ,holder_yaw  , Eular[2], (Gyo[2]/10));
		h_2=PID_CascadeCalc(&double_pid[holder_Pitch],holder_pitch, Eular[1], (Gyo[0]/10));
		Holder_Set_Current_yaw(h_1);
		CAN2_Set_Current(0,h_2,d_1,0); 
	}
	else
	{
		holder_yaw_adjust=Eular[2];
		Holder_Set_Current_yaw(0);
		CAN2_Set_Current(0,0,0,0); 
		holder_yaw=Eular[2];
	}
}

extern uint8_t sentry_go_sign;
void holder_control_sentry_gogogo(void)
{
	/*上电后遥控器控制初始位置与角度*/
	if((RC_CtrlData.rc.s1==3&&offline_contrl_sign!=1)||(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1&&sentry_go_sign==0))
	{
		if(RC_CtrlData.rc.ch0>=1300)
		{
			holder_yaw_adjust=holder_yaw_adjust-0.1f;
		}
		else if(RC_CtrlData.rc.ch0<=748)
		{
			holder_yaw_adjust=holder_yaw_adjust+0.1f;
		}
//		if(RC_CtrlData.rc.ch1>=1300)
//		{
//			holder_pitch=holder_pitch+0.2f;
//		}
//		else if(RC_CtrlData.rc.ch1<=748)
//		{
//			holder_pitch=holder_pitch-0.2f;
//		}
		if(holder_yaw_adjust>=180.0f)
		{
			holder_yaw_adjust=-180.0f;
		}
		else if(holder_yaw_adjust<=-180.0f)
		{
			holder_yaw_adjust=180.0f;
		}
		if(holder_pitch>=30)
		{
			holder_pitch=30;
		}
		else if(holder_pitch<=-15)
		{
			holder_pitch=-15;
		}
		limit_set_sign=1;
		holder_yaw=holder_yaw_adjust;
		holder_yaw_set=holder_yaw;
		holder_yaw_set_S=holder_yaw_set;
		holder_pitch=0.0f;
		h_1=PID_CascadeCalc(&double_pid[holder_Yaw]  ,holder_yaw  , Eular[2], (Gyo[2]/10));
		h_2=PID_CascadeCalc(&double_pid[holder_Pitch],holder_pitch, Eular[1], (Gyo[0]/10));
		Holder_Set_Current_yaw(h_1);
		CAN2_Set_Current(0,h_2,d_1,0); 
	}
	else if(RC_CtrlData.rc.s1==2&&offline_contrl_sign!=1&&sentry_go_sign==1)//yaw 80 -80  pitch -8  15
	{
		holder_limit_set();
		if(visual_sign==0)
		{
			/*限位设置*/
			if(yaw_limit_L<0&&yaw_limit_R>0)
			{
				yaw_limit_L_L=holder_yaw_set_S+yaw_limit_L_set;//85.0f;
				yaw_limit_R_R=holder_yaw_set_S+yaw_limit_R_set;//85.0f;
				
				if(holder_yaw_set>=yaw_limit_L_L&&yaw_limit_sign==0)
				{
					yaw_limit_sign=1;
					turn_back_sign++;//回头
					gyro_stop();
					translation_L();
				}
				else if(holder_yaw_set<=yaw_limit_R_R&&yaw_limit_sign==1)
				{
					yaw_limit_sign=0;
//					translation_R();
//					Translation_G_S();
			  }
		}
			else 
			{
				if(holder_yaw>=yaw_limit_L&&yaw_limit_sign==0)
				{
					yaw_limit_sign=1;
					turn_back_sign++;//回头
					gyro_stop();
//					translation_L();
				}
				else if(holder_yaw<=yaw_limit_R&&yaw_limit_sign==1)
				{
					yaw_limit_sign=0;
//					translation_R();
//					Translation_G_S();
				}
			}
			/*pitch轴限制*/
//			if(holder_pitch>=15)
//			{
//				pitch_limit_sign=1;
//			}
//			else if(holder_pitch<=-8)
//			{
//				pitch_limit_sign=0;
//			}
			/*如果需要修改扫描速度两处都要更改*/
			if(turn_back_sign<=3)
			{
				/*如果需要角度翻转*/
				if(yaw_limit_L<0&&yaw_limit_R>0)
				{
					if(yaw_limit_sign==1)
					{
						holder_yaw_set=holder_yaw_set-0.25f;
					}
					else if(yaw_limit_sign==0)
					{
						holder_yaw_set=holder_yaw_set+0.25f;
					}
					if(holder_yaw_set>=180.0f)
					{
						holder_yaw=holder_yaw_set-360.0f;
					}
					else if(holder_yaw_set<=-180.0f)
					{
						holder_yaw=holder_yaw_set+360.0f;
					}
					else 
					{
						holder_yaw=holder_yaw_set;
					}
				}
				/*正常模式*/
				else
				{
					if(yaw_limit_sign==1)
					{
						holder_yaw=holder_yaw-0.25f;
					}
					else if(yaw_limit_sign==0)
					{
						holder_yaw=holder_yaw+0.25f;
					}
				}
				/*pitch轴转动*/
//		  	if(pitch_limit_sign==1)
//		  	{
//		  		holder_pitch=holder_pitch-0.1f;
//		  	} 
//		  	else if(pitch_limit_sign==0)
//			  {
//		  		holder_pitch=holder_pitch+0.1f;
//		  	}
		   }
			/*回头*/
			else if(turn_back_sign>2)
			{
				holder_yaw=holder_yaw+0.3f;
				holder_yaw_set=holder_yaw;
				if((holder_yaw>yaw_limit_R&&holder_yaw<yaw_limit_L)||(holder_yaw>yaw_limit_R_R&&holder_yaw<yaw_limit_L_L))
				{
					turn_back_sign=0;
				}
			}
			holder_pitch=-1.0f;
	  }
		/*视觉识别模式*/
		else if(visual_sign==1)
		{
			visual_HolderControl();
			holder_yaw_set=holder_yaw;
		}
		if(holder_yaw>=180.0f)
		{
			holder_yaw=-180.0f;
		}
		else if(holder_yaw<=-180.0f)
		{
			holder_yaw=180.0f;
		}
		if(holder_pitch>=10)
		{
			holder_pitch=10;
		}
		else if(holder_pitch<=-15)
		{
			holder_pitch=-15;
		}
		h_1=PID_CascadeCalc(&double_pid[holder_Yaw]  ,holder_yaw  , Eular[2], (Gyo[2]/10));
		h_2=PID_CascadeCalc(&double_pid[holder_Pitch],holder_pitch, Eular[1], (Gyo[0]/10));
		Holder_Set_Current_yaw(h_1);
		CAN2_Set_Current(0,h_2,d_1,0); 
	}
	else
	{
		holder_yaw_adjust=Eular[2];
		Holder_Set_Current_yaw(0);
		CAN2_Set_Current(0,0,0,0); 
		holder_yaw=Eular[2];
	}
}



