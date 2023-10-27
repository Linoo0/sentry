/*
  Q：第一段
  W：第二段
	E：第三段
	R：第四段
	Z：停止复位
	D：前进
	C：后退
	X：左移
	V：右移
	T：左上
	Y：右上
	G：左下
	H：右下
	O：哨兵巡逻
	N：DXCV开关
*/

/*左右一秒30厘米*/
#include "PTZ_Communication.h"
#include "referee.h"
#include "chassis_control.h"
float PTZ_target_x;
float PTZ_target_y;
float PTZ_target_z;
uint8_t PTZ_keyboard;
uint8_t PTZ_DXCV_SIGN=0;
uint8_t PTZ_sentry_mode=0;
uint8_t PTZ_sentry_mode_adnormal=0;
uint8_t PTZ_DXCV_S_T=0;
uint8_t PTZ_rifle_sign=0;

static uint16_t PTZ_Q;
static uint16_t PTZ_W;
static uint16_t PTZ_E;
static uint16_t PTZ_R;
static uint16_t PTZ_DXCV;
static uint16_t PTZ_TYGH;
static uint16_t PTZ_rifle;

extern uint8_t PTZ_GET_SIGN;
extern float Vx;
extern float Vy;
extern float W;
void Get_PTZ(void)
{
	PTZ_target_x=REF_PTZ_X();
	PTZ_target_y=REF_PTZ_Y();
	PTZ_target_z=REF_PTZ_Z();
	PTZ_keyboard=REF_PTZ_KEY();
}

void PTZ_Control(void)
{
	Vx=Vy=W=0;
	Get_PTZ();
	if(PTZ_GET_SIGN==1) //是否发值
	{
		PTZ_sentry_mode=0;
		PTZ_sentry_mode_adnormal=0;
		PTZ_rifle_sign=0;
		if(PTZ_keyboard=='Q'||PTZ_keyboard=='W'||PTZ_keyboard=='E'||PTZ_keyboard=='R'||PTZ_keyboard=='Z'||PTZ_keyboard=='O'||PTZ_keyboard=='N'||PTZ_keyboard=='I')  //软件四段上高地和紧急停止和巡航模式
		{
			PTZ_DXCV_SIGN=1;
			PTZ_DXCV_S_T=0;
			if(PTZ_target_x<0.5f&&PTZ_target_y>14.5f)  //防止数据错误
			{
				if(PTZ_keyboard=='Q')  //左移 1300
				{
					Vx=-1000;
					Vy=0;
					PTZ_Q++;
					if(PTZ_Q>=1365)  
					{
						PTZ_DXCV_SIGN=0;
						PTZ_Q=0;
						PTZ_GET_SIGN=0;
						Vx=Vy=0;
					}
				}
				else if(PTZ_keyboard=='W')  //后退（上坡）5500
				{
					Vx=0;
					Vy=-1500;
					PTZ_W++;
					if(PTZ_W>=2980)  
					{
						PTZ_DXCV_SIGN=0;
						PTZ_W=0;
						PTZ_GET_SIGN=0;
						Vx=Vy=0;
					}
				}
				else if(PTZ_keyboard=='E')  //右移 1700
				{
					Vx=1000;
					Vy=0;
					PTZ_E++;
					if(PTZ_E>=1785)  
					{
						PTZ_DXCV_SIGN=0;
						PTZ_E=0;
						PTZ_GET_SIGN=0;
						Vx=Vy=0;
					}
				}
				else if(PTZ_keyboard=='R')  //前进
				{
					Vx=0;
					Vy=1000;
					//PTZ_R++;
					if(REF_Report_RFID_State()>0) //3s
					{
						PTZ_DXCV_SIGN=0;
						PTZ_R=0;
						PTZ_GET_SIGN=0;
						Vx=Vy=0;
					}
				}
				else if(PTZ_keyboard=='O')  //哨兵巡航模式
				{
					PTZ_sentry_mode=1;
				}
				else if(PTZ_keyboard=='I')
				{
					PTZ_sentry_mode=1;
					PTZ_sentry_mode_adnormal=1;
				}
				else if(PTZ_keyboard=='N')  //DXCV开关
				{
					PTZ_DXCV_SIGN=0;
					PTZ_DXCV_S_T=1;
				}
				if(PTZ_keyboard=='Z')  //停止
				{
					PTZ_DXCV_SIGN=0;
					Vx=Vy=W=0;
					PTZ_Q=0;
					PTZ_W=0;
					PTZ_E=0;
					PTZ_R=0;
					PTZ_DXCV=0;
					PTZ_TYGH=0;
					PTZ_GET_SIGN=0;
					PTZ_sentry_mode=0;
					PTZ_rifle_sign=0;
				}
			}
	  }
		if(PTZ_DXCV_SIGN==0&&PTZ_DXCV_S_T==1)
		{
			if(PTZ_keyboard=='D') //前进
			{
				Vx=0;
				Vy=1000;
				PTZ_DXCV++;
				if(PTZ_DXCV>=PTZ_target_x*333)
				{
					PTZ_DXCV=0;
					PTZ_GET_SIGN=0;
					Vx=Vy=0;
				}
			}
			else if(PTZ_keyboard=='C')  //后退
			{
				Vx=0;
				Vy=-1500;
				PTZ_DXCV++;
				if(PTZ_DXCV>=PTZ_target_x*333)
				{
					PTZ_DXCV=0;
					PTZ_GET_SIGN=0;
					Vx=Vy=0;
				}
			}
			else if(PTZ_keyboard=='X')   //左移
			{
				Vx=-1000;
				Vy=0;
				PTZ_DXCV++;
				if(PTZ_DXCV>=PTZ_target_x*333)
				{
					PTZ_DXCV=0;
					PTZ_GET_SIGN=0;
					Vx=Vy=0;
				}
			}
			else if(PTZ_keyboard=='V')  //右移
			{
				Vx=1000;
				Vy=0;
				PTZ_DXCV++;
				if(PTZ_DXCV>=PTZ_target_x*333)
				{
					PTZ_DXCV=0;
					PTZ_GET_SIGN=0;
					Vx=Vy=0;
				}
			}
			else if(PTZ_keyboard=='T')   //左上
			{
				Vx=-1000;
				Vy=1000;
				PTZ_TYGH++;
				if(PTZ_TYGH>=PTZ_target_x*333)
				{
					PTZ_TYGH=0;
					PTZ_GET_SIGN=0;
					Vx=Vy=0;
				}
			}
			else if(PTZ_keyboard=='Y')   //右上
			{
				Vx=1000;
				Vy=1000;
				PTZ_TYGH++;
				if(PTZ_TYGH>=PTZ_target_x*333)
				{
					PTZ_TYGH=0;
					PTZ_GET_SIGN=0;
					Vx=Vy=0;
				}
			}
			else if(PTZ_keyboard=='G')   //左下
			{
				Vx=-1000;
				Vy=-1000;
				PTZ_TYGH++;
				if(PTZ_TYGH>=PTZ_target_x*333)
				{
					PTZ_TYGH=0;
					PTZ_GET_SIGN=0;
					Vx=Vy=0;
				}
			}
			else if(PTZ_keyboard=='H')   //右下
			{
				Vx=1000;
				Vy=-1000;
				PTZ_TYGH++;
				if(PTZ_TYGH>=PTZ_target_x*333)
				{
					PTZ_TYGH=0;
					PTZ_GET_SIGN=0;
					Vx=Vy=0;
				}
			}
			else if(PTZ_keyboard=='H')   //卡弹
			{
				PTZ_rifle++;
				PTZ_rifle_sign=1;
				if(PTZ_rifle>=PTZ_target_x*333)
				{
					PTZ_rifle_sign=0;
					PTZ_rifle=0;
					PTZ_GET_SIGN=0;
				}
			}
		}
	}

}



