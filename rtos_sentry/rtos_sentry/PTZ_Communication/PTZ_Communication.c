/*
  Q����һ��
  W���ڶ���
	E��������
	R�����Ķ�
	Z��ֹͣ��λ
	D��ǰ��
	C������
	X������
	V������
	T������
	Y������
	G������
	H������
	O���ڱ�Ѳ��
	N��DXCV����
*/

/*����һ��30����*/
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
	if(PTZ_GET_SIGN==1) //�Ƿ�ֵ
	{
		PTZ_sentry_mode=0;
		PTZ_sentry_mode_adnormal=0;
		PTZ_rifle_sign=0;
		if(PTZ_keyboard=='Q'||PTZ_keyboard=='W'||PTZ_keyboard=='E'||PTZ_keyboard=='R'||PTZ_keyboard=='Z'||PTZ_keyboard=='O'||PTZ_keyboard=='N'||PTZ_keyboard=='I')  //����Ķ��ϸߵغͽ���ֹͣ��Ѳ��ģʽ
		{
			PTZ_DXCV_SIGN=1;
			PTZ_DXCV_S_T=0;
			if(PTZ_target_x<0.5f&&PTZ_target_y>14.5f)  //��ֹ���ݴ���
			{
				if(PTZ_keyboard=='Q')  //���� 1300
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
				else if(PTZ_keyboard=='W')  //���ˣ����£�5500
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
				else if(PTZ_keyboard=='E')  //���� 1700
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
				else if(PTZ_keyboard=='R')  //ǰ��
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
				else if(PTZ_keyboard=='O')  //�ڱ�Ѳ��ģʽ
				{
					PTZ_sentry_mode=1;
				}
				else if(PTZ_keyboard=='I')
				{
					PTZ_sentry_mode=1;
					PTZ_sentry_mode_adnormal=1;
				}
				else if(PTZ_keyboard=='N')  //DXCV����
				{
					PTZ_DXCV_SIGN=0;
					PTZ_DXCV_S_T=1;
				}
				if(PTZ_keyboard=='Z')  //ֹͣ
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
			if(PTZ_keyboard=='D') //ǰ��
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
			else if(PTZ_keyboard=='C')  //����
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
			else if(PTZ_keyboard=='X')   //����
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
			else if(PTZ_keyboard=='V')  //����
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
			else if(PTZ_keyboard=='T')   //����
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
			else if(PTZ_keyboard=='Y')   //����
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
			else if(PTZ_keyboard=='G')   //����
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
			else if(PTZ_keyboard=='H')   //����
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
			else if(PTZ_keyboard=='H')   //����
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



