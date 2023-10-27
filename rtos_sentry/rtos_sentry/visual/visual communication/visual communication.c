#include "visual communication.h"
#include "usart.h"
#include "referee.h"
Visual_Data_ Visual_Data;
float visual_coefficient=1.0;
uint8_t visual_data[10];
uint8_t visual_sign=0;
extern DMA_HandleTypeDef hdma_uart4_rx;
uint16_t visual_contrl;

float Bullet_velocity;//����
float_uint8 pitch;
float_uint8 yaw;
float_uint8 pitch_send;
float_uint8 yaw_send;
float_uint8 bullet_velocity;


void Visual_init(uint8_t *rx1_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //ʹ��DMA���ڽ���
    SET_BIT(huart4.Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);

    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_uart4_rx);
    {
        __HAL_DMA_DISABLE(&hdma_uart4_rx);
    }
            hdma_uart4_rx.Instance->PAR = (uint32_t) & (UART4->DR);

    //memory buffer 1
    hdma_uart4_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //data length
    //���ݳ���
    hdma_uart4_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer

    __HAL_DMA_ENABLE(&hdma_uart4_rx);

}
/*���ٻ�ȡ*/
void Bullet_velocity_get(void)
{
	Bullet_velocity=REF_Report_RealShootSpeed();
}
/*�Ӿ����ս���*/
//void visual_receive(void)
//{
//	if(visual_data[0]==0x80&&visual_data[5]==0x7F)
//	{
//		visual_contrl=0;
//		Visual_Data.Pitch_High=visual_data[1];
//		Visual_Data.Pitch_Low =visual_data[2];
//		Visual_Data.Yaw_High  =visual_data[3];
//		Visual_Data.Yaw_Low   =visual_data[4];
//		Visual_Data.Visual_Pitch= ((float)(int16_t)(Visual_Data.Pitch_High<<8|Visual_Data.Pitch_Low)*90/32767)*visual_coefficient;
//		Visual_Data.Visual_Yaw  =-((float)(int16_t)(Visual_Data.Yaw_High  <<8|Visual_Data.Yaw_Low  )*90/32767)*visual_coefficient;
//	}
//	else
//	{
//		Visual_Data.Visual_Pitch=0;
//		Visual_Data.Visual_Yaw  =0;
//	}
//}
void visual_receive(void)
{
	if(visual_data[0]==0x80&&visual_data[9]==0x7F)
	{
		//visual_contrl=0;
		pitch.c[0]=visual_data[1];
		pitch.c[1]=visual_data[2];
		pitch.c[2]=visual_data[3];
		pitch.c[3]=visual_data[4];
		
		yaw.c[0]=visual_data[5];
		yaw.c[1]=visual_data[6];
		yaw.c[2]=visual_data[7];
		yaw.c[3]=visual_data[8];
		Visual_Data.Visual_Pitch=  pitch.f*visual_coefficient;
		Visual_Data.Visual_Yaw  = -yaw.f*visual_coefficient;
		
		if(Visual_Data.Visual_Pitch!=0.0f||Visual_Data.Visual_Yaw!=0.0f)
		{
			visual_contrl=0;
		}
	}
	else
	{
		Visual_Data.Visual_Pitch=0;
		Visual_Data.Visual_Yaw  =0;
	}
}
/*�Ӿ����ݷ���*/
extern float Eular[3];
uint8_t Enemy_Color;//������ɫ
void visual_sent(void)
{
	
	Color_judge();
	uint8_t a[14];
	yaw_send.f=Eular[2];
	pitch_send.f=Eular[1];
//bullet_velocity.f=REF_Report_RealShootSpeed();//���ٻ�ȡ
	/*֡ͷ*/
  a[0]=0xAA;
	/*yaw��*/
	a[1]=yaw_send.c[0];
	a[2]=yaw_send.c[1];
	a[3]=yaw_send.c[2];
	a[4]=yaw_send.c[3];
	a[5]=0;
	/*pitch��*/
	a[6]=pitch_send.c[0];
	a[7]=pitch_send.c[1];
	a[8]=pitch_send.c[2];
	a[9]=pitch_send.c[3];
	a[10]=0;
	/*����*/
	a[11]=28;
	/*��ɫ*/
	a[12]=Enemy_Color;
	/*֡β*/
	a[13]=0xBB;
	for(uint8_t i=0;i<sizeof(a);i++)
	{
		HAL_UART_Transmit( &huart4, &a[i],1,0xffff );
	}
}

/*�Ӿ����ݽ���*/
void visual_data_receive(void)
{
//	  uint8_t ucTemp;	
    __HAL_UART_CLEAR_PEFLAG(&huart4);
    __HAL_DMA_DISABLE(&hdma_uart4_rx);
	  hdma_uart4_rx.Instance->NDTR = 10;
		__HAL_DMA_ENABLE(&hdma_uart4_rx);
		visual_receive();
//		visual_data[0]=visual_data[1]=visual_data[2]=visual_data[3]=0;
	  pitch.f=yaw.f=0;
}
/*�Ӿ�����*/
void visual_sign_set(void) 
{
	visual_contrl++;
	if(visual_contrl>=50)
	{
		visual_contrl=50;
		visual_sign=0;
	}
	else
	{
		visual_sign=1;
	}
}
/*��ɫ�ж�*/
void Color_judge(void)
{
	if(REF_Report_robot_ID()<=10)
	{
		Enemy_Color=0;  
	}
	if(REF_Report_robot_ID()>=100)
	{
		Enemy_Color=1;   
	}
}









