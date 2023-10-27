#include "stm32f4xx.h"
#include "pid.h"
#include "can.h"
#include "can_receive.h"
#include "packet.h"
#include "imu_data_decode.h"
#include "rc.h"
#include "usart.h"
#include "tim.h"
#include "pid_config.h"
#include "crc.h"
#include "referee.h"
int main()
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//�жϷ���
	imu_data_decode_init();                        //�����ǳ�ʼ��
	USART5_Config();                               //�����Ǵ��ڳ�ʼ��
	CRC_init();                                    //crcУ���ʼ��
  USART1_Init();                                 //����ϵͳ���ڳ�ʼ��
	pid_parameter_setting();                       //pid������ʼ��
	RC_Init();                                     //ң������ʼ��
	TIM5_Config();                                 //���������ݶ�ȡ��ʱ�� 3ms
  CAN1_Config();                                 //CAN1�������ĸ��������̨yaw��
	CAN2_Config();                                 //CAN2����̨pitch��
  TIM3_Config();                                 //�����˲������ж�ʱ�� 5ms
	USART6_Config();                               //vofa+���ڳ�ʼ��
	USART4_Config();                               //�Ӿ�ͨ�Ŵ��ڳ�ʼ��
	UART4_DMA_Init();                              //�Ӿ�ͨ��DMA���ճ�ʼ��
	while(1)
	{	
	}
}
