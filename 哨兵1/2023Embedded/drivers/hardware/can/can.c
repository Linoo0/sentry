#include "can.h"
#include "can_receive.h"
void CAN1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOA, ENABLE);
	
  GPIO_PinAFConfig(GPIOA,  GPIO_PinSource11,  GPIO_AF_CAN1);
  GPIO_PinAFConfig(GPIOA,  GPIO_PinSource12,  GPIO_AF_CAN1); 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	NVIC_InitTypeDef NVIC_RX_InitStructure;
	NVIC_RX_InitStructure.NVIC_IRQChannel =CAN1_RX0_IRQn;	 
	NVIC_RX_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;		
	NVIC_RX_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	NVIC_RX_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_RX_InitStructure);
	
	NVIC_InitTypeDef NVIC_TX_InitStructure;
	NVIC_TX_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
	NVIC_TX_InitStructure.NVIC_IRQChannelPreemptionPriority =4;		
	NVIC_TX_InitStructure.NVIC_IRQChannelSubPriority = 0;				
	NVIC_TX_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_TX_InitStructure);
	
	CAN_InitTypeDef        CAN_InitStructure;
	/************************CANͨ�Ų�������**********************************/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 , ENABLE);

	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	/*CAN��Ԫ��ʼ��*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  �ر�ʱ�䴥��ͨ��ģʽʹ��
	CAN_InitStructure.CAN_ABOM=DISABLE;			   //MCR-ABOM  �Զ����߹���  //���մ���һ��ֵ�Զ����߲������շ�
	CAN_InitStructure.CAN_AWUM=DISABLE;			   //MCR-AWUM  ʹ���Զ�����ģʽ
	CAN_InitStructure.CAN_NART=ENABLE;			   //MCR-NART  ��ֹ�����Զ��ش�	  DISABLE-�Զ��ش�
	CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  ����FIFO ����ģʽ  DISABLE-���ʱ�±��ĻḲ��ԭ�б���  
	CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  ����FIFO���ȼ� DISABLE-���ȼ�ȡ���ڱ��ı�ʾ�� 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //�ػ�����ģʽ CAN_Mode_Normal    CAN_Mode_LoopBack
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW ����ͬ����Ծ��� 2��ʱ�䵥Ԫ 
	/* ss=1 bs1=4 bs2=2 λʱ����Ϊ(1+4+2) �����ʼ�Ϊʱ������tq*(1+4+2)  */
	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq;		   //BTR-TS1 ʱ���1 ռ����4��ʱ�䵥Ԫ
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;		   //BTR-TS1 ʱ���2 ռ����2��ʱ�䵥Ԫ	
	/* CAN Baudrate = 1 MBps (1MBps��Ϊstm32��CAN�������) (CAN ʱ��Ƶ��Ϊ APB 1 = 42 MHz) */
	CAN_InitStructure.CAN_Prescaler =3;		   ////BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 42/(1+7+4)/3=1 Mbps
	CAN_Init(CAN1, &CAN_InitStructure);
	
	
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	/*CANɸѡ����ʼ��*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;						//ɸѡ����1
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//����������ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//ɸѡ��λ��Ϊ����32λ��
	/* ʹ��ɸѡ�������ձ�־�����ݽ��бȶ�ɸѡ����չID�������µľ����������ǵĻ��������FIFO0�� */
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000; //((((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF0000)>>16;		//Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x0000;//(((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000;			//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;			//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//ɸѡ����������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//ʹ��ɸѡ��
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CANͨ���ж�ʹ��*/
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
}

void CAN2_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOB, ENABLE);
	
  GPIO_PinAFConfig(GPIOB,  GPIO_PinSource12,  GPIO_AF_CAN2);
  GPIO_PinAFConfig(GPIOB,  GPIO_PinSource13,  GPIO_AF_CAN2); 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel =CAN2_RX0_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;		
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitTypeDef NVIC_TX_InitStructure;
	NVIC_TX_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
	NVIC_TX_InitStructure.NVIC_IRQChannelPreemptionPriority =4;		
	NVIC_TX_InitStructure.NVIC_IRQChannelSubPriority = 0;				
	NVIC_TX_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_TX_InitStructure);
	
	CAN_InitTypeDef        CAN_InitStructure;
	/************************CANͨ�Ų�������**********************************/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 |RCC_APB1Periph_CAN2, ENABLE);

	CAN_DeInit(CAN2);
	CAN_StructInit(&CAN_InitStructure);
	/*CAN��Ԫ��ʼ��*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  �ر�ʱ�䴥��ͨ��ģʽʹ��
	CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  �Զ����߹���  //���մ���һ��ֵ�Զ����߲������շ�
	CAN_InitStructure.CAN_AWUM=ENABLE;			   //MCR-AWUM  ʹ���Զ�����ģʽ
	CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  ��ֹ�����Զ��ش�	  DISABLE-�Զ��ش�
	CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  ����FIFO ����ģʽ  DISABLE-���ʱ�±��ĻḲ��ԭ�б���  
	CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  ����FIFO���ȼ� DISABLE-���ȼ�ȡ���ڱ��ı�ʾ�� 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //�ػ�����ģʽ CAN_Mode_Normal    CAN_Mode_LoopBack
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW ����ͬ����Ծ��� 2��ʱ�䵥Ԫ 
	/* ss=1 bs1=4 bs2=2 λʱ����Ϊ(1+4+2) �����ʼ�Ϊʱ������tq*(1+4+2)  */
	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq;		   //BTR-TS1 ʱ���1 ռ����4��ʱ�䵥Ԫ
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;		   //BTR-TS1 ʱ���2 ռ����2��ʱ�䵥Ԫ	
	/* CAN Baudrate = 1 MBps (1MBps��Ϊstm32��CAN�������) (CAN ʱ��Ƶ��Ϊ APB 1 = 42 MHz) */
	CAN_InitStructure.CAN_Prescaler =3;		   ////BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 42/(1+4+2)/6=1 Mbps
	CAN_Init(CAN2, &CAN_InitStructure);
	
	
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	/*CANɸѡ����ʼ��*/
	CAN_FilterInitStructure.CAN_FilterNumber=14;						//ɸѡ����14
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//����������ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//ɸѡ��λ��Ϊ����32λ��
	/* ʹ��ɸѡ�������ձ�־�����ݽ��бȶ�ɸѡ����չID�������µľ����������ǵĻ��������FIFO0�� */
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000; //((((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF0000)>>16;		//Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x0000;//(((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000;			//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;			//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//ɸѡ����������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//ʹ��ɸѡ��
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CANͨ���ж�ʹ��*/
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
}
/*CAN1�����ݽ����ж�*/
extern CanRxMsg rx_message;
void  CAN1_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	  CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
		Motor_ReadData();
	  Holder_ReaData_yaw();
	}
}
void CAN1_TX_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)
	{
		
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	}
}
/*CAN2�����ݽ����ж�*/
extern CanRxMsg rx_message2;
void  CAN2_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	{
		 CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
	   CAN_Receive(CAN2, CAN_FIFO0, &rx_message2);
     Holder_ReaData_pitch();
		 Friction_ReadData();
		 Dial_ReadData();
	}
}
void CAN2_TX_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET)
	{	
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
	}
}


