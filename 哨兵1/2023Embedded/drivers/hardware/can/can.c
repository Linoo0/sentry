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
	/************************CAN通信参数设置**********************************/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 , ENABLE);

	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	/*CAN单元初始化*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  关闭时间触发通信模式使能
	CAN_InitStructure.CAN_ABOM=DISABLE;			   //MCR-ABOM  自动离线管理  //接收错误到一定值自动离线不进行收发
	CAN_InitStructure.CAN_AWUM=DISABLE;			   //MCR-AWUM  使用自动唤醒模式
	CAN_InitStructure.CAN_NART=ENABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
	CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
	CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //回环工作模式 CAN_Mode_Normal    CAN_Mode_LoopBack
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW 重新同步跳跃宽度 2个时间单元 
	/* ss=1 bs1=4 bs2=2 位时间宽度为(1+4+2) 波特率即为时钟周期tq*(1+4+2)  */
	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq;		   //BTR-TS1 时间段1 占用了4个时间单元
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;		   //BTR-TS1 时间段2 占用了2个时间单元	
	/* CAN Baudrate = 1 MBps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 = 42 MHz) */
	CAN_InitStructure.CAN_Prescaler =3;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 42/(1+7+4)/3=1 Mbps
	CAN_Init(CAN1, &CAN_InitStructure);
	
	
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	/*CAN筛选器初始化*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;						//筛选器组1
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在掩码模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//筛选器位宽为单个32位。
	/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000; //((((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF0000)>>16;		//要筛选的ID高位 
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x0000;//(((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //要筛选的ID低位 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000;			//筛选器高16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;			//筛选器低16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//筛选器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能筛选器
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CAN通信中断使能*/
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
	/************************CAN通信参数设置**********************************/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 |RCC_APB1Periph_CAN2, ENABLE);

	CAN_DeInit(CAN2);
	CAN_StructInit(&CAN_InitStructure);
	/*CAN单元初始化*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  关闭时间触发通信模式使能
	CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  自动离线管理  //接收错误到一定值自动离线不进行收发
	CAN_InitStructure.CAN_AWUM=ENABLE;			   //MCR-AWUM  使用自动唤醒模式
	CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
	CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
	CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //回环工作模式 CAN_Mode_Normal    CAN_Mode_LoopBack
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW 重新同步跳跃宽度 2个时间单元 
	/* ss=1 bs1=4 bs2=2 位时间宽度为(1+4+2) 波特率即为时钟周期tq*(1+4+2)  */
	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq;		   //BTR-TS1 时间段1 占用了4个时间单元
	CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;		   //BTR-TS1 时间段2 占用了2个时间单元	
	/* CAN Baudrate = 1 MBps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 = 42 MHz) */
	CAN_InitStructure.CAN_Prescaler =3;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 42/(1+4+2)/6=1 Mbps
	CAN_Init(CAN2, &CAN_InitStructure);
	
	
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	/*CAN筛选器初始化*/
	CAN_FilterInitStructure.CAN_FilterNumber=14;						//筛选器组14
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在掩码模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//筛选器位宽为单个32位。
	/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000; //((((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF0000)>>16;		//要筛选的ID高位 
	CAN_FilterInitStructure.CAN_FilterIdLow= 0x0000;//(((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //要筛选的ID低位 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000;			//筛选器高16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;			//筛选器低16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//筛选器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能筛选器
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CAN通信中断使能*/
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
}
/*CAN1的数据接收中断*/
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
/*CAN2的数据接收中断*/
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


