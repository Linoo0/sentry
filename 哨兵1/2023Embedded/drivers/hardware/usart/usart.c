#include "usart.h"
uint8_t offline_gyro=0;
/*串口4用于视觉通信*/
void USART4_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2 ;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_UART4,ENABLE);
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOA,ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4);
	GPIO_InitTypeDef gpioa;
	gpioa.GPIO_OType=GPIO_OType_PP;
	gpioa.GPIO_Speed=GPIO_Speed_50MHz;
	gpioa.GPIO_Mode=GPIO_Mode_AF;
	gpioa.GPIO_Pin=GPIO_Pin_0;
	GPIO_Init(GPIOA,&gpioa);
	gpioa.GPIO_OType=GPIO_OType_PP;
	gpioa.GPIO_Speed=GPIO_Speed_50MHz;
	gpioa.GPIO_Mode=GPIO_Mode_AF;
	gpioa.GPIO_Pin=GPIO_Pin_1;
	GPIO_Init(GPIOA,&gpioa);
	USART_InitTypeDef uart4;
	uart4.USART_BaudRate=115200;
	uart4.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	uart4.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	uart4.USART_Parity=USART_Parity_No;
	uart4.USART_StopBits=USART_StopBits_1;
	uart4.USART_WordLength=USART_WordLength_8b;
	USART_Init(UART4,&uart4);
	USART_Cmd(UART4,ENABLE);
	USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);
}	
/*哨兵vofa+*/
void USART6_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 10;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_USART6,ENABLE);
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOC,ENABLE);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);
	GPIO_InitTypeDef gpioc;
	gpioc.GPIO_OType=GPIO_OType_PP;
	gpioc.GPIO_Speed=GPIO_Speed_50MHz;
	gpioc.GPIO_Mode=GPIO_Mode_AF;
	gpioc.GPIO_Pin=GPIO_Pin_6;
	GPIO_Init(GPIOC,&gpioc);
	gpioc.GPIO_OType=GPIO_OType_PP;
	gpioc.GPIO_Speed=GPIO_Speed_50MHz;
	gpioc.GPIO_Mode=GPIO_Mode_AF;
	gpioc.GPIO_Pin=GPIO_Pin_7;
	GPIO_Init(GPIOC,&gpioc);
	
	USART_InitTypeDef uart6;
	uart6.USART_BaudRate=115200;
	uart6.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	uart6.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	uart6.USART_Parity=USART_Parity_No;
	uart6.USART_StopBits=USART_StopBits_1;
	uart6.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART6,&uart6);
	USART_Cmd(USART6,ENABLE);
	USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);
}

/*陀螺仪串口*/
void USART5_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	RCC_APB1PeriphClockCmd (RCC_APB1Periph_UART5,ENABLE);
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD,ENABLE);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5);
	GPIO_InitTypeDef gpioc,gpiod;
	gpioc.GPIO_OType=GPIO_OType_PP;
	gpioc.GPIO_Speed=GPIO_Speed_100MHz;
	gpioc.GPIO_Mode=GPIO_Mode_AF;
	gpioc.GPIO_Pin=GPIO_Pin_12;
	GPIO_Init(GPIOC,&gpioc);
	gpiod.GPIO_OType=GPIO_OType_PP;
	gpiod.GPIO_Speed=GPIO_Speed_100MHz;
	gpiod.GPIO_Mode=GPIO_Mode_AF;
	gpiod.GPIO_Pin=GPIO_Pin_2;
	GPIO_Init(GPIOD,&gpiod);
	
	USART_InitTypeDef uart5;
	uart5.USART_BaudRate=921600;
	uart5.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	uart5.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	uart5.USART_Parity=USART_Parity_No;
	uart5.USART_StopBits=USART_StopBits_1;
	uart5.USART_WordLength=USART_WordLength_8b;
	USART_Init(UART5,&uart5);
	
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART5,ENABLE);
}

/*串口6用于哨兵vofa+*/
int fputc(int ch, FILE *f)//重定义printf；
{
	USART_SendData(USART6, (uint8_t) ch);
	while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);	
	return (ch);
}

int fgetc(FILE *f)//重定义scanf；
{
		while (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == RESET);
		return (int)USART_ReceiveData(USART6);
}

void USART6_IRQHandler(void)
{
  uint8_t ucTemp;
	if(USART_GetITStatus(USART6,USART_IT_RXNE)!=RESET)
	{		
		ucTemp = USART_ReceiveData( USART6 );
	}	 
}
void sent_string(USART_TypeDef * uartx,char *str,int size)
{
	char tail[4]  = {0x00, 0x00, 0x80, 0x7f};
	for(uint8_t i=0;i<size;i++)
	{
		USART_SendData( uartx, str[i] );
		while (USART_GetFlagStatus(uartx, USART_FLAG_TXE) == RESET);
	}
	for(uint8_t i=0;i<4;i++)
	{
		USART_SendData( uartx, tail[i] );
		while (USART_GetFlagStatus(uartx, USART_FLAG_TXE) == RESET);
	}
	while (USART_GetFlagStatus(uartx, USART_FLAG_TXE) == RESET);
}	
/*陀螺仪数据接收中断*/

void UART5_IRQHandler(void)
{
	if(USART_GetITStatus(UART5,USART_IT_RXNE)!=RESET)
	{		 	
		offline_gyro=0;
		uint8_t ch;
		ch =USART_ReceiveData( UART5 );
		Packet_Decode(ch);
	}	 
}
