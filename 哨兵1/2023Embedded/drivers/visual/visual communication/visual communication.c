#include "visual communication.h"
#include "usart.h"
Visual_Data_ Visual_Data;
float visual_coefficient=0.3;
uint8_t visual_data[6];
void UART4_DMA_Init( void )
{		
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA1, ENABLE );
	DMA_InitTypeDef xCom5DMAInit;
	DMA_DeInit( DMA1_Stream2 );
	xCom5DMAInit.DMA_Channel = DMA_Channel_4;

	xCom5DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;//方向外设到存储器

	xCom5DMAInit.DMA_PeripheralBaseAddr  = (uint32_t)&(UART4->DR);
	xCom5DMAInit.DMA_Memory0BaseAddr     = (uint32_t)visual_data;
	xCom5DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
	xCom5DMAInit.DMA_BufferSize = 6;
	xCom5DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	xCom5DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
	xCom5DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	xCom5DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	xCom5DMAInit.DMA_Mode = DMA_Mode_Circular;
	xCom5DMAInit.DMA_Priority = DMA_Priority_Medium;
	xCom5DMAInit.DMA_FIFOMode = DMA_FIFOMode_Enable;
	xCom5DMAInit.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	xCom5DMAInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	xCom5DMAInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	USART_DMACmd( UART4, USART_DMAReq_Rx, ENABLE );
	USART_DMACmd( UART4, USART_DMAReq_Tx, ENABLE );
	DMA_Init( DMA1_Stream2, &xCom5DMAInit );	
	DMA_Cmd( DMA1_Stream2, ENABLE); 
}
void visual_receive(void)
{
	if(visual_data[0]==0x80&&visual_data[5]==0x7F)
	{
		Visual_Data.Pitch_High=visual_data[1];
		Visual_Data.Pitch_Low =visual_data[2];
		Visual_Data.Yaw_High  =visual_data[3];
		Visual_Data.Yaw_Low   =visual_data[4];
		Visual_Data.Visual_Pitch= ((float)(int16_t)(Visual_Data.Pitch_High<<8|Visual_Data.Pitch_Low)*90/32767)*visual_coefficient;
		Visual_Data.Visual_Yaw  =-((float)(int16_t)(Visual_Data.Yaw_High  <<8|Visual_Data.Yaw_Low  )*90/32767)*visual_coefficient;
	}
	else
	{
		Visual_Data.Visual_Pitch=0;
		Visual_Data.Visual_Yaw  =0;
	}
}
/*视觉数据发送*/
void visual_sent(void)
{
	uint8_t a[4];
  a[0]=0x80;
	a[1]=1;
	a[2]=15;
	a[3]=0x7F;
	for(uint8_t i=0;i<4;i++)
	{
		USART_SendData( UART4, a[i] );
		while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
	}
}

/*视觉数据接收*/
void UART4_IRQHandler(void)
{
	uint8_t ucTemp;
	if(USART_GetITStatus(UART4,USART_IT_IDLE)!=RESET)
	{	
		ucTemp = UART4->SR ;
		ucTemp = UART4->DR ;
		DMA_Cmd(DMA2_Stream2,DISABLE);
		DMA_ClearFlag(DMA2_Stream2, DMA_IT_TCIF0);
		DMA2_Stream2->NDTR = 10;	
		DMA_Cmd(DMA2_Stream2,ENABLE);
		visual_receive();
		visual_data[0]=visual_data[1]=visual_data[2]=visual_data[3]=0;
	}
}













