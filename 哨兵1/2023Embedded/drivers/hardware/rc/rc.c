#include "rc.h"

uint8_t zero_drift_sign=0;
uint8_t num=0;
uint16_t actual_ch0;
uint16_t actual_ch1;
uint16_t actual_ch2;
uint16_t actual_ch3;
uint8_t offline_contrl=0;
uint8_t offline_contrl_sign=0;
volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH]; //double sbus rx buffer to save data
RC_Ctl_t RC_CtrlData;
void RC_Init(void)
{
 /* -------------- Enable Module Clock Source ----------------------------*/
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1, ENABLE);
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
 GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_USART2);
 /* -------------- Configure GPIO ---------------------------------------*/
	 
	 GPIO_InitTypeDef gpio;
	 USART_InitTypeDef usart2;
	 gpio.GPIO_Pin = GPIO_Pin_3 ;
	 gpio.GPIO_Mode = GPIO_Mode_AF;
	 gpio.GPIO_OType = GPIO_OType_PP;
	 gpio.GPIO_Speed = GPIO_Speed_100MHz;
	 gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	 GPIO_Init(GPIOA, &gpio);
	 
	 USART_DeInit(USART2);
	 usart2.USART_BaudRate = 100000;
	 usart2.USART_WordLength = USART_WordLength_8b;
	 usart2.USART_StopBits = USART_StopBits_1;
	 usart2.USART_Parity = USART_Parity_Even;
	 usart2.USART_Mode = USART_Mode_Rx;
	 usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	 USART_Init(USART2,&usart2);
	 
	 USART_Cmd(USART2,ENABLE);
	 USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	 
 /* -------------- Configure NVIC ---------------------------------------*/
	 
	 NVIC_InitTypeDef nvic;
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	 nvic.NVIC_IRQChannel = /*DMA1_Stream5_IRQn*/  USART2_IRQn;
	 nvic.NVIC_IRQChannelPreemptionPriority = 3;
	 nvic.NVIC_IRQChannelSubPriority = 0;
	 nvic.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&nvic);
	 
 /* -------------- Configure DMA -----------------------------------------*/
	 
	 DMA_InitTypeDef dma;
	 DMA_DeInit(DMA1_Stream5);
	 dma.DMA_Channel = DMA_Channel_4;
	 dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
	 dma.DMA_Memory0BaseAddr = (uint32_t)&sbus_rx_buffer[0][0];
	 dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	 dma.DMA_BufferSize = 18;
	 dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	 dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	 dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	 dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	 dma.DMA_Mode = DMA_Mode_Circular;
	 dma.DMA_Priority = DMA_Priority_VeryHigh;
	 dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	 dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	 dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	 dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	 DMA_DoubleBufferModeConfig(DMA1_Stream5,(uint32_t)&sbus_rx_buffer[1][0], DMA_Memory_0); //first used memory configuration
	 DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);
	 DMA_Init(DMA1_Stream5,&dma);
	 USART_ITConfig(USART2, USART_IT_IDLE, ENABLE); //usart rx idle interrupt enabled
	 DMA_Cmd(DMA1_Stream5,ENABLE);
	 
 }
void RemoteDataProcess(uint8_t *pData)//将原始的dma收到的数据按照遥控器的数据协议拼接成完整的遥控器数据
{
 if(pData == NULL)
 {
		return;
 }
 
 RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
 RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
 RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
 RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
 RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
 RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);
 RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
 RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
 RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 
 RC_CtrlData.mouse.press_l = pData[12];
 RC_CtrlData.mouse.press_r = pData[13];
 RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);
		if(zero_drift_sign==1)
	 {
		actual_ch0=actual_ch1=actual_ch2=actual_ch3=1024;
	 }
	 else
	{
		actual_ch0=RC_CtrlData.rc.ch0;
		actual_ch1=RC_CtrlData.rc.ch1;
		actual_ch2=RC_CtrlData.rc.ch2;
	}
}
void USART2_IRQHandler(void)
{ 
	offline_contrl=0;
 uint16_t this_time_rx_len = 0;	//当前剩余数据长度
 if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)	//判断是否为空闲中断
    {
        USART_ReceiveData(USART2);	//清除空闲中断标志位
        if(DMA_GetCurrentMemoryTarget(DMA1_Stream5) == DMA_Memory_0)	//获取当前目标内存是否为 DMA_Memory_0
        {
            //重新设置DMA
            DMA_Cmd(DMA1_Stream5, DISABLE);
					  this_time_rx_len = DMA_GetCurrDataCounter(DMA1_Stream5);	//获取当前剩余数据量					
					 DMA_SetCurrDataCounter(DMA1_Stream5, 18);	//重新设置数据量					  
					 DMA_Cmd(DMA1_Stream5, ENABLE);
            if(this_time_rx_len == 18)	//接收成功18个字节长度
            {
                //处理遥控器数据
                RemoteDataProcess((uint8_t*)sbus_rx_buffer[1]);	
            }
        }
        else	//获取当前目标内存是否为 DMA_Memory_1
        {
            //重新设置DMA
            DMA_Cmd(DMA1_Stream5, DISABLE);
					  this_time_rx_len = DMA_GetCurrDataCounter(DMA1_Stream5);		//获取当前剩余数据量
						DMA_SetCurrDataCounter(DMA1_Stream5, 18);	//重新设置数据量
						DMA_Cmd(DMA1_Stream5, ENABLE);
            if( this_time_rx_len== 18)	//接收成功18个字节长度
            {
                //处理遥控器数据
                RemoteDataProcess((uint8_t*)sbus_rx_buffer[0]);
            }
        }
    }
}
void zero_drift_judge(void)
{
	if(  RC_CtrlData.rc.ch0>=1024-8 && RC_CtrlData.rc.ch0<=1024+8 && RC_CtrlData.rc.ch1>=1024-8 && RC_CtrlData.rc.ch1<=1024+8
		&& RC_CtrlData.rc.ch2>=1024-8 && RC_CtrlData.rc.ch2<=1024+8 && RC_CtrlData.rc.ch3>=1024-8 && RC_CtrlData.rc.ch3<=1024+8 )
	{
		num++;
		if(num>=10)
		{
			zero_drift_sign=1;
			num=10;
		}
	}
	else
	{
		num=0;
		zero_drift_sign=0;
	}
	
}
void offline_contrl_judge(void)
{

	offline_contrl++;
	if(offline_contrl>=10)
	{
		offline_contrl=10;
		offline_contrl_sign=1;
	}
	else
	{
		offline_contrl_sign=0;
	}
}
