#include "visual communication.h"
#include "usart.h"
Visual_Data_ Visual_Data;
float visual_coefficient=0.3;
uint8_t visual_data[6];
extern DMA_HandleTypeDef hdma_uart4_rx;
void Visual_init(uint8_t *rx1_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart4.Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_uart4_rx);
    {
        __HAL_DMA_DISABLE(&hdma_uart4_rx);
    }
            hdma_uart4_rx.Instance->PAR = (uint32_t) & (UART4->DR);

    //memory buffer 1
    hdma_uart4_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //data length
    //数据长度
    hdma_uart4_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer

    __HAL_DMA_ENABLE(&hdma_uart4_rx);

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
		HAL_UART_Transmit( &huart4, &a[i],1,0xffff );
	}
}

/*视觉数据接收*/
void visual_data_receive(void)
{
	  uint8_t ucTemp;	
    __HAL_UART_CLEAR_PEFLAG(&huart4);
    __HAL_DMA_DISABLE(&hdma_uart4_rx);
	  hdma_uart4_rx.Instance->NDTR = 10;
		__HAL_DMA_ENABLE(&hdma_uart4_rx);
		visual_receive();
		visual_data[0]=visual_data[1]=visual_data[2]=visual_data[3]=0;
}













