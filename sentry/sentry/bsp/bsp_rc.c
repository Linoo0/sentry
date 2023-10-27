#include "bsp_rc.h"
#include "usart.h"
uint8_t zero_drift_sign=0;
uint8_t num=0;
uint16_t actual_ch0;
uint16_t actual_ch1;
uint16_t actual_ch2;
uint16_t actual_ch3;
uint8_t offline_contrl=0;
uint8_t offline_contrl_sign=0;
RC_Ctl_t RC_CtrlData;
uint8_t sbus_rx_buffer[2][RC_FRAME_LENGTH];
void RemoteDataProcess(uint8_t *pData)//��ԭʼ��dma�յ������ݰ���ң����������Э��ƴ�ӳ�������ң��������
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
 RC_CtrlData.rc.a= ((int16_t)pData[16]) | ((int16_t)pData[17] << 8);
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
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //ʹ��DMA���ڽ���
    SET_BIT(huart2.Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart2_rx);
    {
        __HAL_DMA_DISABLE(&hdma_usart2_rx);
    }
            hdma_usart2_rx.Instance->PAR = (uint32_t) & (USART2->DR);

    //memory buffer 1
    //�ڴ滺����1
    hdma_usart2_rx.Instance->M0AR = (uint32_t)(rx1_buf);

    //memory buffer 2
    //�ڴ滺����2
    hdma_usart2_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //���ݳ���
    hdma_usart2_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(hdma_usart2_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart2_rx);

}
void Remote_control_data_reception(void)
{
	offline_contrl=0;
	 if(huart2.Instance->SR & UART_FLAG_RXNE)//���յ�����
    {
        __HAL_UART_CLEAR_PEFLAG(&huart2);
    }
     if(USART2->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart2);

        if ((hdma_usart2_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart2_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart2_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart2_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //�趨������1
            hdma_usart2_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart2_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                RemoteDataProcess(sbus_rx_buffer[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //ʧЧDMA
            __HAL_DMA_DISABLE(&hdma_usart2_rx);

            //get receive data length, length = set_data_length - remain_length
            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart2_rx.Instance->NDTR;

            //reset set_data_lenght
            //�����趨���ݳ���
            hdma_usart2_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //�趨������0
            DMA1_Stream5->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart2_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //����ң��������
                RemoteDataProcess(sbus_rx_buffer[1]);
            }
        }
    }

}



