#include "vofa.h"
void sent_string(UART_HandleTypeDef *huart,char *str,int size)
{
	char tail[4]  = {0x00, 0x00, 0x80, 0x7f};
	for(uint8_t i=0;i<size;i++)
	{
		HAL_UART_Transmit( huart, (uint8_t *)&str[i],1,0xffff );
	}
	for(uint8_t i=0;i<4;i++)
	{
		HAL_UART_Transmit( huart, (uint8_t *)&tail[i],1,0xffff );
	}
}	
