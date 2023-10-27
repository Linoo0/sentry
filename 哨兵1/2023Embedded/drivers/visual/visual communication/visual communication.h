#ifndef __VISUAL_COMMUNICATION_H
#define __VISUAL_COMMUNICATION_H
#include "stm32f4xx.h"
typedef struct _Visual_Data
{
	uint8_t Pitch_High;
	uint8_t Pitch_Low;
	uint8_t Yaw_High;
	uint8_t Yaw_Low;
	float Visual_Pitch;
	float Visual_Yaw;
}Visual_Data_;

void visual_receive(void);
void visual_sent(void);
void UART4_DMA_Init( void );
#endif
