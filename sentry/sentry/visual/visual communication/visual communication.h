#ifndef __VISUAL_COMMUNICATION_H
#define __VISUAL_COMMUNICATION_H
#include "main.h"
typedef struct _Visual_Data
{
	uint8_t Pitch_High;
	uint8_t Pitch_Low;
	uint8_t Yaw_High;
	uint8_t Yaw_Low;
	float Visual_Pitch;
	float Visual_Yaw;
}Visual_Data_;
void Visual_init(uint8_t *rx1_buf, uint16_t dma_buf_num);
void visual_receive(void);
void visual_sent(void);
void visual_data_receive(void);
#endif
