#ifndef __BSP_RC_H
#define __BSP_RC_H
#include "main.h"
#include <stdio.h>

#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)

#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<7)

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

typedef __packed struct
{
 __packed struct 
 {
 uint16_t ch0;
 uint16_t ch1;
 uint16_t ch2;
 uint16_t ch3;
 uint8_t s1;
 uint8_t s2;
 uint16_t a;
 }rc;
 __packed struct 
 {
 int16_t x;
 int16_t y;
 int16_t z;
 uint8_t press_l;
 uint8_t press_r;
 }mouse;
 __packed struct 
 {
 uint16_t v;
 }key;
}RC_Ctl_t;

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void zero_drift_judge(void);
void offline_contrl_judge(void);
void Remote_control_data_reception(void);
#endif
