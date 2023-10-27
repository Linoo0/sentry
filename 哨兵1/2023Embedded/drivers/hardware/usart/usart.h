#ifndef __USART_C
#define __USART_C
#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include "packet.h"
#include "visual communication.h"
void USART4_Config(void);
void USART5_Config(void);
void USART6_Config(void);
void sent_string(USART_TypeDef * uartx,char *str,int size);
#endif
