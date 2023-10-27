#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H
#include "stm32f4xx.h"
#include "can_receive.h"
#include "rc.h"
#include "holder_control.h"
#define chassis_positive_direction 3410 //底盘正方向电机角度值
void chassis_control(void);
void chassis_gyro(void);
void chassis_follow(void);
#endif
