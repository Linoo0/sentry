#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H
#include "main.h"
#include "can_receive.h"
#include "bsp_rc.h"
#include "holder_control.h"
#define chassis_positive_direction 3410 //�������������Ƕ�ֵ
void chassis_control(void);
void chassis_gyro(void);
void chassis_follow(void);
#endif
