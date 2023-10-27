#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H
#include "main.h"
#include "can_receive.h"
#include "bsp_rc.h"
#include "holder_control.h"
#define chassis_positive_direction 7568 //底盘正方向电机角度值
void wheel_control(float Vx, float Vy, float W);//麦轮解算
void chassis_control(void);
void chassis_gyro(void);
void chassis_follow(void);
void gyro_stop(void);
void sentry_gyro_mode(void);
void translation_L(void);
void translation_R(void);
void sentry_translation_mode(void);
void Translation_G_S(void);
float Find_Y_AnglePNY_atuo_mode(void);
void chassis_adjust(void);
void holder_limit_set(void);
void sentry_gyro_mode_2(void);
void chassis_control_sentry(void);
void sentry_gyro_mode_1(void);
void sentry_start_gogogo(void);
void chassis_control_sentry_gogogo(void);
#endif
