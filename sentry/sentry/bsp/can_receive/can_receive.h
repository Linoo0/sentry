#ifndef __CAN_RECEVICE_H
#define __CAN_RECEVICE_H
#include "main.h"
#include "pid.h"
#define MOTOR_ID_READ1 0x201
#define MOTOR_ID_READ2 0x202
#define MOTOR_ID_READ3 0x203
#define MOTOR_ID_READ4 0x204
#define wheel_axlespacing 1  //前后轴距
#define wheel_spacing     0.9  //左右轮距
#define HOLDER_MOTOR  6020
enum motor_num
{
	motor_1=0,
	motor_2,
	motor_3,
	motor_4,
  follow ,
	friction_left,
	friction_right,
	dial,
};
enum holder_num
{
	holder_Yaw  =0,
	holder_Pitch=1,
};

enum friction_num
{
	left  =0,
	right =1,
};
typedef struct _MOTOR_FEEDBACK
{
	uint16_t angle_value;//转子机械角度
	float speed_rpm;//转子转速
	int16_t real_current;//实际电流
	int8_t temperature;//温度
	uint8_t real_angle;
}MOTOR_FEEDBACK_;

typedef struct _HOLDER_FEEDBACK//6020
{
	uint16_t angle_value;//机械角度
	float speed_rpm;//转速
	int16_t real_current;//实际转矩电流
	int8_t temperature;//温度
	float real_angle;
}HOLDER_FEEDBACK_;

typedef struct _FRICTION_WHEEL//摩擦轮
{
	uint16_t angle_value;//转子机械角度
	float speed_rpm;//转子转速
	int16_t real_current;//实际电流
	int8_t temperature;//温度
	uint8_t real_angle;
}FRICTION_WHEEL_;

typedef struct _DIAL_MOTOR//拨盘
{
	uint16_t angle_value;//转子机械角度
	float speed_rpm;//转子转速
	int16_t real_current;//实际电流
	int8_t temperature;//温度
	uint8_t real_angle;
}DIAL_MOTOR_;
void can_filter_init(void);
void Holder_Set_Current_yaw(signed short int i1);
void Holder_Set_Current_pitch(signed short int i1);
void Motor_Set_Current(signed short int i1,signed short int i2,signed short int i3,signed short int i4);
void Friction_Set_Current(signed short int i1,signed short int i2);
void CAN2_Set_Current(signed short int i1,signed short int i2,signed short int i3,signed short int i4);
void Holder_ReaData_yaw(void);
void Holder_ReaData_pitch(void);
void Motor_ReadData(void);
void Friction_ReadData(void);
void Dial_ReadData(void);
#endif
