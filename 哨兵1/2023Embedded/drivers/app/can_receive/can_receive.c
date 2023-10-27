#include "can_receive.h"
MOTOR_FEEDBACK_ MOTOR_FEEDBACK[4];
FRICTION_WHEEL_ FRICTION_WHEEL[2];
DIAL_MOTOR_ DIAL_MOTOR;
float actual_speed[8];
float rifle_data[6];
extern PID_TypeDef Pid[8];
extern CascadePID double_pid[2];
HOLDER_FEEDBACK_ HOLDER_FEEDBACK[2];
moto_measure_t MOTOR_DATA[4];
//云台yaw轴电压发送程序
void Holder_Set_Current_yaw(signed short int i1)
{
	CanTxMsg tx_message2;
	tx_message2.IDE=CAN_ID_STD;
	tx_message2.RTR=CAN_RTR_DATA;
	tx_message2.DLC=0x08;
	tx_message2.StdId=0x1FF;
	
	tx_message2.Data[0]=i1>>8;
	tx_message2.Data[1]=i1;
	tx_message2.Data[2]=0;
	tx_message2.Data[3]=0;
	tx_message2.Data[4]=0;
	tx_message2.Data[5]=0;
	tx_message2.Data[6]=0;
	tx_message2.Data[7]=0;
	CAN_Transmit(CAN1,&tx_message2);
}
//云台pitch轴电压发送程序（调云台pid时使用）
void Holder_Set_Current_pitch(signed short int i1)
{
	CanTxMsg tx_message2;
	tx_message2.IDE=CAN_ID_STD;
	tx_message2.RTR=CAN_RTR_DATA;
	tx_message2.DLC=0x08;
	tx_message2.StdId=0x1FF;
	tx_message2.Data[0]=0;
	tx_message2.Data[1]=0;
	tx_message2.Data[2]=i1>>8;
	tx_message2.Data[3]=i1;
	tx_message2.Data[4]=0;
	tx_message2.Data[5]=0;
	tx_message2.Data[6]=0;
	tx_message2.Data[7]=0;
	CAN_Transmit(CAN2,&tx_message2);
}
//底盘电机电流发送程序
void Motor_Set_Current(signed short int i1,signed short int i2,signed short int i3,signed short int i4)//16384
{
	CanTxMsg tx_message;
	tx_message.IDE=CAN_ID_STD;
	tx_message.RTR=CAN_RTR_DATA;
	tx_message.DLC=0x08;
	tx_message.StdId=0x200;
	
	tx_message.Data[0]=i1>>8;
	tx_message.Data[1]=i1;
	tx_message.Data[2]=i2>>8;
	tx_message.Data[3]=i2;
	tx_message.Data[4]=i3>>8;
	tx_message.Data[5]=i3;
	tx_message.Data[6]=i4>>8;
	tx_message.Data[7]=i4;
	CAN_Transmit(CAN1,&tx_message);
}
//摩擦轮电机电流发送程序
void Friction_Set_Current(signed short int i1,signed short int i2)//16384
{
	CanTxMsg tx_message;
	tx_message.IDE=CAN_ID_STD;
	tx_message.RTR=CAN_RTR_DATA;
	tx_message.DLC=0x04;
	tx_message.StdId=0x200;
	
	tx_message.Data[0]=i1>>8;
	tx_message.Data[1]=i1;
	tx_message.Data[2]=i2>>8;
	tx_message.Data[3]=i2;
	tx_message.Data[4]=0;
	tx_message.Data[5]=0;
	tx_message.Data[6]=0;
	tx_message.Data[7]=0;
	CAN_Transmit(CAN2,&tx_message);
}
void CAN2_Set_Current(signed short int i1,signed short int i2,signed short int i3,signed short int i4)//拨盘与云台pitch轴需要同时发送
{
	CanTxMsg tx_message;
	tx_message.IDE=CAN_ID_STD;
	tx_message.RTR=CAN_RTR_DATA;
	tx_message.DLC=0x08;
	tx_message.StdId=0x1FF;
	
	tx_message.Data[0]=i1>>8;
	tx_message.Data[1]=i1;
	tx_message.Data[2]=i2>>8;
	tx_message.Data[3]=i2;
	tx_message.Data[4]=i3>>8;
	tx_message.Data[5]=i3;
	tx_message.Data[6]=i4>>8;
	tx_message.Data[7]=i4;
	CAN_Transmit(CAN2,&tx_message);
}
CanRxMsg rx_message;
void Holder_ReaData_yaw(void) //云台反馈
{
	if(rx_message.StdId ==0x205)
	{
		HOLDER_FEEDBACK[holder_Yaw].angle_value  = rx_message.Data[0] << 8 | rx_message.Data[1];
		HOLDER_FEEDBACK[holder_Yaw].speed_rpm    =(float) (int16_t)(rx_message.Data[2] << 8 | rx_message.Data[3]);
		HOLDER_FEEDBACK[holder_Yaw].real_current = (rx_message.Data[4] << 8 | rx_message.Data[5]);
		HOLDER_FEEDBACK[holder_Yaw].temperature = rx_message.Data[6];
		HOLDER_FEEDBACK[holder_Yaw].real_angle=HOLDER_FEEDBACK[holder_Yaw].angle_value/8192.0f*360.0f;
	}
}
CanRxMsg rx_message2;
void Holder_ReaData_pitch(void) //云台反馈
{
	if(rx_message2.StdId ==0x206)
	{
		HOLDER_FEEDBACK[holder_Pitch].angle_value  = rx_message2.Data[0] << 8 | rx_message2.Data[1];
		HOLDER_FEEDBACK[holder_Pitch].speed_rpm    = (float)(int16_t)( rx_message2.Data[2] << 8 | rx_message2.Data[3]);
		HOLDER_FEEDBACK[holder_Pitch].real_current =(rx_message2.Data[4] << 8 | rx_message2.Data[5]);
		HOLDER_FEEDBACK[holder_Pitch].temperature = rx_message2.Data[6];
	}
}
void Motor_ReadData(void) //底盘反馈
{
	if(rx_message.StdId == MOTOR_ID_READ1)
	{			
		MOTOR_DATA[motor_1].msg_cnt++ <= 50	?	get_moto_offset(&MOTOR_DATA[motor_1]) : GET_MODOR_DATA(&MOTOR_DATA[motor_1],motor_1);MOTOR_DATA[motor_1].msg_cnt=100;
		MOTOR_FEEDBACK[motor_1].angle_value  = rx_message.Data[0] << 8 | rx_message.Data[1];
		MOTOR_FEEDBACK[motor_1].speed_rpm    =(float) (int16_t)(rx_message.Data[2] << 8 | rx_message.Data[3]);
		MOTOR_FEEDBACK[motor_1].real_current = (rx_message.Data[4] << 8 | rx_message.Data[5]);
		MOTOR_FEEDBACK[motor_1].temperature = rx_message.Data[6];
		
		MOTOR_FEEDBACK[motor_1].real_angle = MOTOR_FEEDBACK[motor_1].angle_value/8192.0f*360.0f;
		get_total_angle(&MOTOR_DATA[motor_1]);
		GET_MODOR_DATA(&MOTOR_DATA[motor_1],motor_1);
		
	}
	
	if(rx_message.StdId == MOTOR_ID_READ2)
	{		
		MOTOR_FEEDBACK[motor_2].angle_value  = rx_message.Data[0] << 8 | rx_message.Data[1];
		MOTOR_FEEDBACK[motor_2].speed_rpm    = (float)(int16_t)( rx_message.Data[2] << 8 | rx_message.Data[3]);
		MOTOR_FEEDBACK[motor_2].real_current =(rx_message.Data[4] << 8 | rx_message.Data[5]);
		MOTOR_FEEDBACK[motor_2].temperature = rx_message.Data[6];
		
		MOTOR_FEEDBACK[motor_2].real_angle = MOTOR_FEEDBACK[motor_2].angle_value/8192.0f*360.0f;
		GET_MODOR_DATA(&MOTOR_DATA[motor_2],motor_2);
	}
	
	if(rx_message.StdId == MOTOR_ID_READ3)
	{
		MOTOR_FEEDBACK[motor_3].angle_value  = rx_message.Data[0] << 8 | rx_message.Data[1];
		MOTOR_FEEDBACK[motor_3].speed_rpm    =(float)(int16_t) (rx_message.Data[2] << 8 | rx_message.Data[3]);
		MOTOR_FEEDBACK[motor_3].real_current = (rx_message.Data[4] << 8 | rx_message.Data[5]);
		MOTOR_FEEDBACK[motor_3].temperature = rx_message.Data[6];
		
		MOTOR_FEEDBACK[motor_3].real_angle = MOTOR_FEEDBACK[motor_3].angle_value/8192.0f*360.0f;
		GET_MODOR_DATA(&MOTOR_DATA[motor_3],motor_3);
	}
	
	if(rx_message.StdId == MOTOR_ID_READ4)
	{	
		MOTOR_FEEDBACK[motor_4].angle_value  = rx_message.Data[0] << 8 | rx_message.Data[1];
		MOTOR_FEEDBACK[motor_4].speed_rpm    = (float)(int16_t)(rx_message.Data[2] << 8 | rx_message.Data[3]);
		MOTOR_FEEDBACK[motor_4].real_current = rx_message.Data[4] << 8 | rx_message.Data[5];
		MOTOR_FEEDBACK[motor_4].temperature = rx_message.Data[6];
		
		MOTOR_FEEDBACK[motor_4].real_angle = MOTOR_FEEDBACK[motor_4].angle_value/8192.0f*360.0f;
		GET_MODOR_DATA(&MOTOR_DATA[motor_4],motor_4);
	}
	actual_speed[motor_1]=MOTOR_FEEDBACK[motor_1].speed_rpm;
	actual_speed[motor_2]=MOTOR_FEEDBACK[motor_2].speed_rpm;
	actual_speed[motor_3]=MOTOR_FEEDBACK[motor_3].speed_rpm;
	actual_speed[motor_4]=MOTOR_FEEDBACK[motor_4].speed_rpm;
	actual_speed[4]=Pid[motor_1].target;
	actual_speed[5]=Pid[motor_2].target;
	actual_speed[6]=Pid[motor_3].target;
	actual_speed[7]=Pid[motor_4].target;	
}

void Friction_ReadData(void)
{
  if(rx_message2.StdId == 0x201)
	{			
		FRICTION_WHEEL[left].angle_value  = rx_message2.Data[0] << 8 | rx_message2.Data[1];
		FRICTION_WHEEL[left].speed_rpm    =(float) (int16_t)(rx_message2.Data[2] << 8 | rx_message2.Data[3]);
		FRICTION_WHEEL[left].real_current = (rx_message2.Data[4] << 8 | rx_message2.Data[5]);
		FRICTION_WHEEL[left].temperature = rx_message2.Data[6];
		
		FRICTION_WHEEL[left].real_angle = FRICTION_WHEEL[left].angle_value/8192.0f*360.0f;
	}
	if(rx_message2.StdId == 0x202)
	{		
		FRICTION_WHEEL[right].angle_value  = rx_message2.Data[0] << 8 | rx_message2.Data[1];
		FRICTION_WHEEL[right].speed_rpm    = (float)(int16_t)( rx_message2.Data[2] << 8 | rx_message2.Data[3]);
		FRICTION_WHEEL[right].real_current =(rx_message2.Data[4] << 8 | rx_message2.Data[5]);
		FRICTION_WHEEL[right].temperature = rx_message2.Data[6];
		
		FRICTION_WHEEL[right].real_angle = FRICTION_WHEEL[right].angle_value/8192.0f*360.0f;
	}
	
	rifle_data[left]=FRICTION_WHEEL[left].speed_rpm;
	rifle_data[right]=FRICTION_WHEEL[right].speed_rpm;
	rifle_data[3]=-6000;
	rifle_data[4]=6000;
}
	
void Dial_ReadData(void)
{
  if(rx_message2.StdId == 0x207)
	{			
		DIAL_MOTOR.angle_value  = rx_message2.Data[0] << 8 | rx_message2.Data[1];
		DIAL_MOTOR.speed_rpm    =(float) (int16_t)(rx_message2.Data[2] << 8 | rx_message2.Data[3]);
		DIAL_MOTOR.real_current = (rx_message2.Data[4] << 8 | rx_message2.Data[5]);
		DIAL_MOTOR.temperature = rx_message2.Data[6];
		
		DIAL_MOTOR.real_angle = DIAL_MOTOR.angle_value/8192.0f*360.0f;
	}
	rifle_data[2]=DIAL_MOTOR.speed_rpm;
	rifle_data[5]=1500;
}
	
void get_total_angle(moto_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//可能的情况
		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		res2 = p->angle - p->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		res2 = p->angle - p->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}

void GET_MODOR_DATA(moto_measure_t *ptr,int i)
{
	ptr->last_angle = ptr->angle;
	ptr->angle =  (uint16_t)rx_message.Data[0] << 8 | rx_message.Data[1];
	ptr->real_current  = (int16_t)(rx_message.Data[2] << 8 | rx_message.Data[3]);;
	ptr->speed_rpm = ptr->real_current;	//这里是因为两种电调对应位不一样的信息
	ptr->given_current = (int16_t)(rx_message.Data[4] << 8 | rx_message.Data[5])/-5;
	ptr->hall = rx_message.Data[6];
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}
void get_moto_offset(moto_measure_t *ptr)
{
	ptr->angle = (uint16_t)( rx_message.Data[0] << 8 | rx_message.Data[1]) ;
	ptr->offset_angle = ptr->angle;
}

