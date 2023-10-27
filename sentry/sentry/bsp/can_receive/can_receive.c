#include "can_receive.h"
MOTOR_FEEDBACK_ MOTOR_FEEDBACK[4];
FRICTION_WHEEL_ FRICTION_WHEEL[2];
DIAL_MOTOR_ DIAL_MOTOR;
float actual_speed[8];
float rifle_data[6];
extern PID_TypeDef Pid[8];
extern CascadePID double_pid[2];
HOLDER_FEEDBACK_ HOLDER_FEEDBACK[2];
uint8_t rx_data1[8];
uint8_t rx_data2[8];
static uint8_t chassis_can_send_data[8];

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
//云台yaw轴电压发送程序
void Holder_Set_Current_yaw(signed short int i1)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef tx_message2;
	tx_message2.IDE=CAN_ID_STD;
	tx_message2.RTR=CAN_RTR_DATA;
	tx_message2.DLC=0x08;
	tx_message2.StdId=0x1FF;
	uint8_t Data[8];
	Data[0]=i1>>8;
	Data[1]=i1;
	Data[2]=0;
	Data[3]=0;
	Data[4]=0;
	Data[5]=0;
	Data[6]=0;
	Data[7]=0;
	HAL_CAN_AddTxMessage(&hcan1, &tx_message2, Data, &send_mail_box);
}
//云台pitch轴电压发送程序（调云台pid时使用）
void Holder_Set_Current_pitch(signed short int i1)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef tx_message2;
	tx_message2.IDE=CAN_ID_STD;
	tx_message2.RTR=CAN_RTR_DATA;
	tx_message2.DLC=0x08;
	tx_message2.StdId=0x1FF;
	uint8_t Data[8];
  Data[0]=0;
	Data[1]=0;
	Data[2]=i1>>8;
	Data[3]=i1;
	Data[4]=0;
	Data[5]=0;
	Data[6]=0;
	Data[7]=0;
	HAL_CAN_AddTxMessage(&hcan2, &tx_message2, Data, &send_mail_box);
}
//底盘电机电流发送程序
void Motor_Set_Current(signed short int i1,signed short int i2,signed short int i3,signed short int i4)//16384
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef tx_message;
	tx_message.IDE=CAN_ID_STD;
	tx_message.RTR=CAN_RTR_DATA;
	tx_message.DLC=0x08;
	tx_message.StdId=0x200;
	
	chassis_can_send_data[0]=i1>>8;
	chassis_can_send_data[1]=i1;
	chassis_can_send_data[2]=i2>>8;
	chassis_can_send_data[3]=i2;
	chassis_can_send_data[4]=i3>>8;
	chassis_can_send_data[5]=i3;
	chassis_can_send_data[6]=i4>>8;
	chassis_can_send_data[7]=i4;
	HAL_CAN_AddTxMessage(&hcan1, &tx_message, chassis_can_send_data, &send_mail_box);
}
//摩擦轮电机电流发送程序
void Friction_Set_Current(signed short int i1,signed short int i2)//16384
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef tx_message;
	tx_message.IDE=CAN_ID_STD;
	tx_message.RTR=CAN_RTR_DATA;
	tx_message.DLC=0x04;
	tx_message.StdId=0x200;
	uint8_t Data[8];
	Data[0]=i1>>8;
	Data[1]=i1;
	Data[2]=i2>>8;
	Data[3]=i2;
	Data[4]=0;
	Data[5]=0;
	Data[6]=0;
	Data[7]=0;
	HAL_CAN_AddTxMessage(&hcan2, &tx_message, Data, &send_mail_box);
}
void CAN2_Set_Current(signed short int i1,signed short int i2,signed short int i3,signed short int i4)//拨盘与云台pitch轴需要同时发送
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef tx_message;
	tx_message.IDE=CAN_ID_STD;
	tx_message.RTR=CAN_RTR_DATA;
	tx_message.DLC=0x08;
	tx_message.StdId=0x1FF;
	uint8_t Data[8];
	Data[0]=i1>>8;
	Data[1]=i1;
	Data[2]=i2>>8;
	Data[3]=i2;
	Data[4]=i3>>8;
	Data[5]=i3;
	Data[6]=i4>>8;
	Data[7]=i4;
	HAL_CAN_AddTxMessage(&hcan2, &tx_message, Data, &send_mail_box);
}
CAN_RxHeaderTypeDef rx_message;
CAN_RxHeaderTypeDef rx_message2;
void Holder_ReaData_yaw(void) //云台反馈
{
	if(rx_message.StdId ==0x205)
	{
		HOLDER_FEEDBACK[holder_Yaw].angle_value  = rx_data1[0] << 8 | rx_data1[1];
		HOLDER_FEEDBACK[holder_Yaw].speed_rpm    =(float) (int16_t)(rx_data1[2] << 8 | rx_data1[3]);
		HOLDER_FEEDBACK[holder_Yaw].real_current = (rx_data1[4] << 8 | rx_data1[5]);
		HOLDER_FEEDBACK[holder_Yaw].temperature = rx_data1[6];
		HOLDER_FEEDBACK[holder_Yaw].real_angle=HOLDER_FEEDBACK[holder_Yaw].angle_value/8192.0f*360.0f;
	}
}
void Holder_ReaData_pitch(void) //云台反馈
{
	if(rx_message2.StdId ==0x206)
	{
		HOLDER_FEEDBACK[holder_Pitch].angle_value  = rx_data2[0] << 8 | rx_data2[1];
		HOLDER_FEEDBACK[holder_Pitch].speed_rpm    = (float)(int16_t)( rx_data2[2] << 8 | rx_data2[3]);
		HOLDER_FEEDBACK[holder_Pitch].real_current =(rx_data2[4] << 8 | rx_data2[5]);
		HOLDER_FEEDBACK[holder_Pitch].temperature = rx_data2[6];
	}
}
void Motor_ReadData(void) //底盘反馈
{
	if(rx_message.StdId == MOTOR_ID_READ1)
	{			
		MOTOR_FEEDBACK[motor_1].angle_value  = rx_data1[0] << 8 | rx_data1[1];
		MOTOR_FEEDBACK[motor_1].speed_rpm    =(float) (int16_t)(rx_data1[2] << 8 | rx_data1[3]);
		MOTOR_FEEDBACK[motor_1].real_current = (rx_data1[4] << 8 | rx_data1[5]);
		MOTOR_FEEDBACK[motor_1].temperature = rx_data1[6];
		
		MOTOR_FEEDBACK[motor_1].real_angle = MOTOR_FEEDBACK[motor_1].angle_value/8192.0f*360.0f;
	}
	
	if(rx_message.StdId == MOTOR_ID_READ2)
	{		
		MOTOR_FEEDBACK[motor_2].angle_value  = rx_data1[0] << 8 | rx_data1[1];
		MOTOR_FEEDBACK[motor_2].speed_rpm    = (float)(int16_t)( rx_data1[2] << 8 | rx_data1[3]);
		MOTOR_FEEDBACK[motor_2].real_current =(rx_data1[4] << 8 | rx_data1[5]);
		MOTOR_FEEDBACK[motor_2].temperature = rx_data1[6];
		
		MOTOR_FEEDBACK[motor_2].real_angle = MOTOR_FEEDBACK[motor_2].angle_value/8192.0f*360.0f;
	}
	
	if(rx_message.StdId == MOTOR_ID_READ3)
	{
		MOTOR_FEEDBACK[motor_3].angle_value  = rx_data1[0] << 8 | rx_data1[1];
		MOTOR_FEEDBACK[motor_3].speed_rpm    =(float)(int16_t) (rx_data1[2] << 8 | rx_data1[3]);
		MOTOR_FEEDBACK[motor_3].real_current = (rx_data1[4] << 8 | rx_data1[5]);
		MOTOR_FEEDBACK[motor_3].temperature = rx_data1[6];
		
		MOTOR_FEEDBACK[motor_3].real_angle = MOTOR_FEEDBACK[motor_3].angle_value/8192.0f*360.0f;
	}
	
	if(rx_message.StdId == MOTOR_ID_READ4)
	{	
		MOTOR_FEEDBACK[motor_4].angle_value  = rx_data1[0] << 8 | rx_data1[1];
		MOTOR_FEEDBACK[motor_4].speed_rpm    = (float)(int16_t)(rx_data1[2] << 8 | rx_data1[3]);
		MOTOR_FEEDBACK[motor_4].real_current = rx_data1[4] << 8 | rx_data1[5];
		MOTOR_FEEDBACK[motor_4].temperature = rx_data1[6];
		
		MOTOR_FEEDBACK[motor_4].real_angle = MOTOR_FEEDBACK[motor_4].angle_value/8192.0f*360.0f;
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
		FRICTION_WHEEL[left].angle_value  = rx_data2[0] << 8 | rx_data2[1];
		FRICTION_WHEEL[left].speed_rpm    =(float) (int16_t)(rx_data2[2] << 8 | rx_data2[3]);
		FRICTION_WHEEL[left].real_current = (rx_data2[4] << 8 | rx_data2[5]);
		FRICTION_WHEEL[left].temperature = rx_data2[6];
		
		FRICTION_WHEEL[left].real_angle = FRICTION_WHEEL[left].angle_value/8192.0f*360.0f;
	}
	if(rx_message2.StdId == 0x202)
	{		
		FRICTION_WHEEL[right].angle_value  = rx_data2[0] << 8 | rx_data2[1];
		FRICTION_WHEEL[right].speed_rpm    = (float)(int16_t)( rx_data2[2] << 8 | rx_data2[3]);
		FRICTION_WHEEL[right].real_current =(rx_data2[4] << 8 | rx_data2[5]);
		FRICTION_WHEEL[right].temperature = rx_data2[6];
		
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
		DIAL_MOTOR.angle_value  = rx_data2[0] << 8 | rx_data2[1];
		DIAL_MOTOR.speed_rpm    =(float) (int16_t)(rx_data2[2] << 8 | rx_data2[3]);
		DIAL_MOTOR.real_current = (rx_data2[4] << 8 | rx_data2[5]);
		DIAL_MOTOR.temperature = rx_data2[6];
		
		DIAL_MOTOR.real_angle = DIAL_MOTOR.angle_value/8192.0f*360.0f;
	}
	rifle_data[2]=DIAL_MOTOR.speed_rpm;
	rifle_data[5]=1500;
}
	

