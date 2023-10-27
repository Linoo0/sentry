#include "stm32f4xx.h"
#include "pid.h"
#include "can.h"
#include "can_receive.h"
#include "packet.h"
#include "imu_data_decode.h"
#include "rc.h"
#include "usart.h"
#include "tim.h"
#include "pid_config.h"
#include "crc.h"
#include "referee.h"
int main()
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//中断分组
	imu_data_decode_init();                        //陀螺仪初始化
	USART5_Config();                               //陀螺仪串口初始化
	CRC_init();                                    //crc校验初始化
  USART1_Init();                                 //裁判系统串口初始化
	pid_parameter_setting();                       //pid参数初始化
	RC_Init();                                     //遥控器初始化
	TIM5_Config();                                 //陀螺仪数据读取定时器 3ms
  CAN1_Config();                                 //CAN1：底盘四个电机，云台yaw轴
	CAN2_Config();                                 //CAN2：云台pitch轴
  TIM3_Config();                                 //机器人操作运行定时器 5ms
	USART6_Config();                               //vofa+串口初始化
	USART4_Config();                               //视觉通信串口初始化
	UART4_DMA_Init();                              //视觉通信DMA接收初始化
	while(1)
	{	
	}
}
