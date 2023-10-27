#include "tim.h"
#include "chassis_control.h"
#include "holder_control.h"
#include "usart.h"
#include "rifle.h"
#include "packet.h"
#include "imu_data_decode.h"
#include "can_receive.h"
void TIM3_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 			// �����ж���Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;	 		// ������ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // ���������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //����ʱ��
  TIM_TimeBaseStructure.TIM_Period = 50-1;//5ms       
  TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;	
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;  // ����ʱ�ӷ�Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  // ������ʽΪ���ϼ���
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);	// �����ʱ�������жϱ�־λ
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);	// ������ʱ�������ж�
	TIM_Cmd(TIM3, ENABLE);		// ʹ�ܶ�ʱ��
}
extern signed short int h_2;
extern signed short int d_1;
void TIM5_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		    // �����ж���Ϊ4
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn; 			// �����ж���Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	 		// ������ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // ���������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //����ʱ��
  TIM_TimeBaseStructure.TIM_Period = 3000-1;//3ms       
  TIM_TimeBaseStructure.TIM_Prescaler = 84-1;	
  TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;  // ����ʱ�ӷ�Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;  // ������ʽΪ���ϼ���
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);	// �����ʱ�������жϱ�־λ
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);	// ������ʱ�������ж�
	TIM_Cmd(TIM5, ENABLE);		// ʹ�ܶ�ʱ��
}
/*���Ƴ���ʱ��*/
extern float holder_place[4];/*��̨����*/
extern float actual_speed[8];/*���̲���*/
extern float rifle_data[6];         /*Ħ���ֲ���*/
extern FRICTION_WHEEL_ FRICTION_WHEEL[2];
extern PID_TypeDef Pid[8];
signed short int rr;
extern MOTOR_FEEDBACK_ MOTOR_FEEDBACK[4];
extern moto_measure_t MOTOR_DATA[4];
void  TIM3_IRQHandler (void)
{
	if ( TIM_GetITStatus( TIM3, TIM_IT_Update) != RESET ) 
	{	
		
		rr=pid_calculate( &Pid[motor_1], MOTOR_FEEDBACK[motor_1].speed_rpm, 500);
		Motor_Set_Current(rr,0,0,0);
		//Friction_Set_Current(rr,0);
//	  chassis_control();                                     /*���̿���*/
//		holder_control();                                      /*��̨����*/
//		rifle_control();                                       /*�������*/
//		CAN2_Set_Current(0,h_2,d_1,0);                         /*pitch�Ͳ��̱���ͬʱ����*/
		zero_drift_judge();                                    /*ң������Ư����*/
		offline_contrl_judge();                                /*ң�������ߴ���*/
		TIM_ClearITPendingBit(TIM3 , TIM_IT_Update);
		sent_string(USART6,(char *)rifle_data,sizeof(rifle_data));//�ٶȲ���	
		visual_sent();
	}		 	
}

/*���������ݻ�ȡ*/
float holder_place[4];
extern CascadePID double_pid[2];
uint8_t ID;
int16_t Acc[3];
int16_t Gyo[3];
int16_t Mag[3];
float Eular[3];
float Quat[4];
int32_t Pressure;
void  TIM5_IRQHandler (void)
{
	if ( TIM_GetITStatus( TIM5, TIM_IT_Update) != RESET ) 
	{	
		get_raw_acc(Acc);
    get_raw_gyo(Gyo);
    get_raw_mag(Mag);
    get_eular(Eular);
    get_quat(Quat);
    get_id(&ID);
		holder_place[0]=Eular[2];
		holder_place[1]=double_pid[holder_Yaw].outer.target;
		holder_place[2]=Eular[1];
		holder_place[3]=double_pid[holder_Pitch].outer.target;
		TIM_ClearITPendingBit(TIM5 , TIM_IT_Update);  
	}		 	
}


