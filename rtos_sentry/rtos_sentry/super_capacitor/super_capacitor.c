#include "super_capacitor.h"
#include "can_receive.h"
#include "referee.h"
extern CAN_HandleTypeDef hcan1;
void Capacitance_Set_Current(signed short int i)
{
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef tx_message;
	tx_message.IDE=CAN_ID_STD;
	tx_message.RTR=CAN_RTR_DATA;
	tx_message.DLC=0x08;
	tx_message.StdId=0x333;
	uint8_t Data[8];
  Data[0]=i>>8;
	Data[1]=i;
	Data[2]=0;
	Data[3]=0;
	Data[4]=0;
	Data[5]=0;
	Data[6]=0;
	Data[7]=0;
	HAL_CAN_AddTxMessage(&hcan1, &tx_message, Data, &send_mail_box);
}



void Capacitance_Send(void)
{
	uint16_t MaxPower;
	MaxPower=REF_Report_CHAS_MaxPower()-5;
	if(REF_Report_CHAS_MaxPower()==0)
	{
		Capacitance_Set_Current(95);
	}
	else
	{
		Capacitance_Set_Current(MaxPower);
	}
}
