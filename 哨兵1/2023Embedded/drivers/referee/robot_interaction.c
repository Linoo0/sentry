#include "robot_interaction.h"
#define ID_data_segment  0x200      //数据段0x200~0x2FF
#define ID_sender        hero_red   //发送者ID
#define ID_oneself       sentry_red //接收者ID
extern Referee_info_t 	REF;
uint8_t robot_interaction_data[10];
void robot_interaction_data_acqulisition(void)
{
	if(REF.RobotIntereaction_Data.data_cmd_id==ID_data_segment)
	{
		if(REF.RobotIntereaction_Data.send_ID==ID_sender)
		{
			if(REF.RobotIntereaction_Data.receiver_ID==sentry_red)
			{
				memcpy(&robot_interaction_data,&REF.RobotIntereaction_Data.robot_data.data,sizeof(REF.RobotIntereaction_Data.robot_data.data));
			}
		}
	}
}
