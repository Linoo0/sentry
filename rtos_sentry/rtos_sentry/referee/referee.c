#include "referee.h"
#include "usart.h"

#define    COM5_PACKAGE_HEADER       JUDGE_FRAME_HEADER
// 裁判系统信息
Referee_info_t 	REF = {
  .IF_REF_ONL = NO,
};



bool Judge_Data_TF = FALSE;//裁判数据是否可用,辅助函数调用
uint8_t Judge_Self_ID;//当前机器人的ID
uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID

/**************裁判系统数据辅助****************/
uint16_t ShootNum;//统计发弹量,0x0003触发一次则认为发射了一颗
uint8_t PTZ_GET_SIGN=0;
bool Hurt_Data_Update = FALSE;//装甲板伤害数据是否更新,每受一次伤害置TRUE,然后立即置FALSE,给底盘闪避用


/**
  * @brief  读取裁判数据,loop中循环调用此函数来读取数据
  * @param  缓存数据
  * @retval 是否对正误判断做处理
  * @attention  在此判断帧头和CRC校验,无误再写入数据
  */
bool Judege_read_data(u8 *ReadFromUsart )
{
	bool retval_tf = FALSE;//数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误
	uint16_t judge_length;//统计一帧数据长度 
	
	int CmdID = 0;//数据命令码解析	
	
	if(ReadFromUsart == NULL)
	{
		return -1;
	}
	
	memcpy(&REF.FrameHeader,ReadFromUsart,LEN_HEADER);   //储存帧头数据  5字节
	
	if(ReadFromUsart[SOF] == JUDGE_FRAME_HEADER)                   //判断帧头是否为0xa5
	{
		if(Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)  //帧头CRC校验
		{
			judge_length = ReadFromUsart[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;	//统计一帧数据长度,用于CR16校验
			
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)//帧尾CRC16校验
			{
				retval_tf = TRUE;//数据可用
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				switch(CmdID)
				{
					case ID_game_state:     //0x0001
							 memcpy(&REF.GameState, (ReadFromUsart + DATA), LEN_game_state);
							 break;
					
					case ID_game_result:    //0x0002
							 memcpy(&REF.GameResult, (ReadFromUsart + DATA), LEN_game_result);
							 break;
					
					case ID_game_robot_survivors:    //0x0003
							 memcpy(&REF.GameRobotHP, (ReadFromUsart + DATA), LEN_game_robot_survivors);
							 break;
					
					case ID_game_missile_state:    //0x0004
							 memcpy(&REF.GameRobotmissile, (ReadFromUsart + DATA), LED_game_missile_state);
							 break;
					
					case ID_game_buff:    //0x0005
							 memcpy(&REF.Game_ICRA_buff, (ReadFromUsart + DATA), LED_game_buff);
							 break;
					
					case ID_event_data:    //0x0101
							 memcpy(&REF.EventData, (ReadFromUsart + DATA), LEN_event_data);
							 break;
					
					case ID_supply_projectile_action:    //0x0102
							 memcpy(&REF.SupplyProjectileAction, (ReadFromUsart + DATA), LEN_supply_projectile_action);
							 break;
					
					case ID_supply_warm:    //0x0104
							 memcpy(&REF.RefereeWarning, (ReadFromUsart + DATA), LEN_supply_warm);
							 break;
					
					case ID_missile_shoot_time:    //0x0105
							 memcpy(&REF.dart_remaining_time, (ReadFromUsart + DATA), LEN_missile_shoot_time);
							 break;
					
					case ID_game_robot_state:    //0x0201
               Determine_ID();
/*1*/         // Referee_Update(GameRobotStat_ID);
							 memcpy(&REF.GameRobotStat, (ReadFromUsart + DATA), LEN_game_robot_state);
							 break;
				
					case ID_power_heat_data:    //0x0202
/*2*/         // Referee_Update(PowerHeatData_ID);
							 memcpy(&REF.PowerHeatData, (ReadFromUsart + DATA), LEN_power_heat_data);
							 break;
					
					case ID_game_robot_pos:    //0x0203
							 memcpy(&REF.GameRobotPos, (ReadFromUsart + DATA), LEN_game_robot_pos);
							 break;
					
					case ID_buff_musk:    //0x0204
							 memcpy(&REF.Buff, (ReadFromUsart + DATA), LEN_buff_musk);
							 break;
					
					case ID_aerial_robot_energy:    //0x0205
							 memcpy(&REF.AerialRobotEnergy, (ReadFromUsart + DATA), LEN_aerial_robot_energy);
							 break;
					
					case ID_robot_hurt:      			//0x0206
							memcpy(&REF.RobotHurt, (ReadFromUsart + DATA), LEN_robot_hurt);
							if(REF.RobotHurt.hurt_type == 0)//非装甲板离线造成伤害
							{	
								Hurt_Data_Update = TRUE;
							}//装甲数据每更新一次则判定为受到一次伤害
							break;
					case ID_shoot_data:      			//0x0207
/*3*/         // Referee_Update(ShootSpeed_ID);
							 memcpy(&REF.ShootData, (ReadFromUsart + DATA), LEN_shoot_data);
					     break;	
					
					case ID_bullet_remaining:    //0x0208
							 memcpy(&REF.bullet_remaining, (ReadFromUsart + DATA), LEN_bullet_remaining);
							 break;
          
					case ID_rfid_status: //0x0209
               memcpy(&REF.rfid_status, (ReadFromUsart+DATA), LEN_rfid_status);
               break;
              
					case ID_dart_client_directive://0x020A
               memcpy(&REF.dart_client,(ReadFromUsart+DATA),LEN_dart_client_directive);
               break;
					
					case ID_robot_interactive_header_data://0x0301
               memcpy(&REF.RobotIntereaction_Data,(ReadFromUsart+DATA),LEN_robot_interactive_header_data);
               break;
					case ID_map_interactive_header_data: //0x303
							  memcpy(&REF.PTZ_Communication,(ReadFromUsart+DATA),LEN_map_interactive_headerdata);
								PTZ_GET_SIGN=1;
							 break;
				}
					
			}
		}
		//首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
		if(*(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + REF.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
		{
			//如果一个数据包出现了多帧数据,则再次读取
			Judege_read_data(ReadFromUsart + sizeof(xFrameHeader) + LEN_CMDID + REF.FrameHeader.DataLength + LEN_TAIL);
		}
	}
	if (retval_tf == TRUE)
	{
		Judge_Data_TF = TRUE;//辅助函数用
	}
	else		//只要CRC16校验不通过就为FALSE
	{
		Judge_Data_TF = FALSE;//辅助函数用
	}	
	return retval_tf;
}

//裁判系统发过来的数据暂存在这里
uint8_t  Judge_Buffer[ JUDGE_BUFFER_LEN ] = {0};

int Usart1_Clean_IDLE_Flag = 0;
DMA_InitTypeDef xCom5DMAInit;
/***************************裁判系统串口初始化***********************************/


extern uint32_t Refer_time ;//裁判系统
extern DMA_HandleTypeDef hdma_usart1_rx;
void referee_init(uint8_t *rx1_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }
            hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);

    //memory buffer 1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //data length
    //数据长度
    hdma_usart1_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer

    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}
void referee_IQR(void)
{
	Usart1_Clean_IDLE_Flag = USART1->SR ;
	Usart1_Clean_IDLE_Flag = USART1->DR ;
	__HAL_UART_CLEAR_PEFLAG(&huart1);
	__HAL_DMA_DISABLE(&hdma_usart1_rx);
	Judege_read_data(Judge_Buffer);
	memset(Judge_Buffer, 0, 200);
	hdma_usart1_rx.Instance->NDTR = 200;	
	__HAL_DMA_ENABLE(&hdma_usart1_rx);
}



/**
  * @brief  串口一次发送一个字节数据
  * @param  自己打包好的要发给裁判的数据
  * @retval void
  * @attention  串口移位发送
  */
void USART1_SendChar(uint8_t cData)
{
	HAL_UART_Transmit( &huart1, &cData,1,0xffff );
}

/**
 * @brief 判断裁判系统是否连接
 * @param 
 */
uint32_t Refer_time = 0;//裁判系统
bool Judge_IF_Refer_Normal(void)
{
  bool res = true;
//  if(micros() >= Refer_time)
//  {
//    res = false;
//  }
  return res;
}

void Judge_IF_REF_ONL(void)
{
  REF.IF_REF_ONL = Judge_IF_Refer_Normal();
}


//------------PowerHeatData------------//
float REF_Report_CHAS_Power(void)   //4 瞬时功率
{
  return REF.PowerHeatData.chassis_power;
}

uint16_t REF_Report_CHAS_PowerBuffer(void)   //2
{
  return REF.PowerHeatData.chassis_power_buffer;//60焦耳缓冲能量
}  

uint16_t REF_Report_Shooter_Heat(void)    //2
{
  return REF.PowerHeatData.shooter_heat0;//1号17mm枪口热量
}
uint16_t REF_Report_Shooter_Heat_2(void)    //2
{
  return REF.PowerHeatData.shooter_heat1;//2号17mm枪口热量
}
//------------ShootInfo----------------//
float REF_Report_RealShootSpeed(void)
{
  return REF.ShootData.bullet_speed;//子弹射速
}
uint8_t REF_Report_IF_shooter_output(void)
{
  return REF.GameRobotStat.mains_power_shooter_output;//shooter口输出
}

//------------GameRobotStat------------//
uint8_t REF_Report_robot_ID(void)   //1
{
  return REF.GameRobotStat.robot_id;//机器人ID
}

uint8_t REF_Report_Shoot_SpeedLimit(void)  //1
{
  return REF.GameRobotStat.shooter1_17mm_speed_limit;//速度上限
}

uint16_t REF_Report_Shoot_CoolingLimit(void)  //2
{
  return REF.GameRobotStat.shooter1_17mm_cooling_limit;//子弹热量上限
}

uint16_t REF_Report_Shoot_CoolingRate(void)  //2
{
  return REF.GameRobotStat.shooter1_17mm_cooling_rate;//子弹热量冷却速度 单位 /s
}

uint16_t REF_Report_CHAS_MaxPower(void)  //2
{
  return REF.GameRobotStat.max_chassis_power;//底盘功率限制上限
}
uint8_t REF_Report_GAME_TYPE(void)
{
	return REF.GameState.game_type;          //比赛类型
}
uint16_t REF_Report_HP(void)
{
	return REF.GameRobotStat.remain_HP;
}	
uint8_t REF_Again(void)
{
	return REF.GameState.game_progress;
}
uint16_t REF_Bullet(void)
{
	return REF.bullet_remaining.bullet_remaining_num_17mm;
}
float REF_PTZ_X(void)
{
	return REF.PTZ_Communication.target_position_x;
}
float REF_PTZ_Y(void)
{
	return REF.PTZ_Communication.target_position_y;
}
float REF_PTZ_Z(void)
{
	return REF.PTZ_Communication.target_position_z;
}
uint8_t REF_PTZ_KEY(void)
{
	return REF.PTZ_Communication.commd_keyboard;
}
uint32_t REF_Event(void)
{
	return REF.EventData.event_type;
}
//------------------RFID-------------------------//
uint8_t REF_Report_RFID_State(void)
{
  uint8_t res = 0;
  res = res | REF.rfid_status.rfid_status;
  return res;
//  return (uint8_t)REF.rfid_status.rfid_status;
}

//------------------------------------------------------------------------


void Determine_ID(void)//判断自己是哪个队伍
{
	if(REF.GameRobotStat.robot_id < 10)//本机器人的ID，红方
	{ 
		REF.ids.teammate_hero 		 	= 1;
		REF.ids.teammate_engineer  = 2;
		REF.ids.teammate_infantry3 = 3;
		REF.ids.teammate_infantry4 = 4;
		REF.ids.teammate_infantry5 = 5;
		REF.ids.teammate_plane		 	= 6;
		REF.ids.teammate_sentry		= 7;
		
		REF.ids.client_hero 		 	= 0x0101;
		REF.ids.client_engineer  = 0x0102;
		REF.ids.client_infantry3 = 0x0103;
		REF.ids.client_infantry4 = 0x0104;
		REF.ids.client_infantry5 = 0x0105;
		REF.ids.client_plane			= 0x0106;
//		REF.ids.client_sentry			= 0x0107;
		
		if     (REF.GameRobotStat.robot_id == hero_red)//不断刷新放置在比赛中更改颜色
			REF.self_client = REF.ids.client_hero;
		else if(REF.GameRobotStat.robot_id == engineer_red)
			REF.self_client = REF.ids.client_engineer;
		else if(REF.GameRobotStat.robot_id == infantry3_red)
			REF.self_client = REF.ids.client_infantry3;
		else if(REF.GameRobotStat.robot_id == infantry4_red)
			REF.self_client = REF.ids.client_infantry4;
		else if(REF.GameRobotStat.robot_id == infantry5_red)
			REF.self_client = REF.ids.client_infantry5;
		else if(REF.GameRobotStat.robot_id == plane_red)
			REF.self_client = REF.ids.client_plane;
//		else if(REF.GameRobotStat.robot_id == sentry_red)
//			REF.self_client = REF.ids.client_sentry;
	}
	else //蓝方
	{
		REF.ids.teammate_hero 		 	= 101;
		REF.ids.teammate_engineer  = 102;
		REF.ids.teammate_infantry3 = 103;
		REF.ids.teammate_infantry4 = 104;
		REF.ids.teammate_infantry5 = 105;
		REF.ids.teammate_plane		 	= 106;
		REF.ids.teammate_sentry		= 107;
		
		REF.ids.client_hero 		 	= 0x0165;
		REF.ids.client_engineer  = 0x0166;
		REF.ids.client_infantry3 = 0x0167;
		REF.ids.client_infantry4 = 0x0168;
		REF.ids.client_infantry5 = 0x0169;
		REF.ids.client_plane			= 0x016A;
		
		if     (REF.GameRobotStat.robot_id == hero_blue)
			REF.self_client = REF.ids.client_hero;
		else if(REF.GameRobotStat.robot_id == engineer_blue)
			REF.self_client = REF.ids.client_engineer;
		else if(REF.GameRobotStat.robot_id == infantry3_blue)
			REF.self_client = REF.ids.client_infantry3;
		else if(REF.GameRobotStat.robot_id == infantry4_blue)
			REF.self_client = REF.ids.client_infantry4;
		else if(REF.GameRobotStat.robot_id == infantry5_blue)
			REF.self_client = REF.ids.client_infantry5;
		else if(REF.GameRobotStat.robot_id == plane_blue)
			REF.self_client = REF.ids.client_plane;
		
	}

}

/*受伤害判断*/
uint8_t Injury_sign;
void Injury_Judgment(void)
{
	static uint16_t now_blood;
	static uint16_t last_blood;
	static uint16_t num_injury;;
	static uint8_t sign=1;
	if(sign==1)
	{
		now_blood=REF_Report_HP();
		last_blood=REF_Report_HP();
		sign=0;
	}
	now_blood=REF_Report_HP();
	if(now_blood<last_blood)
	{
		Injury_sign=1;
	}
	if(Injury_sign==1)
	{
		num_injury++;
	}
	if((num_injury>=500)&&(now_blood==last_blood))
	{
		num_injury=0;
		Injury_sign=0;
	}
	last_blood=now_blood;
}
uint8_t start=0;
void REF_STATE(void)
{
	if(REF_Again()==4)
	{
		start=1;
	}
	else
	{
		start=0;
	}
}
