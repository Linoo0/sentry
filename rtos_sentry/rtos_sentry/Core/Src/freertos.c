/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "chassis_control.h"
#include "holder_control.h"
#include "rifle.h"
#include "bsp_rc.h"
#include "vofa.h"
#include "super_capacitor.h"
#include "referee.h"
#include "visual communication.h"
#include "PTZ_Communication.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern uint8_t ROBOT_MODE;//1:普通模式 2：哨兵模式
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId CHASSISHandle;
osThreadId HOLDERHandle;
osThreadId RIFLEHandle;
osThreadId RC_OFFLINEHandle;
osThreadId CAPACITANCEHandle;
osThreadId VISUAL_SENDHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void CHASSIS_TASK(void const * argument);
void HOLDER_TASK(void const * argument);
void RIFLE_TASK(void const * argument);
void RC_OFFLINE_TASK(void const * argument);
void CAPACITANCE_TASK(void const * argument);
void VISUAL_SEND_TASK(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of CHASSIS */
  osThreadDef(CHASSIS, CHASSIS_TASK, osPriorityNormal, 0, 256);
  CHASSISHandle = osThreadCreate(osThread(CHASSIS), NULL);

  /* definition and creation of HOLDER */
  osThreadDef(HOLDER, HOLDER_TASK, osPriorityNormal, 0, 256);
  HOLDERHandle = osThreadCreate(osThread(HOLDER), NULL);

  /* definition and creation of RIFLE */
  osThreadDef(RIFLE, RIFLE_TASK, osPriorityIdle, 0, 256);
  RIFLEHandle = osThreadCreate(osThread(RIFLE), NULL);

  /* definition and creation of RC_OFFLINE */
  osThreadDef(RC_OFFLINE, RC_OFFLINE_TASK, osPriorityLow, 0, 128);
  RC_OFFLINEHandle = osThreadCreate(osThread(RC_OFFLINE), NULL);

  /* definition and creation of CAPACITANCE */
  osThreadDef(CAPACITANCE, CAPACITANCE_TASK, osPriorityAboveNormal, 0, 128);
  CAPACITANCEHandle = osThreadCreate(osThread(CAPACITANCE), NULL);

  /* definition and creation of VISUAL_SEND */
  osThreadDef(VISUAL_SEND, VISUAL_SEND_TASK, osPriorityBelowNormal, 0, 256);
  VISUAL_SENDHandle = osThreadCreate(osThread(VISUAL_SEND), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_CHASSIS_TASK */
/**
* @brief Function implementing the CHASSIS thread.
* @param argument: Not used
* @retval None
*/
#define MODE_Sentry 3 //1:普通 2：gogogo 3：PTZ
/* USER CODE END Header_CHASSIS_TASK */
void CHASSIS_TASK(void const * argument)
{
  /* USER CODE BEGIN CHASSIS_TASK */
	uint32_t currentTime;
  /* Infinite loop */
  for(;;)
  {
		currentTime = xTaskGetTickCount();//
		if(ROBOT_MODE==1)
		{
			chassis_control();                                     /*底盘控制*/
		}
		else if(ROBOT_MODE==2)
		{
			#if (MODE_Sentry==1)
			
			chassis_control_sentry();
			
			#elif (MODE_Sentry==2)
			
			chassis_control_sentry_gogogo();
			
			#elif (MODE_Sentry==3)
			
			PTZ_chassis_control();
			
			#endif
		}
		vTaskDelayUntil(&currentTime, 3);//绝对延时
  }
  /* USER CODE END CHASSIS_TASK */
}

/* USER CODE BEGIN Header_HOLDER_TASK */
/**
* @brief Function implementing the HOLDER thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HOLDER_TASK */
void HOLDER_TASK(void const * argument)
{
  /* USER CODE BEGIN HOLDER_TASK */
	uint32_t currentTime;
  /* Infinite loop */
  for(;;)
  {
		currentTime = xTaskGetTickCount();//当前系统时间
		if(ROBOT_MODE==1)
		{
			holder_control();                                      /*云台控制*/
		}
		else if(ROBOT_MODE==2)
		{
			#if (MODE_Sentry==1)
			
			holder_control_sentry();
			
			#elif (MODE_Sentry==2)
			
			holder_control_sentry_gogogo();
			
			#elif (MODE_Sentry==3)
			
			PTZ_holder_control();
			
			#endif
		}
    vTaskDelayUntil(&currentTime, 3);//绝对延时
  }
  /* USER CODE END HOLDER_TASK */
}

/* USER CODE BEGIN Header_RIFLE_TASK */
/**
* @brief Function implementing the RIFLE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RIFLE_TASK */
void RIFLE_TASK(void const * argument)
{
  /* USER CODE BEGIN RIFLE_TASK */
	uint32_t currentTime;
  /* Infinite loop */
  for(;;)
  {
		currentTime = xTaskGetTickCount();//当前系统时间
		if(ROBOT_MODE==1)
		{
			rifle_control();                                       /*发射机构*/
		}
		else if(ROBOT_MODE==2)
		{
			
			#if (MODE_Sentry==1)
			
			rifle_control_sentry();
			
			#elif (MODE_Sentry==2)
			
			rifle_control_gogogo();
			
			#elif (MODE_Sentry==3)
			
			PTZ_rifle_control();
			
			#endif
		}
    vTaskDelayUntil(&currentTime, 3);//绝对延时
  }
  /* USER CODE END RIFLE_TASK */
}

/* USER CODE BEGIN Header_RC_OFFLINE_TASK */
/**
* @brief Function implementing the RC_OFFLINE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RC_OFFLINE_TASK */
void RC_OFFLINE_TASK(void const * argument)
{
  /* USER CODE BEGIN RC_OFFLINE_TASK */
	uint32_t currentTime;
	
  /* Infinite loop */
  for(;;)
  {
    currentTime = xTaskGetTickCount();//当前系统时间
		Mode_Switch();                                         /*模式切换*/
		offline_contrl_judge();                                /*遥控器掉线处理*/
		Injury_Judgment();                                     /*掉血判断*/
		visual_sign_set();                                     /*视觉掉线判断*/
    vTaskDelayUntil(&currentTime, 3);//绝对延时
  }
  /* USER CODE END RC_OFFLINE_TASK */
}

/* USER CODE BEGIN Header_CAPACITANCE_TASK */
/**
* @brief Function implementing the CAPACITANCE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAPACITANCE_TASK */
void CAPACITANCE_TASK(void const * argument)
{
  /* USER CODE BEGIN CAPACITANCE_TASK */
	uint32_t currentTime;
  /* Infinite loop */
  for(;;)
  {
		currentTime = xTaskGetTickCount();//当前系统时间
		Capacitance_Send();                   /*电容功率控制*/
    vTaskDelayUntil(&currentTime, 100);//绝对延时
  }
  /* USER CODE END CAPACITANCE_TASK */
}

/* USER CODE BEGIN Header_VISUAL_SEND_TASK */
/**
* @brief Function implementing the VISUAL_SEND thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_VISUAL_SEND_TASK */
void VISUAL_SEND_TASK(void const * argument)
{
  /* USER CODE BEGIN VISUAL_SEND_TASK */
	uint32_t currentTime;
  /* Infinite loop */
  for(;;)
  {
		currentTime = xTaskGetTickCount();//当前系统时间
		visual_sent();                /*视觉发送*/
    vTaskDelayUntil(&currentTime, 100);//绝对延时
  }
  /* USER CODE END VISUAL_SEND_TASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
