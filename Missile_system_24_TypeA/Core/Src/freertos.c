/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "shoot_task.h"
#include "referee_usart_task.h"
#include "weight_data.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
/* Definitions for Motion_Task */
osThreadId_t Motion_TaskHandle;
const osThreadAttr_t Motion_Task_attributes = {
  .name = "Motion_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ActionTask */
osThreadId_t ActionTaskHandle;
const osThreadAttr_t ActionTask_attributes = {
  .name = "ActionTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for referee_Task */
osThreadId_t referee_TaskHandle;
const osThreadAttr_t referee_Task_attributes = {
  .name = "referee_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void MotionControlTask(void *argument);
void ActionControlTask(void *argument);
void refereeTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of Motion_Task */
  Motion_TaskHandle = osThreadNew(MotionControlTask, NULL, &Motion_Task_attributes);

  /* creation of ActionTask */
  ActionTaskHandle = osThreadNew(ActionControlTask, NULL, &ActionTask_attributes);

  /* creation of referee_Task */
  referee_TaskHandle = osThreadNew(refereeTask, NULL, &referee_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_MotionControlTask */
/**
  * @brief  Function implementing the Motion_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MotionControlTask */
void MotionControlTask(void *argument)
{
  /* USER CODE BEGIN MotionControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MotionControlTask */
}

/* USER CODE BEGIN Header_ActionControlTask */
/**
* @brief Function implementing the ActionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ActionControlTask */
void ActionControlTask(void *argument)
{
  /* USER CODE BEGIN ActionControlTask */
	shoot_task(argument);

  /* USER CODE END ActionControlTask */
}

/* USER CODE BEGIN Header_refereeTask */
/**
* @brief Function implementing the referee_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_refereeTask */
void refereeTask(void *argument)
{
  /* USER CODE BEGIN refereeTask */
	referee_usart_task(argument);
  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
  /* USER CODE END refereeTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

