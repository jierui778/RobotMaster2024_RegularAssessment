/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "imu.h"
#include "SEGGER_RTT.h"
#include "imu.h"
#include "MahonyAHRS.h"
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
    BMI088_ReadGyro(&imu_data);
    BMI088_ReadAccel(&imu_data);
    IST8310_Read(&imu_data);
    // SEGGER_RTT_printf(0, "segger !\n"); //测试RTT接口打印功能
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin); // LED闪烁表明任务在运�??
    osDelay(1000);
    /*对数据进行转�??*/
//    for (int i = 0; i < 3; i++)
//    {
//      imu_gyro[i] = (imu_data.gyro[i]) / 65.536 * (PI / 180);
//      imu_accel[i] = imu_data.accel[i] * 0.0008974f;
//      imu_mag[i] = imu_data.mag[i]*0.3;
//    }
//    /***减去零偏值（零偏�??标定获取�??***/
//    imu_gyro[1] -= (11.5390333f / 65.536) * (PI / 180);
//    imu_gyro[2] -= (10.4231017f / 65.536) * (PI / 180);
//    imu_accel[1] -= (141.763613f * 0.0008974);
//
//    /***均�?�滤�??***/
//    MahonyAHRSupdateIMU(imu_data.angle_q, imu_gyro[0], imu_gyro[1], imu_gyro[2], imu_accel[0], imu_accel[1], imu_accel[2]);
//    imu_data.angle[0] = atan2f(2.0f * (imu_data.angle_q[0] * imu_data.angle_q[3] + imu_data.angle_q[1] * imu_data.angle_q[2]), 2.0f * (imu_data.angle_q[0] * imu_data.angle_q[0] + imu_data.angle_q[1] * imu_data.angle_q[1]) - 1.0f);
//    imu_data.angle[1] = asinf(-2.0f * (imu_data.angle_q[1] * imu_data.angle_q[3] - imu_data.angle_q[0] * imu_data.angle_q[2]));
//    imu_data.angle[2] = atan2f(2.0f * (imu_data.angle_q[0] * imu_data.angle_q[1] + imu_data.angle_q[2] * imu_data.angle_q[3]), 2.0f * (imu_data.angle_q[0] * imu_data.angle_q[0] + imu_data.angle_q[3] * imu_data.angle_q[3]) - 1.0f);
    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

