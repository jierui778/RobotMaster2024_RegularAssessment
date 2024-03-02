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
#include "math.h"
#include "MahonyAHRS.h"
#include "semphr.h"
#include "usart.h"
#include "stdio.h"
#include "Motor.h"
#include "Vofa.h"
#include "Pid.h"
// #include "arm_math.h"
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
/* Definitions for IMU_Read */
osThreadId_t IMU_ReadHandle;
const osThreadAttr_t IMU_Read_attributes = {
  .name = "IMU_Read",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PIDInfo_Transmi */
osThreadId_t PIDInfo_TransmiHandle;
const osThreadAttr_t PIDInfo_Transmi_attributes = {
  .name = "PIDInfo_Transmi",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PID_Control */
osThreadId_t PID_ControlHandle;
const osThreadAttr_t PID_Control_attributes = {
  .name = "PID_Control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartIMU_Read(void *argument);
void StartPIDInfo_Transmit(void *argument);
void StartPID_Control(void *argument);

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

  /* creation of IMU_Read */
  IMU_ReadHandle = osThreadNew(StartIMU_Read, NULL, &IMU_Read_attributes);

  /* creation of PIDInfo_Transmi */
  PIDInfo_TransmiHandle = osThreadNew(StartPIDInfo_Transmit, NULL, &PIDInfo_Transmi_attributes);

  /* creation of PID_Control */
  PID_ControlHandle = osThreadNew(StartPID_Control, NULL, &PID_Control_attributes);

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
    SEGGER_RTT_printf(0, "SEGGER_Test_ok !\n"); // 测试RTT打印功能

    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin); // LED闪烁表明系统在运�?????
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartIMU_Read */
/**
 * @brief Function implementing the IMU_Read thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartIMU_Read */
void StartIMU_Read(void *argument)
{
  /* USER CODE BEGIN StartIMU_Read */
  /* Infinite loop */
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount(); // 获取当前时间绝对延时阻塞导致Can其他任务无法运行
  for (;;)
  {
    IMU_ReadData(&imu_data);
    vTaskDelayUntil(&xLastWakeTime, 1); // 任务延时1ms
  }
  /* USER CODE END StartIMU_Read */
}

/* USER CODE BEGIN Header_StartPIDInfo_Transmit */
/**
 * @brief Function implementing the PIDInfo_Transmi thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPIDInfo_Transmit */
void StartPIDInfo_Transmit(void *argument)
{
  /* USER CODE BEGIN StartPIDInfo_Transmit */
  /* Infinite loop */
  static float temp[6];
  for (;;)
  {
    Vofa_HandleTypedef vofa1;
    temp[0] = motor_info[8].target_angle;
    temp[1] = motor_info[8].angle;
    temp[2] = motor_info[8].target_speed;
    temp[3] = motor_info[8].speed;
    temp[4] = motor_info[8].current;
    temp[5] = motor_info[8].temperature;
    Vofa_JustFloat(&vofa1, temp, 6);
    osDelay(10);
  }
  /* USER CODE END StartPIDInfo_Transmit */
}

/* USER CODE BEGIN Header_StartPID_Control */
/**
* @brief Function implementing the PID_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPID_Control */
void StartPID_Control(void *argument)
{
  /* USER CODE BEGIN StartPID_Control */
  /* Infinite loop */
  static float t = 0;
  for (;;)
  {
    t += 0.005;
    /*角度�?*/
    // motor_info[8].target_angle = 4000 + 4000 * sin(t * PI);
    // PosiPID(&PosiPID_Info[GIMBAL1], &motor_info[8]);
    // Gimbal_SendInfo(PosiPID_Info[0].Output, 0);

    /*速度�?*/
    // motor_info[8].target_speed = 250 * sin(t * PI);
    // IncrPID(&IncrPID_Info[GIMBAL1], &motor_info[8]);
    // Gimbal_SendInfo(IncrPID_Info[GIMBAL1].Output, 0);

    osDelay(1);
  }
  /* USER CODE END StartPID_Control */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

