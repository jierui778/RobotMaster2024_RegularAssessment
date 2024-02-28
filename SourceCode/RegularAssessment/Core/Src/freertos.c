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
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow6,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

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

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(StartTask05, NULL, &myTask05_attributes);

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
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount(); // 获取当前时间
  for (;;)
  {
    BMI088_ReadGyro(&imu_data);
    BMI088_ReadAccel(&imu_data);
    IST8310_Read(&imu_data);
    // SEGGER_RTT_printf(0, "segger !\n"); //测试RTT接口打印功能
    //    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    //    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin); // LED闪烁表明任务在运�???
    //    osDelay(1000);
    /* 对数据进行转�??*/
    for (int i = 0; i < 3; i++)
    {
      imu_gyro[i] = (imu_data.gyro[i]) / 65.536 * (PI / 180);
      imu_accel[i] = imu_data.accel[i] * 0.0008974f;
      imu_mag[i] = imu_data.mag[i] * 0.3;
    }
    /*去零�??*/
    imu_gyro[1] -= (11.5390333f / 65.536) * (PI / 180);
    imu_gyro[2] -= (10.4231017f / 65.536) * (PI / 180);
    imu_gyro[2] -= (10.4288017f / 65.536) * (PI / 180);
    //    imu_accel[1] -= (141.763613f * 0.0008974);

    MahonyAHRSupdateIMU(imu_gyro[0], imu_gyro[1], imu_gyro[2], imu_accel[0], imu_accel[1], imu_accel[2]); // 融合六轴数据
    // MahonyAHRSupdate(imu_gyro[0], imu_gyro[1], imu_gyro[2], imu_accel[0], imu_accel[1], imu_accel[2], imu_mag[0], imu_mag[1], imu_mag[2]);//融合九轴数据
    imu_data.angle_q[0] = q0;
    imu_data.angle_q[1] = q1;
    imu_data.angle_q[2] = q2;
    imu_data.angle_q[3] = q3;

    // 四元数计算欧拉角
    imu_data.angle[0] = (atan2(2.0f * (imu_data.angle_q[0] * imu_data.angle_q[1] + imu_data.angle_q[2] * imu_data.angle_q[3]), 1 - 2.0f * (imu_data.angle_q[1] * imu_data.angle_q[1] + imu_data.angle_q[2] * imu_data.angle_q[2]))) * 180 / PI;
    imu_data.angle[1] = asin(2.0f * (imu_data.angle_q[0] * imu_data.angle_q[2] - imu_data.angle_q[1] * imu_data.angle_q[3])) * 180 / PI;
    imu_data.angle[2] = atan2(2 * imu_data.angle_q[1] * imu_data.angle_q[2] + 2 * imu_data.angle_q[0] * imu_data.angle_q[3], -2 * imu_data.angle_q[2] * imu_data.angle_q[2] - 2 * imu_data.angle_q[3] * imu_data.angle_q[3] + 1) * 180 / PI; // yaw
    // HAL_UART_Transmit_DMA(&huart1, imu_data.angle[0], 12);
    // osDelay(1);
    vTaskDelayUntil(&xLastWakeTime, 1); // 任务延时1ms
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
  for (;;)
  {
    // printf("%d",test);
    // u1_printf("%f",6.666);
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin); // LED闪烁表明系统正常运行
                                                    //    u1_printf("%d,%d,%d\n", -(int16_t)imu_data.angle[0], (int16_t)imu_data.angle[1], (int16_t)imu_data.angle[2]);
    Gimbal_SendInfo(10000, 10000);
    // u1_printf("%.2f,%.2f,%.2f\n",-(int16_t)imu_data.angle_q[0],(int16_t)imu_data.angle_q[1],(int16_t)imu_data.angle_q[2]);
    //  HAL_UART_Transmit_DMA(&huart1, "RoboMaster\r\n", 12);
    //    u1_printf("%d,%d,%d",6,6,6);

    // HAL_UART_Transmit(&huart1, "RoboMaster\r\n", 12, 0xfff);
    //  ret = CAN1_Send_Msg(txdata, 8);
    //  if (ret == 0)
    //    SEGGER_RTT_printf(0, "CAN Send success!\r\n");
    //  else
    //    SEGGER_RTT_printf(0, "CAN Send failed!\r\n");

    // CAN1_Recv_Msg(rxdata);
    // // SEGGER_RTT_printf(0, "+++++++++++++++++++++++++++++++\r\n");
    // HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    // HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin); // LED闪烁表明系统正常运行
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  // Vofa_HandleTypedef vofa1;
  // static volatile float testData[5] = {0};
  for (;;)
  {
    // static const uint8_t cmd_open[] = {0x00, 0x01, 0xAF, 0xFA};
    // static const uint8_t cmd_close[] = {0x00, 0x00, 0xAF, 0xFA};
    // static uint8_t cmdBuffer[10] = {0};
    // if (Vofa_ReadCmd(&vofa1, cmdBuffer, 10))
    // {
    //   if (memCmp(cmdBuffer, (uint8_t *)cmd_open, sizeof(cmd_open)))
    //   {
    //     static float time = 0;
    //     time += 0.0003f;
    //     testData[0] = sin(time * 2 * PI);
    //     testData[1] = sin(time * 2 * PI + 5);
    //     testData[2] = sin(time * 2 * PI - 5);
    //     testData[3] = sin(time * 4 * PI);
    //     testData[4] = 2 * sin(time * 4 * PI);
    //     Vofa_JustFloat(&vofa1, (float *)testData, 5);
    //   }

    //   if (memCmp(cmdBuffer, (uint8_t *)cmd_close, sizeof(cmd_close)))
    //   {
    //   }

    //   memset(cmdBuffer, 0, 10);
    // }
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
 * @brief Function implementing the myTask04 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
 * @brief Function implementing the myTask05 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

