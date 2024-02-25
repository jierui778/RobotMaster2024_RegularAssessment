/**
 * @file Motor.c
 * @author jierui778 (758418101@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "Motor.h"
#include "can.h"
#include "main.h"

// motor_info_t motor_info[2]; // 测试双电机

void Motor_Can_Init(void)
{
    // Motor_Init
}
void Motor_TXInfo_Init(void)
{
    // Motor_TXInfo_Init
    for (uint8_t i = 0; i < 3; i++)
    {
        
    }
}

void Send_GimbalInfo(void)
{
    // Send_GimbalInfo
}
void Receive_MotorInfo(void)
{
    // Receive_MotorInfo
}
