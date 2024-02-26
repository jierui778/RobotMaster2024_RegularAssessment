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

motor_info_t motor_info[2]; // 测试双电机
/**
 * @brief 初始化电机CAN配置
 *
 */
void Motor_Can_Init(void)
{
    // Motor_Init
}
/**
 * @brief 初始化电机发送配置
 *
 */
void Motor_TXInfo_Init(void)
{
    // Motor_TXInfo_Init
    for (uint8_t i = 0; i < 3; i++)
    {
    }
}
/**
 * @brief can发送电机信息
 *
 */
void Send_MotorInfo(void)
{
    // Send_GimbalInfo
}
/**
 * @brief can发送电机信息
 *
 */
void Receive_MotorInfo(void)
{
    // Receive_MotorInfo
}
