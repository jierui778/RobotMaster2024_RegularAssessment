/**
 * @file Motor.h
 * @author jierui778 (758418101@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _MOTOR_H__
#define _MOTOR_H__
#include "main.h"

/*电机参数结构体*/
typedef struct
{
    uint16_t target_angle;  // 目标角度
    uint16_t current_angle; // 当前角度
    uint16_t taraget_speed; // 目标速度
    uint16_t current_speed; // 当前速度
    uint8_t temperature;    // 电机温度
} motor_info_t;
/*电机编号*/
typedef enum
{
    CHASSIS1,
    CHASSIS2
} motor_number_t;

extern motor_number_t motor_number;
extern motor_info_t motor_info;

void Motor_Can_Init(void);     // 初始化电机CAN配置
void Motor_TXInfo_Init(void);  // 初始化电机发送配置
void Send_GimbalInfo(void);    // 发送云台信息
void Receive_GimbalInfo(void); // 接收云台信息

#endif // _MOTOR_H__