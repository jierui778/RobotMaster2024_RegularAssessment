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
    uint16_t angle;         // 当前角度
    uint16_t taraget_speed; // 目标速度
    uint16_t speed;         // 当前速度
    uint16_t current;       // 电机电流
    uint8_t temperature;    // 电机温度
} motor_info_t;
/*电机编号*/
typedef enum
{
    MOTOR1,
    MOTOR2
} motor_number_e;
/*电机ID*/
typedef enum
{
    MOTOR1_ID = 0x201,
    MOTOR2_ID = 0x202
} motor_id_e;

extern motor_number_e motor_number; // 电机编号
extern motor_id_e motor_id;         // 电机ID
extern motor_info_t motor_info[2];  // 测试双电机

void Motor_Can_Init(void);                                       // 初始化电机CAN配置
void Motor_TXInfo_Init(void);                                    // 初始化电机发送配置
void Motor_SendInfo(int16_t motor1, uint16_t motor2);            // 发送电机信息
void Motor_ReceiveInfo(motor_info_t *motor_info, uint8_t *Data); // 接收电机信息

#endif // _MOTOR_H__