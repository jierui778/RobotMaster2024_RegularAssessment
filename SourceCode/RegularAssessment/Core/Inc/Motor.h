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
    uint16_t target_angle; // 目标角度
    uint16_t angle;        // 当前角度
    uint16_t target_speed; // 目标速度
    uint16_t speed;        // 当前速度
    uint16_t current;      // 电机电流
    uint8_t temperature;   // 电机温度
} motor_info_t;
/*电机编号*/
typedef enum
{
    GIMBAL1,
    GIMBAL2
} motor_num_e;
/*电机类型*/
typedef enum
{
    GIMBAL = 1,
} motor_txheader_num_e;
/*电机的反馈报文标识符*/
typedef enum
{
    GIMBAL_TXID = 0x2FF,
    //
} motor_id_e;

extern motor_num_e motor_number;    // 电机编号
extern motor_id_e motor_id;         // 电机ID
extern motor_info_t motor_info[10]; // 测试双电机

void Motor_Can_Init(void);                                       // 初始化电机CAN配置
void Motor_TXInfo_Init(void);                                    // 初始化电机发送配置
void Gimbal_SendInfo(int16_t motor1, uint16_t motor2);           // 发送电机信息
void Motor_ReceiveInfo(motor_info_t *motor_info, uint8_t *Data); // 接收电机信息

#endif // _MOTOR_H__