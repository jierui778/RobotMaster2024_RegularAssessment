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

motor_info_t motor_info[10];             // 测试双电机
motor_num_e motor_number;               // 电机编号
extern motor_id_e motor_id;             // 电机ID
CAN_TxHeaderTypeDef motor_tx_header[2]; // 电机发送报文头
/**
 * @brief 初始化电机CAN配置
 *
 */
void Motor_Can_Init(void)
{
    // Motor_Init
    CAN1_FilterConfig();                                               // 配置CAN1的过滤器
    Motor_TXInfo_Init();                                               // 初始化电机发送配置
    HAL_CAN_Start(&hcan1);                                             // 启动CAN1
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 激活CAN1的接收中断
}
/**
 * @brief 初始化电机发送配置
 *
 */
void Motor_TXInfo_Init(void)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        motor_tx_header[i].IDE = CAN_ID_STD;   // 帧ID：标准帧
        motor_tx_header[i].RTR = CAN_RTR_DATA; // 帧类型：数据帧
        motor_tx_header[i].ExtId = 0x00;       // 扩展帧ID
        motor_tx_header[i].DLC = 8;            // 数据长度
    }
    motor_tx_header[GIMBAL].StdId = GIMBAL_TXID; // 电机报文标识符
}
/**
 * @brief 发送电压参数（范围：-30000 ~ 30000）
 *
 * @param motor1 电机1的电压（范围：-30000 ~ 30000）
 * @param motor2 电机2的电压（范围：-30000 ~ 30000）
 */
void Gimbal_SendInfo(int16_t motor1, uint16_t motor2)
{
    uint8_t data[8] = {0};
    data[0] = motor1 >> 8;
    data[1] = motor1;
    data[2] = motor2 >> 8;
    data[3] = motor2;
    HAL_CAN_AddTxMessage(&hcan1, &motor_tx_header[GIMBAL], data, (uint32_t *)CAN_TX_MAILBOX0);
}
/**
 * @brief 解析接收的can信息
 *
 * @param motor_info 电机数据
 * @param Data 接收的数据
 */
void Motor_ReceiveInfo(motor_info_t *motor_info, uint8_t *Data)
{
    motor_info->angle = (Data[0] << 8) | Data[1];
    motor_info->speed = (Data[2] << 8) | Data[3];
    motor_info->current = (Data[4] << 8) | Data[5];
    motor_info->temperature = Data[6];
}
