/**
 * @file Bsp_can.h
 * @author jierui778 (758418101@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-24
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__
#include "stm32f4xx_hal.h"
void CAN1_Filter_Config(void);
uint8_t CAN1_Send_Msg(uint8_t *msg, uint8_t len);
uint8_t CAN1_Recv_Msg(uint8_t *buf);

#endif
