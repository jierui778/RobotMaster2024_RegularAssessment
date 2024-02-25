/**
 * @file Bsp_can.c
 * @author jierui778 (758418101@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-24
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "Bsp_can.h"
#include "SEGGER_RTT.h"
#include <can.h>

void CAN1_Filter_Config()
{
    CAN_FilterTypeDef CAN1_FilerConf;

    CAN1_FilerConf.FilterBank = 0;                      // 筛选器编号, CAN1是0-13, CAN2是14-27
    CAN1_FilerConf.FilterMode = CAN_FILTERMODE_IDMASK;  // 采用掩码模式
    CAN1_FilerConf.FilterScale = CAN_FILTERSCALE_32BIT; // 设置筛选器的尺度, 采用32位
    CAN1_FilerConf.FilterIdHigh = 0X0000;               // 过滤器ID高16位,即CAN_FxR1寄存器的高16位
    CAN1_FilerConf.FilterIdLow = 0X0000;                // 过滤器ID低16位,即CAN_FxR1寄存器的低16位
    CAN1_FilerConf.FilterMaskIdHigh = 0X0000;           // 过滤器掩码高16位,即CAN_FxR2寄存器的高16位
    CAN1_FilerConf.FilterMaskIdLow = 0X0000;            // 过滤器掩码低16位,即CAN_FxR2寄存器的低16位
    CAN1_FilerConf.FilterFIFOAssignment = CAN_RX_FIFO0; // 设置经过筛选后数据存储到哪个接收FIFO
    CAN1_FilerConf.FilterActivation = ENABLE;           // 是否使能本筛选器
    CAN1_FilerConf.SlaveStartFilterBank = 14;           // 指定为CAN1分配多少个滤波器组

    if (HAL_CAN_ConfigFilter(&hcan1, &CAN1_FilerConf) != HAL_OK)
    {
        Error_Handler();
    }
}
uint8_t CAN1_Send_Msg(uint8_t *msg, uint8_t len)
{
    uint16_t i = 0;
    uint32_t txMailBox;
    uint8_t send_buf[8];

    CAN_TxHeaderTypeDef txHeader;
    txHeader.StdId = 12;
    txHeader.ExtId = 12;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = len;

    for (i = 0; i < len; i++)
        send_buf[i] = msg[i];

    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, send_buf, &txMailBox) != HAL_OK)
        return 1;
    return 0;
}
uint8_t CAN1_Recv_Msg(uint8_t *buf)
{
    uint16_t i = 0;
    CAN_RxHeaderTypeDef rxHeader;
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, buf);

    if (rxHeader.IDE == CAN_ID_STD)
        SEGGER_RTT_printf(0, "StdId ID: %d\r\n", rxHeader.StdId);
    else
        SEGGER_RTT_printf(0, "ExtId ID: %d\r\n", rxHeader.ExtId);

    // SEGGER_RTT_printf(0, "CAN IDE: %d\r\n", rxHeader.IDE);
    // SEGGER_RTT_printf(0, "CAN RTR: %d\r\n", rxHeader.RTR);
    // SEGGER_RTT_printf(0, "CAN DLC: %d\r\n", rxHeader.DLC);
    // SEGGER_RTT_printf(0, "Recv Data: ");

    for (i = 0; i < rxHeader.DLC; i++)
        SEGGER_RTT_printf(0, "%c", buf[i]);

    // SEGGER_RTT_printf(0, "\n");
    return rxHeader.DLC;
}