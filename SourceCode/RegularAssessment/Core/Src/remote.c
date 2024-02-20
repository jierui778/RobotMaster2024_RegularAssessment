/**
 * @file remote.c
 * @author jierui778 (758418101@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "remote.h"
#include "string.h"
#include "usart.h"
#include "stdio.h"
uint8_t rx_buffer[REMOTE_MAX_BUFFER_SIZE];
uint8_t rx_len;
extern DMA_HandleTypeDef hdma_usart1_rx;
/**
 * @brief
 *
 * @param huart 串口句柄
 * @param Size 接收到的数据长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)
    {
        printf("Received data: %s\n",   rx_buffer);                            // 打印接收到的数据
        printf("Received data length: %d\n", Size);                            // 打印接收到的数据长度
        // HAL_UART_Transmit_DMA(&huart1, rx_buffer, Size);                       // 通过DMA发送接收到的数据
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, sizeof(rx_buffer)); // 重新启动DMA接收
        //__HAL_DMA_DISABLE(&hdma_usart1_rx); // 关闭DMA
    }
}