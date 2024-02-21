/**
 * @file remote.h
 * @author jierui778 (758418101@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-13
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "stm32f4xx_hal.h"
#ifndef __REMOTE_H
#define __REMOTE_H
#define REMOTE_MAX_BUFFER_SIZE 50
extern uint8_t rx_len;
extern uint8_t rx_buffer[REMOTE_MAX_BUFFER_SIZE];


#endif
