/*
 * @file Vofa.c
 * @author github@jelin-sh
 * @brief
 * @version 0.1
 * @date 2024-02-29
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "Vofa.h"
#include <stdarg.h>
#include <stdio.h>

static const uint8_t cmdTail[] = VOFA_CMD_TAIL;
static const uint8_t justFloatTail[4] = {0x00, 0x00, 0x80, 0x7f}; // justfloat协议尾部
/*
 * @brief 初始化Vofa
 *
 * @param handle Vofa句柄
 * @param mode Vofa模式
 */
void Vofa_Init(Vofa_HandleTypedef *handle, Vofa_ModeTypeDef mode)
{
	handle->rxBuffer.rp = handle->rxBuffer.buffer;
	handle->rxBuffer.wp = handle->rxBuffer.buffer;
	handle->mode = mode;
}
/*
 * @brief Vofa发送数据
 *
 * @param handle Vofa句柄
 * @param data 数据
 * @param num 数据长度
 */
void Vofa_SendData(Vofa_HandleTypedef *handle, uint8_t *data, uint16_t num)
{
	Vofa_SendDataCallBack(handle, data, num);
}
/*
 * @brief JustFloat协议发送数据
 *
 * @param handle Vofa句柄
 * @param data 数据
 * @param num 数据长度
 */
void Vofa_JustFloat(Vofa_HandleTypedef *handle, float *data, uint16_t num)
{
	Vofa_SendDataCallBack(handle, (uint8_t *)data, num * sizeof(float));
	Vofa_SendDataCallBack(handle, (uint8_t *)justFloatTail, 4);
}
/*
 * @brief Vofa发送字符串
 *
 * @param handle Vofa句柄
 * @param format 字符串
 * @param ... 可变参数
 */
void Vofa_Printf(Vofa_HandleTypedef *handle, const char *format, ...)
{
	uint32_t n;
	va_list args;
	va_start(args, format);
	n = vsnprintf((char *)handle->txBuffer, VOFA_BUFFER_SIZE, format, args);
	Vofa_SendDataCallBack(handle, handle->txBuffer, n);
	va_end(args);
}
/*
 * @brief Vofa接收数据
 *
 * @param handle Vofa句柄
 */
void Vofa_ReceiveData(Vofa_HandleTypedef *handle)
{
	uint8_t data = Vofa_GetDataCallBack(handle);

	if (handle->rxBuffer.overflow && handle->mode == VOFA_MODE_BLOCK_IF_FIFO_FULL)
	{
		return;
	}

	*handle->rxBuffer.wp = data;
	handle->rxBuffer.wp++;

	if (handle->rxBuffer.wp == (handle->rxBuffer.buffer + VOFA_BUFFER_SIZE))
	{
		handle->rxBuffer.wp = handle->rxBuffer.buffer;
	}
	if (handle->rxBuffer.wp == handle->rxBuffer.rp)
	{
		handle->rxBuffer.overflow = true;
	}
}
/*
 * @brief Vofa获取一个字节
 *
 * @param handle Vofa句柄
 * @param byte 字节
 * @return uint8_t 是否获取成功
 */
static uint8_t Vofa_GetByte(Vofa_HandleTypedef *handle, uint8_t *byte)
{
	if (handle->rxBuffer.rp == handle->rxBuffer.wp && !handle->rxBuffer.overflow)
	{
		return false;
	}

	if (handle->rxBuffer.overflow)
	{
		handle->rxBuffer.overflow = false;
	}

	*byte = *handle->rxBuffer.rp;
	*handle->rxBuffer.rp = 0;
	handle->rxBuffer.rp++;

	if (handle->rxBuffer.rp == (handle->rxBuffer.buffer + VOFA_BUFFER_SIZE))
	{
		handle->rxBuffer.rp = handle->rxBuffer.buffer;
	}

	return true;
}
/*
 * @brief 读取命令
 *
 * @param handle Vofa句柄
 * @param buffer 缓存
 * @param bufferLen 缓存长度
 * @return uint16_t 读取长度
 */
uint16_t Vofa_ReadCmd(Vofa_HandleTypedef *handle, uint8_t *buffer, uint16_t bufferLen)
{
	uint16_t length = 0;
	uint16_t i = 0;
	uint16_t tailCount = 0;

	for (i = 0; i < bufferLen && Vofa_GetByte(handle, &buffer[i]) && tailCount < sizeof(cmdTail); i++)
	{
		if (buffer[i] == cmdTail[tailCount])
		{
			tailCount++;
		}
		else
		{
			tailCount = 0;
		}

		length++;
	}
	return length;
}
/*
 * @brief Vofa读取一行数据
 *
 * @param handle Vofa句柄
 * @param buffer 缓存
 * @param bufferLen 缓存长度
 * @return uint16_t 读取长度
 */
uint16_t Vofa_ReadLine(Vofa_HandleTypedef *handle, uint8_t *buffer, uint16_t bufferLen)
{
	uint16_t length = 0;
	uint16_t i = 0;

	for (i = 0; i < bufferLen && Vofa_GetByte(handle, &buffer[i]) && (buffer[i] != '\n'); i++)
	{
		length++;
	}
	return length;
}

uint16_t Vofa_ReadData(Vofa_HandleTypedef *handle, uint8_t *buffer, uint16_t bufferLen)
{
	uint16_t length = 0;
	uint16_t i = 0;

	for (i = 0; i < bufferLen && Vofa_GetByte(handle, &buffer[i]); i++)
	{
		length++;
	}
	return length;
}
/*
 * @brief Vofa发送数据回调函数
 *
 */
#ifdef __GNUC__
__attribute__((weak)) void Vofa_SendDataCallBack(Vofa_HandleTypedef *handle, uint8_t *data, uint16_t length)
{
	return;
}
/*
 * @brief Vofa获取数据回调函数
 *
 */
__attribute__((weak))
uint8_t
Vofa_GetDataCallBack(Vofa_HandleTypedef *handle)
{
	return 0;
}
#endif
