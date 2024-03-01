/**
 * @file Pid.h
 * @author jierui778 (758418101@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef PID_H
#define PID_H

#include "Motor.h"
/*位置式PID参数结构体*/
typedef struct
{
	float Kp, Ki, Kd;
	float Err, LastErr;
	float Sum, MaxSum;
	float Output, MaxOutput;
} sPosiPID_Info;
/*增量式PID参数结构体*/
typedef struct
{
	float Kp, Ki, Kd;
	float Err, LastErr, LastLastErr;
	float Output, MaxOutput;
} sIncrPID_Info;

extern sPosiPID_Info PosiPID_Info[2];
extern sIncrPID_Info IncrPID_Info[2];

void PID_Init(void);
void PID_DeInit(void);
void PosiPID(sPosiPID_Info *PosiPID_Info, motor_info_t *MotorInfo);
void IncrPID(sIncrPID_Info *IncrPID_Info, motor_info_t *MotorInfo);
#endif
