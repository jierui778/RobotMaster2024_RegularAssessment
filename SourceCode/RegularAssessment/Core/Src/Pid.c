/**
 * @file Pid.c
 * @author jierui778 (758418101@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-28
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "Pid.h"
#include "main.h"
#include "math.h"
#include "Motor.h"

sPosiPID_Info PosiPID_Info[2];
sIncrPID_Info IncrPID_Info[2];

// Chassis1=0,Chassis2,Chassis3,Chassis4,frict_L,frict_R,Ammo,Chassis_Posi,Yaw,Pitch

float PidInfo[2][2][5] = {
	// PosiPID{Kp,Ki,Kd,MaxSum,MaxOutput}
	{
		{57.0f, 0, 0.3f, 28000, 28000},
		// {115.0f, 0, 1.9f, 28000, 28000}, // GIMBAL1
		{10.0f, 0, 0, 28000, 28000} // GIMBAL2
	},
	// IncrPID{Kp,Ki,Kd,MaxOutput,0}
	{
		{1.5f, 45.0f, 0, 28000, 28000}, // GIMBAL1
		{0}								// GIMBAL2
	}};

/**
 * @brief 对PID输出进行限幅
 *
 * @param value 输出值
 * @param maxvalue 最大值
 */
void PIDInfo_Limit(float *value, float maxvalue)
{
	if (fabs(*value) > maxvalue)
	{
		if (*value >= 0)
			*value = maxvalue;
		else if (*value < 0)
			*value = -maxvalue;
	}
}

/**
 * @brief 初始化PID参数
 *
 */
void PID_Init(void)
{
	for (uint8_t i = 0; i < 2; i++)
	{
		// 位置式PID参数初始化
		PosiPID_Info[i].Kp = PidInfo[0][i][0];
		PosiPID_Info[i].Ki = PidInfo[0][i][1];
		PosiPID_Info[i].Kd = PidInfo[0][i][2];
		PosiPID_Info[i].MaxSum = PidInfo[0][i][3];
		PosiPID_Info[i].MaxOutput = PidInfo[0][i][4];
		PosiPID_Info[i].Err = 0;
		PosiPID_Info[i].Sum = 0;
		PosiPID_Info[i].Output = 0;
		// 增量式PID参数初始化
		IncrPID_Info[i].Kp = PidInfo[1][i][0];
		IncrPID_Info[i].Ki = PidInfo[1][i][1];
		IncrPID_Info[i].Kd = PidInfo[1][i][2];
		IncrPID_Info[i].MaxOutput = PidInfo[1][i][3];
		IncrPID_Info[i].Err = 0;
		IncrPID_Info[i].LastErr = 0;
		IncrPID_Info[i].Output = 0;
	}
}

/**
 * @brief 位置式PID
 *
 * @param PosiPID_Info 位置式PID结构体
 * @param MotorInfo 电机信息结构体
 */
void PosiPID(sPosiPID_Info *PosiPID_Info, motor_info_t *MotorInfo)
{
	PosiPID_Info->LastErr = PosiPID_Info->Err;
	PosiPID_Info->Err = MotorInfo->target_angle - MotorInfo->angle;
	if (PosiPID_Info->Err > 4096)
		PosiPID_Info->Err -= 8192;
	else if (PosiPID_Info->Err < -4096)
		PosiPID_Info->Err += 8192;
	PosiPID_Info->Sum += PosiPID_Info->Err;
	PIDInfo_Limit(&PosiPID_Info->Sum, PosiPID_Info->MaxSum);
	PosiPID_Info->Output = PosiPID_Info->Kp * PosiPID_Info->Err + PosiPID_Info->Ki * PosiPID_Info->Sum + PosiPID_Info->Kd * (PosiPID_Info->Err - PosiPID_Info->LastErr);
	PIDInfo_Limit(&PosiPID_Info->Output, PosiPID_Info->MaxOutput);
}

/**
 * @brief 增量式PID
 *
 * @param IncrPID_Info 增量式PID结构体
 * @param MotorInfo 电机信息结构体
 */
void IncrPID(sIncrPID_Info *IncrPID_Info, motor_info_t *MotorInfo)
{
	IncrPID_Info->LastLastErr = IncrPID_Info->LastErr;
	IncrPID_Info->LastErr = IncrPID_Info->Err;
	IncrPID_Info->Err = MotorInfo->target_speed - MotorInfo->speed;
	IncrPID_Info->Output += IncrPID_Info->Kp * (IncrPID_Info->Err - IncrPID_Info->LastErr) + IncrPID_Info->Ki * IncrPID_Info->Err + IncrPID_Info->Kd * (IncrPID_Info->Err - 2 * IncrPID_Info->LastErr + IncrPID_Info->LastLastErr);
	PIDInfo_Limit(&IncrPID_Info->Output, IncrPID_Info->MaxOutput);
}

/**
 * @brief PID参数去初始化
 *
 */
void PID_DeInit(void)
{
	for (uint8_t i = 0; i < 2; i++)
	{
		IncrPID_Info[i].Kp = 0;
		IncrPID_Info[i].Ki = 0;
		IncrPID_Info[i].Kd = 0;
		IncrPID_Info[i].MaxOutput = 0;
		IncrPID_Info[i].Err = 0;
		IncrPID_Info[i].LastErr = 0;
		IncrPID_Info[i].Output = 0;
	}
}
