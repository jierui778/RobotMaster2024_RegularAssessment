/*
 * @file Imu.c
 * @author jierui778 (758418101@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "Imu.h"
#include "i2c.h"
#include "spi.h"
#include "main.h"
#include "MahonyAHRS.h"

enum BMI088_WriteMode
{
	AccelWrite,
	GyroWrite,
};
static void IST8310_WriteByte(uint8_t reg, uint8_t data);
static void BMI088_WriteByte(uint8_t addr, uint8_t data, enum BMI088_WriteMode a_enum);

IMU_TypeDef imu_data;

/*
 * @brief 向IST8310写入一个字节
 *
 * @param reg 寄存器地址
 * @param data 要写入的数据
 */
static void IST8310_WriteByte(uint8_t reg, uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c3, (IST8310_I2C_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);
}
/*
 * @brief IST8310初始化
 *
 */
uint8_t IST8310_Init(void)
{
	// int8_t data = 0;

	/*重启磁力计*/
	HAL_GPIO_WritePin(IST8310_Reset_GPIO_Port, IST8310_Reset_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(IST8310_Reset_GPIO_Port, IST8310_Reset_Pin, GPIO_PIN_SET);
	HAL_Delay(50);

	// data =
	// if (data != IST8310_CHIP_ID_VAL)
	// {
	//     return 1; // 连接失败
	// }

	IST8310_WriteByte(IST8310_STAT2_ADDR, IST8310_STAT2_NONE_ALL);	 // 不使能中断，直接读取数据
	IST8310_WriteByte(IST8310_AVGCNTL_ADDR, IST8310_AVGCNTL_FOURTH); // 平均采样四次
	IST8310_WriteByte(IST8310_CNTL1_ADDR, IST8310_CNTL1_CONTINUE);	 // 输出频率200Hz
	return 0;														 // 连接成功
}
/*
 * @brief 读取IST8310数据
 *
 * @param imu 保存数据的结构体
 */
void IST8310_Read(IMU_TypeDef *imu)
{
	uint8_t buf[6]; // 保存读取到的数据
	HAL_I2C_Mem_Read(&hi2c3, (IST8310_I2C_ADDR << 1), IST8310_DATA_XL_ADDR, I2C_MEMADD_SIZE_8BIT, buf, 6, 50);
	imu->mag[0] = buf[0] + (buf[1] << 8);
	imu->mag[1] = buf[2] + (buf[3] << 8);
	imu->mag[2] = buf[4] + (buf[5] << 8);
}
/*
 * @brief 向BMI088写入一个字节
 *
 * @param addr 寄存器地址
 * @param data 要写入的数据
 * @param a_enum 写入模式
 */
static void BMI088_WriteByte(uint8_t addr, uint8_t data, enum BMI088_WriteMode a_enum)
{
	uint8_t spi_TxData;
	switch (a_enum)
	{
	case AccelWrite:
		HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, RESET);
		break;
	case GyroWrite:
		HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, RESET);
		break;
	}
	spi_TxData = addr & 0x7F; // bit0为0，向imu写
	HAL_SPI_Transmit(&hspi1, &spi_TxData, 1, 500);
	while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX)
		;
	HAL_SPI_Transmit(&hspi1, &data, 1, 500);
	HAL_Delay(30);
	switch (a_enum)
	{
	case AccelWrite:
		HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, SET);
		break;
	case GyroWrite:
		HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, SET);
		break;
	}
}

/*
 * @brief BMI088初始化
 *
 * @return uint8_t 0:成功 1:失败
 */
uint8_t BMI088_Init(void)
{
	/*加速度计初始化*/
	BMI088_WriteByte(BMI088_ACCEL_SOFTRESET_ADDR, BMI088_ACCEL_SOFTRESET_VAL, AccelWrite); // 向0x7E写入0xb6以软件复位加速度计
	BMI088_WriteByte(BMI088_ACCEL_PWR_CTRL_ADDR, BMI088_ACCEL_PWR_CTRL_ON, AccelWrite);	   // 向0x7D写入0x04以取消加速度计暂停
	BMI088_WriteByte(BMI088_ACCEL_RANGE_ADDR, BMI088_ACCEL_RANGE_3G, AccelWrite);		   // 设置量程为±3g
	BMI088_WriteByte(BMI088_ACCEL_CONF_ADDR, BMI088_ACCEL_CONF_RESERVED << 7 | BMI088_ACCEL_CONF_BWP_OSR4 << 6 | BMI088_ACCEL_CONF_ODR_200_Hz, AccelWrite);

	/*陀螺仪初始化*/
	BMI088_WriteByte(BMI088_GYRO_SOFTRESET_ADDR, BMI088_GYRO_SOFTRESET_VAL, GyroWrite); // 向0x14写入0xb6以软件复位陀螺仪
	BMI088_WriteByte(BMI088_GYRO_LPM1_ADDR, BMI088_GYRO_LPM1_NOR, GyroWrite);
	BMI088_WriteByte(BMI088_GYRO_RANGE_ADDR, BMI088_GYRO_RANGE_500_DEG_S, GyroWrite); // ±500
	BMI088_WriteByte(BMI088_GYRO_BANDWIDTH_ADDR, BMI088_GYRO_ODR_200Hz_BANDWIDTH_64Hz, GyroWrite);
	return 0;
}

/*
 * @brief 读取BMI088陀螺仪数据
 *
 * @param imu
 */
void BMI088_ReadGyro(IMU_TypeDef *imu)
{
	uint8_t buf[6];
	uint8_t spi_TxData, spi_RxData;
	HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, RESET);
	spi_TxData = 0x02 | 0x80;
	HAL_SPI_Transmit(&hspi1, &spi_TxData, 1, 300);
	spi_TxData = 0x55;
	for (int i = 0; i < 6; i++) // 接受读取信息
	{
		HAL_SPI_TransmitReceive(&hspi1, &spi_TxData, &spi_RxData, 1, 300);
		buf[i] = spi_RxData;
	}

	imu->gyro[0] = (buf[0] + (buf[1] << 8));
	imu->gyro[1] = (buf[2] + (buf[3] << 8));
	imu->gyro[2] = (buf[4] + (buf[5] << 8));
	HAL_GPIO_WritePin(CS1_Gyro_GPIO_Port, CS1_Gyro_Pin, SET); // 传输停止
}
/*
 * @brief 读取BMI088加速度计数据
 *
 * @param imu 保存数据的结构体
 */
void BMI088_ReadAccel(IMU_TypeDef *imu)
{
	uint8_t buf[6];
	uint8_t spi_TxData, spi_RxData;
	HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, RESET);
	spi_TxData = 0x12 | 0x80;
	HAL_SPI_TransmitReceive(&hspi1, &spi_TxData, &spi_RxData, 1, 300);
	spi_TxData = 0x12 | 0x80;
	HAL_SPI_TransmitReceive(&hspi1, &spi_TxData, &spi_RxData, 1, 300);
	spi_TxData = 0x55;
	for (uint8_t i = 0; i < 6; i++)
	{
		HAL_SPI_TransmitReceive(&hspi1, &spi_TxData, &spi_RxData, 1, 300);
		buf[i] = spi_RxData;
	}
	imu->accel[0] = (buf[0] + (buf[1] << 8));
	imu->accel[1] = (buf[2] + (buf[3] << 8));
	imu->accel[2] = (buf[4] + (buf[5] << 8));

	HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, SET); // 传输停止
}

/*
 * @brief 读取BMI088传感器温度
 *
 * @param imu
 */
void BMI088_ReadTemp(IMU_TypeDef *imu)
{
	uint8_t buf[2];
	uint8_t spi_TxData, spi_RxData;
	HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_RESET);
	spi_TxData = (BMI088_TEMP_MSB_ADDR | 0x80);
	HAL_SPI_Transmit(&hspi1, &spi_TxData, 1, 1000);
	while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX)
		;
	HAL_SPI_Receive(&hspi1, &spi_TxData, 1, 1000);
	while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX)
		;

	for (uint8_t i = 0; i < 2; i++)
	{
		HAL_SPI_Receive(&hspi1, &spi_RxData, 1, 1000);
		while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX)
			;
		buf[i] = spi_RxData;
	}
	uint16_t Temp = (buf[0] << 3) + (buf[1] >> 5);
	if (Temp > 1023)
		Temp -= 2048;
	else
		Temp = (int16_t)Temp;
	imu->temp = Temp * BMI088_TEMP_UNIT + BMI088_TEMP_BIAS;
	HAL_GPIO_WritePin(CS1_Accel_GPIO_Port, CS1_Accel_Pin, GPIO_PIN_SET);
}
/*
 * @brief 获取BMI088数据
 *
 * @param imu 保存数据的结构体
 */
void IMU_ReadData(IMU_TypeDef *imu)
{
	BMI088_ReadAccel(imu);
	BMI088_ReadGyro(imu);
	// BMI088_ReadTemp(imu);
	/*对原始数据进行转换*/
	for (int i = 0; i < 3; i++)
	{
		imu_data.gyro_f[i] = (imu_data.gyro[i]) / 65.536 * (PI / 180); // 单位：rad/s
		imu_data.accel_f[i] = imu_data.accel[i] * 0.0008974f;		   // 单位：g
		imu_data.mag_f[i] = imu_data.mag[i] * 0.3;					   // 单位：uT

		imu_data.gyro_f[1] -= (11.5390333f / 65.536) * (PI / 180);
		imu_data.gyro_f[2] -= (22.4231017f / 65.536) * (PI / 180);
		// imu_data.accel_f[1] -= (50.846753f / 65.536) * (PI / 180);
	}

	MahonyAHRSupdateIMU(imu_data.gyro_f[0], imu_data.gyro_f[1], imu_data.gyro_f[2], imu_data.accel_f[0], imu_data.accel_f[1], imu_data.accel_f[2]); // Mahony融合六轴数据
	// MahonyAHRSupdate(imu_gyro[0], imu_gyro[1], imu_gyro[2], imu_accel[0], imu_accel[1], imu_accel[2], imu_mag[0], imu_mag[1], imu_mag[2]);//融合九轴数据

	/*获取四元数*/
	imu_data.angle_q[0] = q0;
	imu_data.angle_q[1] = q1;
	imu_data.angle_q[2] = q2;
	imu_data.angle_q[3] = q3;

	// 四元数计算欧拉角
	imu_data.angle[0] = (atan2(2.0f * (imu_data.angle_q[0] * imu_data.angle_q[1] + imu_data.angle_q[2] * imu_data.angle_q[3]), 1 - 2.0f * (imu_data.angle_q[1] * imu_data.angle_q[1] + imu_data.angle_q[2] * imu_data.angle_q[2]))) * 180 / PI;
	imu_data.angle[1] = asin(2.0f * (imu_data.angle_q[0] * imu_data.angle_q[2] - imu_data.angle_q[1] * imu_data.angle_q[3])) * 180 / PI;
	imu_data.angle[2] = atan2(2 * imu_data.angle_q[1] * imu_data.angle_q[2] + 2 * imu_data.angle_q[0] * imu_data.angle_q[3], -2 * imu_data.angle_q[2] * imu_data.angle_q[2] - 2 * imu_data.angle_q[3] * imu_data.angle_q[3] + 1) * 180 / PI; // yaw
}
