/**
 * @file Imu.h
 * @author jierui778 (758418101@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef IMU_H
#define IMU_H
#include "main.h"

#ifndef PI
#define PI 3.1415926535f
#endif

/*BMI088寄存器地址*/
#define BMI088_CHIP_ID_ADDR 0X00

#define BMI088_ACCEL_SOFTRESET_ADDR 0x7E
#define BMI088_ACCEL_SOFTRESET_VAL 0xB6

#define BMI088_ACCEL_X_LSB_ADDR 0X12
#define BMI088_ACCEL_X_MSB_ADDR 0X13
#define BMI088_ACCEL_Y_LSB_ADDR 0X14
#define BMI088_ACCEL_Y_MSB_ADDR 0X15
#define BMI088_ACCEL_Z_LSB_ADDR 0X16
#define BMI088_ACCEL_Z_MSB_ADDR 0X17

#define BMI088_ACCEL_CONF_ADDR 0x40
#define BMI088_ACCEL_CONF_RESERVED 0x01
#define BMI088_ACCEL_CONF_BWP_OSR4 0x00
#define BMI088_ACCEL_CONF_BWP_OSR2 0x01
#define BMI088_ACCEL_CONF_BWP_NORM 0x02
#define BMI088_ACCEL_CONF_ODR_12_5_Hz 0x05
#define BMI088_ACCEL_CONF_ODR_25_Hz 0x06
#define BMI088_ACCEL_CONF_ODR_50_Hz 0x07
#define BMI088_ACCEL_CONF_ODR_100_Hz 0x08
#define BMI088_ACCEL_CONF_ODR_200_Hz 0x09
#define BMI088_ACCEL_CONF_ODR_400_Hz 0x0A
#define BMI088_ACCEL_CONF_ODR_800_Hz 0x0B
#define BMI088_ACCEL_CONF_ODR_1600_Hz 0x0C
#define BMI088_ACCEL_RANGE_ADDR 0x41
#define BMI088_ACCEL_RANGE_3G 0x00
#define BMI088_ACCEL_RANGE_3G 0x00
#define BMI088_ACCEL_RANGE_6G 0x01
#define BMI088_ACCEL_RANGE_12G 0x02
#define BMI088_ACCEL_RANGE_24G 0x03

#define BMI088_ACCEL_PWR_CTRL_ADDR 0x7D
#define BMI088_ACCEL_PWR_CTRL_ON 0x04
#define BMI088_ACCEL_PWR_CTRL_OFF 0x00

#define BMI088_GYRO_SOFTRESET_ADDR 0x14
#define BMI088_GYRO_SOFTRESET_VAL 0xB6

#define BMI088_GYRO_X_LSB_ADDR 0x02
#define BMI088_GYRO_X_MSB_ADDR 0x03
#define BMI088_GYRO_Y_LSB_ADDR 0x04
#define BMI088_GYRO_Y_MSB_ADDR 0x05
#define BMI088_GYRO_Z_LSB_ADDR 0x06
#define BMI088_GYRO_Z_MSB_ADDR 0x07
#define BMI088_GYRO_RANGE_ADDR 0x0F
#define BMI088_GYRO_RANGE_2000_DEG_S 0x00
#define BMI088_GYRO_RANGE_1000_DEG_S 0x01
#define BMI088_GYRO_RANGE_500_DEG_S 0x02
#define BMI088_GYRO_RANGE_250_DEG_S 0x03
#define BMI088_GYRO_RANGE_125_DEG_S 0x04
#define BMI088_GYRO_BANDWIDTH_ADDR 0x10
#define BMI088_GYRO_ODR_2000Hz_BANDWIDTH_532Hz 0x00
#define BMI088_GYRO_ODR_2000Hz_BANDWIDTH_230Hz 0x01
#define BMI088_GYRO_ODR_1000Hz_BANDWIDTH_116Hz 0x02
#define BMI088_GYRO_ODR_400Hz_BANDWIDTH_47Hz 0x03
#define BMI088_GYRO_ODR_200Hz_BANDWIDTH_23Hz 0x04
#define BMI088_GYRO_ODR_100Hz_BANDWIDTH_12Hz 0x05
#define BMI088_GYRO_ODR_200Hz_BANDWIDTH_64Hz 0x06
#define BMI088_GYRO_ODR_100Hz_BANDWIDTH_32Hz 0x07

#define BMI088_GYRO_LPM1_ADDR 0x11
#define BMI088_GYRO_LPM1_NOR 0x00
#define BMI088_GYRO_LPM1_SUS 0x80
#define BMI088_GYRO_LPM1_DEEP_SUS 0x20

#define BMI088_TEMP_MSB_ADDR 0x22 // 低三位  [2:0]
#define BMI088_TEMP_LSB_ADDR 0x23 // 高8位   [10:3]
#define BMI088_TEMP_LEN 2
#define BMI088_TEMP_UNIT 0.125f
#define BMI088_TEMP_BIAS 23.0f

/*IST8310寄存器地址*/
#define IST8310_CHIP_ID_ADDR 0x00
#define IST8310_CHIP_ID_VAL 0x10
#define IST8310_STAT1_ADDR 0x02
#define IST8310_DATA_XL_ADDR 0x03
#define IST8310_DATA_XH_ADDR 0x04
#define IST8310_DATA_YL_ADDR 0x05
#define IST8310_DATA_YH_ADDR 0x06
#define IST8310_DATA_ZL_ADDR 0x07
#define IST8310_DATA_ZH_ADDR 0x08
#define IST8310_STAT2_ADDR 0x09
#define IST8310_CNTL1_ADDR 0x0A
#define IST8310_CNTL1_SLEEP 0x00
#define IST8310_CNTL1_SINGLE 0x01
#define IST8310_CNTL1_CONTINUE 0x0B
#define IST8310_CNTL2_ADDR 0x0B
#define IST8310_STAT2_NONE_ALL 0x00
#define IST8310_SELF_CHECK_ADDR 0x0C
#define IST8310_TEMPL_ADDR 0x1C
#define IST8310_TEMPH_ADDR 0x1D
#define IST8310_AVGCNTL_ADDR 0x41
#define IST8310_AVGCNTL_TWICE 0x09
#define IST8310_AVGCNTL_FOURTH 0x12

#define IST8310_I2C_ADDR 0x0E // IST8310从机地址
#define IST8310_ID 0x10		  // IST8310的ID号

typedef struct
{
	int16_t gyro[3];  // 陀螺仪
	int16_t accel[3]; // 加速度计
	int16_t mag[3];	  // 磁力计
	float gyro_f[3];  // 陀螺仪
	float accel_f[3]; // 加速度计
	float mag_f[3];	  // 磁力计
	float angle_q[4]; // 四元数
	float angle[3];	  // 欧拉角
	uint8_t temp;	  // 温度
} IMU_TypeDef;

extern IMU_TypeDef imu_data;
uint8_t IST8310_Init(void);
void IST8310_Read(IMU_TypeDef *imu);

uint8_t BMI088_Init(void);
void BMI088_Read(IMU_TypeDef *imu);
void BMI088_ReadGyro(IMU_TypeDef *imu);
void BMI088_ReadAccel(IMU_TypeDef *imu);
void IMU_ReadData(IMU_TypeDef *imu);
#endif // __IMU_H
