/**
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

static void IST8310_WriteByte(uint8_t reg, uint8_t data);
static uint8_t IST8310_ReadByte(uint8_t reg);

static void BMI088_WriteByte(uint8_t reg, uint8_t data);
static void BMI088_ReadByte(uint8_t reg, uint8_t *data);

IMU_TypeDef imu_data;

/**
 * @brief 向IST8310写入一个字节
 *
 * @param reg 寄存器地址
 * @param data 要写入的数据
 */
static void IST8310_WriteByte(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c3, (IST8310_I2C_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 50);
}
/**
 * @brief 读取IST8310一个字节
 *
 * @param reg 寄存器地址
 * @return uint8_t 读取到的数据
 */
static uint8_t IST8310_ReadByte(uint8_t reg)
{
    uint8_t data;
    HAL_I2C_Mem_Read(&hi2c3, (IST8310_I2C_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
    return data;
}
/**
 * @brief IST8310初始化
 *
 */
uint8_t IST8310_Init(void)
{
    int8_t data = 0;

    /*重启磁力计*/
    HAL_GPIO_WritePin(IST8310_Reset_GPIO_Port, IST8310_Reset_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(IST8310_Reset_GPIO_Port, IST8310_Reset_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    // data = IST8310_ReadByte(IST8310_CHIP_ID_ADDR); // 读取ID，判断是否连接成功
    // if (data != IST8310_CHIP_ID_VAL)
    // {
    //     return 1; // 连接失败
    // }

    IST8310_WriteByte(IST8310_STAT2_ADDR, IST8310_STAT2_NONE_ALL);   // 不使能中断，直接读取数据
    IST8310_WriteByte(IST8310_AVGCNTL_ADDR, IST8310_AVGCNTL_FOURTH); // 平均采样四次
    IST8310_WriteByte(IST8310_CNTL1_ADDR, IST8310_CNTL1_CONTINUE);   // 输出频率200Hz
    return 0;                                                        // 连接成功
}
/**
 * @brief 读取IST8310数据
 *
 * @param imu 保存数据的结构体
 */
void IST8310_Read(IMU_TypeDef *imu)
{
    uint8_t buf[6]; // 保存读取到的数据
    // uint16_t temp_data; //
    HAL_I2C_Mem_Read(&hi2c3, (IST8310_I2C_ADDR << 1), IST8310_DATA_XL_ADDR, I2C_MEMADD_SIZE_8BIT, buf, 6, 50);
    imu->mag[0] = buf[0] + (buf[1] << 8);
    imu->mag[1] = buf[2] + (buf[3] << 8);
    imu->mag[2] = buf[4] + (buf[5] << 8);
}
/**
 * @brief BMI088初始化
 *
 * @return uint8_t 0:成功 1:失败
 */
uint8_t BMI088_Init(void)
{
    return 0;
}

static void BMI088_WriteByte(uint8_t reg, uint8_t data)
{
    HAL_SPI_Transmit(&hspi1, &reg, 1, 50);
}

void BMI088_ReadGyro(IMU_TypeDef *imu)
{
    uint8_t buf[6];
    HAL_GPIO_WritePin(BMI088_Gyro_CS_GPIO_Port, BMI088_Accel_CS_Pin, GPIO_PIN_RESET); // 使能陀螺仪
}

void BMI088_ReadAccel(IMU_TypeDef *imu)
{
    uint8_t buf[6];
    HAL_GPIO_WritePin(BMI088_Accel_CS_GPIO_Port, BMI088_Accel_CS_Pin, GPIO_PIN_RESET); // 使能加速度计
}

void IMU_Read(IMU_TypeDef *imu)
{
    // uint8_t buf[20];
    // HAL_SPI_TransmitReceive(&hspi1, buf, buf, 20, 50);
    // imu->gyro[0] = (buf[1] << 8) | buf[0];
    // imu->gyro[1] = (buf[3] << 8) | buf[2];
    // imu->gyro[2] = (buf[5] << 8) | buf[4];
    // imu->accel[0] = (buf[7] << 8) | buf[6];
    // imu->accel[1] = (buf[9] << 8) | buf[8];
    // imu->accel[2] = (buf[11] << 8) | buf[10];
    // imu->temp = (buf[13] << 8) | buf[12];
    // imu->mag[0] = (buf[15] << 8) | buf[14];
    // imu->mag[1] = (buf[17] << 8) | buf[16];
    // imu->mag[2] = (buf[19] << 8) | buf[18];
}