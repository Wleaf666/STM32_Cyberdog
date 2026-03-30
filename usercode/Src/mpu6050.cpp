#include "mpu6050.hpp"
#include "cmsis_os2.h"
#include <cmath>

#define MPU_ADDR 0xD0
#define REG_WHO_AM_I 0x75
#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_CFG 0x1C
#define REG_GYRO_CFG 0x1B
#define REG_DLPF_CFG 0x1A
#define REG_DATA 0x3B

bool MPU6050::Init()
{
    uint8_t check;
    uint8_t data;

    HAL_I2C_Mem_Read(hi2c, MPU_ADDR, REG_WHO_AM_I, 1, &check, 1, 100);
    if (check != 0x68)
    {
        return false;
    }

    data = 0x01;
    HAL_I2C_Mem_Write(hi2c, MPU_ADDR, REG_PWR_MGMT_1, 1, &data, 1, 100);
    osDelay(50);

    data = 0x08;
    HAL_I2C_Mem_Write(hi2c, MPU_ADDR, REG_ACCEL_CFG, 1, &data, 1, 100);

    data = 0x10;
    HAL_I2C_Mem_Write(hi2c, MPU_ADDR, REG_GYRO_CFG, 1, &data, 1, 100);

    data = 0x03;
    HAL_I2C_Mem_Write(hi2c, MPU_ADDR, REG_DLPF_CFG, 1, &data, 1, 100);

    this->lastTick = osKernelGetTickCount();

    return true;
}

void MPU6050::Update()
{
    uint8_t data_buffer[14] = {0};
    HAL_I2C_Mem_Read(hi2c, MPU_ADDR, REG_DATA, 1, data_buffer, 14, 100);

    int16_t RawAccleX = (int16_t)((data_buffer[0] << 8) | data_buffer[1]);
    int16_t RawAccleY = ((int16_t)(data_buffer[2] << 8) | data_buffer[3]);
    int16_t RawAccleZ = ((int16_t)(data_buffer[4] << 8) | data_buffer[5]);

    int16_t RawTemperature = ((int16_t)(data_buffer[6] << 8) | data_buffer[7]);

    int16_t RawGyroX = ((int16_t)(data_buffer[8] << 8) | data_buffer[9]);
    int16_t RawGyroY = ((int16_t)(data_buffer[10] << 8) | data_buffer[11]);
    int16_t RawGyroZ = ((int16_t)(data_buffer[12] << 8) | data_buffer[13]);

    this->accelX = RawAccleX / 8192.0f;
    this->accelY = RawAccleY / 8192.0f;
    this->accelZ = RawAccleZ / 8192.0f;

    this->gyroX = RawGyroX / 32.8f;
    this->gyroY = RawGyroY / 32.8f;
    this->gyroZ = RawGyroZ / 32.8f;

    this->temptreture = RawTemperature / 340.0f + 35;

    float accelPitch = atan2(this->accelX, sqrt(this->accelX * this->accelX + this->accelY * this->accelY + this->accelZ * this->accelZ)) *57.29578f;
    float accelRoll = atan2(this->accelY, this->accelZ) * 57.29578f;

    uint32_t currentTick = osKernelGetTickCount();
    uint32_t deltaTick = currentTick - lastTick;

    float dt = (float)deltaTick / (float)osKernelGetTickFreq();

    if(dt<=0.0f||dt>0.1f)
    {
        dt = 0.2f;
    }

    this->pitch = 0.98f * (this->pitch + this->gyroY * dt) + 0.02f * accelPitch;
    this->roll = 0.98f * (this->roll + this->gyroX * dt) + 0.02f * accelRoll;

    this->yaw = this->yaw + this->gyroZ * dt;
}
