#pragma once
#include "main.h" 
#include "cstdint"
#include "cmsis_os2.h"

class MPU6050
{
private:
    I2C_HandleTypeDef *hi2c; 
    uint16_t deviceAddress;  

    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;

    float temptreture;

    float pitch, roll, yaw;

    uint32_t lastTick;

    osMutexId_t i2cMutex;

public:
    MPU6050(I2C_HandleTypeDef *_hi2c,osMutexId_t _i2cMutex=nullptr):hi2c(_hi2c),i2cMutex(_i2cMutex){}

    bool Init();

    void Update();

    float GetPitch() const { return pitch; }
    float GetRoll() const { return roll; }
    float Getyaw() const { return yaw; }
};
