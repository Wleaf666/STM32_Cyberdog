#pragma once
#include "main.h" 
#include "cstdint"
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

public:
    MPU6050(I2C_HandleTypeDef *_hi2c):hi2c(_hi2c){};

    bool Init();

    void Update();

    float GetPitch() const { return pitch; }
    float GetRoll() const { return roll; }
    float Getyaw() const { return yaw; }
};
