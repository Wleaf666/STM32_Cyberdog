#include "pca9685.hpp"
#include <cmath>
#include "main.h"

PCA9685::PCA9685(I2C_HandleTypeDef *_hi2c, uint8_t _address, osMutexId_t _i2cMutex ) : hi2c(_hi2c), address(_address), i2cMutex(_i2cMutex) 
{
    for (int i = 0; i < 5;i++)
    {
        servoConfigs[i] = {0.0f, 180.0f, 102, 512, static_cast<uint16_t>(i * 1000)};
    }
}

bool PCA9685::Init(float frequency)
{
    if (HAL_I2C_IsDeviceReady(hi2c,address, 3, 10) != HAL_OK)
    {
        device_ready = false;
        return false;
    }
    device_ready = true;

    writeRegister(PCA9685_MODE1, 0x20);
    osDelay(5);

    setPWMFreq(frequency);
    return true;
}

void PCA9685::setPWMFreq(float freq)
{
    freq = (freq > 1000.0f) ? 1000.0f : ((freq < 24.0f) ? 24.0f : freq);

    float prescaleval = 25000000.0f / (4096.0f * freq);
    uint8_t prescale = static_cast<uint8_t>(std::round(prescaleval) - 1.0f);

    uint8_t oldmode = readRegister(PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10;

    writeRegister(PCA9685_MODE1, newmode);
    writeRegister(PCA9685_PRESCALE, prescale);
    writeRegister(PCA9685_MODE1, oldmode);

    osDelay(5);

    writeRegister(PCA9685_MODE1, oldmode | 0x80);
}

void PCA9685::TurnOff(LegChanel leg)
{
    uint8_t ch = static_cast<uint8_t>(leg);
    // PCA9685 的魔法指令：将 off_tick 的第 12 位写 1（即 4096），可以强制关闭该通道
    setPWM(ch, 0, 4096);
}

void PCA9685::setPWM(uint8_t channel,uint16_t on_tick,uint16_t off_tick)
{
    uint8_t reg = LED0_ON_L + 4 * channel;
    uint8_t data[4] = {static_cast<uint8_t>(on_tick & 0xFF),
                       static_cast<uint8_t>(on_tick >> 8),
                       static_cast<uint8_t>(off_tick & 0xFF),
                       static_cast<uint8_t>(off_tick >> 8)};
    if (i2cMutex != nullptr && osMutexAcquire(i2cMutex, osWaitForever) == osOK)
    {
        uint8_t retry = 3; // 【新增】：最多重试 3 次！

        while (retry > 0)
        {
            // 尝试发送数据
            if (HAL_I2C_Mem_Write(hi2c, address, reg, 1, data, 4, 10) == HAL_OK)
            {
                break; // 【关键】：一旦发送成功，立刻跳出重试循环！
            }

            // ⚠️ 发送失败（遇到电磁干扰）
            // 暴力复苏 I2C 外设
            HAL_I2C_DeInit(hi2c);
            HAL_I2C_Init(hi2c);

            retry--; // 扣除一次重试机会，继续死磕！
        }

        osMutexRelease(i2cMutex);
    }
}

void PCA9685::setServoConfig(LegChanel leg,ServoConfig config)
{
    servoConfigs[static_cast<uint8_t>(leg)] = config;
}

void PCA9685::setAngle(LegChanel leg,float angle)
{
    uint8_t ch = static_cast<uint8_t>(leg);
    ServoConfig &cfg = servoConfigs[ch];

    angle = angle < cfg.min_angle ? cfg.min_angle : ((angle > cfg.max_angle) ? cfg.max_angle : angle);

    float angleRatio = (angle - cfg.min_angle) / (cfg.max_angle - cfg.min_angle);
    uint16_t pulse_width_ticks = static_cast<uint16_t>(cfg.min_tick + angleRatio * (cfg.max_tick - cfg.min_tick));

    uint16_t on_tick = cfg.phase_offset;
    uint16_t off_tick = (on_tick + pulse_width_ticks) % 4096;

    setPWM(ch, on_tick, off_tick);
}

bool PCA9685::writeRegister(uint8_t reg,uint8_t data)
{
    bool success = false;
    if(i2cMutex!=nullptr&&osMutexAcquire(i2cMutex,osWaitForever)==osOK)
    {
        if (HAL_I2C_Mem_Write(hi2c,address,reg,1,&data,1,100)==HAL_OK)
        {
            success = true;
        }
        osMutexRelease(i2cMutex);
    }
    return success;
}

uint8_t PCA9685::readRegister(uint8_t reg)
{
    uint8_t data = 0;
    if (i2cMutex != nullptr && osMutexAcquire(i2cMutex, osWaitForever) == osOK)
    {
        HAL_I2C_Mem_Read(hi2c, address, reg, 1, &data, 1, 100);
        osMutexRelease(i2cMutex);
    }
    return data;
}

