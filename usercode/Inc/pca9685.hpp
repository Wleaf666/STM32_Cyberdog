#pragma once

#include "main.h"
#include "cmsis_os2.h"

enum class LegChanel : uint8_t
{
    FrontLeft = 0,
    FrontRight = 1,
    RearLeft = 2,
    RearRight = 3
};

struct  ServoConfig
{
    float min_angle;
    float max_angle;
    uint16_t min_tick;
    uint16_t max_tick;
    uint16_t phase_offset;
};


class PCA9685
{
    private:
        I2C_HandleTypeDef* hi2c;
        uint8_t address;
        osMutexId_t i2cMutex;

        ServoConfig servoConfigs[4];

        bool device_ready;

                static constexpr uint8_t PCA9685_MODE1 = 0x00;
        static constexpr uint8_t PCA9685_PRESCALE = 0xFE;
        static constexpr uint8_t LED0_ON_L = 0x06;

        bool writeRegister(uint8_t reg, uint8_t data);
        uint8_t readRegister(uint8_t reg);
    public:
        PCA9685(I2C_HandleTypeDef *_hi2c, uint8_t _address = 0x80, osMutexId_t _i2cMutex = nullptr);

        bool Init(float frequecy = 50.0f);

        void setPWMFreq(float freq);

        void setPWM(uint8_t channel, uint16_t on_tick, uint16_t off_tick);

        void setAngle(LegChanel leg, float angle);

        void setServoConfig(LegChanel leg, ServoConfig config);
};
