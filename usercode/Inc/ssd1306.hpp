#pragma once

#include "main.h"
#include "cmsis_os2.h"
#include <cstring>

class SSD1306
{
private:
    I2C_HandleTypeDef *hi2c;
    osMutexId_t i2cMutex;
    uint8_t address;

    bool device_ready;

    uint8_t buffer[1024];

    void WriteCommand(uint8_t cmd);

    void WriteData(uint8_t *data, uint16_t size);

public:
    // 默认 I2C 地址通常为 0x78 (也有少部分模块是 0x7A)
    SSD1306(I2C_HandleTypeDef *_hi2c, osMutexId_t _i2cMutex, uint8_t _address = 0x78);

    bool Init();

    void Clear();

    void DrawImage(const uint8_t *image_data);

    void DrawPixel(int16_t x, int16_t y, bool color);

    void DrawChar(int16_t x, int16_t y, char c);

    void DrawString(int16_t x, int16_t y, const char *str);

    void Update();
};