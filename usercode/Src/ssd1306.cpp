#include "ssd1306.hpp"


SSD1306::SSD1306(I2C_HandleTypeDef *_hi2c, osMutexId_t _i2cMutex, uint8_t _address)
    : hi2c(_hi2c), i2cMutex(_i2cMutex), address(_address)
{

    memset(buffer, 0, sizeof(buffer));
}

void SSD1306::WriteCommand(uint8_t cmd)
{

    if (i2cMutex != nullptr && osMutexAcquire(i2cMutex, osWaitForever) == osOK)
    {

        HAL_I2C_Mem_Write(hi2c, address, 0x00, 1, &cmd, 1, 100);
        osMutexRelease(i2cMutex);
    }
}

void SSD1306::WriteData(uint8_t *data, uint16_t size)
{
    if (i2cMutex != nullptr && osMutexAcquire(i2cMutex, osWaitForever) == osOK)
    {

        HAL_I2C_Mem_Write(hi2c, address, 0x40, 1, data, size, 100);
        osMutexRelease(i2cMutex);
    }
}

bool SSD1306::Init()
{
    // 等待屏幕上电稳定
    osDelay(100);

    // SSD1306 标准初始化序列 (点亮屏幕必经的黑魔法咒语)
    WriteCommand(0xAE); // 关闭显示

    WriteCommand(0x20); // 设置内存寻址模式
    WriteCommand(0x00); // 0x00 为水平寻址模式 (非常适合我们一口气发 1024 字节)

    WriteCommand(0x81); // 设置对比度
    WriteCommand(0xFF); // 最大对比度 (最亮)

    WriteCommand(0xA1); // 段重映射 (左右反转，如果你的屏幕画面是反的，改成 0xA0)
    WriteCommand(0xC8); // COM 扫描方向 (上下反转，如果画面倒了，改成 0xC0)

    WriteCommand(0x8D); // 开启电荷泵 (必须开，否则屏幕黑的)
    WriteCommand(0x14);

    WriteCommand(0xAF); // 开启显示

    // 初始化后，把全黑的显存刷上去，防止上电时的雪花屏
    Update();
    return true;
}

void SSD1306::Clear()
{
    memset(buffer, 0, sizeof(buffer));
}

void SSD1306::DrawImage(const uint8_t *image_data)
{
    // 直接把传入的图像数组，拷贝到我们的显存里 (耗时不到 1 微秒)
    if (image_data != nullptr)
    {
        memcpy(buffer, image_data, sizeof(buffer));
    }
}

void SSD1306::DrawPixel(int16_t x, int16_t y, bool color)
{
    // 防越界保护
    if (x >= 128 || x < 0 || y >= 64 || y < 0)
        return;

    // 核心算法：计算 (x,y) 在一维数组中对应的具体的某一个 Bit
    if (color)
    {
        buffer[x + (y / 8) * 128] |= (1 << (y % 8)); // 点亮
    }
    else
    {
        buffer[x + (y / 8) * 128] &= ~(1 << (y % 8)); // 熄灭
    }
}

void SSD1306::Update()
{
    // 告诉 OLED：我要从第 0 列画到第 127 列
    WriteCommand(0x21);
    WriteCommand(0);
    WriteCommand(127);

    // 告诉 OLED：我要从第 0 页 (Page) 画到第 7 页
    WriteCommand(0x22);
    WriteCommand(0);
    WriteCommand(7);

    // 一口气把 1024 字节的显存全部丢进去！
    // 为了防止占用 I2C 总线时间过长，我们分成 8 块 (每次发一整页 128 字节)
    if (i2cMutex != nullptr && osMutexAcquire(i2cMutex, osWaitForever) == osOK)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            HAL_I2C_Mem_Write(hi2c, address, 0x40, 1, &buffer[i * 128], 128, 100);
        }
        osMutexRelease(i2cMutex);
    }
}