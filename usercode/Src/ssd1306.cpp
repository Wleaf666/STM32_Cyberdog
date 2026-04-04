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
    if (HAL_I2C_IsDeviceReady(hi2c, address, 3, 10) != HAL_OK)
    {
        device_ready = false;
        return false;
    }
    // 等待屏幕上电稳定
    osDelay(100);

    // SSD1306 标准初始化序列 (点亮屏幕必经的黑魔法咒语)
    WriteCommand(0xAE); // 关闭显示

    WriteCommand(0x20); // 设置内存寻址模式
    WriteCommand(0x00); // 0x00 为水平寻址模式 (非常适合我们一口气发 1024 字节)

    WriteCommand(0xA8); // 设置多路复用率 (Multiplex Ratio)
    WriteCommand(0x3F); // 强制设为 64 行 (0x3F = 63)

    WriteCommand(0xDA); // 设置 COM 引脚硬件配置
    WriteCommand(0x12); // 128x64 标准配置 (强制交叉映射)

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
        for (int x = 0; x < 128; x++)
        {
            for (int page = 0; page < 8; page++)
            {
                // image_data[page + x * 8] 是取模软件生成的列数据
                // buffer[x + page * 128] 是我们显存需要的行数据
                buffer[x + page * 128] = image_data[page + x * 8];
            }
        }
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
    if(!device_ready)
        return;
    if (i2cMutex != nullptr && osMutexAcquire(i2cMutex, osWaitForever) == osOK)
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            // 1. 瞄准行：告诉屏幕我们要画第 i 页 (Page 0 ~ Page 7)
            uint8_t page_cmd = 0xB0 + i;
            HAL_I2C_Mem_Write(hi2c, address, 0x00, 1, &page_cmd, 1, 10);

            // 2. 瞄准列：告诉屏幕我们从第 0 列开始画 (分为低四位 0x00 和高四位 0x10)
            uint8_t col_low = 0x00;
            uint8_t col_high = 0x10;
            HAL_I2C_Mem_Write(hi2c, address, 0x00, 1, &col_low, 1, 10);
            HAL_I2C_Mem_Write(hi2c, address, 0x00, 1, &col_high, 1, 10);

            // 3. 开火：稳稳地把这 128 个字节的数据刷进去！
            HAL_I2C_Mem_Write(hi2c, address, 0x40, 1, &buffer[i * 128], 128, 100);
        }
        osMutexRelease(i2cMutex);
    }
}