#include "app.hpp"
#include "main.h"
#include "cmsis_os2.h"
#include "mpu6050.hpp"
#include "i2c.h"
#include "hc05.hpp"
#include "usart.h"
#include "ssd1306.hpp"
#include "stdio.h"
#include "su03t.hpp"
#include "pca9685.hpp"

osMessageQueueId_t my_hc05_queue;
osMessageQueueId_t my_su03t_queue = nullptr;

// 以后所有的 C++ 头文件（比如你的 static_arena.hpp）都在这里尽情 include！
uint32_t test_count = 0;
uint32_t test_count1 = 0;
char last_voice_cmd[32] = "Voice: None";

osMutexId_t i2c1_mutex = nullptr;
osMutexId_t uart_mutex = nullptr;
osMutexId_t uart2_mutex = nullptr; // 给串口2单独开一把锁

MPU6050 *dogImu = nullptr;
HC05 *blueTooth = nullptr;
SSD1306 *oled = nullptr;
SU03T *voiceModule = nullptr;
PCA9685 *pca9685 = nullptr;

void HC05_RxCallback_Wrapper(UART_HandleTypeDef *huart)
{
    if (blueTooth != nullptr)
    {
        blueTooth->onRxCpltCallback();
    }
}

void SU03T_RxCallback_Wrapper(UART_HandleTypeDef *huart)
{
    if (voiceModule != nullptr)
    {
        voiceModule->onRxCpltCallback();
    }
}

void task_voice_handler(void *argument)
{
    for (;;)
    {
        VoiceCmd cmd = voiceModule->getCommand();

        if (cmd != VoiceCmd::NONE)
        {
            switch (cmd)
            {
            case VoiceCmd::WAKE_UP:
                sprintf(last_voice_cmd, "Voice: Awake");
                break;
            case VoiceCmd::FORWARD:
                sprintf(last_voice_cmd, "Voice: Forward");
                break;
            case VoiceCmd::SIT_DOWN:
                sprintf(last_voice_cmd, "Voice: Sit Down");
                break;
            default:
                sprintf(last_voice_cmd, "Voice: Unknown");
                break;
            }
        }
        osDelay(20);
    }
}

void task_Mpu6050(void *argument)
{
    for (;;)
    {
        (*(uint32_t *)argument)++;
        // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        dogImu->Update();
        osDelay(20);
    }
}

void task_bluetooth_test(void *argument)
{
    RobotCommand cmd;
    for (;;)
    {
        (*(uint32_t *)argument)++;
        if (blueTooth->getCommand(cmd))
        {
            blueTooth->sendString("Got a Valid Packet!\r\n");

            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            voiceModule->playVoice(VoicePlay::BEEP);
        }
        // uint8_t raw_byte;
        // if (osMessageQueueGet(my_hc05_queue, &raw_byte, NULL, 0) == osOK)
        // {
        //     // 收到啥字母，就立刻原样弹回去！
        //     blueTooth.sendBytes(&raw_byte, 1);

        //     // 闪烁小灯，给你视觉反馈
        //     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        // }

        osDelay(10);
    }
}

void task_display(void *argument)
{
    char dog_pitch[32];
    for (;;)
    {
        oled->Clear();
        oled->DrawString(0, 0, "CyberDog Status:");

        // 画 IMU 数据
        sprintf(dog_pitch, "Pitch:%d", (int16_t)dogImu->GetPitch());
        oled->DrawString(0, 16, dog_pitch);

        // 画出刚刚最后一次听到的语音指令
        oled->DrawString(0, 32, last_voice_cmd);

        oled->Update();
        osDelay(50);
    }
}

void task_pca9685(void *argument)
{
    osDelay(3000);

    for (;;)
    {
        // pca9685->setAngle(LegChanel::FrontLeft, 0.0f);
        // osDelay(1000); // 间隔 1 秒

        pca9685->setAngle(LegChanel::FrontRight, 0.0f);
        osDelay(1000);

        // pca9685->setAngle(LegChanel::RearLeft, 0.0f);
        // osDelay(1000);

        // pca9685->setAngle(LegChanel::RearRight, 0.0f);
        // osDelay(1000);

        // 2. 全部回到 90 度（中位），这才是你装狗腿的基准点！
        // pca9685->setAngle(LegChanel::FrontLeft, 90.0f);
        pca9685->setAngle(LegChanel::FrontRight, 90.0f);
        // pca9685->setAngle(LegChanel::RearLeft, 90.0f);
        // pca9685->setAngle(LegChanel::RearRight, 90.0f);

        // 在中位休息 3 秒，然后再循环
        osDelay(3000);
    }
}

    void App_Main()
{

    i2c1_mutex = osMutexNew(NULL);
    uart_mutex = osMutexNew(NULL);
    uart2_mutex = osMutexNew(NULL);

    oled = new SSD1306(&hi2c1, i2c1_mutex);
    voiceModule = new SU03T(&huart2, uart2_mutex);
    dogImu = new MPU6050(&hi2c1, i2c1_mutex);
    blueTooth = new HC05(&huart1, uart_mutex);
    pca9685 = new PCA9685(&hi2c1, 0x80,i2c1_mutex);


    my_hc05_queue = osMessageQueueNew(64, sizeof(uint8_t), NULL);
    my_su03t_queue = osMessageQueueNew(32, sizeof(uint8_t), NULL);

    oled->Init();
    dogImu->Init();
    blueTooth->Init(my_hc05_queue);
    voiceModule->Init(my_su03t_queue);
    pca9685->Init();

    osThreadAttr_t task_bluetooth_test_attr = {.priority = osPriorityNormal};
    osThreadAttr_t task_Mpu6050_attr = {.priority = osPriorityNormal};
    osThreadAttr_t task_display_attr = {.priority = osPriorityNormal};
    osThreadAttr_t task_voice_attr = {.priority = osPriorityNormal};
    osThreadAttr_t task_pca9685_attr = {.priority = osPriorityNormal};

    osThreadNew(task_bluetooth_test, &test_count, &task_bluetooth_test_attr);
    osThreadNew(task_display, NULL, &task_display_attr);
    osThreadNew(task_Mpu6050, &test_count1, &task_Mpu6050_attr);

    osThreadNew(task_voice_handler, NULL, &task_voice_attr);
    osThreadNew(task_pca9685, NULL, &task_pca9685_attr);
}