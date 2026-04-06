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

osMessageQueueId_t my_hc05_queue;
osMessageQueueId_t my_su03t_queue = nullptr;

// 以后所有的 C++ 头文件（比如你的 static_arena.hpp）都在这里尽情 include！
uint32_t test_count = 0;
uint32_t test_count1 = 0;


osMutexId_t i2c1_mutex = nullptr;
osMutexId_t uart_mutex = nullptr;
osMutexId_t uart2_mutex = nullptr; // 给串口2单独开一把锁

MPU6050 *dogImu = nullptr;
HC05 *blueTooth = nullptr;
SSD1306 *oled = nullptr;
SU03T *voiceModule = nullptr;


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
            oled->Clear();
            oled->DrawString(0, 48, "Voice Command:");

            switch (cmd)
            {
            case VoiceCmd::WAKE_UP:
                oled->DrawString(0, 64, "Status: Awake");
                break;
            case VoiceCmd::FORWARD:
                oled->DrawString(0, 64, "Action: Forward");
                break;
            case VoiceCmd::SIT_DOWN:
                oled->DrawString(0, 64, "Action: Sit Down");
                break;
            default:
                oled->DrawString(0, 64, "Unknown CMD");
                break;
            }
            oled->Update();
        }
        oled->DrawString(0, 48, "NO Command:");
        osDelay(20); // 必须有 Delay，让出 CPU
    }
}

void task_test1(void *argument)
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
    char debug_buf[32];
    for (;;)
    {
        oled->Clear();

        oled->DrawString(0, 0, "CyberDog Status:");

        sprintf(debug_buf , "Count: %lu", test_count1);
        oled->DrawString(0, 16, debug_buf);

        oled->DrawString(0, 32, "Wleaf");

        oled->Update();
        osDelay(50);
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
    my_hc05_queue = osMessageQueueNew(64, sizeof(uint8_t), NULL);
    my_su03t_queue = osMessageQueueNew(32, sizeof(uint8_t), NULL);

    oled->Init();
    // dogImu->Init();
    blueTooth->Init(my_hc05_queue);
    voiceModule->Init(my_su03t_queue);

    osThreadAttr_t task_bluetooth_test_attr = {.priority = osPriorityNormal};
    osThreadAttr_t task_test_attr1 = {.priority = osPriorityNormal};
    osThreadAttr_t task_display_attr = {.priority = osPriorityNormal};
    osThreadAttr_t task_voice_attr = {.priority = osPriorityNormal};

    osThreadNew(task_bluetooth_test, &test_count, &task_bluetooth_test_attr);
    osThreadNew(task_display, NULL, &task_display_attr);
    osThreadNew(task_test1, &test_count1, &task_test_attr1);

    osThreadNew(task_voice_handler, NULL, &task_voice_attr);
}