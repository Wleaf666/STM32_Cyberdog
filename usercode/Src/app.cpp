#include "app.hpp"
#include "main.h"
#include "cmsis_os2.h"
#include "mpu6050.hpp"
#include "i2c.h"
#include "hc05.hpp"
#include "usart.h"

osMessageQueueId_t my_hc05_queue;

// 以后所有的 C++ 头文件（比如你的 static_arena.hpp）都在这里尽情 include！
uint32_t test_count = 0;
uint32_t test_count1 = 0;

osMutexId_t i2c1_mutex = osMutexNew(NULL);

MPU6050 dogImu(&hi2c1, i2c1_mutex);

osMutexId_t uart_mutex = osMutexNew(NULL);

HC05 blueTooth(&huart1, uart_mutex);

void HC05_RxCallback_Wrapper(UART_HandleTypeDef *huart)
{
    blueTooth.onRxCpltCallback();
}

// void task_test1(void *argument)
// {
//     for (;;)
//     {
//         (*(uint32_t *)argument)++;
//         // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//         dogImu.Update();
//         osDelay(20);
//     }
// }

void task_bluetooth_test(void *argument)
{
    RobotCommand cmd;
    for (;;)
    {
        (*(uint32_t *)argument)++;
        if (blueTooth.getCommand(cmd))
        {
            blueTooth.sendString("Got a Valid Packet!\r\n");

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

void App_Main()
{

    // osThreadAttr_t task_test_attr1 = {.priority = osPriorityNormal};
    // osThreadNew(task_test1, &test_count1, &task_test_attr1);
    osThreadAttr_t task_bluetooth_test_attr = {.priority = osPriorityNormal};
    osThreadNew(task_bluetooth_test, &test_count, &task_bluetooth_test_attr);

    my_hc05_queue = osMessageQueueNew(64, sizeof(uint8_t), NULL);
    // dogImu.Init();
    // blueTooth.Init(hc05RxQueueHandle);
    blueTooth.Init(my_hc05_queue);
}