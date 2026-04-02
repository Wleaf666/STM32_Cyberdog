#include "app.hpp"
#include "main.h"
#include "cmsis_os2.h"
#include "mpu6050.hpp"
#include "i2c.h"

// 以后所有的 C++ 头文件（比如你的 static_arena.hpp）都在这里尽情 include！
uint32_t test_count = 0;
uint32_t test_count1 = 0;

osMutexId_t i2c1_mutex = osMutexNew(NULL);

MPU6050 dogImu(&hi2c1,i2c1_mutex);

void task_test(void *argument)
{
    for (;;)
    {
        (*(uint32_t *)argument)++;
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        osDelay(500);
    }
}

void task_test1(void *argument)
{
    for (;;)
    {
        (*(uint32_t *)argument)++;
        // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        dogImu.Update();
        osDelay(20);
    }
}

void App_Main()
{
    // 🎉 欢迎来到纯正的 C++17 世界！
    // 以后你的 FreeRTOS 任务创建、类的实例化、机器狗逻辑，全部写在这个函数里。
    osThreadAttr_t task_test_attr = {.priority = osPriorityNormal};
    osThreadNew(task_test, &test_count, &task_test_attr);
    osThreadAttr_t task_test_attr1 = {.priority = osPriorityNormal};
    osThreadNew(task_test1, &test_count1, &task_test_attr1);
    dogImu.Init();
}