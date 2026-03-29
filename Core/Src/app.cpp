#include "app.hpp"
#include "main.h"
// 以后所有的 C++ 头文件（比如你的 static_arena.hpp）都在这里尽情 include！

void App_Main()
{
    // 🎉 欢迎来到纯正的 C++17 世界！
    // 以后你的 FreeRTOS 任务创建、类的实例化、机器狗逻辑，全部写在这个函数里。

    while (1)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(250);
    }
}