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
#include "motion_4dof.hpp" // 【新增】引入步态控制库

volatile VoiceCmd global_dog_action = VoiceCmd::WAKE_UP;

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
Motion4DOF *motionBrain = nullptr; // 【新增】声明运动小脑指针

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
            // 【关键】：把听到的指令直接挂载到全局动作上！
            global_dog_action = cmd;

            switch (cmd)
            {
            case VoiceCmd::WAKE_UP:
                sprintf(last_voice_cmd, "Voice: Awake");
                break;
            case VoiceCmd::FORWARD:
                sprintf(last_voice_cmd, "Voice: Forward");
                break;
            case VoiceCmd::SIT_DOWN: // 这里我们可以把 SIT_DOWN 当作“停止”来用
                sprintf(last_voice_cmd, "Voice: Stop");
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
        if (blueTooth->getCommand(cmd))
        {
            // 收到手机指令时，让狗叫一声作为反馈（根据需要取消注释）
            voiceModule->playVoice(VoicePlay::BEEP);

            // =========================================================
            // 【全动作映射区】
            // 将手机发来的 Hex 控制码 (cmd.cmd_type) 完美映射到全局枚举
            // 手机按键配置格式为：AA XX 55 (其中 XX 为下方的 Case 值)
            // =========================================================
            switch (cmd.cmd_type)
            {
            // --- 1. 系统与状态控制 ---
            case 0x01:
                global_dog_action = VoiceCmd::WAKE_UP;
                break;
            case 0x02:
                global_dog_action = VoiceCmd::SLEEP;
                break;
            case 0x03:
                global_dog_action = VoiceCmd::REPORT_BAT;
                break;

            // --- 2. 基础移动指令 ---
            case 0x10:
                global_dog_action = VoiceCmd::FORWARD;
                blueTooth->sendString("forward\r\n"); // 串口反馈
                break;
            case 0x11:
                global_dog_action = VoiceCmd::BACKWARD;
                blueTooth->sendString("backward\r\n");
                break;
            case 0x12:
                global_dog_action = VoiceCmd::TURN_LEFT;
                blueTooth->sendString("turn left\r\n");
                break;
            case 0x13:
                global_dog_action = VoiceCmd::TURN_RIGHT;
                blueTooth->sendString("turn right\r\n");
                break;
            case 0x14:
                global_dog_action = VoiceCmd::SHIFT_LEFT;
                break;
            case 0x15:
                global_dog_action = VoiceCmd::SHIFT_RIGHT;
                break;
            case 0x16:
                global_dog_action = VoiceCmd::STOP_MOVE;
                break;

            // --- 3. 静态姿态控制 ---
            case 0x20:
                global_dog_action = VoiceCmd::STAND_UP;
                break;
            case 0x21:
                global_dog_action = VoiceCmd::SIT_DOWN;
                break;
            case 0x22:
                global_dog_action = VoiceCmd::LIE_DOWN;
                break;
            case 0x23:
                global_dog_action = VoiceCmd::LOOK_UP;
                break;
            case 0x24:
                global_dog_action = VoiceCmd::LOOK_DOWN;
                break;
            case 0x25:
                global_dog_action = VoiceCmd::LEAN_LEFT;
                break;
            case 0x26:
                global_dog_action = VoiceCmd::LEAN_RIGHT;
                break;

            // --- 4. 互动与花式动作 ---
            case 0x30:
                global_dog_action = VoiceCmd::SHAKE_HAND_L;
                break;
            case 0x31:
                global_dog_action = VoiceCmd::SHAKE_HAND_R;
                break;
            case 0x32:
                global_dog_action = VoiceCmd::GREETING;
                break;
            case 0x33:
                global_dog_action = VoiceCmd::STRETCH;
                break;
            case 0x34:
                global_dog_action = VoiceCmd::DANCE;
                break;
            case 0x35:
                global_dog_action = VoiceCmd::ATTACK_MODE;
                break;

            // --- 兼容旧版本的停止指令 ---
            case 0x00:
                global_dog_action = VoiceCmd::SIT_DOWN;
                break;

            default:
                break; // 收到未定义的码，忽略
            }

            // 在 OLED 屏幕上同步显示蓝牙发来的十六进制指令，方便你对照调试
            sprintf(last_voice_cmd, "BT: CMD %02X", cmd.cmd_type);
        }

        // 【极其关键】osDelay 必须放在 if 的外面，for 循环的底部！
        // 这样在没有收到蓝牙数据时，任务会主动休眠 10ms 把 CPU 让给步态控制任务
        osDelay(10);
    }
}

void task_display(void *argument)
{
    char dog_pitch[32];
    for (;;)
    {
        if (global_dog_action == VoiceCmd::WAKE_UP || global_dog_action == VoiceCmd::SIT_DOWN)
        {
            oled->Clear();
            oled->DrawString(0, 0, "CyberDog Status:");

            sprintf(dog_pitch, "Pitch:%d", (int16_t)dogImu->GetPitch());
            oled->DrawString(0, 16, dog_pitch);

            oled->DrawString(0, 32, last_voice_cmd);

            oled->Update();
        }

        // 刷新频率依然保持较低的 200ms
        osDelay(200);
    }
}


void task_motion_control(void *argument)
{
    

    osDelay(2000); // 开机等待
    for (;;)
    {
        // 直接调用封装好的执行接口
        motionBrain->ExecuteCommand(global_dog_action);

        // // 如果是单次触发动作（如招手），执行完后自动归位
        // if (global_dog_action == VoiceCmd::SHAKE_HAND_L || global_dog_action == VoiceCmd::SHAKE_HAND_R ||
        //     global_dog_action == VoiceCmd::DANCE)
        // {
        //     global_dog_action = VoiceCmd::WAKE_UP;
        // }

        global_dog_action = VoiceCmd::WAKE_UP;

        osDelay(20);
    }
}

void App_Main()
{

    HAL_Delay(100);

    i2c1_mutex = osMutexNew(NULL);
    uart_mutex = osMutexNew(NULL);
    uart2_mutex = osMutexNew(NULL);

    oled = new SSD1306(&hi2c1, i2c1_mutex);
    voiceModule = new SU03T(&huart2, uart2_mutex);
    dogImu = new MPU6050(&hi2c1, i2c1_mutex);
    blueTooth = new HC05(&huart1, uart_mutex);
    pca9685 = new PCA9685(&hi2c1, 0x80, i2c1_mutex);
    motionBrain = new Motion4DOF(pca9685); // 【新增】实例化小脑，把肌肉(pca9685)传进去

    my_hc05_queue = osMessageQueueNew(64, sizeof(uint8_t), NULL);
    my_su03t_queue = osMessageQueueNew(32, sizeof(uint8_t), NULL);

    oled->Init();
    dogImu->Init();
    blueTooth->Init(my_hc05_queue);
    voiceModule->Init(my_su03t_queue);
    pca9685->Init();
    motionBrain->Init(); // 【新增】初始化步态相位参数



    osThreadAttr_t task_bluetooth_test_attr = {.priority = osPriorityNormal};
    osThreadAttr_t task_Mpu6050_attr = {.priority = osPriorityNormal};
    osThreadAttr_t task_display_attr = {.priority = osPriorityNormal};
    osThreadAttr_t task_voice_attr = {.priority = osPriorityNormal};

    osThreadAttr_t task_motion_attr = {.priority = osPriorityNormal};

    osThreadNew(task_bluetooth_test, &test_count, &task_bluetooth_test_attr);
    osThreadNew(task_display, NULL, &task_display_attr);
    osThreadNew(task_Mpu6050, &test_count1, &task_Mpu6050_attr);

    osThreadNew(task_voice_handler, NULL, &task_voice_attr);

    osThreadNew(task_motion_control, NULL, &task_motion_attr);
}