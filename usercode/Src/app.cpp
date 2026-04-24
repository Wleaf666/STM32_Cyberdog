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
#include "adc.h"           // 确保你的 CubeMX 生成了 adc.h
#include <cmath>

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

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 1. 暴力清除所有错误标志位 (溢出、噪音、帧错误)
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);

    // 2. 重新强行开启中断接收！
    if (huart->Instance == USART1 && blueTooth != nullptr)
    {
        // 蓝牙挂了，重新初始化蓝牙接收
        blueTooth->Init(my_hc05_queue);
    }
    if (huart->Instance == USART2 && voiceModule != nullptr)
    {
        // 语音挂了，重新初始化语音接收
        voiceModule->Init(my_su03t_queue);
    }
}

uint8_t GetBatteryPercentage()
{
    // 启动 ADC 采样
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        uint32_t raw_adc = HAL_ADC_GetValue(&hadc1);

        // 【极其重要】：你测的是 7.4V，单片机只能测 3.3V 以内。
        // 你必定使用了电阻分压。假设你用了 10k 和 2k 的电阻分压，比例就是 (10+2)/2 = 6.0倍。
        // ⚠️ 请把这里的 6.0f 替换成你真实的硬件分压倍数！
        float divide_ratio = 6.0f;

        float voltage = (raw_adc / 4096.0f) * 3.3f * divide_ratio;

        // 2S 锂电池：满电约 8.4V，没电保护约 6.4V
        float percent_f = (voltage - 6.4f) / (8.4f - 6.4f) * 100.0f;

        int percent = (int)percent_f;
        if (percent > 100)
            percent = 100;
        if (percent < 0)
            percent = 0;

        return (uint8_t)percent;
    }
    return 0; // 读取失败默认返回0
}

// ==========================================
// 2. Cozmo 风格矢量动态表情渲染引擎
// ==========================================
void DrawDynamicFace(VoiceCmd cmd, uint32_t tick)
{
    // 计算全局平滑悬浮偏移量 (利用正弦波产生呼吸感，周期约1.2秒，上下浮动3像素)
    int y_breathe = (int)(sin(tick / 200.0f) * 3.0f);

    // 计算全局眨眼触发器 (每隔 4000 毫秒，触发一次 150 毫秒的闭眼)
    bool is_blinking = (tick % 4000) < 150;

    switch (cmd)
    {
    case VoiceCmd::WAKE_UP:
    case VoiceCmd::STAND_UP:
    case VoiceCmd::STOP_MOVE:
    case VoiceCmd::REPORT_BAT:
        // 【待机状态】：带呼吸感和自动眨眼的好奇大眼睛
        if (is_blinking)
        {
            // 眨眼瞬间：画一条细线
            oled->FillRect(32, 34 + y_breathe, 24, 4, true);
            oled->FillRect(72, 34 + y_breathe, 24, 4, true);
        }
        else
        {
            // 正常睁眼：随呼吸上下浮动
            oled->FillRect(32, 20 + y_breathe, 24, 30, true);
            oled->FillRect(72, 20 + y_breathe, 24, 30, true);
        }
        break;

    case VoiceCmd::SLEEP:
    case VoiceCmd::SIT_DOWN:
    case VoiceCmd::LIE_DOWN:
        // 【睡觉状态】：眼睛眯成一条缝，右上角带有动态弹出的 Zzz 动画
        oled->FillRect(32, 38, 24, 6, true);
        oled->FillRect(72, 38, 24, 6, true);

        // 动态 Zzz 计算：每 600ms 增加一个 Z，循环 3 个状态
        {
            int z_state = (tick / 600) % 4;
            if (z_state >= 1)
                oled->DrawString(95, 30, "Z");
            if (z_state >= 2)
                oled->DrawString(105, 20, "z");
            if (z_state >= 3)
                oled->DrawString(115, 10, "z");
        }
        break;

    case VoiceCmd::FORWARD:
    case VoiceCmd::BACKWARD:
        // 【运动状态】：专注且警觉！眼睛变宽变扁，取消眨眼，且带有微微的震动感模拟发力
        {
            int shake = (tick % 100 < 50) ? 1 : -1; // 50ms的高频震动
            oled->FillRect(28, 16 + shake, 28, 20, true);
            oled->FillRect(72, 16 + shake, 28, 20, true);
        }
        break;

    case VoiceCmd::TURN_LEFT:
    case VoiceCmd::SHIFT_LEFT:
        // 【左看状态】：眼球移到极左，且左眼变小（模拟透视感）
        oled->FillRect(10, 24, 18, 24, true);
        oled->FillRect(45, 20, 24, 30, true);
        break;

    case VoiceCmd::TURN_RIGHT:
    case VoiceCmd::SHIFT_RIGHT:
        // 【右看状态】：眼球移到极右，且右眼变小
        oled->FillRect(59, 20, 24, 30, true);
        oled->FillRect(100, 24, 18, 24, true);
        break;

    case VoiceCmd::ATTACK_MODE:
        // 【攻击状态】：极其凶狠！取消眨眼，倒八字眉毛闪烁 (警告意味)
        oled->FillRect(32, 32, 24, 18, true);
        oled->FillRect(72, 32, 24, 18, true);

        // 眉毛以 200ms 频率红暴闪烁
        if (tick % 200 < 100)
        {
            for (int i = 0; i < 8; i++)
            {
                oled->DrawLine(16, 18 + i, 56, 32 + i, true);  //
                oled->DrawLine(112, 18 + i, 72, 32 + i, true); // /
            }
        }
        break;

    case VoiceCmd::SHAKE_HAND_L:
    case VoiceCmd::SHAKE_HAND_R:
    case VoiceCmd::GREETING:
    case VoiceCmd::DANCE:
    case VoiceCmd::STRETCH:
        // 【开心状态】：极其开心的笑眼 ^ ^ ，伴随激动的上下跳动
        {
            int y_jump = (tick % 300 < 150) ? -4 : 0; // 快速跳动
            for (int i = 0; i < 6; i++)
            {
                oled->DrawLine(30, 35 + i + y_jump, 44, 18 + i + y_jump, true);
                oled->DrawLine(44, 18 + i + y_jump, 58, 35 + i + y_jump, true);

                oled->DrawLine(70, 35 + i + y_jump, 84, 18 + i + y_jump, true);
                oled->DrawLine(84, 18 + i + y_jump, 98, 35 + i + y_jump, true);
            }
        }
        break;

    default:
        oled->FillRect(32, 20, 24, 30, true);
        oled->FillRect(72, 20, 24, 30, true);
        break;
    }
}

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
    char text_buffer[32];
    for (;;)
    {
        oled->Clear();

        // 1. 获取系统运行的毫秒数，这是所有动画的“时间引擎”
        uint32_t current_tick = osKernelGetTickCount();

        // 2. 渲染带时间参数的动态表情！
        DrawDynamicFace(global_dog_action, current_tick);

        // 3. 右上角依然保留非常实用的电池 UI (去掉了顶层的 Voice 文字)
        uint8_t bat_percent = GetBatteryPercentage();
        oled->DrawBattery(104, 0, bat_percent);
        sprintf(text_buffer, "%d%%", bat_percent);
        oled->DrawString(106, 12, text_buffer);

        // 4. 左下角极客仪表盘 (如果你以后觉得碍眼，把这两行删掉就是一张极其纯粹的脸了)
        sprintf(text_buffer, "P:%d R:%d", (int16_t)dogImu->GetPitch(), (int16_t)dogImu->GetRoll());
        oled->DrawString(0, 54, text_buffer);

        oled->Update();

        // 【关键】：为了让动画顺滑，将屏幕刷新率提速到 50ms 一帧 (相当于 20 FPS)
        // 配合硬件 I2C，这个帧率既不拖累单片机，又能让人眼感觉极其丝滑
        osDelay(50);
    }
}

void task_motion_control(void *argument)
{
    osDelay(2000);
    for (;;)
    {
        VoiceCmd current_cmd = global_dog_action;
        motionBrain->ExecuteCommand(current_cmd);

        if (global_dog_action == current_cmd)
        {
            // 【核心修复】：白名单机制！只有走路、转弯或一次性表演动作，执行完才自动起立。
            // 坐下、趴下、歪头等静态动作被排除在外，它们将被永久保持，直到下一次命令打断！
            if (current_cmd == VoiceCmd::FORWARD ||
                current_cmd == VoiceCmd::BACKWARD ||
                current_cmd == VoiceCmd::TURN_LEFT ||
                current_cmd == VoiceCmd::TURN_RIGHT ||
                current_cmd == VoiceCmd::SHIFT_LEFT ||
                current_cmd == VoiceCmd::SHIFT_RIGHT ||
                current_cmd == VoiceCmd::SHAKE_HAND_L ||
                current_cmd == VoiceCmd::SHAKE_HAND_R ||
                current_cmd == VoiceCmd::GREETING ||
                current_cmd == VoiceCmd::STRETCH ||
                current_cmd == VoiceCmd::DANCE ||
                current_cmd == VoiceCmd::ATTACK_MODE)
            {
                global_dog_action = VoiceCmd::WAKE_UP;
            }
        }

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