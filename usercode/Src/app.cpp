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
#include "motion_4dof.hpp" 
#include "adc.h"           

volatile VoiceCmd global_dog_action = VoiceCmd::WAKE_UP;
char last_voice_cmd[32] = "Voice: None";

osMessageQueueId_t my_hc05_queue=nullptr;
osMessageQueueId_t my_su03t_queue = nullptr;


osMutexId_t i2c1_mutex = nullptr;
osMutexId_t uart_mutex = nullptr;
osMutexId_t uart2_mutex = nullptr; 

MPU6050 *dogImu = nullptr;
HC05 *blueTooth = nullptr;
SSD1306 *oled = nullptr;
SU03T *voiceModule = nullptr;
PCA9685 *pca9685 = nullptr;
Motion4DOF *motionBrain = nullptr; 

uint8_t MapCmdToHex(VoiceCmd cmd)
{
    switch (cmd)
    {
    case VoiceCmd::WAKE_UP:
        return 0x01;
    case VoiceCmd::SLEEP:
        return 0x02;
    case VoiceCmd::REPORT_BAT:
        return 0x03;
    case VoiceCmd::FORWARD:
        return 0x10;
    case VoiceCmd::BACKWARD:
        return 0x11;
    case VoiceCmd::TURN_LEFT:
        return 0x12;
    case VoiceCmd::TURN_RIGHT:
        return 0x13;
    case VoiceCmd::SHIFT_LEFT:
        return 0x14;
    case VoiceCmd::SHIFT_RIGHT:
        return 0x15;
    case VoiceCmd::STOP_MOVE:
        return 0x16;
    case VoiceCmd::STAND_UP:
        return 0x20;
    case VoiceCmd::SIT_DOWN:
        return 0x21;
    case VoiceCmd::LIE_DOWN:
        return 0x22;
    case VoiceCmd::LOOK_UP:
        return 0x23;
    case VoiceCmd::LOOK_DOWN:
        return 0x24;
    case VoiceCmd::LEAN_LEFT:
        return 0x25;
    case VoiceCmd::LEAN_RIGHT:
        return 0x26;
    case VoiceCmd::SHAKE_HAND_L:
        return 0x30;
    case VoiceCmd::SHAKE_HAND_R:
        return 0x31;
    case VoiceCmd::GREETING:
        return 0x32;
    case VoiceCmd::STRETCH:
        return 0x33;
    case VoiceCmd::DANCE:
        return 0x34;
    case VoiceCmd::ATTACK_MODE:
        return 0x35;
    default:
        return 0xFF; // 未知动作不发送
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 1. 规范清除错误标志位 (绝对不要在这里强行 Unlock！)
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE) != RESET)
    {
        __HAL_UART_CLEAR_NEFLAG(huart);
    }
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE) != RESET)
    {
        __HAL_UART_CLEAR_FEFLAG(huart);
    }

    // 2. 优雅通知上层重启接收
    if (huart->Instance == USART1 && blueTooth != nullptr)
    {
        blueTooth->Resume();
    }
    if (huart->Instance == USART2 && voiceModule != nullptr)
    {
        voiceModule->Resume();
    }
}

uint8_t GetBatteryPercentage()
{
    // 用 uint32_t 代替 float，单位全部转换为 毫伏 (mV)
    static uint32_t filtered_mv = 0;
    static bool is_first_read = true;

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        uint32_t raw_adc = HAL_ADC_GetValue(&hadc1);

        // 【优化】：用整数乘除代替浮点运算
        // 原始浮点计算: (raw_adc / 4096.0) * 3.3 * 6.0 = raw_adc * 19.8 / 4096
        // 整数毫伏计算: raw_adc * 19800 / 4096
        uint32_t current_mv = (raw_adc * 19800) / 4096;

        // 整数版低通滤波 (系数 0.95 和 0.05 放大 100 倍计算)
        if (is_first_read)
        {
            filtered_mv = current_mv;
            is_first_read = false;
        }
        else
        {
            // 公式：(历史值 * 95 + 当前值 * 5) / 100
            filtered_mv = (filtered_mv * 95 + current_mv * 5) / 100;
        }

        // 计算电量百分比
        // 满电 8.4V = 8400mV，没电 6.4V = 6400mV
        // 原始浮点: (V - 6.4) / 2.0 * 100
        // 整数化简: (mV - 6400) * 100 / (8400 - 6400) = (mV - 6400) / 20
        int percent = (int)(filtered_mv - 6400) / 20;

        if (percent > 100)
            percent = 100;
        if (percent < 0)
            percent = 0;

        return (uint8_t)percent;
    }
    return 0;
}

// ==========================================
// 2. Cozmo 风格矢量动态表情渲染引擎
// ==========================================
void DrawDynamicFace(VoiceCmd cmd, uint32_t tick)
{
    // 计算全局平滑悬浮偏移量 (利用正弦波产生呼吸感，周期约1.2秒，上下浮动3像素)
    int phase = tick % 1200;
    int y_breathe = (phase < 600) ? (phase / 100 - 3) : ((1200 - phase) / 100 - 3);

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
            // 【语音接轨】：收到手机指令时，让语音模块“滴”一声作为确认反馈（需确保SU03T固件支持该串口指令）
            voiceModule->playVoice(VoicePlay::BEEP);

            // 【全动作精确映射】将手机 Hex 控制码完全匹配到系统的 VoiceCmd 枚举
            switch (cmd.cmd_type)
            {
            case 0x01:
                global_dog_action = VoiceCmd::WAKE_UP;
                blueTooth->sendString("ACK: WAKE_UP\r\n");
                break;
            case 0x02:
                global_dog_action = VoiceCmd::SLEEP;
                blueTooth->sendString("ACK: SLEEP\r\n");
                break;
            case 0x03:
                global_dog_action = VoiceCmd::REPORT_BAT;
                char bat_msg[32];
                sprintf(bat_msg, "Battery: %d%%\r\n", GetBatteryPercentage());
                blueTooth->sendString(bat_msg);
                break;

            case 0x10:
                global_dog_action = VoiceCmd::FORWARD;
                blueTooth->sendString("ACK: FORWARD\r\n");
                break;
            case 0x11:
                global_dog_action = VoiceCmd::BACKWARD;
                blueTooth->sendString("ACK: BACKWARD\r\n");
                break;
            case 0x12:
                global_dog_action = VoiceCmd::TURN_LEFT;
                blueTooth->sendString("ACK: TURN_L\r\n");
                break;
            case 0x13:
                global_dog_action = VoiceCmd::TURN_RIGHT;
                blueTooth->sendString("ACK: TURN_R\r\n");
                break;
            case 0x14:
                global_dog_action = VoiceCmd::SHIFT_LEFT;
                blueTooth->sendString("ACK: SHIFT_L\r\n");
                break;
            case 0x15:
                global_dog_action = VoiceCmd::SHIFT_RIGHT;
                blueTooth->sendString("ACK: SHIFT_R\r\n");
                break;
            case 0x16:
                global_dog_action = VoiceCmd::STOP_MOVE;
                blueTooth->sendString("ACK: STOP\r\n");
                break;

            case 0x20:
                global_dog_action = VoiceCmd::STAND_UP;
                blueTooth->sendString("ACK: STAND\r\n");
                break;
            case 0x21:
                global_dog_action = VoiceCmd::SIT_DOWN;
                blueTooth->sendString("ACK: SIT\r\n");
                break;
            case 0x22:
                global_dog_action = VoiceCmd::LIE_DOWN;
                blueTooth->sendString("ACK: LIE\r\n");
                break;
            case 0x23:
                global_dog_action = VoiceCmd::LOOK_UP;
                blueTooth->sendString("ACK: LOOK_UP\r\n");
                break;
            case 0x24:
                global_dog_action = VoiceCmd::LOOK_DOWN;
                blueTooth->sendString("ACK: LOOK_DN\r\n");
                break;
            case 0x25:
                global_dog_action = VoiceCmd::LEAN_LEFT;
                blueTooth->sendString("ACK: LEAN_L\r\n");
                break;
            case 0x26:
                global_dog_action = VoiceCmd::LEAN_RIGHT;
                blueTooth->sendString("ACK: LEAN_R\r\n");
                break;

            case 0x30:
                global_dog_action = VoiceCmd::SHAKE_HAND_L;
                blueTooth->sendString("ACK: HAND_L\r\n");
                break;
            case 0x31:
                global_dog_action = VoiceCmd::SHAKE_HAND_R;
                blueTooth->sendString("ACK: HAND_R\r\n");
                break;
            case 0x32:
                global_dog_action = VoiceCmd::GREETING;
                blueTooth->sendString("ACK: GREET\r\n");
                break;
            case 0x33:
                global_dog_action = VoiceCmd::STRETCH;
                blueTooth->sendString("ACK: STRETCH\r\n");
                break;
            case 0x34:
                global_dog_action = VoiceCmd::DANCE;
                blueTooth->sendString("ACK: DANCE\r\n");
                break;
            case 0x35:
                global_dog_action = VoiceCmd::ATTACK_MODE;
                blueTooth->sendString("ACK: ATTACK\r\n");
                break;

            case 0x00:
                global_dog_action = VoiceCmd::SIT_DOWN;
                blueTooth->sendString("ACK: STOP(LEGACY)\r\n");
                break;
            default:
                blueTooth->sendString("ACK: UNKNOWN CMD\r\n");
                break;
            }
        }
        // 关键：不阻塞其它系统进程
        osDelay(10);
    }
}

void task_display(void *argument)
{
    char text_buffer[16];
    for (;;)
    {
        oled->Clear();

        // 获取系统运行毫秒数，驱动表情动画
        uint32_t current_tick = osKernelGetTickCount();

        // 渲染动态表情
        DrawDynamicFace(global_dog_action, current_tick);

        // ==========================================
        // 【核心修改】：左上角平滑电量显示
        // ==========================================
        uint8_t bat_percent = GetBatteryPercentage();

        // 坐标修改为 X=0, Y=0 (屏幕极左上角)
        oled->DrawBattery(0, 0, bat_percent);

        // 坐标修改为 X=2, Y=12 (在电池图标正下方居中显示文字)
        sprintf(text_buffer, "%d%%", bat_percent);
        oled->DrawString(2, 12, text_buffer);

        oled->Update();
        osDelay(50); // 维持 20FPS 的丝滑刷新率
    }
}

void task_motion_control(void *argument)
{
    // 等待系统外设完全起飞
    osDelay(2000);

    // 【核心防堵塞变量】：记录上一次执行的动作，确保只在“改变”时发一次串口
    VoiceCmd last_executed_cmd = VoiceCmd::NONE;

    for (;;)
    {
        // 锁定当前周期的指令
        VoiceCmd current_cmd = global_dog_action;

        // ===============================================
        // 【关键新增】：动作发生切换瞬间，向 SU03T 发送反馈包
        // ===============================================
        if (current_cmd != last_executed_cmd)
        {
            uint8_t hex_code = MapCmdToHex(current_cmd);
            if (hex_code != 0xFF)
            {
                // 发送形如 AA 10 55 的数据包到语音模块
                voiceModule->sendPacket(hex_code);
            }
            last_executed_cmd = current_cmd; // 更新记录
        }

        // 驱动底层舵机矩阵计算
        motionBrain->ExecuteCommand(current_cmd);

        // 如果执行完整个动作周期后，外部没有发来新指令：
        if (global_dog_action == current_cmd)
        {
            // 对“动态位移”和“一次性表演”进行状态复位。静态姿态绝不复位。
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
                // 动作做完后自动切回待机状态
                // （注意：切回待机时，下一个循环检测到状态变为了 WAKE_UP，
                // 也会向语音模块发送一次 AA 01 55 的包，表示动作结束/狗已站稳）
                global_dog_action = VoiceCmd::WAKE_UP;
            }
        }

        // 留出 CPU 喘息时间
        osDelay(20);
    }
}

void task_tail_control(void *argument)
{
    osDelay(2000); // 等待系统初始化完成
    for (;;)
    {
        // 如果狗处于睡觉状态，尾巴就停止摇摆，自然下垂
        if (global_dog_action == VoiceCmd::SLEEP)
        {
            pca9685->setAngle(LegChanel::Tail, 90.0f); // 假设 90 度是正中
            osDelay(500);
        }
        else
        {
            uint32_t tick = osKernelGetTickCount();

      
            int phase = tick % 1000;
            float tail_offset = (phase < 500) ? (phase * 70.0f / 500.0f - 35.0f) : ((1000 - phase) * 70.0f / 500.0f - 35.0f);

            pca9685->setAngle(LegChanel::Tail, 90.0f + tail_offset);

            // 20ms 一帧 (50FPS)，保证尾巴摇起来像真狗一样顺滑
            osDelay(50);
        }
    }
}

void task_system_init(void *argument)
{
    // 【极其关键】让子弹飞一会儿！给所有外设 100ms 的硬件上电稳定时间
    osDelay(100);

    // 此时调度器已经完美运行，TIM4 中断正常，Mutex 安全，超时机制满血复活！
    oled->Init();
    dogImu->Init();
    blueTooth->Init(my_hc05_queue);
    voiceModule->Init(my_su03t_queue);
    pca9685->Init();
    motionBrain->Init();

    // 初始化完成后，再点火启动所有干活的线程
    osThreadAttr_t default_attr = {.priority = osPriorityNormal};
    osThreadAttr_t display_attr = {.priority = osPriorityNormal};
    display_attr.stack_size = 256 * 4;

    // 3. 给需要做浮点 sin() 运算的尾巴和运动任务更大的栈
    osThreadAttr_t math_attr = {.priority = osPriorityNormal};
    math_attr.stack_size = 256 * 4;
    osThreadNew(task_bluetooth_test, NULL, &default_attr);
    osThreadNew(task_display, NULL, &display_attr);
    osThreadNew(task_Mpu6050, NULL, &default_attr);
    osThreadNew(task_voice_handler, NULL, &default_attr);
    osThreadNew(task_motion_control, NULL, &math_attr);
    osThreadNew(task_tail_control, NULL, &math_attr);

    // 初始化任务完成使命，自杀释放内存
    osThreadExit();
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
    motionBrain = new Motion4DOF(pca9685); 

    my_hc05_queue = osMessageQueueNew(64, sizeof(uint8_t), NULL);
    my_su03t_queue = osMessageQueueNew(32, sizeof(uint8_t), NULL);

    osThreadAttr_t init_attr = {.priority = osPriorityHigh};
    osThreadNew(task_system_init, NULL, &init_attr);
}