#pragma once

#include "main.h"
#include "cmsis_os2.h"
#include <vector>

enum class VoiceCmd : uint8_t
{
    NONE = 0x00,

    // ==========================================
    // 1. 系统与状态控制 (0x01 ~ 0x0F)
    // ==========================================
    WAKE_UP = 0x01,    // 唤醒词 (例如：“小狗小狗”、“机器狗”)
    SLEEP = 0x02,      // 休眠词 (例如：“退下吧”、“进入休眠”，断开舵机扭矩省电)
    REPORT_BAT = 0x03, // 状态查询 (例如：“汇报电量”，触发 OELD 刷新或语音播报)

    // ==========================================
    // 2. 基础移动指令 (0x10 ~ 0x1F) - 对应步态解算器 (Trot/Walk)
    // ==========================================
    FORWARD = 0x10,     // “前进”、“往前走”
    BACKWARD = 0x11,    // “后退”、“往后退”
    TURN_LEFT = 0x12,   // “左转”、“往左转”
    TURN_RIGHT = 0x13,  // “右转”、“往右转”
    SHIFT_LEFT = 0x14,  // “左平移”、“向左挪” (横向蟹行)
    SHIFT_RIGHT = 0x15, // “右平移”、“向右挪”
    STOP_MOVE = 0x16,   // “停”、“停下来” (急停，步态速度清零)

    // ==========================================
    // 3. 静态姿态控制 (0x20 ~ 0x2F) - 对应逆运动学 (IK) 姿态角
    // ==========================================
    STAND_UP = 0x20,   // “站起来”、“起立” (恢复默认站姿)
    SIT_DOWN = 0x21,   // “坐下” (后腿蹲，前腿直)
    LIE_DOWN = 0x22,   // “趴下” (四条腿全部折叠收缩)
    LOOK_UP = 0x23,    // “抬头”、“往上看” (Pitch 角抬高)
    LOOK_DOWN = 0x24,  // “低头” (Pitch 角降低)
    LEAN_LEFT = 0x25,  // “左倾斜” (Roll 角改变)
    LEAN_RIGHT = 0x26, // “右倾斜”

    // ==========================================
    // 4. 互动与花式动作 (0x30 ~ 0x3F) - 对应预设的关节关键帧动画
    // ==========================================
    SHAKE_HAND_L = 0x30, // “左手”、“握个手” (抬起左前腿)
    SHAKE_HAND_R = 0x31, // “右手” (抬起右前腿)
    GREETING = 0x32,     // “打招呼”、“招招手” (抬起单腿并来回摆动)
    STRETCH = 0x33,      // “伸个懒腰” (前腿前伸，后腿站立，类似猫/狗伸懒腰)
    DANCE = 0x34,        // “跳个舞” (循环执行一套预设动作)
    ATTACK_MODE = 0x35   // “战斗模式” (压低重心，准备扑咬姿态)
};

enum class VoicePlay : uint8_t
{
    BEEP = 0x10,      // 播报提示音 (如“滴”)
    BAT_LOW = 0x11,   // 播报“电量不足”
    CMD_FAILED = 0x12 // 播报“指令错误”
};

class SU03T
{
private:
    UART_HandleTypeDef *huart;
    osMessageQueueId_t rxQueue;
    osMutexId_t txMutex; // 说话时需要占用串口，加上互斥锁更安全

    std::vector<uint8_t> rxBuffer;
    uint8_t rxByte;

    // 内部方法：解析包头包尾
    VoiceCmd parsePacket(const std::vector<uint8_t> &packet);

public:
    // 构造函数，需要传入你分配给语音模块的串口句柄和发送锁
    SU03T(UART_HandleTypeDef *_huart, osMutexId_t _txMutex);

    // 初始化，传入 RTOS 消息队列
    bool Init(osMessageQueueId_t _rxQueue);

    // 串口接收中断回调函数
    void onRxCpltCallback();

    // 供应用层调用的接口：尝试获取语音指令（非阻塞）
    VoiceCmd getCommand();

    // 供应用层调用的接口：让语音模块播报指定音频
    void playVoice(VoicePlay audio_id);
};

// C 语言中断的回调包装器
void SU03T_RxCallback_Wrapper(UART_HandleTypeDef *huart);