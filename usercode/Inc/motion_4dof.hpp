#pragma once
#include "pca9685.hpp"
#include <math.h>
#include "su03t.hpp" // 引用 VoiceCmd 定义
#include "cmsis_os2.h"

struct Leg4DOF
{
    LegChanel leg_id;   // 【修改点】：直接使用 PCA9685 库中定义的枚举，而不是 uint8_t
    float center_angle; // 舵机的机械中位角 (通常是 90 度)
    float phase_offset; // 这条腿的正弦波相位偏移 (弧度)
};

class Motion4DOF
{
private:
    PCA9685 *servoDriver;
    Leg4DOF legs[4]; // 顺序：左前(LF), 右前(RF), 左后(LH), 右后(RH)
    float cur_angles[4]; // 记录四条腿当前角度：LF, RF, LH, RH

    // 步态参数
    const float amp = 25.0f;
    const float mid = 90.0f;

    // 物理坐标极性定义 (左侧 -，右侧 + 为前进)
    const float LF_F = 90 - 25, LF_B = 90 + 25;
    const float RF_F = 90 + 25, RF_B = 90 - 25;
    const float LH_F = 90 - 25, LH_B = 90 + 25;
    const float RH_F = 90 + 25, RH_B = 90 - 25;

    // 通用矩阵引擎：参数为步态矩阵、相位数、循环次数
    void matrixEngine(const int (*gait)[4], int phases, int repeats);

    // --- 专项动作私有实现 ---
    void actionShakeHand(bool left); // 握手/招手
    void actionLook(int type);       // 抬头/低头/倾斜
    void actionDance();              // 舞蹈序列
    void actionStretch();            // 伸懒腰

public:
    float current_time; // 步态系统的内部虚拟时间戳
    // 构造函数
    Motion4DOF(PCA9685 *_driver);

    void Init();

    void ExecuteCommand(VoiceCmd cmd); // 统一动作入口
    void StandIdle();                  // 90度站立
    void Relax();                      // 肌肉脱力保护



    // 步态引擎心跳：放在 RTOS 中以 50Hz 循环调用
    // speed_multiplier: 频率(快慢)
    // amplitude: 迈步幅度(度)
    // dir: 1 为前进，-1 为后退
    void TrotStep(float speed_multiplier, float amplitude, float dir);

    void RelaxIdle();
};