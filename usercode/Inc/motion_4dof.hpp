#pragma once
#include "pca9685.hpp"
#include <math.h>

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

    float current_time; // 步态系统的内部虚拟时间戳

public:
    // 构造函数
    Motion4DOF(PCA9685 *_driver);

    void Init();

    // 控制接口：让机器狗站直待命
    void StandIdle();

    // 步态引擎心跳：放在 RTOS 中以 50Hz 循环调用
    // speed_multiplier: 频率(快慢)
    // amplitude: 迈步幅度(度)
    // dir: 1 为前进，-1 为后退
    void TrotStep(float speed_multiplier, float amplitude, float dir);
};