#pragma once
#include "pca9685.hpp"
#include <math.h>
#include "su03t.hpp" // 引用 VoiceCmd 定义
#include "cmsis_os2.h"

struct Leg4DOF
{
    LegChanel leg_id;
    float center_angle;
    float phase_offset;
};

class Motion4DOF
{
public:
    Motion4DOF(PCA9685 *_driver);
    void Init();

    // 核心命令执行入口
    void ExecuteCommand(VoiceCmd cmd);
    void StandIdle();
    void Relax();
    void actionWalkForward();

    void actionWalkBackward();

    void actionTurnLeft();

    void actionTurnRight();
    void postureSitDown();
    void postureLieDown();
    void postureLookUp();
    void postureLookDown();
    void postureLeanLeft();
    void postureLeanRight();

    // ==========================================
    // 互动花式动作库 (关键帧动画)
    // ==========================================
    void actionShakeHand(bool left);
    void actionGreeting();
    void actionStretch();
    void actionDance();
    void actionAttackMode();
    void actionWagTail();

private:
    PCA9685 *servoDriver;
    float cur_angles[5];

    // 基础运动参数
    const float amp = 25.0f; // 步幅
    const float mid = 90.0f; // 中位角

    // 物理坐标极性定义 (左侧减小为前进，右侧增大为前进)
    const float LF_F = 90 - 25, LF_B = 90 + 25;
    const float RF_F = 90 + 25, RF_B = 90 - 25;
    const float LH_F = 90 - 25, LH_B = 90 + 25;
    const float RH_F = 90 + 25, RH_B = 90 - 25;

    // 高级矩阵引擎：支持动态极性覆盖
    // void matrixEngine(const int (*gait)[4], int phases, int repeats,
    //                   float lf_s, float lf_e, float rf_s, float rf_e,
    //                   float lh_s, float lh_e, float rh_s, float rh_e);



    // ==========================================
    // 静态姿态库 (IK 基础姿态)
    // ==========================================
    
};