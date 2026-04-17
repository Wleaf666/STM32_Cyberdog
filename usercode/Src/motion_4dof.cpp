#include "motion_4dof.hpp"

// 宏定义圆周率
#define PI 3.1415926535f

// 【新增】：构造函数实现，将传入的 driver 绑定到私有指针
Motion4DOF::Motion4DOF(PCA9685 *_driver) : servoDriver(_driver)
{
}

void Motion4DOF::Init()
{
    for (int i = 0; i < 4; i++)
        cur_angles[i] = mid;
    Relax();
}

void Motion4DOF::Relax()
{
    for (int i = 0; i < 4; i++)
        servoDriver->setPWM(i, 0, 4096); // PCA9685 强制关闭通道指令
}

void Motion4DOF::StandIdle()
{
    // 让所有舵机回到 90 度中位
    servoDriver->setAngle(LegChanel::FrontLeft, mid);
    servoDriver->setAngle(LegChanel::FrontRight, mid);
    servoDriver->setAngle(LegChanel::RearLeft, mid);
    servoDriver->setAngle(LegChanel::RearRight, mid);
}

void Motion4DOF::matrixEngine(const int (*gait)[4], int phases, int repeats)
{
    for (int r = 0; r < repeats; r++)
    {
        for (int p = 0; p < phases; p++)
        {
            // 1. 快移 (Swing)
            if (gait[p][0])
                servoDriver->setAngle(LegChanel::FrontLeft, LF_F);
            if (gait[p][1])
                servoDriver->setAngle(LegChanel::FrontRight, RF_F);
            if (gait[p][2])
                servoDriver->setAngle(LegChanel::RearLeft, LH_F);
            if (gait[p][3])
                servoDriver->setAngle(LegChanel::RearRight, RH_F);
            osDelay(120);

            // 2. 慢划 (Stance) - 简化为单次线性补偿以节省 I2C 带宽
            servoDriver->setAngle(LegChanel::FrontLeft, gait[p][0] ? LF_F : LF_B);
            servoDriver->setAngle(LegChanel::FrontRight, gait[p][1] ? RF_F : RF_B);
            servoDriver->setAngle(LegChanel::RearLeft, gait[p][2] ? LH_F : LH_B);
            servoDriver->setAngle(LegChanel::RearRight, gait[p][3] ? RH_F : RH_B);
            osDelay(200);
        }
    }
}

void Motion4DOF::actionShakeHand(bool left)
{
    StandIdle();
    osDelay(200);
    // 抬起指定的前腿
    LegChanel target = left ? LegChanel::FrontLeft : LegChanel::FrontRight;
    float up_angle = left ? 40.0f : 140.0f; // 根据镜像计算抬起角度
    for (int i = 0; i < 3; i++)
    { // 摇晃3次
        servoDriver->setAngle(target, up_angle);
        osDelay(200);
        servoDriver->setAngle(target, mid);
        osDelay(200);
    }
}

void Motion4DOF::ExecuteCommand(VoiceCmd cmd)
{
    static const int creep[4][4] = {{0, 0, 0, 1}, {0, 1, 0, 0}, {0, 0, 1, 0}, {1, 0, 0, 0}};

    switch (cmd)
    {
    case VoiceCmd::FORWARD:
        matrixEngine(creep, 4, 1);
        break;
    case VoiceCmd::SHAKE_HAND_L:
        actionShakeHand(true);
        break;
    case VoiceCmd::SHAKE_HAND_R:
        actionShakeHand(false);
        break;
    case VoiceCmd::LOOK_UP:
        servoDriver->setAngle(LegChanel::FrontLeft, 120);
        servoDriver->setAngle(LegChanel::FrontRight, 60);
        break;
    case VoiceCmd::DANCE:
        actionDance();
        break;
    case VoiceCmd::WAKE_UP:
        StandIdle();
        break;
    case VoiceCmd::SLEEP:
        Relax();
        break;
    default:
        break;
    }
}

void Motion4DOF::TrotStep(float speed_multiplier, float amplitude, float dir)
{
    // 1. 推进虚拟时间 (0.02秒是 50Hz 刷新率的周期)
    current_time += 0.02f * speed_multiplier * dir;

    // 2. 为 4 条腿生成正弦波角度并驱动
    const float phys_dir[4] = {1.0f, -1.0f, 1.0f, -1.0f};

    // 2. 为 4 条腿生成正弦波角度并驱动
    for (int i = 0; i < 4; i++)
    {
        // 公式：中心点 + 方向补偿 * 幅度 * sin(时间 + 相位差)
        float target_angle = legs[i].center_angle + phys_dir[i] * amplitude * sin(current_time + legs[i].phase_offset);

        // 调用 pca9685 setAngle 函数
        servoDriver->setAngle(legs[i].leg_id, target_angle);
    }
}

void Motion4DOF::RelaxIdle()
{
    // 让 4 条腿瞬间脱力变软
    for (int i = 0; i < 4; i++)
    {
        servoDriver->TurnOff(legs[i].leg_id);
    }
}

void Motion4DOF::actionDance()
{
    StandIdle();
    osDelay(200);
    // 左扭右扭跳舞循环 3 次
    for (int i = 0; i < 3; i++)
    {
        servoDriver->setAngle(LegChanel::FrontLeft, 120);
        servoDriver->setAngle(LegChanel::FrontRight, 60);
        osDelay(300);
        servoDriver->setAngle(LegChanel::FrontLeft, 60);
        servoDriver->setAngle(LegChanel::FrontRight, 120);
        osDelay(300);
    }
    StandIdle();
}

// 就算暂时不用，也必须写一个空壳放着，否则会报错！
void Motion4DOF::actionLook(int type)
{
    // 预留位置，后续开发
}

void Motion4DOF::actionStretch()
{
    // 预留位置，后续开发
}