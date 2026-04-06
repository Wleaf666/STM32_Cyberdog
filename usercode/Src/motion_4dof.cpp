#include "motion_4dof.hpp"

// 宏定义圆周率
#define PI 3.1415926535f

// 【新增】：构造函数实现，将传入的 driver 绑定到私有指针
Motion4DOF::Motion4DOF(PCA9685 *_driver) : servoDriver(_driver)
{
}

void Motion4DOF::Init()
{
    current_time = 0.0f;

    // 极度关键：定义小跑(Trot)步态的相位差！
    // 【修改点】：使用 pca9685.hpp 里的 LegChanel 枚举对应四条腿
    // 左前(FrontLeft) 和 右后(RearRight) 必须动作一致，相位同为 0
    // 右前(FrontRight) 和 左后(RearLeft) 必须动作一致，但错开半个周期，所以加 PI (180度)

    legs[0] = {LegChanel::FrontLeft, 90.0f, 0.0f}; // 左前 LF
    legs[1] = {LegChanel::FrontRight, 90.0f, PI};  // 右前 RF
    legs[2] = {LegChanel::RearLeft, 90.0f, PI};    // 左后 LH
    legs[3] = {LegChanel::RearRight, 90.0f, 0.0f}; // 右后 RH
}

void Motion4DOF::StandIdle()
{
    // 让所有舵机回到 90 度中位
    for (int i = 0; i < 4; i++)
    {
        // 传入枚举类型和角度
        servoDriver->setAngle(legs[i].leg_id, legs[i].center_angle);
    }
}

void Motion4DOF::TrotStep(float speed_multiplier, float amplitude, float dir)
{
    // 1. 推进虚拟时间 (0.02秒是 50Hz 刷新率的周期)
    current_time += 0.02f * speed_multiplier * dir;

    // 2. 为 4 条腿生成正弦波角度并驱动
    for (int i = 0; i < 4; i++)
    {
        // 公式：中心点 + 幅度 * sin(时间 + 相位差)
        float target_angle = legs[i].center_angle + amplitude * sin(current_time + legs[i].phase_offset);

        // 调用你的 pca9685 setAngle 函数
        servoDriver->setAngle(legs[i].leg_id, target_angle);
    }
}