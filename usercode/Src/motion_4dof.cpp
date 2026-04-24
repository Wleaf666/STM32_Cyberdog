#include "motion_4dof.hpp"

// 宏定义圆周率
#define PI 3.1415926535f

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
        servoDriver->setPWM(i, 0, 4096); // 强制脱力关闭 PWM
}

void Motion4DOF::StandIdle()
{
    servoDriver->setAngle(LegChanel::FrontLeft, mid);
    servoDriver->setAngle(LegChanel::FrontRight, mid);
    servoDriver->setAngle(LegChanel::RearLeft, mid);
    servoDriver->setAngle(LegChanel::RearRight, mid);
}

// 终极平滑矩阵引擎
void Motion4DOF::matrixEngine(const int (*gait)[4], int phases, int repeats,
                              float lf_swing, float lf_stance, float rf_swing, float rf_stance,
                              float lh_swing, float lh_stance, float rh_swing, float rh_stance)
{
    float swing_angles[4] = {lf_swing, rf_swing, lh_swing, rh_swing};
    float stance_angles[4] = {lf_stance, rf_stance, lh_stance, rh_stance};
    LegChanel legs[4] = {LegChanel::FrontLeft, LegChanel::FrontRight, LegChanel::RearLeft, LegChanel::RearRight};

    // 计算在抓地阶段，每经过一个相位需要移动的角度增量
    // (总行程 = stance - swing, 分配给 phases-1 个抓地相位)
    float stance_step[4];
    for (int i = 0; i < 4; i++)
    {
        stance_step[i] = (stance_angles[i] - swing_angles[i]) / (phases - 1);
    }

    for (int r = 0; r < repeats; r++)
    {
        for (int p = 0; p < phases; p++)
        {
            // 记录这一阶段骤开始时的角度，作为平滑插值的起点
            float start_angles[4];
            float target_angles[4];

            for (int i = 0; i < 4; i++)
            {
                start_angles[i] = cur_angles[i]; // 读取当前真实的腿部角度

                if (gait[p][i] == 1)
                {
                    // 抬腿迈步状态：直接把目标定在最前方
                    target_angles[i] = swing_angles[i];
                }
                else
                {
                    // 抓地推移状态：在当前角度的基础上，往后推移一个步长增量
                    target_angles[i] = cur_angles[i] + stance_step[i];
                }
            }

            // ==========================================
            // 1. 抬腿迈步 (打滑阶段：极速前伸)
            // ==========================================
            for (int i = 0; i < 4; i++)
            {
                if (gait[p][i] == 1)
                {
                    servoDriver->setAngle(legs[i], target_angles[i]);
                    cur_angles[i] = target_angles[i]; // 更新记录
                }
            }
            osDelay(30); // 稍微缩短前伸等待时间，让动作更利落

            // ==========================================
            // 2. 慢推移动作 (抓地阶段：缓慢往后推移)
            // ==========================================
            int smooth_frames = 10;
            for (int j = 1; j <= smooth_frames; j++)
            {
                float ratio = (float)j / smooth_frames;
                for (int i = 0; i < 4; i++)
                {
                    if (gait[p][i] == 0)
                    {
                        // 线性插值：从当前起点，平滑过渡到这一个相位的目标点
                        float interpolated = start_angles[i] + (target_angles[i] - start_angles[i]) * ratio;
                        servoDriver->setAngle(legs[i], interpolated);

                        // 在最后一帧，把当前角度正式记录下来，留给下一个相位使用
                        if (j == smooth_frames)
                        {
                            cur_angles[i] = target_angles[i];
                        }
                    }
                }
                osDelay(15);
            }
        }
    }
}

// ==========================================
// 核心中枢：指令分发器
// ==========================================
void Motion4DOF::ExecuteCommand(VoiceCmd cmd)
{
    // 极其稳定的经典对角爬行步态矩阵 (Creep Gait)
    static const int creep[4][4] = {
        {0, 0, 0, 1},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {1, 0, 0, 0}};

    switch (cmd)
    {
    // --- 1. 系统状态 ---
    case VoiceCmd::WAKE_UP:
    case VoiceCmd::STAND_UP:
    case VoiceCmd::STOP_MOVE:
        StandIdle();
        break;
    case VoiceCmd::SLEEP:
        Relax();
        break;

    // --- 2. 运动指令 ---
    case VoiceCmd::FORWARD:
        // 前进：正常极性
        matrixEngine(creep, 4, 3, LF_F, LF_B, RF_F, RF_B, LH_F, LH_B, RH_F, RH_B);
        break;
    case VoiceCmd::BACKWARD:
        // 后退：Swing 和 Stance 极性对调
        matrixEngine(creep, 4, 3, LF_B, LF_F, RF_B, RF_F, LH_B, LH_F, RH_B, RH_F);
        break;
    case VoiceCmd::TURN_LEFT:
        // 左转：左边后退极性，右边前进极性
        matrixEngine(creep, 4, 2, LF_B, LF_F, RF_F, RF_B, LH_B, LH_F, RH_F, RH_B);
        break;
    case VoiceCmd::TURN_RIGHT:
        // 右转：左边前进极性，右边后退极性
        matrixEngine(creep, 4, 2, LF_F, LF_B, RF_B, RF_F, LH_F, LH_B, RH_B, RH_F);
        break;
    case VoiceCmd::SHIFT_LEFT:
        // 4自由度无机械侧移能力，用“原地左右扭屁股”代替
        postureLeanLeft();
        osDelay(200);
        StandIdle();
        break;
    case VoiceCmd::SHIFT_RIGHT:
        postureLeanRight();
        osDelay(200);
        StandIdle();
        break;

    // --- 3. 静态姿态 ---
    case VoiceCmd::SIT_DOWN:
        postureSitDown();
        break;
    case VoiceCmd::LIE_DOWN:
        postureLieDown();
        break;
    case VoiceCmd::LOOK_UP:
        postureLookUp();
        break;
    case VoiceCmd::LOOK_DOWN:
        postureLookDown();
        break;
    case VoiceCmd::LEAN_LEFT:
        postureLeanLeft();
        break;
    case VoiceCmd::LEAN_RIGHT:
        postureLeanRight();
        break;

    // --- 4. 花式动作 ---
    case VoiceCmd::SHAKE_HAND_L:
        actionShakeHand(true);
        break;
    case VoiceCmd::SHAKE_HAND_R:
        actionShakeHand(false);
        break;
    case VoiceCmd::GREETING:
        actionGreeting();
        break;
    case VoiceCmd::STRETCH:
        actionStretch();
        break;
    case VoiceCmd::DANCE:
        actionDance();
        break;
    case VoiceCmd::ATTACK_MODE:
        actionAttackMode();
        break;

    default:
        break;
    }
}

// ==========================================
// 姿态与动作底层实现 (严格遵守左右镜像法则)
// ==========================================

void Motion4DOF::postureSitDown()
{
    servoDriver->setAngle(LegChanel::FrontLeft, mid);
    servoDriver->setAngle(LegChanel::FrontRight, mid);
    // 后腿往后折叠蹲下 (左侧增大为后退，右侧减小为后退)
    servoDriver->setAngle(LegChanel::RearLeft, mid + 40);
    servoDriver->setAngle(LegChanel::RearRight, mid - 40);
}

void Motion4DOF::postureLieDown()
{
    // 四腿趴平 (前腿前伸，后腿后伸)
    servoDriver->setAngle(LegChanel::FrontLeft, mid - 50);
    servoDriver->setAngle(LegChanel::FrontRight, mid + 50);
    servoDriver->setAngle(LegChanel::RearLeft, mid + 50);
    servoDriver->setAngle(LegChanel::RearRight, mid - 50);
}

void Motion4DOF::postureLookUp()
{
    // 前腿往后收起抬高，后腿往外撇降低
    servoDriver->setAngle(LegChanel::FrontLeft, mid + 30);
    servoDriver->setAngle(LegChanel::FrontRight, mid - 30);
    servoDriver->setAngle(LegChanel::RearLeft, mid + 30);
    servoDriver->setAngle(LegChanel::RearRight, mid - 30);
}

void Motion4DOF::postureLookDown()
{
    // 前腿往外撇降低，后腿往后收起抬高
    servoDriver->setAngle(LegChanel::FrontLeft, mid - 30);
    servoDriver->setAngle(LegChanel::FrontRight, mid + 30);
    servoDriver->setAngle(LegChanel::RearLeft, mid - 30);
    servoDriver->setAngle(LegChanel::RearRight, mid + 30);
}

void Motion4DOF::postureLeanLeft()
{
    // 左侧腿趴下，右侧腿站直
    servoDriver->setAngle(LegChanel::FrontLeft, mid - 40);
    servoDriver->setAngle(LegChanel::RearLeft, mid + 40);
    servoDriver->setAngle(LegChanel::FrontRight, mid);
    servoDriver->setAngle(LegChanel::RearRight, mid);
}

void Motion4DOF::postureLeanRight()
{
    // 右侧腿趴下，左侧腿站直
    servoDriver->setAngle(LegChanel::FrontLeft, mid);
    servoDriver->setAngle(LegChanel::RearLeft, mid);
    servoDriver->setAngle(LegChanel::FrontRight, mid + 40);
    servoDriver->setAngle(LegChanel::RearRight, mid - 40);
}

void Motion4DOF::actionShakeHand(bool left)
{
    postureSitDown(); // 先坐下保证底盘稳定
    osDelay(300);
    LegChanel target = left ? LegChanel::FrontLeft : LegChanel::FrontRight;
    float up_angle = left ? (mid - 60) : (mid + 60); // 往前上方抬起

    for (int i = 0; i < 3; i++)
    {
        servoDriver->setAngle(target, up_angle);
        osDelay(150);
        servoDriver->setAngle(target, up_angle + (left ? 25 : -25)); // 上下抖动
        osDelay(150);
    }
}

void Motion4DOF::actionGreeting()
{
    // 先抬头，然后抬起右手招手
    postureLookUp();
    osDelay(200);
    actionShakeHand(false);
}

void Motion4DOF::actionStretch()
{
    // 类似真狗伸懒腰：前腿极度前伸趴下，后腿站直
    servoDriver->setAngle(LegChanel::FrontLeft, mid - 60);
    servoDriver->setAngle(LegChanel::FrontRight, mid + 60);
    servoDriver->setAngle(LegChanel::RearLeft, mid);
    servoDriver->setAngle(LegChanel::RearRight, mid);
    osDelay(1200); // 舒服地拉伸一秒钟
}

void Motion4DOF::actionDance()
{
    // 左右摇摆舞
    for (int i = 0; i < 2; i++)
    {
        postureLeanLeft();
        osDelay(300);
        postureLeanRight();
        osDelay(300);
    }
    // 抬头低头蹦迪
    for (int i = 0; i < 2; i++)
    {
        postureLookUp();
        osDelay(200);
        postureLookDown();
        osDelay(200);
    }
}

void Motion4DOF::actionAttackMode()
{
    // 压低前身，后腿随时准备发力蹬出
    servoDriver->setAngle(LegChanel::FrontLeft, mid - 30);
    servoDriver->setAngle(LegChanel::FrontRight, mid + 30);
    servoDriver->setAngle(LegChanel::RearLeft, mid - 15);
    servoDriver->setAngle(LegChanel::RearRight, mid + 15);
    osDelay(1000);
}