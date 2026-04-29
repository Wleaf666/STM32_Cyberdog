#include "motion_4dof.hpp"
extern volatile VoiceCmd global_dog_action;

Motion4DOF::Motion4DOF(PCA9685 *_driver) : servoDriver(_driver)
{
}

void Motion4DOF::Init()
{
    for (int i = 0; i < 4; i++)
        cur_angles[i] = mid;
    StandIdle();
}

void Motion4DOF::Relax()
{
    for (int i = 0; i < 4; i++)
        servoDriver->setPWM(i, 0, 4096); // 强制脱力关闭 PWM
}

//
// 私有辅助：批量设置 4 个舵机角度并记录，若角度未变则跳过 I2C 写入
//
void Motion4DOF::applyPosture(float fl, float fr, float rl, float rr)
{
    float targets[4] = {fl, fr, rl, rr};
    LegChanel legs[4] = {LegChanel::FrontLeft, LegChanel::FrontRight,
                         LegChanel::RearLeft, LegChanel::RearRight};

    for (int i = 0; i < 4; i++)
    {
        if (cur_angles[i] != targets[i])
        {
            servoDriver->setAngle(legs[i], targets[i]);
            cur_angles[i] = targets[i];
        }
    }
}

void Motion4DOF::actionWalkForward()
{
    int speed = 250;
    for (int i = 0; i < 2; i++)
    {
        if (global_dog_action != VoiceCmd::FORWARD)
            return;

        // 【Phase 1】：左侧向前伸展，右侧直立支撑
        servoDriver->setAngle(LegChanel::FrontLeft, mid - 40);
        servoDriver->setAngle(LegChanel::RearLeft, mid + 40);
        servoDriver->setAngle(LegChanel::FrontRight, mid);
        servoDriver->setAngle(LegChanel::RearRight, mid);
        cur_angles[0] = mid - 40;
        cur_angles[1] = mid;
        cur_angles[2] = mid + 40;
        cur_angles[3] = mid;
        osDelay(speed);

        if (global_dog_action != VoiceCmd::FORWARD)
            return;

        // 【Phase 2】：左侧恢复直立(产生前进推力)，右侧向前伸展
        servoDriver->setAngle(LegChanel::FrontLeft, mid);
        servoDriver->setAngle(LegChanel::RearLeft, mid);
        servoDriver->setAngle(LegChanel::FrontRight, mid + 40);
        servoDriver->setAngle(LegChanel::RearRight, mid - 40);
        cur_angles[0] = mid;
        cur_angles[1] = mid + 40;
        cur_angles[2] = mid;
        cur_angles[3] = mid - 40;
        osDelay(speed);
    }
}

void Motion4DOF::actionWalkBackward()
{
    int speed = 250;
    for (int i = 0; i < 2; i++)
    {
        if (global_dog_action != VoiceCmd::BACKWARD)
            return;

        // 【Phase 1】：左侧向后伸展，右侧直立支撑
        servoDriver->setAngle(LegChanel::FrontLeft, mid + 40);
        servoDriver->setAngle(LegChanel::RearLeft, mid - 40);
        servoDriver->setAngle(LegChanel::FrontRight, mid);
        servoDriver->setAngle(LegChanel::RearRight, mid);
        cur_angles[0] = mid + 40;
        cur_angles[1] = mid;
        cur_angles[2] = mid - 40;
        cur_angles[3] = mid;
        osDelay(speed);

        if (global_dog_action != VoiceCmd::BACKWARD)
            return;

        // 【Phase 2】：左侧恢复直立(产生后退推力)，右侧向后伸展
        servoDriver->setAngle(LegChanel::FrontLeft, mid);
        servoDriver->setAngle(LegChanel::RearLeft, mid);
        servoDriver->setAngle(LegChanel::FrontRight, mid - 40);
        servoDriver->setAngle(LegChanel::RearRight, mid + 40);
        cur_angles[0] = mid;
        cur_angles[1] = mid - 40;
        cur_angles[2] = mid;
        cur_angles[3] = mid + 40;
        osDelay(speed);
    }
}

void Motion4DOF::actionTurnLeft()
{
    int speed = 250;
    for (int i = 0; i < 2; i++)
    {
        if (global_dog_action != VoiceCmd::TURN_LEFT)
            return;

        // 【原地左转 Phase 1】：左侧向后伸展 (准备向后划水)
        servoDriver->setAngle(LegChanel::FrontLeft, mid + 40);
        servoDriver->setAngle(LegChanel::RearLeft, mid - 40);
        servoDriver->setAngle(LegChanel::FrontRight, mid);
        servoDriver->setAngle(LegChanel::RearRight, mid);
        cur_angles[0] = mid + 40;
        cur_angles[1] = mid;
        cur_angles[2] = mid - 40;
        cur_angles[3] = mid;
        osDelay(speed);

        if (global_dog_action != VoiceCmd::TURN_LEFT)
            return;

        // 【原地左转 Phase 2】：左侧收回产生倒车力，右侧向前伸展 (准备向前划水)
        servoDriver->setAngle(LegChanel::FrontLeft, mid);
        servoDriver->setAngle(LegChanel::RearLeft, mid);
        servoDriver->setAngle(LegChanel::FrontRight, mid + 40);
        servoDriver->setAngle(LegChanel::RearRight, mid - 40);
        cur_angles[0] = mid;
        cur_angles[1] = mid + 40;
        cur_angles[2] = mid;
        cur_angles[3] = mid - 40;
        osDelay(speed);
    }
}

void Motion4DOF::actionTurnRight()
{
    int speed = 250;
    for (int i = 0; i < 2; i++)
    {
        if (global_dog_action != VoiceCmd::TURN_RIGHT)
            return;

        // 【原地右转 Phase 1】：左侧向前伸展 (准备向前划水)
        servoDriver->setAngle(LegChanel::FrontLeft, mid - 40);
        servoDriver->setAngle(LegChanel::RearLeft, mid + 40);
        servoDriver->setAngle(LegChanel::FrontRight, mid);
        servoDriver->setAngle(LegChanel::RearRight, mid);
        cur_angles[0] = mid - 40;
        cur_angles[1] = mid;
        cur_angles[2] = mid + 40;
        cur_angles[3] = mid;
        osDelay(speed);

        if (global_dog_action != VoiceCmd::TURN_RIGHT)
            return;

        // 【原地右转 Phase 2】：左侧收回产生前进力，右侧向后伸展 (准备向后划水)
        servoDriver->setAngle(LegChanel::FrontLeft, mid);
        servoDriver->setAngle(LegChanel::RearLeft, mid);
        servoDriver->setAngle(LegChanel::FrontRight, mid - 40);
        servoDriver->setAngle(LegChanel::RearRight, mid + 40);
        cur_angles[0] = mid;
        cur_angles[1] = mid - 40;
        cur_angles[2] = mid;
        cur_angles[3] = mid + 40;
        osDelay(speed);
    }
}

// ==========================================
// 核心中枢：指令分发器
// ==========================================
void Motion4DOF::ExecuteCommand(VoiceCmd cmd)
{
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
        actionWalkForward();
        break;
    case VoiceCmd::BACKWARD:
        actionWalkBackward();
        break;
    case VoiceCmd::TURN_LEFT:
        actionTurnLeft();
        break;
    case VoiceCmd::TURN_RIGHT:
        actionTurnRight();
        break;

    case VoiceCmd::SHIFT_LEFT:
        postureLeanLeft();
        osDelay(200);
        postureLookDown();
        osDelay(300);
        StandIdle();
        break;

    case VoiceCmd::SHIFT_RIGHT:
        postureLeanRight();
        osDelay(200);
        postureLookDown();
        osDelay(300);
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

void Motion4DOF::StandIdle()
{
    applyPosture(mid, mid, mid, mid);
}

void Motion4DOF::postureSitDown()
{
    applyPosture(mid, mid, mid + 40, mid - 40);
}

void Motion4DOF::postureLieDown()
{
    applyPosture(mid - 50, mid + 50, mid + 50, mid - 50);
}

void Motion4DOF::postureLookUp()
{
    applyPosture(mid + 30, mid - 30, mid + 30, mid - 30);
}

void Motion4DOF::postureLookDown()
{
    applyPosture(mid - 30, mid + 30, mid - 30, mid + 30);
}

void Motion4DOF::postureLeanLeft()
{
    applyPosture(mid - 40, mid, mid + 40, mid);
}

void Motion4DOF::postureLeanRight()
{
    applyPosture(mid, mid + 40, mid, mid - 40);
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