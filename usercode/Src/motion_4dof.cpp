#include "motion_4dof.hpp"
extern volatile VoiceCmd global_dog_action;

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



// 终极平滑矩阵引擎
// void Motion4DOF::matrixEngine(const int (*gait)[4], int phases, int repeats,
//                               float lf_swing, float lf_stance, float rf_swing, float rf_stance,
//                               float lh_swing, float lh_stance, float rh_swing, float rh_stance)
// {
//     // 【记录初衷】：记住我是因为什么指令进来的
//     VoiceCmd entry_cmd = global_dog_action;

//     float swing_angles[4] = {lf_swing, rf_swing, lh_swing, rh_swing};
//     float stance_angles[4] = {lf_stance, rf_stance, lh_stance, rh_stance};
//     LegChanel legs[4] = {LegChanel::FrontLeft, LegChanel::FrontRight, LegChanel::RearLeft, LegChanel::RearRight};

//     float stance_step[4];
//     for (int i = 0; i < 4; i++)
//     {
//         stance_step[i] = (stance_angles[i] - swing_angles[i]) / (phases - 1);
//     }

//     for (int r = 0; r < repeats; r++)
//     {
//         for (int p = 0; p < phases; p++)
//         {
//             // ==========================================
//             // 🚨 【紧急刹车打断机制】
//             // ==========================================
//             // 每次切换相位前，看一眼指令变了没有。如果变了（比如用户按了停止或转弯），立刻退出引擎！
//             if (global_dog_action != entry_cmd)
//             {
//                 return;
//             }

//             float start_angles[4];
//             float target_angles[4];

//             for (int i = 0; i < 4; i++)
//             {
//                 start_angles[i] = cur_angles[i];
//                 if (gait[p][i] == 1)
//                 {
//                     target_angles[i] = swing_angles[i];
//                 }
//                 else
//                 {
//                     target_angles[i] = cur_angles[i] + stance_step[i];

//                     // ==========================================
//                     // 🚨 【新增防护装甲：动态限幅器 (Clamp)】
//                     // 防止切换方向时，累加值超出物理极限导致舵机“骨折”
//                     // ==========================================

//                     // 1. 自动判断当前动作的上限和下限
//                     float min_angle = (swing_angles[i] < stance_angles[i]) ? swing_angles[i] : stance_angles[i];
//                     float max_angle = (swing_angles[i] > stance_angles[i]) ? swing_angles[i] : stance_angles[i];

//                     // 2. 如果盲目加算导致越界，强制将其锁死在合法边界上
//                     if (target_angles[i] < min_angle)
//                         target_angles[i] = min_angle;
//                     if (target_angles[i] > max_angle)
//                         target_angles[i] = max_angle;
//                 }
//             }

//             // ==========================================
//             // 1. 抬腿迈步 (极速前伸)
//             // ==========================================
//             for (int i = 0; i < 4; i++)
//             {
//                 if (gait[p][i] == 1)
//                 {
//                     servoDriver->setAngle(legs[i], target_angles[i]);
//                     cur_angles[i] = target_angles[i];
//                 }
//             }
//             // 【关键修改】：从过快的 15ms 恢复到 40ms，让舵机真正走到位！
//             osDelay(40);

//             // ==========================================
//             // 2. 慢推移动作 (抓地推进)
//             // ==========================================
//             // 【关键修改】：将插值帧数增加到 8 帧，每帧 15ms (总计120ms抓地推力)
//             // 这样能模仿“跳舞”时的重心摇摆，让前进变得极其扎实！
//             int smooth_frames = 8;
//             for (int j = 1; j <= smooth_frames; j++)
//             {
//                 if (global_dog_action != entry_cmd)
//                     return;

//                 float ratio = (float)j / smooth_frames;
//                 for (int i = 0; i < 4; i++)
//                 {
//                     if (gait[p][i] == 0)
//                     {
//                         float interpolated = start_angles[i] + (target_angles[i] - start_angles[i]) * ratio;
//                         servoDriver->setAngle(legs[i], interpolated);
//                         cur_angles[i] = interpolated;
//                     }
//                 }
//                 osDelay(15);
//             }
//         }
//     }
// }

void Motion4DOF::actionWalkForward()
{
    int speed = 250; // 稍微拉长一点时间，让跳舞的重心摇摆充分发挥物理惯性
    for (int i = 0; i < 2; i++)
    {
        if (global_dog_action != VoiceCmd::FORWARD)
            return;

        // 【跳舞式前进 - Phase 1】：向左倾斜
        // 左侧腿外翻压低（撑起重心），右侧腿伸直放松（借着摇摆惯性往前蹭）
        servoDriver->setAngle(LegChanel::FrontLeft, mid - 40);
        servoDriver->setAngle(LegChanel::RearLeft, mid + 40);
        servoDriver->setAngle(LegChanel::FrontRight, mid);
        servoDriver->setAngle(LegChanel::RearRight, mid);
        osDelay(speed);

        if (global_dog_action != VoiceCmd::FORWARD)
            return;

        // 【跳舞式前进 - Phase 2】：向右倾斜
        // 右侧腿外翻压低（撑起重心），左侧腿伸直放松（借着摇摆惯性往前蹭）
        servoDriver->setAngle(LegChanel::FrontLeft, mid);
        servoDriver->setAngle(LegChanel::RearLeft, mid);
        servoDriver->setAngle(LegChanel::FrontRight, mid + 40);
        servoDriver->setAngle(LegChanel::RearRight, mid - 40);
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

        // 【跳舞式后退 - Phase 1】：向左倾斜支撑
        // 同时右侧放松的腿，给一个向后的轻微角度
        servoDriver->setAngle(LegChanel::FrontLeft, mid - 40);
        servoDriver->setAngle(LegChanel::RearLeft, mid + 40);
        servoDriver->setAngle(LegChanel::FrontRight, mid + 20); // 往后拨
        servoDriver->setAngle(LegChanel::RearRight, mid + 20);
        osDelay(speed);

        if (global_dog_action != VoiceCmd::BACKWARD)
            return;

        // 【跳舞式后退 - Phase 2】：向右倾斜支撑
        // 同时左侧放松的腿，给一个向后的轻微角度
        servoDriver->setAngle(LegChanel::FrontLeft, mid + 20); // 往后拨
        servoDriver->setAngle(LegChanel::RearLeft, mid + 20);
        servoDriver->setAngle(LegChanel::FrontRight, mid + 40);
        servoDriver->setAngle(LegChanel::RearRight, mid - 40);
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

        // 向左转：左倾支撑时，让右侧悬空的腿向前上跨
        servoDriver->setAngle(LegChanel::FrontLeft, mid - 40);
        servoDriver->setAngle(LegChanel::RearLeft, mid + 40);
        servoDriver->setAngle(LegChanel::FrontRight, mid - 20);
        servoDriver->setAngle(LegChanel::RearRight, mid + 20);
        osDelay(speed);

        if (global_dog_action != VoiceCmd::TURN_LEFT)
            return;

        // 向左转：右倾支撑时，让左侧悬空的腿向内后收
        servoDriver->setAngle(LegChanel::FrontLeft, mid + 20);
        servoDriver->setAngle(LegChanel::RearLeft, mid - 20);
        servoDriver->setAngle(LegChanel::FrontRight, mid + 40);
        servoDriver->setAngle(LegChanel::RearRight, mid - 40);
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

        // 向右转：左倾支撑时，让右侧悬空的腿向内后收
        servoDriver->setAngle(LegChanel::FrontLeft, mid - 40);
        servoDriver->setAngle(LegChanel::RearLeft, mid + 40);
        servoDriver->setAngle(LegChanel::FrontRight, mid + 20);
        servoDriver->setAngle(LegChanel::RearRight, mid - 20);
        osDelay(speed);

        if (global_dog_action != VoiceCmd::TURN_RIGHT)
            return;

        // 向右转：右倾支撑时，让左侧悬空的腿向前上跨
        servoDriver->setAngle(LegChanel::FrontLeft, mid - 20);
        servoDriver->setAngle(LegChanel::RearLeft, mid + 20);
        servoDriver->setAngle(LegChanel::FrontRight, mid + 40);
        servoDriver->setAngle(LegChanel::RearRight, mid - 40);
        osDelay(speed);
    }
}

// ==========================================
// 核心中枢：指令分发器
// ==========================================
void Motion4DOF::ExecuteCommand(VoiceCmd cmd)
{
    // 极其稳定的经典对角爬行步态 (Creep Gait) - 适合慢速、复杂地形
    // static const int creep[4][4] = {
    //     {0, 0, 0, 1},
    //     {0, 1, 0, 0},
    //     {0, 0, 1, 0},
    //     {1, 0, 0, 0}};

    // // 🚀【重磅新增】：对角小跑步态 (Trot Gait)
    // // 利用对角线双腿同时运动，强制重心左右摇摆，是最适合无膝盖 4DOF 狗的高速步态！
    // static const int trot[2][4] = {
    //     {1, 0, 0, 1}, // 相位1：左前、右后迈步；右前、左后抓地
    //     {0, 1, 1, 0}  // 相位2：右前、左后迈步；左前、右后抓地
    // };

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

    // --- 2. 运动指令 (全部换装 Trot 极速引擎) ---
    // case VoiceCmd::FORWARD:
    //     // 前进后退使用 Trot (极速小跑引擎)，循环 6 次
    //     matrixEngine(trot, 2, 6, LF_F, LF_B, RF_F, RF_B, LH_F, LH_B, RH_F, RH_B);
    //     break;
    // case VoiceCmd::BACKWARD:
    //     matrixEngine(trot, 2, 6, LF_B, LF_F, RF_B, RF_F, LH_B, LH_F, RH_B, RH_F);
    //     break;

    // case VoiceCmd::TURN_LEFT:
    //     // 【关键修改】：转弯必须退回极其稳定的 Creep (四相爬行步态)，利用 3 点支撑产生强大扭力！
    //     // 并且参数已和原右转互换，解决转向相反问题
    //     matrixEngine(creep, 4, 3, LF_F, LF_B, RF_B, RF_F, LH_F, LH_B, RH_B, RH_F);
    //     break;

    // case VoiceCmd::TURN_RIGHT:
    //     // 使用 Creep 步态向右转
    //     matrixEngine(creep, 4, 3, LF_B, LF_F, RF_F, RF_B, LH_B, LH_F, RH_F, RH_B);
    //     break;
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
        // 既然没法真平移，我们就做一个极其可爱的“向左侧身卧倒闪避”的连贯动作
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

    // ... 下面的 3. 静态姿态 和 4. 花式动作 保持你原来的代码完全不变即可 ...
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
    if (cur_angles[0] == mid && cur_angles[1] == mid &&
        cur_angles[2] == mid && cur_angles[3] == mid)
    {
        return;
    }

    servoDriver->setAngle(LegChanel::FrontLeft, mid);
    servoDriver->setAngle(LegChanel::FrontRight, mid);
    servoDriver->setAngle(LegChanel::RearLeft, mid);
    servoDriver->setAngle(LegChanel::RearRight, mid);

    // 更新记忆
    cur_angles[0] = mid;
    cur_angles[1] = mid;
    cur_angles[2] = mid;
    cur_angles[3] = mid;
}

void Motion4DOF::postureSitDown()
{
    servoDriver->setAngle(LegChanel::FrontLeft, mid);
    servoDriver->setAngle(LegChanel::FrontRight, mid);
    servoDriver->setAngle(LegChanel::RearLeft, mid + 40);
    servoDriver->setAngle(LegChanel::RearRight, mid - 40);

    cur_angles[0] = mid;
    cur_angles[1] = mid;
    cur_angles[2] = mid + 40;
    cur_angles[3] = mid - 40;
}

void Motion4DOF::postureLieDown()
{
    servoDriver->setAngle(LegChanel::FrontLeft, mid - 50);
    servoDriver->setAngle(LegChanel::FrontRight, mid + 50);
    servoDriver->setAngle(LegChanel::RearLeft, mid + 50);
    servoDriver->setAngle(LegChanel::RearRight, mid - 50);

    cur_angles[0] = mid - 50;
    cur_angles[1] = mid + 50;
    cur_angles[2] = mid + 50;
    cur_angles[3] = mid - 50;
}

void Motion4DOF::postureLookUp()
{
    servoDriver->setAngle(LegChanel::FrontLeft, mid + 30);
    servoDriver->setAngle(LegChanel::FrontRight, mid - 30);
    servoDriver->setAngle(LegChanel::RearLeft, mid + 30);
    servoDriver->setAngle(LegChanel::RearRight, mid - 30);

    cur_angles[0] = mid + 30;
    cur_angles[1] = mid - 30;
    cur_angles[2] = mid + 30;
    cur_angles[3] = mid - 30;
}

void Motion4DOF::postureLookDown()
{
    servoDriver->setAngle(LegChanel::FrontLeft, mid - 30);
    servoDriver->setAngle(LegChanel::FrontRight, mid + 30);
    servoDriver->setAngle(LegChanel::RearLeft, mid - 30);
    servoDriver->setAngle(LegChanel::RearRight, mid + 30);

    cur_angles[0] = mid - 30;
    cur_angles[1] = mid + 30;
    cur_angles[2] = mid - 30;
    cur_angles[3] = mid + 30;
}

void Motion4DOF::postureLeanLeft()
{
    servoDriver->setAngle(LegChanel::FrontLeft, mid - 40);
    servoDriver->setAngle(LegChanel::RearLeft, mid + 40);
    servoDriver->setAngle(LegChanel::FrontRight, mid);
    servoDriver->setAngle(LegChanel::RearRight, mid);

    cur_angles[0] = mid - 40;
    cur_angles[1] = mid;
    cur_angles[2] = mid + 40;
    cur_angles[3] = mid;
}

void Motion4DOF::postureLeanRight()
{
    servoDriver->setAngle(LegChanel::FrontLeft, mid);
    servoDriver->setAngle(LegChanel::RearLeft, mid);
    servoDriver->setAngle(LegChanel::FrontRight, mid + 40);
    servoDriver->setAngle(LegChanel::RearRight, mid - 40);

    cur_angles[0] = mid;
    cur_angles[1] = mid + 40;
    cur_angles[2] = mid;
    cur_angles[3] = mid - 40;
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