#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)

#include "Chassis_Mecanum.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "Global_status.h"
#

#include "referee_system.h"
#include "supercup.h"
#include "stm32_time.h"

#include "pid.h"
#include "ramp_generator.h"
#include "User_math.h"

pid_t chassis_speed_pid_FL, chassis_speed_pid_FR, chassis_speed_pid_BL, chassis_speed_pid_BR; // 轮子速度控制Pid
pid_t chassis_T_pid_x, chassis_T_pid_y, chassis_T_pid_w;
pid_t chassis_follow_pid;    // 底盘跟随
pid_t chassis_power_pid;     // 功率闭环控制pid
float Vx_now, Vy_now, W_now; // 正解底盘速度
extern RampGenerator Vx_ramp, Vy_ramp, Vw_ramp;
extern pid_t yaw_auto_location_pid;

// 软件功率控制，功率闭环
float power_limt(float FL_current, float FR_current, float BL_current, float BR_current,
                 float FL_speed, float FR_speed, float BL_speed, float BR_speed, float max_p)
{
    float current[4] = {FL_current, FR_current, BL_current, BR_current};
    float speed[4] = {FL_speed, FR_speed, BL_speed, BR_speed};
    float now_p = 0.0f;

    float a00 = 0.48872161;
    float a01 = -2.93589057e-04;
    float a10 = 5.3241338928e-05;
    float a02 = 2.70936086e-07;
    float a11 = 2.03985936e-06;
    float a20 = 2.17417767e-07;
    /*最大功率设置*/
    Supercap_set_power(max_p - 2.0f); //
    if ((cap.remain_vol <= 12) || (Global.Cap.mode == Not_FULL))
    {
        max_p -= 2.0f; // 2w余量
        if (cap.remain_vol <= 10)
            max_p -= 2.0f;
        if (cap.remain_vol <= 8)
            max_p -= 4.0f;
    }
    else if (cap.remain_vol > 12)
    {
        max_p += cap.remain_vol * 10; // 12
    }
    /*估算当前功率*/
    for (int i = 0; i < 4; i++)
    {
        now_p += fabs(a00 + a01 * speed[1] + a10 * current[i] +
                      a02 * speed[i] * speed[i] +
                      a11 * speed[i] * current[i] +
                      a20 * current[i] * current[i]);
    }
    float percentage = max_p / now_p;
    if (cap.Chassis_power >= (max_p / 2.0f)) // 底盘当前功率过小不使用闭环
        percentage += pid_cal(&chassis_power_pid, cap.Chassis_power, max_p);
    // if (Global.Chssis.input.y == 0 && Global.Chssis.input.x == 0)//急刹车
    //     percentage = 1;
    if ((/*(-Global.Chssis.input.x > 0.5 && Vx_now < -25) || (-Global.Chssis.input.x < -0.5 && Vx_now > 25) ||*/
        (Global.Chssis.input.y > 0.5 && Vy_now < -25) || (Global.Chssis.input.y < -0.5 && Vy_now > 25)&&Global.Chssis.mode == FLOW))
        percentage = 1;

    if(percentage<0)
        return 0.0f;
    if(percentage > 1.0f) // 防止输出过大
        return 1.0f;

    return percentage;
}
/************************************************************************************************************************************/
void val_limit(float *val, float MAX)
{
    /*输入限制*/
    if (fabs(*val) > MAX)
    {
        if (*val > 0)
            *val = MAX;
        else
            *val = -MAX;
    }
}

/**
 * @brief 底盘初始化
 *
 */
void Chassis_init()
{
    /*电机初始化*/
    CHASSISMotor_init(WHEEL_FL_MOVE_MOTOR_TYPE, WHEEL_FL);
    CHASSISMotor_init(WHEEL_FR_MOVE_MOTOR_TYPE, WHEEL_FR);
    CHASSISMotor_init(WHEEL_BL_MOVE_MOTOR_TYPE, WHEEL_BL);
    CHASSISMotor_init(WHEEL_BR_MOVE_MOTOR_TYPE, WHEEL_BR);

    /*轮组电机PID速度环初始化*/
    pid_set(&chassis_speed_pid_FR, 12000, 0, 120, CHASSISMOTOR_MAX_CURRENT, 5000); // 16000 1000
    pid_set(&chassis_speed_pid_FL, 12000, 0, 120, CHASSISMOTOR_MAX_CURRENT, 5000);
    pid_set(&chassis_speed_pid_BL, 12000, 0, 120, CHASSISMOTOR_MAX_CURRENT, 5000);
    pid_set(&chassis_speed_pid_BR, 12000, 0, 120, CHASSISMOTOR_MAX_CURRENT, 5000);
    /*底盘跟随PID*/
    pid_set(&chassis_follow_pid, 5.0f, 0.0f, 50.0f, 200, 40);
    /*底盘功率控制pid*/
    pid_set(&chassis_power_pid, 0.01f, 0.0f, 0.3f, 0.09f, 0.09f);
}

/**
 * @brief 底盘电机控制
 *
 * @param FL_speed 左前轮速度
 * @param FR_speed 右前轮速度
 * @param BL_speed 左后轮速度
 * @param BR_speed 右后轮速度
 */
float Plimit = 0.0f;
void ChassisMotor_Control(float Vx_speed, float Vy_speed, float R_speed)
{
    float FL_speed, FR_speed, BL_speed, BR_speed;

    /*最大速度限制*/
    val_limit(&Vx_speed, MAX_SPEED_VX);
    val_limit(&Vy_speed, MAX_SPEED_VY);
    val_limit(&R_speed, MAX_SPEED_WR);

    /*逆解轮速*/
    FR_speed = +Vx_speed - Vy_speed + (1 + CHASSIS_WZ_SET_SCALE) * R_speed;
    FL_speed = +Vx_speed + Vy_speed + (1 + CHASSIS_WZ_SET_SCALE) * R_speed;
    BL_speed = -Vx_speed + Vy_speed + (1 - CHASSIS_WZ_SET_SCALE) * R_speed;
    BR_speed = -Vx_speed - Vy_speed + (1 - CHASSIS_WZ_SET_SCALE) * R_speed;

    /*正解车速*/
    float omega_fl =  CHASSISMotor_get_data(WHEEL_FL).speed_rpm * 2 * PI / 60.0f; // 将rpm转换为弧度/秒
    float omega_fr = -CHASSISMotor_get_data(WHEEL_FR).speed_rpm * 2 * PI / 60.0f;
    float omega_bl =  CHASSISMotor_get_data(WHEEL_BL).speed_rpm * 2 * PI / 60.0f;
    float omega_br = -CHASSISMotor_get_data(WHEEL_BR).speed_rpm * 2 * PI / 60.0f;

    Vy_now =  (r_wheel / 4.0f) * (omega_fl + omega_fr + omega_bl + omega_br);
    Vx_now = -(r_wheel / 4.0f) * (omega_fl - omega_fr - omega_bl + omega_br);
    // 计算角速度（单位：弧度/秒）
    W_now  = -(r_wheel / (4.0f * R_body)) * (omega_fl - omega_fr + omega_bl - omega_br);

    // Vx_now = CHASSISMotor_get_data(WHEEL_FL).speed_rpm / 2.0f - CHASSISMotor_get_data(WHEEL_BL).speed_rpm / 2.0f;
    // Vy_now = CHASSISMotor_get_data(WHEEL_FL).speed_rpm / 2.0f - CHASSISMotor_get_data(WHEEL_FR).speed_rpm / 2.0f;
    // W_now  = CHASSISMotor_get_data(WHEEL_FR).speed_rpm / 2.0f + CHASSISMotor_get_data(WHEEL_BL).speed_rpm / 2.0f;

    /*电流计算*/
    float current[4] = {0};

    current[1] = pid_cal(&chassis_speed_pid_FR, (CHASSISMotor_get_data(WHEEL_FR).speed_rpm) / 19.0f * 0.104719755 * r_wheel, FR_speed); // 右前
    current[3] = pid_cal(&chassis_speed_pid_BR, (CHASSISMotor_get_data(WHEEL_BR).speed_rpm) / 19.0f * 0.104719755 * r_wheel, BR_speed);
    current[0] = pid_cal(&chassis_speed_pid_FL, (CHASSISMotor_get_data(WHEEL_FL).speed_rpm) / 19.0f * 0.104719755 * r_wheel, FL_speed);
    current[2] = pid_cal(&chassis_speed_pid_BL, (CHASSISMotor_get_data(WHEEL_BL).speed_rpm) / 19.0f * 0.104719755 * r_wheel, BL_speed);

    for (int i = 0; i < 4; i++)
    {
        if (current[i] >= (CHASSISMOTOR_MAX_CURRENT - 1))
            current[i] =   CHASSISMOTOR_MAX_CURRENT - 1;
        if (current[i] <= -(CHASSISMOTOR_MAX_CURRENT - 1))
            current[i] =  -(CHASSISMOTOR_MAX_CURRENT - 1);
    }

    /*功率限制*/
    if(Referee_data.Chassis_Power_Limit == 0)
        Plimit = 1.0f;
    else
        Plimit = power_limt(current[0], current[1], current[2], current[3],
                            CHASSISMotor_get_data(WHEEL_FL).speed_rpm,
                            CHASSISMotor_get_data(WHEEL_FR).speed_rpm,
                            CHASSISMotor_get_data(WHEEL_BL).speed_rpm,
                            CHASSISMotor_get_data(WHEEL_BR).speed_rpm,
                            Referee_data.Chassis_Power_Limit);

    /*电流设置*/
        CHASSISMotor_set(Plimit * current[0], WHEEL_FL);
        CHASSISMotor_set(Plimit * current[1], WHEEL_FR);
        CHASSISMotor_set(Plimit * current[2], WHEEL_BL);
        CHASSISMotor_set(Plimit * current[3], WHEEL_BR);

    if (Global.Control.mode == LOCK)
    {
        CHASSISMotor_set(0, WHEEL_FL);
        CHASSISMotor_set(0, WHEEL_FR);
        CHASSISMotor_set(0, WHEEL_BL);
        CHASSISMotor_set(0, WHEEL_BR);
    }
        // CHASSISMotor_set(0, WHEEL_FL);
        // CHASSISMotor_set(0, WHEEL_FR);
        // CHASSISMotor_set(0, WHEEL_BL);
        // CHASSISMotor_set(0, WHEEL_BR);
}

//
/**
 * @brief 底盘移动解算
 *
 */
float relative_angle;
float flow_speed(float relative) // 麦轮如果跟随过快很浪费能量，所以加一个这个
{
    if (abs(relative) > 90)
        return 0.1;
    else if (abs(relative) > 60)
        return 0.3;
    else if (abs(relative) > 25)
        return 0.5;
    else if (abs(relative) > 5)
        return 0.8;
    else
        return 1;
}
float spin_speed() // 小陀螺转速隨底盤最大功率限制变化
{
    if (Referee_data.Chassis_Power_Limit <= 60)
    {
        return CHASSIS_SPIN_SPEED;
    }
    else if (Referee_data.Chassis_Power_Limit > 60 && Referee_data.Chassis_Power_Limit <= 85)
    {
        return (CHASSIS_SPIN_SPEED + 0.8);
    }
    else if (Referee_data.Chassis_Power_Limit > 85 && Referee_data.Chassis_Power_Limit <= 100)
    {
        return (CHASSIS_SPIN_SPEED + 1.6);
    }
    else if (Referee_data.Chassis_Power_Limit > 100)
    {
        return (CHASSIS_SPIN_SPEED + 2.4);
    }
}
float X_speed, Y_speed, R_speed;
void Chassis_move()
{
    
    float sin_beta, cos_beta;

    /*化简多圈角度*/
    uint32_t mul;
    mul = fabs(relative_angle) / 180.0f;
    if (relative_angle > 180.0f)
    {
        if (mul % 2 == 1) // 处于-180度
            relative_angle -= (mul + 1) * 180.0f;
        else // 处于180度
            relative_angle -= mul * 180.0f;
    }
    if (relative_angle < -180.0f)
    {
        if (mul % 2 == 1) // 处于180度
            relative_angle += (mul + 1) * 180.0f;
        else // 处于-180度
            relative_angle += mul * 180.0f;
    }
    // relative_angle = 0;
    sin_beta = sinf(relative_angle / 180.0f * PI);
    cos_beta = cosf(relative_angle / 180.0f * PI);

    /*底盘速度更新*/
    RampGenerator_Update(&Vx_ramp, Get_sys_time_ms());
    RampGenerator_Update(&Vy_ramp, Get_sys_time_ms());
    Global.Chssis.input.x = RampGenerator_GetCurrent(&Vx_ramp);
    Global.Chssis.input.y = RampGenerator_GetCurrent(&Vy_ramp);

    /*运动分解*/
    X_speed = Global.Chssis.input.x * cos_beta - sin_beta * Global.Chssis.input.y;
    Y_speed = Global.Chssis.input.x * sin_beta + Global.Chssis.input.y * cos_beta;
    R_speed = Global.Chssis.input.r;

   // Chassisui_change(Global.Gimbal.Hanging_Shot_err); // ui

    /*模式选择*/
    switch (Global.Chssis.mode)
    {
    case FLOW:
        R_speed = flow_speed(relative_angle) * pid_cal(&chassis_follow_pid, DEG_TO_RAD * relative_angle, 0.0f) - 0.9 * yaw_auto_location_pid.total_out * DEG_TO_RAD;
        relative_angle = -(GIMBALMotor_get_data(YAWMotor).angle_cnt - 12.524324417114258);
        // relative_angle = (GIMBALMotor_get_data(YAWMotor).angle_cnt - 12.524324417114258);

        //    relative_angle = 0;
        break;
    case SPIN_P:
        R_speed = spin_speed();
        relative_angle = -(GIMBALMotor_get_data(YAWMotor).angle_cnt - 12.524324417114258 - GIMBALMotor_get_data(YAWMotor).round_speed * (-0.75f)); //-10.9
        break;
    case SPIN_N:
        R_speed = -spin_speed();
        relative_angle = -(GIMBALMotor_get_data(YAWMotor).angle_cnt - 12.524324417114258 - GIMBALMotor_get_data(YAWMotor).round_speed * (-0.5f));
        break;
    case SIDEWAYS: // 比赛期间临时改的，很麻烦，后面分成两个模式
        if (Global.Shoot.glass_mode == Glass_close)
        {
            R_speed = flow_speed(relative_angle) * pid_cal(&chassis_follow_pid, DEG_TO_RAD * relative_angle, 0.0f) - 0.9 * yaw_auto_location_pid.total_out * DEG_TO_RAD;
            relative_angle = -(GIMBALMotor_get_data(YAWMotor).angle_cnt - 12.524324417114258 + 90);
        }
        else
        {
            R_speed = 0;
            X_speed = 0;
            Y_speed = 0;
        }
    }

    /*底盘电机控制*/
    ChassisMotor_Control(X_speed, Y_speed, R_speed);
}
#endif

/************************************************************************************************************** */

// /**
//  * @brief 底盘初始化
//  *
//  */
// void Chassis_init()
// {
//     /*电机初始化*/
//     CHASSISMotor_init(M3508_P, WHEEL_FL);
//     CHASSISMotor_init(M3508_P, WHEEL_FR);
//     CHASSISMotor_init(M3508_P, WHEEL_BL);
//     CHASSISMotor_init(M3508_P, WHEEL_BR);
//     /*PID速度环初始化*/
//     // pid_set(&chassis_speed_pid_FL, 200.0f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
//     // pid_set(&chassis_speed_pid_FR, 200.0f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
//     // pid_set(&chassis_speed_pid_BL, 200.0f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
//     // pid_set(&chassis_speed_pid_BR, 200.0f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
//     pid_set(&chassis_speed_pid_FL, 2.0f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
//     pid_set(&chassis_speed_pid_FR, 2.0f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
//     pid_set(&chassis_speed_pid_BL, 2.0f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
//     pid_set(&chassis_speed_pid_BR, 2.0f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
//     /*底盘跟随PID*/
//     pid_set(&chassis_follow_pid, 5.0f, 0.0f, 10.0f, 200, 40);
//     /*底盘力控PID*/
//     pid_set(&chassis_T_pid_x, 20.0f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
//     pid_set(&chassis_T_pid_y, 20.0f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
//     pid_set(&chassis_T_pid_w, 10.0f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
//     /*底盘运动斜坡*/
//     RampGenerator_Init(&Vx_ramp, CHASSIS_TASK_TIME, 4, 30, 2);
//     RampGenerator_Init(&Vy_ramp, CHASSIS_TASK_TIME, 4, 30, 4);
//     /*默认地盘跟随模式*/
//     Global.Chssis.mode = FLOW;
// }

// /**
//  * @brief 设置底盘移动速度
//  *
//  */
// void Chassis_set_x(float x)
// {
//     RampGenerator_SetTarget(&Vx_ramp, x);
//     // Global.Chssis.input.x = x;
// }
// void Chassis_set_y(float y)
// {
//     RampGenerator_SetTarget(&Vy_ramp, y);
//     // Global.Chssis.input.y = y;
// }
// void Chassis_set_r(float r)
// {
//     Global.Chssis.input.r = r;
// }

// void Chassis_set_accel(float acc)
// {
//     RampGenerator_SetAccel(&Vx_ramp, acc);
//     RampGenerator_SetAccel(&Vy_ramp, acc);
// }

// /**
//  * @brief 底盘电机控制
//  *
//  * @param FL_speed 左前轮速度
//  * @param FR_speed 右前轮速度
//  * @param BL_speed 左后轮速度
//  * @param BR_speed 右后轮速度
//  */
// float Vx_now, Vy_now, W_now;
// float T_FR, T_FL, T_BR, T_BL, K;
// void ChassisMotor_Control(float Vx_speed, float Vy_speed, float R_speed)
// {
//     float FL_speed, FR_speed, BL_speed, BR_speed;
//     float Fx, Fy, T_w;
//     /*逆解轮速*/
//     FL_speed = RAD_S_TO_RPM * IK_WHEEL_FL(Vx_speed, Vy_speed, R_speed);
//     FR_speed = RAD_S_TO_RPM * IK_WHEEL_FR(Vx_speed, Vy_speed, R_speed);
//     BL_speed = RAD_S_TO_RPM * IK_WHEEL_BL(Vx_speed, Vy_speed, R_speed);
//     BR_speed = RAD_S_TO_RPM * IK_WHEEL_BR(Vx_speed, Vy_speed, R_speed);
//     /*正解车速*/
//     Vx_now = FK_VX(CHASSISMotor_get_data(WHEEL_FL).speed_rpm / 19.0f * RPM_TO_RAD_S,
//                    CHASSISMotor_get_data(WHEEL_FR).speed_rpm / 19.0f * RPM_TO_RAD_S,
//                    CHASSISMotor_get_data(WHEEL_BL).speed_rpm / 19.0f * RPM_TO_RAD_S,
//                    CHASSISMotor_get_data(WHEEL_BR).speed_rpm / 19.0f * RPM_TO_RAD_S);
//     Vy_now = FK_VY(CHASSISMotor_get_data(WHEEL_FL).speed_rpm / 19.0f * RPM_TO_RAD_S,
//                    CHASSISMotor_get_data(WHEEL_FR).speed_rpm / 19.0f * RPM_TO_RAD_S,
//                    CHASSISMotor_get_data(WHEEL_BL).speed_rpm / 19.0f * RPM_TO_RAD_S,
//                    CHASSISMotor_get_data(WHEEL_BR).speed_rpm / 19.0f * RPM_TO_RAD_S);
//     W_now = FK_OMEGA(CHASSISMotor_get_data(WHEEL_FL).speed_rpm / 19.0f * RPM_TO_RAD_S,
//                      CHASSISMotor_get_data(WHEEL_FR).speed_rpm / 19.0f * RPM_TO_RAD_S,
//                      CHASSISMotor_get_data(WHEEL_BL).speed_rpm / 19.0f * RPM_TO_RAD_S,
//                      CHASSISMotor_get_data(WHEEL_BR).speed_rpm / 19.0f * RPM_TO_RAD_S);
//     /*PID计算得出前馈力矩*/
//     Fx = pid_cal(&chassis_T_pid_x, Vx_now, Vx_speed);
//     Fy = pid_cal(&chassis_T_pid_y, Vy_now, Vy_speed);
//     T_w = pid_cal(&chassis_T_pid_w, W_now, R_speed);
//     T_FR = IK_WHEEL_FR_T(Fx, Fy, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
//     T_FL = IK_WHEEL_FL_T(Fx, Fy, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
//     T_BR = IK_WHEEL_BR_T(Fx, Fy, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
//     T_BL = IK_WHEEL_BL_T(Fx, Fy, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
//     float Plimit = 0.0f;

//     /*电流计算*/
//     float current[4] = {T_FL + pid_cal(&chassis_speed_pid_FL, CHASSISMotor_get_data(WHEEL_FL).speed_rpm / 14.0f, FL_speed),
//                         T_FR + pid_cal(&chassis_speed_pid_FR, CHASSISMotor_get_data(WHEEL_FR).speed_rpm / 14.0f, FR_speed),
//                         T_BL + pid_cal(&chassis_speed_pid_BL, CHASSISMotor_get_data(WHEEL_BL).speed_rpm / 14.0f, BL_speed),
//                         T_BR + pid_cal(&chassis_speed_pid_BR, CHASSISMotor_get_data(WHEEL_BR).speed_rpm / 14.0f, BR_speed)};
//     for (int i = 0; i < 4; i++)
//     {
//         if (current[i] >= (CHASSISMOTOR_MAX_CURRENT - 1))
//             current[i] = CHASSISMOTOR_MAX_CURRENT - 1;
//         if (current[i] <= -(CHASSISMOTOR_MAX_CURRENT - 1))
//             current[i] = -(CHASSISMOTOR_MAX_CURRENT - 1);
//     }
//     /*功率限制*/
//     Plimit = power_limt(current[0], current[1], current[2], current[3],
//                         CHASSISMotor_get_data(WHEEL_FL).speed_rpm,
//                         CHASSISMotor_get_data(WHEEL_FR).speed_rpm,
//                         CHASSISMotor_get_data(WHEEL_BL).speed_rpm,
//                         CHASSISMotor_get_data(WHEEL_BR).speed_rpm,
//                         Referee_data.Chassis_Power_Limit);
//     Plimit = 1; // 关闭功率控制
//     /*电流设置*/
//     CHASSISMotor_set(Plimit * current[0], WHEEL_FL);
//     CHASSISMotor_set(Plimit * current[1], WHEEL_FR);
//     CHASSISMotor_set(Plimit * current[2], WHEEL_BL);
//     CHASSISMotor_set(Plimit * current[3], WHEEL_BR);
// }

// /**
//  * @brief 底盘移动解算
//  *
//  */

// void Chassis_move()
// {
//     float X_speed, Y_speed, R_speed;
//     float relative_angle;
//     /*底盘速度更新*/
//     RampGenerator_Update(&Vx_ramp, Get_sys_time_ms());
//     RampGenerator_Update(&Vy_ramp, Get_sys_time_ms());
//     Global.Chssis.input.x = RampGenerator_GetCurrent(&Vx_ramp);
//     Global.Chssis.input.y = RampGenerator_GetCurrent(&Vy_ramp);
//     // relative_angle = GIMBALMotor_get_data(YAWMotor).angle_cnt - GIMBALMotor_get_data(YAWMotor).angle_zero;
//     relative_angle = 0;
//     // 运动分解
//     Y_speed = Global.Chssis.input.x * sinf(DEG_TO_RAD * GIMBALMotor_get_data(YAWMotor).angle) +
//               Global.Chssis.input.y * cosf(DEG_TO_RAD * GIMBALMotor_get_data(YAWMotor).angle);
//     X_speed = Global.Chssis.input.x * cosf(DEG_TO_RAD * GIMBALMotor_get_data(YAWMotor).angle) -
//               sinf(DEG_TO_RAD * GIMBALMotor_get_data(YAWMotor).angle) * Global.Chssis.input.y;
//     R_speed = Global.Chssis.input.r;
//     /*化简多圈角度*/
//     uint32_t mul = fabs(relative_angle) / 180.0f;
//     if (relative_angle > 180.0f)
//     {
//         if (mul % 2 == 1) // 处于-180度
//             relative_angle -= (mul + 1) * 180.0f;
//         else // 处于180度
//             relative_angle -= mul * 180.0f;
//     }
//     if (relative_angle < -180.0f)
//     {
//         if (mul % 2 == 1) // 处于180度
//             relative_angle += (mul + 1) * 180.0f;
//         else // 处于-180度
//             relative_angle += mul * 180.0f;
//     }
//     Chassisui_change(relative_angle);
//     /*模式选择*/
//     switch (Global.Chssis.mode)
//     {
//     case FLOW:
//         if(Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE)
//         // R_speed = pid_cal(&chassis_follow_pid, DEG_TO_RAD * relative_angle, 0.0f) - 0.9 * yaw_location_pid.total_out * DEG_TO_RAD;
//         R_speed = 0;
//         else
//         R_speed = pid_cal(&chassis_follow_pid, DEG_TO_RAD * relative_angle, 0.0f) - 0.9 * yaw_auto_location_pid.total_out * DEG_TO_RAD;
//         break;
//     case SPIN_P:
//         R_speed = 60 * RPM_TO_RAD_S;
//         break;
//     case SPIN_N:
//         R_speed = -60 * RPM_TO_RAD_S;
//         break;
//     }
//     ChassisMotor_Control(X_speed, Y_speed, R_speed);
// }