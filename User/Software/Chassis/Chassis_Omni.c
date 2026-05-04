#include "robot_param.h"

#if (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)
#include "Chassis_Omni.h"
#include "Chassis.h"
#include "Global_status.h"
#include "Gimbal.h"

#include "referee_system.h"
#include "supercup.h"
#include "stm32_time.h"
#include "IMU_updata.h"

#include "pid.h"
#include "ramp_generator.h"
#include "User_math.h"

pid_t chassis_speed_pid_FL, chassis_speed_pid_FR, chassis_speed_pid_BL, chassis_speed_pid_BR; // 轮子速度控制Pid
pid_t chassis_T_pid_x, chassis_T_pid_y, chassis_T_pid_w;
pid_t chassis_follow_pid; // 底盘跟随
pid_t chassis_power_pid;
extern RampGenerator Vx_ramp, Vy_ramp, Vw_ramp;
float Chassis_pitch_angle;
extern pid_t yaw_auto_location_pid;

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

    /*PID速度环初始化*/
    pid_set(&chassis_speed_pid_FL, 0.5f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    pid_set(&chassis_speed_pid_FR, 0.5f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    pid_set(&chassis_speed_pid_BL, 0.5f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    pid_set(&chassis_speed_pid_BR, 0.5f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    /*底盘跟随PID*/
    pid_set(&chassis_follow_pid, 10.0f, 0.0f, 2.0f, 200, 40);
    /*底盘力控PID*/
    pid_set(&chassis_T_pid_x, 30.0f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    pid_set(&chassis_T_pid_y, 30.0f, 0.0, 0, CHASSISMOTOR_MAX_CURRENT, 10000);
    pid_set(&chassis_T_pid_w, 10.0f, 0.0, 20.0, CHASSISMOTOR_MAX_CURRENT, 10000);
    /*底盘功率控制pid*/
    pid_set(&chassis_power_pid, 0.01f, 0.0f, 0.0f, 0.1f, 0.1f);
    /*底盘运动斜坡*/
    RampGenerator_Init(&Vx_ramp, CHASSIS_TASK_TIME, 40, 40, 2);
    RampGenerator_Init(&Vy_ramp, CHASSIS_TASK_TIME, 40, 40, 4);
    RampGenerator_Init(&Vw_ramp, CHASSIS_TASK_TIME, 300, 300, 4);

    /*默认地盘跟随模式*/
    Global.Chssis.mode = FLOW;
}

/**
 * @brief 获取底盘pitch角度数据，需pitch电机精确设置零点，仅能在云台底盘同方向的时候使用。
 *
 * @return float
 */
float Chassis_angle_getpitch()
{
    return GIMBALMotor_get_data(PITCHMotor).motor_data.para.angle + IMU_data.AHRS.pitch * RAD_TO_DEG;
}

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
    Supercap_set_power(max_p - 4.0f);
    if ((cap.remain_vol <= 9) || (Global.Cap.mode == Not_FULL))
    {
        max_p -= 2.0f; // 2w余量
        if (cap.remain_vol <= 9)
            max_p -= 3.0f;
    }

    else if (cap.remain_vol > 9)
    {
        max_p += cap.remain_vol * 12;
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
    if (cap.Chassis_power >= (max_p / 2.0f) && (pid_cal(&chassis_power_pid, cap.Chassis_power, max_p) < 0)) // 底盘当前功率过小不使用闭环
        percentage += pid_cal(&chassis_power_pid, cap.Chassis_power, max_p);
    if (percentage > 1.0f)
        return 1.0f;
    return percentage;
}



/**
 * @brief 底盘电机控制
 *
 * @param FL_speed 左前轮速度
 * @param FR_speed 右前轮速度
 * @param BL_speed 左后轮速度
 * @param BR_speed 右后轮速度
 */
float Vx_now, Vy_now, W_now;
void ChassisMotor_Control(float Vx_speed, float Vy_speed, float R_speed)
{
    float T_FR, T_FL, T_BR, T_BL;
    float F_FR, F_FL, F_BR, F_BL;

    float FL_speed, FR_speed, BL_speed, BR_speed;
    float Fx, Fy, T_w;
    float current[4];
    /*逆解轮速*/
    FL_speed = RAD_S_TO_RPM * IK_WHEEL_FL(Vx_speed, Vy_speed, R_speed);
    FR_speed = RAD_S_TO_RPM * IK_WHEEL_FR(Vx_speed, Vy_speed, R_speed);
    BL_speed = RAD_S_TO_RPM * IK_WHEEL_BL(Vx_speed, Vy_speed, R_speed);
    BR_speed = RAD_S_TO_RPM * IK_WHEEL_BR(Vx_speed, Vy_speed, R_speed);
    /*正解车速*/
    Vx_now = FK_VX(CHASSISMotor_get_data(WHEEL_FL).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S,
                   CHASSISMotor_get_data(WHEEL_FR).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S,
                   CHASSISMotor_get_data(WHEEL_BL).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S,
                   CHASSISMotor_get_data(WHEEL_BR).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S);
    Vy_now = FK_VY(CHASSISMotor_get_data(WHEEL_FL).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S,
                   CHASSISMotor_get_data(WHEEL_FR).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S,
                   CHASSISMotor_get_data(WHEEL_BL).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S,
                   CHASSISMotor_get_data(WHEEL_BR).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S);
    W_now = FK_OMEGA(CHASSISMotor_get_data(WHEEL_FL).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S,
                     CHASSISMotor_get_data(WHEEL_FR).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S,
                     CHASSISMotor_get_data(WHEEL_BL).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S,
                     CHASSISMotor_get_data(WHEEL_BR).speed_rpm / WHEEL_RATIO * RPM_TO_RAD_S);
    /*PID计算得出前馈力矩*/
    Fx = pid_cal(&chassis_T_pid_x, Vx_now, Vx_speed);
    Fy = pid_cal(&chassis_T_pid_y, Vy_now, Vy_speed);
    T_w = pid_cal(&chassis_T_pid_w, W_now, R_speed);
    T_FR = IK_WHEEL_FR_T(0, 0, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    T_FL = IK_WHEEL_FL_T(0, 0, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    T_BR = IK_WHEEL_BR_T(0, 0, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    T_BL = IK_WHEEL_BL_T(0, 0, T_w) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    F_FR = IK_WHEEL_FR_T(Fx, Fy, 0) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    F_FL = IK_WHEEL_FL_T(Fx, Fy, 0) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    F_BR = IK_WHEEL_BR_T(Fx, Fy, 0) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    F_BL = IK_WHEEL_BL_T(Fx, Fy, 0) / CHASSISMOTOR_T_A * CHASSISMOTOR_MAX_CURRENT / 20.0f;
    Chassis_pitch_angle = Chassis_angle_getpitch(); // 获取底盘角度判断是否上坡；
    if (fabsf(Chassis_pitch_angle) >= 15)           // 解算结果为坡上
    {
        T_FR *= 0.5f;
        T_FL *= 0.5f;
        T_BL *= 2.0f;
        T_BR *= 2.0f;
        F_FR *= 0.5f;
        F_FL *= 0.5f;
        F_BR *= 2.0f;
        F_BL *= 2.0f;
    }
    float Plimit = 0.0f;
    /*电流计算*/
    current[0] = (T_FL + F_FL) + pid_cal(&chassis_speed_pid_FL, CHASSISMotor_get_data(WHEEL_FL).speed_rpm / WHEEL_RATIO, FL_speed);
    current[1] = (T_FR + F_FR) + pid_cal(&chassis_speed_pid_FR, CHASSISMotor_get_data(WHEEL_FR).speed_rpm / WHEEL_RATIO, FL_speed);
    current[2] = (T_BL + F_BL) + pid_cal(&chassis_speed_pid_BL, CHASSISMotor_get_data(WHEEL_BL).speed_rpm / WHEEL_RATIO, FL_speed);
    current[3] = (T_BR + F_BR) + pid_cal(&chassis_speed_pid_BR, CHASSISMotor_get_data(WHEEL_BR).speed_rpm / WHEEL_RATIO, FL_speed);
    // 统一限幅
    float max_current = 0;
    int i = 0;
    for (i = 0; i < 4; i++)
    { // 求最大值
        if (fabsf(current[i]) > max_current)
            max_current = fabsf(current[i]);
    }
    if (max_current > MAX_CURRENT)
        for (i = 0; i < 4; i++)
        { // 限幅
            current[i] *= (MAX_CURRENT / max_current);
        }
    /*功率限制*/
    Plimit = power_limt(current[0], current[1], current[2], current[3],
                        CHASSISMotor_get_data(WHEEL_FL).speed_rpm,
                        CHASSISMotor_get_data(WHEEL_FR).speed_rpm,
                        CHASSISMotor_get_data(WHEEL_BL).speed_rpm,
                        CHASSISMotor_get_data(WHEEL_BR).speed_rpm,
                        Referee_data.Chassis_Power_Limit);
    // Plimit = 1; // 关闭功率控制
    float limit[4] = {
        (Plimit * current[0] - T_FL - chassis_speed_pid_FL.total_out) / F_FL,
        (Plimit * current[1] - T_FR - chassis_speed_pid_FR.total_out) / F_FR,
        (Plimit * current[2] - T_BL - chassis_speed_pid_BL.total_out) / F_BL,
        (Plimit * current[3] - T_BR - chassis_speed_pid_BR.total_out) / F_BR};

    /*电流设置*/
    CHASSISMotor_set(limit[0] * F_FL + T_FL + chassis_speed_pid_FL.total_out, WHEEL_FL);
    CHASSISMotor_set(limit[1] * F_FR + T_FR + chassis_speed_pid_FR.total_out, WHEEL_FR);
    CHASSISMotor_set(limit[2] * F_BL + T_BL + chassis_speed_pid_BL.total_out, WHEEL_BL);
    CHASSISMotor_set(limit[3] * F_BR + T_BR + chassis_speed_pid_BR.total_out, WHEEL_BR);
}

/**
 * @brief 底盘移动解算
 *
 */


#define K 0.05

void Chassis_move()
{
    float X_speed, Y_speed, R_speed;
    float relative_angle;
    /*底盘速度更新*/
    RampGenerator_Update(&Vx_ramp, Get_sys_time_ms());
    RampGenerator_Update(&Vy_ramp, Get_sys_time_ms());
    Global.Chssis.input.x = RampGenerator_GetCurrent(&Vx_ramp);
    Global.Chssis.input.y = RampGenerator_GetCurrent(&Vy_ramp);
    relative_angle = GIMBALMotor_get_data(YAWMotor).motor_data.para.angle_cnt - YAW_ZERO;
    // 运动分解
    if (Global.Chssis.mode == FLOW)
    {
        Y_speed = Global.Chssis.input.x * sinf(DEG_TO_RAD * relative_angle) +
                  Global.Chssis.input.y * cosf(DEG_TO_RAD * relative_angle);
        X_speed = Global.Chssis.input.x * cosf(DEG_TO_RAD * relative_angle) -
                  Global.Chssis.input.y * sinf(DEG_TO_RAD * relative_angle);
        R_speed = Global.Chssis.input.r;
    }
    else if (Global.Chssis.mode == SPIN_P || Global.Chssis.mode == SPIN_N)
    {
        Y_speed = Global.Chssis.input.x * sinf(K * W_now + DEG_TO_RAD * GIMBALMotor_get_data(YAWMotor).motor_data.para.angle) +
                  Global.Chssis.input.y * cosf(K * W_now + DEG_TO_RAD * GIMBALMotor_get_data(YAWMotor).motor_data.para.angle);
        X_speed = Global.Chssis.input.x * cosf(K * W_now + DEG_TO_RAD * GIMBALMotor_get_data(YAWMotor).motor_data.para.angle) -
                  Global.Chssis.input.y * sinf(K * W_now + DEG_TO_RAD * GIMBALMotor_get_data(YAWMotor).motor_data.para.angle) ;
        R_speed = Global.Chssis.input.r;
    }
    relative_angle = GIMBALMotor_get_data(YAWMotor).motor_data.para.angle_cnt - GIMBALMotor_get_data(YAWMotor).motor_data.angle_zero;
    /*化简多圈角度*/
    uint32_t mul = fabs(relative_angle) / 180.0f;
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
    //Chassisui_change(relative_angle);
    /*模式选择*/
    switch (Global.Chssis.mode)
    {
    case FLOW:
        // if(Chassis_pitch_angle>)
        if (Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE)
            R_speed = pid_cal(&chassis_follow_pid, DEG_TO_RAD * relative_angle, 0.0f);
        else
            R_speed = pid_cal(&chassis_follow_pid, DEG_TO_RAD * relative_angle, 0.0f); // - 0.9 * yaw_auto_location_pid.total_out * DEG_TO_RAD;
        break;
    case SPIN_P:
        R_speed = R_SPEED_OMNI * RPM_TO_RAD_S;
        break;
    case SPIN_N:
        R_speed = -R_SPEED_OMNI * RPM_TO_RAD_S;
        break;
    }
    ChassisMotor_Control(X_speed, Y_speed, R_speed);
}
#endif