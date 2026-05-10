/*
 * @Date: 2025-09-21 18:40:08
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-04 21:35:04
 * @FilePath: \Season-26-Code\User\Software\Gimbal\Gimbal_Direct.c
 */
#include "robot_param.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)

#include "Gimbal_Direct.h"
#include "Gimbal.h"
#include "Global_status.h"
#include "UART_data_txrx.h"
#include "User_math.h"
#include "pid.h"
#include "ramp_generator.h"
#include "IMU_updata.h"

extern float W_now;

#define PITCH_GRAVITY_FEEDFORWARD 800  // 重力补偿前馈值，根据实际情况调整

/**
 * @brief 云台初始化
 *
 */
void Gimbal_init()
{
    /*电机初始化*/
    GIMBALMotor_init(GIMBAL_YAW_MOTOR_TYPE, YAWMotor);
    GIMBALMotor_init(GIMBAL_PITCH_MOTOR_TYPE, PITCHMotor);

    /*外环PID初始化 (位置环): 输入视觉像素error, 输出期望速度*/
    pid_set(&Gimbal.yaw_location_pid, 0.0f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 2000);
    pid_set(&Gimbal.pitch_location_pid, 0.0f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 2000);

    /*内环PID初始化 (速度环): 输入期望速度与实际速度, 输出电压值*/
    pid_set(&Gimbal.yaw_speed_pid, 3500.0f, 0.5f, 2.0f, GIMBALMOTOR_MAX_CURRENT, 5000);
    pid_set(&Gimbal.pitch_speed_pid, 3800.0f, 0.5f, 1.0f, GIMBALMOTOR_MAX_CURRENT, 5000);

    // 大疆6020电机
    GIMBALMotor_setzero(YAW_ZERO, YAWMotor);
    GIMBALMotor_setzero(PITCH_ZERO, PITCHMotor);
}
static uint16_t pitch_ecd;
static uint16_t yaw_ecd;
void Gimbal_control()
{
    static float pitch_speed_set;
    static float yaw_speed_set;
    static int16_t pitch_out;
    static int16_t yaw_out;

    // 双环PID控制
    // 外环 (位置环): 视觉像素error → 期望速度
    if (vision_data.target_detected == 1)
    {
        // pitch_speed_set = pid_cal(&Gimbal.pitch_location_pid, vision_data.pitch_error, 0);
        // yaw_speed_set = pid_cal(&Gimbal.yaw_location_pid, vision_data.yaw_error, 0);
    }
    else
    {
        static int8_t pitch_scan_dir = 1;
        static int8_t yaw_scan_dir = 1;

        pitch_ecd = GIMBALMotor_get_data(PITCHMotor).ecd;
        yaw_ecd = GIMBALMotor_get_data(YAWMotor).ecd;

        // Pitch在ecd 300~800之间扫描
        if (pitch_ecd >= 800 && pitch_ecd < 5000)
            pitch_scan_dir = -1;
        else if (pitch_ecd >= 7000)
            pitch_scan_dir = 1;
        // pitch_speed_set = 0.2f * pitch_scan_dir;
        pitch_speed_set=0;

        // Yaw在ecd 800~2700之间扫描
        if (yaw_ecd >= 2700 && yaw_ecd < 5300)
            yaw_scan_dir = -1;
        else if (yaw_ecd<=1100||yaw_ecd >= 5300)
            yaw_scan_dir = 1;
        // yaw_speed_set = 0.5f * yaw_scan_dir;
        yaw_speed_set=0;
    }
    // 内环 (速度环): 期望速度 vs 电机实际速度 → 电压
    pitch_out = (int16_t)(pid_cal(&Gimbal.pitch_speed_pid, Gimbal.pitch_speed_now, pitch_speed_set) + PITCH_GRAVITY_FEEDFORWARD);
    yaw_out = (int16_t)pid_cal(&Gimbal.yaw_speed_pid, Gimbal.yaw_speed_now, yaw_speed_set);
    GIMBALMotor_set(pitch_out, PITCHMotor);
    GIMBALMotor_set(yaw_out, YAWMotor);

    if (Global.Auto.input.Auto_control_online > 0)
        Global.Auto.input.Auto_control_online--;
}

#endif
