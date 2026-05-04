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
#include"UART_data_txrx.h"
#include "User_math.h"
#include "pid.h"
#include "ramp_generator.h"
#include "IMU_updata.h"


extern float W_now;
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
    pid_set(&Gimbal.yaw_location_pid, 0.1f, 0.0f, 0.0f, GIMBALMOTOR_MAX_CURRENT, 2000);
    pid_set(&Gimbal.pitch_location_pid, 4.5f, 0.0f, 1.0f, GIMBALMOTOR_MAX_CURRENT, 2000);

    /*内环PID初始化 (速度环): 输入期望速度与实际速度, 输出电压值*/
    pid_set(&Gimbal.yaw_speed_pid, 250.0f, 0.5f, 2.0f, GIMBALMOTOR_MAX_CURRENT, 5000);
    pid_set(&Gimbal.pitch_speed_pid, 20.0f, 0.5f, 1.0f, GIMBALMOTOR_MAX_CURRENT, 5000);

    // 大疆6020电机
    GIMBALMotor_setzero(YAW_ZERO, YAWMotor);
    GIMBALMotor_setzero(PITCH_ZERO, PITCHMotor);
}

void Gimbal_control()
{
    // 双环PID控制
    // 外环 (位置环): 视觉像素error → 期望速度
    float pitch_speed_set = pid_cal(&Gimbal.pitch_location_pid, vision_data.pitch_error, 0);
    float yaw_speed_set   = pid_cal(&Gimbal.yaw_location_pid, vision_data.yaw_error, 0);

    // 内环 (速度环): 期望速度 vs 电机实际速度 → 电压
    int16_t pitch_out = (int16_t)pid_cal(&Gimbal.pitch_speed_pid, Gimbal.pitch_speed_now, pitch_speed_set);
    int16_t yaw_out   = (int16_t)pid_cal(&Gimbal.yaw_speed_pid, Gimbal.yaw_speed_now, yaw_speed_set);

    GIMBALMotor_set(pitch_out, PITCHMotor);
    GIMBALMotor_set(yaw_out,   YAWMotor);

    if (Global.Auto.input.Auto_control_online > 0)
        Global.Auto.input.Auto_control_online--;
}

#endif
