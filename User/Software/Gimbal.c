#include "Gimbal.h"
#include "Global_status.h"
#include "remote_control.h"
#include "motor.h"
#include "pid.h"
#include "User_math.h"
#include "UART_data_txrx.h"
#include "IMU_updata.h"
#include "dm_imu.h"
#include "USB_VirCom.h"

extern volatile uint8_t motor_kill_switch;

Gimbal_t Gimbal;
// static char buffer[200];
// static int send_num;
#define PITCH_GRAVITY_FEEDFORWARD 1000 // 重力补偿前馈值，1200

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param          none
 * @retval         none
 */
void Gimbal_Init()
{
    // 云台电机初始化
    GIMBALMotor_init(GIMBAL_YAW_MOTOR_TYPE, YAWMotor);
    DJIMotor_Init(GIMBAL_PITCH_MOTOR_TYPE, PITCHMotor);

    /*PID速度环初始化*/
    // 速度环
    // pitch电机
    PID_Set(&Gimbal.pitch_speed_pid, 30.0f, 0.0f, 0.0f, 1000000.0f, 1000000.0f);

    // 位置环
    // yaw电机
    PID_Set(&Gimbal.yaw_location_pid, 0.1f, 0.0f, 0.0f, 1000000.0f, 1000000.0f);
    // pitch电机
    PID_Set(&Gimbal.pitch_location_pid, 0.1f, 0.0f, 0.0f, 1000000.0f, 1000000.0f);
}

/*-------------------- Update --------------------*/

/**
 * @brief          控制量更新（包括状态量和目标量）
 * @param          none
 * @retval         none
 */
void Gimbal_Updater()
{

    Gimbal.pitch_speed_now = DJIMotor_GetData(PITCHMotor).speed_rpm;
    Gimbal.pitch_location_now = DJIMotor_GetData(PITCHMotor).ecd_cnt;
    Gimbal.yaw_location_now = DMMotor_GetData(YAWMotor).motor_data.para.pos_cnt;
    Gimbal.yaw_speed_now = DMMotor_GetData(YAWMotor).motor_data.para.vel;
}

/*-------------------- Calculate --------------------*/

/**
 * @brief          控制量解算
 * @param          none
 * @retval         none
 */
void Gimbal_Calculater()
{
}

/*-------------------- Control --------------------*/

/**
 * @brief          电流值设置
 * @param          none
 * @retval         none
 */
void Gimbal_Controller()
{
    if (motor_kill_switch)
        return;

    static int8_t pitch_scan_dir = 1;
    static int8_t yaw_scan_dir = 1;

    if (vision_data.target_detected == 1)
    // 自瞄，双环pid
    {
        Gimbal.yaw_location_set = vision_data.yaw_error;
        Gimbal.pitch_location_set = vision_data.pitch_error;

        if (Gimbal.pitch_location_set > 4)
        {
            pitch_scan_dir = 1;
        }
        else if (Gimbal.pitch_location_set < -4)
        {
            pitch_scan_dir = -1;
        }
        else
        {
            pitch_scan_dir = 0;
        }

        Gimbal.yaw_speed_set = -PID_Cal(&Gimbal.yaw_location_pid, 0, Gimbal.yaw_location_set) * 0.05;
        Gimbal.pitch_speed_set = PID_Cal(&Gimbal.pitch_location_pid, 0, Gimbal.pitch_location_set);
    }
    else
    // 搜寻，单环pid
    {

        // Pitch在ecd 2000~20000之间扫描
        if (Gimbal.pitch_location_now >= 20000)
            pitch_scan_dir = -1;
        else if (Gimbal.pitch_location_now <= 2000)
            pitch_scan_dir = 1;
        Gimbal.pitch_speed_set = 20.0f * pitch_scan_dir;
        // Gimbal.pitch_speed_set = 0;

        // Yaw在ecd 800~2700之间扫描
        if (Gimbal.yaw_location_now >= 0.8)
            yaw_scan_dir = -1;
        else if (Gimbal.yaw_location_now <= -0.8)
            yaw_scan_dir = 1;
        Gimbal.yaw_speed_set = 0.4f * yaw_scan_dir;
        // Gimbal.yaw_speed_set = 0;

    } // 电机失能时跳过控制输出

    // 内环 (速度环)
    Gimbal.pitch_speed_set = (int16_t)(PID_Cal(&Gimbal.pitch_speed_pid, Gimbal.pitch_speed_now, Gimbal.pitch_speed_set) + PITCH_GRAVITY_FEEDFORWARD * pitch_scan_dir);
    DMMotor_Set(DM_CAN_2_2,           // motor_id
                0.0f,                 // pos
                Gimbal.yaw_speed_set, // vel
                0.0f,                 // tor:  前馈力矩，通常设0
                0.0f,                 // kp
                1.5);                 // kd

    DJIMotor_Set(Gimbal.pitch_speed_set, PITCHMotor);
}

/*-------------------- Task --------------------*/

/**
 * @brief          云台任务
 * @param          none
 * @retval         none
 */
void Gimbal_Tasks(void)
{
#if (USE_GIMBAL != 0)
    // 云台数据更新
    Gimbal_Updater();
    // 云台控制解算
    Gimbal_Calculater();
    // 云台电机控制
    Gimbal_Controller();
#endif
}

/*-------------------- Set --------------------*/
/**
 * @brief 设置云台PITCHI轴角度
 *
 * @param angle 云台PITCHI轴角度
 */
void Gimbal_SetPitchAngle(float angle)
{
}

/**
 * @brief 设置云台YAW轴角度
 *
 * @param angle 云台YAW轴角度
 */
void Gimbal_SetYawAngle(float angle)
{
}
