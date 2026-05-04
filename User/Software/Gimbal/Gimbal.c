#include "Gimbal.h"
#include "Global_status.h"
#include "robot_param.h"


Gimbal_t Gimbal;

/**
 * @brief 设置云台PITCHI轴角度
 *
 * @param angle 云台PITCHI轴角度
 */
void Gimbal_set_pitch_angle(float angle)
{
    if (angle < PITCHI_MIN_ANGLE)
        angle = PITCHI_MIN_ANGLE;
    if (angle > PITCHI_MAX_ANGLE)
        angle = PITCHI_MAX_ANGLE;
    Global.Gimbal.input.pitch = angle;
}

/**
 * @brief 设置云台YAW轴角度
 *
 * @param angle 云台YAW轴角度
 */
void Gimbal_set_yaw_angle(float angle)
{
    Global.Gimbal.input.yaw = angle;
}

void Gimbal_update(DJI_motor_data_s DJIMotor_data[QUANTITY_OF_CAN][QUANTITY_OF_DJIMOTOR],Gimbal_t *motor_data){

    motor_data->yaw_speed_now=DJIMotor_data[GIMBAL_YAW_MOTOR_TYPE/11][GIMBAL_YAW_MOTOR_TYPE%11].round_speed;
    motor_data->pitch_speed_now=DJIMotor_data[GIMBAL_PITCH_MOTOR_TYPE/11][GIMBAL_PITCH_MOTOR_TYPE%11].round_speed;

}