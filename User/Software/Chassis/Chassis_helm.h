#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_HELM_WHEEL)

#ifndef __CHASSIS_HELM_H__
#define __CHASSIS_HELM_H__

#include "CAN_receive_send.h"
#include "User_math.h"
#include "motor.h"

#define TURN_FR_ECD 7560
#define TURN_FL_ECD 4097
#define TURN_BL_ECD 5435
#define TURN_BR_ECD 6181

/*电机配置*/
#ifdef USE_DJIMotor

#define CHASSISMotor_init(type, id)  DJIMotor_init(type ,id)
#define CHASSISMotor_set(val, id)    DJIMotor_set(val, id)
#define CHASSISMotor_get_data(id)    DJIMotor_get_data(id)

/*电机参数*/
#define CHASSISMOTOR_MAX_CURRENT DJIMOTOR_MAX_CURRENT
#define CHASSISMOTOR_T_A DJIMOTOR_T_A

#endif //USE_DJMotor

typedef struct 
{
    struct 
    {
        float set_angle;             //舵向电机目标角度
        float set_mangle;            //舵向电机目标编码器值
        float tmp_delta_angle;       //目标角度
        float target_velocity;       //行进电机目标速度
        float set, now, last, offset;
        float stable;
        float imu_set, imu_now, imu_last, imu_offect;
        float turn_speed;
        float set_turn_speed;
        float wheel_current;
    }FL;

    struct 
    {
        float set_angle;             //舵向电机目标角度
        float set_mangle;            //舵向电机目标编码器值
        float tmp_delta_angle;       //目标角度
        float target_velocity;       //行进电机目标速度
        float set, now, last, offset;
        float stable;
        float imu_set, imu_now, imu_last, imu_offect;
        float turn_speed;
        float set_turn_speed;
        float wheel_current;
    }FR;

    struct 
    {
        float set_angle;             //舵向电机目标角度
        float set_mangle;            //舵向电机目标编码器值
        float tmp_delta_angle;       //目标角度
        float target_velocity;       //行进电机目标速度
        float set, now, last, offset;
        float stable;
        float imu_set, imu_now, imu_last, imu_offect;
        float turn_speed;
        float set_turn_speed;
        float wheel_current;
    }BL;

    struct 
    {
        float set_angle;             //舵向电机目标角度
        float set_mangle;            //舵向电机目标编码器值
        float tmp_delta_angle;       //目标角度
        float target_velocity;       //行进电机目标速度
        float set, now, last, offset;
        float stable;
        float imu_set, imu_now, imu_last, imu_offect;
        float turn_speed;
        float set_turn_speed;
        float wheel_current;
    }BR;
    struct 
    {
        float x,y,r,yaw;
        float now_x,now_y,now_r;
        float last_x,last_y,last_r;
        float max_x,max_y,max_w;
    }speed;
    struct 
    {
        float now_x,now_y,now_r;
        float max_x,max_y,max_r;
    }acc;

    
}Chassis_data_s;

/*外部调用*/
void Chassis_init();
void Rudder_Angle_calculation(float x,float y,float w);
void Chassis_updata();
void Chassis_pid_cal();
void Chassis_velocity_calc(float vx,float vy,float vw);
void Chassis_move();
void steer_transfer_nearby();
float obtain_modulus_normalization(float x, float modulus);
void val_limit(float *val, float MAX);

#define CHASSIS_TASK_TIME 1 //底盘任务刷新间隔

#endif
#endif