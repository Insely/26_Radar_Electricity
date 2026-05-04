#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "Gimbal_Direct.h"
#include "Gimbal_Hang_Shot.h"
#include "pid.h"

void Gimbal_set_pitch_angle(float angle);
void Gimbal_set_yaw_angle(float angle);

typedef struct
{
    /*-------PID-------*/
    pid_t pitch_speed_pid;
    pid_t pitch_location_pid;
    pid_t yaw_speed_pid;
    pid_t yaw_location_pid;
    pid_t pitch_auto_speed_pid;
    pid_t pitch_auto_location_pid;
    pid_t yaw_auto_speed_pid;
    pid_t yaw_auto_location_pid;

    /*-------状态量-------*/
    //速度
    float yaw_speed_now;
    float pitch_speed_now;
    //位置 
    float yaw_location_now;
    float pitch_location_now;

    /*-------目标量-------*/
    //速度
    float yaw_speed_set;
    float pitch_speed_set;
    //位置 
    float yaw_location_set;
    float pitch_location_set; 

}Gimbal_t;

extern Gimbal_t Gimbal;

#endif