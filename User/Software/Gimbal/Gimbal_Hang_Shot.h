/*
 * @Date: 2025-09-01 21:10:20
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-03 17:17:25
 * @FilePath: \Season-26-Code\User\Software\Gimbal\Gimbal_Hang_Shot.h
 */
#include "robot_param.h"
#if (GIMBAL_TYPE == GIMBAL_HANG_SHOT)

#ifndef __GIMBAL__HANG_SHOT_H__
#define __GIMBAL__HANG_SHOT_H__

#include "CAN_receive_send.h"
#include "Global_status.h"
#include "motor.h"

// #define gimbal_target_num = ENUM_LENGTH(small_pitch_mode_e);


/*云台电机*/
#define USE_DJIMOTOR_AS_GIMBALMOTOR

#ifdef USE_DJIMOTOR_AS_GIMBALMOTOR
/*函数*/
#define GIMBALMotor_init(type, id)      DJIMotor_init(type, id) 
#define GIMBALMotor_set(val, id)        DJIMotor_set(val, id)
#define GIMBALMotor_get_data(id)        DJIMotor_get_data(id)
#define GIMBALMotor_setzero(angle, id)  DJIMotor_setzero(angle, id)

/*电机参数*/
#define GIMBALMOTOR_MAX_CURRENT MAX_CURRENT

#endif // USE_DJIMOTOR_AS_GIMBALMOTOR


void Gimbal_init();
void Gimbal_control();

#endif // !__GIMBAL_H__
#endif
