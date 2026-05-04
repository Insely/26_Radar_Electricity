/*
 * @Date: 2025-09-21 18:40:10
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-04 08:52:50
 * @FilePath: \Season-26-Code\User\Software\Gimbal\Gimbal_Direct.h
 */
#include "robot_param.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)

#ifndef __GIMBAL_DIRECT_H__
#define __GIMBAL_DIRECT_H__

#include "CAN_receive_send.h"

#include "motor.h"

/*ï¿½ï¿½ï¿½ï¿½*/
#define GIMBALMotor_init(type, id)      DJIMotor_init(type, id)
#define GIMBALMotor_set(val, id)        DJIMotor_set(val, id)
#define GIMBALMotor_get_data(id)        DJIMotor_get_data(id)
#define GIMBALMotor_setzero(angle, id)  DJIMotor_setzero((double)(angle), id)

/*ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?*/
#define GIMBALMOTOR_MAX_CURRENT MAX_6020_VOL

void Gimbal_init();
void Gimbal_control();

#endif
#endif
