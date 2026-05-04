#include "robot_param.h"
#if (SHOOT_TYPE == SHOOT_FRIC_TRIGGER)

#ifndef __SHOOT_FRIC_TIGGER_H__
#define __SHOOT_FRIC_TIGGER_H__

#include "CAN_receive_send.h"

#include "motor.h"
/*发射机构参数*/
enum shoot_speed_e // 摩擦轮速度
{
    SHOOT_SPEED_BEGIN = FRIC_SPEED_BEGIN,//开始反转值
    SHOOT_SPEED_CLOSE = 0,    //停止速度值
    SHOOT_SPEED_READY = FRIC_SPEED_REDAY,  //正常工作值
    SHOOT_SPEED_DEBUG = FRIC_SPEED_DEBUG, //退弹低速值
};
extern enum shoot_speed_e shoot_speed;

enum trigger_speed_e // 拨弹电机速度
{
    TRIGGER_SPEED_CLOSE = 0,
    TRIGGER_SPEED_HIGH  = TRIGGER_SPEED_H,
    TRIGGER_SPEED_MID   = TRIGGER_SPEED_M,
    TRIGGER_SPEED_LOW   = TRIGGER_SPEED_L,
    TRIGGER_SPEED_DEBUG = 3000,
};
extern enum trigger_speed_e trigger_speed;

/* 内部调用 */
#define SHOOTMotor_init(type, id)    DJIMotor_init(type, id)
#define SHOOTMotor_set(val, id)      DJIMotor_set(val, id)
#define TriggerMotor_init(type, id)  DJIMotor_init(type, id)
#define TriggerMotor_set(val, id)    DJIMotor_set(val, id)
#define TriggerMotor_get_data(id)    DJIMotor_get_data(id)
#define SHOOTMotor_get_data(id)      DJIMotor_get_data(id)

/* 电机参数 */
#define SHOOTMOTOR_MAX_CURRENT MAX_CURRENT

/* 外部调用 */
void Shoot_init();
void Shoot_task();

#endif 
#endif