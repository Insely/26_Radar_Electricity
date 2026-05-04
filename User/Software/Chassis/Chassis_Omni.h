#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)

#ifndef __CHASSIS_OMNI_H__
#define __CHASSIS_OMNI_H__

#include "CAN_receive_send.h"

#include "User_math.h"

#include "motor.h"

//---------------------------
// 逆运动学宏定义（速度 → 轮速）（M_S → RAD_S）
//---------------------------
// 输入：底盘速度 vx, vy, 角速度 omega
// 输出：四个轮子的转速（浮点型）

// 前左轮 FL 转速计算单位
#define IK_WHEEL_FL(vx, vy, omega) \
    ((SQRT1_2 * (vy) + SQRT1_2 * (vx) + (R_body) * (omega)) / r_wheel)

// 前右轮 FR 转速计算
#define IK_WHEEL_FR(vx, vy, omega) \
    ((-SQRT1_2 * (vy) + SQRT1_2 * (vx) + (R_body) * (omega)) / r_wheel)

// 后左轮 BL 转速计算
#define IK_WHEEL_BL(vx, vy, omega) \
    ((+SQRT1_2 * (vy) - SQRT1_2 * (vx) + (R_body) * (omega)) / r_wheel)

// 后右轮 BR 转速计算
#define IK_WHEEL_BR(vx, vy, omega) \
    ((-SQRT1_2 * (vy) - SQRT1_2 * (vx) + (R_body) * (omega)) / r_wheel)

//---------------------------
// 正运动学宏定义（轮速 → 速度）（RAD_S → M_S）
//---------------------------
// 输入：四个轮速 wFL, wFR, wBL, wBR
// 输出：底盘速度 vx, vy, 角速度 omega（通过指针返回）


// 计算 vx
#define FK_VX(wFL, wFR, wBL, wBR) \
    (((-(wBR) + (wFR) + (wFL) - (wBL)) * SQRT2 * r_wheel) / 4.0f)

// 计算 vy
#define FK_VY(wFL, wFR, wBL, wBR) \
    ((((wFL) - (wBR) - (wFR) + (wBL)) * SQRT2 * r_wheel) / 4.0f)

// 计算 omega（角速度）
#define FK_OMEGA(wFL, wFR, wBL, wBR) \
    ((((wFL) + (wFR) + (wBL) + (wBR)) * r_wheel) / (4.0f * R_body))



//---------------------------
// 逆动力学宏定义（底盘驱动力 → 轮毂力矩）（F → T）
//---------------------------
// 输入：底盘驱动力 Fx, Fy, 旋转力矩   T
// 输出：四个轮子的力矩（浮点型）

// 前左轮 FL 力矩计算
#define IK_WHEEL_FL_T(Fx, Fy, T) \
    ((SQRT2 * (Fy) + SQRT2 * (Fx) + (T) / 4.0f / (R_body)) * r_wheel)

// 前右轮 FR 力矩计算
#define IK_WHEEL_FR_T(Fx, Fy, T) \
    ((-SQRT2 * (Fy) + SQRT2 * (Fx) + (T) / 4.0f / (R_body)) * r_wheel)

// 后左轮 BL 力矩计算
#define IK_WHEEL_BL_T(Fx, Fy, T) \
    ((+SQRT2 * (Fy) - SQRT2 * (Fx) + (T) / 4.0f / (R_body)) * r_wheel)

// 后右轮 BR 力矩计算
#define IK_WHEEL_BR_T(Fx, Fy, T) \
    ((-SQRT2 * (Fy) - SQRT2 * (Fx) + (T) / 4.0f / (R_body)) * r_wheel)


/*电机配置*/
#ifdef USE_DJIMotor

#define CHASSISMotor_init(type, id)  DJIMotor_init(type ,id)
#define CHASSISMotor_set(val, id)    DJIMotor_set(val, id)
#define CHASSISMotor_get_data(id)    DJIMotor_get_data(id)

/*电机参数*/
#define CHASSISMOTOR_MAX_CURRENT MAX_CURRENT
#define CHASSISMOTOR_T_A DJIMOTOR_T_A

#endif // USE_DJIMotor

/*内部数据类型*/
typedef struct
{
    /* data */
    
}Chassis_t;

/*外部调用*/
void Chassis_init();
void Chassis_move();

#define CHASSIS_TASK_TIME 1 // 底盘任务刷新间隔

#endif
#endif 
