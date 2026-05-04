#include "Chassis.h"
#include "Global_status.h"
#include "robot_param.h"

#include "stm32_time.h"

#include "ramp_generator.h"


RampGenerator Vx_ramp, Vy_ramp, Vw_ramp;

/**
 * @brief 设置底盘水平移动速度
 */
void Chassis_set_x(float x)
{
    RampGenerator_SetTarget(&Vx_ramp, x);
    if (x * RampGenerator_GetCurrent(&Vx_ramp) < 0) // 符号相反
        RampGenerator_SetCurrent(&Vx_ramp, 0.0f);
    // Global.Chssis.input.x = x;
}

/**
 * @brief 设置底盘竖直移动速度
 */
void Chassis_set_y(float y)
{
    RampGenerator_SetTarget(&Vy_ramp, y);
    if (y * RampGenerator_GetCurrent(&Vy_ramp) < 0) // 符号相反
        RampGenerator_SetCurrent(&Vy_ramp, 0.0f);
    // Global.Chssis.input.y = y;
}

/**
 * @brief 设置底盘角速度
 */
void Chassis_set_r(float r)
{
    Global.Chssis.input.r = r;
}

/**
 * @brief 设置斜坡规划器加速度
 */
void Chassis_set_accel(float acc)
{
    RampGenerator_SetAccel(&Vx_ramp, acc);
    RampGenerator_SetAccel(&Vy_ramp, acc);
}
