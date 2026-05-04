#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_HELM_WHEEL)

#include "Chassis_helm.h"
#include "Chassis.h"
#include "Global_status.h"
#include "Gimbal.h"

#include "referee_system.h"
#include "supercup.h"
#include "stm32_time.h"
#include "IMU_updata.h"

#include "pid.h"
#include "ramp_generator.h"
#include "User_math.h"
#include "remote_control.h"

pid_t motor_speed_3508_FL, motor_speed_3508_FR, motor_speed_3508_BL, motor_speed_3508_BR;             // 行进电机速度环pid
pid_t motor_location_6020_FL, motor_location_6020_FR, motor_location_6020_BL, motor_location_6020_BR; // 舵向电机位置环pid
pid_t motor_speed_6020_FL, motor_speed_6020_FR, motor_speed_6020_BL, motor_speed_6020_BR;             // 舵向电机速度环pid
float X_AXIS_ECD_FL=4096, X_AXIS_ECD_FR=7600, X_AXIS_ECD_BL=2766, X_AXIS_ECD_BR=6190;                 // 轮子向前时6020编码器的值
float dirt_FL = -1.0f, dirt_FR = 1.0f, dirt_BL = 1.0f, dirt_BR = -1.0f;                               // 3508电机转向设置
static fp32 deta[4] = {45.0f,45.0f,45.0f,45.0f};                                                      //计算转向时用到的角度
Chassis_data_s chassis;
float Vx_now, Vy_now, W_now;
extern RC_ctrl_t RC_data;

void Chassis_init()
{
    //行进电机初始化
    CHASSISMotor_init(WHEEL_FL_MOVE_MOTOR_TYPE,WHEEL_move_FL);
    CHASSISMotor_init(WHEEL_FR_MOVE_MOTOR_TYPE,WHEEL_move_FR);
    CHASSISMotor_init(WHEEL_BL_MOVE_MOTOR_TYPE,WHEEL_move_BL);
    CHASSISMotor_init(WHEEL_BR_MOVE_MOTOR_TYPE,WHEEL_move_BR);

    //舵向电机初始化
    CHASSISMotor_init(WHEEL_FL_TURN_MOTOR_TYPE,WHEEL_turn_FL);
    CHASSISMotor_init(WHEEL_FR_TURN_MOTOR_TYPE,WHEEL_turn_FR);
    CHASSISMotor_init(WHEEL_BL_TURN_MOTOR_TYPE,WHEEL_turn_BL);
    CHASSISMotor_init(WHEEL_BR_TURN_MOTOR_TYPE,WHEEL_turn_BR);

    //舵向电机参数初始化
    chassis.FL.now = 0;
    chassis.FL.set = TURN_FL_ECD;
    chassis.FL.offset = 0;
    chassis.FL.stable = 0 ;
    chassis.FL.set_turn_speed = 0;

    chassis.FR.now = 0;
    chassis.FR.set = TURN_FR_ECD;
    chassis.FR.offset = 0;
    chassis.FR.stable = 0 ;
    chassis.FR.set_turn_speed = 0; 
    
    chassis.BL.now = 0;
    chassis.BL.set = TURN_BL_ECD;
    chassis.BL.offset = 0;
    chassis.BL.stable = 0 ;
    chassis.BL.set_turn_speed = 0; 
    
    chassis.BR.now = 0;
    chassis.BR.set = TURN_BR_ECD;
    chassis.BR.offset = 0;
    chassis.BR.stable = 0 ;
    chassis.BR.set_turn_speed = 0;

    /*旋转速度分解的角度deta*/
    for(int i=0;i<4;i++)
    {
        deta[i] = deta[i]*PI/180.0F;   //角度转弧度
    }
} 

/**
 * @brief 舵向角度解算
 *
 */
void Rudder_Angle_calculation(float x,float y,float w)
{
    //线速度
    w = w * r_rotation;

    //旋转运动
    if(x==0&&y==0&&w==0)
    {
        chassis.FL.set_angle = 0.0f;
        chassis.FR.set_angle = 0.0f;
        chassis.BL.set_angle = 0.0f;
        chassis.BR.set_angle = 0.0f;
    }
    else
    {
        chassis.FL.set_angle = atan2((y + w * 0.707107f), (x - w * 0.707107f)) * 180.0f / PI;
        chassis.FR.set_angle = atan2((y + w * 0.707107f), (x - w * 0.707107f)) * 180.0f / PI;
        chassis.BL.set_angle = atan2((y + w * 0.707107f), (x - w * 0.707107f)) * 180.0f / PI;
        chassis.BR.set_angle = atan2((y + w * 0.707107f), (x - w * 0.707107f)) * 180.0f / PI;
    }

    chassis.FL.set_mangle = X_AXIS_ECD_FL + chassis.FL.set_angle * 8192.0 / 360.0;
    chassis.FR.set_mangle = X_AXIS_ECD_FR + chassis.FR.set_angle * 8192.0 / 360.0;
    chassis.BL.set_mangle = X_AXIS_ECD_BL + chassis.BL.set_angle * 8192.0 / 360.0;
    chassis.BR.set_mangle = X_AXIS_ECD_BR + chassis.BR.set_angle * 8192.0 / 360.0;

    //角度赋值 设置需要的角度对应在编码器上的位置
    chassis.FL.set = (int32_t)chassis.FL.set_mangle;
    chassis.FR.set = (int32_t)chassis.FR.set_mangle;
    chassis.BL.set = (int32_t)chassis.BL.set_mangle;
    chassis.BR.set = (int32_t)chassis.BR.set_mangle;
}

/**
 * @brief 舵向电机数据更新
 *
 */
void Chassis_updata()
{
    //编码器赋值
    chassis.FL.now = CHASSISMotor_get_data(WHEEL_turn_FL).ecd;
    chassis.FR.now = CHASSISMotor_get_data(WHEEL_turn_FR).ecd;
    chassis.BL.now = CHASSISMotor_get_data(WHEEL_turn_BL).ecd;
    chassis.BR.now = CHASSISMotor_get_data(WHEEL_turn_BR).ecd;
}

/**
 * @brief 舵向电机PID计算
 *
 */
void Chassis_pid_cal()
{
    //就近转位
    steer_transfer_nearby();
    chassis.FL.turn_speed = pid_cal(&motor_location_6020_FL,chassis.FL.now,chassis.FL.set);
    chassis.FR.turn_speed = pid_cal(&motor_location_6020_FR,chassis.FR.now,chassis.FR.set);
    chassis.BL.turn_speed = pid_cal(&motor_location_6020_BL,chassis.BL.now,chassis.BL.set);
    chassis.BR.turn_speed = pid_cal(&motor_location_6020_BR,chassis.BR.now,chassis.BR.set);
    
    //电流计算
    CHASSISMotor_set(pid_cal(&motor_speed_6020_FL,CHASSISMotor_get_data(WHEEL_turn_FL).speed_rpm,chassis.FL.turn_speed),WHEEL_turn_FL);
    CHASSISMotor_set(pid_cal(&motor_speed_6020_FR,CHASSISMotor_get_data(WHEEL_turn_FR).speed_rpm,chassis.FR.turn_speed),WHEEL_turn_FR);
    CHASSISMotor_set(pid_cal(&motor_speed_6020_BL,CHASSISMotor_get_data(WHEEL_turn_BL).speed_rpm,chassis.BL.turn_speed),WHEEL_turn_BL);
    CHASSISMotor_set(pid_cal(&motor_speed_6020_BR,CHASSISMotor_get_data(WHEEL_turn_BR).speed_rpm,chassis.BR.turn_speed),WHEEL_turn_BR);

}

/**
 * @brief 行进电机速度电流计算
 *
 */
void Chassis_velocity_calc(float vx,float vy,float vw)
{
    //计算和速度
    vw = vw * r_rotation;
    float V_FL,V_FR,V_BL,V_BR;
    
    /*和速度-->分速度*/
    V_FL = sqrt((-vx - vw * sinf(deta[1])) * (-vx - vw * sinf(deta[1])) + (-vy + vw * cosf(deta[1])) * (-vy + vw * cosf(deta[1])));
    V_FR = sqrt((vx + vw * sinf(deta[0])) * (vx + vw * sinf(deta[0])) + (vy + vw * cosf(deta[0])) * (vy + vw * cosf(deta[0])));
    V_BL = sqrt((vx - vw * sinf(deta[2])) * (vx - vw * sinf(deta[2])) + (vy - vw * cosf(deta[2])) * (vy - vw * cosf(deta[2])));
    V_BR = sqrt((vx + vw * sinf(deta[3])) * (vx + vw * sinf(deta[3])) + (vy - vw * cosf(deta[3])) * (vy - vw * cosf(deta[3])));

    //确定目标速度的正负
    chassis.FL.target_velocity = V_FL * dirt_FL;
    chassis.FR.target_velocity = V_FR * dirt_FR;
    chassis.BL.target_velocity = V_BL * dirt_BL;
    chassis.BR.target_velocity = V_BR * dirt_BR;

    //最大速度限制
    val_limit(&vx,chassis.speed.max_x);
    val_limit(&vy,chassis.speed.max_y);
    val_limit(&vw,chassis.speed.max_w);

    //计算马达电流
    chassis.FL.wheel_current = pid_cal(&motor_speed_3508_FL,CHASSISMotor_get_data(WHEEL_move_FL).round_speed * r_wheel * PI,chassis.FL.target_velocity);
    chassis.FR.wheel_current = pid_cal(&motor_speed_3508_FR,CHASSISMotor_get_data(WHEEL_move_FR).round_speed * r_wheel * PI,chassis.FR.target_velocity);
    chassis.BL.wheel_current = pid_cal(&motor_speed_3508_BL,CHASSISMotor_get_data(WHEEL_move_BL).round_speed * r_wheel * PI,chassis.BL.target_velocity);
    chassis.BR.wheel_current = pid_cal(&motor_speed_3508_BR,CHASSISMotor_get_data(WHEEL_move_BR).round_speed * r_wheel * PI,chassis.BR.target_velocity);

    //发送马达电流
    CHASSISMotor_set(chassis.FL.wheel_current,WHEEL_move_FL);
    CHASSISMotor_set(chassis.FR.wheel_current,WHEEL_move_FR);
    CHASSISMotor_set(chassis.BL.wheel_current,WHEEL_move_BL);
    CHASSISMotor_set(chassis.BR.wheel_current,WHEEL_move_BR);
}

/**
 * @brief 就近转位
 *
 */
void steer_transfer_nearby()
{
    chassis.FL.tmp_delta_angle = obtain_modulus_normalization(chassis.FL.set - chassis.FL.now,8192.0f);
    chassis.FR.tmp_delta_angle = obtain_modulus_normalization(chassis.FR.set - chassis.FR.now,8192.0f);
    chassis.BL.tmp_delta_angle = obtain_modulus_normalization(chassis.BL.set - chassis.BL.now,8192.0f);
    chassis.BR.tmp_delta_angle = obtain_modulus_normalization(chassis.BR.set - chassis.BR.now,8192.0f);

    //根据转动角度范围决定是否需要就近转位 FL
    if(-2048.0f <= chassis.FL.tmp_delta_angle&&chassis.FL.tmp_delta_angle <= 2048.0f)
    {
        //±PI / 2之间无需反向就近转位
        chassis.FL.set = chassis.FL.tmp_delta_angle + chassis.FL.now;
        dirt_FL = -1.0f;
    }
    else
    {
        //需要反转扣圈的情况
        chassis.FL.set = obtain_modulus_normalization(chassis.FL.tmp_delta_angle + 4096.0f,8192.0f) + chassis.FL.now;
        dirt_FL = 1.0f;
    }
    
    //根据转动角度范围决定是否需要就近转位 FR
    if(-2048.0f <= chassis.FR.tmp_delta_angle&&chassis.FR.tmp_delta_angle <= 2048.0f)
    {
        //±PI / 2之间无需反向就近转位
        chassis.FR.set = chassis.FR.tmp_delta_angle + chassis.FR.now;
        dirt_FR = -1.0f;
    }
    else
    {
        //需要反转扣圈的情况
        chassis.FR.set = obtain_modulus_normalization(chassis.FR.tmp_delta_angle + 4096.0f,8192.0f) + chassis.FR.now;
        dirt_FR = 1.0f;
    }
    
    //根据转动角度范围决定是否需要就近转位 BL
    if(-2048.0f <= chassis.BL.tmp_delta_angle&&chassis.BL.tmp_delta_angle <= 2048.0f)
    {
        //±PI / 2之间无需反向就近转位
        chassis.BL.set = chassis.BL.tmp_delta_angle + chassis.BL.now;
        dirt_BL = -1.0f;
    }
    else
    {
        //需要反转扣圈的情况
        chassis.BL.set = obtain_modulus_normalization(chassis.BL.tmp_delta_angle + 4096.0f,8192.0f) + chassis.BL.now;
        dirt_BL = 1.0f;
    }
    
    //根据转动角度范围决定是否需要就近转位 BR
    if(-2048.0f <= chassis.BR.tmp_delta_angle&&chassis.BR.tmp_delta_angle <= 2048.0f)
    {
        //±PI / 2之间无需反向就近转位
        chassis.BR.set = chassis.BR.tmp_delta_angle + chassis.BR.now;
        dirt_BR = -1.0f;
    }
    else
    {
        //需要反转扣圈的情况
        chassis.BR.set = obtain_modulus_normalization(chassis.BR.tmp_delta_angle + 4096.0f,8192.0f) + chassis.BR.now;
        dirt_BR = 1.0f;
    }
}
void Chassis_move()
{
    Rudder_Angle_calculation(RC_data.rc.ch[1] / 2.0f, -RC_data.rc.ch[0] / 2.0f,  -RC_data.rc.ch[2] / 5.0f);
    Chassis_updata();
    Chassis_pid_cal();
    Chassis_velocity_calc(RC_data.rc.ch[1] / 2.0f, -RC_data.rc.ch[0] / 2.0f,  -RC_data.rc.ch[2] / 5.0f);
}

/**
 * @brief 获取底盘pitch角度数据，需pitch电机精确设置零点，仅能在云台底盘同方向的时候使用。
 *
 * @return float
 */
/* float Chassis_angle_getpitch()
{
    return GIMBALMotor_get_data(PITCHMotor).angle + IMU_data.AHRS.pitch * RAD_TO_DEG;
}*/

/**
 * @brief 求取模归化  转动角度控制在-PI----PI
 *
 * @param x
 * @param modulus
 * @return float
 */
float obtain_modulus_normalization(float x, float modulus)
{
    float tmp;
    tmp = fmod(x + modulus / 2.0f, modulus);
    if (tmp < 0.0f)
    {
        tmp += modulus;
    }
    return (tmp - modulus / 2.0f);
}


/**
 * @brief 限制值
 *
 * @param val
 * @param MAX
 */
void val_limit(float *val, float MAX)
{
    if (fabs(*val) > MAX)
    {
        if (*val > 0)
            *val = MAX;
        else
            *val = -MAX;
    }
}

#endif 