/**
  * @file       robot_param_omni_infantry.h
  * @brief      这里是舵轮步兵机器人参数配置文件
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

/*------------------------------------------------- Global -----------------------------------------------------------*/

//模块类型选择 (搭积木)
#define CHASSIS_TYPE            CHASSIS_HELM_WHEEL         // 选择底盘类型
#define GIMBAL_TYPE             GIMBAL_YAW_PITCH_DIRECT    // 选择云台类型
#define SHOOT_TYPE              SHOOT_FRIC_TRIGGER         // 选择发射机构类型
#define MECHANICAL_ARM_TYPE     MECHANICAL_ARM_NONE        // 选择机械臂类型
#define CUSTOM_CONTROLLER_TYPE  CUSTOM_CONTROLLER_NONE     // 选择自定义控制器类型

//电机类型选择  （0不使用  1使用）
#define USE_DJIMotor (1)   //大疆电机
#define USE_DMMotor  (0)   //达妙电机
#define USE_LZMotor  (0)   //灵足电机



/*------------------------------------------------- Chassis ----------------------------------------------------------*/

//底盘物理参数
#define r_wheel     (0.26282f)   // (m)轮子半径
#define r_rotation  (0.37169f)   // (m)轮距   
#define WHEEL_RATIO (14.0f)      // 底盘电机减速比 

//电机CAN ID 
//驱动电机
#define WHEEL_move_FL CAN_1_1 //1
#define WHEEL_move_FR CAN_1_2 //2
#define WHEEL_move_BL CAN_1_3 //3
#define WHEEL_move_BR CAN_1_4 //4
//舵向电机
#define WHEEL_turn_FL CAN_1_5 //5
#define WHEEL_turn_FR CAN_1_6 //6
#define WHEEL_turn_BL CAN_1_7 //7
#define WHEEL_turn_BR CAN_1_8 //8

//电机种类
//驱动电机
#define WHEEL_FL_MOVE_MOTOR_TYPE ((Motor_Type_e)DJI_M3508)
#define WHEEL_FR_MOVE_MOTOR_TYPE ((Motor_Type_e)DJI_M3508)
#define WHEEL_BL_MOVE_MOTOR_TYPE ((Motor_Type_e)DJI_M3508)
#define WHEEL_BR_MOVE_MOTOR_TYPE ((Motor_Type_e)DJI_M3508)
//舵向电机
#define WHEEL_FL_TURN_MOTOR_TYPE ((Motor_Type_e)DJI_GM6020)
#define WHEEL_FR_TURN_MOTOR_TYPE ((Motor_Type_e)DJI_GM6020)
#define WHEEL_BL_TURN_MOTOR_TYPE ((Motor_Type_e)DJI_GM6020)
#define WHEEL_BR_TURN_MOTOR_TYPE ((Motor_Type_e)DJI_GM6020)

//小陀螺速度(rpm)
#define R_SPEED_HELM (60)



/*------------------------------------------------- Gimbal -----------------------------------------------------------*/

//云台物理参数
#define PITCHI_MAX_ANGLE (40.0f)      //最大仰角
#define PITCHI_MIN_ANGLE (-20.0f)     //最大俯角
#define YAW_RATIO   (1)               //yaw轴电机减速比
#define PITCH_RATIO (1)               //pitch轴电机减速比

//电机CAN ID
#define YAWMotor   CAN_2_6
#define PITCHMotor CAN_2_5

//电机种类
#define GIMBAL_YAW_MOTOR_TYPE   ((Motor_Type_e)DJI_GM6020)
#define GIMBAL_PITCH_MOTOR_TYPE ((Motor_Type_e)DJI_GM6020)

//电机方向 (旋转方向)
#define GIMBAL_YAW_DIRECTION   (1)
#define GIMBAL_PITCH_DIRECTION (1)

//电机零点设置
#define YAW_ZERO   (102.5f)
#define PITCH_ZERO (73.8f)



/*------------------------------------------------- Shoot -------------------------------------------------------------*/

//发射机构物理参数
#define FRIC_RADIUS 0.03f              // (m)摩擦轮半径
#define BULLET_NUM 12                  // 拨弹盘容纳弹丸个数

//电机ID
#define ShootMotor_L  CAN_2_2
#define ShootMotor_R  CAN_2_4
#define TRIGGER_MOTOR CAN_2_7

//电机种类
#define TRIGGER_MOTOR_TYPE  ((Motor_Type_e)DJI_M2006)
#define SHOOT_MOTOR_TYPE    ((Motor_Type_e)DJI_M3508)

//拨弹速度
#define TRIGGER_SPEED_H     (4000) //高射频
#define TRIGGER_SPEED_M     (3000) //中射频
#define TRIGGER_SPEED_L     (1000) //低射频

//摩擦轮速度
#define FRIC_SPEED_BEGIN    (2000)  //开始反转
#define FRIC_SPEED_REDAY    (-6000) //正常工作值
#define FRIC_SPEED_DEBUG    (-1500) //退弹低速值


#endif /* INCLUDED_ROBOT_PARAM_H */
