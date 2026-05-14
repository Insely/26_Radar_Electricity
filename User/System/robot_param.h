/**
  * @file       robot_param_omni_infantry.h
  * @brief      这里是舵轮步兵机器人参数配置文件
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H

/*------------------------------------------------- Global -----------------------------------------------------------*/

//模式选择      （0不使用  1使用）
#define CONTROL_TYPE     (1)   // 控制模式
#define DEBUG_TYPE       (0)   // 调试模式模式    

//模块类型选择  （0不使用  1使用）
#define USE_CHASSIS_HELM       (1)   // 启用舵轮底盘
#define USE_GIMBAL             (1)   // 启用云台
#define USE_SHOOT              (1)   // 启用发射机构

//电机类型选择  （0不使用  1使用）
#define USE_DJIMotor    (1)   // 大疆电机
#define USE_DMMotor     (1)   // 达妙电机
#define USE_DMMotor124  (0)   // 达妙电机1拖4
#define USE_LZMotor     (0)   // 灵足电机


/*------------------------------------------------- Chassis ----------------------------------------------------------*/
  #define WHEEL_RADIUS     (0.04050f)   // (m)轮子半径
  #define WHEEL_TRACK         (0.2262741699f)  // (m)轮组到旋转中心的距离        
  #define WHEEL_RATIO (15.8f)     // 底盘电机减速比

  #define R_body 0.273  // 底盘安装半径（单位：米，根据实际值修改）
  #define r_wheel 0.077 // 轮子半径（单位：米，根据实际值修改）
  #define WHEEL_RATIO (15.8f)     // 底盘电机减速比
 
  #define WHEEL_MOVE_FR CAN_3_1
  #define WHEEL_MOVE_FL CAN_3_2
  #define WHEEL_MOVE_BR CAN_1_1
  #define WHEEL_MOVE_BL CAN_1_2

  #define WHEEL_TURN_FR CAN_3_5
  #define WHEEL_TURN_FL CAN_3_6
  #define WHEEL_TURN_BR CAN_1_5
  #define WHEEL_TURN_BL CAN_1_6

//小陀螺速度(rpm)
#define R_SPEED_HELM (60)



/*------------------------------------------------- Gimbal -----------------------------------------------------------*/

//云台物理参数
#define PITCHI_MAX_ANGLE (45.0f)      // 最大仰角
#define PITCHI_MIN_ANGLE (0.0f)      // 最大俯角 
#define YAW_RATIO   (1)               // yaw轴电机减速比
#define PITCH_RATIO (1)               // pitch轴电机减速比

//电机CAN ID
#define YAWMotor   DM_CAN_2_2
#define PITCHMotor CAN_2_1

//电机种类
#define GIMBAL_YAW_MOTOR_TYPE   ((Motor_Type_e)DM_4310)
#define GIMBAL_PITCH_MOTOR_TYPE ((Motor_Type_e)DJI_M3508)

//电机方向 (旋转方向)
#define GIMBAL_YAW_DIRECTION   (1)
#define GIMBAL_PITCH_DIRECTION (1)

//电机零点设置
#define YAW_ZERO   (102.5f)
#define PITCH_ZERO (-115.58f)



/*------------------------------------------------- Shoot -------------------------------------------------------------*/

//发射机构物理参数
#define FRIC_RADIUS 0.03f              // (m)摩擦轮半径
#define BULLET_NUM 12                  // 拨弹盘容纳弹丸个数

//电机ID
#define ShootMotor_L  CAN_2_3
#define ShootMotor_R  CAN_2_4
#define TRIGGER_MOTOR CAN_1_3

//电机种类
#define TRIGGER_MOTOR_TYPE  ((Motor_Type_e)DJI_M2006)
#define SHOOT_MOTOR_TYPE    ((Motor_Type_e)DJI_M3508)

//拨弹速度
#define TRIGGER_SPEED_H     (9000) // 高射频
#define TRIGGER_SPEED_M     (3000) // 中射频
#define TRIGGER_SPEED_L     (1000) // 低射频

//摩擦轮速度
#define FRIC_SPEED_BEGIN    (-2000)  // 开始反转
#define FRIC_SPEED_REDAY    (6000) // 正常工作值
#define FRIC_SPEED_DEBUG    (1500) // 退弹低速值


/*------------------------------------------------- REMOTE -------------------------------------------------------------*/

//灵敏度
#define MOVE_SENSITIVITY 10.0f   // 移动灵敏度
#define PITCH_SENSITIVITY 0.004f // pitch轴灵敏度
#define YAW_SENSITIVITY 0.005f     // yaw轴灵敏度

/*------------------------------------------------- MOTOR -------------------------------------------------------------*/

// 可用电机类型
typedef enum __MotorType {
    DJI_M2006 = 0,       // 大疆M2006
    DJI_M3508,           // 大疆M3508
    DJI_GM6020,          // 大疆GM6020
    DM_4310,             // 达妙4310
    LZ_00,               // 灵足00
} Motor_Type_e;

#endif /* INCLUDED_ROBOT_PARAM_H */
