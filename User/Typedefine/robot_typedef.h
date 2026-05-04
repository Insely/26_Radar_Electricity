/*
 * @Date: 2025-08-31 21:44:10
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-03 15:37:55
 * @FilePath: \Season-26-Code\User\Typedefine\robot_typedef.h
 */
#ifndef ROBOT_TYPEDEF_H
#define ROBOT_TYPEDEF_H


// 可用底盘硬件类型
#define CHASSIS_NONE            0  // 无底盘
#define CHASSIS_MECANUM_WHEEL   1  // 麦克纳姆轮底盘
#define CHASSIS_OMNI_WHEEL      2  // 全向轮底盘
#define CHASSIS_HELM_WHEEL      3  // 舵轮底盘
#define CHASSIS_BALANCE         4  // 平衡底盘


// 可用云台硬件类型
#define GIMBAL_NONE                  0  // 无云台
#define GIMBAL_YAW_PITCH_DIRECT      1  // yaw_piych直连云台
#define GIMBAL_HANG_SHOT             2  // 吊射云台
#define GIMBAL_DOUBLE_YAW            3  // 双yaw云台


// 可用的发射机构硬件类型
#define SHOOT_NONE               0  // 无发射机构
#define SHOOT_FRIC_TRIGGER       1  // 摩擦轮+拨弹盘发射机构
#define SHOOT_THREE_FRIC          2  // 三摩擦发射机构


// 可用机械臂硬件类型
#define MECHANICAL_ARM_NONE              0  // 无机械臂
#define MECHANICAL_ARM_ENGINEER_ARM      1  // 工程机械臂


// 可用自定义控制器硬件类型
#define CUSTOM_CONTROLLER_NONE         0  // 无自定义控制器
#define CUSTOM_CONTROLLER_ENGINEER     1  // 工程自定义控制器


// 控制类型
#define CHASSIS_ONLY          0  // 只控制底盘
#define GIMBAL_ONLY           1  // 只控制云台
#define CHASSIS_AND_GIMBAL    2  // 控制底盘和云台


// 遥控器类型
#define RC_DT7      0  // DT7遥控器
#define RC_VT13     1  // VT13遥控器（图传）
#define RC_FSI6X    2  // FSI6X遥控器


// 自瞄通信选择
#define Auto_NONE         0x000 // 无自瞄
#define Auto_ENTITY       0x001 // 实体串口
#define Auto_VIRTUAL      0x002 // 虚拟串口


// 可用电机类型
typedef enum __MotorType {
    DJI_M2006 = 0,
    DJI_M3508,
    DJI_GM6020,
    DM_4310,
    LZ_00,
} Motor_Type_e;


#endif /* ROBOT_TYPEDEF_H */
/*------------------------------ End of File ------------------------------*/