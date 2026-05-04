#include "robot_param.h"
#if (GIMBAL_TYPE == GIMBAL_HANG_SHOT)

#include "Gimbal_Hang_Shot.h"
#include "Global_status.h"

#include "User_math.h"
#include "pid.h"
#include "ramp_generator.h"

#include "IMU_updata.h"
#include "Servos.h"

uint8_t ui_shoot_gimbal;
// 手控
pid_t pitch_speed_pid, pitch_location_pid;
pid_t yaw_speed_pid, yaw_location_pid;
// 自喵
pid_t pitch_auto_speed_pid, pitch_auto_location_pid;
pid_t yaw_auto_speed_pid, yaw_auto_location_pid;
// pitch自锁
pid_t pitch_lock_speed_pid, pitch_lock_location_pid;
// yaw自锁
pid_t yaw_lock_speed_pid, yaw_lock_location_pid;
/*模式切换间的数据处理*/
float Setzero_imu2ecd = 0.0f; // 因为吊射模式和普通模式切换时候，
float imu2ecd_err = 0.0f;     // yaw有误差，通过这两个值补偿
float Setzero_ecd2imu = 0.0f, ecd2imu_err = 0.0f;

/*斜坡*/
RampGenerator pitch_ramp, yaw_ramp;

float Hanging_Shot_offset() // 吊射模式時云台与底盘的角度差
{
    Global.Gimbal.Hanging_Shot_err = GIMBALMotor_get_data(YAWMotor).angle + 90;

    uint32_t mul;
    mul = fabs(Global.Gimbal.Hanging_Shot_err) / 180.0f;
    if (Global.Gimbal.Hanging_Shot_err > 180.0f)
    {
        if (mul % 2 == 1) // 处于-180度
            Global.Gimbal.Hanging_Shot_err -= (mul + 1) * 180.0f;
        else // 处于180度
            Global.Gimbal.Hanging_Shot_err -= mul * 180.0f;
    }
    if (Global.Gimbal.Hanging_Shot_err < -180.0f)
    {
        if (mul % 2 == 1) // 处于180度
            Global.Gimbal.Hanging_Shot_err += (mul + 1) * 180.0f;
        else // 处于-180度
            Global.Gimbal.Hanging_Shot_err += mul * 180.0f;
    }
}

/**
 * @brief 云台初始化
 *
 */
void Gimbal_init()
{
    Global.Gimbal.pitch_mode == Pitch_disability; // 默认上电pitch电机未初始化
    /*电机初始化*/
    GIMBALMotor_init(GIMBAL_YAW_MOTOR_TYPE, YAWMotor);
    GIMBALMotor_init(GIMBAL_PITCH_MOTOR_TYPE, PITCHMotor);

     // 遥控，吊射用位置，平时用速度
    // pid_set(&pitch_speed_pid, 50.0f, 0.0f, 0.0f, 5000, 0);
    pid_set(&pitch_speed_pid, 35.0f, 0.0f, 5.0f, 8000, 0);
    pid_set(&pitch_location_pid, 200.0f, 0.0f, 5.0f, 5000, 0);

    pid_set(&yaw_speed_pid, 350.0f, 0.0f, 280.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    pid_set(&yaw_location_pid, 10.0f, 0.0f, 7.0f, GIMBALMOTOR_MAX_CURRENT, 10);

    pid_set(&yaw_lock_speed_pid, 450.0f, 0.0f, 300.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    pid_set(&yaw_lock_location_pid, 12.0f, 0.0f, 10.0f, GIMBALMOTOR_MAX_CURRENT, 10);

    pid_set(&pitch_lock_speed_pid, 13.0f, 0, 5, GIMBALMOTOR_MAX_CURRENT, 1000);
    pid_set(&pitch_lock_location_pid, 1.0f, 0.0f, 0, GIMBALMOTOR_MAX_CURRENT, 100);

    pid_set(&yaw_lock_speed_pid, 12.0f, 0, 40.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    pid_set(&yaw_lock_location_pid, 350.0f, 0.00f, 300.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    // 自瞄
    pid_set(&pitch_auto_speed_pid, 300.0f, 0, 100, 8000, 1000);
    pid_set(&pitch_auto_location_pid, 10.0f, 0.001f, 5, 5000, 100);

    pid_set(&yaw_auto_speed_pid, 350.0f, 0.0f, 200.0f, GIMBALMOTOR_MAX_CURRENT, GIMBALMOTOR_MAX_CURRENT);
    pid_set(&yaw_auto_location_pid, 15.0f, 0.00f, 5.0f, GIMBALMOTOR_MAX_CURRENT, 1000);


    /*斜坡初始化*/
    // RampGenerator_Init(&pitch_ramp,1,)
    /*零点与限位*/
    GIMBALMotor_setzero(YAW_ZERO, YAWMotor);

    Global.Shoot.glass_mode = Glass_close;          // 倍镜默认关闭
    Global.Gimbal.small_pitch_target = level;       // 小云台默认前哨战站
    Global.Shoot.Hanging_Shot = Hanging_Shot_CLOSE; // 吊射模式必須默认关闭
    Global.Gimbal.pitch_lock = Pitch_move;          // PITCH默认不锁定
}

double yaw_now, yaw_set;
float pitch_lock_set;
uint32_t flag_shot;
void Gimbal_control()
{
    float pitch_speed, yaw_speed; // 局部变量电机速度
    /*小云台*/
    // 倍镜
    if (Global.Shoot.glass_mode == Glass_open)
        set_servo_angle(PWM_PIN_4, SERVO_ANGLE_OPEN); // 开瞄准镜
    else
        set_servo_angle(PWM_PIN_4, SERVO_ANGLE_OFF); // 关瞄准镜
        
    //设置小pitch角度
    float target_angle = SMALL_PITCH_OFFSET;
    if (Global.Gimbal.small_pitch_target == rush)
        target_angle -= SMALL_PITCH_RUSHB;
    else if (Global.Gimbal.small_pitch_target == outpost)
        target_angle -= SMALL_PITCH_OUTPOST;
    else if (Global.Gimbal.small_pitch_target == base)
        target_angle -= SMALL_PITCH_BASE;

    set_servo_angle(PWM_PIN_3, target_angle - Global.Gimbal.small_pitch_offset);
    /*yaw轴*/
    // yaw轴模式切换间的数据处理
    if (Global.Shoot.Hanging_Shot == Hanging_Shot_CLOSE || (Global.Shoot.Hanging_Shot == Hanging_Shot_OPEN && Global.Shoot.glass_mode == Glass_close)) // 普通模式陀螺仪控制
    {
        yaw_now = -RAD_TO_DEG * IMU_data.AHRS.yaw_rad_cnt - ecd2imu_err;
        yaw_set = Global.Gimbal.input.yaw - Setzero_ecd2imu;

        Setzero_imu2ecd = Global.Gimbal.input.yaw;
        imu2ecd_err = GIMBALMotor_get_data(YAWMotor).angle_cnt;
    }
    else // 吊射模式转成编码器控制
    {
        yaw_now = GIMBALMotor_get_data(YAWMotor).angle_cnt - imu2ecd_err;
        yaw_set = Global.Gimbal.input.yaw - Setzero_imu2ecd; //-90

        Setzero_ecd2imu = Global.Gimbal.input.yaw;
        ecd2imu_err = -RAD_TO_DEG * IMU_data.AHRS.yaw_rad_cnt;
    }

    // 设置yaw电流
    if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE) && (Global.Shoot.Hanging_Shot == Hanging_Shot_CLOSE || (Global.Shoot.Hanging_Shot == Hanging_Shot_OPEN && Global.Shoot.glass_mode == Glass_close)))
    {
        yaw_speed = pid_cal(&yaw_location_pid, yaw_now, yaw_set);
        GIMBALMotor_set(pid_cal(&yaw_speed_pid, (-cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] + sin(IMU_data.AHRS.pitch) * IMU_data.gyro[0]) * RAD_TO_DEG, yaw_speed), YAWMotor);
    }
    else if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE) && (Global.Shoot.Hanging_Shot == Hanging_Shot_OPEN))
    {
        yaw_speed = pid_cal(&yaw_lock_location_pid, yaw_now, yaw_set);
        GIMBALMotor_set(pid_cal(&yaw_lock_speed_pid, GIMBALMotor_get_data(YAWMotor).speed_rpm, yaw_speed), YAWMotor);
    }
    else
    { // 自瞄
        yaw_speed = -pid_cal(&yaw_auto_location_pid, Global.Auto.input.shoot_yaw, 0);
        GIMBALMotor_set(pid_cal(&yaw_auto_speed_pid, (-cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] + sin(IMU_data.AHRS.pitch) * IMU_data.gyro[0]) * RAD_TO_DEG, yaw_speed), YAWMotor);
        Global.Auto.input.Auto_control_online--;
        // if (Global.Shoot.Hanging_Shot == Hanging_Shot_CLOSE)
        //     Global.Gimbal.input.yaw = -RAD_TO_DEG * IMU_data.AHRS.yaw_rad_cnt - ecd2imu_err + Setzero_ecd2imu;
        // else
        //     Global.Gimbal.input.yaw = Setzero_imu2ecd  + GIMBALMotor_get_data(YAWMotor).angle_cnt - imu2ecd_err;
    }
    if (Global.Control.mode == LOCK)
        GIMBALMotor_set(0, YAWMotor);
    /*pitch*/

    if (Global.Gimbal.pitch_lock == Pitch_move)
        pitch_lock_set = GIMBALMotor_get_data(PITCHMotor).ecd_cnt;

    // 设置pitch电流
    // if (Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE)
    // { // 非自瞄
    if (Global.Gimbal.pitch_lock == Pitch_move)
    {
        pitch_speed = -pid_cal(&pitch_location_pid, RAD_TO_DEG * IMU_data.AHRS.pitch, Global.Gimbal.input.pitch);
        GIMBALMotor_set(pid_cal(&pitch_speed_pid, GIMBALMotor_get_data(PITCHMotor).speed_rpm, pitch_speed), PITCHMotor);
    }
    else
    {
        pitch_speed = pid_cal(&pitch_lock_location_pid, GIMBALMotor_get_data(PITCHMotor).ecd_cnt, pitch_lock_set);
        GIMBALMotor_set(pid_cal(&pitch_lock_speed_pid, GIMBALMotor_get_data(PITCHMotor).speed_rpm, pitch_speed), PITCHMotor);
    }
    if (Global.Control.mode == LOCK)
        GIMBALMotor_set(0, PITCHMotor);
    // }
    // else
    // { // 自瞄
    //     pitch_speed = pid_cal(&pitch_auto_location_pid, Global.Auto.input.shoot_pitch, 0);
    //     GIMBALMotor_set(pid_cal(&pitch_auto_speed_pid, GIMBALMotor_get_data(PITCHMotor).speed_rpm, pitch_speed), PITCHMotor);

    //     Global.Gimbal.pitch_lock = Pitch_move;
    //     Global.Gimbal.input.pitch = RAD_TO_DEG * IMU_data.AHRS.pitch;
    // }

    Hanging_Shot_offset();
}
#endif
