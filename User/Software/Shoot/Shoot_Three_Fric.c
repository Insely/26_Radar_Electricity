#include "robot_param.h"
#if (SHOOT_TYPE == SHOOT_THREE_FRIC)

#include "Shoot_Three_Fric.h"
#include "Global_status.h"
#include "Gimbal.h"
#include "pid.h"
#include "referee_system.h"
#include "Stm32_time.h"
#include "IMU_updata.h"
uint8_t shoot_flag_u, shoot_flag_l, shoot_flag_r;

#define TRIGGER_SPEED_CONTROL
uint32_t shoot_delay;
void trigger_fire_speed(int time);	  // 拨弹盘速度控制
void trigger_fire_location(int time); // 拨弹盘位置控制

enum shoot_speed_e shoot_U_speed;
enum shoot_speed_e shoot_L_speed;
enum shoot_speed_e shoot_R_speed;

// pid_t trigger_speed_pid, trigger_location_pid;
#ifdef TRIGGER_LOCATION_CONTROL
pid_t trigger_location_speed_pid, trigger_location_ecd_pid;
#endif // TRIGGER_LOCATION_CONTROL

#ifdef TRIGGER_SPEED_CONTROL
pid_t trigger_speed_pid;
#endif // TRIGGER_SPEED_CONTROL
pid_t shoot_L_speed_pid, shoot_R_speed_pid, shoot_U_speed_pid;
void Shoot_init()
{
	// 摩擦轮电机初始化
	SHOOTMotor_init(FRIC_MOTOR_TYPE, ShootMotor_L);
	SHOOTMotor_init(FRIC_MOTOR_TYPE, ShootMotor_R);
	SHOOTMotor_init(FRIC_MOTOR_TYPE, ShootMotor_U);
	// 拨弹电机初始化
	TriggerMotor_init(TRIGGER_MOTOR_TYPE, TRIGGER_MOTOR);

	// 摩擦轮电机
	pid_set(&shoot_U_speed_pid, 8 , 0.0, 0, SHOOTMOTOR_MAX_CURRENT, 0);
	pid_set(&shoot_L_speed_pid, 10, 0.0, 0, SHOOTMOTOR_MAX_CURRENT, 0);
	pid_set(&shoot_R_speed_pid, 10, 0.0, 0, SHOOTMOTOR_MAX_CURRENT, 0);

// 拨弹电机
#ifdef TRIGGER_SPEED_CONTROL
	pid_set(&trigger_speed_pid, 7, 0.0, 10, SHOOTMOTOR_MAX_CURRENT, 0);
#endif // TRIGGER_SPEED_CONTROL
#ifdef TRIGGER_LOCATION_CONTROL
	pid_set(&trigger_location_speed_pid, 3, 0, 10, SHOOTMOTOR_MAX_CURRENT, 0); // 7
	pid_set(&trigger_location_ecd_pid, 0.5, 0, 0, SHOOTMOTOR_MAX_CURRENT, 0);
#endif // TRIGGER_LOCATION_CONTROL
	Global.Shoot.ONtigger = tigger_open;
}
/**
 * @brief 摩擦轮控制
 *
 */
void Shoot_control()
{
	if (Global.Shoot.shoot_mode == CLOSE) // 摩擦轮关闭
	{
		shoot_U_speed = SHOOT_SPEED_CLOSE;
		shoot_L_speed = SHOOT_SPEED_CLOSE;
		shoot_R_speed = SHOOT_SPEED_CLOSE;
	}
	else if (Global.Shoot.shoot_mode == READY)
	{
		shoot_U_speed = SHOOT_SPEED_READY + Global.Shoot.shoot_deviation;
		shoot_L_speed = SHOOT_SPEED_READY + Global.Shoot.shoot_deviation + SHOOT_DIFFERENCE;
		shoot_R_speed = SHOOT_SPEED_READY + Global.Shoot.shoot_deviation + SHOOT_DIFFERENCE;
	}

	SHOOTMotor_set(pid_cal(&shoot_L_speed_pid, SHOOTMotor_get_data(ShootMotor_L).speed_rpm, shoot_L_speed), ShootMotor_L);
	SHOOTMotor_set(pid_cal(&shoot_R_speed_pid, SHOOTMotor_get_data(ShootMotor_R).speed_rpm, -shoot_R_speed), ShootMotor_R);
	SHOOTMotor_set(pid_cal(&shoot_U_speed_pid, SHOOTMotor_get_data(ShootMotor_U).speed_rpm, -shoot_U_speed), ShootMotor_U);

	// UploadData_vofa(SHOOTMotor_get_data(ShootMotor_L).speed_rpm, -SHOOTMotor_get_data(ShootMotor_R).speed_rpm,
					// -SHOOTMotor_get_data(ShootMotor_U).speed_rpm, 0);
}

/**
 * @brief 拨弹电机控制
 *
 */

void Trigger_control(void)
{
	static int trigger_kill_cnt; // 卡弹回退次数
	float trigger_speed;
	/*根据模式选择速度*/
	if (shoot_U_speed == SHOOT_SPEED_CLOSE || (Referee_data.Barrel_Heat > (Referee_data.Heat_Limit - 80)))
		Global.Shoot.tigger_mode = TRIGGER_CLOSE;
#ifdef TRIGGER_SPEED_CONTROL
	switch (Global.Shoot.tigger_mode)
	{
	case TRIGGER_CLOSE:
		shoot_delay = 0;
		Global.Shoot.trigger.speed_set = TRIGGER_SPEED_CLOSE;
		break;
	case DEBUG_TRIGGER:
		trigger_fire_speed(500);
		break;
	case SINGLE:
		trigger_fire_speed(1000);
		break;
	}
	uint8_t trigger_flag = 0;
	if (Global.Shoot.trigger.speed_set != TRIGGER_SPEED_CLOSE)
		trigger_flag = 1; // 假前馈

	if (Global.Shoot.ONtigger == tigger_open)
		TriggerMotor_set(((trigger_flag) * 0.7 + 1) * pid_cal(&trigger_speed_pid, TriggerMotor_get_data(TRIGGER_MOTOR).speed_rpm, Global.Shoot.trigger.speed_set), TRIGGER_MOTOR);
	else
		TriggerMotor_set(0, TRIGGER_MOTOR);
#endif // TRIGGER_SPEED_CONTROL
#ifdef TRIGGER_LOCATION_CONTROL
	switch (Global.Shoot.tigger_mode)
	{
	case TRIGGER_CLOSE:
		shoot_delay = 0;
		break;
	case DEBUG_TRIGGER:
		trigger_fire_location(500);
		break;
	case SINGLE:
		trigger_fire_location(800);
		break;
	}
	if (Global.Shoot.ONtigger == tigger_open)
		TriggerMotor_set(pid_cal(&trigger_location_speed_pid, TriggerMotor_get_data(TRIGGER_MOTOR).speed_rpm,
								 pid_cal(&trigger_location_ecd_pid, TriggerMotor_get_data(TRIGGER_MOTOR).ecd_cnt, Global.Shoot.trigger.ecd_set)),
						 TRIGGER_MOTOR); // 拨弹电机设置
	else
		TriggerMotor_set(0, TRIGGER_MOTOR);

#endif

	if (Global.Control.mode == LOCK)
	{
		SHOOTMotor_set(0, ShootMotor_L);
		SHOOTMotor_set(0, ShootMotor_R);
		SHOOTMotor_set(0, ShootMotor_U);

		TriggerMotor_set(0, TRIGGER_MOTOR);
	}
}

/**
 * @brief 发射机构任务
 *
 */
void Shoot_task(void)
{
	/*摩擦轮控制*/
	Shoot_control();
	/*拨弹电机控制*/
	Trigger_control();
}
float trigger_last_now;
void trigger_fire_speed(int time) // 拨弹盘速度控制
{
	if (fabs(SHOOTMotor_get_data(ShootMotor_U).speed_rpm) < fabs(shoot_U_speed) - 100) // 70
		shoot_flag_u = 1;
	if (fabs(SHOOTMotor_get_data(ShootMotor_L).speed_rpm) < fabs(shoot_L_speed) - 100) // 70
		shoot_flag_l = 1;
	if (fabs(SHOOTMotor_get_data(ShootMotor_R).speed_rpm) < fabs(shoot_R_speed) - 100) // 70
		shoot_flag_r = 1;
	if (shoot_flag_u + shoot_flag_l + shoot_flag_r >= 1)//拨弹盘掉速
	{
		shoot_flag_u = 0;
		shoot_flag_l = 0;
		shoot_flag_r = 0;
		shoot_delay = Get_sys_time_ms();
	}
	if (Global.Shoot.tigger_mode == SINGLE && Get_sys_time_ms() - shoot_delay > time &&
		fabs(SHOOTMotor_get_data(ShootMotor_U).speed_rpm) > 1500)
	{
		// Global.Shoot.trigger.speed_set = TRIGGER_SPEED_MID;
		// if (Global.Shoot.Hanging_Shot == Hanging_Shot_OPEN)
		// 	Global.Shoot.trigger.speed_set = TRIGGER_SPEED_MID;
		// else
		Global.Shoot.trigger.speed_set = TRIGGER_SPEED_LOW;
		Global.Shoot.shoot_angle.pitch_angle = RAD_TO_DEG * IMU_data.AHRS.pitch;
		Global.Shoot.shoot_angle.yaw_angle = RAD_TO_DEG * IMU_data.AHRS.yaw;
	}
	else /// 起码间隔八百毫秒，才能再发射一次弹丸
	{
		Global.Shoot.trigger.speed_set = TRIGGER_SPEED_CLOSE;
		trigger_last_now = SHOOTMotor_get_data(TRIGGER_MOTOR).ecd_cnt;
	}
}

void trigger_fire_location(int time) // 拨弹盘位置控制
{
	if (Global.Shoot.tigger_mode == SINGLE && Get_sys_time_ms() - shoot_delay > time /*&& fabs(SHOOTMotor_get_data(ShootMotor_U).speed_rpm) > 4000*/)
	{
		Global.Shoot.trigger.ecd_set += A_BULLET_ECD;
		shoot_delay = Get_sys_time_ms();
	}
}
#endif
