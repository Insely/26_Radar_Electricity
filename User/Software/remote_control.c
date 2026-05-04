#include "remote_control.h"
#include "Global_status.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "ramp_generator.h"
#include "referee_system.h"

#include "DT7.h"
#include "VT13.h"
#include "FSI6X.h"
#include "Power_switch.h"
#include "IMU_updata.h"

#include "Stm32_time.h"
#include "Power_switch.h"

#include "cmsis_os2.h"

RC_ctrl_t RC_data;

int16_t wait_time[SIZE_OF_WAIT] = {0}; // 键盘消抖用时间
uint32_t beijing_delay = 0;
uint32_t yuntai_delay = 0;

uint16_t shot_close_delay;

uint8_t shot_close_flag;
uint8_t vt13_flag = 1;
uint8_t shoot_gimmbal_flag = 0;


#define MOVE_SENSITIVITY 10.0f   // 移动灵敏度，
#define PITCH_SENSITIVITY 0.008f // pitch轴灵敏度
#define YAW_SENSITIVITY 5.0f     // yaw轴灵敏度

/**
 * @brief 统一消抖
 *
 * @param key 按键宏
 * @return uint8_t 0未到时间，1到时间
 */
uint8_t Wait(uint8_t key)
{
    if (wait_time[key] >= 2)
    {
        wait_time[key]--;
        return 0;
    }
    else if ((wait_time[key] == 0) || (wait_time[key] == 1))
    {
        wait_time[key]--;
        return 1;
    }
    else
    {
        return 0;
    }
}

void SetWait(uint8_t key)
{
    wait_time[key] = 300;
}
/**
 * @brief 遥控器数据置零
 *
 */
void RC_DATA_ZERO()
{
    RC_data.rc.ch[0] = 0;
    RC_data.rc.ch[1] = 0;
    RC_data.rc.ch[2] = 0;
    RC_data.rc.ch[3] = 0;
    RC_data.rc.s[0] = 2;
    RC_data.rc.s[1] = 2;
    RC_data.key.v = 0;
    RC_data.mouse.x = 0;
    RC_data.mouse.y = 0;
    RC_data.mouse.z = 0;
    RC_data.mouse.press_l = 0;
    RC_data.mouse.press_r = 0;
    RC_data.mouse.press_mid = 0;
}
/**
 * @brief 循环更换模式
 *
 * @param mode  模式
 * @param num   模式的数量
 * @param identity 循环方式 0 正循环 1 反循环
 */
void mode_circulate(uint8_t *mode , uint8_t num,uint8_t identity)
{
    if(identity == 0)
        *mode = (*mode + 1) % num;
    else
        *mode = (*mode + num - 1) % num;
}

#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
/**
 * @brief 遥控数据来源于DT7遥控器
 *
 */
void DT7toRCdata()
{
    /*遥控器数据*/
    RC_data.rc.ch[0] = DT7_data.rc.ch[0];
    RC_data.rc.ch[1] = DT7_data.rc.ch[1];
    RC_data.rc.ch[2] = DT7_data.rc.ch[2];
    RC_data.rc.ch[3] = DT7_data.rc.ch[3];
    RC_data.rc.ch[4] = DT7_data.rc.ch[4];
    RC_data.rc.s[0] = DT7_data.rc.s[0];
    RC_data.rc.s[1] = DT7_data.rc.s[1];
    /*键鼠数据 */
    RC_data.key.v = DT7_data.key.v;
    RC_data.mouse.x = DT7_data.mouse.x;
    RC_data.mouse.y = DT7_data.mouse.y;
    RC_data.mouse.z = DT7_data.mouse.z;
    RC_data.mouse.press_l = DT7_data.mouse.press_l;
    RC_data.mouse.press_r = DT7_data.mouse.press_r;
    RC_data.mouse.press_mid = 0;
    DT7_data.online--;
    RC_data.online = DT7_data.online;
}

/**
 * @brief 来自图传的遥控数据
 *
 */
void VT13toRCdata()
{
    /*遥控器数据*/
    RC_data.rc.ch[0] = VT13_data.rc.ch[0];
    RC_data.rc.ch[1] = VT13_data.rc.ch[1];
    RC_data.rc.ch[2] = VT13_data.rc.ch[3];
    RC_data.rc.ch[3] = VT13_data.rc.ch[2];
    if (VT13_data.rc.shutter == 1) // 扳机键与开火相对应
        RC_data.rc.ch[4] = 660;
    else
        RC_data.rc.ch[4] = 0;
    // 挡位与拨杆映射
    if (VT13_data.rc.mode_sw == 1) // N
        RC_data.rc.s[0] = RC_SW_MID;
    if (VT13_data.rc.mode_sw == 0) // C
        RC_data.rc.s[0] = RC_SW_DOWN;
    if (VT13_data.rc.mode_sw == 2) // S
    {
        RC_data.rc.s[0] = RC_SW_UP;
        RC_data.rc.s[1] = RC_SW_UP;
    }
    // 滚轮与拨杆映射
    if (VT13_data.rc.wheel < -330)
        RC_data.rc.s[1] = RC_SW_DOWN;
    if ((VT13_data.rc.wheel > -330) && (VT13_data.rc.wheel < 330) && (VT13_data.rc.mode_sw != 2))
        RC_data.rc.s[1] = RC_SW_MID;
    if (VT13_data.rc.wheel >= 330)
        RC_data.rc.s[1] = RC_SW_UP;
    if (VT13_data.rc.left_button == 1)
        Power_Turn_off(power2);
    else
        Power_Turn_on(power2);
    /* if (VT13_data.rc.right_button == 1)
        GIMBALMotor_setzero(YAW_ZERO + 135.0f, YAWMotor);
    else
        GIMBALMotor_setzero(YAW_ZERO, YAWMotor); */

    /*键鼠数据 */
    RC_data.key.v = VT13_data.key.v;
    RC_data.mouse.x = VT13_data.mouse.x;
    RC_data.mouse.y = VT13_data.mouse.y;
    RC_data.mouse.z = VT13_data.mouse.z;
    RC_data.mouse.press_l = VT13_data.mouse.press_l;
    RC_data.mouse.press_r = VT13_data.mouse.press_r;
    RC_data.mouse.press_mid = VT13_data.mouse.middle;
    VT13_data.online--;
    RC_data.online = VT13_data.online;
}

/**
 * @brief FS_I6X遥控器控制
 *
 */
void FSI6XtoRCdata()
{ 
   RC_data.rc.ch[0]=SBUS_CH.CH1;
   RC_data.rc.ch[1]=SBUS_CH.CH2;
   RC_data.rc.ch[2]=SBUS_CH.CH4;
   RC_data.rc.ch[3]=SBUS_CH.CH3;

   for(int i = 0 ; i<4 ; i++) //死区判断
   {
    if(fabs(RC_data.rc.ch[i]) < 10)
    RC_data.rc.ch[i] = 0;
   }
   
   if(SBUS_CH.CH8==FS_DOWN)//LOCK
   {
    RC_data.rc.s[0]=RC_SW_DOWN;
    RC_data.rc.s[1]=RC_SW_DOWN;
   }
   else if (SBUS_CH.CH8==FS_UP)
   {
     if (SBUS_CH.CH7==FS_MID && SBUS_CH.CH6 == FS_UP)//正小陀螺
     {
        RC_data.rc.s[0]=RC_SW_UP ;
        RC_data.rc.s[1]=RC_SW_MID;
     }
     else if (SBUS_CH.CH7==FS_DOWN && SBUS_CH.CH6 == FS_UP)//逆小陀螺
     {
        RC_data.rc.s[1]=RC_SW_DOWN;
        RC_data.rc.s[0]=RC_SW_MID;
     }
     else if(SBUS_CH.CH6 == FS_DOWN && SBUS_CH.CH7 == FS_UP)//开启摩擦轮
     {
       RC_data.rc.s[1]=RC_SW_UP; 
       RC_data.rc.s[0]=RC_SW_MID;
       
     }
     else
     {
       RC_data.rc.s[1]=RC_SW_MID; 
       RC_data.rc.s[0]=RC_SW_MID;
     }
     if(SBUS_CH.CH5 == FS_DOWN) //开火
       {
        RC_data.rc.ch[4] = 660;
       }
       else if(SBUS_CH.CH5 == FS_UP)
       {
        RC_data.rc.ch[4] = 0;
       }
   }
   SBUS_CH.online--;
   RC_data.online = SBUS_CH.online;
}

/**
 * @brief 遥控器控制
 *
 */
void RC_control()
{
    if (RC_data.online >= 0)
        RC_data.online--;
    /*控制模式选择*/
    if ((RC_data.rc.s[0]==RC_SW_DOWN && RC_data.rc.s[1]==RC_SW_DOWN) || (RC_data.online <= 0)) // 左下右下，锁死
        Global.Control.mode = LOCK;
    else if (RC_data.rc.s[0]==RC_SW_UP && RC_data.rc.s[1]==RC_SW_UP) // 左上右上，键盘控制
        Global.Control.mode = KEY;
    else
        Global.Control.mode = RC;
    if (Global.Control.mode != RC)
        return;
    /*底盘控制*/
    if (RC_data.rc.s[0]==RC_SW_UP && RC_data.rc.s[1]==RC_SW_MID) // 右上,左中||左上，正小陀螺
        Global.Chssis.mode = SPIN_P;
    else if (RC_data.rc.s[1]==RC_SW_DOWN && RC_data.rc.s[0]==RC_SW_MID ) // 左下,右中||右上，負小陀螺
        Global.Chssis.mode = SPIN_N;
    else
        Global.Chssis.mode = FLOW;
    Chassis_set_x(RC_data.rc.ch[0] / 40.0f);
    Chassis_set_y(RC_data.rc.ch[1] / 40.0f);
    /*云台控制*/
    if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.fire == -1) && Global.Gimbal.mode == NORMAL)
    {
        Gimbal_set_pitch_angle(Global.Gimbal.input.pitch + RC_data.rc.ch[3] / 2000.0f);
        Gimbal_set_yaw_angle(-RC_data.rc.ch[2]);
    }
    /*自瞄控制*/
    if (RC_data.rc.s[0]==RC_SW_DOWN && (RC_data.rc.s[1]==RC_SW_MID || RC_data.rc.s[1]==RC_SW_UP)) // 右下,左中||左上，自瞄,射击模式
    {
        Global.Auto.mode = CAR;
    }
    else
    {
        Global.Auto.mode = NONE;
    }
    /*发弹机构控制*/
    if (RC_data.rc.s[1]==RC_SW_UP && (RC_data.rc.s[0]==RC_SW_MID || RC_data.rc.s[0]==RC_SW_DOWN)) // 左上，右中||右下，开启摩擦轮
        Global.Shoot.shoot_mode = READY;
    else
        Global.Shoot.shoot_mode = CLOSE;
    if (RC_data.rc.ch[4] >= 300 &&
        RC_data.rc.ch[4] <= 660 &&
        Global.Shoot.shoot_mode != CLOSE &&
        (Global.Auto.mode == NONE ||
         Global.Auto.input.fire == 1)) // 滚轮最下头，高速发弹，若自瞄打开，发弹标志位置1允许发弹
        Global.Shoot.tigger_mode = HIGH;
    else if (RC_data.rc.ch[4] >= 50 &&
             RC_data.rc.ch[4] <= 300 &&
             Global.Shoot.shoot_mode != CLOSE &&
             (Global.Auto.mode == NONE ||
              Global.Auto.input.fire == 1)) // 滚轮中部，低速发弹,若自瞄打开，发弹标志位置1允许发弹
        Global.Shoot.tigger_mode = LOW;
    else if (RC_data.rc.ch[4] > 660 &&
             Global.Shoot.shoot_mode != CLOSE)
    {
        Global.Shoot.shoot_mode = DEBUG_SHOOT;
        Global.Shoot.tigger_mode = DEBUG_SHOOT;
    }
    else
        Global.Shoot.tigger_mode = TRIGGER_CLOSE;
}


void Keyboard_mouse_control(void)
{
    if (Global.Control.mode != KEY)
        return;
    /*底盘控制*/
    if (IF_KEY_PRESSED_W)
        Chassis_set_y(5.0);
    if (IF_KEY_PRESSED_S)
        Chassis_set_y(-5.0);
    if (IF_KEY_PRESSED_D)
        Chassis_set_x(5.0);
    if (IF_KEY_PRESSED_A)
        Chassis_set_x(-5.0);
    if (!IF_KEY_PRESSED_W && !IF_KEY_PRESSED_S)
        Chassis_set_y(0);
    if (!IF_KEY_PRESSED_A && !IF_KEY_PRESSED_D)
        Chassis_set_x(0);
    if (IF_KEY_PRESSED_SHIFT) // 超电开关
    {
        Global.Cap.mode = FULL;
        // Chassis_set_accel(8.0);
    }
    else
    {
        Global.Cap.mode = Not_FULL;
        // Chassis_set_accel(4.0);
    }
    /* if (IF_KEY_PRESSED_X)
        GIMBALMotor_setzero(YAW_ZERO + 135.0f, YAWMotor);
    else
        GIMBALMotor_setzero(YAW_ZERO, YAWMotor); */
    if (IF_KEY_PRESSED_Q || Wait(WAIT_Q)) // 小陀螺开关
    {
        if (Wait(WAIT_Q)) // 消抖结束触发
        {
            if (Global.Chssis.mode != SPIN_P)
                Global.Chssis.mode = SPIN_P;
            else
                Global.Chssis.mode = FLOW;
        }
        if (IF_KEY_PRESSED_Q)
            SetWait(WAIT_Q);
    }
    if (IF_KEY_PRESSED_Z) // UI开关
    {
        //ui_init();
    }
    if (IF_KEY_PRESSED_CTRL)
        Global.Chssis.input.reset = 1;
    else
        Global.Chssis.input.reset = 0;
    /*自瞄射击模式*/
    if (IF_MOUSE_PRESSED_RIGH)
    {
        Global.Auto.mode = CAR;
        Global.Gimbal.mode = SHOOT;
    }
    else
    {
        Global.Auto.mode = NONE;
        Global.Gimbal.mode = NORMAL;
    }
    /*云台控制*/
    if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.fire == -1) && Global.Gimbal.mode == NORMAL)
    {
        Gimbal_set_pitch_angle(Global.Gimbal.input.pitch + MOUSE_Y_MOVE_SPEED * PITCH_SENSITIVITY);
        Gimbal_set_yaw_angle(-MOUSE_X_MOVE_SPEED * YAW_SENSITIVITY);
    }
    else if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.fire == -1) && Global.Gimbal.mode == SHOOT)
    {
        Gimbal_set_pitch_angle(Global.Gimbal.input.pitch + MOUSE_Y_MOVE_SPEED * PITCH_SENSITIVITY);
        Gimbal_set_yaw_angle(Global.Gimbal.input.yaw - MOUSE_X_MOVE_SPEED * 0.005f);
    }
    /*相机重启*/
    if (IF_KEY_PRESSED_B)
        Power_Turn_off(power2);
    else
        Power_Turn_on(power2);
    /*发射机构控制*/
    if (IF_KEY_PRESSED_R || Wait(WAIT_R)) // 摩擦轮开关
    {
        if (Wait(WAIT_R)) // 消抖
            Global.Shoot.shoot_mode = !Global.Shoot.shoot_mode;
        if (IF_KEY_PRESSED_R)
            SetWait(WAIT_R);
    }
    if (IF_MOUSE_PRESSED_LEFT &&
        Global.Shoot.shoot_mode != CLOSE &&
        (Global.Auto.mode == NONE ||
         Global.Auto.input.fire == 1 ||
         Global.Auto.input.fire == -1)) // 拨弹电机控制
        Global.Shoot.tigger_mode = HIGH;
    else
        Global.Shoot.tigger_mode = TRIGGER_CLOSE;
}
#endif //(GIMBAL_TYPE == DJIMOTOR_AS_GIMBALMOTOR)

#if (GIMBAL_TYPE == GIMBAL_HANG_SHOT)

void shot2flow();
/**
 * @brief 遥控数据来源于DT7遥控器
 *
 */
void DT7toRCdata()
{
    /*遥控器数据*/
    RC_data.rc.ch[0] = DT7_data.rc.ch[0];
    RC_data.rc.ch[1] = DT7_data.rc.ch[1];
    RC_data.rc.ch[2] = DT7_data.rc.ch[2];
    RC_data.rc.ch[3] = DT7_data.rc.ch[3];
    RC_data.rc.ch[4] = DT7_data.rc.ch[4];
    RC_data.rc.s[0] = DT7_data.rc.s[0];
    RC_data.rc.s[1] = DT7_data.rc.s[1];
    /*键鼠数据 */
    RC_data.key.v = DT7_data.key.v;
    RC_data.mouse.x = DT7_data.mouse.x;
    RC_data.mouse.y = DT7_data.mouse.y;
    RC_data.mouse.z = DT7_data.mouse.z;
    RC_data.mouse.press_l = DT7_data.mouse.press_l;
    RC_data.mouse.press_r = DT7_data.mouse.press_r;
    DT7_data.online--;
    RC_data.online = DT7_data.online;

    RC_data.Sreial_mode = Sreial_RC;
    vt13_flag = 1;
}
/**
 * @brief 遥控数据来源于VT13遥控器
 *
 */
void VT13toRCdata()
{
    /*遥控器数据*/
    RC_data.rc.ch[0] = VT13_data.rc.ch[0];
    RC_data.rc.ch[1] = VT13_data.rc.ch[1];
    RC_data.rc.ch[2] = VT13_data.rc.ch[3];
    RC_data.rc.ch[3] = VT13_data.rc.ch[2];
    if (VT13_data.rc.shutter == 1) // 扳机键与开火相对应
        RC_data.rc.ch[4] = -660;

    else if (VT13_data.rc.right_button == 1) // 扳机键与开火相对应
        RC_data.rc.ch[4] = 660;
    else
        RC_data.rc.ch[4] = 0;
    // 挡位与拨杆映射
    if (VT13_data.rc.mode_sw == 1) // N
        RC_data.rc.s[0] = RC_SW_MID;
    if (VT13_data.rc.mode_sw == 0) // C
        RC_data.rc.s[0] = RC_SW_DOWN;
    if (VT13_data.rc.mode_sw == 2) // S
        RC_data.rc.s[0] = RC_SW_UP;
    // 滚轮与拨杆映射
    if (VT13_data.rc.wheel < -330)
        RC_data.rc.s[1] = RC_SW_DOWN;
    if ((VT13_data.rc.wheel > -330) && (VT13_data.rc.wheel < 330))
        RC_data.rc.s[1] = RC_SW_MID;
    if (VT13_data.rc.wheel >= 330)
        RC_data.rc.s[1] = RC_SW_UP;

    if (vt13_flag == 1) // 为了上场减少链路间切换的流程
    {
        RC_data.Sreial_mode = Sreial_KEY;
        vt13_flag = 0;
    }
    if ((VT13_data.rc.stop) || Wait(WAIT_SREIAL_STOP)) // 切换遥控器跟键鼠
    {
        if (Wait(WAIT_SREIAL_STOP))
        {
            if (RC_data.Sreial_mode == Sreial_KEY)
                RC_data.Sreial_mode = Sreial_RC;
            else
                RC_data.Sreial_mode = Sreial_KEY;
        }
        if (VT13_data.rc.stop)
            SetWait(WAIT_SREIAL_STOP);
    }
    /*键鼠数据 */
    RC_data.key.v = VT13_data.key.v;
    RC_data.mouse.x = VT13_data.mouse.x;
    RC_data.mouse.y = VT13_data.mouse.y;
    RC_data.mouse.z = VT13_data.mouse.z;
    RC_data.mouse.press_l = VT13_data.mouse.press_l;
    RC_data.mouse.press_r = VT13_data.mouse.press_r;
    RC_data.mouse.press_mid = VT13_data.mouse.middle;
    VT13_data.online--;
    RC_data.online = VT13_data.online;
}
uint8_t beijing_flag;
uint8_t diaoshe_flag = 0;
/**
 * @brief 遥控器控制
 *
 */
void RC_control()
{
    /*控制模式选择*/
    shoot_gimmbal_flag = 0;
    if (RC_data.Sreial_mode == Sreial_RC)
    {
        if (switch_is_up(RC_R_SW) && switch_is_up(RC_L_SW)) // 左上右上，键盘控制
            Global.Control.mode = KEY;
        else
            Global.Control.mode = RC;
    }
    else
        Global.Control.mode = KEY;

    if (switch_is_down(RC_R_SW) && switch_is_down(RC_L_SW)) // 左下右下，锁死
        Global.Control.mode = LOCK;

    if (Global.Control.mode != RC)
        return;

    /*底盘模式*/
    if (switch_is_mid(RC_L_SW) && switch_is_down(RC_R_SW)) // 左中右下，顺时针小陀螺
        Global.Chssis.mode = SPIN_P;
    else if (switch_is_mid(RC_L_SW) && switch_is_up(RC_R_SW)) // 左中,右上，逆时针小陀螺
        Global.Chssis.mode = SPIN_N;
    else
        Global.Chssis.mode = FLOW;

    // /*自瞄 */
    // if (switch_is_mid(RC_L_SW) && switch_is_up(RC_R_SW)) //
    //     Global.Auto.mode = CAR;
    // else
    //     Global.Auto.mode = NONE;

    /*摩擦轮控制*/
    if ((switch_is_up(RC_L_SW) || switch_is_down(RC_L_SW)) && (switch_is_mid(RC_R_SW))) // 左上，右中||右下，开启摩擦轮
        Global.Shoot.shoot_mode = READY;
    else
        Global.Shoot.shoot_mode = CLOSE;

    /*吊射模式*/
    if (switch_is_down(RC_L_SW) && (switch_is_up(RC_R_SW)))
    {
        Global.Shoot.Hanging_Shot = Hanging_Shot_OPEN;
        // Global.Shoot.shoot_mode = READY;
        Global.Chssis.mode = SIDEWAYS;
    }
    else
        Global.Shoot.Hanging_Shot = Hanging_Shot_CLOSE;

    /*底盘移动*/
    Chassis_set_x(RC_data.rc.ch[0] / 220.0f);
    Chassis_set_y(RC_data.rc.ch[1] / 220.0f);
    /*云台控制*/
    if (Global.Auto.mode == NONE) // 手瞄
    {
        if (Global.Shoot.Hanging_Shot == Hanging_Shot_CLOSE) // 普通模式陀螺仪控制
        {
            Gimbal_set_pitch_angle(Global.Gimbal.input.pitch + RC_data.rc.ch[3] / 20000.0f);
            // Global.Gimbal.input.pitch = -RC_data.rc.ch[3]; // 速度控制做测试
            Gimbal_set_yaw_angle(Global.Gimbal.input.yaw + RC_data.rc.ch[2] / 5000.0f);
        }
        else // 吊射模式
        {
            Gimbal_set_pitch_angle(Global.Gimbal.input.pitch + RC_data.rc.ch[3] / 300000.0f);
            Gimbal_set_yaw_angle(Global.Gimbal.input.yaw + RC_data.rc.ch[2] / 100000.0f);
        }
    }
    else
    {
        Gimbal_set_pitch_angle(Global.Gimbal.input.pitch);
        Gimbal_set_yaw_angle(Global.Gimbal.input.yaw);
    }
    // else
    // {
    //     Gimbal_set_pitch_angle(Global.Gimbal.input.pitch + DEG_TO_RAD * Global.Auto.input.shoot_pitch * PITCH_SENSITIVITY_FromNUC);
    //     Gimbal_set_yaw_angle(Global.Gimbal.input.yaw + DEG_TO_RAD * Global.Auto.input.shoot_yaw * YAW_SENSITIVITY_FromNUC);
    // }
    // /*自瞄控制*/
    // if (switch_is_down(RC_R_SW) && (switch_is_mid(RC_L_SW) || switch_is_up(RC_L_SW))) // 右下,左中||左上，自瞄
    //     Global.Auto.mode = CAR;
    // else
    //     Global.Auto.mode = NONE;
    /*发弹机构控制*/
    // if (switch_is_up(RC_L_SW) && (switch_is_mid(RC_R_SW) || switch_is_down(RC_R_SW))) // 左上，右中||右下，开启摩擦轮
    //     Global.Shoot.shoot_mode = READY;
    // else
    //     Global.Shoot.shoot_mode = CLOSE;

    /*拨弹盘控制*/
    if (RC_data.rc.ch[4] == -660 && Global.Shoot.shoot_mode != CLOSE)
        Global.Shoot.tigger_mode = SINGLE;
    else
        Global.Shoot.tigger_mode = TRIGGER_CLOSE;
    /*倍镜控制*/
    if (RC_data.rc.ch[4] == 660)
    {
        if (Get_sys_time_ms() - beijing_delay > 200)
        {
            beijing_delay = Get_sys_time_ms();
            if (Global.Shoot.glass_mode == Glass_open)
                Global.Shoot.glass_mode = Glass_close;
            else
                Global.Shoot.glass_mode = Glass_open;
        }
    }
    /*小雲台*/
    if (RC_data.rc.ch[4] == -660 && Global.Shoot.shoot_mode == CLOSE)
    {
        if (Get_sys_time_ms() - yuntai_delay > 500)
        {
            shoot_gimmbal_flag = 1;
            // if (Global.Gimbal.small_pitch_target == rush)
            //     Global.Gimbal.small_pitch_target = outpost;
            // else if (Global.Gimbal.small_pitch_target == outpost)
            //     Global.Gimbal.small_pitch_target = base;
            // else if (Global.Gimbal.small_pitch_target == base)
            //     Global.Gimbal.small_pitch_target = level;
            // else
            //     Global.Gimbal.small_pitch_target = rush;
            mode_circulate(&Global.Gimbal.small_pitch_target , smallPitch_target_num,0);
            yuntai_delay = Get_sys_time_ms();
        }
    }

    if (shoot_gimmbal_flag == 1) // 根据模式一件设置pitch角度
    {
        if (Global.Gimbal.small_pitch_target == rush)
            Gimbal_set_pitch_angle(-SMALL_PITCH_RUSHB);
        else if (Global.Gimbal.small_pitch_target == outpost)
            Gimbal_set_pitch_angle(-SMALL_PITCH_OUTPOST);
        else if (Global.Gimbal.small_pitch_target == base)
            Gimbal_set_pitch_angle(-SMALL_PITCH_BASE);
        else if (Global.Gimbal.small_pitch_target == level)
            Gimbal_set_pitch_angle(0);
    }
}

float speed_cap = 0;
void Keyboard_mouse_control(void)
{
    shoot_gimmbal_flag = 0;
    if (Global.Control.mode != KEY)
        return;
    /*底盘控制*/
    if (IF_KEY_PRESSED_SHIFT && (IF_KEY_PRESSED_W || IF_KEY_PRESSED_S))
    {
        Global.Cap.mode = FULL;
        speed_cap += 0.002;
        if (speed_cap > (1.5 + 1 * (Referee_data.Chassis_Power_Limit + 30) / 150)) // 根据裁判系统底盘功率限制做最大速度限制
            speed_cap = (1.5 + 1 * (Referee_data.Chassis_Power_Limit + 30) / 150);
    }
    else if ((!IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_W))
        Global.Cap.mode = Not_FULL;
    else
    {
        Global.Cap.mode = Not_FULL;
        speed_cap = 0;
    }
    if (Global.Shoot.Hanging_Shot == Hanging_Shot_CLOSE)
    {
        if (IF_KEY_PRESSED_W)
            Chassis_set_y(1.5 + speed_cap);
        if (IF_KEY_PRESSED_S)
            Chassis_set_y(-(1.5 + speed_cap));
        if (IF_KEY_PRESSED_D)
            Chassis_set_x(1.3);
        if (IF_KEY_PRESSED_A)
            Chassis_set_x(-1.3);
        if (!IF_KEY_PRESSED_W && !IF_KEY_PRESSED_S)
            Chassis_set_y(0);
        if (!IF_KEY_PRESSED_A && !IF_KEY_PRESSED_D)
            Chassis_set_x(0);
    }
    else if (Global.Shoot.Hanging_Shot == Hanging_Shot_OPEN && Global.Shoot.glass_mode == Glass_close)
    {
        if (IF_KEY_PRESSED_W)
            Chassis_set_x(-(0.5));
        if (IF_KEY_PRESSED_S)
            Chassis_set_x(0.5);
        if (IF_KEY_PRESSED_D)
            Chassis_set_y(0.5);
        if (IF_KEY_PRESSED_A)
            Chassis_set_y(-0.5);

        if (!IF_KEY_PRESSED_W && !IF_KEY_PRESSED_S)
            Chassis_set_x(0);
        if (!IF_KEY_PRESSED_A && !IF_KEY_PRESSED_D)
            Chassis_set_y(0);
    }
    else // 吊射模式下底盘不动
    {
        Chassis_set_y(0);
        Chassis_set_x(0);
    }

    /*自瞄*/
    if (IF_KEY_PRESSED_V) // 自瞄
        Global.Auto.mode = CAR;
    else
        Global.Auto.mode = NONE;

    /*小陀螺*/
    if ((IF_KEY_PRESSED_Q || Wait(WAIT_Q)) &&
        Global.Shoot.Hanging_Shot != Hanging_Shot_OPEN && !IF_KEY_PRESSED_CTRL) // 小陀螺开关
    {
        if (Wait(WAIT_Q)) // 消抖
        {
            if (Global.Chssis.mode != SPIN_P)
                Global.Chssis.mode = SPIN_P;
            else
                Global.Chssis.mode = FLOW;
        }
        if (IF_KEY_PRESSED_Q)
            SetWait(WAIT_Q);
    }
    /*UI*/
    if (IF_KEY_PRESSED_Z) // UI开关
    {
        //ui_init();
    }
    // if (IF_KEY_PRESSED_CTRL)
    //     Global.Chssis.input.reset = 1;
    // else
    //     Global.Chssis.input.reset = 0;

    /*云台控制*/
    Global.Gimbal.pitch_lock = Pitch_move;
    if (Global.Auto.mode == NONE) // 手瞄，为不同情况下做的云台控制模式
    {
        if (Global.Shoot.Hanging_Shot == Hanging_Shot_CLOSE && Global.Gimbal.small_pitch_target == level) // 普通模式陀螺仪控制（不开吊射模式）
        {
            Gimbal_set_pitch_angle(Global.Gimbal.input.pitch + MOUSE_Y_MOVE_SPEED * PITCH_SENSITIVITY / (Global.Shoot.glass_mode * 9 + 1)); // 开镜灵敏度降低
            Gimbal_set_yaw_angle(Global.Gimbal.input.yaw + MOUSE_X_MOVE_SPEED * YAW_SENSITIVITY / (Global.Shoot.glass_mode * 9 + 1));
        }
        else if (Global.Shoot.Hanging_Shot == Hanging_Shot_CLOSE && (Global.Gimbal.small_pitch_target != level))//紧急情况下无法进入吊射模式为每个吊射角度降低灵敏度
        {
            Gimbal_set_pitch_angle(Global.Gimbal.input.pitch + MOUSE_Y_MOVE_SPEED * PITCH_SENSITIVITY / (Global.Shoot.glass_mode * 6 + 4));
            Gimbal_set_yaw_angle(Global.Gimbal.input.yaw + MOUSE_X_MOVE_SPEED * YAW_SENSITIVITY / (Global.Shoot.glass_mode * 6 + 4));
        }
        else if (((IF_KEY_PRESSED_CTRL && (!IF_KEY_PRESSED_Q) && (!IF_KEY_PRESSED_E)) || Global.Shoot.glass_mode == CLOSE) && 
            Global.Shoot.Hanging_Shot == Hanging_Shot_OPEN) // 吊射模式下速度偏快的移动，用鼠标大调(吊射模式下按住ctrl或者不开镜)
        {
            Gimbal_set_pitch_angle(Global.Gimbal.input.pitch + MOUSE_Y_MOVE_SPEED * PITCH_SENSITIVITY / 15);
            Gimbal_set_yaw_angle(Global.Gimbal.input.yaw + MOUSE_X_MOVE_SPEED * YAW_SENSITIVITY / 15);
        }
        else // 吊射模式下云台细调（不按ctrl，用wsad微调）
        {
            if ((IF_KEY_PRESSED_W))
                Gimbal_set_pitch_angle(Global.Gimbal.input.pitch + 0.0005);
            if ((IF_KEY_PRESSED_S))
                Gimbal_set_pitch_angle(Global.Gimbal.input.pitch - 0.0005);
            if ((IF_KEY_PRESSED_A))
                Gimbal_set_yaw_angle(Global.Gimbal.input.yaw - 0.0005);
            if ((IF_KEY_PRESSED_D))
                Gimbal_set_yaw_angle(Global.Gimbal.input.yaw + 0.0005);

            if ((!IF_KEY_PRESSED_W) && (!IF_KEY_PRESSED_S))
                Global.Gimbal.pitch_lock = Pitch_lock;
            else
                Global.Gimbal.pitch_lock = Pitch_move;
        }
    }

    /*小云台目标选择*/ //***************小雲台模式切換，有點麻煩了，後面整理*/
    if ((IF_KEY_PRESSED_G) || Wait(WAIT_G))
    {
        if (Wait(WAIT_G))
        {
            Global.Gimbal.small_pitch_offset = 0;
            shoot_gimmbal_flag = 1;
            // if (Global.Gimbal.small_pitch_target == rush)
            //     Global.Gimbal.small_pitch_target = outpost;
            // else if (Global.Gimbal.small_pitch_target == outpost)
            //     Global.Gimbal.small_pitch_target = base;
            // else if (Global.Gimbal.small_pitch_target == base)
            //     Global.Gimbal.small_pitch_target = level;
            // else
            //     Global.Gimbal.small_pitch_target = rush;
            mode_circulate(&Global.Gimbal.small_pitch_target , smallPitch_target_num,0);
        }
        if (IF_KEY_PRESSED_G)
            SetWait(WAIT_G);
    }
    if ((IF_KEY_PRESSED_F) || Wait(WAIT_F))
    {
        if (Wait(WAIT_F))
        {
            Global.Gimbal.small_pitch_offset = 0;
            shoot_gimmbal_flag = 1;

            // if (Global.Gimbal.small_pitch_target == level)
            //     Global.Gimbal.small_pitch_target = base;
            // else if (Global.Gimbal.small_pitch_target == base)
            //     Global.Gimbal.small_pitch_target = outpost;
            // else if (Global.Gimbal.small_pitch_target == outpost)
            //     Global.Gimbal.small_pitch_target = rush;
            // else
            //     Global.Gimbal.small_pitch_target = level;
            mode_circulate(&Global.Gimbal.small_pitch_target , smallPitch_target_num,1);
        }
        if (IF_KEY_PRESSED_F)
            SetWait(WAIT_F);
    }
    /*发射机构控制*/
    if ((IF_KEY_PRESSED_R) || Wait(WAIT_R)) // 摩擦轮开关R
    {
        if (Wait(WAIT_R)) // 消抖
        {
            if (Global.Shoot.shoot_mode == CLOSE)
                Global.Shoot.shoot_mode = READY;
            else
                Global.Shoot.shoot_mode = CLOSE;
        }
        if (IF_KEY_PRESSED_R)
            SetWait(WAIT_R);
    }

    if ((IF_MOUSE_PRESSED_RIGH) || Wait(WAIT_PRESSED_RIGH)) // 倍镜右键
    {
        if (Wait(WAIT_PRESSED_RIGH)) // 消抖
        {
            if (Global.Shoot.glass_mode == Glass_open)
                Global.Shoot.glass_mode = Glass_close;
            else
                Global.Shoot.glass_mode = Glass_open;
        }
        if (IF_MOUSE_PRESSED_RIGH)
            SetWait(WAIT_PRESSED_RIGH);
    }
    shot2flow();
    if ((IF_KEY_PRESSED_E && !IF_KEY_PRESSED_CTRL) || Wait(WAIT_E)) // 吊射模式E
    {
        if (Wait(WAIT_E))
        {
            if (Global.Shoot.Hanging_Shot == Hanging_Shot_OPEN)
            {
                shot_close_flag = 0;
            }
            else
            {
                Global.Shoot.Hanging_Shot = Hanging_Shot_OPEN;
                Global.Chssis.mode = SIDEWAYS;
            }
        }
        if (IF_KEY_PRESSED_E && !IF_KEY_PRESSED_CTRL)
            SetWait(WAIT_E);
    }

    if (IF_KEY_PRESSED_CTRL && abs(MOUSE_Z_MOVE_SPEED) > 30) // 加SMALL_PITCH
    {
        if (MOUSE_Z_MOVE_SPEED > 10)
            Global.Gimbal.small_pitch_offset += 0.005;
        else if (MOUSE_Z_MOVE_SPEED < -10)
            Global.Gimbal.small_pitch_offset -= 0.005;
        float SmallPitch_Downlimit;
        float SmallPitch_Uplimit;
        switch (Global.Gimbal.small_pitch_target)
        {
        case level:
            SmallPitch_Downlimit = -5 + SMALL_PITCH_BASE;
            SmallPitch_Uplimit = 5;
            break;
        case rush:
            SmallPitch_Downlimit = -5 + SMALL_PITCH_BASE - SMALL_PITCH_RUSHB;
            SmallPitch_Uplimit = 5 - SMALL_PITCH_RUSHB;
            break;
        case outpost:
            SmallPitch_Downlimit = -5 + SMALL_PITCH_BASE - SMALL_PITCH_OUTPOST;
            SmallPitch_Uplimit = 5 - SMALL_PITCH_OUTPOST;
            break;
        case base:
            SmallPitch_Downlimit = -5 + SMALL_PITCH_BASE - SMALL_PITCH_BASE;
            SmallPitch_Uplimit = 5 - SMALL_PITCH_BASE;
            break;
        default:
            break;
        }
        if (Global.Gimbal.small_pitch_offset > SmallPitch_Uplimit)
            Global.Gimbal.small_pitch_offset = SmallPitch_Uplimit;
        if (Global.Gimbal.small_pitch_offset < SmallPitch_Downlimit)
            Global.Gimbal.small_pitch_offset = SmallPitch_Downlimit;
    }
    if ((IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_Q || Wait(WAIT_CTRL_Q))) // 加射速
    {
        if (Wait(WAIT_CTRL_Q))
        {
            Global.Shoot.shoot_deviation -= 20;
        }
        if (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_Q)
            SetWait(WAIT_CTRL_Q);
    }
    if ((IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_E || Wait(WAIT_CTRL_E))) // 加射速
    {
        if (Wait(WAIT_CTRL_E))
        {
            Global.Shoot.shoot_deviation += 20;
        }
        if (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_E)
            SetWait(WAIT_CTRL_E);
    }
    if (IF_MOUSE_PRESSED_LEFT &&
        Global.Shoot.shoot_mode != CLOSE) // 拨弹电机控制发蛋
        Global.Shoot.tigger_mode = SINGLE;
    else
        Global.Shoot.tigger_mode = TRIGGER_CLOSE;
    if (IF_KEY_PRESSED_B)
        Global.Shoot.ONtigger = tigger_close;
    else
        Global.Shoot.ONtigger = tigger_open;

    if (shoot_gimmbal_flag == 1)
    {
        if (Global.Gimbal.small_pitch_target == rush)
            Gimbal_set_pitch_angle(-SMALL_PITCH_RUSHB);
        else if (Global.Gimbal.small_pitch_target == outpost)
            Gimbal_set_pitch_angle(-SMALL_PITCH_OUTPOST);
        else if (Global.Gimbal.small_pitch_target == base)
            Gimbal_set_pitch_angle(-SMALL_PITCH_BASE);
        else if (Global.Gimbal.small_pitch_target == level)
            Gimbal_set_pitch_angle(0);
    }
}

void shot2flow()
{
    if (shot_close_flag == 0)
    {
        // Global.Gimbal.input.yaw -= Global.Gimbal.Hanging_Shot_err;
        shot_close_flag = 1;
    }
    if (shot_close_flag == 1)
        shot_close_delay++;
    if (shot_close_delay > 500)
    {
        Global.Shoot.Hanging_Shot = Hanging_Shot_CLOSE;
        Global.Chssis.mode = FLOW;
        shot_close_delay = 0;
        shot_close_flag = 2;
    }
}

#endif //(GIMBAL_TYPE == GIMBAL_HANG_SHOT)