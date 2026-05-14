#include "remote_control.h"
#include "Global_status.h"
#include "Chassis_helm.h"
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
#include "ui.h"

#include "cmsis_os2.h"

#define RC_JOYSTICK_MAX 660.0f
#define CHASSIS_MAX_SPEED_X 2.5f   //底盘x轴最大物理速度  m/s
#define CHASSIS_MAX_SPEED_Y 2.5f   //底盘y轴最大物理速度  m/s
static uint32_t mouse_l_press_start_time = 0;  //记录按下的起始时间戳
static uint8_t is_mouse_l_pressed = 0;         //记录鼠标左键的按压状态
const uint32_t LONG_PRESS_MS = 250;            //长按判定阈值
static uint8_t look_aside_state = 0;           //X键侧视状态 0:正常 1:+90° 2:+270°
static float look_aside_saved_yaw = 0;         //侧视前保存的yaw角度

/*----------------------------------- 按键消抖 --------------------------------------*/

int16_t wait_time[SIZE_OF_WAIT] = {0}; // 键盘消抖用时间

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
    wait_time[key] = 100;
}


/*----------------------------------- 遥控数据更新 --------------------------------------*/
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
        Power_TurnOff(power2);
    else
        Power_TurnOn(power2);
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
 * @brief 遥控数据来源于FS_I6X遥控器
 *
 */
void FSI6XtoRCdata()
{ 
   RC_data.rc.ch[0]=FSI6X_data.CH1;
   RC_data.rc.ch[1]=FSI6X_data.CH2;
   RC_data.rc.ch[2]=FSI6X_data.CH4;
   RC_data.rc.ch[3]=FSI6X_data.CH3;

   for(int i = 0 ; i<4 ; i++) // 死区判断
   {
    if(fabs(RC_data.rc.ch[i]) < 10)
    RC_data.rc.ch[i] = 0;
   }
   
   if(FSI6X_data.CH8==FS_DOWN)//LOCK
   {
    RC_data.rc.s[0]=RC_SW_DOWN;
    RC_data.rc.s[1]=RC_SW_DOWN;
   }
   else if (FSI6X_data.CH8==FS_UP)
   {
     if (FSI6X_data.CH7==FS_MID && FSI6X_data.CH6 == FS_UP)// 正小陀螺
     {
        RC_data.rc.s[0]=RC_SW_UP ;
        RC_data.rc.s[1]=RC_SW_MID;
     }
     else if (FSI6X_data.CH7==FS_DOWN && FSI6X_data.CH6 == FS_UP)// 逆小陀螺
     {
        RC_data.rc.s[1]=RC_SW_DOWN;
        RC_data.rc.s[0]=RC_SW_MID;
     }
     else if(FSI6X_data.CH6 == FS_DOWN && FSI6X_data.CH7 == FS_UP)// 开启摩擦轮
     {
       RC_data.rc.s[1]=RC_SW_UP; 
       RC_data.rc.s[0]=RC_SW_MID;
       
     }
     else
     {
       RC_data.rc.s[1]=RC_SW_MID; 
       RC_data.rc.s[0]=RC_SW_MID;
     }
     if(FSI6X_data.CH5 == FS_DOWN) // 开火
       {
        RC_data.rc.ch[4] = 660;
       }
       else if(FSI6X_data.CH5 == FS_UP)
       {
        RC_data.rc.ch[4] = 0;
       }
   }
   FSI6X_data.online--;
   RC_data.online = FSI6X_data.online;
}

/**
 * @brief 根据数据来源更新遥控数据
 *
 */
void RCdata_Updater()
{
    if (DT7_data.online >= 0)
      DT7toRCdata();
    else if (FSI6X_data.online >= 0)
      FSI6XtoRCdata();
    else if (VT13_data.online >= 0)
      VT13toRCdata(); 
}


/*----------------------------------- 遥控器控制逻辑 --------------------------------------*/
/**
 * @brief 遥控器控制
 *
 */
void RC_Controller()
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
        Global.Chassis.mode = SPIN_P;
    else if (RC_data.rc.s[1]==RC_SW_DOWN && RC_data.rc.s[0]==RC_SW_MID ) // 左下,右中||右上，負小陀螺
        Global.Chassis.mode = SPIN_N;
    else
        Global.Chassis.mode = FLOW;
    float target_x = (RC_data.rc.ch[1]/ RC_JOYSTICK_MAX) * CHASSIS_MAX_SPEED_X;
    float target_y = (RC_data.rc.ch[0]/ RC_JOYSTICK_MAX) * CHASSIS_MAX_SPEED_Y;
    Chassis_SetX(target_x);
    Chassis_SetY(target_y);
        //Chassis_SetX(RC_data.rc.ch[1] / 2.0f);
    //Chassis_SetY(RC_data.rc.ch[0] / 2.0f);
    /*云台控制*/
    if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.fire == -1) && Global.Gimbal.mode == NORMAL)
    {
        Gimbal_SetPitchAngle(Global.Gimbal.input.pitch + RC_data.rc.ch[3] / 2000.0f);
        Gimbal_SetYawAngle(Global.Gimbal.input.yaw - RC_data.rc.ch[2] / 1500.0f);
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
    if (RC_data.rc.ch[4] >= 300 &&RC_data.rc.ch[4] <= 660 &&Global.Shoot.shoot_mode != CLOSE &&
        (Global.Auto.mode == NONE ||Global.Auto.input.fire == 1)) // 滚轮最下头，高速发弹，若自瞄打开，发弹标志位置1允许发弹
        Global.Shoot.tigger_mode = HIGH;
    /*else if (RC_data.rc.ch[4] >= 50 &&RC_data.rc.ch[4] <= 300 &&Global.Shoot.shoot_mode != CLOSE &&
             (Global.Auto.mode == NONE ||Global.Auto.input.fire == 1)) // 滚轮中部，低速发弹,若自瞄打开，发弹标志位置1允许发弹
        Global.Shoot.tigger_mode = LOW;*/
    else if (RC_data.rc.ch[4] > 660 &&Global.Shoot.shoot_mode != CLOSE)
    {
        Global.Shoot.shoot_mode = DEBUG_SHOOT;
        Global.Shoot.tigger_mode = DEBUG_SHOOT;
    }
    else if(RC_data.rc.ch[4]<=-300 &&RC_data.rc.ch[4]>=-660 &&Global.Shoot.shoot_mode != CLOSE &&
       (Global.Auto.mode == NONE || Global.Auto.input.fire == 1))
    {
        Global.Shoot.tigger_mode = SINGLE;
    }
    else
        Global.Shoot.tigger_mode = TRIGGER_CLOSE;
}

/*----------------------------------- 键鼠控制逻辑 --------------------------------------*/
/**
 * @brief 遥控器控制
 *
 */
void Keyboard_MouseController(void)
{
    if (Global.Control.mode != KEY)
        return;
    /*底盘控制*/
    if (IF_KEY_PRESSED_W)
        Chassis_SetX(Global.Cap.speed);
    if (IF_KEY_PRESSED_S)
        Chassis_SetX(-Global.Cap.speed);
    if (IF_KEY_PRESSED_D)
        Chassis_SetY(Global.Cap.speed);
    if (IF_KEY_PRESSED_A)
        Chassis_SetY(-Global.Cap.speed);
    if (!IF_KEY_PRESSED_W && !IF_KEY_PRESSED_S)
        Chassis_SetX(0);
    if (!IF_KEY_PRESSED_A && !IF_KEY_PRESSED_D)
        Chassis_SetY(0);
    if (IF_KEY_PRESSED_SHIFT) // 超电开关
    {
        Global.Cap.mode = FULL;
        Global.Cap.speed = 2.5f;
    }
    else
    {
        Global.Cap.mode = Not_FULL;
        Global.Cap.speed = 1.5f;
    }
    
    if (IF_KEY_PRESSED_Q || Wait(WAIT_Q)) // 小陀螺开关
    {
        if (Wait(WAIT_Q)) // 消抖结束触发
        {
            if (Global.Chassis.mode != SPIN_P)
                Global.Chassis.mode = SPIN_P;
            else
                Global.Chassis.mode = FLOW;
        }
        if (IF_KEY_PRESSED_Q)
            SetWait(WAIT_Q);
    }
    if (IF_KEY_PRESSED_Z) // UI开关
    {
        ui_init_helm();
    }
    if (IF_KEY_PRESSED_X || Wait(WAIT_X)) // 侧视开关
    {
        if (Wait(WAIT_X))
        {
            switch (look_aside_state)
            {
            case 0: // 正常→转90°
                look_aside_saved_yaw = Global.Gimbal.input.yaw;
                Gimbal_SetYawAngle(look_aside_saved_yaw + 90.0f);
                Global.Chassis.mode = NO_FOLLOW;
                look_aside_state = 1;
                break;
            case 1: // +90°→再转180°(到+270°)
                Gimbal_SetYawAngle(look_aside_saved_yaw + 270.0f);
                look_aside_state = 2;
                break;
            case 2: // +270°→再转90°回到原点
                Gimbal_SetYawAngle(look_aside_saved_yaw);
                Global.Chassis.mode = FLOW;
                look_aside_state = 0;
                break;
            }
        }
        if (IF_KEY_PRESSED_X)
            SetWait(WAIT_X);
    }
    
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
        Gimbal_SetPitchAngle(Global.Gimbal.input.pitch + MOUSE_Y_MOVE_SPEED * PITCH_SENSITIVITY);
        Gimbal_SetYawAngle(Global.Gimbal.input.yaw - MOUSE_X_MOVE_SPEED * YAW_SENSITIVITY);
    }
    else if ((Global.Auto.input.Auto_control_online <= 0 || Global.Auto.mode == NONE || Global.Auto.input.fire == -1) && Global.Gimbal.mode == SHOOT)
    {
        Gimbal_SetPitchAngle(Global.Gimbal.input.pitch + MOUSE_Y_MOVE_SPEED * PITCH_SENSITIVITY);
        Gimbal_SetYawAngle(Global.Gimbal.input.yaw - MOUSE_X_MOVE_SPEED * YAW_SENSITIVITY);
    }
    /*相机重启*/
    if (IF_KEY_PRESSED_B)
        Power_TurnOff(power2);
    else
        Power_TurnOn(power2);
    /*发射机构控制*/
    if (IF_KEY_PRESSED_R || Wait(WAIT_R)) // 摩擦轮开关
    {
        if (Wait(WAIT_R)) // 消抖
            Global.Shoot.shoot_mode = !Global.Shoot.shoot_mode;
        if (IF_KEY_PRESSED_R)
            SetWait(WAIT_R);
    }
    // 自瞄模式下C键切换拨弹频率
    static uint8_t auto_trigger_hz = 0; // 0:AUTO_1HZ, 1:AUTO_3HZ, 2:AUTO_5HZ
    if (IF_MOUSE_PRESSED_RIGH)
    {
        if (IF_KEY_PRESSED_C || Wait(WAIT_C))
        {
            if (Wait(WAIT_C))
            {
                auto_trigger_hz++;
                if (auto_trigger_hz > 2)
                    auto_trigger_hz = 0;
            }
            if (IF_KEY_PRESSED_C)
                SetWait(WAIT_C);
        }
    }

    if (IF_MOUSE_PRESSED_LEFT &&
        Global.Shoot.shoot_mode != CLOSE &&
        (Global.Auto.mode == NONE ||
         Global.Auto.input.fire == 1 ||
         Global.Auto.input.fire == -1)) // 拨弹电机控制
    {
        if (Global.Auto.mode != NONE)
        {
            // 自瞄模式下使用C键选择的频率
            if (auto_trigger_hz == 0)
                Global.Shoot.tigger_mode = AUTO_1HZ;
            else if (auto_trigger_hz == 1)
                Global.Shoot.tigger_mode = AUTO_3HZ;
            else
                Global.Shoot.tigger_mode = AUTO_5HZ;
        }
        else
            Global.Shoot.tigger_mode = HIGH;
    }
    else
        Global.Shoot.tigger_mode = TRIGGER_CLOSE;
    // Ctrl键切换LOCK/KEY模式
    if (IF_KEY_PRESSED_CTRL || Wait(WAIT_CTRL))
        {
            if (Wait(WAIT_CTRL))
            {
                if (Global.Control.mode == KEY)
                    Global.Control.mode = LOCK;
                else if (Global.Control.mode == LOCK)
                    Global.Control.mode = KEY;
            }
            if (IF_KEY_PRESSED_CTRL)
                SetWait(WAIT_CTRL);
        }
}


/*------------------------------------ Task -------------------------------------*/

/**
 * @brief          遥控任务
 * @param          none
 * @retval         none
 */
void Remote_Tasks(void)
{
	// 遥控数据更新
	RCdata_Updater();
    // 遥控器控制
    RC_Controller();
    // 键鼠控制
    Keyboard_MouseController();
}