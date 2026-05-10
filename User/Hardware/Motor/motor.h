#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "stdint.h"

#include "User_math.h"
#include "robot_param.h"
#include "CAN_receive_send.h"

// CANͨ������
#define QUANTITY_OF_CAN 3       // �����õ�can���������������õĶ�����ˣ���������3����ˣ�


#if (USE_DJIMotor == 1)

#define QUANTITY_OF_DJIMOTOR 11 // ���ص������һ·can�����õĵ��
// �󽮵������
#define ECD_TO_ANGEL_DJI 0.043945f //(360/8192),��������ֵת��Ϊ�Ƕ���
#define DJIMOTOR_T_A 0.3           // 3508ת�س���
#define ECD_MAX 8192.0f            // ���������ֵ
#define M3508_P WHEEL_RATIO        // M3508������ٱȣ���ͬ���ļ��ٱȲ�ͬ�������ڲ��������ļ������ã�
#define M2006_P 36.0f              // M2006������ٱ�
#define MAX_CURRENT 16384          // M2006+M3508������ 20A / MAX_CURRENT
#define MAX_6020_VOL 30000         // 6020����ѹ 24V / MAX_6020_VOL

/* DJImotorCAN send and receive ID */
typedef enum
{
    CAN_20063508_1_4_send_ID = 0x200,

    CAN_20063508_5_8_send_ID = 0x1FF,

    CAN_6020_1_4_send_ID = 0x1FF,

    CAN_6020_5_7_send_ID = 0x2FF,

    CAN_ID1 = 0x201,

    DJI_CAN_1 = 0,

    DJI_CAN_2 = 1,

    DJI_CAN_3 = 2,

    CAN_20063508_1_4_ID = 3,

    CAN_20063508_5_8_ID = 4,

    CAN_6020_1_4_ID = 5,

    CAN_6020_5_7_ID = 6,
} DJIcan_send_id_e;

// 6020�� CAN_1_5(��ӦID1) �� CAN_1_6020_7����ӦID7��
// 3508/2006�� CAN_1_1(��ӦID1) �� CAN_1_8����ӦID8��
// ��Ȼ��6020��3508/2006�����ص�ID,����ʱ��ע��
typedef enum
{
    CAN_1_1 = 0,  // 0
    CAN_1_2,      // 1
    CAN_1_3,      // 2
    CAN_1_4,      // 3
    CAN_1_5,      // 4
    CAN_1_6,      // 5
    CAN_1_7,      // 6
    CAN_1_8,      // 7
    CAN_1_6020_5, // 8
    CAN_1_6020_6, // 9
    CAN_1_6020_7, // 10

    CAN_2_1, // 11
    CAN_2_2, // 12
    CAN_2_3,
    CAN_2_4,
    CAN_2_5, // 15
    CAN_2_6,
    CAN_2_7, // 17
    CAN_2_8,
    CAN_2_6020_5,
    CAN_2_6020_6,
    CAN_2_6020_7, // 21

    CAN_3_1, // 22
    CAN_3_2, // 23
    CAN_3_3,
    CAN_3_4,
    CAN_3_5,
    CAN_3_6,
    CAN_3_7,
    CAN_3_8,
    CAN_3_6020_5,
    CAN_3_6020_6,
    CAN_3_6020_7, // 32
    DJI_MOTOR_NUM,
} DJIcan_id;

typedef struct
{
    // ��������
    int16_t set; // �趨�ĵ��� / ��ѹ
    Motor_Type_e Motor_type;

    // ԭʼ����
    uint16_t ecd;          // ��������ֵ
    int16_t speed_rpm;     // ת��RPM
    int16_t given_current; // ʵ��ת�ص���
    uint8_t temperate;     // �¶ȣ���ȡ������
    uint16_t last_ecd;     // ��һ�α���������ֵ

    // ��������
    long long ecd_cnt;  // ������������
    double angle_cnt;   // ת�����ܽǶ� degree
    double angle_zero;  // ������0��Ƕ� degree
    double angle;       // -180~180 degree
    double round_speed; // ����ת�� rpm
} DJI_motor_data_s;

void DJIMotor_init(Motor_Type_e motor_type, DJIcan_id motor_id);
void DJIMotor_setzero(double zero_angle, DJIcan_id motor_id);
void DJIMotor_set(int16_t val, DJIcan_id motor_id);
void DJIMotor_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);
void DJIMotor_send_current(DJIcan_send_id_e CAN_Send_ID, DJIcan_send_id_e CAN_Type);
DJI_motor_data_s DJIMotor_get_data(DJIcan_id motor_id);

#endif // USE_DJIMotor



#if (USE_DMMotor == 1)

#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"

// ÿ��CAN���ߵĵ������
#define QUANTITY_OF_DMMOTOR 6

typedef enum
{
    DM_CAN_1_1 = 0,  // 0
    DM_CAN_1_2,      // 1
    DM_CAN_1_3,      // 2
    DM_CAN_1_4,      // 3
    DM_CAN_1_5,      // 4
    DM_CAN_1_6,      // 5

    DM_CAN_2_1, // 6
    DM_CAN_2_2, // 7
    DM_CAN_2_3,
    DM_CAN_2_4,
    DM_CAN_2_5, // 
    DM_CAN_2_6,

    DM_CAN_3_1, // 
    DM_CAN_3_2, // 
    DM_CAN_3_3,
    DM_CAN_3_4,
    DM_CAN_3_5,
    DM_CAN_3_6,
    DM_MOTOR_NUM,

} DMcan_id;

// ���������ݽṹ
typedef struct
{
    float T;
    float W;
    float Pos;
    
    DM_motor_t motor_data;
} DM_motor_data_s;

void DMMotor_init(Motor_Type_e motor_type,DMcan_id motor_id);
void DMMotor_set(DMcan_id motor_id, float pos, float vel, float tor, float kp, float kd);
void DMMotor_setzero(double zero_angle, DMcan_id motor_id);
int DMMotor_send_ctrl(DMcan_id motor_id);
void DMMotor_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);
DM_motor_data_s DMMotor_get_data(DMcan_id motor_id);

extern DM_motor_data_s DM_Motor_data[QUANTITY_OF_CAN][6];

#endif // USE_DAMIAO_MOTOR


#if (USE_LZMotor == 1)
#include "LZ_motor_driver.h"

// ÿ��CAN���ߵĵ������
#define QUANTITY_OF_LZMOTOR 6

// ���IDö�٣�֧��3·CAN��ÿ·6�������
typedef enum {
    LZ_CAN_1_1 = 0,
    LZ_CAN_1_2,
    LZ_CAN_1_3,
    LZ_CAN_1_4,
    LZ_CAN_1_5,
    LZ_CAN_1_6,
    
    LZ_CAN_2_1,
    LZ_CAN_2_2,
    LZ_CAN_2_3,
    LZ_CAN_2_4,
    LZ_CAN_2_5,
    LZ_CAN_2_6,
    
    LZ_CAN_3_1,
    LZ_CAN_3_2,
    LZ_CAN_3_3,
    LZ_CAN_3_4,
    LZ_CAN_3_5,
    LZ_CAN_3_6,
    
    LZ_MOTOR_NUM
} LZ_Motor_ID_t;

void LZMotor_init(LZ_Motor_ID_t motor_id);
void LZMotor_enable(LZ_Motor_ID_t motor_id);
void LZMotor_disable(LZ_Motor_ID_t motor_id);
void LZMotor_set_mode(LZ_Motor_ID_t motor_id, LZ_Mode_t mode);
void LZMotor_set_params(LZ_Motor_ID_t motor_id, float pos, float vel, float tor, float kp, float kd, float current_limit);
void LZMotor_send_command(LZ_Motor_ID_t motor_id);
void LZMotor_decode_candata(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data);

// ��ȡ�������ָ��
LZ_Motor_t* LZMotor_get(LZ_Motor_ID_t motor_id);

#endif // USE_LINGZU_MOTOR
#endif // !__MOTOR_H__

