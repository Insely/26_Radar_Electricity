#ifndef __AUTO_CONTROL_H__
#define __AUTO_CONTROL_H__

#include "stdint.h"

#pragma pack(1)

typedef struct
{
    struct
    {
        uint8_t sof;
        uint8_t crc8;
    } FrameHeader; // 2
    struct
    {
        float curr_yaw;
        float curr_pitch;
        float curr_omega;
        uint8_t state;// state 0�Ǵ� 1�Ǵ�ǰ��վ 2�Ǵ�С�� 3�Ǵ��?
        uint8_t autoaim; // autoaim�Ǹ�0�ǲ������� 1�ǿ�����
        uint8_t enemy_color;// 0Ϊ��ɫ��1Ϊ��ɫ
    } To_minipc_data; // 15
    struct
    {
        uint16_t crc16;
    } FrameTailer;//2
    uint8_t enter;//1
} STM32_data_t;

typedef struct
{
    struct
    {
        uint8_t sof;
        uint8_t crc8;
    } FrameHeader; // 2
    struct
    {
        float shoot_yaw;
        float shoot_pitch;
        uint8_t fire;      // 发射信号
        uint8_t target_id; // 目标ID,UI显示�?
    } from_minipc_data;    // 10
    struct
    {
        uint16_t crc16;
    } FrameTailer;
} MINIPC_data_t;

/* Radar_Vision 发送的数据结构�? (与视觉端 VisionData 对齐) */
typedef struct __attribute__((packed))
{
    uint8_t  header;       // 帧头 0xA5
    float    yaw_error;    // 水平像素偏差
    float    pitch_error;  // 垂直像素偏差
    uint8_t  at_center;    // 是否到达中心 (0/1)
    uint8_t  allow_fire;   // 是否允许发射 (0/1)
} VisionData_t;

#pragma pack(4)

void STM32_to_MINIPC(float yaw,float pitch,float omega);
void decodeMINIPCdata(VisionData_t *target, uint8_t *buff, uint16_t len);
int decodeVisionData(VisionData_t *target, unsigned char buff[], unsigned int len);
void Vision_to_STM32(void);
void Auto_control();
void MINIPC_to_STM32();

extern MINIPC_data_t fromMINIPC;
extern STM32_data_t toMINIPC;

#endif