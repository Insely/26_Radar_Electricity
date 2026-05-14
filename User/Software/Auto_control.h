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
        uint8_t state;// state 0是打车 1是打前哨站 2是打小符 3是打符
        uint8_t autoaim; // autoaim那个0是不用自瞄 1是开自瞄
        uint8_t enemy_color;// 0为蓝色，1为红色
    } To_minipc_data; // 15
    struct
    {
        uint16_t crc16;
    } FrameTailer;// 2
    uint8_t enter;// 1
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
        uint8_t fire;      // 发弹信号
        uint8_t target_id; // 目标ID,UI显示用
    } from_minipc_data;    // 15
    struct
    {
        uint16_t crc16;
    } FrameTailer;
} MINIPC_data_t;

/* Radar_Vision 鍙戦�佺殑鏁版嵁缁撴瀯浣? (涓庤瑙夌 VisionData 瀵归綈) */
typedef struct __attribute__((packed))
{
    uint8_t  header;       // 甯уご 0xA5
    float    yaw_error;    // 姘村钩鍍忕礌鍋忓樊
    float    pitch_error;  // 鍨傜洿鍍忕礌鍋忓樊
    uint8_t  target_detected;  // 4. 是否识别到目标 (0/1)

} VisionData_t;

#pragma pack(4)

void STM32_to_MINIPC(float yaw,float pitch,float omega);
void decodeMINIPCdata(VisionData_t *target, uint8_t *buff, uint16_t len);
void Auto_Control();
void MINIPC_to_STM32();

extern MINIPC_data_t fromMINIPC;
extern STM32_data_t toMINIPC;

#endif