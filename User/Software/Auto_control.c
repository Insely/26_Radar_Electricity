#include "Auto_control.h"
#include "Global_status.h"
#include "Gimbal.h"

#include "IMU_updata.h"
#include "referee_system.h"
#include "UART_data_txrx.h"

#include "CRC8_CRC16.h"
#include "string.h"

STM32_data_t toMINIPC;
MINIPC_data_t fromMINIPC;

uint8_t data[128];
uint8_t rx_data[100];

void decodeMINIPCdata(VisionData_t *target, uint8_t *buff, uint16_t len)
{
    // 1. 长度检查：结构体现在是 7 字节
    if (len < sizeof(VisionData_t)) return;

    // 2. 搜帧逻辑：在整个收到的包里找 0xA5
    for (int i = 0; i <= (len - sizeof(VisionData_t)); i++) 
    {
        if (buff[i] == 0xA5) 
        {
            // 找到帧头后，把后续数据拷贝进结构体
            memcpy(target, &buff[i], sizeof(VisionData_t));
            break; // 解码完成
        }
    }
}

// void decodeMINIPCdata(MINIPC_data_t *target, unsigned char buff[], unsigned int len)
// {
//     memcpy(target, buff, len);
// }

/**
 * @brief 从缓冲区中搜索帧头0xA5并解码VisionData
 * @return 0: 成功, -1: 失败
 */
int decodeVisionData(VisionData_t *target, unsigned char buff[], unsigned int len)
{
    // 在缓冲区中搜索帧头
    for (unsigned int i = 0; i < len; i++)
    {
        if (buff[i] == 0xA5 && (len - i) >= sizeof(VisionData_t))
        {
            memcpy(target, &buff[i], sizeof(VisionData_t));
            return 0;
        }
    }
    return -1;
}

int encodeSTM32(STM32_data_t *target, unsigned char tx_buff[], unsigned int len)
{
    memcpy(tx_buff, target, len);
    return 0;
}

/**
 * @brief ����λ�����������������?
 * 
 * @param yaw yaw�ᵱǰ�Ƕȣ����ȣ�
 * @param pitch pitch�ᵱǰ�Ƕȣ����ȣ�
 * @param omega yaw�ᵱǰ���ٶȣ�rad/s��
 */
void STM32_to_MINIPC(float yaw,float pitch,float omega)
{
    toMINIPC.FrameHeader.sof = 0xA5;
    toMINIPC.FrameHeader.crc8 = 0x00;
    toMINIPC.To_minipc_data.curr_pitch = pitch;//IMU_data.AHRS.pitch;
    toMINIPC.To_minipc_data.curr_yaw = yaw;//IMU_data.AHRS.yaw;
    toMINIPC.To_minipc_data.curr_omega = omega;//cos(IMU_data.AHRS.pitch) * IMU_data.gyro[2] - sin(IMU_data.AHRS.pitch) * IMU_data.gyro[1];
    toMINIPC.To_minipc_data.autoaim = 1;
    if (Referee_data.robot_id >= 100)
        toMINIPC.To_minipc_data.enemy_color = 1;
    else
        toMINIPC.To_minipc_data.enemy_color = 0;
    toMINIPC.To_minipc_data.state = 0;
    toMINIPC.FrameTailer.crc16 = get_CRC16_check_sum((uint8_t *)&toMINIPC.FrameHeader.sof, 17, 0xffff);
    toMINIPC.enter = 0x0A;
    encodeSTM32(&toMINIPC, data, sizeof(STM32_data_t));
    // VirCom_send(data, sizeof(STM32_data_t));
    UART_send_data(UART1_data, data, sizeof(STM32_data_t));
}

void MINIPC_to_STM32()
{
    if (fabs(fromMINIPC.from_minipc_data.shoot_yaw - IMU_data.AHRS.yaw) > PI / 2.0f) // ����㴦��?
    {
        if (fromMINIPC.from_minipc_data.shoot_yaw > PI / 2.0f)
            fromMINIPC.from_minipc_data.shoot_yaw -= 2 * PI;
        else if (fromMINIPC.from_minipc_data.shoot_yaw < -PI / 2.0f)
            fromMINIPC.from_minipc_data.shoot_yaw += 2 * PI;
    }
    Global.Auto.input.shoot_pitch = fromMINIPC.from_minipc_data.shoot_pitch - IMU_data.AHRS.pitch;
    Global.Auto.input.shoot_yaw = fromMINIPC.from_minipc_data.shoot_yaw - IMU_data.AHRS.yaw;
    Global.Auto.input.fire = fromMINIPC.from_minipc_data.fire;
    Global.Auto.input.target_id = fromMINIPC.from_minipc_data.target_id;
    if(fromMINIPC.from_minipc_data.shoot_pitch==0&&fromMINIPC.from_minipc_data.shoot_yaw==0)
        Global.Auto.input.fire = -1;
}

/**
 * @brief Radar_Vision数据解析，直接存储yaw/pitch的error�?
 */
void Vision_to_STM32(void)
{
    // 直接使用视觉端发来的像素error
    Global.Auto.input.shoot_yaw   = vision_data.yaw_error;
    Global.Auto.input.shoot_pitch = vision_data.pitch_error;
    Global.Auto.input.fire = vision_data.allow_fire ? 1 : -1;

    // yaw_error和pitch_error都为0时视为未识别到目�?
    if (vision_data.yaw_error == 0 && vision_data.pitch_error == 0)
        Global.Auto.input.fire = -1;
}

void Auto_control()
{
    // 雷达自瞄模式: error值已在Gimbal_control()中直接使用，无需在此转换为绝对角�?
    // Gimbal_control()的自瞄分支直接读�? Global.Auto.input.shoot_yaw/shoot_pitch (像素error)
}

