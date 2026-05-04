/*
 * @Author: hao hao@qlu.edu.cn
 * @Date: 2025-08-31 21:36:57
 * @LastEditors: hao && (hao@qlu.edu.cn)
 * @LastEditTime: 2025-10-03 10:06:39
 * @FilePath: \Season-26-Code\User\BSP\CAN_receive_send.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __CAN_RECEIVE_SEND_H__
#define __CAN_RECEIVE_SEND_H__

//#include "cover_headerfile_h.h"
#include "fdcan.h"
extern FDCAN_HandleTypeDef* get_can_handle(uint8_t can_bus);

extern void can_init(void);

extern uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
extern uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan,	FDCAN_RxHeaderTypeDef *fdcan_RxHeader, uint8_t *buf);

extern void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);


#endif /* __CAN_RECEIVE_SEND_H__ */






