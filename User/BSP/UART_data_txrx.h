/**
 * @file UART_data_txrx.h
 * @author set
 \home
 * @brief �������ݷ��ͽ���
 * @version 0.1
 * @date 2022-11-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __UART_DATA_TXRX__
#define __UART_DATA_TXRX__

#include "stm32h7xx.h" 
#include "stm32h7xx_hal_uart.h"
#include "main.h"
#include "usart.h"
#include "struct_typedef.h"

#define UART_BUFFER_SIZE 256//���ڻ�������С
//�������ݽṹ��

typedef PACKED_STRUCT()
{
  UART_HandleTypeDef *huart;
  DMA_HandleTypeDef *hdma_usart_rx;
  DMA_HandleTypeDef *hdma_usart_tx;

  uint8_t rev_data[UART_BUFFER_SIZE];
  HAL_StatusTypeDef Uart_status;
} transmit_data;

#include "Auto_control.h"

extern transmit_data UART7_data;
extern transmit_data UART10_data;
extern transmit_data UART1_data;
extern VisionData_t vision_data;


//�ⲿ����
void uart_init(void);                                                                                                                         //��ʼ��
void UART_DMA_rxtx_start(transmit_data *data, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, DMA_HandleTypeDef *hdma_usart_tx); //��ʼ��                                                                                               //�����ж�
void UART_send_data(transmit_data uart, uint8_t data[], uint16_t size);                                                                       //����

//�ڲ�����
void UART_DMA_rxtx_start(transmit_data *data, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx, DMA_HandleTypeDef *hdma_usart_tx); //��ʼ��
#endif
// end of file
