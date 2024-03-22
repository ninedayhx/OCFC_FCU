/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
#define UART_RX_BUFFER_SIZE	254
#define UART1_RX_BUFFER_SIZE  100
#define UART2_RX_BUFFER_SIZE  100
#define UART3_RX_BUFFER_SIZE  64

typedef struct {
	uint8_t rx_buffer[UART_RX_BUFFER_SIZE];   // 接收数据缓存数组
	uint8_t rx_len;             			// 接收�???帧数据的长度
	bool recv_end_flag;  				//�???帧数据接收完成标�???
} UART_ReceiveTypeDef;

#define USART1_REC_LEN 255//RS232_1,�???大发送字节数
#define USART1_SEN_LEN 255//RS232_1,�???大接收字节数

#define USART2_REC_LEN 255//RS232_2,�???大发送字节数
#define USART2_SEN_LEN 255//RS232_2,�???大接收字节数

#define USART3_REC_LEN 328//RS485 �???大发送字节数
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

