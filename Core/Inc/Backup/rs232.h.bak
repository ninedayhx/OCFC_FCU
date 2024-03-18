/*
 * rs232.h
 *
 *  Created on: 2023年4月5日
 *      Author: edgar
 */

#ifndef INC_RS232_H_
#define INC_RS232_H_

#include "stm32g4xx_hal.h" //HAL库文件声明

extern UART_HandleTypeDef huart1;//声明USART1的HAL库结构体

void RS232_1_printf (char *fmt, ...);
void RS232_2_sendModbusFrame(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4,
                             uint8_t byte5, uint8_t byte6, uint8_t byte7, uint8_t byte8);
void process_RS232_uart1_command(void);
void process_RS232_uart2_command(void);

#endif /* INC_RS232_H_ */
