/*
 * rs485.h
 *
 *  Created on: Mar 24, 2023
 *      Author: edgar
 */

#ifndef INC_RS485_H_
#define INC_RS485_H_


#include "stm32g4xx_hal.h" //HAL库文件声明

extern UART_HandleTypeDef huart3;

void RS485_printf (char *fmt, ...);  //RS485发送

void RS485_3_sendModbusFrame(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4,
                             uint8_t byte5, uint8_t byte6, uint8_t byte7, uint8_t byte8);

void queryFlowMeter();//问询流量计
void resetFlowMeter();//流量计清空累计值

void process_RS485_uart3_command(void);

#endif /* INC_RS485_H_ */
