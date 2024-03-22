/**
 * @file DAC8568.h
 * @author ninedayhx (1170535490@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef INC_DAC8568_H_
#define INC_DAC8568_H_

#include "main.h"
#include "spi.h"

extern int16_t ZeroCode[8];

#define dac8568_range       10000.0	
#define dac8568_shifting    5000.0	

#define dac8568_sync_l  HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET)
#define dac8568_sync_h  HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET)

void dac8568_init(void);
void dac8568_write_data(uint32_t data);
void dac8568_write_channel(uint8_t ch, uint16_t data);
void dac8568_write_all_channel(uint16_t data);
void spi_read_write_byte(uint16_t txdata);

#endif /* INC_DAC8568_H_ */
