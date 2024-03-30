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

extern float ZeroCode[8];

#define dac8568_range       10000.0	
#define dac8568_shifting    5000.0	

#define dac8568_sync_l  HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_RESET)
#define dac8568_sync_h  HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET)

#define SOFTWARE_RESET          0x07000000
#define POWER_UP_ALL            0x040000FF
#define POWER_DOWN_ALL_1k       0x040001FF
#define POWER_DOWN_ALL_100k     0x040002FF
#define POWER_DOWN_ALL_HIGHZ    0x040003FF
#define EN_INTERNAL_REFERENCE   0x090A0000
#define CLEAR_TO_ZEROSCALE      0x05000000
#define CLEAR_TO_MIDSCALE       0x05000001
#define CLEAR_TO_FULLSCALE      0x05000002

void dac8568_init(void);
void dac8568_write_data(uint32_t data);
void dac8568_writeupdate_channel(uint8_t ch, uint16_t data);
void dac8568_writeupdate_channelval(uint8_t ch, float val);
void dac8568_writeupdate_all_channel(uint16_t data);
void dac8568_writeupdate_all_channelval(float val);
void dac8568_powerup_channel(uint8_t ch);
void spi_read_write_byte(uint16_t txdata);

#endif /* INC_DAC8568_H_ */
