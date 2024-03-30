/**
 * @file DAC8568.c
 * @author ninedayhx (1170535490@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "DAC8568.h"

// 零偏校准 mv
float ZeroCode[8] = {0.001f, 0, 0, 0.002f, 0, 0, 0, 0};

void dac8568_init(void)
{
	__HAL_SPI_ENABLE(&hspi3);

	spi_read_write_byte(0Xff);
	// 复位指令 清除寄存器中的值
	dac8568_write_data(SOFTWARE_RESET);	
	// HAL_Delay(100);
	// 先写入寄存器，使其处于中值32767，如果不写入，初始值会变成0
	// 那么通道上电时，会输出负电压
//	dac8568_writeupdate_channelval(0,0.0);
//	dac8568_writeupdate_channelval(1,0.0);
//	dac8568_writeupdate_channelval(2,0.0);
//	dac8568_writeupdate_channelval(3,0.0);
//	// 使能对应通道上电
//	dac8568_powerup_channel(0);
//	dac8568_powerup_channel(1);
//	dac8568_powerup_channel(2);
//	dac8568_powerup_channel(3);
//
//	// 使用内部参考基准
//	dac8568_write_data(EN_INTERNAL_REFERENCE);

}

/**
 * @brief 写数据
 * 
 * @param data 
 */
void dac8568_write_data(uint32_t data)
{
	// 24位字节分成两个16位整形传输，低位数据空余位使用任意数据代替
	/**
		4-bit 前缀位: DB31 ~ DB28 (最高位必须置零，余下Don’t care)
		4-bit控制位	: DB27 ~ DB24 (控制DAC模式)
		4-bit 地址位：DB23~ DB20 (选择DAC通道)
		16-bit数据位: DB19 ~ DB4 (送入DAC DATA Buffer)
		4-bit特征位	: DB3 ~ DB0
	*/
	uint16_t data_h = (uint16_t)(data >> 16);
	uint16_t data_l = (uint16_t)(data & 0x0000FFFF);

	// 同步信号拉低，开始数据传输
	dac8568_sync_l;
	spi_read_write_byte(data_h);
	spi_read_write_byte(data_l);
	// 拉高停止数据传输
	dac8568_sync_h;
}

/**
 * @brief 写指定通道的寄存器，并更新
 * 
 * @param ch 通道数 0-7 对应 A-G
 * @param data 0 ~ 65535
 */
void dac8568_writeupdate_channel(uint8_t ch, uint16_t data)
{
	uint32_t txdata = 0x03000000;
	txdata |= (uint32_t)(ch&0x0F)<<20;
	txdata |= (uint32_t)data<<4;
	
	dac8568_write_data(txdata);
}

/**
 * @brief 写指定通道的电压，并更新，在函数中进行了零偏矫正
 * 
 * @param ch 通道数 0-7 对应 A-G
 * @param val 0-2.0V
 */
void dac8568_writeupdate_channelval(uint8_t ch, float val)
{
	if(val>=2.0) val = 2.0;
	if(val<=0.0) val = 0.0;
	uint16_t data = 32767 + (uint16_t)((val-ZeroCode[ch])/5.0f*32767.0f);
	uint32_t txdata = 0x03000000;
	txdata |= (uint32_t)(ch&0x0F)<<20;
	txdata |= (uint32_t)data<<4;
	
	dac8568_write_data(txdata);
}

/**
 * @brief 写所有通道寄存器，并更新
 * 
 * @param data 
 */
void dac8568_writeupdate_all_channel(uint16_t data)
{
	uint32_t txdata = 0x03F00000;
	
	txdata |= (uint32_t)data<<4;
	
	dac8568_write_data(txdata);
}

/**
 * @brief 写所有通道电压值，并进行更新，注意此函数无法对零偏进行矫正
 * 
 * @param val
 */
void dac8568_writeupdate_all_channelval(float val)
{
	if(val>=2.0) val = 2.0;
	if(val<=0.0) val = 0.0;
	uint16_t data = 32767 + (uint16_t)(val/5.0f*32767.0f);

	uint32_t txdata = 0x03F00000;
	
	txdata |= (uint32_t)data<<4;
	
	dac8568_write_data(txdata);
}

void dac8568_powerup_channel(uint8_t ch){
	static uint8_t channel = 0x00;
	channel |= 1<<(ch-1);
	uint32_t txdata = 0x04000000;
	txdata |= (uint32_t)channel;

	dac8568_write_data(txdata);
}

/**
 * @brief spi传输重新封装
 * 
 * @param txdata 
 * @return uint16_t 
 */
void spi_read_write_byte(uint16_t txdata)
{
    //HAL_SPI_Transmit(&hspi3, (uint8_t *)&txdata, 1, 1000);
    uint16_t rxdata;
    HAL_SPI_TransmitReceive(&hspi3, (uint8_t *)&txdata, (uint8_t *)&rxdata, 1, 1000);
    return rxdata;
}
