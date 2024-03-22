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

// 零偏校准
int16_t ZeroCode[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void dac8568_init(void)
{
	__HAL_SPI_ENABLE(&hspi3);

	spi_read_write_byte(0Xff);
	// 复位指令
	dac8568_write_data(0x07000000);	
	//HAL_Delay(100);
	// 全部通道上电
	dac8568_write_data(0x040000FF);	
	// 使用内部参考基准
	dac8568_write_data(0x090A0000);	
	// 初始化为0V
	dac8568_write_all_channel(0.5*65535);
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
 * @brief 更新指定通道的数据
 * 
 * @param ch 通道数 0-7 对应 A-G
 * @param data 0 ~ 65535
 */
void dac8568_write_channel(uint8_t ch, uint16_t data)
{
	uint32_t txdata = 0x03000000;
	txdata |= (uint32_t)(ch&0x0F)<<20;
	txdata |= (uint32_t)data<<4;
	
	dac8568_write_data(txdata);
}

/**
 * @brief 直接配置所有通道
 * 
 * @param data 
 */
void dac8568_write_all_channel(uint16_t data)
{
	uint32_t txdata = 0x03F00000;
	
	txdata |= (uint32_t)data<<4;
	
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
