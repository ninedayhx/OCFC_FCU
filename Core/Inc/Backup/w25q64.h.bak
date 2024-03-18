/*
 * w25q64.h
 *
 *  Created on: 2023年8月27日
 *      Author: edgar
 */

#ifndef INC_W25Q64_H_
#define INC_W25Q64_H_

#include "spi.h"

/*句柄重命名*/
#define W25Q_SPI					hspi1

/*片选引脚定义与函数调用*/
#define W25Q_CS_Pin 			GPIO_PIN_4
#define W25Q_CS_Port			GPIOA

#define W25Q_CS_Level(_CS_STATE__)		HAL_GPIO_WritePin(W25Q_CS_Port, W25Q_CS_Pin, (GPIO_PinState)_CS_STATE__)

//#define W25Q_CS_Level(_CS_STATE__) (*((volatile unsigned int *)(0x42000000+((uint32_t)&GPIOC->ODR-0x40000000)*32+0*4))) = _CS_STATE__

#define W25Q_W_ENA				0x06		//写使能
#define W25Q_W_DIS				0x04		//写禁止

#define W25Q_R_Dat				0x03		//读数据

#define W25Q_R_STA_REG1		0x05		//读状态寄存器1，紧跟着的字节就是当前状态
#define	W25Q_R_STA_REG2		0x35		//读状态寄存器2，紧跟着的字节就是当前状态

#define W25Q_W_STA_REG_		0x01		//写状态寄存器,写入两个字节,分别到寄存器1,和寄存器2

#define W25Q_Page_Program 0x02		//页编程，先跟3个地址字节，再跟一个数据字节

#define W25Q_Block_Erase	0xD8		//块擦除64k，三个地址字节

#define W25Q_Sector_Erase	0x20		//扇区擦除，跟三个地址

#define W25Q_Full_Erase		0xC7		//全片擦除
												//0x60

#define W25Q_Susp_Erase		0x75		//暂停擦除

#define W25Q_Rest_Erase		0x7A		//恢复擦除

#define W25Q_PowDow_Mode	0xB9		//掉电模式

#define W25Q_HPer_Mode		0xA3		//高性能模式

#define W25Q_JEDEC_ID			0x9F		//读3个字节分别是生产厂家、存储器类型、容量


#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base address of Sector 0, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base address of Sector 1, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base address of Sector 2, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base address of Sector 3, 16 Kbytes   */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base address of Sector 4, 64 Kbytes   */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base address of Sector 5, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base address of Sector 6, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base address of Sector 7, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base address of Sector 8, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base address of Sector 9, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base address of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base address of Sector 11, 128 Kbytes */



/*写使能或失能*/
HAL_StatusTypeDef Write_En_De(uint8_t Type);

/*读状态寄存器*/
HAL_StatusTypeDef Read_State_Reg(uint8_t Select, uint8_t* State);

/*判忙*/
void Judge_Busy(void);

/*写状态寄存器*/
HAL_StatusTypeDef Write_State_Reg(uint8_t * State);

/*读数据*/
HAL_StatusTypeDef Read_Data(uint32_t R_Addr, uint8_t * R_Data, uint16_t R_Size);

/*页写*/
HAL_StatusTypeDef Page_Write(uint32_t WriteAddr, uint8_t * PW_Data, uint16_t PW_Size);

/*扇区擦除*/
HAL_StatusTypeDef Sector_Erase(uint32_t Sector_Addr);

/*块擦除*/
HAL_StatusTypeDef	Block_Erase(uint32_t Block_Addr);

/*全片擦除*/
HAL_StatusTypeDef Full_Erase(void);

/*读ID*/
HAL_StatusTypeDef Read_Jedec_ID(uint8_t * R_Jedec_ID);

uint32_t ReadFlash( uint32_t faddr );
void WriteFlash_byte(uint32_t faddr,uint32_t data);
void easeFlash(uint8_t sector);
uint8_t stmflash_erase_sector(uint32_t saddr);

#endif /* INC_W25Q64_H_ */
