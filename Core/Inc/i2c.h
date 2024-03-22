/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */
extern PCA9555_IO_Status_t ext_io_status;

#define PCA9555_A0	0x00
#define PCA9555_A1	0x00
#define PCA9555_A2	0x01
// PCA9555设备地址规则 0100 + A2A1A0 + R/W
#define PCA9555_DEVICE_BASE_ADDRESS 0x40
/********************* 定义PAC9555的寄存器地址 ***********************/
#define	PCA9555_REG_IN0					0x00									//输入寄存�??0地址
#define	PCA9555_REG_IN1					0x01									//输入寄存�??1地址
#define	PCA9555_REG_OUTPUT_PORT0				0x02									//输出寄存�??0地址
#define	PCA9555_REG_OUTPUT_PORT1				0x03									//输出寄存�??1地址
#define	PCA9555_REG_POL0				0x04									//极�?�反转寄存器0地址(PIN脚为输入时有�??)
#define	PCA9555_REG_POL1				0x05									//极�?�反转寄存器1地址
#define	PCA9555_REG_CONF_PORT0				0x06									//方向配置寄存�??0地址
#define	PCA9555_REG_CONF_PORT1				0x07

/**
 * 这些宏用于方便地操作�??个uint16_t类型变量
 * 将其拆分成两个uint8_t类型的变�??
 * 以及读取,设置,清除和翻转其中一个位的�??
 */
#define IO_STATUS_PORT0(io_status) ((io_status) & 0xFF)							//获取�??8位，也就�??0-7位的状�?��??
#define IO_STATUS_PORT1(io_status) ((io_status) >> 8)							//获取�??8位，也就�??8-15位的状�?��??
#define IO_STATUS_BIT(io_status, bit) (((io_status) >> (bit)) & 0x01)			//获取io_status变量中的特定�??(bit)的状态�??
#define SET_IO_STATUS_BIT(io_status, bit) ((io_status) |= (1 << (bit)))		//设置io_status变量中的特定�??(bit)�?? 1
#define CLEAR_IO_STATUS_BIT(io_status, bit) ((io_status) &= ~(1 << (bit)))	//清除io_status变量中的特定�??(bit)，将其设置为 0
#define TOGGLE_IO_STATUS_BIT(io_status, bit) ((io_status) ^= (1 << (bit)))	//将io_status变量中的特定�??(bit)的状态�?�翻转，即从1变为0，从0变为1

#define PCA_IO0_4_DCDC_Enable							(4)
#define PCA_IO0_6_FC_Fan_Enable							(6)
#define PCA_IO0_5_Heatsink_Fan_Enable					(5)
#define PCA_IO0_1_Hydrogen_Exhaust_Valve_Enable		(1)
#define PCA_IO0_0_Hydrogen_Inlet_Valve_Enable			(0)
#define PCA_IO0_3_Contactor_Fc_Enable					(3)
#define PCA_IO0_2_Contactor_Load_Enable				(2)

//无需翻转的io控制
#define Heatsink_Fan_Open SET_IO_STATUS_BIT(ext_io_status.pca9555_expected_state, PCA_IO0_5_Heatsink_Fan_Enable);
#define Heatsink_Fan_Close CLEAR_IO_STATUS_BIT(ext_io_status.pca9555_expected_state, PCA_IO0_5_Heatsink_Fan_Enable);
//�??要翻转的io控制
#define FC_Fan_Open CLEAR_IO_STATUS_BIT(ext_io_status.pca9555_expected_state, PCA_IO0_6_FC_Fan_Enable);
#define FC_Fan_Close SET_IO_STATUS_BIT(ext_io_status.pca9555_expected_state, PCA_IO0_6_FC_Fan_Enable);
#define DCDC_Open CLEAR_IO_STATUS_BIT(ext_io_status.pca9555_expected_state, PCA_IO0_4_DCDC_Enable);
#define DCDC_Close SET_IO_STATUS_BIT(ext_io_status.pca9555_expected_state, PCA_IO0_4_DCDC_Enable);
#define Hydrogen_Exhaust_Valve_Open CLEAR_IO_STATUS_BIT(ext_io_status.pca9555_expected_state, PCA_IO0_1_Hydrogen_Exhaust_Valve_Enable);
#define Hydrogen_Exhaust_Valve_Close SET_IO_STATUS_BIT(ext_io_status.pca9555_expected_state, PCA_IO0_1_Hydrogen_Exhaust_Valve_Enable);
#define Hydrogen_Inlet_Valve_Open CLEAR_IO_STATUS_BIT(ext_io_status.pca9555_expected_state, PCA_IO0_0_Hydrogen_Inlet_Valve_Enable);
#define Hydrogen_Inlet_Valve_Close SET_IO_STATUS_BIT(ext_io_status.pca9555_expected_state, PCA_IO0_0_Hydrogen_Inlet_Valve_Enable);
#define Contactor_Fc_Open CLEAR_IO_STATUS_BIT(ext_io_status.pca9555_expected_state, PCA_IO0_3_Contactor_Fc_Enable);
#define Contactor_Fc_Close SET_IO_STATUS_BIT(ext_io_status.pca9555_expected_state, PCA_IO0_3_Contactor_Fc_Enable);
#define Contactor_Load_Open CLEAR_IO_STATUS_BIT(ext_io_status.pca9555_expected_state, PCA_IO0_2_Contactor_Load_Enable);
#define Contactor_Load_Close SET_IO_STATUS_BIT(ext_io_status.pca9555_expected_state, PCA_IO0_2_Contactor_Load_Enable);
/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */
uint8_t PCA9555_Get_Device_Address(uint8_t A2, uint8_t A1, uint8_t A0, uint8_t RW);
void PCA9555_Init(I2C_HandleTypeDef *i2cHandle, uint8_t device_address);
void PCA9555_Set_IO_Mode(I2C_HandleTypeDef *i2cHandle, uint8_t device_address, uint16_t io_mode);
uint16_t PCA9555_Read_IO_State(I2C_HandleTypeDef *i2cHandle, uint8_t device_address);
void PCA9555_Write_IO_State(I2C_HandleTypeDef *i2cHandle, uint8_t device_address, uint16_t io_state);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

