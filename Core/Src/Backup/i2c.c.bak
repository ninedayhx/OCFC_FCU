/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB7     ------> I2C1_SDA
    PB8-BOOT0     ------> I2C1_SCL
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB7     ------> I2C1_SDA
    PB8-BOOT0     ------> I2C1_SCL
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

uint8_t PCA9555_Get_Device_Address(uint8_t A2, uint8_t A1, uint8_t A0, uint8_t RW) {
    uint8_t address = PCA9555_DEVICE_BASE_ADDRESS;
    address |= (A2 << 3) | (A1 << 2) | (A0 << 1) | RW;
	#ifdef  USE_DEBUG_PCA9555
		RS232_1_printf("PCA9555_Get_Device_Address:0x%02X \n ", address);
    #endif
    return address;
}
//初始化PCA9555设备
void PCA9555_Init(I2C_HandleTypeDef *i2cHandle, uint8_t device_address) {
    uint8_t data = 0x00;
    HAL_I2C_Master_Transmit(i2cHandle, device_address, &data, 1, 10);
	#ifdef  USE_DEBUG_PCA9555
		RS232_1_printf("PCA9555_Init:0x%02X \n ", data);
    #endif
}
//配置PCA9555的IO口模�??
void PCA9555_Set_IO_Mode(I2C_HandleTypeDef *i2cHandle, uint8_t device_address, uint16_t io_mode) {
    uint8_t data[3];
    data[0] = PCA9555_REG_CONF_PORT0;
    data[1] = io_mode & 0xFF;
    data[2] = io_mode >> 8;
    HAL_I2C_Master_Transmit(i2cHandle, device_address, data, 3, 10);
	#ifdef  USE_DEBUG_PCA9555
    	RS232_1_printf("PCA9555_Set_IO_Mode:0x%02X 0x%02X 0x%02X\n", data[0], data[1], data[2]);
    #endif
}
//读取PCA9555的IO口状�??
uint16_t PCA9555_Read_IO_State(I2C_HandleTypeDef *i2cHandle, uint8_t device_address) {
    uint8_t data[2];
    HAL_I2C_Master_Receive(i2cHandle, device_address, data, 2, 10);
	#ifdef  USE_DEBUG_PCA9555
		RS232_1_printf("PCA9555_Read_IO_State:0x%02X 0x%02X \n", data[0], data[1]);
	#endif
    return (uint16_t)data[1] << 8 | data[0];
}
//写入PCA9555的IO口状�??
void PCA9555_Write_IO_State(I2C_HandleTypeDef *i2cHandle, uint8_t device_address, uint16_t io_state) {
    uint8_t data[3];
    data[0] = PCA9555_REG_OUTPUT_PORT0;
    data[1] = io_state & 0xFF;
    data[2] = io_state >> 8;
    HAL_I2C_Master_Transmit(i2cHandle, device_address , data, 3, 10);
	#ifdef  USE_DEBUG_PCA9555
    	RS232_1_printf("PCA9555_Write_IO_State:0x%02X 0x%02X 0x%02X\n", data[0], data[1], data[2]);
    #endif
}

/* USER CODE END 1 */
