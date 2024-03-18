/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define NUM_ADC_CHANNELS 9  // 定义ADC通道数量
/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */
void read_all_adc1_values(uint16_t* adc_values, uint8_t num_channels);
void read_all_adc2_values(uint16_t* adc_values, uint8_t num_channels);
float Get_Tempture(uint16_t adc,uint32_t ntc_rn ,uint32_t ntc_r1);
float convert_adc_to_voltage(uint16_t adc_value, float voltage_ref, float voltage_divider_ratio);
float convert_adc_to_current(uint16_t adc_value, float voltage_ref, float shunt_max_current);
float power_calculation(float voltage, float current);
float energy_calculation(float power, uint16_t time_interval_ms);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

