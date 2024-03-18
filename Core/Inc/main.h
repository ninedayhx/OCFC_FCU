/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
    STATE_LoLo,
    STATE_Lo,
	STATE_Normal,
	STATE_High,
	STATE_HiHi
} ValState_t;
typedef struct {
    float LoLo_Val;		//极低
    float Lo_Val;		//过低
    float High_Val;		//过高
    float HiHi_Val;		//极高
    float Current_Val;	//当前�???
    ValState_t Status;	//状前状�?�枚举类型变�???
} Parameter_t;
typedef struct {
	/**
	 * DMA获取数组值与通道对应关系
	 * adc1_0->AD1_IN3	->PA2	->Power_Voltage; 						//FCU输入电压(单位V)
	 * adc1_1->AD1_IN4	->PA3	->External_Hydrogen_Concentration;	//堆外氢浓
	 * adc1_2->AD1_IN5	->PC4	->Hydrogen_Cylinder_Pressure;			//氢气瓶压
	 * adc1_3->AD1_IN11	->PB12	->FC_Internal_Temperature;				//堆内温度	(单位摄氏�???)
	 * adc1_4->AD1_IN12	->PB1	->FC_External_Temperature;				//堆外温度	(单位摄氏�???)
	 * adc2_0->AD2_IN1	->PA0	->Shunt_A_Current;						//分流器A电流(单位A)
	 * adc2_1->AD2_IN2	->PA1	->Shunt_A_Voltage;						//分流器A电压(单位V)
	 * adc2_2->AD2_IN5	->PC4	->Shunt_B_Current;						//分流器B电流(单位A)
	 * adc2_3->AD2_IN12	->PB2	->Shunt_B_Voltage;						//分流器B电压(单位V)
	 */
	uint16_t adc1_values[5];							//adc1原始 �?
	uint16_t adc2_values[4];							//adc2原始 �?
	Parameter_t Power_Voltage; 						//FCU输入电压(单位V)
	Parameter_t Shunt_A_Voltage;						//分流器A电压(单位V)
	Parameter_t Shunt_A_Current;						//分流器A电流(单位A)
	Parameter_t Shunt_A_Power;						//*分流器A瞬时功率(单位kW)
	Parameter_t Shunt_A_Total_Energy;				//*分流器A累计电能(单位kWh)
	Parameter_t Shunt_B_Voltage;						//分流器B电压(单位V)
	Parameter_t Shunt_B_Current;						//分流器B电流(单位A)
	Parameter_t Shunt_B_Power;						//*分流器B瞬时功率(单位kW)
	Parameter_t Shunt_B_Total_Energy;				//*分流器B累计电能(单位kWh)
	Parameter_t FC_Internal_Temperature;			//堆内温度	(单位摄氏�???)
	Parameter_t FC_External_Temperature;			//堆外温度	(单位摄氏�???)
	Parameter_t Hydrogen_Cylinder_Pressure;		//氢气瓶压
	Parameter_t External_Hydrogen_Concentration;	//堆外氢浓
	float Shooting_Flow;		//瞬时流量
	float Integrated_Flow;	//累计流量

} AnalogInputs_TypeDef;
typedef struct {
	uint8_t Expected_FC_Fan_Speed;					//目标�??? FC主风扇转�??? PWM 占空�??? 0-100%
	bool Expected_FC_Fan_Enable;						//目标�??? FC主风扇使�???
	bool Expected_DCDC_Enable;							//目标�??? DCDC使能
	bool Expected_Heatsink_Fan_Enable;				//目标�??? FCU散热风扇使能
	bool Expected_Hydrogen_Inlet_Valve_Enable;		//目标�??? 氢气进气�???使能
	bool Expected_Hydrogen_Exhaust_Valve_Enable;	//目标�??? 氢气排气�???使能
	bool Expected_Contactor_Fc_Enable;				//目标�??? 燃料电池接触器A使能
  	bool Expected_Contactor_Load_Enable;				//目标�??? 终端负载接触器B使能
} SysControl_TypeDef;

typedef enum {
	DEVICE_STOPPED, 	//停机
    DEVICE_RUNNING, 	//正常运行
    DEVICE_STANDBY, 	//节能待机
    DEVICE_ERROR,		//异常
    DEVICE_FAULT		//故障
} DeviceStatus_t;

typedef struct {
    bool device_paused;     //设备是否已暂�?
    bool device_started;    //设备是否已启�?
    bool device_stopped;    //设备是否已停�?
    bool device_error;      //设备是否出现错误
    bool device_fault;      //设备是否故障
} DeviceFlags_t;

typedef struct {
	//用于储存PCA9555当前 IO 状�?�和目标 IO 状�??
    uint16_t pca9555_current_state;	//当前
    uint16_t pca9555_expected_state;	//期望
} PCA9555_IO_Status_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define W25Q64_CS_Pin GPIO_PIN_4
#define W25Q64_CS_GPIO_Port GPIOA
#define WS2812_800KHz_Pin GPIO_PIN_6
#define WS2812_800KHz_GPIO_Port GPIOC
#define BUZZER_2700Hz_Pin GPIO_PIN_15
#define BUZZER_2700Hz_GPIO_Port GPIOA
#define PCA9555_INT_Pin GPIO_PIN_9
#define PCA9555_INT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define CAN_ID 0x456			// 定义CAN帧的标识�?
#define SAMPLING_INTERVAL (300) //ADC采样间隔
#define DELAY_TIME (1000)

/*----------------------模拟值采样参数设�?----------------------*/
#define NTC_B           (3950.0)     //温度系数
#define NTC_TN          (298.15)     //额定温度(绝对温度加常�???:273.15+25)
#define NTC_RN          (10000.0)   // 热敏电阻阻�?? 25 摄氏度时的阻�???(单位欧姆)
#define NTC_R1          (10000.0)   // 分压电阻阻�??
#define NTC_BaseVol     (3.29)        //AD基准电压
/*-------------------------系统参数设置------------------------*/
//#define DEFAULT_SYS_TWOFC 			//双堆定义
#define DEFAULT_SYS_STATUS DEVICE_STOPPED			//系统默认状�??
#define SYS_MAX_POWER		(0.7)					//整个系统�?大输出KW
#define FC_LOLO_V			(55*0.55) 			//电堆级限�?低电�? 单片截止电压0.55V（低于不可�?�损坏，）最大电�? 1V
#define FC_HIHI_TEMP		(55)					//电堆级限温度
#define FC_NORMAL_TEMP		(46)					//电堆理想温度
#define FC_EXHAUST_PERIED   (7500)					//排气周期 ms
#define FC_EXHAUST_TIME     (200)					//排气时长 ms

#ifdef DEFAULT_SYS_TWOFC   //双堆使能
#define A_FC_LOLO_V	FC_LOLO_V
#define B_FC_LOLO_V	(45.1-2)
#endif

#define RS485_FLOWMETER								//如果没有流量计注释这�?�?

#define DISPLAY_ICON_R (0)
#define DISPLAY_ICON_Y (1)
#define DISPLAY_ICON_G (2)

//#define USE_DEBUG_PCA9555
//#define USE_DEBUG_ANALOG

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
