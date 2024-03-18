/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */
uint16_t CAN_RxID;
uint8_t CAN_RxDat[8];

FDCAN_FilterTypeDef fdcan1_RxFilter;
FDCAN_RxHeaderTypeDef fdcan1_RxHeader;
FDCAN_TxHeaderTypeDef fdcan1_TxHeader;

extern AnalogInputs_TypeDef my_analog_inputs;
extern DeviceStatus_t Sys_status;

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 20;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 12;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  //配置RX滤波�??
  fdcan1_RxFilter.IdType=FDCAN_STANDARD_ID;                       //标准ID
  fdcan1_RxFilter.FilterIndex=0;                                  //滤波器索�??
  fdcan1_RxFilter.FilterType=FDCAN_FILTER_MASK;                   //滤波器类�??
  fdcan1_RxFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //过滤�??0关联到FIFO0
  fdcan1_RxFilter.FilterID1=0x456 << 18;                               //设置过滤�?? ID1 �?? 0x456
  fdcan1_RxFilter.FilterID2=0x7FF << 18;                               //设置过滤�?? ID2 �?? 0x7FF（掩码）
	if(HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan1_RxFilter)!=HAL_OK) //滤波器初始化
	{
		Error_Handler();
	}


	/* Configure global filter:下面这一句是配置全局滤波器配置寄存器的，�??定要写，否则配置了也没用
	   Filter all remote frames with STD and EXT ID
	   Reject non matching frames with STD ID and EXT ID */

//	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
//	{
//	  Error_Handler();
//	}


	HAL_FDCAN_Start(&hfdcan1);                               //�??启FDCAN
	HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0); //�??启中断接�??

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    HAL_NVIC_DisableIRQ(FDCAN1_IT1_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

//提取位操�??
bool getBit(uint8_t value, uint8_t bitPosition) {
    uint8_t mask = 1 << bitPosition;
    return (value & mask) != 0;
}
//转CAN数据
void convertFloatsToCANData(float value1, float value2, float value3, float value4, uint8_t* CAN_RxDat_Buffer)
{
    uint16_t scaledValue1 = (uint16_t)(value1 * 100);
    uint16_t scaledValue2 = (uint16_t)(value2 * 100);
    uint16_t scaledValue3 = (uint16_t)(value3 * 100);
    uint16_t scaledValue4 = (uint16_t)(value4 * 100);

    CAN_RxDat_Buffer[0] = (uint8_t)(scaledValue1 >> 8); // Value 1, high byte
    CAN_RxDat_Buffer[1] = (uint8_t)(scaledValue1);      // Value 1, low byte
    CAN_RxDat_Buffer[2] = (uint8_t)(scaledValue2 >> 8); // Value 2, high byte
    CAN_RxDat_Buffer[3] = (uint8_t)(scaledValue2);      // Value 2, low byte
    CAN_RxDat_Buffer[4] = (uint8_t)(scaledValue3 >> 8); // Value 3, high byte
    CAN_RxDat_Buffer[5] = (uint8_t)(scaledValue3);      // Value 3, low byte
    CAN_RxDat_Buffer[6] = (uint8_t)(scaledValue4 >> 8); // Value 4, high byte
    CAN_RxDat_Buffer[7] = (uint8_t)(scaledValue4);      // Value 4, low byte
}

//CAN发�??
uint8_t FDCAN1_Send_Msg(uint8_t* msg,uint32_t len, uint32_t canId)
{
    fdcan1_TxHeader.Identifier=canId;                           //32位ID
    fdcan1_TxHeader.IdType=FDCAN_STANDARD_ID;                  //标准ID
    fdcan1_TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //数据�??
    fdcan1_TxHeader.DataLength=len;                             //数据长度
    fdcan1_TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    fdcan1_TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    fdcan1_TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    fdcan1_TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事�??
    fdcan1_TxHeader.MessageMarker=0;

    if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&fdcan1_TxHeader,msg)!=HAL_OK) return 1;//发�??
    return 0;
}
//CAN接收中断回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	uint16_t CAN_RxID_Buffer;
	uint8_t CAN_RxDat_Buffer[8];
	uint8_t CAN_TxDat[8];
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
	{
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan1_RxHeader, CAN_RxDat_Buffer) == HAL_OK)
		{
			CAN_RxID_Buffer = fdcan1_RxHeader.Identifier;
			// 在这里处理接收到的CAN消息
		}
		//如果CAN帧ID匹配,就吧数据转存�?? CAN_RxDat
		if(CAN_RxID_Buffer == CAN_ID){
			CAN_RxID = CAN_RxID_Buffer;
			memcpy(CAN_RxDat,CAN_RxDat_Buffer, 8);
#ifdef USE_DEBUG_CAN
			RS232_1_printf("RxID:0x%04X = ", CAN_RxID);
			for (uint8_t i = 0; i < 8; i++) {
				RS232_1_printf("0x%02X ", CAN_RxDat[i]);
			}RS232_1_printf("\r\n");
#endif
			//0�??:控制电堆启停
			if(getBit(CAN_RxDat[0], 0)){
				Sys_status = DEVICE_RUNNING;
			}else Sys_status = DEVICE_STOPPED;
			//7�??:问询�??有参�??
			if(getBit(CAN_RxDat[0], 7)){
				convertFloatsToCANData(
						my_analog_inputs.Power_Voltage.Current_Val,
						my_analog_inputs.Hydrogen_Cylinder_Pressure.Current_Val,
						my_analog_inputs.FC_Internal_Temperature.Current_Val,
						my_analog_inputs.FC_External_Temperature.Current_Val,
						CAN_TxDat
				);
				FDCAN1_Send_Msg(CAN_TxDat,FDCAN_DLC_BYTES_8,0x123);
				memset(CAN_RxDat, 0, sizeof(CAN_RxDat));
				convertFloatsToCANData(
						my_analog_inputs.Shunt_A_Current.Current_Val,
						my_analog_inputs.Shunt_A_Voltage.Current_Val,
						my_analog_inputs.Shunt_B_Current.Current_Val,
						my_analog_inputs.Shunt_B_Voltage.Current_Val,
						CAN_TxDat
				);
				FDCAN1_Send_Msg(CAN_TxDat,FDCAN_DLC_BYTES_8,0x124);
				memset(CAN_RxDat, 0, sizeof(CAN_RxDat));
				convertFloatsToCANData(
										my_analog_inputs.Shunt_A_Power.Current_Val,
										my_analog_inputs.Shunt_A_Total_Energy.Current_Val,
										my_analog_inputs.Shunt_B_Power.Current_Val,
										my_analog_inputs.Shunt_B_Total_Energy.Current_Val,
										CAN_TxDat
								);
				FDCAN1_Send_Msg(CAN_TxDat,FDCAN_DLC_BYTES_8,0x125);
				memset(CAN_RxDat, 0, sizeof(CAN_RxDat));
			}
			memset(CAN_RxDat, 0, sizeof(CAN_RxDat));  // 将数组元素全部置�?
		}
	}
}

/* USER CODE END 1 */
