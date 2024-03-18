/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "rtc.h"
#include "adc.h"
#include "i2c.h"
#include "adaptive_pid.h"

#include <string.h>

#include "rs485_display.h"
#include "rs485.h"
#include "rs232.h"
#include "fdcan.h"

#include "w25q64.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern uint16_t CAN_RxID;
extern uint8_t CAN_RxDat[8];
extern uint8_t CAN_TtDat[8];

extern UART_ReceiveTypeDef uart1_receive;
extern UART_ReceiveTypeDef uart2_receive;
extern UART_ReceiveTypeDef uart3_receive;
//extern FDCAN_HandleTypeDef hfdcan1;
extern DateTime rtc_time;

AnalogInputs_TypeDef my_analog_inputs;
PCA9555_IO_Status_t ext_io_status;
SysControl_TypeDef sysControl;
DeviceStatus_t Sys_status = DEFAULT_SYS_STATUS;//默认上电状态
DeviceFlags_t Sys_flags = {false, false, false, false, false};

extern I2C_HandleTypeDef hi2c1;
extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim4;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId Display_Task04Handle;
/* USER CODE END Variables */
osThreadId Top_TaskHandle;
osThreadId ActiveExtIO_TasHandle;
osThreadId Analog_Task03Handle;
osTimerId RtosTimer01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void SysDataInit(void);
/* USER CODE END FunctionPrototypes */

void Start_Top_Task(void const * argument);
void Start_ActiveExtIO_Task02(void const * argument);
void Start_Analog_Task03(void const * argument);
void RtosCallback01(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of RtosTimer01 */
  osTimerDef(RtosTimer01, RtosCallback01);
  RtosTimer01Handle = osTimerCreate(osTimer(RtosTimer01), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Top_Task */
  osThreadDef(Top_Task, Start_Top_Task, osPriorityNormal, 0, 256);
  Top_TaskHandle = osThreadCreate(osThread(Top_Task), NULL);

  /* definition and creation of ActiveExtIO_Tas */
  osThreadDef(ActiveExtIO_Tas, Start_ActiveExtIO_Task02, osPriorityIdle, 0, 128);
  ActiveExtIO_TasHandle = osThreadCreate(osThread(ActiveExtIO_Tas), NULL);

  /* definition and creation of Analog_Task03 */
  osThreadDef(Analog_Task03, Start_Analog_Task03, osPriorityIdle, 0, 180);
  Analog_Task03Handle = osThreadCreate(osThread(Analog_Task03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */


  //屏幕控制子程�?
//  osThreadDef(Display_Task04, Rs485_Display_Task04, osPriorityIdle, 0, 256);
//  Display_Task04Handle = osThreadCreate(osThread(Display_Task04), NULL);

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Start_Top_Task */
/**
  * @brief  Function implementing the Top_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Top_Task */
__weak void Start_Top_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Top_Task */
  /* Infinite loop */
	FanControl_TypeDef fan_control;
	PID_Init(&(fan_control.pid), K_P, K_I, K_D, -1.0f, 1.0f, FAN_PWM_MAX_VALUE, -FAN_PWM_MAX_VALUE, 0.1f, 50.0f);
	/**
	* 设置温度和负载的阈值
	* 如果温度或负载超过了预设的阈值，
	* 就会触发PID控制算法，自动增加风扇的转速
	* 以防止温度继续上升或负载过大
	*/
	fan_control.temp_threshold = 30.0f;
	fan_control.load_threshold = 0.0f;
	// 设置初始风扇转速
	fan_control.fan_speed = 0;

	//排气阀开阀计时器
	uint32_t Exhaust_O_start_time = 0;
	uint32_t Exhaust_O_current_time = 0;
	//排气阀关阀计时器
	uint32_t Exhaust_C_start_time = 0;
	uint32_t Exhaust_C_current_time = 0;
	bool ExhaustFlag = false;

	osDelay(200);
	sysControl.Expected_FC_Fan_Enable = 0;

	//获取系统时间
	uint32_t count = HAL_GetTick();

	uint8_t analogInputs_count = 0;

	while(true)
	{
		// 循环接收RS232的信息，并执行对应动作
		process_RS232_uart1_command();
		osDelay(20);
		// 更新风扇控制律
		//FanControl_Update(&fan_control, my_analog_inputs.FC_Internal_Temperature.Current_Val, 0);

		// 设置散热风扇使能标志位，注意，在此处仅仅只设置了标志位，真正的使能在另外一个线程中
		// 由于在另外一个线程中，所以可以认为，此处设置了标志位，马上在另外一个线程中执行动作
		sysControl.Expected_Heatsink_Fan_Enable = true;

		//FCU状态机
		switch(Sys_status) {
			case DEVICE_STOPPED://系统停机
				/*----------------单堆停机流程-------------------*/
				//如果系统正在运行,进入以下停机流程
				if(Sys_flags.device_started){

					getRtcDateTime(&rtc_time.year, &rtc_time.month, &rtc_time.day, &rtc_time.hour, &rtc_time.min, &rtc_time.sec, &rtc_time.ms);
					RS232_1_printf("<$-%04d/%02d/%02d-%02d:%02d:%02d.%03d-$>", rtc_time.year, rtc_time.month, rtc_time.day, rtc_time.hour, rtc_time.min, rtc_time.sec, rtc_time.ms);
					RS232_1_printf("System is shutting down! Please wait.....\n");

					// 1.设置总输出接触器使能标志为关闭，即关闭系统对外输出，但此处只更新标志，真正的使能在另外一个线程中
					// 由于是在另外一个线程中，所以可以认为进入osdelay函数以后，将进入另外一个线程，马上执行动作
					sysControl.Expected_Contactor_Load_Enable = false;
					osDelay(3000);
					//2.设置输出DCDC标志位为关闭
					sysControl.Expected_DCDC_Enable = false;
					osDelay(1000);
					//2.设置氢气进气阀为关闭状态
					sysControl.Expected_Hydrogen_Inlet_Valve_Enable = false;
					osDelay(2000);
					//3.风扇转速缓减到0
					while(sysControl.Expected_FC_Fan_Speed > 0){
						sysControl.Expected_FC_Fan_Speed --;
						osDelay(100);
					}
					//3.设置主风扇电源标志位为关闭
					sysControl.Expected_FC_Fan_Enable = false;
					//4.关闭电堆输出接触器
					sysControl.Expected_Contactor_Fc_Enable = false;
					//5.设置系统运行状态为关闭
					Sys_flags.device_started = false;

					getRtcDateTime(&rtc_time.year, &rtc_time.month, &rtc_time.day, &rtc_time.hour, &rtc_time.min, &rtc_time.sec, &rtc_time.ms);
					RS232_1_printf("<$-%04d/%02d/%02d-%02d:%02d:%02d.%03d-$>", rtc_time.year, rtc_time.month, rtc_time.day, rtc_time.hour, rtc_time.min, rtc_time.sec, rtc_time.ms);
					RS232_1_printf("System has shut down successfully.\n");
				}
			break;

			case DEVICE_RUNNING://系统启动
				/*-----------------单堆启动流程-----------------*/
				//如果系统未启动,进入以下单堆启动流程
				if(!Sys_flags.device_started){

					getRtcDateTime(&rtc_time.year, &rtc_time.month, &rtc_time.day, &rtc_time.hour, &rtc_time.min, &rtc_time.sec, &rtc_time.ms);
					RS232_1_printf("<$-%04d/%02d/%02d-%02d:%02d:%02d.%03d-$>", rtc_time.year, rtc_time.month, rtc_time.day, rtc_time.hour, rtc_time.min, rtc_time.sec, rtc_time.ms);

					RS232_1_printf("System is starting up! Please wait.....\n");
					//1.散热风扇电源使能
					sysControl.Expected_FC_Fan_Enable = true;
					osDelay(1000);
					//  散热风扇转速设置为99
					sysControl.Expected_FC_Fan_Speed = 99;
					osDelay(500);
					//2.氢气进气阀使能
					sysControl.Expected_Hydrogen_Inlet_Valve_Enable = true;
					osDelay(1000);
					//3.电堆输出接触器使能
					sysControl.Expected_Contactor_Fc_Enable = true;
					osDelay(1000);
					//4.输出DCDC使能
					sysControl.Expected_DCDC_Enable = true;
					osDelay(1000);
					//5.总输出接触器使能，开始对外输出
					sysControl.Expected_Contactor_Load_Enable = true;
					//6.设置系统运行状态为启动
					Sys_flags.device_started = true;

					getRtcDateTime(&rtc_time.year, &rtc_time.month, &rtc_time.day, &rtc_time.hour, &rtc_time.min, &rtc_time.sec, &rtc_time.ms);
					RS232_1_printf("<$-%04d/%02d/%02d-%02d:%02d:%02d.%03d-$>", rtc_time.year, rtc_time.month, rtc_time.day, rtc_time.hour, rtc_time.min, rtc_time.sec, rtc_time.ms);
					RS232_1_printf("System has started successfully.\n");
				}

				// 非阻塞控制排气阀延时通断，即通过计时来开关
				// 如果排气阀是关闭状态
				if(!ExhaustFlag){
					Exhaust_O_start_time = HAL_GetTick();// 获取当前系统时间，统计的是已经关阀的时间
					// 如果排气阀关闭时间超过了排气周期
					if ((Exhaust_O_start_time - Exhaust_O_current_time) >= FC_EXHAUST_PERIED) {
						// 排气阀打开
						sysControl.Expected_Hydrogen_Exhaust_Valve_Enable = true;
						// 开始对开阀时间进行计时
						Exhaust_O_current_time = Exhaust_O_start_time;
						Exhaust_C_current_time = Exhaust_O_current_time;
						ExhaustFlag = true;
						// 显示在图形界面 useless
						//AnimationPlayFrame(1,7,DISPLAY_ICON_G);
					}
				// 如果排气阀是开启状态
				}else{
					Exhaust_C_start_time =  HAL_GetTick();// 获取当前系统时间，统计的是已经开阀的时间
					// 如果开阀时间超过了开阀时长
					if ((Exhaust_C_start_time - Exhaust_C_current_time) >= FC_EXHAUST_TIME){
						// 排气阀关闭
						sysControl.Expected_Hydrogen_Exhaust_Valve_Enable = false;
						// 开始对关阀时间进行计时
						Exhaust_C_current_time = Exhaust_C_start_time;
						ExhaustFlag = false;
						//AnimationPlayFrame(1,7,DISPLAY_ICON_Y);
					}
				}

				//--------------------------单堆保护及风扇转速控制策略-------------------------------//
				// 循环检测电堆运行状态，不对则进行停机
				// 检测时间为 200ms 检测一次
				if(count + 200 <= HAL_GetTick())
				{
					//燃料电池电压低于FC_LOLO_V，或者电堆温度超过FC_HIHI_TEMP
					//关闭总输出接触器
					if(my_analog_inputs.FC_Internal_Temperature.Current_Val >= FC_HIHI_TEMP)
					{
						sysControl.Expected_Contactor_Fc_Enable = false;
					}
					else if(my_analog_inputs.Shunt_A_Voltage.Current_Val <= FC_LOLO_V)
					{
						// 如果采集次数大于20次就认为电压低于保护值，则触发保护
						analogInputs_count += 1;
						if(analogInputs_count >= 20)
						{
							sysControl.Expected_Contactor_Fc_Enable = false;
							analogInputs_count = 0;
						}
					}
					else
					{
						sysControl.Expected_Contactor_Fc_Enable =true;
						sysControl.Expected_Contactor_Load_Enable = true;
					}

					// 每隔0.2s对散热风扇进行转速控制，默认策略为查表法
					sysControl.Expected_FC_Fan_Speed = calculateFanSpeed(
						my_analog_inputs.FC_Internal_Temperature.Current_Val,
						my_analog_inputs.Shunt_B_Power.Current_Val);

					count = HAL_GetTick();
				}
				break;

			case DEVICE_STANDBY://系统待机
				RS232_1_printf("Device is in standby mode\n");
				break;

			case DEVICE_ERROR://系统错误
				RS232_1_printf("Device is in error state\n");
				break;

			case DEVICE_FAULT://系统故障
				RS232_1_printf("Device is in fault state\n");
				break;

			default://未知设备状状态
				RS232_1_printf("Unknown device status\n");
				break;
		}
	}
  /* USER CODE END Start_Top_Task */
}

/* USER CODE BEGIN Header_Start_ActiveExtIO_Task02 */
/**
* @brief 外部iO控制线程，在此线程中，实时处理在顶层任务中，设置的标志位，执行标志位对应动作
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_ActiveExtIO_Task02 */
__weak void Start_ActiveExtIO_Task02(void const * argument)
{
	/* USER CODE BEGIN Start_ActiveExtIO_Task02 */

	// 获取PCA9555的设备地址
	uint8_t pca9555_device_address = PCA9555_Get_Device_Address(PCA9555_A2, PCA9555_A1, PCA9555_A0, 0);

	for(uint8_t i = 0 ; i < 3 ; i++ ){
		// 初始化PCA9555
		PCA9555_Init(&hi2c1, pca9555_device_address);
		// 设置IO口模式 ,Port0是输出模式，Port1是输入模式
		PCA9555_Set_IO_Mode(&hi2c1, pca9555_device_address, 0x0080);
		osDelay(50);
		PCA9555_Write_IO_State(&hi2c1, pca9555_device_address, 0x0F);
	}
	ext_io_status.pca9555_current_state = 0x0F;
	ext_io_status.pca9555_expected_state = 0x0F;

	// 将系统控制状态同步到设置变量中

	// 散热风扇PWM占空比 0-100%
	uint8_t Current_FC_Fan_Speed = sysControl.Expected_FC_Fan_Speed;
	// 散热风扇使能状态
	bool Current_FC_Fan_Enable = sysControl.Expected_FC_Fan_Enable;
	// 当前总输出DCDC使能状态
	bool Current_DCDC_Enable = sysControl.Expected_DCDC_Enable;
	// DCDC散热风扇使能状态
	bool Current_Heatsink_Fan_Enable = sysControl.Expected_Heatsink_Fan_Enable;
	// 氢气进气阀工作状态
	bool Current_Hydrogen_Inlet_Valve_Enable = sysControl.Expected_Hydrogen_Inlet_Valve_Enable;
	// 排气阀工作状态
	bool Current_Hydrogen_Exhaust_Valve_Enable = sysControl.Expected_Hydrogen_Exhaust_Valve_Enable;
	// 电堆输出接触器使能状态
	bool Current_Contactor_Fc_Enable = sysControl.Expected_Contactor_Fc_Enable;
	// 总输出接触器使能状态
	bool Current_Contactor_Load_Enable = sysControl.Expected_Contactor_Load_Enable;

	FC_Fan_Close;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, htim4.Init.Period - 10);

	/* Infinite loop */
	while(true){
		// 散热风扇转速状态更新
		if(sysControl.Expected_FC_Fan_Speed != Current_FC_Fan_Speed){
			if(sysControl.Expected_FC_Fan_Speed > 99) sysControl.Expected_FC_Fan_Speed = 99;
			// 将占空比取反,经过光耦再翻转为正占空比(定时器周期减去正占空比的值，即为负占空比值)
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, htim4.Init.Period - map_0_to_100_to_12_to_100(sysControl.Expected_FC_Fan_Speed));
			Current_FC_Fan_Speed = sysControl.Expected_FC_Fan_Speed;
		}
		// 散热风扇使能状态更新
		// 如果上次风扇运行状态与当前设置状态不一致，即风扇运行状态发生了改变，则对风扇状态进行更新
		if(sysControl.Expected_FC_Fan_Enable != Current_FC_Fan_Enable){
			if(sysControl.Expected_FC_Fan_Enable){
				FC_Fan_Open;
				Current_FC_Fan_Enable = sysControl.Expected_FC_Fan_Enable;
			}else{
				FC_Fan_Close;
				Current_FC_Fan_Enable = sysControl.Expected_FC_Fan_Enable;
			}
		}
		// 输出DCDC使能状态更新
		if(sysControl.Expected_DCDC_Enable != Current_DCDC_Enable){
			if(sysControl.Expected_DCDC_Enable){
				DCDC_Open;
				Current_DCDC_Enable = sysControl.Expected_DCDC_Enable;
			}else{
				DCDC_Close;
				Current_DCDC_Enable = sysControl.Expected_DCDC_Enable;
			}
		}
		// DCDC散热风扇使能状态更新
		if(sysControl.Expected_Heatsink_Fan_Enable != Current_Heatsink_Fan_Enable){
			if(sysControl.Expected_Heatsink_Fan_Enable){
				Heatsink_Fan_Open;
				Current_Heatsink_Fan_Enable = sysControl.Expected_Heatsink_Fan_Enable;
			}else{
				Heatsink_Fan_Close;
				Current_Heatsink_Fan_Enable = sysControl.Expected_Heatsink_Fan_Enable;
			}
		}
		// 氢气进气阀使能状态更新
		if(sysControl.Expected_Hydrogen_Inlet_Valve_Enable != Current_Hydrogen_Inlet_Valve_Enable){
			if(sysControl.Expected_Hydrogen_Inlet_Valve_Enable){
				Hydrogen_Inlet_Valve_Open;
				Current_Hydrogen_Inlet_Valve_Enable = sysControl.Expected_Hydrogen_Inlet_Valve_Enable;
			}else{
				Hydrogen_Inlet_Valve_Close;
				Current_Hydrogen_Inlet_Valve_Enable = sysControl.Expected_Hydrogen_Inlet_Valve_Enable;
			}
		}
		// 氢气排气阀使能状态更新
		if(sysControl.Expected_Hydrogen_Exhaust_Valve_Enable != Current_Hydrogen_Exhaust_Valve_Enable){
			if(sysControl.Expected_Hydrogen_Exhaust_Valve_Enable){
				Hydrogen_Exhaust_Valve_Open;
				Current_Hydrogen_Exhaust_Valve_Enable = sysControl.Expected_Hydrogen_Exhaust_Valve_Enable;
			}else{
				Hydrogen_Exhaust_Valve_Close;
				Current_Hydrogen_Exhaust_Valve_Enable = sysControl.Expected_Hydrogen_Exhaust_Valve_Enable;
			}
		}
		// 电堆输出端接触器使能状态更新
		if(sysControl.Expected_Contactor_Fc_Enable != Current_Contactor_Fc_Enable){
			if(sysControl.Expected_Contactor_Fc_Enable){
				Contactor_Fc_Open;
				Current_Contactor_Fc_Enable = sysControl.Expected_Contactor_Fc_Enable;
			}else{
				Contactor_Fc_Close;
				Current_Contactor_Fc_Enable = sysControl.Expected_Contactor_Fc_Enable;
			}
		}
		// 总输出接触器使能状态更新
		if(sysControl.Expected_Contactor_Load_Enable != Current_Contactor_Load_Enable){
			if(sysControl.Expected_Contactor_Load_Enable){
				Contactor_Load_Open;
				Current_Contactor_Load_Enable = sysControl.Expected_Contactor_Load_Enable;
			}else{
				Contactor_Load_Close;
				Current_Contactor_Load_Enable = sysControl.Expected_Contactor_Load_Enable;
			}
		}
		// Pca9555输出管脚使能状态更新
		if(ext_io_status.pca9555_current_state != ext_io_status.pca9555_expected_state){
			PCA9555_Write_IO_State(&hi2c1, pca9555_device_address, ext_io_status.pca9555_expected_state);
			ext_io_status.pca9555_current_state = ext_io_status.pca9555_expected_state;
		}
		osDelay(1);
	}
  /* USER CODE END Start_ActiveExtIO_Task02 */
}

/* USER CODE BEGIN Header_Start_Analog_Task03 */
/**
* @brief ADC 数据采集线程
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Analog_Task03 */
__weak void Start_Analog_Task03(void const * argument)
{
  /* USER CODE BEGIN Start_Analog_Task03 */
	uint32_t count = HAL_GetTick();//获取系统时间

	//resetFlowMeter();//流量计累计
  /* Infinite loop */
	while(true)
	{
		// 读取所有AD通道AD值
		read_all_adc1_values(my_analog_inputs.adc1_values, 5);
		read_all_adc2_values(my_analog_inputs.adc2_values, 4);
		// AD值转换为物理值
		my_analog_inputs.Power_Voltage.Current_Val = convert_adc_to_voltage(my_analog_inputs.adc1_values[0], 3.262, 0.022);			//电压转换
		my_analog_inputs.External_Hydrogen_Concentration.Current_Val = my_analog_inputs.adc1_values[1] /40.96;						//浓度ppm
		my_analog_inputs.Hydrogen_Cylinder_Pressure.Current_Val = (( my_analog_inputs.adc1_values[2] * 0.001221) - 0.5 ) * 12.5 ;	//气压转换
		my_analog_inputs.FC_Internal_Temperature.Current_Val = Get_Tempture(my_analog_inputs.adc1_values[3],10000,10020); 			//堆内温度转换
		my_analog_inputs.FC_External_Temperature.Current_Val = Get_Tempture(my_analog_inputs.adc1_values[4],10000,10020); 			//堆外
		my_analog_inputs.Shunt_A_Current.Current_Val = convert_adc_to_current(my_analog_inputs.adc2_values[0], 3.264, 75);			// shunt A 电堆输出
		my_analog_inputs.Shunt_A_Voltage.Current_Val = convert_adc_to_voltage(my_analog_inputs.adc2_values[1], 3.262, 0.0214);
		my_analog_inputs.Shunt_B_Current.Current_Val = convert_adc_to_current(my_analog_inputs.adc2_values[2], 3.264, 75); 			// shunt B 负载输出
		my_analog_inputs.Shunt_B_Voltage.Current_Val = convert_adc_to_voltage(my_analog_inputs.adc2_values[3], 3.262, 0.0213);

		//实时功率与电能计转换
		my_analog_inputs.Shunt_A_Power.Current_Val = power_calculation(my_analog_inputs.Shunt_A_Voltage.Current_Val , my_analog_inputs.Shunt_A_Current.Current_Val);
		my_analog_inputs.Shunt_A_Total_Energy.Current_Val += energy_calculation(my_analog_inputs.Shunt_A_Power.Current_Val , SAMPLING_INTERVAL);
		my_analog_inputs.Shunt_B_Power.Current_Val = power_calculation(my_analog_inputs.Shunt_B_Voltage.Current_Val , my_analog_inputs.Shunt_B_Current.Current_Val);
		my_analog_inputs.Shunt_B_Total_Energy.Current_Val += energy_calculation(my_analog_inputs.Shunt_B_Power.Current_Val , SAMPLING_INTERVAL);

		/**
		 * 采样范围
		 * $PowV=			0-120.00V, 	倍率100		uint16_t
		 * $ExtHC=			0-100.00%,
		 * $HCP=			0-65.00Mpa,
		 * $FcTemp=			0-80.00℃,  倍率100
		 * $ExtTemp=		0-80.00℃,  倍率100
		 * $ShuntA_C=		0-100.00A,	倍率100
		 * $ShuntA_V=		0-120.00V,	倍率100
		 * $ShuntB_C=		0-100.00A,	倍率100
		 * $ShuntB_V=		0-120.00V,	倍率100
		 * $ShuntA_Power=0-100.00KW,	倍率100
		 * $ShuntA_Total=0-100.00KWh,	倍率100
		 * $ShuntB_Power=0-100.00KW,	倍率100
		 * $ShuntB_Total=0-100.00KWh,	倍率100
		 */

		//#ifdef RS485_FLOWMETER
		//if(count + DELAY_TIME <= HAL_GetTick()){
		//	  queryFlowMeter();//获取流量计瞬时与累计流量
		//    count = HAL_GetTick();
		//}
		//#endif



		#ifdef USE_DEBUG_ANALOG
		RS232_1_printf("$ADC1_0=%d,$ADC1_1=%d,$ADC1_2=%d,$ADC1_3=%d,$ADC1_4=%d,",
			  my_analog_inputs.adc1_values[0],
			  my_analog_inputs.adc1_values[1],
			  my_analog_inputs.adc1_values[2],
			  my_analog_inputs.adc1_values[3],
			  my_analog_inputs.adc1_values[4]
		);
		RS232_1_printf("$ADC2_0=%d,$ADC2_1=%d,$ADC2_2=%d,$ADC2_3=%d\r\n",
			  my_analog_inputs.adc2_values[0],
			  my_analog_inputs.adc2_values[1],
			  my_analog_inputs.adc2_values[2],
			  my_analog_inputs.adc2_values[3]
		);
		#endif

		//FDCAN1_Send_Msg(can_Tdat,FDCAN_DLC_BYTES_8);
		osDelay(SAMPLING_INTERVAL);
		HAL_IWDG_Refresh(&hiwdg);//喂狗 超过2s复位
	}
  /* USER CODE END Start_Analog_Task03 */
}

/* RtosCallback01 function */
__weak void RtosCallback01(void const * argument)
{
  /* USER CODE BEGIN RtosCallback01 */

  /* USER CODE END RtosCallback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void SysDataInit(void){
//	sysControl.Expected_FC_Fan_Speed = 0;
//	sysControl.Expected_FC_Fan_Enable = true;
//	sysControl.Expected_DCDC_Enable;
//	sysControl.Expected_Heatsink_Fan_Enable;
//	sysControl.Expected_Hydrogen_Inlet_Valve_Enable;
//	sysControl.Expected_Hydrogen_Exhaust_Valve_Enable;
//	sysControl.Expected_Contactor_Fc_Enable;
//	sysControl.Expected_Contactor_Load_Enable;
}
/* USER CODE END Application */

