/*
 * rs485_display.c
 *
 *  Created on: 2023年7月23日
 *      Author: edgar
 */

#include "hmi/hmi_driver.h"
#include "hmi/hmi_user_uart.h"
#include "hmi/cmd_queue.h"
#include "hmi/cmd_process.h"
#include "rs485_display.h"
#include "main.h"
#include "usart.h"

extern AnalogInputs_TypeDef my_analog_inputs;
extern SysControl_TypeDef sysControl;
extern DeviceStatus_t Sys_status;
extern UART_ReceiveTypeDef uart3_receive;

uint8  hmi_cmd_buffer[CMD_MAX_SIZE];			//指令缓存
static uint16 current_screen_id = 0;			//当前画面ID
static uint16 submenu_screen_id = 1;		//当前子菜单页面ID
static uint8 sys_status_flag;
static int32 progress_value = 0;				//进度条测试值
static int32 test_value = 0;					//测试值

#define MAX_GRAPH_DATA_POINTS 182
//1:FcTemp,2:ExtTemp,3:EHC
static float graph_value[3][MAX_GRAPH_DATA_POINTS];
static uint8 current_index  = 0;

static uint8 update_en = 0;					//屏幕跟新标记
static int32 meter_flag = 0;					//仪表指针往返标志位

bool ExhaustFlag = false;

//---------------------曲线控件实现
void FindMaxMin(float arr[][MAX_GRAPH_DATA_POINTS], int rows, int cols, float *max, float *min) {
    *max = arr[0][0]; // 初始化最大值为负无穷
    *min = arr[0][0];  // 初始化最小值为正无穷

    // 遍历整个数组，找到最大值和最小值
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (arr[i][j] > *max) {
                *max = arr[i][j]; // 更新最大值
            }
            if (arr[i][j] < *min) {
                *min = arr[i][j]; // 更新最小值
            }
        }
    }
}

int mapFloatToInt(float x, float inMin, float inMax, int outMin, int outMax) {
    // 首先将输入值映射到0到1的范围
    float normalizedValue = (x - inMin) / (inMax - inMin);
    // 然后将0到1的范围映射到输出范围
    int result = outMin + (int)(normalizedValue * (outMax - outMin));
    return result;
}

// 向数组中添加新值并进行循环移动
void AddNewValue(float data1, float data2, float data3) {
	current_index = current_index + 1;
    // 添加新值到最后一个位置
    graph_value[0][current_index] = data1;
    //GraphChannelDataAdd(1,4,0,(uint8_t*)&graph_value[0][current_index],1);
    graph_value[1][current_index] = data2;
    graph_value[2][current_index] = data3;
    if(current_index == MAX_GRAPH_DATA_POINTS - 1)current_index = 0;
}



__weak void Rs485_Display_Task04(void const * argument){

	uint32 timer_tick_last_update = 0;	//屏幕上次跟新的时间
	qsize size = 0;
	float graph_max = 0.0;
	float graph_min = 0.0;
	char floatString[20];

	while(true)
	{
		//从串口接收区读取数据
		if (uart3_receive.recv_end_flag)
		{
			for(uint8_t i = 0;i < uart3_receive.rx_len; i++)
			{
				queue_push(uart3_receive.rx_buffer[i]);
			}
			//清除缓冲区和计数器
			uart3_receive.rx_len = 0;
			uart3_receive.recv_end_flag = false;
			memset(uart3_receive.rx_buffer, 0, UART3_RX_BUFFER_SIZE);
			//重启DMA接收
			HAL_UART_Receive_DMA(&huart3, uart3_receive.rx_buffer, UART3_RX_BUFFER_SIZE);
		}

		size = queue_find_cmd(hmi_cmd_buffer,CMD_MAX_SIZE);	//从缓冲区获取一条指令
		if(size>0&&hmi_cmd_buffer[1]!=0x07)//接收到指令 ，及判断是否为开机提示
		{
			ProcessMessage((PCTRL_MSG)hmi_cmd_buffer, size);//指令处理
		}
		else if(size>0&&hmi_cmd_buffer[1]==0x07)//如果为指令0x07就软重置STM32
		{
			__disable_fault_irq();
			NVIC_SystemReset();
		}

		//    特别注意
		//    MCU不要频繁向串口屏发送数据，否则串口屏的内部缓存区会满，从而导致数据丢失(缓冲区大小：标准型8K，基本型4.7K)
		//    1) 一般情况下，控制MCU向串口屏发送数据的周期大于100ms，就可以避免数据丢失的问题；
		//    2) 如果仍然有数据丢失的问题，请判断串口屏的BUSY引脚，为高时不能发送数据给串口屏。

		//    TODO: 添加用户代码
		//    数据有更新时标志位发生改变，定时100毫秒刷新屏幕

		if(timer_tick_last_update + 1000 <= HAL_GetTick()){
			update_en = 0;

			if(current_screen_id == 0 && submenu_screen_id == 1){
				AddNewValue(my_analog_inputs.FC_Internal_Temperature.Current_Val,
			                my_analog_inputs.FC_External_Temperature.Current_Val,
			                my_analog_inputs.External_Hydrogen_Concentration.Current_Val);
				FindMaxMin(graph_value, 3, MAX_GRAPH_DATA_POINTS - 1, &graph_max, &graph_min); // 调用函数查找最大和最小值
				uint8 data1 = mapFloatToInt(my_analog_inputs.FC_Internal_Temperature.Current_Val, graph_min, graph_max, 0, 112);
				uint8 data2 = mapFloatToInt(my_analog_inputs.FC_External_Temperature.Current_Val, graph_min, graph_max, 0, 112);
				uint8 data3 = mapFloatToInt(my_analog_inputs.External_Hydrogen_Concentration.Current_Val, graph_min, graph_max, 0, 112);
				GraphChannelDataAdd(1,4,0,&data1,1);
				osDelay(5);
				GraphChannelDataAdd(1,4,1,&data2,1);
				osDelay(5);
				GraphChannelDataAdd(1,4,2,&data3,1);
				osDelay(5);
				//页面1中的曲线控件刻度值更新
				BatchBegin(1);//批量跟新页面1
				sprintf(floatString, "%.2f", graph_max);
				BatchSetText(27,floatString);
				sprintf(floatString, "%.2f", graph_min);
				BatchSetText(28,floatString);
				BatchEnd();//结束批量跟新
				osDelay(5);
			}

			UpdateUI();
			timer_tick_last_update = HAL_GetTick();;
		}


	}
}

/*!
*  \brief  消息处理流程
*  \param msg 待处理消息
*  \param size 消息长度
*/
void ProcessMessage( PCTRL_MSG msg, uint16 size )
{
	uint8 cmd_type = msg->cmd_type;								//指令类型
	uint8 ctrl_msg = msg->ctrl_msg;								//消息的类型
	uint8 control_type = msg->control_type;						//控件类型
	uint16 screen_id = PTR2U16(&msg->screen_id);				//画面ID
	uint16 control_id = PTR2U16(&msg->control_id);				//控件ID
	uint32 value = PTR2U32(msg->param);							//数值


	switch(cmd_type)
	{
	case NOTIFY_TOUCH_PRESS:												//触摸屏按下
	case NOTIFY_TOUCH_RELEASE:												//触摸屏松开
		NotifyTouchXY(hmi_cmd_buffer[1],PTR2U16(hmi_cmd_buffer+2),PTR2U16(hmi_cmd_buffer+4));
		break;
	case NOTIFY_WRITE_FLASH_OK:											//写FLASH成功
		NotifyWriteFlash(1);
		break;
	case NOTIFY_WRITE_FLASH_FAILD:										//写FLASH失败
		NotifyWriteFlash(0);
		break;
	case NOTIFY_READ_FLASH_OK:											//读取FLASH成功
		NotifyReadFlash(1,hmi_cmd_buffer+2,size-6);							//去除帧头帧尾
		break;
	case NOTIFY_READ_FLASH_FAILD:											//读取FLASH失败
		NotifyReadFlash(0,0,0);
		break;
	case NOTIFY_READ_RTC:													//读取RTC时间
		NotifyReadRTC(hmi_cmd_buffer[2],hmi_cmd_buffer[3],hmi_cmd_buffer[4],hmi_cmd_buffer[5],hmi_cmd_buffer[6],hmi_cmd_buffer[7],hmi_cmd_buffer[8]);
		break;
	case NOTIFY_CONTROL:
		{
			if(ctrl_msg==MSG_GET_CURRENT_SCREEN)						//画面ID变化通知
			{
				NotifyScreen(screen_id);									//画面切换调动的函数
			}
			else
			{
				switch(control_type)
				{
				case kCtrlButton:											//按钮控件
					NotifyButton(screen_id,control_id,msg->param[1]);
					break;
				case kCtrlText:												//文本控件
					NotifyText(screen_id,control_id,msg->param);
					break;
				case kCtrlProgress:											//进度条控件
					NotifyProgress(screen_id,control_id,value);
					break;
				case kCtrlSlider:											//滑动条控件
					NotifySlider(screen_id,control_id,value);
					break;
				case kCtrlMeter:												//仪表控件
					NotifyMeter(screen_id,control_id,value);
					break;
				case kCtrlMenu:												//菜单控件
					NotifyMenu(screen_id,control_id,msg->param[0],msg->param[1]);
					break;
				case kCtrlSelector:											//选择控件
					NotifySelector(screen_id,control_id,msg->param[0]);
					break;
				case kCtrlRTC:												//倒计时控件
					NotifyTimer(screen_id,control_id);
					break;
				default:
					break;
				}
			}
			break;
		}
	case NOTIFY_HandShake:			//握手通知
		NOTIFYHandShake();
		break;
	default:
		break;
	}
}

/*!
*  \brief  握手通知
*/
void NOTIFYHandShake()
{

}

/*!
*  \brief  画面切换通知
*  \details  当前画面改变时(或调用GetScreen)，执行此函数
*  \param screen_id 当前画面ID
*/
void NotifyScreen(uint16 screen_id)
{
	current_screen_id = screen_id;
}

/*!
*  \brief  触摸坐标事件响应
*  \param press 1按下触摸屏，3松开触摸屏
*  \param x x坐标
*  \param y y坐标
*/
void NotifyTouchXY(uint8 press,uint16 x,uint16 y)
{
    //TODO: 添加用户代码
}


/*!
*  \brief  更新数据
*  注意：若串口屏没有RTC 请先看下工程画面ID，对应工功能画面的编号与下面数据刷新的一致
*/
void UpdateUI()
{
	char floatString[20];
	//如果当前为画面0 更新启停图标
	if(current_screen_id == 0 && Sys_status != sys_status_flag){
		BatchBegin(0);//批量跟新页面0
		if(Sys_status != DEVICE_RUNNING){
			BatchSetFrame(10,0);
			sys_status_flag = Sys_status;
		}else{
			BatchSetFrame(10,1);
			sys_status_flag = Sys_status;
		}
		BatchEnd();//结束批量跟新
		osDelay(10);
	}

	if(current_screen_id == 0 && (submenu_screen_id == 1 || submenu_screen_id == 2)){
		BatchBegin(1);//批量跟新页面1
		sprintf(floatString, "%.2f", my_analog_inputs.FC_Internal_Temperature.Current_Val);
		BatchSetText(3,floatString);//堆温度
		sprintf(floatString, "%.2f", my_analog_inputs.Shunt_A_Voltage.Current_Val);
		BatchSetText(15,floatString);//堆电压
		sprintf(floatString, "%.2f", my_analog_inputs.Shunt_A_Current.Current_Val);
		BatchSetText(16,floatString);//堆电流
		sprintf(floatString, "%.2f", my_analog_inputs.Shunt_A_Power.Current_Val);
		BatchSetText(17,floatString);//堆功率
		sprintf(floatString, "%.2f", my_analog_inputs.Shunt_A_Total_Energy.Current_Val);
		BatchSetText(18,floatString);//堆电能计
		sprintf(floatString, "%.2f", my_analog_inputs.Shunt_B_Voltage.Current_Val);
		BatchSetText(19,floatString);//终端电压
		sprintf(floatString, "%.2f", my_analog_inputs.Shunt_B_Current.Current_Val);
		BatchSetText(20,floatString);//终端电流
		sprintf(floatString, "%.2f", my_analog_inputs.Shunt_B_Power.Current_Val);
		BatchSetText(21,floatString);//终端功率
		sprintf(floatString, "%.2f", my_analog_inputs.Shunt_B_Total_Energy.Current_Val);
		BatchSetText(22,floatString);//终端电能计
		sprintf(floatString, "%.2f", my_analog_inputs.FC_External_Temperature.Current_Val);
		BatchSetText(23,floatString);//堆外温度
		sprintf(floatString, "%.2f", my_analog_inputs.External_Hydrogen_Concentration.Current_Val);
		BatchSetText(24,floatString);//氢浓度
		sprintf(floatString, "%d", map_0_to_100_to_12_to_100(sysControl.Expected_FC_Fan_Speed));
		BatchSetText(25,floatString);//风扇转速
		sprintf(floatString, "%.2f", my_analog_inputs.Power_Voltage.Current_Val);
		BatchSetText(26,floatString);//fcu输入电压
		sprintf(floatString, "%.2f", my_analog_inputs.Hydrogen_Cylinder_Pressure.Current_Val);
		BatchSetText(29,floatString);//气瓶气压
		BatchEnd();//结束批量跟新
		osDelay(10);
	}

	if(current_screen_id == 0 && submenu_screen_id == 1){
		BatchBegin(1);//批量跟新页面1
		switch(sysControl.Expected_FC_Fan_Enable){//风扇使能
			case false:
				BatchSetFrame(9,DISPLAY_ICON_Y);
				break;
			case true:
				BatchSetFrame(9,DISPLAY_ICON_G);
				break;
			default:
			break;
		}
		switch(sysControl.Expected_DCDC_Enable){//DCDC
				case false:
					BatchSetFrame(11,DISPLAY_ICON_Y);
					break;
				case true:
					BatchSetFrame(11,DISPLAY_ICON_G);
					break;
				default:
				break;
		}
		switch(sysControl.Expected_Hydrogen_Inlet_Valve_Enable){//进气阀
				case false:
					BatchSetFrame(6,DISPLAY_ICON_Y);
					break;
				case true:
					BatchSetFrame(6,DISPLAY_ICON_G);
					break;
				default:
				break;
		}
		switch(sysControl.Expected_Hydrogen_Exhaust_Valve_Enable){//燃料电池接触器
					case false:
						BatchSetFrame(10,DISPLAY_ICON_Y);
						break;
					case true:
						BatchSetFrame(10,DISPLAY_ICON_G);
						break;
					default:
					break;
		}
		switch(sysControl.Expected_Hydrogen_Exhaust_Valve_Enable){//终端接触器
					case false:
						BatchSetFrame(12,DISPLAY_ICON_Y);
						break;
					case true:
						BatchSetFrame(12,DISPLAY_ICON_G);
						break;
					default:
					break;
		}
		BatchEnd();//结束批量跟新
		osDelay(10);
		//更新曲线控件
		//GraphChannelDataAdd(1,4,0,&graph_value[0],1);
	}else if(current_screen_id == 0 && submenu_screen_id == 2){

	}else if(current_screen_id == 0 && submenu_screen_id == 3){

	}


	//SetTextFloat(1,3,my_analog_inputs.FC_Internal_Temperature.Current_Val,1,1);
}

/*!
*  \brief  按钮控件通知
*  \details  当按钮状态改变(或调用GetControlValue)时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param state 按钮状态：0弹起，1按下
*/
void NotifyButton(uint16 screen_id, uint16 control_id, uint8  state)
{
	if(screen_id == 0){
		if(control_id == 1 && state == 1){//当前子页面1
			submenu_screen_id = 1;
		}
		if(control_id == 2 && state == 1){//当前子页面2
			submenu_screen_id = 2;
		}
		if(control_id == 3 && state == 1){//当前子页面3
			submenu_screen_id = 3;
		}
		if(control_id == 4 && state == 1){//如果启停按钮被按下
			SetScreen(4);//跳转到画面4
			if(Sys_status == DEVICE_RUNNING){
				//SetTextValue(4,4,"STOPPED FC?");
				SetTextValue(4,4,"要立即停止电堆吗?");
			}else if(Sys_status == DEVICE_STOPPED){
				SetTextValue(4,4,"要立即启动电堆吗?");
			}
		}
	}
	if(screen_id == 4){
		if(control_id == 2 && state == 1){//如果页面四中的yes按钮被按下
			SetScreen(0);
			if(Sys_status == DEVICE_RUNNING){
				Sys_status = DEVICE_STOPPED;
				Record_SetEvent(2,2,0,0);
				Record_ResetEvent(2,2,1,0);
				StopTimer(0,12);//停止计时
			}else if(Sys_status == DEVICE_STOPPED){
				Sys_status = DEVICE_RUNNING;
				Record_SetEvent(2,2,1,0);
				Record_ResetEvent(2,2,0,0);
				StartTimer(0,12);//开始计时
			}
		}
		if(control_id == 3 && state == 1){//如果页面四中的no按钮被按下
			SetScreen(0);
		}
	}
}

/*!
*  \brief  文本控件通知
*  \details  当文本通过键盘更新(或调用GetControlValue)时，执行此函数
*  \details  文本控件的内容以字符串形式下发到MCU，如果文本控件内容是浮点值，
*  \details  则需要在此函数中将下发字符串重新转回浮点值。
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param str 文本控件内容
*/
void NotifyText(uint16 screen_id, uint16 control_id, uint8 *str)
{

}

/*!
*  \brief  进度条控件通知
*  \details  调用GetControlValue时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param value 值
*/
void NotifyProgress(uint16 screen_id, uint16 control_id, uint32 value)
{

}

/*!
*  \brief  滑动条控件通知
*  \details  当滑动条改变(或调用GetControlValue)时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param value 值
*/
void NotifySlider(uint16 screen_id, uint16 control_id, uint32 value)
{

}

/*!
*  \brief  仪表控件通知
*  \details  调用GetControlValue时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param value 值
*/
void NotifyMeter(uint16 screen_id, uint16 control_id, uint32 value)
{
    //TODO: 添加用户代码
}

/*!
*  \brief  菜单控件通知
*  \details  当菜单项按下或松开时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param item 菜单项索引
*  \param state 按钮状态：0松开，1按下
*/
void NotifyMenu(uint16 screen_id, uint16 control_id, uint8 item, uint8 state)
{
    //TODO: 添加用户代码
}

/*!
*  \brief  选择控件通知
*  \details  当选择控件变化时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param item 当前选项
*/
void NotifySelector(uint16 screen_id, uint16 control_id, uint8  item)
{

}

/*!
*  \brief  定时器超时通知处理
*  \param screen_id 画面ID
*  \param control_id 控件ID
*/
void NotifyTimer(uint16 screen_id, uint16 control_id)
{

}

/*!
*  \brief  读取用户FLASH状态返回
*  \param status 0失败，1成功
*  \param _data 返回数据
*  \param length 数据长度
*/
void NotifyReadFlash(uint8 status,uint8 *_data,uint16 length)
{
    //TODO: 添加用户代码
}

/*!
*  \brief  写用户FLASH状态返回
*  \param status 0失败，1成功
*/
void NotifyWriteFlash(uint8 status)
{
    //TODO: 添加用户代码
}

/*!
*  \brief  读取RTC时间，注意返回的是BCD码
*  \param year 年（BCD）
*  \param month 月（BCD）
*  \param week 星期（BCD）
*  \param day 日（BCD）
*  \param hour 时（BCD）
*  \param minute 分（BCD）
*  \param second 秒（BCD）
*/
void NotifyReadRTC(uint8 year,uint8 month,uint8 week,uint8 day,uint8 hour,uint8 minute,uint8 second)
{

}
