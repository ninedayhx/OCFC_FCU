#include "usart.h"
#include "adc.h"
#include "main.h"
#include <stdarg.h>
#include <ctype.h>

extern UART_ReceiveTypeDef uart1_receive;
extern UART_ReceiveTypeDef uart2_receive;
extern AnalogInputs_TypeDef my_analog_inputs;
extern SysControl_TypeDef sysControl;

//extern DeviceStatus_t SysCurrentStatus;
extern DeviceStatus_t Sys_status;

void str_to_upper(char *str);
void str_to_lower(char *str);
uint8_t strnicmp(const char *s1, const char *s2, uint8_t n);

// 将字符串转换为全大写
void str_to_upper(char *str)
{
    while(*str)
    {
        *str = toupper((unsigned char) *str);
        str++;
    }
}

// 将字符串转换为全小写
void str_to_lower(char *str)
{
    while(*str)
    {
        *str = tolower((unsigned char) *str);
        str++;
    }
}

// 比较两个字符串是否相等（不区分大小写）
uint8_t strnicmp(const char *s1, const char *s2, uint8_t n)
{
    char c1, c2;
    uint8_t count = 0;
    do
    {
        c1 = tolower((unsigned char)*s1++);
        c2 = tolower((unsigned char)*s2++);
        count++;
    }
    while (c1 == c2 && c1 != '\0' && count < n);
    return c1 - c2;
}

/*
RS232通信，使用UART1
这是RS232专用的printf函数
*/
/**
 * RS232通信，使用UART1
 *
 */
void RS232_1_printf (char *fmt, ...)
{
	char buff[USART1_REC_LEN+1];  //用于存放转换后的数据 [长度]
	uint16_t i=0;
	va_list arg_ptr;
	va_start(arg_ptr,fmt);
	vsnprintf(buff, USART1_REC_LEN+1,fmt,arg_ptr);//数据转换
	i=strlen(buff);
	if(strlen(buff)>USART1_REC_LEN)i=USART1_REC_LEN;
	HAL_UART_Transmit(&huart1,(uint8_t *)buff,i,0Xffff);
	//HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buff, i);//DMA方式发送串口数据
	va_end(arg_ptr);
}

void RS232_2_sendModbusFrame(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4,
                             uint8_t byte5, uint8_t byte6, uint8_t byte7, uint8_t byte8)
{
    uint8_t modbusFrame[] = {byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8};
    HAL_UART_Transmit(&huart2, modbusFrame, sizeof(modbusFrame), HAL_MAX_DELAY);
}

void process_RS232_uart1_command(void)
{
	if(uart1_receive.recv_end_flag) //接收完成标志
	{
		//处理串口指令
		if(strnicmp(uart1_receive.rx_buffer, "help" ,4) == 0)
		{
			//执行指令1的操作
			//...
			RS232_1_printf("Usart_help....\n");
		}
		else if(strnicmp(uart1_receive.rx_buffer, "run" ,3) == 0)
		{
			//执行指令2的操作
			//...

			Sys_status = DEVICE_RUNNING;
			RS232_1_printf("Usart_run_ok\n");
		}
		else if(strnicmp(uart1_receive.rx_buffer, "stop" ,4) == 0)
		{
			//执行指令2的操作
			//...
			Sys_status = DEVICE_STOPPED;
			RS232_1_printf("Usart_stop_ok\n");
		}
		else if(strnicmp(uart1_receive.rx_buffer, "data" ,4) == 0)
		{
			//执行指令2的操作
			//...
			RS232_1_printf("$PowV=%.2f,$FcTemp=%.2f,$ExtTemp=%.2f,$ShuntA_C=%.2f,$ShuntA_V=%.2f,$ShuntB_C=%.2f,$ShuntB_V=%.2f",
				my_analog_inputs.Power_Voltage.Current_Val,
				my_analog_inputs.FC_Internal_Temperature.Current_Val,
				my_analog_inputs.FC_External_Temperature.Current_Val,
				my_analog_inputs.Shunt_A_Current.Current_Val,
				my_analog_inputs.Shunt_A_Voltage.Current_Val,
				my_analog_inputs.Shunt_B_Current.Current_Val,
				my_analog_inputs.Shunt_B_Voltage.Current_Val
			);
			RS232_1_printf(",$ShuntA_Power=%.3f,$ShuntA_Total=%.4f,$ShuntB_Power=%.3f,$ShuntB_Total=%.4f\r\n",
				my_analog_inputs.Shunt_A_Power.Current_Val,
				my_analog_inputs.Shunt_A_Total_Energy.Current_Val,
				my_analog_inputs.Shunt_B_Power.Current_Val,
				my_analog_inputs.Shunt_B_Total_Energy.Current_Val
			);
			RS232_1_printf("SS=%.3f,LL=%.3f \r\n",my_analog_inputs.Shooting_Flow,my_analog_inputs.Integrated_Flow);
		}
		else if(strncmp(uart1_receive.rx_buffer, "setrtc_", 7) == 0)
		{
//			uint8_t hour, minute, second;
//			const char* data = "2:2:52";
//
//			if (scanf(data, "%hhu:%hhu:%hhu", &hour, &minute, &second) == 3)
//			{
//				RS232_1_printf("RTC set to %02d:%02d:%02d\n", hour, minute, second);
//			}
//			else
//			{
//				RS232_1_printf("Invalid time format\n");
//			}
		}

		else
		{
			//无效指令
			//...
			RS232_1_printf("Rxlen:%d,",uart1_receive.rx_len);
			RS232_1_printf("Invalid command !\n");
		}

		//清除缓存和计数器
		uart1_receive.rx_len = 0;
		uart1_receive.recv_end_flag = false;
		memset(uart1_receive.rx_buffer, 0, UART1_RX_BUFFER_SIZE);


	}
	//重新打开DMA接收
	 HAL_UART_Receive_DMA(&huart1, uart1_receive.rx_buffer, UART1_RX_BUFFER_SIZE);
}

void process_RS232_uart2_command(void)
{
	if(uart2_receive.recv_end_flag) //接收完成标志
	{
		//处理串口指令
		RS232_1_printf("uart2_Rx_ok!\n");

		//清除缓存和计数器
		uart2_receive.rx_len = 0;
		uart2_receive.recv_end_flag = false;
		memset(uart2_receive.rx_buffer, 0, UART2_RX_BUFFER_SIZE);
	}
	//重新打开DMA接收
	 HAL_UART_Receive_DMA(&huart2, uart2_receive.rx_buffer, UART2_RX_BUFFER_SIZE);
}
