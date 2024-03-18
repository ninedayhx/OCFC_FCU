
#include "usart.h"
#include "main.h"
#include <stdarg.h>

extern UART_ReceiveTypeDef uart3_receive;

extern AnalogInputs_TypeDef my_analog_inputs;

static uint16_t CRC_Check(uint8_t*,uint8_t);  //CRC校验

/*
* @name   CRC_Check
* @brief  CRC校验
* @param  CRC_Ptr->数组指针，LEN->长度
* @retval CRC校验值
*/
uint16_t CRC_Check(uint8_t *CRC_Ptr,uint8_t LEN)
{
	uint16_t CRC_Value = 0;
	uint8_t  i         = 0;
	uint8_t  j         = 0;

	CRC_Value = 0xffff;
	for(i=0;i<LEN;i++)
	{
		CRC_Value ^= *(CRC_Ptr+i);
		for(j=0;j<8;j++)
		{
			if(CRC_Value & 0x00001)
				CRC_Value = (CRC_Value >> 1) ^ 0xA001;
			else
				CRC_Value = (CRC_Value >> 1);
		}
	}
	CRC_Value = ((CRC_Value >> 8) +  (CRC_Value << 8)); //交换高低字节
	return CRC_Value;
}

/**
 * RS485总线通信，使用UART3，这是RS485专用的printf函数
*/
void RS485_printf (char *fmt, ...)
{
    char buff[USART3_REC_LEN+1];  //用于存放转换后的数据 [长度]
    uint16_t i=0;
    va_list arg_ptr;
    va_start(arg_ptr,fmt);
    vsnprintf(buff, USART3_REC_LEN+1,fmt,arg_ptr);//数据转换
    i=strlen(buff);//得出数据长度
    if(strlen(buff)>USART3_REC_LEN)i=USART3_REC_LEN;//如果长度大于最大值，则长度等于最大值（多出部分忽略）
    HAL_UART_Transmit(&huart3,(uint8_t *)buff,i,0Xffff);//串口发送函数（串口号，内容，数量，溢出时间）
    va_end(arg_ptr);
}

void RS485_3_sendModbusFrame(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4,
                             uint8_t byte5, uint8_t byte6, uint8_t byte7, uint8_t byte8)
{
    uint8_t modbusFrame[] = {byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8};
    HAL_UART_Transmit(&huart3, modbusFrame, sizeof(modbusFrame), HAL_MAX_DELAY);
}

/**
 * 读取流量计参数
 * 02 03 00 02 00 06 64 3B
 * 02 03 0C 00 00 15 7C 00 00 00 01 02 06 00 00 2D 4F
 * 02 03 00 02 00 06 64 3B 02 03 0C 00 00 15 7C 00 00 00 01 02 61 00 00 9C 90
 */
void process_RS485_uart3_command(void)
{
	if (uart3_receive.recv_end_flag)
	{
		if(uart3_receive.rx_buffer[1] == 0x02 &&
			uart3_receive.rx_buffer[2] == 0x03)
		{
			if(uart3_receive.rx_buffer[3] ==0x0C)
			{
				//瞬时流量
				uint16_t shooting_flow = ((uint16_t)uart3_receive.rx_buffer[6] << 8) | uart3_receive.rx_buffer[7];
				//累积流量
				uint16_t integrated_flow_int = ((uint16_t)uart3_receive.rx_buffer[10] << 8) | uart3_receive.rx_buffer[11];
				uint16_t integrated_flow_float = ((uint16_t)uart3_receive.rx_buffer[12] << 8) | uart3_receive.rx_buffer[13];

				my_analog_inputs.Shooting_Flow = (float)shooting_flow /1000;
				my_analog_inputs.Integrated_Flow = (float)integrated_flow_int + ((float)integrated_flow_float / 1000);
			}
		}

		//清除缓存和计数器
		uart3_receive.rx_len = 0;
		uart3_receive.recv_end_flag = false;
		memset(uart3_receive.rx_buffer, 0, UART3_RX_BUFFER_SIZE);
//		for(uint8_t i = 0; i < UART3_RX_BUFFER_SIZE ; i++){
//			uart3_receive.rx_buffer[i] = 0x00;
//		}
		// 重新启动接收
		HAL_UART_Receive_DMA(&huart3, uart3_receive.rx_buffer, UART3_RX_BUFFER_SIZE);
	}
}
/**
 * 问询流量计参数
 */
void queryFlowMeter(){
	RS485_3_sendModbusFrame(0x02, 0x03, 0x00, 0x02, 0x00, 0x06, 0x64, 0x3B);
	//osDelay(5);
	process_RS485_uart3_command();

}

/**
 * 流量计累计值清零函数
 */
void resetFlowMeter()
{
    // 清零累计值的Modbus帧数据
    // 使用发送函数发送Modbus帧
    RS485_3_sendModbusFrame(0x02, 0x06, 0x00, 0x04, 0x00, 0x00, 0xC8, 0x38);
    osDelay(10);
    RS485_3_sendModbusFrame(0x02, 0x06, 0x00, 0x05, 0x00, 0x00, 0x99, 0xF8);
    osDelay(10);
    RS485_3_sendModbusFrame(0x02, 0x06, 0x00, 0x06, 0x00, 0x00, 0x69, 0xF8);
}

/**
 * 问询电池巡检
 */
void queryBatteryMonitor(){
	//波特率9600
	//发送问询帧：01 03 00 60 00 40 44 24
	RS485_3_sendModbusFrame(0x01, 0x03, 0x00, 0x60, 0x00, 0x40, 0x44, 0x24);

}
