/*
 * w25q64.c
 *
 *  Created on: 2023年8月27日
 *      Author: edgar
 */

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_flash.h"
#include "w25q64.h"

//-----------------芯片内部flash操作

//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
#define FLASH_WAITETIME  50000          //FLASH等待超时时间
#define 	FLASH_TYPEPROGRAM_BYTE   0x00000000U
#define 	FLASH_TYPEPROGRAM_HALFWORD   0x00000001U
#define 	FLASH_TYPEPROGRAM_WORD   0x00000002U
#define 	FLASH_TYPEPROGRAM_DOUBLEWORD   0x00000003U

/* 读取flash数据 */
uint32_t ReadFlash( uint32_t faddr )
{
	return *(__IO uint32_t*)faddr;
}
/* 写flash数据 */
void WriteFlash_byte(uint32_t faddr,uint32_t data)
{
  HAL_FLASH_Unlock();						                           //解锁
  FLASH_WaitForLastOperation(300);                        // 等待FLASH操作完成 3000:超时时间
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,faddr,data);    /* 写操作 */
  FLASH_WaitForLastOperation(300);                        // 等待FLASH操作完成 3000:超时时间
  HAL_FLASH_Lock();                                        //上锁
}
/* 擦除操作 */
void easeFlash(uint8_t sector)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t err;
  HAL_FLASH_Unlock();					//解锁
  FLASH_WaitForLastOperation(3000);
  // 修改为适用于库版本1.2.2的成员名称和定义
   EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;  // 只是擦除操作
   EraseInitStruct.Banks = FLASH_BANK_1;                // 选择哪个Bank
   EraseInitStruct.Page = sector;                       // 要擦除的初始Flash页
   EraseInitStruct.NbPages = 1;                         // 一次只擦除一个扇区
  HAL_FLASHEx_Erase(&EraseInitStruct,&err);  /* 擦除某一扇区 */
  FLASH_WaitForLastOperation(3000);
  HAL_FLASH_Lock();                          //上锁
}

//检测flash
static uint8_t stmflash_get_error_status(void)
{
    uint32_t res;
    res = FLASH->SR;

    if (res & (1 << 0))return 1;    /* BSY = 1      , 忙 */
    if (res & (1 << 2))return 2;    /* PGERR = 1    , 编程错误*/
    if (res & (1 << 4))return 3;    /* WRPRTERR = 1 , 写保护错误 */

    return 0;   /* 没有任何错误 操作完成. */
}

static uint8_t stmflash_wait_done(uint32_t time)
{
    uint8_t res;

    do
    {
        res = stmflash_get_error_status();

        if (res != 1)
        {
            break;      /* 非忙, 无需等待了, 直接退出 */
        }

        time--;
    } while (time);

    if (time == 0)res = 0XFF;   /* 超时 */

    return res;
}

/**
 * @brief       擦除扇区
 * @param       saddr   : 扇区地址 0 ~ 256
 * @retval      执行结果
 *   @arg       0   : 已完成
 *   @arg       2   : 编程错误
 *   @arg       3   : 写保护错误
 *   @arg       0XFF: 超时
 */
uint8_t stmflash_erase_sector(uint32_t saddr)
{
    uint8_t res = 0;  /* STM32F103擦除的时候是指定半字地址 */
    res = stmflash_wait_done(0X5FFFFF);     /* 等待上次操作结束, >20ms */

    if (res == 0)
    {
        FLASH->CR |= 1 << 1;    /* 页擦除 */
        FLASH->ACR = saddr;      /* 设置页地址(实际是半字地址) */
        FLASH->CR |= 1 << 6;    /* 开始擦除 */
        res = stmflash_wait_done(0X5FFFFF); /* 等待操作结束, >20ms */

        if (res != 1)   /* 非忙 */
        {
            FLASH->CR &= ~(1 << 1); /* 清除页擦除标志 */
        }
    }

    return res;
}

//-------------------------------以上内部flash操作

/*内部函数声明区*/
static void LargeToSmall(uint8_t* type, uint8_t typeSize);
static HAL_StatusTypeDef w25q64_Transmit(uint8_t * T_pData, uint16_t T_Size);
static HAL_StatusTypeDef w25q64_Receive(uint8_t * R_pData, uint16_t R_Size);

/*内部函数定义区*/

//大端转小端
static void LargeToSmall(uint8_t* type, uint8_t typeSize)
{
	// 当大小小于1或者不是2的幂次位时退出
	if( typeSize <= 1 )
	{
		return;
	}
	for(uint8_t i=0; i<typeSize/2; i++)
	{
		*(type+i) = *(type+i) ^ *(type+typeSize-i-1);
		*(type+typeSize-i-1) = *(type+i) ^ *(type+typeSize-i-1);
		*(type+i) = *(type+i) ^ *(type+typeSize-i-1);
	}
}

/*
函数参数：
		1、T_pData：发送数据缓冲区中取出数据发送出去
		2、T_Size ：需要发送的数据的长度
*/
static HAL_StatusTypeDef w25q64_Transmit(uint8_t * T_pData, uint16_t T_Size)
{
	return HAL_SPI_Transmit(&W25Q_SPI, T_pData, T_Size, 0xff);
}

/*
函数参数：
		1、R_pData：接收数据并放置到接收数据缓冲区中
		2、R_Size ：需要接收的数据的长度
*/
static HAL_StatusTypeDef w25q64_Receive(uint8_t * R_pData, uint16_t R_Size)
{
	return HAL_SPI_Receive(&W25Q_SPI, R_pData, R_Size, 0xff);
}

/*
写使能或失能
	参数：
		Type:
			1、为1时使能
			2、为0时失能
*/
HAL_StatusTypeDef Write_En_De(uint8_t Type)
{
	uint8_t cmd;
	HAL_StatusTypeDef STD = HAL_ERROR;

	W25Q_CS_Level(0);

	switch(Type)
	{
		case 1:
			cmd = W25Q_W_ENA;
			break;
		case 0:
			cmd = W25Q_W_DIS;
			break;
		default:
			cmd = W25Q_W_DIS;
			break;
	}

	if(w25q64_Transmit(&cmd, 1) == HAL_OK)
	{
		STD = HAL_OK;
	}

	W25Q_CS_Level(1);
	return STD;
}

/*
读状态寄存器
	参数：Select
		1、为0时是寄存器1
		2、为1时是寄存器2
	参数：State（指针）
		1、返回的状态标志

	流程：先写入命令，然后读取状态
*/
HAL_StatusTypeDef Read_State_Reg(uint8_t Select, uint8_t* State)
{
	uint8_t cmd[4] = {0,0,0,0};
	HAL_StatusTypeDef STD = HAL_ERROR;

	W25Q_CS_Level(0);

	switch(Select)
	{
		case 0:
			cmd[0] = W25Q_R_STA_REG1;
			break;
		case 1:
			cmd[0] = W25Q_R_STA_REG2;
			break;
		default:
			cmd[0] = W25Q_R_STA_REG1;
			break;
	}

	if(w25q64_Transmit(cmd, 4) == HAL_OK)
	{
		if(w25q64_Receive(State,1) == HAL_OK)
		{
			STD = HAL_OK;
		}
	}

	W25Q_CS_Level(1);

	return STD;
}

/*
判忙
		用处：判断当前flash是否在忙碌状态
*/
void Judge_Busy(void)
{
//	uint8_t State;
//	do{
//		Read_State_Reg(0, &State);	//不要用指针类型局部变量传进去，必被卡死
//		State &= 0x01;
//	}while(State == 0x01);
}


/*
写状态寄存器
	参数：State（数组指针）
	参数解释：长度为两个字节的数组指针，
						第一个字节写入状态寄存器1；
						第二个字节写入状态寄存器2。

	流程：先写命令，再写状态
*/
HAL_StatusTypeDef Write_State_Reg(uint8_t * State)
{
	uint8_t cmd = W25Q_W_STA_REG_;
	HAL_StatusTypeDef STD = HAL_ERROR;

	Judge_Busy();
	Write_En_De(1);
	W25Q_CS_Level(0);
//	Judge_Busy();

	if(w25q64_Transmit(&cmd, 1) == HAL_OK)
	{
		if(w25q64_Transmit(State, 2) == HAL_OK)
		{
			STD = HAL_OK;
		}
	}

	W25Q_CS_Level(1);
	Write_En_De(0);
	return STD;
}

/*
读数据
			参数：R_Addr
						1、读取数据的地址
			参数：R_Data(数组指针)
						1、获取读取的数据
			参数：R_Size
						1、读取的数据的大小
*/
HAL_StatusTypeDef Read_Data(uint32_t R_Addr, uint8_t * R_Data, uint16_t R_Size)
{
	uint8_t cmd = W25Q_R_Dat;
	HAL_StatusTypeDef STD = HAL_ERROR;

	R_Addr <<= 8;	//只要24位，3个字节
	LargeToSmall((uint8_t *)&R_Addr, sizeof(R_Addr));

	Judge_Busy();	//判忙
	W25Q_CS_Level(0);
//	Judge_Busy();

	if(w25q64_Transmit(&cmd, 1) == HAL_OK)
	{
		if(w25q64_Transmit((uint8_t *)&R_Addr, 3) == HAL_OK)
		{
			if(w25q64_Receive(R_Data,R_Size) == HAL_OK)
			{
				STD = HAL_OK;
			}
		}
	}

	W25Q_CS_Level(1);

	return STD;
}


/*
页编程
	页的描述：1字节到 256 字节（一页）
	页写的前提条件：编程之前必须保证额你存空间是0xff,
									所以得先进行擦除（擦除后模式全为1）
	页写的注意事项：进行页编程时，如果数据字节数超过了 256 字节，
									地址将自动回到页的起始地址，覆盖掉之前的数据。

	参数：WriteAddr
				1、地址，三个字节地址
	参数：PW_Data（数组指针）
				1、要写入的数据，长度根据PW_size来定
				2、高位先传
	参数：PW_Size
				2、要写入的数据长度

	流程：先开写使能、判忙，再写命令，
				再写3个字节的地址，后写入数据，最后写失能
*/

HAL_StatusTypeDef Page_Write(uint32_t WriteAddr, uint8_t * PW_Data, uint16_t PW_Size)
{
	uint8_t cmd = W25Q_Page_Program;
	HAL_StatusTypeDef STD = HAL_ERROR;

	WriteAddr <<= 8;	//只要24位，3个字节
	LargeToSmall((uint8_t *)&WriteAddr, sizeof(WriteAddr));

	Judge_Busy();	//判忙
	Write_En_De(1);
	Judge_Busy();

	W25Q_CS_Level(0);

	if(w25q64_Transmit(&cmd, 1) == HAL_OK)
	{
		if(w25q64_Transmit((uint8_t *)&WriteAddr, 3) == HAL_OK)
		{
			if(w25q64_Transmit(PW_Data, PW_Size) == HAL_OK)
			{
				STD = HAL_OK;
			}
		}
	}

	W25Q_CS_Level(1);
	Judge_Busy();

	return STD;
}



/*
扇区擦除
	扇区的描述：W25Q64总共8MB,分为128块(每块64KB)，
							每块16个扇区，每个扇区4K个字节。
	扇区的备注：W25Q64的最小擦除单位就是一个扇区
							所以至少给芯片开辟一个4KB的缓存区，
							以防止一次性删除太多，而丢失数据。(显然单片机不会给他开辟这么大的空间)

	参数：Sector_Addr
				1、扇区地址，以4KB为单位寻址
				2、高位先发
*/
HAL_StatusTypeDef Sector_Erase(uint32_t Sector_Addr)
{
	uint8_t cmd = W25Q_Sector_Erase;
	HAL_StatusTypeDef STD = HAL_ERROR;

	//一个扇区有4KB的大小，
	//为了使找到对应的扇区地址，所以要乘以4KB
	Sector_Addr *= (1<<12);
	Sector_Addr <<= 8;	//只需要24位表示地址，并且高位先传
	LargeToSmall((uint8_t *)&Sector_Addr, sizeof(Sector_Addr));

	Judge_Busy();
	Write_En_De(1);
	W25Q_CS_Level(0);

	if(w25q64_Transmit(&cmd, 1) == HAL_OK)
	{
		if(w25q64_Transmit((uint8_t *)&Sector_Addr, 3) == HAL_OK)
		{
			STD = HAL_OK;
		}
	}

	W25Q_CS_Level(1);

	Judge_Busy();

	return STD;
}



/*
块擦除
	块的描述：W25Q64有8MB的容量，而8MB有128个块，所以1块有64kB的大小，
						所以这个函数一次能擦除64KB的大小。

	参数：Block_Addr
				1、块地址，共128个块，对应128个地址，以64K为单位寻址
				2、高位先传

	流程：先开写使能、判忙，再写命令，
				再写3个字节的地址，最后写失能
*/
HAL_StatusTypeDef	Block_Erase(uint32_t Block_Addr)
{
	uint8_t cmd = W25Q_Block_Erase;
	HAL_StatusTypeDef STD = HAL_ERROR;

	//总共有128个块，而一个块有64KB的大小，
	//为了使找到对应的块地址，所以要乘以64KB
	Block_Addr *= (1<<16);
	Block_Addr <<= 8;	//只需要24位表示地址，并且高位先传
	LargeToSmall((uint8_t *)&Block_Addr, sizeof(Block_Addr));

	Judge_Busy();
	Write_En_De(1);
	Judge_Busy();
	W25Q_CS_Level(0);

	if(w25q64_Transmit(&cmd, 1) == HAL_OK)
	{
		if(w25q64_Transmit((uint8_t *)&Block_Addr, 3) == HAL_OK)
		{
			STD = HAL_OK;
		}
	}

	W25Q_CS_Level(1);
	Judge_Busy();
	return STD;
}


/*
全片擦除
	描述：直接把芯片全部擦除
*/
HAL_StatusTypeDef Full_Erase(void)
{
	uint8_t cmd = W25Q_Full_Erase;
	HAL_StatusTypeDef STD = HAL_ERROR;

	Judge_Busy();
	Write_En_De(1);
	W25Q_CS_Level(0);

	if(w25q64_Transmit(&cmd, 1) == HAL_OK)
	{
		STD = HAL_OK;
	}

	W25Q_CS_Level(1);
	Judge_Busy();

	return STD;
}


/*
读ID
	描述：读3个字节分别是生产厂家、存储器类型、容量
*/
HAL_StatusTypeDef Read_Jedec_ID(uint8_t * R_Jedec_ID)
{
	uint8_t cmd = W25Q_JEDEC_ID;
	HAL_StatusTypeDef STD = HAL_ERROR;

	Judge_Busy();
	W25Q_CS_Level(0);
//	Judge_Busy();

	if(w25q64_Transmit(&cmd, 1) == HAL_OK)
	{
		if(w25q64_Receive(R_Jedec_ID, 3) == HAL_OK)
		{
			STD = HAL_OK;
		}
	}

	W25Q_CS_Level(1);

	return STD;
}
