

#ifndef __USER_APP__H
#define __USER_APP__H

#ifdef __cplusplus
 extern "C" {
#endif	

#include "stm32l0xx_hal.h"
#include "adc.h"

#define GET_CRC(__X__,DATA)    ((__X__)[1] = ((DATA & 0xff00) >> 8), (__X__)[0] = (DATA & 0x00ff))

#define POSITIVE_ADDR			0x08007FFC

#define COMPENSATE_ADDR		0x08007FF8

/*
 *	FlashReadPage:		读取1页数据
 *	参数PageAddr：		页地址
 *	参数pBuffer：		保存读取到的数据的指针
 *	返回值：			1成功 0失败	
 */
uint8_t FlashReadPage(uint32_t PageAddr, uint32_t *pBuffer);
/*
 *	FlashWritePage:		写1页数据
 *	参数PageAddr：		页地址
 *	参数pBuffer：		用于写入Flash中的数据指针
 *	返回值：			1成功 0失败	
 */
uint8_t FlashWritePage( uint32_t PageAddr, uint32_t *pPageBuffer);

/*
 *	FlashWrite32:		写4字节(32位)数据
 *	参数WriteAddr：		该数据在Flash中的地址
 *	pBuffer:			用于写入Flash中的数据指针
 *	NumToWrite			数据长度(小于页大小/4)
 *	返回值：			1成功 0失败			
 */
uint8_t FlashWrite32( uint32_t WriteAddr, uint32_t * pBuffer, uint16_t NumToWrite );
/*
 *	FlashWrite16:		写2字节(16位)数据
 *	参数WriteAddr：		该数据在Flash中的地址
 *	pBuffer:			用于写入Flash中的数据指针
 *	NumToWrite			数据长度(小于页大小/2)
 *	返回值：			1成功 0失败			
 */
uint8_t FlashWrite16( uint32_t WriteAddr, uint16_t * pBuffer, uint16_t NumToWrite );

/*
 *	FlashRead32:		读取4字节(32位)数据
 *	参数ReadAddr：		该数据在Flash中的地址
 *	返回值：			返回读取到的数据		
 */
uint32_t FlashRead32(uint32_t ReadAddr );
/*
 *	FlashRead16:		读取2字节(16位)数据
 *	参数ReadAddr：		该数据在Flash中的地址
 *	返回值：			返回读取到的数据		
 */
uint16_t FlashRead16(uint32_t ReadAddr );
	 
void writeFlashTest(void);

void printFlashTest(void);
	 
void Rs485Init(void);
	
void Rs485RevceHandle(int16_t *SenSorBuf); 

uint16_t CalcCRC16(uint8_t *data, uint8_t len);
	 
#ifdef __cplusplus
}
#endif
#endif 

