

#include <string.h>
#include "user_app.h"
#include "usart.h"

uint8_t Rs485Addr = 0;

#define USER_FLASH_SIZE			32 * 1024

uint32_t writeFlashData = 0x55555555;
uint32_t addr = 0x08007FF8;


/*
 *	FlashRead32:		��ȡ4�ֽ�(32λ)����
 *	����ReadAddr��		��������Flash�еĵ�ַ
 *	����ֵ��			���ض�ȡ��������		
 */
uint32_t FlashRead32(uint32_t ReadAddr )
{
	if(ReadAddr<FLASH_BASE||ReadAddr>FLASH_BASE+USER_FLASH_SIZE)
		return 0;
	
	return (uint32_t)(*(__IO uint32_t *)ReadAddr);
}

/*
 *	FlashRead16:		��ȡ2�ֽ�(16λ)����
 *	����ReadAddr��		��������Flash�еĵ�ַ
 *	����ֵ��			���ض�ȡ��������		
 */
uint16_t FlashRead16(uint32_t ReadAddr )
{
	if(ReadAddr<FLASH_BASE||ReadAddr>FLASH_BASE+USER_FLASH_SIZE)
		return 0;
	return (uint16_t)(*(__IO uint16_t *)ReadAddr);
}

/*
 *	FlashReadPage:		��ȡ1ҳ����
 *	����PageAddr��		ҳ��ַ
 *	����pBuffer��		�����ȡ�������ݵ�ָ��
 *	����ֵ��			1�ɹ� 0ʧ��	
 */
uint8_t FlashReadPage(uint32_t PageAddr, uint32_t *pBuffer)
{
	uint32_t Address = 0;
	if(PageAddr<FLASH_BASE||PageAddr+FLASH_PAGE_SIZE>FLASH_BASE+USER_FLASH_SIZE)
	{
		printf("line = %d\r\n",__LINE__);
		return 0;
	}
	if(PageAddr%FLASH_PAGE_SIZE!=0)
	{
		printf("line = %d\r\n",__LINE__);
		return 0;
	}
	Address = PageAddr;
	while (Address < PageAddr+FLASH_PAGE_SIZE)
	{
		*pBuffer = *(__IO uint32_t *)Address;
		Address = Address + 4;
		pBuffer++;
	}
	return 1;
}

/*
 *	FlashWritePage:		д1ҳ����
 *	����PageAddr��		ҳ��ַ
 *	����pBuffer��		����д��Flash�е�����ָ��
 *	����ֵ��			1�ɹ� 0ʧ��	
 */
uint8_t FlashWritePage( uint32_t PageAddr, uint32_t *pPageBuffer)
{
	uint32_t Address = 0, PAGEError =0;
	if(PageAddr<FLASH_BASE||PageAddr+FLASH_PAGE_SIZE>FLASH_BASE+USER_FLASH_SIZE)
		return 0;
	
	if(PageAddr%FLASH_PAGE_SIZE!=0)
		return 0;
	//����
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = PageAddr;
	EraseInitStruct.NbPages     = 1;
//	printf("PageAddr:%x\r\n",PageAddr);
	
	for(uint8_t i = 0; i < 10; i ++)
	{
		if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
		{
			printf("ERASE FLASH ERROR: %x i = %d\r\n",HAL_FLASH_GetError(), i);
			HAL_Delay(50);
			if(i == 9)
			{
				HAL_FLASH_Lock();

				return 0;
			}
		}
		else
			break;
	}
	
	Address=PageAddr;
	while (Address < PageAddr+FLASH_PAGE_SIZE)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *pPageBuffer) == HAL_OK)
		{
			Address = Address + 4;
			pPageBuffer++;
		}
		else
		{
			printf("Write Flash Error\r\n");
			HAL_FLASH_Lock();
			return 0;
		}
	}
	HAL_FLASH_Lock();
	return 1;
}

/*
 *	FlashWrite32:		д4�ֽ�(32λ)����,д�����ݲ��ܿ�ҳ(Ŀǰ�������ò�����ҳд����)
 *	����WriteAddr��		��������Flash�еĵ�ַ
 *	pBuffer:			����д��Flash�е�����ָ��
 *	NumToWrite			���ݳ���(С��ҳ��С/4)
 *	����ֵ��			1�ɹ� 0ʧ��			
 */
uint8_t FlashWrite32( uint32_t WriteAddr, uint32_t * pBuffer, uint16_t NumToWrite )
{
	uint32_t FLASH_BUF [ FLASH_PAGE_SIZE/4 ]={0};
	uint32_t PageAdd;	//WriteAddr��Ӧ��ҳ��ַ
	uint32_t OffSet;	//ҳ��ַ��ƫ��  PageAdd+OffSet=WriteAddr
	uint16_t i=0;
	//��ȡWriteAddr��Ӧ��ҳ��ַ
	if(WriteAddr<FLASH_BASE||WriteAddr+NumToWrite*4>FLASH_BASE+USER_FLASH_SIZE)
	{
		printf("line = %d\r\n",__LINE__);
		return 0;
	}
		
	if(WriteAddr%4!=0)
	{
		printf("line = %d\r\n",__LINE__);
		return 0;
	}	
	if(NumToWrite>FLASH_PAGE_SIZE/4)
	{
		printf("line = %d\r\n",__LINE__);
		return 0;
	}
	PageAdd=WriteAddr & 0xffffff00;		//L072��ҳ��СΪ128(0x80),����λ���㼴��
	OffSet=WriteAddr & 0x000000ff;
	if(OffSet/FLASH_PAGE_SIZE>0)//
	{
		PageAdd+=FLASH_PAGE_SIZE;
		OffSet-=FLASH_PAGE_SIZE;
	}
	if(PageAdd+FLASH_PAGE_SIZE < WriteAddr+NumToWrite*4)//���ݿ�ҳ
	{
		printf("line = %d\r\n",__LINE__);
		return 0;
	}
	if(FlashReadPage(PageAdd,FLASH_BUF)!=1)	//�ȶ�һҳ�����������
	{
		printf("line = %d\r\n",__LINE__);
		return 0;
	}
	//�޸ľ�����
	
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_BUF[OffSet/4+i]=*pBuffer;
		pBuffer++;
	}
	return FlashWritePage(PageAdd,FLASH_BUF);	//���������ݡ�д��������
}

/*
 *	FlashWrite16:		д2�ֽ�(16λ)����,д�����ݲ��ܿ�ҳ(Ŀǰ�������ò�����ҳд����)
 *	����WriteAddr��		��������Flash�еĵ�ַ
 *	pBuffer:			����д��Flash�е�����ָ��
 *	NumToWrite			���ݳ���(С��ҳ��С)
 *	����ֵ��			1�ɹ� 0ʧ��			
 */
uint8_t FlashWrite16( uint32_t WriteAddr, uint16_t * pBuffer, uint16_t NumToWrite )
{
	uint32_t FLASH_BUF [ FLASH_PAGE_SIZE/4 ];
	uint32_t PageAdd;	//WriteAddr��Ӧ��ҳ��ַ
	uint32_t OffSet;	//ҳ��ַ��ƫ��  PageAdd+OffSet=WriteAddr
//	uint16_t i;
	
	//��ȡWriteAddr��Ӧ��ҳ��ַ
	if(WriteAddr<FLASH_BASE||WriteAddr+NumToWrite*2>USER_FLASH_SIZE+FLASH_BASE)
		return 0;
		
	if(WriteAddr%2!=0)
		return 0;
		
	if(NumToWrite>FLASH_PAGE_SIZE/4*2)
		return 0;
	
	PageAdd=WriteAddr & 0xffffff00;
	OffSet=WriteAddr & 0x000000ff;
	if(OffSet/FLASH_PAGE_SIZE>0)//
	{
		PageAdd+=FLASH_PAGE_SIZE;
		OffSet-=FLASH_PAGE_SIZE;
	}
	if(PageAdd+FLASH_PAGE_SIZE < WriteAddr+NumToWrite*2)//���ݿ�ҳ
		return 0;
	if(FlashReadPage(PageAdd,FLASH_BUF)!=1)	//�ȶ�һҳ�����������
		return 0;
	//�޸ľ�����
	memcpy(FLASH_BUF+OffSet/4,pBuffer,NumToWrite*2);
//	for(i=0;i<NumToWrite/2;i++)
//	{
//		uint32_t temp1,temp2;
//		//��16λ����ƴ��Ϊ32λ
//		temp1=*pBuffer;
//		temp2=*(pBuffer+1);
//		FLASH_BUF[OffSet/4+i]=(temp2<<16|temp1);
//		pBuffer+=2;
//	}
//	for(uint8_t j=0;j<NumToWrite%2;j++)
//	{
//		FLASH_BUF[OffSet/4+i]=FLASH_BUF[OffSet/4+i] & 0xffff0000;	//��պ�2�ֽ�
//		FLASH_BUF[OffSet/4+i]=FLASH_BUF[OffSet/4+i] | *pBuffer;
//	}
	if(FlashWritePage(PageAdd,FLASH_BUF)!=1)	//���������ݡ�д��������
		return 0;
	return 1;
}

//FLASHд�����ݲ���
void writeFlashTest(void)
{
    //1������FLASH
  HAL_FLASH_Unlock();

	//2������FLASH
	//��ʼ��FLASH_EraseInitTypeDef
	FLASH_EraseInitTypeDef f;
	f.TypeErase = FLASH_TYPEERASE_PAGES;
	f.PageAddress = addr;
	f.NbPages = 1;
	//����PageError
	uint32_t PageError = 0;
	//���ò�������
	HAL_FLASHEx_Erase(&f, &PageError);

	//3����FLASH��д
	HAL_FLASH_Program(TYPEPROGRAM_WORD, addr, writeFlashData);

    //4����סFLASH
  HAL_FLASH_Lock();
}

//FLASH��ȡ���ݲ���
void printFlashTest(void)
{
  uint32_t temp = *(__IO uint32_t*)(addr);

  printf("addr:0x%x, data:0x%x\r\n", addr, temp);
}


/*
*RS485�� ��ʼ��485
*������  ��
*����ֵ����
*/
void Rs485Init(void)
{
	RS485_TO_TX(  );
	Rs485Addr = 0xF9;
	
	if(FlashRead16(POSITIVE_ADDR)==0||FlashRead16(POSITIVE_ADDR)==0xffff)
	{
		Para.Positive = true;
	}
	else
	{
		Para.Positive = FlashRead16(POSITIVE_ADDR);
	}
	
	if(FlashRead16(COMPENSATE_ADDR)==0||FlashRead16(COMPENSATE_ADDR)==0xffff)
	{
		Para.Data = 0;
	}
	else
	{
		Para.Data = FlashRead16(COMPENSATE_ADDR);
	}
	
	for(uint8_t i = 0; i < 5; ++i)
	{
		printf("Para.Positive = %d Para.Data = %d\r\n ",Para.Positive,Para.Data);
	
		HAL_Delay(200);
	}
	
	RS485_TO_RX(  );
}

/*
*Rs485RevceHandle�� Rs485���ݴ���
*������  			 			����������
*����ֵ��			 			��
*/
void Rs485RevceHandle(int16_t *SenSorBuf)
{
	uint8_t temp[15] = {0};
	uint8_t len = 0;
	
	RS485_TO_TX(  );

	if(CalcCRC16(UART_RX_UART2.USART_RX_BUF,UART_RX_UART2.USART_RX_Len) == 0)
	{		
		///�ж�Rs485������
		switch(UART_RX_UART2.USART_RX_BUF[1])
		{
			case 0x03:  ///1���㲥���� fe030400000000F53C
				if(UART_RX_UART2.USART_RX_BUF[0] == 0xFE && UART_RX_UART2.USART_RX_Len == 0x09)
				{
					///fe030400000000F53C
					temp[len++] = 0xfe;
					temp[len++] = 0x03;
					temp[len++] = 0x04;
				
					temp[len++] = Rs485Addr;
					temp[len++] = 0x00;
					temp[len++] = 0x00;
					temp[len++] = 0x00;
				
					CalcCRC16(temp,len);
					
					HAL_Delay(200);
					HAL_UART_Transmit(&huart2, temp, (len+2),0xFFFF);	
				}
				else if(UART_RX_UART2.USART_RX_BUF[0] == Rs485Addr) ///2����ȡ����ָ��¶�*10��EC*1000
				{
					memset(SensorData, 0, 2);
					AdcHandle(  );
					
					///rev: f90300000002D1B3
					temp[len++] = Rs485Addr;
					temp[len++] = 0x03;
					temp[len++] = 0x04;
				
					temp[len++] = (SensorData[0]>>8 & 0xff); ///�¶�
					temp[len++] = (SensorData[0]>>0 & 0xff);
					temp[len++] = (SensorData[1]>>8 & 0xff);  ///ʪ��
					temp[len++] = (SensorData[1]>>0 & 0xff);
				
					CalcCRC16(temp,len);
											
					HAL_UART_Transmit(&huart2, temp, (len+2),0xFFFF);							
				}
			
				break;
			
			
			case 0xA5: ///3��ֱ�������ѹ
			
					///rev: 0xA5	0x03	0x00	0x00	0x00	0x02	0xc4	0x38
			
				break;
			
			case 0x06: ///�޸ĵ�ַ
				if(UART_RX_UART2.USART_RX_BUF[0] == Rs485Addr && UART_RX_UART2.USART_RX_Len == 0x09)	
				{
					///Rs485Addr	0x06	0x04	0x00	0x00	0x00	new	0x7b	0xa7
					
					memcpy(temp, UART_RX_UART2.USART_RX_BUF, UART_RX_UART2.USART_RX_Len);
					
					HAL_UART_Transmit(&huart2, temp, UART_RX_UART2.USART_RX_Len,0xFFFF);	
					
					///Rs485Addr = new
					
					Rs485Addr = temp[6];
				}			
				break;
				
			case 0x07: ///f90700010028F06C
				if(UART_RX_UART2.USART_RX_BUF[0] == Rs485Addr && UART_RX_UART2.USART_RX_Len == 0x08)
				{
					if(UART_RX_UART2.USART_RX_BUF[3])
					{
					  Para.Positive = true;
					}
					else
						Para.Positive = false;
					
					Para.Data  = UART_RX_UART2.USART_RX_BUF[4] << 8;
				  Para.Data |= UART_RX_UART2.USART_RX_BUF[5];
									
					FlashWrite16(POSITIVE_ADDR, (uint16_t *)Para.Positive, 1);					
					FlashWrite16(COMPENSATE_ADDR, &Para.Data, 1);
					
				  temp[len++] = Rs485Addr;
					temp[len++] = 0x07;
					temp[len++] = 0x04;			
					temp[len++] = 0x00;
					temp[len++] = 0x01;
					
					temp[len++] = 0x00;
					temp[len++] = 0x01;
				
					CalcCRC16(temp,len);
				
					RS485_TO_TX(  );
					
					HAL_UART_Transmit(&huart2, temp, (len+2),0xFFFF);
					
					HAL_NVIC_SystemReset(		);
				}
			break;
				
			default:
				break;		
		}
	}	
	memset(UART_RX_UART2.USART_RX_BUF, 0, UART_RX_UART2.USART_RX_Len);
	UART_RX_UART2.USART_RX_Len = 0;	
}

/*
 *	CalcCRC16:	����CRC16У��ֵ
 *	data:		����ָ��
 *	len:		���ݳ���
 *	����ֵ��	16λ��CRCУ��ֵ
 */
uint16_t CalcCRC16(uint8_t *data, uint8_t len)
{
	uint16_t result = 0xffff;
	uint8_t i, j;

	for (i=0; i<len; i++)
	{
		result ^= data[i];
		for (j=0; j<8; j++)
		{
			if ( result&0x01 )
			{
					result >>= 1;
					result ^= 0xa001;
			}
			else
			{
					result >>= 1;
			}
		}
	}
	GET_CRC(&(data[len]), result);
	
	return result;
}

