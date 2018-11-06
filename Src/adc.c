/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */

#include <string.h>
#include <stdbool.h>
#include "usart.h"

adc_t	 Adc 	= {0, {0}, 0, 0, false};

para_t Para = {0, true};


/* USER CODE END 0 */

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

/* ADC init function */
void MX_ADC_Init(void)
{
	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
	*/
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_39CYCLES_5; ///����EC�ɼ�Ϊ4KHZ��39.5+12.5������Ϊ�������
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC GPIO Configuration    
    PA1     ------> ADC_IN1
    PA4     ------> ADC_IN4
    PA5     ------> ADC_IN5
    PA6     ------> ADC_IN6 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6; 
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC Init */
    hdma_adc.Instance = DMA1_Channel1;
    hdma_adc.Init.Request = DMA_REQUEST_0;
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc.Init.Mode = DMA_NORMAL;
    hdma_adc.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC GPIO Configuration    
    PA1     ------> ADC_IN1
    PA4     ------> ADC_IN4
    PA5     ------> ADC_IN5
    PA6     ------> ADC_IN6 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/*
*��ѹ���㹫ʽ��Vchannel = VDDA * ADC_DATA/FULL_SCALE
*VDDA���ڲ���׼��ѹ��: = 3*VREFINT_CAL/VREFINT_DATA (VREFINT_CAL:У׼ֵ��VREFINT_DATA:ad17)
*���ѹ���㹫ʽΪ: Vchannel = (3*VREFINT_CAL*ADC_DATA)/(VREFINT_DATA*FULL_SCALE)
*/
uint16_t GetAdcData(uint32_t Channel, uint8_t Counter)
{
	ADC_ChannelConfTypeDef sConfig;
	
	Adc.Data = 0;
	memset(Adc.Buf, 0, Counter);	
		
	/**Configure for the selected ADC regular channel to be converted. 
   */
  sConfig.Channel = Channel;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
		printf("---error---\r\n");
    _Error_Handler(__FILE__, __LINE__);
  }
	
	/**********************ADCУ׼**************************/
	if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
  {
		printf("---error---\r\n");
    Error_Handler();
  }
		
	HAL_ADC_Start_DMA(&hadc,Adc.Buf,Counter);
		
	for(uint16_t t = 0; t < 800; ++t) //56us = 200����800 = 224us����Ҫ�㹻ʱ�䣬����ȡ��ʧ��
	__NOP();

	while(!Adc.Complete);
	HAL_ADC_Stop_DMA(&hadc);
	
	Adc.Complete = false;

	/**ɾ���ڵ�
   */
	sConfig.Channel = Channel;
  sConfig.Rank = ADC_RANK_NONE;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	Bublesort(Adc.Buf,Counter);
	
	uint8_t index = Counter/2;
	
	Adc.Data = Adc.Buf[index];
			
	return Adc.Data;
}

/*
*��ѹ���㹫ʽ��Vchannel = VDDA * ADC_DATA/FULL_SCALE
*VDDA���ڲ���׼��ѹ��: = 3*VREFINT_CAL/VREFINT_DATA (VREFINT_CAL:У׼ֵ��VREFINT_DATA:ad17)
*���ѹ���㹫ʽΪ: Vchannel = (3*VREFINT_CAL*ADC_DATA)/(VREFINT_DATA*FULL_SCALE)
*/
/************************��ֵ������ȡͨ����ѹ****************************/
uint16_t GetAdcVref(uint32_t Channel, uint8_t Counter)
{
	ADC_ChannelConfTypeDef sConfig;
	
	Adc.Data = 0;

	/**Configure for the selected ADC regular channel to be converted. 
   */
  sConfig.Channel = Channel;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	/**********************ADCУ׼**************************/
	if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
	
	memset(Adc.Buf, 0, Counter);	
	
	HAL_ADC_Start_DMA(&hadc,Adc.Buf,Counter);
	
	HAL_Delay(1);

	while(!Adc.Complete);
	HAL_ADC_Stop_DMA(&hadc);
	
	Adc.Complete = false;

	/**ɾ���ڵ�
   */
	sConfig.Channel = Channel;
  sConfig.Rank = ADC_RANK_NONE;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	for(uint8_t index = 1; index < Counter-1; ++index)
	{
		Adc.Data += Adc.Buf[index];
	}
		
	return (Adc.Data/9);
}

int16_t SensorData[2] = {0};

void AdcHandle(void)
{
	float    Adcdata = 0;
	uint32_t Vchannel = 0;

	float 	 Tempure = 0;
	
	float 	 temp = 0;
	
	uint16_t adc_data[4] = {0};
	float 	 SensorBuf[4] = {0};	
	
	uint32_t MuilpData = 0;
	
	uint8_t counter = 0;
	
	///�����˲��㷨
	for(counter = 0; counter < 20; ++counter) ///��ֵ
	{
		MuilpData += GetAdcData(ADC_CHANNEL_VREFINT, BUFLEN); //(��ֵ��)�ڲ�����
	}
	adc_data[0] = MuilpData/counter;
	MuilpData = 0;
	
	for(counter = 0; counter < 20; ++counter)
	{
		MuilpData += GetAdcData(ADC_CHANNEL_1, BUFLEN);  ///�ⲿ��׼
	}
	adc_data[1] = MuilpData/counter;
	MuilpData = 0;
	
	for(counter = 0; counter < 20; ++counter)
	{
		MuilpData += GetAdcData(ADC_CHANNEL_5, BUFLEN); ///ʪ��		
	}
	adc_data[2] = MuilpData/counter;
	MuilpData = 0;
	
	for(counter = 0; counter < 20; ++counter)
	{
		MuilpData += GetAdcData(ADC_CHANNEL_6, BUFLEN); ///�¶�
	}
	adc_data[3] = MuilpData/counter;
	MuilpData = 0;
							
	Vchannel = VFULL * adc_data[0] * VREFEXT_CAL_VREF;
	
	Adcdata = (VREFINT_CAL_VREF * adc_data[1]);					
	
	Adcdata = (float)(Vchannel/Adcdata);
				
	temp = VREFINT_CAL_VREF * Adcdata;
		
//			printf("�ڲ����� = %d, �ⲿ��׼ = %d �¶� = %d ʪ�� = %d %.2f\r\n",adc_data[0], adc_data[1],adc_data[2],adc_data[3], temp);
	
	for(uint8_t i = 0, j = 1; i < 4; ++i, ++j)
	{
		SensorBuf[i] = (float)(temp*adc_data[j])/(adc_data[0] * VFULL);	
	}				
	
//			printf("�ⲿ��׼ = %.4f �ڲ���׼ = %.4f\r\n", SensorBuf[0],(float)(temp*adc_data[1])/(adc_data[1] * VFULL));
	
	///�¶�
	Tempure = (float)(SensorBuf[2] - VREFEXT_CAL_VREF) * 100;
//	printf("�¶�00 = %.2f��C %.1f\r\n ",Tempure,(float)Para.Data/100);
	///�¶Ȳ���		
	if(Para.Positive)
	{
		Tempure += (float)Para.Data/100;
	}
	else
	{
		Tempure -= (float)Para.Data/100;
	}		
//	printf("�¶�11 = %.2f��C\r\n ",Tempure);	
	SensorData[0] = (int16_t)((Tempure * 10) + 0.5);
	
 ///ʪ��
//			printf("�¶� = %.4f ʪ�� = %.4f\r\n", SensorBuf[1], SensorBuf[2]);
												
	SensorData[1] = SensorBuf[1] * 1000;
			
	printf("\r\n�¶� = %d��C, ʪ�� = %d \r\n", SensorData[0], SensorData[1]);		
}

#if 0

/*
*��ѹ���㹫ʽ��Vchannel = VDDA * ADC_DATA/FULL_SCALE
*VDDA���ڲ���׼��ѹ��: = 3*VREFINT_CAL/VREFINT_DATA (VREFINT_CAL:У׼ֵ��VREFINT_DATA:ad17)
*���ѹ���㹫ʽΪ: Vchannel = (3*VREFINT_CAL*ADC_DATA)/(VREFINT_DATA*FULL_SCALE)
**************************************************�����ɼ��������ڱ���<4khz�� ��������250us����ɣ�����ɼ����ݴ���***************************************
*/
void GetEcAdc(uint32_t Channel, uint8_t Counter)
{		
	ADC_ChannelConfTypeDef sConfig;
	
	/**Configure for the selected ADC regular channel to be converted. 
   */
  sConfig.Channel = Channel;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;

  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	/**********************ADCУ׼**************************/
	if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
	
	memset(Adc.Buf, 0, Counter * 2);	
	
	for(uint16_t t = 0; t < 357; ++t) ///14us = 50
	__NOP();
	
	HAL_ADC_Start_DMA(&hadc,Adc.Buf,Counter * 2); //88us
	
	for(uint16_t t = 0; t < 400; ++t) ///56us = 200 400
	__NOP();

	while(!Adc.Complete);
	HAL_ADC_Stop_DMA(&hadc);

	Adc.Complete = false;

	/**ɾ���ڵ�
   */
	sConfig.Channel = Channel;
  sConfig.Rank = ADC_RANK_NONE;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_RANK_NONE;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/*
*EcHandle ��EC ���ݴ���
*����			���ɼ����ڣ�һ������~=4ms
*����ֵ		����
*/
void EcHandle(uint8_t CollectNum)
{
	uint8_t counter = 0;
	memset(Adc.Buf, 0, BUFLEN*2);
					
	Adc.CollectEcEnable = true;
	Adc.EC_HData = 0;
	Adc.EC_LData = 0;

	for( ; counter < CollectNum; ++counter) ///ѭ���ɼ�10����������
	{
		Adc.PwmEc = false;
		HAL_Delay(3); ///timer = 2.5ms

		if(Adc.PwmEc)
		{
			Adc.PwmEc = false;
			for(uint8_t i = 0; i < BUFLEN*2; ++i)
			{
				if(i%2==0)
				Adc.EC_HData += Adc.Buf[i];	
				
				if(i%2==1)
				Adc.EC_LData += Adc.Buf[i];
			}		
			memset(Adc.Buf, 0, BUFLEN*2);
		}		
	}
	
	Adc.EC_HData /= (BUFLEN * counter);
	Adc.EC_LData /= (BUFLEN * counter);
		
//	printf("\r\nEH = %d EL = %d\r\n",Adc.EC_HData,Adc.EC_LData);	
	Adc.CollectEcEnable = false;
}



/*
*GetEcHAdc��EC H�˲ɼ�����
*����			��ADCͨ�����ɼ�����
*����ֵ		����
*/
void GetEcHAdc(uint32_t Channel, uint8_t Counter)
{		
	ADC_ChannelConfTypeDef sConfig;
	
	/**Configure for the selected ADC regular channel to be converted. 
   */
  sConfig.Channel = Channel;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

	/**********************ADCУ׼**************************/
	if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
	
	memset(Adc.HBuf, 0, Counter);	
	
	for(uint16_t t = 0; t < 40; ++t) ///56us
	__nop();
	
	HAL_ADC_Start_DMA(&hadc,Adc.HBuf,Counter); //88us
	
	for(uint16_t t = 0; t < 100; ++t) ///56us
	__nop();

	while(!AdcChance);
	HAL_ADC_Stop_DMA(&hadc);

	AdcChance = false;

	/**ɾ���ڵ�
   */
	sConfig.Channel = Channel;
  sConfig.Rank = ADC_RANK_NONE;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/*
*GetEcLAdc��EC L�˲ɼ�����
*����			��ADCͨ�����ɼ�����
*����ֵ		����
*/
void GetEcLAdc(uint32_t Channel, uint8_t Counter)
{		
	ADC_ChannelConfTypeDef sConfig;
	
	/**Configure for the selected ADC regular channel to be converted. 
   */
  sConfig.Channel = Channel;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

	/**********************ADCУ׼**************************/
	if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
	
	memset(Adc.LBuf, 0, Counter);	
	
	HAL_ADC_Start_DMA(&hadc,Adc.LBuf,Counter); //88us
	
	for(uint16_t t = 0; t < 100; ++t) ///56us = 200
	__nop();

	while(!AdcChance);
	HAL_ADC_Stop_DMA(&hadc);

	AdcChance = false;

	/**ɾ���ڵ�
   */
	sConfig.Channel = Channel;
  sConfig.Rank = ADC_RANK_NONE;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

#endif

/*
*HAL_ADC_ConvHalfCpltCallback��ADC�ص�����
*����												 ����ʵ�ʲ���
*����ֵ											 ����
*/
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	Adc.Complete = true;
}

/*
*Bublesort��ð�ݷ�
*����			��BUF��LEN
*����			����
*/
void Bublesort(uint32_t *a,uint8_t n)
{
	int i,j,k;
	
	for(j=0;j<n-1;j++)   /* ð�ݷ�����n�� */
	{
		for(i=0;i<n-j-1;i++)  /* ֵ�Ƚ�С��Ԫ�س���ȥ��ֻ��ʣ�µ�Ԫ����Сֵ�ٳ���ȥ */
		{
			 if(a[i]<a[i+1])  /* ��Сֵ������ */
			 {
					k=a[i];
					a[i]=a[i+1];
					a[i+1]=k;
			 }
		}
	}
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
