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
#include <gpio.h>
#include "adc.h"

#include "dma.h"

/* USER CODE BEGIN 0 */
#include "MCUMap.h"
#include "Framework.h"

#define HAL_ADC_REFRESH_PERIOD_MS	10					///<
#define HAL_ADC_MAX_NB_SAMPLES		32					///< Maximum number of samples accumulated.

///
/// \struct 	HAL_ADC_AAI_s
/// \brief 		Structure for averaged analog inputs (AAI).
typedef struct
{
	uint32_t	u32Accumulator;		///< Accumulator of all the samples.
	uint32_t	u32NbSamples;		///< Number of samples accumulated.
} HAL_ADC_AAI_s;

uint32_t		g_u32RefreshTimestamp	= 0;
uint16_t 		g_u16ADCBuffer[HAL_ADC_AI_TOTAL_NB];	///< DMA storage structure for all samples.
HAL_ADC_AAI_s	g_aAAI[HAL_ADC_AI_TOTAL_NB];			///< Averaged analog inputs structure.

/* USER CODE END 0 */

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

/* ADC init function */
void MX_ADC_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_11;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_12;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_14;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_15;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
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
    PC1     ------> ADC_IN11
    PC2     ------> ADC_IN12
    PA0     ------> ADC_IN0
    PA1     ------> ADC_IN1
    PA2     ------> ADC_IN2
    PC4     ------> ADC_IN14
    PC5     ------> ADC_IN15
    PB0     ------> ADC_IN8
    PB1     ------> ADC_IN9
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC Init */
    hdma_adc.Instance = DMA1_Channel1;
    hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc.Init.Mode = DMA_CIRCULAR;
    hdma_adc.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_DMA1_REMAP(HAL_DMA1_CH1_ADC);

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
    PC1     ------> ADC_IN11
    PC2     ------> ADC_IN12
    PA0     ------> ADC_IN0
    PA1     ------> ADC_IN1
    PA2     ------> ADC_IN2
    PC4     ------> ADC_IN14
    PC5     ------> ADC_IN15
    PB0     ------> ADC_IN8
    PB1     ------> ADC_IN9
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
////////////////////////////////////////////////////////////////////////////////
/// \brief 		HAL_ADC_Task
/// \details	Averaging task of the acquired values.
/// \public
////////////////////////////////////////////////////////////////////////////////
bool HAL_ADC_Task(void)
{
	uint32_t u32Idx = 0;

	if (FrameworkTimeIsDue(g_u32RefreshTimestamp, HAL_ADC_REFRESH_PERIOD_MS) == true)
	{
		// Accumulate.
		for (u32Idx = 0; u32Idx < HAL_ADC_AI_TOTAL_NB; ++u32Idx)
		{
			g_aAAI[u32Idx].u32Accumulator += g_u16ADCBuffer[u32Idx];
			if (++g_aAAI[u32Idx].u32NbSamples >= HAL_ADC_MAX_NB_SAMPLES)
			{
				g_aAAI[u32Idx].u32Accumulator 	= g_aAAI[u32Idx].u32Accumulator / HAL_ADC_MAX_NB_SAMPLES;
				g_aAAI[u32Idx].u32NbSamples 	= 1;
			}
		}

		g_u32RefreshTimestamp = FrameworkGetTime();
	}

	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		HAL_ADC_Custom_Start
/// \details	Start the conversions.
/// \public
////////////////////////////////////////////////////////////////////////////////
void HAL_ADC_Custom_Start(void)
{
	// Calibrate before starting.
	HAL_ADCEx_Calibration_Start(&hadc);

	// Start the acquistion.
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)g_u16ADCBuffer, HAL_ADC_AI_TOTAL_NB);
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		HAL_ADC_GetValueRaw
/// \details	Retrieve the current raw value.
/// \public
///
/// \param[in]	eAI:		the analog input.
/// \param[out]	pu16Value:	current raw value.
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool HAL_ADC_GetValueRaw(HAL_ADC_AnalogInputs_e eAI, uint16_t* pu16Value)
{
	// Verify parameters.
	if (pu16Value == NULL) return false;
	if (eAI >= HAL_ADC_AI_TOTAL_NB) return false;

	*pu16Value = g_u16ADCBuffer[eAI];

	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		HAL_ADC_GetValueAveraged
/// \details	Retrieve the averaged analog input value.
/// \public
///
/// \param[in]	eAI:		the analog input.
/// \param[out]	pu16Value:	the averaged analog input.
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool HAL_ADC_GetValueAveraged(HAL_ADC_AnalogInputs_e eAI, uint16_t* pu16Value)
{
	// Verify parameters.
	if (pu16Value == NULL) return false;
	if (eAI >= HAL_ADC_AI_TOTAL_NB) return false;

	if (g_aAAI[eAI].u32NbSamples > 0)
	{
		*pu16Value = g_aAAI[eAI].u32Accumulator / g_aAAI[eAI].u32NbSamples;
	}
	else
	{
		*pu16Value = 0;
	}

	return true;
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
