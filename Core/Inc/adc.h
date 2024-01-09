/**
  ******************************************************************************
  * File Name          : ADC.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __adc_H
#define __adc_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32f0xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc;

/* USER CODE BEGIN Private defines */

///
/// \enum 	HAL_ADC_AnalogInputs_e
/// \brief 	Supported analog inputs.
typedef enum
{
	HAL_ADC_AI_TEMP_WATER_HOT_ADJUST = 0,	///< ADC_IN0
	HAL_ADC_AI_TEMP_WATER_COLD_ADJUST,		///< ADC_IN1
	HAL_ADC_AI_MODEL_ID_CH,					///< ADC_IN2
	///< ADC_IN3
	///< ADC_IN4
	///< ADC_IN5
	///< ADC_IN6
	///< ADC_IN7
	HAL_ADC_AI_HW_VER_MINOR,				///< ADC_IN8
	HAL_ADC_AI_HW_VER_MAJOR,				///< ADC_IN9
	///< ADC_IN10
	HAL_ADC_AI_TEMP_WATER_COLD_1,			///< ADC_IN11
	HAL_ADC_AI_TEMP_WATER_COLD_2,			///< ADC_IN12
	///< ADC_IN13
	HAL_ADC_AI_INTERNAL_C02_CORE_TEMP, 		///< ADC_IN14
	HAL_ADC_AI_TEMP_WATER_HOT,				///< ADC_IN15
	///< ADC_IN16 (Internal Temp Sensor channel)
	///< ADC_IN17 (Internal Vrefint Channel)
	///< ADC_IN18 (Internal Vbat Channel)

	HAL_ADC_AI_TOTAL_NB
} HAL_ADC_AnalogInputs_e;

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_ADC_Init(void);

/* USER CODE BEGIN Prototypes */
bool HAL_ADC_Task(void);
void HAL_ADC_Custom_Start(void);
bool HAL_ADC_GetValueRaw(HAL_ADC_AnalogInputs_e eAI, uint16_t* pu16Value);
bool HAL_ADC_GetValueAveraged(HAL_ADC_AnalogInputs_e eAI, uint16_t* pu16Value);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ adc_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

