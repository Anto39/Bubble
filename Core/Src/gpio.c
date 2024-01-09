/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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
#include "gpio.h"
/* USER CODE BEGIN 0 */
#include "MCUMap.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
#define GPIO_DEBOUNCE_MAX_DEFAULT			(3)     	///< Default debounced counter max value.
#define GPIO_DEBOUNCE_MAX_WATER_LEVEL 		(200)		///< Water level's debounced counter max value.
#define GPIO_DEBOUNCE_TOP_BOTTOM_FLOOD 		(200)		///< Water level's debounced counter max value.

///
/// \struct 	sGPIODDI
/// \brief 		Structure for debounced digital inputs (DDI).
typedef struct
{
  GPIO_TypeDef*	pGPIOx;			          ///< The digital input port/peripheral.
  uint32_t		  u32Pin;			          ///< The digital input pin number.
  bool			    bLevel;			          ///< The current debounced level.
  bool			    bActiveLow;		        ///< The Active Low (true) Active High (false) flag.
  uint32_t 		  u32Counter;		        ///< The debouncing counter.
  uint32_t 		  u32PressTimer;	      ///< The press counter (holding pressed).
  uint8_t       u8GPIODebounceMax;  	///< Debounced counter max value.

} GPIODDI_s;

GPIODDI_s	g_aDDI[GPIO_DDI_TOTAL_NB];	///< Array containing all the debounced digital inputs.

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level E */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_11|GPIO_PIN_13
                          |GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level C */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level B */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level D */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_5
		  	  	  	  	  |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO D pin Output Level LOW*/
   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_2 , GPIO_PIN_SET);

  /*Configure GPIO pin Output Level A */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level F */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE7 PE11 PE13
                           PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_11|GPIO_PIN_13
                          |GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE8 PE9 PE10
                           PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC6 PC7 PC8
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF2 PF3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 PB15
                           PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD10 PD12 PD15 PD2
                           PD3 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PF11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* USER CODE BEGIN 2 */
////////////////////////////////////////////////////////////////////////////////
/// \brief 		GPIOInitDDI
/// \details	Initialize all the debounced digital inputs.
/// \public
////////////////////////////////////////////////////////////////////////////////
void GPIOInitDDI(void)
{
  uint32_t u32Idx = 0;

  // Generic variables
  for (u32Idx = 0; u32Idx < GPIO_DDI_TOTAL_NB; ++u32Idx)
  {
    g_aDDI[u32Idx].u32Counter			= 0;
    g_aDDI[u32Idx].u32PressTimer		= 0;
    g_aDDI[u32Idx].u8GPIODebounceMax 	= GPIO_DEBOUNCE_MAX_DEFAULT;
  }

  // Specific variables.
  g_aDDI[GPIO_DDI_PRESENCE_ICE].pGPIOx                = MCUMAP_DI_PRESENCE_ICE_PERIPH;		//HIGH
  g_aDDI[GPIO_DDI_PRESENCE_ICE].u32Pin                = MCUMAP_DI_PRESENCE_ICE_PIN;
  g_aDDI[GPIO_DDI_PRESENCE_ICE].bLevel                = true;
  g_aDDI[GPIO_DDI_PRESENCE_ICE].bActiveLow            = true;
  g_aDDI[GPIO_DDI_PRESENCE_ICE].u8GPIODebounceMax 	  = GPIO_DEBOUNCE_MAX_WATER_LEVEL;

  g_aDDI[GPIO_DDI_LEVEL_ICE].pGPIOx                   = MCUMAP_DI_LEVEL_ICE_PERIPH;			//LOW
  g_aDDI[GPIO_DDI_LEVEL_ICE].u32Pin                   = MCUMAP_DI_LEVEL_ICE_PIN;
  g_aDDI[GPIO_DDI_LEVEL_ICE].bLevel                   = true;
  g_aDDI[GPIO_DDI_LEVEL_ICE].bActiveLow               = true;
  g_aDDI[GPIO_DDI_LEVEL_ICE].u8GPIODebounceMax		  = GPIO_DEBOUNCE_MAX_WATER_LEVEL;

  g_aDDI[GPIO_DDI_PRESENCE_FLOOD_BOTTOM].pGPIOx       = MCUMAP_DI_PRESENCE_FLOOD_BOTTOM_PERIPH;
  g_aDDI[GPIO_DDI_PRESENCE_FLOOD_BOTTOM].u32Pin       = MCUMAP_DI_PRESENCE_FLOOD_BOTTOM_PIN;
  g_aDDI[GPIO_DDI_PRESENCE_FLOOD_BOTTOM].bLevel       = true;
  g_aDDI[GPIO_DDI_PRESENCE_FLOOD_BOTTOM].bActiveLow   = true;
  g_aDDI[GPIO_DDI_PRESENCE_FLOOD_BOTTOM].u8GPIODebounceMax  = GPIO_DEBOUNCE_TOP_BOTTOM_FLOOD;

  g_aDDI[GPIO_DDI_PRESENCE_FLOOD_TOP].pGPIOx          = MCUMAP_DI_PRESENCE_FLOOD_TOP_PERIPH;
  g_aDDI[GPIO_DDI_PRESENCE_FLOOD_TOP].u32Pin          = MCUMAP_DI_PRESENCE_FLOOD_TOP_PIN;
  g_aDDI[GPIO_DDI_PRESENCE_FLOOD_TOP].bLevel          = true;
  g_aDDI[GPIO_DDI_PRESENCE_FLOOD_TOP].bActiveLow      = true;
  g_aDDI[GPIO_DDI_PRESENCE_FLOOD_TOP].u8GPIODebounceMax		= GPIO_DEBOUNCE_TOP_BOTTOM_FLOOD;

  g_aDDI[GPIO_DDI_WATER_LEVEL_LOW].pGPIOx             = MCUMAP_DI_WATER_LEVEL_LOW_PERIPH;
  g_aDDI[GPIO_DDI_WATER_LEVEL_LOW].u32Pin             = MCUMAP_DI_WATER_LEVEL_LOW_PIN;
  g_aDDI[GPIO_DDI_WATER_LEVEL_LOW].bLevel             = true;
  g_aDDI[GPIO_DDI_WATER_LEVEL_LOW].bActiveLow         = true;
  g_aDDI[GPIO_DDI_WATER_LEVEL_LOW].u8GPIODebounceMax  = GPIO_DEBOUNCE_MAX_WATER_LEVEL;

  g_aDDI[GPIO_DDI_WATER_LEVEL_HIGH].pGPIOx            = MCUMAP_DI_WATER_LEVEL_HIGH_PERIPH;
  g_aDDI[GPIO_DDI_WATER_LEVEL_HIGH].u32Pin            = MCUMAP_DI_WATER_LEVEL_HIGH_PIN;
  g_aDDI[GPIO_DDI_WATER_LEVEL_HIGH].bLevel            = true;
  g_aDDI[GPIO_DDI_WATER_LEVEL_HIGH].bActiveLow        = true;
  g_aDDI[GPIO_DDI_WATER_LEVEL_HIGH].u8GPIODebounceMax = GPIO_DEBOUNCE_MAX_WATER_LEVEL;

  g_aDDI[GPIO_DDI_PRESSURE_CO2].pGPIOx                = MCUMAP_DI_PRESSURE_CO2_PERIPH;
  g_aDDI[GPIO_DDI_PRESSURE_CO2].u32Pin                = MCUMAP_DI_PRESSURE_CO2_PIN;
  g_aDDI[GPIO_DDI_PRESSURE_CO2].bLevel                = true;
  g_aDDI[GPIO_DDI_PRESSURE_CO2].bActiveLow            = true;

  g_aDDI[GPIO_DDI_DOOR_OPEN].pGPIOx                   = MCUMAP_DI_DOOR_OPEN_PERIPH;
  g_aDDI[GPIO_DDI_DOOR_OPEN].u32Pin                   = MCUMAP_DI_DOOR_OPEN_PIN;
  g_aDDI[GPIO_DDI_DOOR_OPEN].bLevel                   = true;
  g_aDDI[GPIO_DDI_DOOR_OPEN].bActiveLow               = true;

  g_aDDI[GPIO_DDI_BUTTON_1].pGPIOx                    = MCUMAP_DI_BUTTON_1_PERIPH;
  g_aDDI[GPIO_DDI_BUTTON_1].u32Pin                    = MCUMAP_DI_BUTTON_1_PIN;
  g_aDDI[GPIO_DDI_BUTTON_1].bLevel                    = true;
  g_aDDI[GPIO_DDI_BUTTON_1].bActiveLow                = true;

  g_aDDI[GPIO_DDI_BUTTON_2].pGPIOx                    = MCUMAP_DI_BUTTON_2_PERIPH;
  g_aDDI[GPIO_DDI_BUTTON_2].u32Pin                    = MCUMAP_DI_BUTTON_2_PIN;
  g_aDDI[GPIO_DDI_BUTTON_2].bLevel                    = true;
  g_aDDI[GPIO_DDI_BUTTON_2].bActiveLow                = true;

  g_aDDI[GPIO_DDI_BUTTON_3].pGPIOx                    = MCUMAP_DI_BUTTON_3_PERIPH;
  g_aDDI[GPIO_DDI_BUTTON_3].u32Pin                    = MCUMAP_DI_BUTTON_3_PIN;
  g_aDDI[GPIO_DDI_BUTTON_3].bLevel                    = true;
  g_aDDI[GPIO_DDI_BUTTON_3].bActiveLow                = true;

  g_aDDI[GPIO_DDI_BUTTON_4].pGPIOx                    = MCUMAP_DI_BUTTON_4_PERIPH;
  g_aDDI[GPIO_DDI_BUTTON_4].u32Pin                    = MCUMAP_DI_BUTTON_4_PIN;
  g_aDDI[GPIO_DDI_BUTTON_4].bLevel                    = true;
  g_aDDI[GPIO_DDI_BUTTON_4].bActiveLow                = true;

  g_aDDI[GPIO_DDI_EMPTY_BOTTLE].pGPIOx                = MCUMAP_DI_EMPTY_BOTTLE_PERIPH;
  g_aDDI[GPIO_DDI_EMPTY_BOTTLE].u32Pin                = MCUMAP_DI_EMPTY_BOTTLE_PIN;
  g_aDDI[GPIO_DDI_EMPTY_BOTTLE].bLevel                = true;
  g_aDDI[GPIO_DDI_EMPTY_BOTTLE].bActiveLow            = true;

  g_aDDI[GPIO_DDI_CAP_BUT_INTERRUPT].pGPIOx           = MCUMAP_DI_CAP_BUT_INTERRUPT_PERIPH;
  g_aDDI[GPIO_DDI_CAP_BUT_INTERRUPT].u32Pin           = MCUMAP_DI_CAP_BUT_INTERRUPT_PIN;
  g_aDDI[GPIO_DDI_CAP_BUT_INTERRUPT].bLevel           = false;
  g_aDDI[GPIO_DDI_CAP_BUT_INTERRUPT].bActiveLow       = false;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		GPIODebounceDDI
/// \details	Debounce all the digital inputs.
/// \public
////////////////////////////////////////////////////////////////////////////////
void GPIODebounceDDI(void)
{
  uint32_t u32Idx			= 0;
  GPIO_PinState pinState 	= GPIO_PIN_RESET;

  for (u32Idx = 0; u32Idx < GPIO_DDI_TOTAL_NB; ++u32Idx)
  {
    pinState = HAL_GPIO_ReadPin(g_aDDI[u32Idx].pGPIOx, g_aDDI[u32Idx].u32Pin);

    if (pinState == GPIO_PIN_SET)
    {
      if(g_aDDI[u32Idx].u32Counter < g_aDDI[u32Idx].u8GPIODebounceMax)
      {
        ++g_aDDI[u32Idx].u32Counter;
      }
      else
      {
        g_aDDI[u32Idx].bLevel = true;
      }
    }
    else
    {
      if(g_aDDI[u32Idx].u32Counter > 0)
      {
        --g_aDDI[u32Idx].u32Counter;
      }
      else
      {
        g_aDDI[u32Idx].bLevel = false;
      }
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		GPIOGetDDI
/// \details	Retrieve the debounced digital input value.
/// \public
///
/// \param[in]	eDDI:		the debounced digital input.
/// \param[out]	pbPinState:	debounced state.
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool GPIOGetDDI(GPIODebouncedDigitalInput_e eDDI, bool* pbPinState)
{
  if (eDDI >= GPIO_DDI_TOTAL_NB) return false;
  if (pbPinState == NULL) return false;

  if (g_aDDI[eDDI].bActiveLow == false)
  {
    *pbPinState = g_aDDI[eDDI].bLevel;
  }
  else
  {
    *pbPinState = !g_aDDI[eDDI].bLevel;
  }

  return true;
}


/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
