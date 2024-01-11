
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include <string>
#include <cstdlib>


/* USER CODE BEGIN Includes */
#ifdef __cplusplus
extern "C" {
#endif

#include "Framework.h"
#include "CommManager.h"
#include "LedDriver.h"
#include "UTLookUpTable.h"
//#include "BottomLoad.h"
#include "IceBoxx.h"
#include "HotDegree.h"
#include "Co2Pou.h"
#include "Co2Bottle.h"
#include "gaz.h"

#ifdef __cplusplus
}
#endif

#include "ProtocolI2C.h"
#include "EspSlave.h"
#include "wifiInfos.h"
using namespace ProtocolI2C;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define APPLICATION_HW_TEST			1		///< Activate specific hardware startup tests.
#define I2C_SLAVE_ADDR 				(0x55 <<1)

#if (APPLICATION_HW_TEST == 1)
#warning "*** Application HW Test is active ***"
#include "MCUMap.h"

uint32_t CarboLevel = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN);
uint32_t BucketLevel = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_2_PERIPH, MCUMAP_DI_BUTTON_2_PIN);
uint32_t DetectIce = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_3_PERIPH, MCUMAP_DI_BUTTON_3_PIN);
uint32_t Co2Low = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_4_PERIPH, MCUMAP_DI_BUTTON_4_PIN);
uint32_t Leaking = HAL_GPIO_ReadPin(MCUMAP_DI_DOOR_OPEN_PERIPH, MCUMAP_DI_DOOR_OPEN_PIN);
std::string selectedLanguage = "English";

uint8_t		SettingsButtonCurrent = 0;
uint8_t		SettingsButtonBefore = 0;
uint8_t		AlertButtonCurrent = 0;
uint8_t		AlertButtonBefore = 0;

uint8_t		WaterLevelCount = 0;
uint8_t		SettingsCount = 0;
uint8_t		SettingsClick = 0;
uint8_t		AlertClick = 0;
uint16_t	Timer = 0;
uint32_t	WaterSaved = 0;
uint32_t	DDPCounter = 80;
uint32_t	WaterCounter = 0;
uint16_t	u16ADCValueRaw1, u16ADCValueRaw2, u16ADCValueRaw3, u16ADCValueRaw4, u16ADCValueRaw5, u16ADCValueRaw6, u16ADCValueRaw7, u16ADCValueRaw8, u16ADCValueRaw9;
uint16_t	u16ADCValueAvg1, u16ADCValueAvg2, u16ADCValueAvg3, u16ADCValueAvg4, u16ADCValueAvg5, u16ADCValueAvg6, u16ADCValueAvg7, u16ADCValueAvg8, u16ADCValueAvg9;
bool		bState1, bState2, bState3, bState4, bState5, bState6, bState7, bState8, bState9, bState10, bState11, bState12, bState13;
#endif

eDDP_t DDP = DDP_deactivated;
ecompresseur_t compresseur = comp_deactivated;
etypePage_t typePage;
etypeEau_t typeEau = EAU_NOTPRESSED;
using namespace boutons_t;

EspSlave espSlave((0x55<<1), hi2c1);
volatile uint8_t flagInterrupt = 0;
uint8_t DispensingCounter = 0;
uint16_t refreshTemp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);




/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void ApplicationFatalError(void);

#if (APPLICATION_HW_TEST == 1)
void ApplicationHWTest(void);
#endif


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
////////////////////////////////////////////////////////////////////////////////
/// \brief		ApplicationFatalError
/// \details	Function managing the fatal error mode of the application.
///	\warning	This function is blocking and should never return.
///
/// \private
/// \param		None.
/// \return		void.
////////////////////////////////////////////////////////////////////////////////
void ApplicationFatalError(void)
{
	// Infinite loop to prevent malfunctions.
	while (1)
	{
		FrameworkFatalErrorTask();
	}
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		ApplicationHWTest
/// \details	Function reading input data to validate the hardware.
///
/// \private
/// \param		None.
/// \return		void.
////////////////////////////////////////////////////////////////////////////////
/*
#if (APPLICATION_HW_TEST == 1)
void ApplicationHWTest(void)
{
	// Read raw analogic values.
	HAL_ADC_GetValueRaw(HAL_ADC_AI_TEMP_WATER_HOT_ADJUST, &u16ADCValueRaw1);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_TEMP_WATER_COLD_ADJUST, &u16ADCValueRaw2);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_MODEL_ID_CH, &u16ADCValueRaw3);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_HW_VER_MAJOR, &u16ADCValueRaw4);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_HW_VER_MINOR, &u16ADCValueRaw5);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_TEMP_WATER_COLD_1, &u16ADCValueRaw6);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_TEMP_WATER_COLD_2, &u16ADCValueRaw7);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_INTERNAL_C02_CORE_TEMP, &u16ADCValueRaw8);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_TEMP_WATER_HOT, &u16ADCValueRaw9);

	// Read averaged analogic values.
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_HOT_ADJUST, &u16ADCValueAvg1);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_COLD_ADJUST, &u16ADCValueAvg2);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_MODEL_ID_CH, &u16ADCValueAvg3);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_HW_VER_MAJOR, &u16ADCValueAvg4);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_HW_VER_MINOR, &u16ADCValueAvg5);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_COLD_1, &u16ADCValueAvg6);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_COLD_2, &u16ADCValueAvg7);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_INTERNAL_C02_CORE_TEMP, &u16ADCValueAvg8);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_HOT, &u16ADCValueAvg9);

	// Read digital input.
	GPIOGetDDI(GPIO_DDI_PRESENCE_ICE, &bState1);
	//GPIOGetDDI(GPIO_DDI_PRESENCE_FLOOD_BOTTOM, &bState2);
	GPIOGetDDI(GPIO_DDI_PRESENCE_FLOOD_TOP, &bState3);
	GPIOGetDDI(GPIO_DDI_WATER_LEVEL_LOW, &bState4);
	GPIOGetDDI(GPIO_DDI_WATER_LEVEL_HIGH, &bState5);
	GPIOGetDDI(GPIO_DDI_PRESSURE_CO2, &bState6);
	GPIOGetDDI(GPIO_DDI_DOOR_OPEN, &bState7);
	GPIOGetDDI(GPIO_DDI_BUTTON_1, &bState8);
	GPIOGetDDI(GPIO_DDI_BUTTON_2, &bState9);
	GPIOGetDDI(GPIO_DDI_BUTTON_3, &bState10);
	GPIOGetDDI(GPIO_DDI_BUTTON_4, &bState11);
	GPIOGetDDI(GPIO_DDI_EMPTY_BOTTLE, &bState12);
	GPIOGetDDI(GPIO_DDI_CAP_BUT_INTERRUPT, &bState13);

	// Set digital outputs.

	HAL_GPIO_WritePin(MCUMAP_DO_LED_NIGHT_PERIPH, MCUMAP_DO_LED_NIGHT_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_LED_NIGHT_PERIPH, MCUMAP_DO_LED_NIGHT_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_LED_BLUE_PERIPH, MCUMAP_DO_LED_BLUE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_LED_BLUE_PERIPH, MCUMAP_DO_LED_BLUE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_LED_GREEN_BI_PERIPH, MCUMAP_DO_LED_GREEN_BI_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_LED_GREEN_BI_PERIPH, MCUMAP_DO_LED_GREEN_BI_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_LED_RED_PERIPH, MCUMAP_DO_LED_RED_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_LED_RED_PERIPH, MCUMAP_DO_LED_RED_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_LED_YELLOW_PERIPH, MCUMAP_DO_LED_YELLOW_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_LED_YELLOW_PERIPH, MCUMAP_DO_LED_YELLOW_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_PUMP_RECIR_PERIPH, MCUMAP_DO_PUMP_RECIR_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_PUMP_RECIR_PERIPH, MCUMAP_DO_PUMP_RECIR_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_DDP_PERIPH, MCUMAP_DO_DDP_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_DDP_PERIPH, MCUMAP_DO_DDP_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_PUMP_LOW_PERIPH, MCUMAP_DO_PUMP_LOW_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_PUMP_LOW_PERIPH, MCUMAP_DO_PUMP_LOW_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_PUMP_HIGH_PERIPH, MCUMAP_DO_PUMP_HIGH_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_PUMP_HIGH_PERIPH, MCUMAP_DO_PUMP_HIGH_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_VENTILATOR_PERIPH, MCUMAP_DO_VENTILATOR_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_VENTILATOR_PERIPH, MCUMAP_DO_VENTILATOR_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_3WAY_VALVE_PERIPH, MCUMAP_DO_3WAY_VALVE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_3WAY_VALVE_PERIPH, MCUMAP_DO_3WAY_VALVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_WIFI_BLE_ENABLE_PERIPH, MCUMAP_DO_WIFI_BLE_ENABLE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_WIFI_BLE_ENABLE_PERIPH, MCUMAP_DO_WIFI_BLE_ENABLE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_WIFI_BLE_RESET_PERIPH, MCUMAP_DO_WIFI_BLE_RESET_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_WIFI_BLE_RESET_PERIPH, MCUMAP_DO_WIFI_BLE_RESET_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_HOT_PERIPH, MCUMAP_DO_SLND_WATER_HOT_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_HOT_PERIPH, MCUMAP_DO_SLND_WATER_HOT_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_TEMPERATE_PERIPH, MCUMAP_DO_SLND_WATER_TEMPERATE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_TEMPERATE_PERIPH, MCUMAP_DO_SLND_WATER_TEMPERATE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_AQUEDUCT_PERIPH, MCUMAP_DO_SLND_AQUEDUCT_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_AQUEDUCT_PERIPH, MCUMAP_DO_SLND_AQUEDUCT_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_CO2_ON_PERIPH, MCUMAP_DO_SLND_CO2_ON_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_CO2_ON_PERIPH, MCUMAP_DO_SLND_CO2_ON_PIN, GPIO_PIN_RESET);

	// For the compressor, if it is re-activated too fast, the internal fuse will blow and reset itself after some time.
	HAL_GPIO_WritePin(MCUMAP_DO_COMPRESSOR_PERIPH, MCUMAP_DO_COMPRESSOR_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_COMPRESSOR_PERIPH, MCUMAP_DO_COMPRESSOR_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(MCUMAP_DO_LED_HB_PERIPH,MCUMAP_DO_LED_HB_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_LED_HB_PERIPH,MCUMAP_DO_LED_HB_PIN,GPIO_PIN_RESET);

}
#endif
*/
/* USER CODE END 0 */

////////////////////////////////////////////////////////////////////////////////
/// \brief		ApplicationFunctions
/// \details	Functions for the main.
///
/// \private
/// \param		None.
/// \return		void.
////////////////////////////////////////////////////////////////////////////////

//FUNCTION TO CONTROL led , Will be use to know if we finished the init()
void activateLed(){
	HAL_GPIO_WritePin(MCUMAP_DO_LED_HB_PERIPH,MCUMAP_DO_LED_HB_PIN,GPIO_PIN_SET);
}

void deactivateLed(){
	HAL_GPIO_WritePin(MCUMAP_DO_LED_HB_PERIPH,MCUMAP_DO_LED_HB_PIN,GPIO_PIN_RESET);
}

//FUNCTIONS TO CONTROL AC OUTPUT FOR EXTERNAL PERIPH. AC

/**
 * Activating the Inlet Water
 */
void activateAqueduct(){
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_AQUEDUCT_PERIPH, MCUMAP_DO_SLND_AQUEDUCT_PIN, GPIO_PIN_SET);
}
/**
 * Deactivating the Inlet Water
 */
void deactivateAqueduct(){
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_AQUEDUCT_PERIPH, MCUMAP_DO_SLND_AQUEDUCT_PIN, GPIO_PIN_RESET);
}

/**
 * Activating the Inlet Water
 */
void activateColdWater(){
	HAL_GPIO_WritePin(MCUMAP_DO_3WAY_VALVE_PERIPH, MCUMAP_DO_3WAY_VALVE_PIN, GPIO_PIN_SET);
}
/**
 * Deactivating the Inlet Water
 */
void deactivateColdWater(){
	HAL_GPIO_WritePin(MCUMAP_DO_3WAY_VALVE_PERIPH, MCUMAP_DO_3WAY_VALVE_PIN, GPIO_PIN_RESET);
}

/**
 * Activating the Inlet Water
 */
void activateNormalWater(){
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_CO2_ON_PERIPH, MCUMAP_DO_SLND_CO2_ON_PIN, GPIO_PIN_SET);
}
/**
 * Deactivating the Inlet Water
 */
void deactivateNormalWater(){
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_CO2_ON_PERIPH, MCUMAP_DO_SLND_CO2_ON_PIN, GPIO_PIN_RESET);
}

/**
 * Activating the Inlet Water
 */
void activateBubbleWater(){
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_HOT_PERIPH, MCUMAP_DO_SLND_WATER_HOT_PIN, GPIO_PIN_SET);
}
/**
 * Deactivating the Inlet Water
 */
void deactivateBubbleWater(){
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_HOT_PERIPH, MCUMAP_DO_SLND_WATER_HOT_PIN, GPIO_PIN_RESET);
}

/**
 * Activating the current on heatBand
 */
void activateDDP(){
	HAL_GPIO_WritePin(MCUMAP_DO_DDP_PERIPH, MCUMAP_DO_DDP_PIN, GPIO_PIN_RESET);
	DDP = DDP_activated;
}
/**
 * Deactivating the current on heatBand
 */
void deactivateDDP(){
	HAL_GPIO_WritePin(MCUMAP_DO_DDP_PERIPH, MCUMAP_DO_DDP_PIN, GPIO_PIN_SET);
	DDP = DDP_deactivated;
}
/**
 * Activating the current on Pump
 */
void activateCompressor(){
	HAL_GPIO_WritePin(MCUMAP_DO_COMPRESSOR_PERIPH, MCUMAP_DO_COMPRESSOR_PIN, GPIO_PIN_RESET);
	compresseur = comp_activated;
}
/**
 * Deactivating the current on Pump
 */
void deactivateCompressor(){
	HAL_GPIO_WritePin(MCUMAP_DO_COMPRESSOR_PERIPH, MCUMAP_DO_COMPRESSOR_PIN, GPIO_PIN_SET);
	compresseur = comp_deactivated;
}
/**
 * Activating the current on Ventilator
 */
void activateVentilator(){
	 HAL_GPIO_WritePin(MCUMAP_DO_VENTILATOR_PERIPH, MCUMAP_DO_VENTILATOR_PIN, GPIO_PIN_RESET);
}
/**
 * Deactivating the current on Ventilator
 */
void deactivateVentilator(){
	 HAL_GPIO_WritePin(MCUMAP_DO_VENTILATOR_PERIPH, MCUMAP_DO_VENTILATOR_PIN, GPIO_PIN_SET);
}

// DDP tasks
void TaskDDP()
{
	if(typeEau == EAU_NOTPRESSED && DDPCounter > 0  && DDP == DDP_deactivated && typePage == HomePage)
	{
		activateAqueduct();
		activateDDP();
	}

	if(typeEau == EAU_NOTPRESSED && DDPCounter < 1 && (DDP == DDP_activated || typePage != HomePage))
	{
		deactivateDDP();
		deactivateAqueduct();
		DDPCounter = 0;
	}
}

//
void TaskCompressor()
{
	if(DetectIce == 0  && compresseur == comp_deactivated)
	{
		activateCompressor();
	}

	if(DetectIce == 1  && compresseur == comp_activated)
	{
		deactivateCompressor();
	}
}

//
void deactivateAll()
{
	deactivateAqueduct();
	deactivateBubbleWater();
	deactivateColdWater();
	deactivateNormalWater();
	deactivateCompressor();
	deactivateDDP();
}

void SettingsTask()
{
	if(typePage != SettingsPage)
	{
		espSlave.putNewJpeg(("/BareSettings.jpg"), 0, 0, 800, 480);
		typePage = SettingsPage;
	}
	espSlave.putNewJpeg(("/" + selectedLanguage + "Button.jpg").c_str(), 655, 0, 145, 60);
	espSlave.putNewJpeg(("/" + selectedLanguage + "Wifi.jpg").c_str(), 240, 200, 319, 50);
	espSlave.putNewJpeg(("/" + selectedLanguage + "Text.jpg").c_str(), 0, 280, 226, 42);
}

void AlertTask()
{
	Co2Low = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_4_PERIPH, MCUMAP_DI_BUTTON_4_PIN);
	Leaking = HAL_GPIO_ReadPin(MCUMAP_DI_DOOR_OPEN_PERIPH, MCUMAP_DI_DOOR_OPEN_PIN);
	if(typePage == AlertPage)
	{
		if(Co2Low == true && Leaking == false)
		{
			espSlave.putNewJpeg(("/" + selectedLanguage + "LowCo2.jpg").c_str(), 0, 0, 800, 480);
		}
		if(Co2Low == false && Leaking == true)
		{
			espSlave.putNewJpeg(("/" + selectedLanguage + "Filters.jpg").c_str(), 0, 0, 800, 480);
		}
		if(Co2Low == true && Leaking == true)
		{
			espSlave.putNewJpeg(("/" + selectedLanguage + "Double.jpg").c_str(), 0, 0, 800, 480);
		}
	}

	if(typePage == HomePage)
	{
		if(Co2Low == true && Leaking == false)
		{
			espSlave.putNewJpeg(("/Alert1.jpg"), 70, 40, 20, 20);
		}
		if(Co2Low == false && Leaking == true)
		{
			espSlave.putNewJpeg(("/Alert1.jpg"), 70, 40, 20, 20);
		}
		if(Co2Low == true && Leaking == true)
		{
			espSlave.putNewJpeg(("/Alert2.jpg"), 70, 40, 20, 20);
		}
		if(Co2Low == false && Leaking == false)
		{
			espSlave.putNewJpeg(("/Alert0.jpg"), 70, 40, 20, 20);
		}
	}

}

void HomeTask()
{
	espSlave.putNewJpeg(("/" + selectedLanguage + "Screen.jpg").c_str(), 0, 0, 800, 480);
	typePage = HomePage;
	AlertTask();
}

//
void BucketLevelTask()
{

	while (BucketLevel == 1)
	{
		BucketLevel = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_2_PERIPH, MCUMAP_DI_BUTTON_2_PIN);
		if(typePage != BucketLevelPage){
			deactivateAll();
			espSlave.putNewJpeg(("/" + selectedLanguage + "BucketLevel.jpg").c_str(), 0, 0, 800, 480);
			espSlave.putNewJpeg(("/RedBucketLevel.jpg"), 589, 335, 52, 52);
			typePage = BucketLevelPage;
		}
	}

	if (BucketLevel == 0)
	{
		if(typePage == BucketLevelPage){
			refreshTemp = 0;
			espSlave.putNewJpeg(("/GreenBucketLevel.jpg"), 589, 335, 52, 52);
			while(refreshTemp < 3)
			{
				HAL_Delay(1000);
				refreshTemp++;
			}
			HomeTask();
		}
	}
}

// Screen button detection
void ButtonState()
{

	if(flagInterrupt == 0 && typeEau != EAU_NOTPRESSED && DispensingCounter < 2 && typePage == HomePage)
	{
		deactivateAqueduct();
		deactivateBubbleWater();
		deactivateColdWater();
		deactivateNormalWater();
		typeEau = EAU_NOTPRESSED;
	}

	if(DispensingCounter > 0)
			{
				DispensingCounter--;
			}

	//if touch is detected on an input on the screen, update the labels
	if(flagInterrupt)
	{
		flagInterrupt = 0;
		optional<vector<uint8_t>> buttonStates = espSlave.getButtonStates();

		//Debounce Settings button
		if(SettingsButtonCurrent != SettingsButtonBefore && refreshTemp < 2)
		{
			buttonStates.value()[3] = SettingsButtonBefore;
			SettingsButtonBefore = SettingsButtonCurrent;
		}

		//Debounce Alert button
		if(AlertButtonCurrent != AlertButtonBefore && refreshTemp < 2)
		{
			buttonStates.value()[4] = AlertButtonBefore;
			AlertButtonBefore = AlertButtonCurrent;
		}


		if(buttonStates.has_value())
		{
			for(uint8_t i = 0; i < buttonStates.value().size(); i++)
			{
				switch(i)
				{
				case BOUTON_EAU_FROIDE:
					if(buttonStates.value()[i] == 1 && typePage == HomePage && Co2Low == false)
					{
						if(buttonStates.value()[i] == 1 && typeEau != EAU_FROIDE)
						{
							activateAqueduct();
							activateColdWater();
							//espSlave.setLabel("LabelEau","Eau froide",300,100,3, 0x0000, 0x001F,1,1);
							typeEau = EAU_FROIDE;
						}
						WaterCounter ++;
						DDPCounter = DDPCounter + 3;
						DispensingCounter = 2;
					}
					break;

				case BOUTON_EAU_CARB:
					if(buttonStates.value()[i] == 1 && typePage == HomePage)
					{
						if(buttonStates.value()[i] == 1 && typeEau != EAU_CARBONATEE)
						{
							activateAqueduct();
							activateBubbleWater();
							//espSlave.setLabel("LabelEau","Eau carbonated",300,100,3, 0x0000, 0xF800,1,1);
							typeEau = EAU_CARBONATEE;
						}
						WaterCounter ++;
						deactivateDDP();
						DispensingCounter = 2;
					}
					break;

				case boutons_t::BOUTON_EAU_TEMP:
					if(buttonStates.value()[i] == 1 && typePage == HomePage)
					{
						if(buttonStates.value()[i] == 1 && typeEau != EAU_TEMPEREE)
						{
							activateAqueduct();
							activateNormalWater();
							//espSlave.setLabel("LabelEau","Eau temperee",300,100,3, 0x0000, 0xFFE0,1,1);
							typeEau = EAU_TEMPEREE;
						}
						WaterCounter ++;
						deactivateDDP();
						DispensingCounter = 2;
					}
					break;

				case boutons_t::BOUTON_Settings:

					//Settings button and language button actions
					if(buttonStates.value()[i] == 1)// && SettingsClick == 1)
					{
						SettingsClick++;
						deactivateAqueduct();
						deactivateBubbleWater();
						deactivateColdWater();
						deactivateNormalWater();
						if(typePage != SettingsPage && refreshTemp > 4)
						{
							SettingsTask();
							SettingsClick = 0;
							refreshTemp = 0;
						}

						if(selectedLanguage == "English" && refreshTemp > 4)
						{
							selectedLanguage = "French";
							SettingsTask();
							refreshTemp = 0;
						}

						if(selectedLanguage == "French" && refreshTemp > 4)
						{
							selectedLanguage = "English";
							SettingsTask();
							refreshTemp = 0;
						}
						return;
					}
					break;

				case BOUTON_Alert:

					if(buttonStates.value()[i] == 1 && typePage == HomePage && refreshTemp > 2)// && AlertClick == 1)
					{
						typePage = AlertPage;
						AlertTask();
						refreshTemp = 0;
					}
					//Alert and back button actions
					if(buttonStates.value()[i] == 1 && typePage != HomePage && refreshTemp > 2)// && AlertClick == 1)
					{
						AlertClick = 0;
						SettingsCount = 0;
						SettingsClick = 0;
						HomeTask();
						refreshTemp = 0;
					}
					break;
				}
			}
		}
		//else
	}
}


/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */



	FrameworkModelID_e modelID = FRAMEWORK_MODEL_ID_UNKNOWN;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //Allow time for esp slave init to complete
  HAL_Delay(5000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  //MX_IWDG_Init();
  MX_SPI1_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  HAL_I2C_MspInit(&hi2c1);
  CarboLevel = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN);
  BucketLevel = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_2_PERIPH, MCUMAP_DI_BUTTON_2_PIN);
  DetectIce = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_3_PERIPH, MCUMAP_DI_BUTTON_3_PIN);
  deactivateAll();
  activateCompressor();
  HomeTask();

  /* USER CODE BEGIN 2 */
  IceBoxxInit();

  //Configuration for the 4 AC external controls
  /*Configure GPIO pins : PD10 */
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = MCUMAP_DO_COMPRESSOR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MCUMAP_DO_COMPRESSOR_PERIPH, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MCUMAP_DO_DDP_PIN;
  HAL_GPIO_Init(MCUMAP_DO_DDP_PERIPH, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MCUMAP_DO_PUMP_RECIR_PIN;
  HAL_GPIO_Init(MCUMAP_DO_PUMP_RECIR_PERIPH, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MCUMAP_DO_VENTILATOR_PIN;
  HAL_GPIO_Init(MCUMAP_DO_VENTILATOR_PERIPH, &GPIO_InitStruct);

  // Allow time for electronics to startup correctly.
  HAL_Delay(100);
  	//exemple client
  //espSlave.putNewTitle("Thermonconcept");
  //espSlave.putNewString("filtre: ",20,200,3,0x0000,0xFFFF);
  //espSlave.putNewString("pression: ",20,100,3,0x0000,0xFFFF);
  //espSlave.putNewButton(50,350,75,75,0xF800,0);

	espSlave.putNewButton(318,290,164,84,0xFFFF,0xFFFF); //eau Temperee
	espSlave.putNewButton(570,290,164,84,0xFFFF,0xFFFF); //eau froide
	espSlave.putNewButton(65,290,164,84,0xFFFF,0xFFFF); //eau Bulles
	espSlave.putNewButton(718,0,82,84,0xFFE0,0xFFE0); //parametr or Language
	espSlave.putNewButton(0,0,82,84,0xFFE0,0xFFE0); //Alert or Back

  //espSlave.wifiConnect(WIFI_SSID,WIFI_PASSWORD);
  //espSlave.mqttConnect(MQTT_HOSTNAME, MQTT_USERNAME, MQTT_PASSWORD, 0);
  //espSlave.mqttSubscribe("esp/telemetry");
  //espSlave.setLabel("LabelEau","Not Pressed",300,100,3, 0x0000, 0xF850,1,1);
  //espSlave.setLabel("LabelTemp", "temp",400,300,3, 0x0000, 0x0000,1,1);
  //espSlave.setLabel("LabelFilter", "BON", 210,112,3,0x0000, 0x07E0,1,1);
  //espSlave.setLabel("LabelPressure", "OK", 150,212,3,0x0000, 0xFFE0,1,1);
  //espSlave.putNewLogo("/screen.jpg");

  activateLed();


	// Modules initialization.
	//if (FrameworkInit() == false) ApplicationFatalError();
	//if (CommManagerInit() == false) ApplicationFatalError();

	//if (FrameworkGetModelID(&modelID) == false) ApplicationFatalError();

	//if (Co2BottleInit() == false)	ApplicationFatalError();
	//if (BottomLoadInit() == false)	ApplicationFatalError();
	//if (IceBoxxInit() == false)		ApplicationFatalError();
	/*switch (modelID)
	{
		case FRAMEWORK_MODEL_ID_BOTTOMLOAD:	if (BottomLoadInit() == false)	ApplicationFatalError(); break;
		case FRAMEWORK_MODEL_ID_ICEBOXX:	if (IceBoxxInit() == false)		ApplicationFatalError(); break;
		case FRAMEWORK_MODEL_ID_HOTDEGREE:	if (HotDegreeInit() == false)	ApplicationFatalError(); break;
		case FRAMEWORK_MODEL_ID_CO2POU:		if (Co2PouInit() == false)		ApplicationFatalError(); break;
		case FRAMEWORK_MODEL_ID_CO2BOTTLE:	if (Co2BottleInit() == false)	ApplicationFatalError(); break;
		default: ApplicationFatalError();
	}*/

	// Unit tests.
/*#if (FRAMEWORK_UNIT_TEST == 1)
	if (FrameworkUnitTest() == false) ApplicationFatalError();
#endif
#if (BOTTOMLOAD_UNIT_TEST == 1)
	if (BottomLoadUnitTest() == false) ApplicationFatalError();
#endif
#if (LEDDRIVER_UNIT_TEST == 1)
	if (LedDriverUnitTest() == false) ApplicationFatalError();
#endif*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// Start the probe polarity toggle.
	//MX_PROBE_POL_PWM_START();
	//MX_BUZZER_PWM_START();

	// Forever loop.
	while (1)
	{
		//send mqtt data to thingsboard every 5 sec
		HAL_Delay(150);
		ButtonState();

		if (refreshTemp < 200000)
		{
			refreshTemp++;
		}

		if (typeEau == EAU_NOTPRESSED && typePage == HomePage)
		{

			if (DDPCounter > 0)
			{
				DDPCounter--;
			}

			//CarboLevel = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN);
			//BucketLevel = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_2_PERIPH, MCUMAP_DI_BUTTON_2_PIN);
			//DetectIce = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_3_PERIPH, MCUMAP_DI_BUTTON_3_PIN);


			if (typePage == HomePage && refreshTemp > 8)
			{
				AlertTask();
				WaterSaved = WaterCounter;
				//espSlave.putNewString(std::to_string(WaterSaved), 485,441,2,0x0000,0x0000);
				//espSlave.putNewString("185", 485,441,2,0x0000,0x0000);
				refreshTemp = 0;
			}

			//espSlave.setLabel("LabelPressure", std::to_string(WaterSaved), 485,441,3,0x0000, 0xFFFE,1,1);

			//BucketLevelTask();

			//TaskCompressor();

			TaskDDP();

		}


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

// EXTI Line9 External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_14) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
    	//setupEspFinished = 1;
    	flagInterrupt = 1;
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
