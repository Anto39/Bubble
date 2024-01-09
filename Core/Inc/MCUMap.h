///
/// \file 		MCUMap.h
/// \brief 		[Header file]
///				
/// \author 	NOVO
///
#ifndef __MCUMAP_H_
#define __MCUMAP_H_


////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <stdbool.h>
#include <stdio.h>
#include "stm32f0xx_hal.h"


////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

// Choose your current board
//-------------------------------------
#define MCUMAP_SELECT_STM32F091XC
//#define MCUMAP_SELECT_NUCLEOF091RC_DEVBOARD
//-------------------------------------


//------------------------------------------------------------------------------
#ifdef MCUMAP_SELECT_STM32F091XC
//------------------------------------------------------------------------------

// DIGITAL INPUTS
#define MCUMAP_DI_WATER_LEVEL_LOW_PERIPH		GPIOA			// A
#define MCUMAP_DI_WATER_LEVEL_LOW_PIN			GPIO_PIN_15
#define MCUMAP_DI_PRESSURE_CO2_PERIPH			GPIOC			// C
#define MCUMAP_DI_PRESSURE_CO2_PIN				GPIO_PIN_0		// High Pressure CO2 Pump
#define MCUMAP_DI_BUTTON_3_PERIPH				GPIOC
#define MCUMAP_DI_BUTTON_3_PIN					GPIO_PIN_6
#define MCUMAP_DI_BUTTON_1_PERIPH				GPIOC
#define MCUMAP_DI_BUTTON_1_PIN					GPIO_PIN_7
#define MCUMAP_DI_BUTTON_2_PERIPH				GPIOC
#define MCUMAP_DI_BUTTON_2_PIN					GPIO_PIN_8
#define MCUMAP_DI_BUTTON_4_PERIPH				GPIOC
#define MCUMAP_DI_BUTTON_4_PIN					GPIO_PIN_9
#define MCUMAP_DI_WATER_LEVEL_HIGH_PERIPH		GPIOD			// D
#define MCUMAP_DI_WATER_LEVEL_HIGH_PIN			GPIO_PIN_4
#define MCUMAP_DI_CAP_BUT_INTERRUPT_PERIPH		GPIOD
#define MCUMAP_DI_CAP_BUT_INTERRUPT_PIN			GPIO_PIN_14
#define MCUMAP_DI_EMPTY_BOTTLE_PERIPH			GPIOE			// E
#define MCUMAP_DI_EMPTY_BOTTLE_PIN				GPIO_PIN_1
#define MCUMAP_DI_DOOR_OPEN_PERIPH				GPIOE
#define MCUMAP_DI_DOOR_OPEN_PIN					GPIO_PIN_4
#define MCUMAP_DI_PRESENCE_FLOOD_BOTTOM_PERIPH	GPIOE
#define MCUMAP_DI_PRESENCE_FLOOD_BOTTOM_PIN		GPIO_PIN_9
#define MCUMAP_DI_PRESENCE_FLOOD_TOP_PERIPH		GPIOE
#define MCUMAP_DI_PRESENCE_FLOOD_TOP_PIN		GPIO_PIN_10
#define MCUMAP_DI_PRESENCE_ICE_PERIPH			GPIOF			// F
#define MCUMAP_DI_PRESENCE_ICE_PIN				GPIO_PIN_2
#define MCUMAP_DI_LEVEL_ICE_PERIPH				GPIOF
#define MCUMAP_DI_LEVEL_ICE_PIN					GPIO_PIN_3

// DIGITAL OUTPUTS
#define MCUMAP_DO_TP_PERIPH						GPIOA			// A						//< WIFI_SCK  (TP86)
#define MCUMAP_DO_TP_PIN						GPIO_PIN_5  								//< WIFI_SCK  (TP86)
#define MCUMAP_DO_SLND_AQUEDUCT_PERIPH			GPIOA
#define MCUMAP_DO_SLND_AQUEDUCT_PIN				GPIO_PIN_11
#define MCUMAP_DO_SLND_WATER_COLD_PERIPH		GPIOA
#define MCUMAP_DO_SLND_WATER_COLD_PIN			GPIO_PIN_12
#define MCUMAP_DO_LED_NIGHT_PERIPH				GPIOB			// B
#define MCUMAP_DO_LED_NIGHT_PIN					GPIO_PIN_2
#define MCUMAP_DO_LED_BLUE_PERIPH				GPIOB
#define MCUMAP_DO_LED_BLUE_PIN					GPIO_PIN_8
#define MCUMAP_DO_DDP_PERIPH					GPIOB
#define MCUMAP_DO_DDP_PIN						GPIO_PIN_12
#define MCUMAP_DO_PUMP_RECIR_PERIPH				GPIOB
#define MCUMAP_DO_PUMP_RECIR_PIN				GPIO_PIN_13
#define MCUMAP_DO_PUMP_LOW_PERIPH				GPIOB
#define MCUMAP_DO_PUMP_LOW_PIN					GPIO_PIN_15
#define MCUMAP_DO_SLND_WATER_HOT_PERIPH			GPIOC			// C
#define MCUMAP_DO_SLND_WATER_HOT_PIN			GPIO_PIN_12
#define MCUMAP_DO_SLND_CO2_ON_PERIPH			GPIOC
#define MCUMAP_DO_SLND_CO2_ON_PIN				GPIO_PIN_13
#define MCUMAP_DO_VENTILATOR_PERIPH				GPIOD			// D
#define MCUMAP_DO_VENTILATOR_PIN				GPIO_PIN_10
#define MCUMAP_DO_SLND_WATER_TEMPERATE_PERIPH	GPIOD
#define MCUMAP_DO_SLND_WATER_TEMPERATE_PIN		GPIO_PIN_3
#define MCUMAP_DO_LED_GREEN_BI_PERIPH			GPIOD
#define MCUMAP_DO_LED_GREEN_BI_PIN				GPIO_PIN_5
#define MCUMAP_DO_LED_BLUE_BI_PERIPH			GPIOD
#define MCUMAP_DO_LED_BLUE_BI_PIN				GPIO_PIN_6
#define MCUMAP_DO_LED_RED_PERIPH				GPIOD
#define MCUMAP_DO_LED_RED_PIN					GPIO_PIN_7
#define MCUMAP_DO_COMPRESSOR_PERIPH				GPIOD
#define MCUMAP_DO_COMPRESSOR_PIN				GPIO_PIN_2
#define MCUMAP_DO_PUMP_HIGH_PERIPH				GPIOD
#define MCUMAP_DO_PUMP_HIGH_PIN					GPIO_PIN_12
#define MCUMAP_DO_3WAY_VALVE_PERIPH				GPIOE			// E
#define MCUMAP_DO_3WAY_VALVE_PIN				GPIO_PIN_3
#define MCUMAP_DO_LED_YELLOW_PERIPH				GPIOE
#define MCUMAP_DO_LED_YELLOW_PIN				GPIO_PIN_7
#define MCUMAP_DO_LED_HB_PERIPH					GPIOE
#define MCUMAP_DO_LED_HB_PIN					GPIO_PIN_11
#define MCUMAP_DO_WIFI_BLE_RESET_PERIPH			GPIOE
#define MCUMAP_DO_WIFI_BLE_RESET_PIN			GPIO_PIN_13
#define MCUMAP_DO_WIFI_BLE_ENABLE_PERIPH		GPIOE
#define MCUMAP_DO_WIFI_BLE_ENABLE_PIN			GPIO_PIN_14


#endif // MCUMAP_SELECT_STM32F091XC


//------------------------------------------------------------------------------
#ifdef MCUMAP_SELECT_NUCLEOF091RC_DEVBOARD
#warning "MCUMAP FOR DEVBOARD SELECTED"
//------------------------------------------------------------------------------

#define MCUMAP_DI_GENERIC_PORT						GPIOC
#define MCUMAP_DI_GENERIC_PIN						GPIO_PIN_8

#define MCUMAP_DO_GENERIC_PORT						GPIOC
#define MCUMAP_DO_GENERIC_PIN						GPIO_PIN_6

// DIGITAL INPUTS
#define MCUMAP_DI_PRESENCE_ICE_PERIPH				MCUMAP_DI_GENERIC_PORT
#define MCUMAP_DI_PRESENCE_ICE_PIN					MCUMAP_DI_GENERIC_PIN
#define MCUMAP_DI_LEVEL_ICE_PERIPH					MCUMAP_DI_GENERIC_PORT
#define MCUMAP_DI_LEVEL_ICE_PIN						MCUMAP_DI_GENERIC_PIN
#define MCUMAP_DI_PRESENCE_BOTTOM_FLOOD_PERIPH		MCUMAP_DI_GENERIC_PORT
#define MCUMAP_DI_PRESENCE_BOTTOM_FLOOD_PIN			MCUMAP_DI_GENERIC_PIN
#define MCUMAP_DI_PRESENCE_TOP_FLOOD_PERIPH			MCUMAP_DI_GENERIC_PORT
#define MCUMAP_DI_PRESENCE_TOP_FLOOD_PIN			MCUMAP_DI_GENERIC_PIN
#define MCUMAP_DI_WATER_LEVEL_LOW_PERIPH			MCUMAP_DI_GENERIC_PORT
#define MCUMAP_DI_WATER_LEVEL_LOW_PIN				MCUMAP_DI_GENERIC_PIN
#define MCUMAP_DI_WATER_LEVEL_HIGH_PERIPH			MCUMAP_DI_GENERIC_PORT
#define MCUMAP_DI_WATER_LEVEL_HIGH_PIN				MCUMAP_DI_GENERIC_PIN
#define MCUMAP_DI_WATER_LEVEL_CO2_HEART_PERIPH		MCUMAP_DI_GENERIC_PORT
#define MCUMAP_DI_WATER_LEVEL_CO2_HEART_PIN			MCUMAP_DI_GENERIC_PIN
#define MCUMAP_DI_DOOR_OPEN_PERIPH					MCUMAP_DI_GENERIC_PORT
#define MCUMAP_DI_DOOR_OPEN_PIN						MCUMAP_DI_GENERIC_PIN
#define MCUMAP_DI_BUTTON_1_PERIPH					MCUMAP_DI_GENERIC_PORT
#define MCUMAP_DI_BUTTON_1_PIN						MCUMAP_DI_GENERIC_PIN
#define MCUMAP_DI_BUTTON_2_PERIPH					MCUMAP_DI_GENERIC_PORT
#define MCUMAP_DI_BUTTON_2_PIN						MCUMAP_DI_GENERIC_PIN
#define MCUMAP_DI_BUTTON_3_PERIPH					MCUMAP_DI_GENERIC_PORT
#define MCUMAP_DI_BUTTON_3_PIN						MCUMAP_DI_GENERIC_PIN
#define MCUMAP_DI_BUTTON_4_PERIPH					MCUMAP_DI_GENERIC_PORT
#define MCUMAP_DI_BUTTON_4_PIN						MCUMAP_DI_GENERIC_PIN
#define MCUMAP_DI_EMPTY_BOTTLE_PERIPH				MCUMAP_DI_GENERIC_PORT
#define MCUMAP_DI_EMPTY_BOTTLE_PIN					MCUMAP_DI_GENERIC_PIN
#define MCUMAP_DI_CAP_BUT_INTERRUPT_PERIPH			MCUMAP_DI_GENERIC_PORT
#define MCUMAP_DI_CAP_BUT_INTERRUPT_PIN				MCUMAP_DI_GENERIC_PIN


// DIGITAL OUTPUTS
#define MCUMAP_DO_LED_HB_PERIPH          			GPIOA
#define MCUMAP_DO_LED_HB_PIN             			GPIO_PIN_5
#define MCUMAP_DO_LED_NIGHT_PERIPH       			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_LED_NIGHT_PIN          			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_LED_BLUE_PERIPH        			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_LED_BLUE_PIN           			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_LED_GREEN_PERIPH       			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_LED_GREEN_PIN          			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_LED_RED_PERIPH         			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_LED_RED_PIN            			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_LED_YELLOW_PERIPH      			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_LED_YELLOW_PIN         			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_COMPRESSOR_PERIPH      			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_COMPRESSOR_PIN         			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_PUMP_RECIR_PERIPH      			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_PUMP_RECIR_PIN         			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_HEATBAND_PERIPH        			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_HEATBAND_PIN           			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_PUMP_LOW_PERIPH        			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_PUMP_LOW_PIN           			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_PUMP_HIGH_PERIPH       			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_PUMP_HIGH_PIN          			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_VENTILATOR_PERIPH      			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_VENTILATOR_PIN         			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_VALVE_PERIPH           			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_VALVE_PIN              			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_WIFI_BLE_ENABLE_PERIPH 			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_WIFI_BLE_ENABLE_PIN    			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_WIFI_BLE_RESET_PERIPH  			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_WIFI_BLE_RESET_PIN     			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_SLND_WATER_COLD_PERIPH     		MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_SLND_WATER_COLD_PIN     			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_SLND_WATER_HOT_PERIPH      		MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_SLND_WATER_HOT_PIN      			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_SLND_WATER_TEMPERATE_PERIPH		MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_SLND_WATER_TEMPERATE_PIN			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_SLND_AQUEDUCT_PERIPH       		MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_SLND_AQUEDUCT_PIN       			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_SLND_CO2_IN_PERIPH      			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_SLND_CO2_IN_PIN         			MCUMAP_DO_GENERIC_PIN
#define MCUMAP_DO_TP_PERIPH              			MCUMAP_DO_GENERIC_PORT
#define MCUMAP_DO_TP_PIN                 			MCUMAP_DO_GENERIC_PIN

#endif // MCUMAP_SELECT_NUCLEOF091RC_DEVBOARD

#endif // __MCUMAP_H_