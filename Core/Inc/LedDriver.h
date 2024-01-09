///
/// \file 		LedDriver.h
/// \brief 		[Header file]
///
/// \author 	NOVO
///
#ifndef _LEDDRIVER_H_
#define _LEDDRIVER_H_


////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <stdbool.h>
#include <stdio.h>
#include <main.h>
#include "stm32f0xx_hal.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
#define LEDDRIVER_UNIT_TEST 	0		///< Module unit test activation.


///
/// \enum	LedDriverLedMode_e
/// \brief	Enumeration of the possible mode of the LEDs.
///
typedef enum
{
	LEDDRIVER_LED_MODE_OFF = 0,       ///< LED is OFF.
	LEDDRIVER_LED_MODE_ON,            ///< LED is OFF.
	LEDDRIVER_LED_MODE_BLINK,         ///< LED is Blinking (the blink pattern is define in oBlinkConfig)
	LEDDRIVER_LED_MODE_MAX_VALUE
} LedDriverLedMode_e;

///
/// \enum	LedDriverLEDBlinkState_e
/// \brief	Enumeration of the possible states of the LEDs while blinking.
///
typedef enum
{
  	LEDDRIVER_LED_BLINK_STATE_WAIT_START = 0,	///< Wait start time if required
	LEDDRIVER_LED_BLINK_STATE_START,	   		///< Initial state: Prepare led memory to start blinking
	LEDDRIVER_LED_BLINK_STATE_ACTIVE,      		///< Middle State: Will do the blink pattern
	LEDDRIVER_LED_BLINK_STATE_PAUSE,       		///< Final state: Pause (LED=RESET) after each blink period
	LEDDRIVER_LED_BLINK_STATE_MAX_VALUE
} LedDriverLEDBlinkState_e;

/// \struct	oLedDriverLEDBlinkMemory_t
/// \brief	Structure containing LED memory specific variables.
///
typedef struct
{
	LedDriverLEDBlinkState_e	eBlinkState;         ///< Internal Led blinking state
	uint16_t 					u16ToggleCounter;    ///< Internal counter to keep track of pulse in the period
	uint16_t 					u16PeriodCounter;    ///< Internal counter to keep track of the period with discontinuous blink pattern
	uint32_t 					u32Timestamp;        ///< Internal timestamp in ms, used to periodically manage LED state.
	bool 						bLedIsOn;            ///< Internal value to keep track of the LED physical states
} oLedDriverLEDBlinkMemory_t;

///
/// \struct	oLedDriverLEDBlinkConfig_t
/// \brief	Structure containing LED congiguration specific variables.
///
///            NumberOfBlink = 3
///       v          v           v
///     ____        ____        ____                      ____
///   _|    |______|    |______|    |____________________|    |_ ...
///
///    |<-->|<---->|                       |<----------->|
///      tOn   tOff                             tPause
///
///    |<----------------------------------------------->|<----
///                           1 period
///
typedef struct
{
	uint8_t   	u8NumberOfBlink;   	///< Number of pulse (on/off cycle) for each period
	uint32_t	u32TOn;				///< Time where LED is ON, for each pulse (on/off cycle) in ms.
	uint32_t	u32TOff;			///< Time where LED is OFF, for each pulse (on/off cycle) in ms.
	uint32_t	u32TPause;			///< Time where LED is OFF, for the pause at the end of the perdiod.
	bool    	bPeriodic;          ///< If the pattern is periodic. If false, the LED return to LEDDRIVER_LED_MODE_OFF
	uint16_t	u16NumberOfPeriod;	///< If the pattern is NOT periodic, number of period before the LED return to LEDDRIVER_LED_MODE_OFF
	bool		bInvertedLedLogic;	///< on-off / off-on logic pattern selection
} oLedDriverLEDBlinkConfig_t;

///
/// \struct	oLedDriverLed_t
/// \brief	Structure containing LED specific variables.
///
typedef struct
{
	LedDriverLedMode_e			eMode;				///< Define the current mode of the LED
	GPIO_TypeDef*  				psPeriph;			///< GPIO Peripheric of the LED
	uint16_t					u16Pin;				///< GPIO pin of the LED
	oLedDriverLEDBlinkConfig_t 	oBlinkConfig;		///< Only for BLINK mode: Configure the blink pattern
	oLedDriverLEDBlinkMemory_t 	oBlinkMemory;		///< Only for BLINK mode: Internal led variable
} oLedDriverLed_t;


////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

// LED Intializing
bool LedDriverInitLed(oLedDriverLed_t *poLed, GPIO_TypeDef *psPeriph, uint16_t u16Pin);

// LED configuration
bool LedDriverSetModeLed(oLedDriverLed_t *poLed, LedDriverLedMode_e eLedMode);
bool LedDriverConfigLedContinuousBlinkPattern(	 oLedDriverLed_t *poLed, 							 uint8_t u8NumberOfBlink, uint32_t u32TOn, uint32_t	u32TOff, uint32_t u32TPause, bool bInvertedLedLogic);
bool LedDriverConfigLedDiscontinuousBlinkPattern(oLedDriverLed_t *poLed, uint16_t u16NumberOfPeriod, uint8_t u8NumberOfBlink, uint32_t u32TOn, uint32_t	u32TOff, uint32_t u32TPause);

// Non blocking LED physical state update
bool LedDriverUpdateLed(oLedDriverLed_t *poLed);

// Unit test
#if (LEDDRIVER_UNIT_TEST == 1)
	bool LedDriverUnitTest(void);
#endif

#endif // _LEDDRIVER_H_
