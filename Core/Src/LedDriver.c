///
/// \file 		LedDriver.c
/// \brief 		[Source file]
///
/// \author 	NOVO
///

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include "LedDriver.h"
#include "Framework.h"
#include "MCUMap.h"


////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

// Default mode for led structure initialisation
#define LEDDRIVER_LED_DEFAULT_MODE						LEDDRIVER_LED_MODE_OFF
#define LEDDRIVER_LED_DEFAULT_BLINK_STATE				LEDDRIVER_LED_BLINK_STATE_START
#define LEDDRIVER_LED_DEFAULT_BLINK_NUMBEROFBLINK		1
#define LEDDRIVER_LED_DEFAULT_BLINK_TON					350
#define LEDDRIVER_LED_DEFAULT_BLINK_TOFF				350
#define LEDDRIVER_LED_DEFAULT_BLINK_TPAUSE				0
#define LEDDRIVER_LED_DEFAULT_BLINK_PERIODIC			true

// MIN MAX values
#define LEDDRIVER_LED_DEFAULT_BLINK_NUMBEROFPERIOD_MIN	1
#define LEDDRIVER_LED_DEFAULT_BLINK_NUMBEROFPERIOD_MAX	65535
#define LEDDRIVER_LED_DEFAULT_BLINK_NUMBEROFBLINK_MIN	1
#define LEDDRIVER_LED_DEFAULT_BLINK_NUMBEROFBLINK_MAX	254
#define LEDDRIVER_LED_DEFAULT_BLINK_TON_MIN				1
#define LEDDRIVER_LED_DEFAULT_BLINK_TON_MAX				3600000 // 1 hour
#define LEDDRIVER_LED_DEFAULT_BLINK_TOFF_MIN			1
#define LEDDRIVER_LED_DEFAULT_BLINK_TOFF_MAX			3600000 // 1 hour
#define LEDDRIVER_LED_DEFAULT_BLINK_TPAUSE_MIN			0
#define LEDDRIVER_LED_DEFAULT_BLINK_TPAUSE_MAX			3600000 // 1 hour


////////////////////////////////////////////////////////////////////////////////
// Private functions
////////////////////////////////////////////////////////////////////////////////
static bool LedDriverBlink(oLedDriverLed_t *poLed);
static bool LedDriverBlinkUpdateLedPhysicalState(oLedDriverLed_t *poLed);
static inline bool LedDriverIsLedOn(oLedDriverLed_t *poLed);


////////////////////////////////////////////////////////////////////////////////
/// \brief 		LedDriverUpdateLed
/// \details	Function that updates a LED physical states depending on
///				it's stucture states.
/// \private
///
/// \param		poLed: led structure.
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool LedDriverUpdateLed(oLedDriverLed_t *poLed)
{
	if (poLed == NULL) return false; // Gards against null ptr

	switch (poLed->eMode)
	{
		case LEDDRIVER_LED_MODE_OFF:
		{
			HAL_GPIO_WritePin(poLed->psPeriph, poLed->u16Pin, GPIO_PIN_RESET);
			poLed->oBlinkMemory.eBlinkState = LEDDRIVER_LED_BLINK_STATE_WAIT_START; // Force led pattern reset
		} break;

		case LEDDRIVER_LED_MODE_ON:
		{
			HAL_GPIO_WritePin(poLed->psPeriph, poLed->u16Pin, GPIO_PIN_SET);
			poLed->oBlinkMemory.eBlinkState = LEDDRIVER_LED_BLINK_STATE_WAIT_START; // Force led pattern reset
		} break;

		case LEDDRIVER_LED_MODE_BLINK:
		{
			if (LedDriverBlink(poLed) == false) return false;
		} break;

		default:
			return false;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief		LedDriverInitLed.
/// \details	Initilased led structure.
/// \private
///
/// \param		poLed: led structure.
/// \param		psPeriph: The led pin periferic (HAL)
/// \param		u16Pin: The led pin number (HAL)
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool LedDriverInitLed(oLedDriverLed_t *poLed, GPIO_TypeDef* psPeriph, uint16_t u16Pin)
{
	if ((poLed == NULL) || (psPeriph == NULL)) return false; // Gards against null ptr
	
	// Save the GPIO in the object.
	poLed->psPeriph = psPeriph;
	poLed->u16Pin = u16Pin;

	// Ensure LED is in an known state.
	poLed->eMode                        = LEDDRIVER_LED_DEFAULT_MODE;
	poLed->oBlinkMemory.eBlinkState 	= LEDDRIVER_LED_DEFAULT_BLINK_STATE;
	
	// Configure a default LED blinking pattern.
	poLed->oBlinkConfig.u8NumberOfBlink = LEDDRIVER_LED_DEFAULT_BLINK_NUMBEROFBLINK;
	poLed->oBlinkConfig.u32TOn          = LEDDRIVER_LED_DEFAULT_BLINK_TON;
	poLed->oBlinkConfig.u32TOff         = LEDDRIVER_LED_DEFAULT_BLINK_TOFF;
	poLed->oBlinkConfig.u32TPause       = LEDDRIVER_LED_DEFAULT_BLINK_TPAUSE;
	poLed->oBlinkConfig.bPeriodic       = LEDDRIVER_LED_DEFAULT_BLINK_PERIODIC;
	
	poLed->oBlinkConfig.bInvertedLedLogic = false;
	
	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief		LedDriverConfigLedContinuousBlinkPattern
/// \details	API to help user configure the blink pattern. This pattern will
/// 				be continuous i.e. will repeat indefinitely.
/// \private
///
/// \param		poLed: led structure.
/// \param		u8NumberOfBlink: Blink parameter for the number of blink for 1 period.
/// \param		u32TOn: Blink parameter for the time where the LED is on for 1 pulse.
/// \param		u32TOff: Blink parameter for the time where the LED is off  for 1 pulse.
/// \param		u32TPause: Blink parameter for the time of pause for 1 period.
/// \param		bInvertedLedLogic: If set to true, logic will be inverted, off-on pattern instead of on-off
///
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool LedDriverConfigLedContinuousBlinkPattern(oLedDriverLed_t *poLed,
											  uint8_t 		   u8NumberOfBlink,
											  uint32_t 		   u32TOn,
											  uint32_t		   u32TOff,
											  uint32_t		   u32TPause,
											  bool			   bInvertedLedLogic)
{
	if (poLed == NULL) return false; // Gards against null ptr
	if (u8NumberOfBlink < LEDDRIVER_LED_DEFAULT_BLINK_NUMBEROFBLINK_MIN) return false;
	if (u8NumberOfBlink > LEDDRIVER_LED_DEFAULT_BLINK_NUMBEROFBLINK_MAX) return false;
	if (u32TOn < LEDDRIVER_LED_DEFAULT_BLINK_TON_MIN) return false;
	if (u32TOn > LEDDRIVER_LED_DEFAULT_BLINK_TON_MAX) return false;
	if (u32TOff < LEDDRIVER_LED_DEFAULT_BLINK_TOFF_MIN) return false;
	if (u32TOff > LEDDRIVER_LED_DEFAULT_BLINK_TOFF_MAX) return false;
	//if (u32TPause < LEDDRIVER_LED_DEFAULT_BLINK_TPAUSE_MIN) return false; // Not necessary with negative values. ( < 0)
	if (u32TPause > LEDDRIVER_LED_DEFAULT_BLINK_TPAUSE_MAX) return false;

	poLed->oBlinkConfig.u8NumberOfBlink = u8NumberOfBlink;
	poLed->oBlinkConfig.u32TOn = u32TOn;
	poLed->oBlinkConfig.u32TOff = u32TOff;
	poLed->oBlinkConfig.u32TPause = u32TPause;
	poLed->oBlinkConfig.bInvertedLedLogic = bInvertedLedLogic;

	poLed->oBlinkConfig.bPeriodic = true;	//because Continuous
	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief		LedDriverConfigLedDiscontinuousBlinkPattern
/// \details	API to help user configure the blink pattern. This pattern will
/// 				be discontinuous i.e. will not repeat indefinitely.
/// \private
///
/// \param		poLed: led structure.
/// \param 		u16NumberOfPeriod: Blink parameter for the number of period before the led switch off.
/// \param		u8NumberOfBlink: Blink parameter for the number of blink for 1 period.
/// \param		u32TOn: Blink parameter for the time where the LED is on for 1 pulse.
/// \param		u32TOff: Blink parameter for the time where the LED is off  for 1 pulse.
/// \param		u32TPause: Blink parameter for the time of pause for 1 period.
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool LedDriverConfigLedDiscontinuousBlinkPattern(oLedDriverLed_t   *poLed,
												 uint16_t	        u16NumberOfPeriod,
											     uint8_t 			u8NumberOfBlink,
												 uint32_t 		  	u32TOn,
												 uint32_t			u32TOff,
												 uint32_t			u32TPause)
{
	if (poLed == NULL) return false; // Gards against null ptr
	if (u16NumberOfPeriod < LEDDRIVER_LED_DEFAULT_BLINK_NUMBEROFPERIOD_MIN) return false;
	// if (u16NumberOfPeriod > LEDDRIVER_LED_DEFAULT_BLINK_NUMBEROFPERIOD_MAX) return false;
	if (u8NumberOfBlink < LEDDRIVER_LED_DEFAULT_BLINK_NUMBEROFBLINK_MIN) return false;
	if (u8NumberOfBlink > LEDDRIVER_LED_DEFAULT_BLINK_NUMBEROFBLINK_MAX) return false;
	if (u32TOn < LEDDRIVER_LED_DEFAULT_BLINK_TON_MIN) return false;
	if (u32TOn > LEDDRIVER_LED_DEFAULT_BLINK_TON_MAX) return false;
	if (u32TOff < LEDDRIVER_LED_DEFAULT_BLINK_TOFF_MIN) return false;
	if (u32TOff > LEDDRIVER_LED_DEFAULT_BLINK_TOFF_MAX) return false;
	//if (u32TPause < LEDDRIVER_LED_DEFAULT_BLINK_TPAUSE_MIN) return false; // Not necessary with negative values. ( < 0)
	if (u32TPause > LEDDRIVER_LED_DEFAULT_BLINK_TPAUSE_MAX) return false;

	poLed->oBlinkConfig.u8NumberOfBlink = u8NumberOfBlink;
	poLed->oBlinkConfig.u32TOn = u32TOn;
	poLed->oBlinkConfig.u32TOff = u32TOff;
	poLed->oBlinkConfig.u32TPause = u32TPause;
	poLed->oBlinkConfig.u16NumberOfPeriod = u16NumberOfPeriod;

	poLed->oBlinkConfig.bPeriodic = false; //because Discontinuous
	poLed->oBlinkMemory.u16PeriodCounter = poLed->oBlinkConfig.u16NumberOfPeriod; // ready the counter
	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief		LedDriverSetModeLed.
/// \details	Change LED mode.
/// \private
///
/// \param		poLed: led structure.
/// \param		eLedMode: The wanted LED mode's name.
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool LedDriverSetModeLed(oLedDriverLed_t *poLed, LedDriverLedMode_e eLedMode)
{
	if ((poLed == NULL) || (eLedMode >= LEDDRIVER_LED_MODE_MAX_VALUE)) return false; // Gards against null ptr
	poLed->eMode = eLedMode;
	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		LedDriverBlink
/// \details	Function that will handle all the timing requiered to produce
///				the blinking pattern specified in the led configuration structure
/// \private
///
/// \param		poLed: led structure.
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
static bool LedDriverBlink(oLedDriverLed_t *poLed)
{
   if(poLed == NULL) return false; // Gards against null ptr

	switch (poLed->oBlinkMemory.eBlinkState)
	{
		case LEDDRIVER_LED_BLINK_STATE_WAIT_START:
	  	{
		  	//Wait a little longer, it will invert sequence
	  		if(poLed->oBlinkConfig.bInvertedLedLogic == true)
			{
		  		if (FrameworkTimeIsDue(poLed->oBlinkMemory.u32Timestamp, poLed->oBlinkConfig.u32TOff))
				{
				  	poLed->oBlinkMemory.eBlinkState = LEDDRIVER_LED_BLINK_STATE_START;
				}
			}
		  	else
		  	{
			  	poLed->oBlinkMemory.eBlinkState = LEDDRIVER_LED_BLINK_STATE_START;
		  	}
	  	} break;
	  
		case LEDDRIVER_LED_BLINK_STATE_START:
		{
			// Prepare internal memory for the period
			poLed->oBlinkMemory.u32Timestamp = FrameworkGetTime();
			poLed->oBlinkMemory.u16ToggleCounter = poLed->oBlinkConfig.u8NumberOfBlink*2;

			// handle the LED initial state
			poLed->oBlinkMemory.bLedIsOn = LedDriverIsLedOn(poLed);
			
			if (poLed->oBlinkMemory.bLedIsOn == true)
			{
				poLed->oBlinkMemory.u16ToggleCounter++; // start with an additional TOff
			}

			// start a new period
			if (poLed->oBlinkConfig.bPeriodic == false)
			{
				poLed->oBlinkMemory.u16PeriodCounter--;
			}

			poLed->oBlinkMemory.eBlinkState = LEDDRIVER_LED_BLINK_STATE_ACTIVE;
		} break;

		case LEDDRIVER_LED_BLINK_STATE_ACTIVE:
		{
			if (poLed->oBlinkMemory.u16ToggleCounter > 0)
			{
				if (LedDriverBlinkUpdateLedPhysicalState(poLed) == false) return false;
			}
			else
			{
				poLed->oBlinkMemory.eBlinkState = LEDDRIVER_LED_BLINK_STATE_PAUSE;
				HAL_GPIO_WritePin(poLed->psPeriph, poLed->u16Pin, GPIO_PIN_RESET);
				poLed->oBlinkMemory.u32Timestamp = FrameworkGetTime(); // Note when the pause started.
			}
		} break;

		case LEDDRIVER_LED_BLINK_STATE_PAUSE:
		{
			// Wait for the pause time to finish
			if (FrameworkTimeIsDue(poLed->oBlinkMemory.u32Timestamp, poLed->oBlinkConfig.u32TPause))
			{
				if (poLed->oBlinkConfig.bPeriodic == false)
				{
					if (poLed->oBlinkMemory.u16PeriodCounter <= 0)
					{
						poLed->eMode = LEDDRIVER_LED_MODE_OFF;  // Quit BLINK mode -> all the period are done.
						poLed->oBlinkMemory.u16PeriodCounter = poLed->oBlinkConfig.u16NumberOfPeriod; // reset the counter.
					}
				}
				
				poLed->oBlinkMemory.eBlinkState = LEDDRIVER_LED_BLINK_STATE_START;
			}
		} break;

		default:
			return false;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief		LedDriverIsLedOn
/// \details	Check if the led is ON
/// \private
///
/// \param		poLed: Led structure.
/// \return		bool: true if Led is on, false otherwise.
////////////////////////////////////////////////////////////////////////////////
static inline bool LedDriverIsLedOn(oLedDriverLed_t *poLed)
{
	return (bool)(HAL_GPIO_ReadPin(poLed->psPeriph, poLed->u16Pin) == GPIO_PIN_SET);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		LedDriverBlinkUpdateLedPhysicalState
/// \details	Non-blocking function that will handle the LED toggling with
///				respect to time, in order to produce the correct periode and
///				duty cycle.
/// \private
///
/// \param		poLed: led structure.
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
static bool LedDriverBlinkUpdateLedPhysicalState(oLedDriverLed_t *poLed)
{
   if(poLed == NULL) return false; // Gards against null ptr

	bool bUpdate = false;
	switch (poLed->oBlinkMemory.bLedIsOn)
	{
		// The LED is ON and we are waiting of the time to turn it OFF
		case true:
		{
			if (FrameworkTimeIsDue(poLed->oBlinkMemory.u32Timestamp, poLed->oBlinkConfig.u32TOn))
			{
				HAL_GPIO_WritePin(poLed->psPeriph, poLed->u16Pin, GPIO_PIN_RESET);
				poLed->oBlinkMemory.bLedIsOn = false;
                bUpdate = true;
			}
		} break;

	 	// The LED is OFF and we are waiting of the time to turn it ON
		case false:
		{
			if (FrameworkTimeIsDue(poLed->oBlinkMemory.u32Timestamp, poLed->oBlinkConfig.u32TOff))
			{
				HAL_GPIO_WritePin(poLed->psPeriph, poLed->u16Pin, GPIO_PIN_SET);
				poLed->oBlinkMemory.bLedIsOn = true;
                bUpdate = true;
			}
		} break;

		default:
			return false;
	}
   if(bUpdate == true)
   {
        poLed->oBlinkMemory.u16ToggleCounter--;
        poLed->oBlinkMemory.u32Timestamp = FrameworkGetTime(); // Note when the led has been toggled.
   }
	return true;
}



////////////////////////////////////////////////////////////////////////////////
/// UNIT TESTS
////////////////////////////////////////////////////////////////////////////////
#if (LEDDRIVER_UNIT_TEST == 1)
#warning "LedDriver unit test enable"

#define BUFFER_SIZE 21

// UT global variable
oLedDriverLed_t oUTLED;
bool UTLedPinState[BUFFER_SIZE];

// UT
static bool LedDriverUTTimingSystemNormalCase(void);
static bool LedDriverUTForceModeChange(void);
static bool LedDriverUTDiscontinuousPattern(void);

///////////////////////////////////////////////////////////////////////////////
/// LedDriverUnitTest
////////////////////////////////////////////////////////////////////////////////
bool LedDriverUnitTest(void)
{
	printf("\r\n");
	printf("------------------------------------------------------- \r\n");
	printf("---- LEDDRIVER UNIT TESTS\r\n");
	printf("------------------------------------------------------- \r\n");

	LedDriverUTTimingSystemNormalCase();
	LedDriverUTForceModeChange();
	LedDriverUTDiscontinuousPattern();

	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// LedDriverUTTimingSystemNormalCase
////////////////////////////////////////////////////////////////////////////////
static bool LedDriverUTTimingSystemNormalCase(void)
{
	bool bResult = false; // test results
	bool UTexpectedLedPinState[BUFFER_SIZE] = {
				false,  // t = 0       (Initial Cond.)
				true,   // t = 100		ON    pulse 1
				true,   // t = 200		ON    pulse 1
				false,  // t = 300		OFF   pulse 1
				false,  // t = 400      OFF	pulse 1
				true,   // t = 500		ON    pulse 2
				true,   // t = 600		ON    pulse 2
				false,  // t = 700		OFF   pulse 2
				false,  // t = 800      OFF 	pulse 2
				false, false, false, false, false, 			// t = 900 to 1300 	Off pause
				false, false, false, false, false, false, // t = 1400 to 2000 Off pause
				true    // t = 2100  	ON 	pulse 1 (new period)
	};

	// prepare the test initial condition
	LedDriverInitLed(&oUTLED, MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN);
	HAL_GPIO_WritePin(MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN, GPIO_PIN_RESET);

	LedDriverConfigLedContinuousBlinkPattern(&oUTLED, 2, 200, 200, 1000);
	LedDriverSetModeLed(&oUTLED, LEDDRIVER_LED_MODE_BLINK);

	// Do the test
	printf("\r\n*** LedDriver UT TimingSystemNormalCase ***\r\n");
	FrameworkTimeDelay(100); // make sure the system initial condition are "realistic"
	for (size_t i = 0; i < BUFFER_SIZE; i++)
	{
		LedDriverUpdateLed(&oUTLED);
		UTLedPinState[i] = LedDriverIsLedOn(&oUTLED); // note current pin state
		FrameworkTimeDelay(100);
	}

	// Valid results: the two arrays should be identical
	for (size_t i = 0; i < BUFFER_SIZE; i++)
	{
		if(UTLedPinState[i] != UTexpectedLedPinState[i]) goto END;
	}

	bResult = true;
END:
	if (bResult)
	{
		printf("----> LedDriverUTTimingSystemNormalCase - PASS.\r\n");
	}
	else
	{
		printf("----> LedDriverUTTimingSystemNormalCase - FAIL.\r\n");
	}
	return bResult;
}

////////////////////////////////////////////////////////////////////////////////
/// LedDriverUTForceModeChange
/// This test will start, interrupt and restart a blink pattern.
/// It will validate that the pattern restart correctly.
////////////////////////////////////////////////////////////////////////////////
static bool LedDriverUTForceModeChange(void)
{
	bool bResult = false;

	// Building setup
	bool UTexpectedLedPinState[BUFFER_SIZE] = {
				/*sequence 1*/
				false,  // t = 0       (Initial Cond.)
				true,   // t = 100		ON    pulse 1
				true,   // t = 200		ON    pulse 1
				false,  // t = 300		OFF   pulse 1
				false,  // t = 400      OFF	  pulse 1
				true,   // t = 500		ON    pulse 2
				/*Interupted*/

				/*sequence 2*/
				false,  // t = 0       (Initial Cond.)
				true,   // t = 100		ON    pulse 1
				true,   // t = 200		ON    pulse 1
				false,  // t = 300		OFF   pulse 1
				false,  // t = 400      OFF   pulse 1
				true,   // t = 500		ON    pulse 2
				true,   // t = 600		ON    pulse 2
				false,  // t = 700		OFF   pulse 2
				false,  // t = 800      OFF   pulse 2
				false, false, false, false, false, false	// t = 900 to 1400 	Off pause
	};

	// prepare the test initial condition
	LedDriverInitLed(&oUTLED, MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN);
	HAL_GPIO_WritePin(MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN, GPIO_PIN_RESET);
	LedDriverConfigLedContinuousBlinkPattern(&oUTLED, 2, 200, 200, 1000);
	size_t i;


	// Testing
	printf("\r\n*** LedDriverUTForceModeChange ***\r\n");

	/* sequence 1*/
	LedDriverSetModeLed(&oUTLED, LEDDRIVER_LED_MODE_BLINK);
	for (i = 0; i < 6; i++) // begin blink
	{
		LedDriverUpdateLed(&oUTLED);
		UTLedPinState[i] = LedDriverIsLedOn(&oUTLED); // note current pin state
		FrameworkTimeDelay(100);
	}
	/* Interupted 1*/
	LedDriverSetModeLed(&oUTLED, LEDDRIVER_LED_MODE_OFF);; //Stop blinking
	LedDriverUpdateLed(&oUTLED);

	/* sequence 2*/
	LedDriverSetModeLed(&oUTLED, LEDDRIVER_LED_MODE_BLINK);
	for (i = 6; i < BUFFER_SIZE; i++) // Restart
	{
		LedDriverUpdateLed(&oUTLED);
		UTLedPinState[i] = LedDriverIsLedOn(&oUTLED); // note current pin state
		FrameworkTimeDelay(100);
	}

	// Valid results: the two arrays should be identical
	for (size_t i = 0; i < BUFFER_SIZE; i++)
	{
		if(UTLedPinState[i] != UTexpectedLedPinState[i]) goto END;
	}

	bResult = true;
END:
	if (bResult)
	{
		printf("----> LedDriverUTForceModeChange - PASS.\r\n");
	}
	else
	{
		printf("----> LedDriverUTForceModeChange - FAIL.\r\n");
	}
	return bResult;
}

////////////////////////////////////////////////////////////////////////////////
/// LedDriverUTDiscontinuousPattern
////////////////////////////////////////////////////////////////////////////////
static bool LedDriverUTDiscontinuousPattern(void)
{
	bool bResult = false;

	// Building setup
	uint16_t u1ExpectedNumberOfTransition = 4*2*2; // NPeriod * NPulse * 2Edge

	// prepare the test initial condition
	LedDriverInitLed(&oUTLED, MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN);
	HAL_GPIO_WritePin(MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN, GPIO_PIN_RESET);
	LedDriverConfigLedDiscontinuousBlinkPattern(&oUTLED, 4, 2, 200, 200, 1000);


	// Testing
	printf("\r\n*** LedDriverUTDiscontinuousPattern ***\r\n");
	LedDriverSetModeLed(&oUTLED, LEDDRIVER_LED_MODE_BLINK);
	FrameworkTimeDelay(200); // make sure the system initial condition are "realistic"

	uint16_t u16TransitionCounter=0;
	bool 		bPreviousPinState = false;
	for(size_t i = 0; i < 100; i++) // The time needed to do the full pattern should be 72 * 100ms
	{
		LedDriverUpdateLed(&oUTLED);
		if (LedDriverIsLedOn(&oUTLED) ^ bPreviousPinState)
		{
			u16TransitionCounter++;
		}
		FrameworkTimeDelay(100);
	}

	// Valid results
	if (u16TransitionCounter !=u1ExpectedNumberOfTransition) goto END;
	bResult = true;

END:
	if (bResult)
	{
		printf("----> LedDriverUTDiscontinuousPattern - PASS.\r\n");
	}
	else
	{
		printf("----> LedDriverUTDiscontinuousPattern - FAIL.\r\n");
	}
	return bResult;
}


#endif /*Unit tests*/
