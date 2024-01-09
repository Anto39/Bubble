///
/// \file 		Framework.c
/// \brief 		[Source file]
///				
/// \author 	NOVO
///
////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <gpio.h>
#include "Framework.h"
#include "usart.h"
#include "iwdg.h"
#include "adc.h"
#include "MCUMap.h"
#include <stdio.h>
#include <string.h>


////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
#define FRAMEWORK_FW_VERSION_MAJOR 					1		///< Current firmware version, major number.
#define FRAMEWORK_FW_VERSION_MINOR 					0		///< Current firmware version, minor number.
#define FRAMEWORK_FW_VERSION_REV					2		///< Current firmware version, revision number.

#define FRAMEWORK_HW_VERSION_LEVEL_ERROR_TO_0		300		///< Analog value for hardware version to tell if version is 0 or we have an error
#define FRAMEWORK_HW_VERSION_LEVEL_0_TO_1			800		///< Analog value for hardware version to tell if version is 0 or 1
#define FRAMEWORK_HW_VERSION_LEVEL_1_TO_2			1175	///< Analog value for hardware version to tell if version is 1 or 2
#define FRAMEWORK_HW_VERSION_LEVEL_2_TO_3			1550	///< Analog value for hardware version to tell if version is 2 or 3
#define FRAMEWORK_HW_VERSION_LEVEL_3_TO_4			1950	///< Analog value for hardware version to tell if version is 3 or 4
#define FRAMEWORK_HW_VERSION_LEVEL_4_TO_5			2350	///< Analog value for hardware version to tell if version is 4 or 5
#define FRAMEWORK_HW_VERSION_LEVEL_5_TO_6			2700	///< Analog value for hardware version to tell if version is 5 or 6
#define FRAMEWORK_HW_VERSION_LEVEL_6_TO_7			3050	///< Analog value for hardware version to tell if version is 6 or 7
#define FRAMEWORK_HW_VERSION_LEVEL_7_TO_8			3400	///< Analog value for hardware version to tell if version is 7 or 8
#define FRAMEWORK_HW_VERSION_LEVEL_8_TO_9			3775	///< Analog value for hardware version to tell if version is 8 or 9
#define FRAMEWORK_HW_VERSION_LEVEL_9_TO_ERROR		4025	///< Analog value for hardware version to tell if version is 9 or we have an error

#define FRAMEWORK_MODEL_ID_LEVEL_BOTTOMLOAD			900		///< Analog value for hardware detection of the product model. Bottom Load.
#define FRAMEWORK_MODEL_ID_LEVEL_ICEBOXX			1600	///< Analog value for hardware detection of the product model. Iceboxx.
#define FRAMEWORK_MODEL_ID_LEVEL_HOTDEGREE			2400	///< Analog value for hardware detection of the product model. Hot Degree.
#define FRAMEWORK_MODEL_ID_LEVEL_CO2POU				3100	///< Analog value for hardware detection of the product model. Co2 POU.
#define FRAMEWORK_MODEL_ID_LEVEL_CO2BOTTLE			3900	///< Analog value for hardware detection of the product model. Co2 Bottle.

#define FRAMEWORK_DEBOUNCE_PERIOD_MS				10		///< Digital inputs debouncing period, in ms.
#define FRAMEWORK_HB_TOGGLE_PERIOD_MS				500		///< Toggling period for the heartbeat LED, in ms.
#define FRAMEWORK_HB_TOGGLE_FATAL_ERROR_PERIOD_MS	50		///< Toggling period for the heartbeat LED, in ms.
#define FRAMEWORK_INT_WD_FEED_PERIOD_MS				100		///< Feeding period for the internal watchdog, in ms.
#define FRAMEWORK_WD_COMP_MASK						(FRAMEWORK_WD_COMPONENT_MAIN)		/// Internal watchdog component mask.

#ifdef __GNUC__
   // With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   // set to 'Yes') calls __io_putchar()
   #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
   #else
   #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif // __GNUC__


////////////////////////////////////////////////////////////////////////////////
// Private functions
////////////////////////////////////////////////////////////////////////////////
void 	FrameworkFeedInternalWD(void);
uint8_t	FrameworkGetVersionNumber(uint16_t u16AnalogValue);


////////////////////////////////////////////////////////////////////////////////
// Private variables
////////////////////////////////////////////////////////////////////////////////
volatile uint16_t 	g_u16WDCompMask		= 0x0000;						///< Internal Watchdog component mask.
uint32_t 			g_u32HBTime			= 0;							///< Heartbeat timestamp in ms, used to toggle the HB LED.
uint32_t 			g_u32IWDTime		= 0;							///< Internal Watchdog timestamp in ms, used to periodically feed the watchdog.
uint32_t			g_u32DebounceTime	= 0;							///< Digital inputs debouncing timestamp.
FrameworkModelID_e	g_modelID			= FRAMEWORK_MODEL_ID_UNKNOWN;	///< Detected product model.
bool				g_bAllowHB			= true;

#if (FRAMEWORK_UNIT_TEST == 1)
uint16_t u16ADCRaw[HAL_ADC_AI_TOTAL_NB];
uint16_t u16ADCAvg[HAL_ADC_AI_TOTAL_NB];
#endif


////////////////////////////////////////////////////////////////////////////////
/// \brief 		FrameworkInit
/// \details	Initialization of the framework module.
///           
/// \public
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool FrameworkInit(void)
{
	uint16_t u16Data 	= 0;
	
	// Initialize the module.
	g_u16WDCompMask		= 0x0000;
	g_u32HBTime 		= 0;
	g_u32IWDTime 		= 0;
	g_u32DebounceTime	= 0;
	
	// Start the internal watchdog.
#if (FRAMEWORK_INTERNAL_WATCHDOG_ENABLED == 0)
	#warning "Framework - Internal Watchdog Disabled."
#endif
	
	// Setup digital inputs debouncing.
	GPIOInitDDI();
	
	// Setup the adc conversion.
	HAL_ADC_Custom_Start();
	
	// Retrieve the model ID.
	HAL_ADC_GetValueRaw(HAL_ADC_AI_MODEL_ID_CH, &u16Data);
	
	g_modelID = FRAMEWORK_MODEL_ID_UNKNOWN;
	if (u16Data < FRAMEWORK_MODEL_ID_LEVEL_BOTTOMLOAD)
	{
		g_modelID = FRAMEWORK_MODEL_ID_BOTTOMLOAD;
	}
	else if (u16Data < FRAMEWORK_MODEL_ID_LEVEL_ICEBOXX)
	{
		g_modelID = FRAMEWORK_MODEL_ID_ICEBOXX;
	}
	else if (u16Data < FRAMEWORK_MODEL_ID_LEVEL_HOTDEGREE)
	{
		g_modelID = FRAMEWORK_MODEL_ID_HOTDEGREE;
	}
	else if (u16Data < FRAMEWORK_MODEL_ID_LEVEL_CO2POU)
	{
		g_modelID = FRAMEWORK_MODEL_ID_CO2POU;
	}
	else if (u16Data < FRAMEWORK_MODEL_ID_LEVEL_CO2BOTTLE)
	{
		g_modelID = FRAMEWORK_MODEL_ID_CO2BOTTLE;
	}
	else
	{
		return false;
	}
	
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		FrameworkTask
/// \details	Execution of the main task of the module.
///           
/// \public
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool FrameworkTask(void)
{
#if (FRAMEWORK_IWDT_2_UNIT_TEST == 0)
	// Signal the execution of the task.
	FrameworkSignalWDComp(FRAMEWORK_WD_COMPONENT_MAIN);
#endif
	
	// Run the periodic ADC averaging task.
	if (HAL_ADC_Task() == false) return false;
	
	// Debounce all digital inputs.
	if (FrameworkTimeIsDue(g_u32DebounceTime, FRAMEWORK_DEBOUNCE_PERIOD_MS) == true)
	{
		GPIODebounceDDI();
		g_u32DebounceTime = FrameworkGetTime();
	}
	
	// Toggle the heartbeat LED if needed.
	if (FrameworkTimeIsDue(g_u32HBTime, FRAMEWORK_HB_TOGGLE_PERIOD_MS) == true)
	{
		if (g_bAllowHB == true)
		{
			HAL_GPIO_TogglePin(MCUMAP_DO_LED_HB_PERIPH, MCUMAP_DO_LED_HB_PIN);
		}
		
		g_u32HBTime = FrameworkGetTime();
	}
	
	// Signal the internal watchdog, if needed.
	if ((FrameworkTimeIsDue(g_u32IWDTime, FRAMEWORK_INT_WD_FEED_PERIOD_MS) == true) && ((g_u16WDCompMask & FRAMEWORK_WD_COMP_MASK) == FRAMEWORK_WD_COMP_MASK))
	{
		g_u32IWDTime = FrameworkGetTime();
		FrameworkFeedInternalWD();
		
		// Reset the mask.
		g_u16WDCompMask	= 0x0000;
	}
	
	// Monitor the button to check if we start the HB LED
	GPIO_PinState pinState 	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8);
	if (pinState == GPIO_PIN_RESET)
	{
		g_bAllowHB = true;
	}
	
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		FrameworkFatalErrorTask
/// \details	Execution of the main task when in fatal error.
///           
/// \public
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
void FrameworkFatalErrorTask(void)
{
	// Toggle the heartbeat LED if needed.
	if (FrameworkTimeIsDue(g_u32HBTime, FRAMEWORK_HB_TOGGLE_FATAL_ERROR_PERIOD_MS) == true)
	{
		HAL_GPIO_TogglePin(MCUMAP_DO_LED_HB_PERIPH, MCUMAP_DO_LED_HB_PIN);
		g_u32HBTime = FrameworkGetTime();
	}
	
	// Signal the internal watchdog, if needed.
	if (FrameworkTimeIsDue(g_u32IWDTime, FRAMEWORK_INT_WD_FEED_PERIOD_MS) == true)
	{
		g_u32IWDTime = FrameworkGetTime();
		FrameworkFeedInternalWD();
	}
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		FrameworkGetHWVersion
/// \details	Retrieve the HW version.
///
/// \public
/// \param[out]	pu8HWVerMajor: Handle to put HW version, major.
/// \param[out]	pu8HWVerMinor: Handle to put HW version, minor.
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool FrameworkGetHWVersion(uint8_t* pu8HWVerMajor, uint8_t* pu8HWVerMinor)
{
	uint16_t u16Data = 0;
	if((pu8HWVerMajor == NULL) || (pu8HWVerMinor == NULL)) return false;

	HAL_ADC_GetValueRaw(HAL_ADC_AI_HW_VER_MAJOR, &u16Data);
	*pu8HWVerMajor = FrameworkGetVersionNumber(u16Data);

	HAL_ADC_GetValueRaw(HAL_ADC_AI_HW_VER_MINOR, &u16Data);
	*pu8HWVerMinor = FrameworkGetVersionNumber(u16Data);
   
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		FrameworkGetFWVersion
/// \details	Retrieve the FW version.
/// 
/// \public
/// \param[out]	pu8FWVerMajor: Handle to put FW version, major.
/// \param[out]	pu8FWVerMinor: Handle to put FW version, minor.
/// \param[out]	pu8FWVerRev: Handle to put FW version, revision.
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool FrameworkGetFWVersion(uint8_t * pu8FWVerMajor, uint8_t * pu8FWVerMinor, uint8_t * pu8FWVerRev)
{
	if ((pu8FWVerMajor == NULL) || (pu8FWVerMinor == NULL) || (pu8FWVerRev == NULL)) return false;
	*pu8FWVerMajor 	= FRAMEWORK_FW_VERSION_MAJOR;
	*pu8FWVerMinor 	= FRAMEWORK_FW_VERSION_MINOR;
	*pu8FWVerRev 	= FRAMEWORK_FW_VERSION_REV;
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		FrameworkGetTime()
/// \details	Return the freerunning timer
/// \public
/// \return		uint32_t: freerunning timer value
////////////////////////////////////////////////////////////////////////////////
uint32_t FrameworkGetTime(void)
{
	return HAL_GetTick();
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		FrameworkGetTimeDiff
/// \details	Return the difference of time between the compare timer and 
///					the freerunning timer.
/// \public
/// \param[in]	u32Compare: Timer to compare with the freerunning timer.
/// \return		uint32_t: time between the 2 timer
////////////////////////////////////////////////////////////////////////////////
uint32_t FrameworkGetTimeDiff(uint32_t u32Compare)
{
	return (FrameworkGetTime() - u32Compare);
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		FrameworkTimeIsDue
/// \details	Indicates if the verified time is elapse.
///
/// \public
/// \param[in]	u32Compare: Time to compare.
/// \param[in]	u32Interval: Interval of due time
/// \return		bool: true if time is elapsed, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool FrameworkTimeIsDue(uint32_t u32Compare, uint32_t u32Interval)
{
	if (FrameworkGetTimeDiff(u32Compare) >= u32Interval){return true;}
	else {return false;}
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		FrameworkTimeDelay
/// \details	This function is useful when the caller wants to wait some
///				time before continuing its behavior. For example, waiting
///				a specific amount of time between two actions.
///
/// \public
/// \param[in] 	u32DelayInMs: Number of ms to wait (block). Must be lower than the external watchdog reset delay.
/// \return		void.
////////////////////////////////////////////////////////////////////////////////
void FrameworkTimeDelay(uint32_t u32DelayInMs)
{
	uint32_t u32SystickCnt = FrameworkGetTime();
	do
	{
		FrameworkFeedInternalWD();
	} while (FrameworkGetTimeDiff(u32SystickCnt) < u32DelayInMs);
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		FrameworkFeedInternalWD
/// \details	Feed the internal watchdog.
///
/// \private
/// \param		None
/// \return		void.
////////////////////////////////////////////////////////////////////////////////
void FrameworkFeedInternalWD(void)
{
#if (FRAMEWORK_INTERNAL_WATCHDOG_ENABLED == 1)
	MX_IWDG_Refresh();
#endif
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		FrameworkGetVersionNumber
/// \details	Get board hardware version according to received analog value.
///
/// \private
/// \param[in]	u16AnalogValue: Analog value to convert into board version.
/// \return		uint8_t: Version number (0 to 9), 0xFF if analog value is out-of-range.
////////////////////////////////////////////////////////////////////////////////
uint8_t FrameworkGetVersionNumber(uint16_t u16AnalogValue)
{
	uint8_t u8Version = 0xFF;

	if (u16AnalogValue < FRAMEWORK_HW_VERSION_LEVEL_ERROR_TO_0)
	{
		//Error
		u8Version = 0xFF;
	}
	else if(u16AnalogValue < FRAMEWORK_HW_VERSION_LEVEL_0_TO_1)
	{
		u8Version = 0x00;
	}
	else if(u16AnalogValue < FRAMEWORK_HW_VERSION_LEVEL_1_TO_2)
	{
		u8Version = 0x01;
	}
	else if(u16AnalogValue < FRAMEWORK_HW_VERSION_LEVEL_2_TO_3)
	{
		u8Version = 0x02;
	}
	else if(u16AnalogValue < FRAMEWORK_HW_VERSION_LEVEL_3_TO_4)
	{
		u8Version = 0x03;
	}
	else if(u16AnalogValue < FRAMEWORK_HW_VERSION_LEVEL_4_TO_5)
	{
		u8Version = 0x04;
	}
	else if(u16AnalogValue < FRAMEWORK_HW_VERSION_LEVEL_5_TO_6)
	{
		u8Version = 0x05;
	}
	else if(u16AnalogValue < FRAMEWORK_HW_VERSION_LEVEL_6_TO_7)
	{
		u8Version = 0x06;
	}
	else if(u16AnalogValue < FRAMEWORK_HW_VERSION_LEVEL_7_TO_8)
	{
		u8Version = 0x07;
	}
	else if(u16AnalogValue < FRAMEWORK_HW_VERSION_LEVEL_8_TO_9)
	{
		u8Version = 0x08;
	}
	else if(u16AnalogValue < FRAMEWORK_HW_VERSION_LEVEL_9_TO_ERROR)
	{
		u8Version = 0x09;
	}
	else
	{
		u8Version = 0xFF;
	}

	return u8Version;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		FrameworkSignalWDComp
/// \details	Signal the execution of a WD component.
///
/// \public
/// \param[in]	eWDCompToSignal: the watchdog component to signal. See FrameworkWDComp_e.
/// \return		void.
////////////////////////////////////////////////////////////////////////////////
void FrameworkSignalWDComp(FrameworkWDComp_e eWDCompToSignal)
{
	g_u16WDCompMask |= eWDCompToSignal;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		FrameworkGetModelID
/// \details	Retrieve the detected product model.
///
/// \public
/// \param[out]	pModelID	the detected product model ID.
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool FrameworkGetModelID(FrameworkModelID_e* pModelID)
{
	if (pModelID == NULL) return false;
	
	*pModelID = g_modelID;
	
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		FrameworkSystemTimeISR
/// \details	Interrupt handler called by the System Time interrupt. For Framework management.
///
/// \public
/// \param[in]	None
/// \return		void.
////////////////////////////////////////////////////////////////////////////////
void FrameworkSystemTimeISR(void)
{
	// DO NOT compute internal system time, all managed by the ST HAL layer.
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		FrameworkExtract32bits
/// \details	Extraction of a 32bits value from a 4 bytes array.
///
/// \public
/// \param[out]	pu32Dst: Handle to to put the extracted 32bits value.
/// \param[in]	pu8Src: Handle to the 4 bytes array
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool FrameworkExtract32bits(uint32_t * pu32Dst, uint8_t * pu8Src)
{
	if ((pu32Dst == NULL) || (pu8Src == NULL)) return false;
	memcpy(pu32Dst,pu8Src, 4);
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		PUTCHAR_PROTOTYPE
/// \details	Retargets the C library printf function to the verbose UART.
///
/// \public
/// \return		uint8_t: Character to put in the IO printf
////////////////////////////////////////////////////////////////////////////////
#if !IAR_SIMULATION
PUTCHAR_PROTOTYPE
{
	// Write a character to the framework UART handle and Loop until the end of transmission
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}
#endif

////////////////////////////////////////////////////////////////////////////////
/// UNIT TESTS
////////////////////////////////////////////////////////////////////////////////
#if (FRAMEWORK_UNIT_TEST == 1)
#warning "Framework unit test enable"

//#include "arm.h"

bool FrameworkUTSystemTime(void);
bool FrameworkUTFWVersion(void);
bool FrameworkUTHWVersion(void);
bool FrameworkUTHDLED(void);
bool FrameworkUTInternalWD(void);
bool FrameworkUTADC(void);
bool FrameworkUTDIN(void);

////////////////////////////////////////////////////////////////////////////////
/// FrameworkUnitTest
////////////////////////////////////////////////////////////////////////////////
bool FrameworkUnitTest(void)
{
	printf("\r\n");
	printf("------------------------------------------------------- \r\n");
	printf("---- FRAMEWORK UNIT TESTS\r\n");
	printf("------------------------------------------------------- \r\n");
	
	// Blocking tests (if enabled).
	if (FrameworkUTHDLED() == false) while(1);
#if (FRAMEWORK_INTERNAL_WATCHDOG_ENABLED == 1)
	if (FrameworkUTInternalWD() == false) while(1);
#endif
#if (FRAMEWORK_ADC_UNIT_TEST == 1)
	if (FrameworkUTADC() == false) while(1);
#endif
	
	// Non-blocking tests.
	if (FrameworkUTFWVersion() == false) while(1);
	if (FrameworkUTHWVersion() == false) while(1);
	if (FrameworkUTSystemTime() == false) while(1);
	if (FrameworkUTDIN() == false) while(1);
	
	
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// FrameworkUTSystemTime
////////////////////////////////////////////////////////////////////////////////
bool FrameworkUTSystemTime(void)
{
	bool 		bResult					= false;
	uint32_t	u32TestTimestamp		= 0;
	uint32_t	u32TimeIsDueTimestamp	= 0;
	
	printf("\r\n*** FrameworkUTSystemTime ***\r\n");
	
	// Setup UT TP for visual verification.
	HAL_GPIO_WritePin(MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN, GPIO_PIN_SET);
	FrameworkTimeDelay(1);
	HAL_GPIO_WritePin(MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN, GPIO_PIN_RESET);
	FrameworkTimeDelay(1);
	HAL_GPIO_WritePin(MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN, GPIO_PIN_SET);
	
	// Timestamp the start of the test.
	u32TestTimestamp = FrameworkGetTime();
	
	// Verification of the delay function.
	HAL_GPIO_TogglePin(MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN);
	FrameworkTimeDelay(10);
	HAL_GPIO_TogglePin(MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN);
	FrameworkTimeDelay(10);
	HAL_GPIO_TogglePin(MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN);
	FrameworkTimeDelay(10);
	HAL_GPIO_TogglePin(MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN);
	FrameworkTimeDelay(5);
	HAL_GPIO_TogglePin(MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN);
	
	// Verification of the Time Is Due function.
	u32TimeIsDueTimestamp = FrameworkGetTime();
	while (1)
	{
		FrameworkFeedInternalWD();
		if (FrameworkTask() == false) goto END;
		if (FrameworkTimeIsDue(u32TimeIsDueTimestamp, 6250) == true) break;
	}
	
	// Verification of the Time Difference function.
	if (FrameworkGetTimeDiff(u32TestTimestamp) != 6285) goto END;
	HAL_GPIO_WritePin(MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN, GPIO_PIN_RESET);
	printf("STEP - Verify the timings on the scope.\r\n");
	
	bResult = true;
	
END:
	if (bResult)
	{
		printf("----> FrameworkUTSystemTime - PASS.\r\n");
	}
	else
	{
		printf("----> FrameworkUTSystemTime - FAIL.\r\n");
	}
	
	return bResult;
}
////////////////////////////////////////////////////////////////////////////////
/// FrameworkUTFWVersion
////////////////////////////////////////////////////////////////////////////////
bool FrameworkUTFWVersion(void)
{
	bool 	bResult		= false;
	uint8_t u8Major		= 0;
	uint8_t u8Minor		= 0;
	uint8_t u8Revision	= 0;
	
	printf("\r\n*** FrameworkUTFWVersion ***\r\n");
	
	// Try retrieving with an invalid pointer.
	if (FrameworkGetFWVersion(0, &u8Minor, &u8Revision) == true) goto END;
	if (FrameworkGetFWVersion(&u8Major, 0, &u8Revision) == true) goto END;
	if (FrameworkGetFWVersion(&u8Major, &u8Minor, 0) == true) goto END;
	
	// Get and validate.
	if (FrameworkGetFWVersion(&u8Major, &u8Minor, &u8Revision) == false) goto END;
	if ((u8Major != FRAMEWORK_FW_VERSION_MAJOR) || (u8Minor != FRAMEWORK_FW_VERSION_MINOR) || (u8Revision != FRAMEWORK_FW_VERSION_REV)) goto END;
	
	bResult = true;
	
END:
	if (bResult)
	{
		printf("----> FrameworkUTFWVersion - PASS.\r\n");
	}
	else
	{
		printf("----> FrameworkUTFWVersion - FAIL.\r\n");
	}
	
	return bResult;
}
////////////////////////////////////////////////////////////////////////////////
/// FrameworkUTHWVersion
////////////////////////////////////////////////////////////////////////////////
bool FrameworkUTHWVersion(void)
{
	bool 		bResult			= false;
	uint8_t		u8VerMajor		= 0;
	uint8_t		u8VerMinor		= 0;
	
	printf("\r\n*** FrameworkUTHWVersion ***\r\n");
	
	// Validate version table.
	if (FrameworkGetVersionNumber(0) != 0xFF) goto END;
	if (FrameworkGetVersionNumber(299) != 0xFF) goto END;
	if (FrameworkGetVersionNumber(300) != 0) goto END;
	if (FrameworkGetVersionNumber(799) != 0) goto END;
	if (FrameworkGetVersionNumber(800) != 1) goto END;
	if (FrameworkGetVersionNumber(1174) != 1) goto END;
	if (FrameworkGetVersionNumber(1175) != 2) goto END;
	if (FrameworkGetVersionNumber(1549) != 2) goto END;
	if (FrameworkGetVersionNumber(1550) != 3) goto END;
	if (FrameworkGetVersionNumber(1949) != 3) goto END;
	if (FrameworkGetVersionNumber(1950) != 4) goto END;
	if (FrameworkGetVersionNumber(2349) != 4) goto END;
	if (FrameworkGetVersionNumber(2350) != 5) goto END;
	if (FrameworkGetVersionNumber(2699) != 5) goto END;
	if (FrameworkGetVersionNumber(2700) != 6) goto END;
	if (FrameworkGetVersionNumber(3049) != 6) goto END;
	if (FrameworkGetVersionNumber(3050) != 7) goto END;
	if (FrameworkGetVersionNumber(3399) != 7) goto END;
	if (FrameworkGetVersionNumber(3400) != 8) goto END;
	if (FrameworkGetVersionNumber(3774) != 8) goto END;
	if (FrameworkGetVersionNumber(3775) != 9) goto END;
	if (FrameworkGetVersionNumber(4024) != 9) goto END;
	if (FrameworkGetVersionNumber(4025) != 0xFF) goto END;
	if (FrameworkGetVersionNumber(4095) != 0xFF) goto END;

	// Validate current board version.
	if (FrameworkGetHWVersion(0, &u8VerMinor) == true) goto END;
	if (FrameworkGetHWVersion(&u8VerMajor, 0) == true) goto END;
	if (FrameworkGetHWVersion(&u8VerMajor, &u8VerMinor) == false) goto END;
	if ((u8VerMajor == 0xFF) || (u8VerMinor == 0xFF)) goto END;
	
	printf("STEP - Verify the hardware version. Detected = %d.%d.\r\n", u8VerMajor, u8VerMinor);
	bResult = true;
	
END:
	if (bResult)
	{
		printf("----> FrameworkUTHWVersion - PASS.\r\n");
	}
	else
	{
		printf("----> FrameworkUTHWVersion - FAIL.\r\n");
	}
	
	return bResult;
}
////////////////////////////////////////////////////////////////////////////////
/// FrameworkUTHDLED
////////////////////////////////////////////////////////////////////////////////
bool FrameworkUTHDLED(void)
{
	bool 		bResult			= false;
	
	uint32_t	u32Timestamp	= 0;
	
	printf("\r\n*** FrameworkUTHDLED ***\r\n");
	printf("Normal behavior for 5s.\r\n");

	u32Timestamp = FrameworkGetTime();
	while (FrameworkTimeIsDue(u32Timestamp, 5000) != true)
	{
		FrameworkFeedInternalWD();
		if (FrameworkTask() == false) goto END;
	}
	
	printf("Fatal error behavior for 5s.\r\n");
	u32Timestamp = FrameworkGetTime();
	while (FrameworkTimeIsDue(u32Timestamp, 5000) != true)
	{
		FrameworkFatalErrorTask();
	}

	bResult = true;
	
END:
	if (bResult)
	{
		printf("----> FrameworkUTHDLED - PASS.\r\n");
	}
	else
	{
		printf("----> FrameworkUTHDLED - FAIL.\r\n");
	}
	
	return bResult;
}
////////////////////////////////////////////////////////////////////////////////
/// FrameworkUTInternalWD
////////////////////////////////////////////////////////////////////////////////
bool FrameworkUTInternalWD(void)
{
	uint32_t 	u32Timestamp	= FrameworkGetTime();
	
	printf("\r\n*** FrameworkUTInternalWD ***\r\n");
	
	HAL_GPIO_WritePin(MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN, GPIO_PIN_SET);
	FrameworkTimeDelay(1);
	HAL_GPIO_WritePin(MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN, GPIO_PIN_RESET);
	
#if (FRAMEWORK_INTERNAL_WATCHDOG_ENABLED == 0)
	printf("WARNING, INTERNAL WATCHDOG IS NOT ACTIVATED.\r\n");
#endif
	
	printf("Case 1 - Normal behavior for 1500ms.\r\n");
	while (FrameworkTimeIsDue(u32Timestamp, 1500) != true)
	{
		FrameworkSignalWDComp(FRAMEWORK_WD_COMPONENT_MAIN);
		if (FrameworkTask() == false) return false;
	}
	
	// Provoque a IWDT reset.
	HAL_GPIO_WritePin(MCUMAP_DO_TP_PERIPH, MCUMAP_DO_TP_PIN, GPIO_PIN_SET);
	printf("Starving the watchdog... Reset in 409ms.\r\n");
	while(1);
	
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// FrameworkUTADC
////////////////////////////////////////////////////////////////////////////////
bool FrameworkUTADC(void)
{
	uint8_t u8Idx = 0;

	printf("\r\n*** FrameworkUTADC ***\r\n");
	printf("Use the debugger Live View and monitor all averages and raw data. Apply voltage to AIN.\r\n");

	memset(u16ADCRaw, 0, sizeof(u16ADCRaw));
	memset(u16ADCAvg, 0, sizeof(u16ADCAvg));

	while (1)
	{
		FrameworkSignalWDComp(FRAMEWORK_WD_COMPONENT_MAIN);
		if (FrameworkTask() == false) return false;
		
		for (u8Idx = 0; u8Idx < HAL_ADC_AI_TOTAL_NB; ++u8Idx)
		{
			if (HAL_ADC_GetValueRaw((HAL_ADC_AnalogInputs_e)u8Idx, &u16ADCRaw[u8Idx]) == false) return false;
			if (HAL_ADC_GetValueAveraged((HAL_ADC_AnalogInputs_e)u8Idx, &u16ADCAvg[u8Idx]) == false) return false;
		}
	}

	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// FrameworkUTDIN
////////////////////////////////////////////////////////////////////////////////
bool FrameworkUTDIN(void)
{
	GPIO_PinState 	pinState 		= GPIO_PIN_RESET;
	bool			bDebouncedState	= false;

	printf("\r\n*** FrameworkUTDIN ***\r\n");

	printf("\r\n*** Force pin value to 0V ***\r\n");
	// Get the raw input and debounced input.
	pinState = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN);
	GPIOGetDDI(GPIO_DDI_BUTTON_1, &bDebouncedState);
	if ((pinState != GPIO_PIN_RESET) || (bDebouncedState != false)) return false;
	
	printf("\r\n*** Force pin value to 3.3V ***\r\n");
	// Apply voltage to pin. Debounce counter = 0.
	pinState = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN);
	GPIOGetDDI(GPIO_DDI_BUTTON_1, &bDebouncedState);
	if ((pinState != GPIO_PIN_SET) || (bDebouncedState != false)) return false;
	
	// Force one cycle of debouncing.
	GPIODebounceDDI();
	
	// Debounce counter = 1.
	pinState = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN);
	GPIOGetDDI(GPIO_DDI_BUTTON_1, &bDebouncedState);
	if ((pinState != GPIO_PIN_SET) || (bDebouncedState != false)) return false;
	
	// Force one cycle of debouncing.
	GPIODebounceDDI();
	
	// Debounce counter = 2.
	pinState = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN);
	GPIOGetDDI(GPIO_DDI_BUTTON_1, &bDebouncedState);
	if ((pinState != GPIO_PIN_SET) || (bDebouncedState != false)) return false;
	
	// Force one cycle of debouncing.
	GPIODebounceDDI();
	
	// Debounce counter = 3.
	pinState = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN);
	GPIOGetDDI(GPIO_DDI_BUTTON_1, &bDebouncedState);
	if ((pinState != GPIO_PIN_SET) || (bDebouncedState != false)) return false;
	
	// Force one cycle of debouncing.
	GPIODebounceDDI();
	
	// Debounce counter = 3 (max).
	pinState = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN);
	GPIOGetDDI(GPIO_DDI_BUTTON_1, &bDebouncedState);
	if ((pinState != GPIO_PIN_SET) || (bDebouncedState != true)) return false;
	
	// Force one cycle of debouncing.
	GPIODebounceDDI();
	
	// Debounce counter = 3 (max).
	pinState = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN);
	GPIOGetDDI(GPIO_DDI_BUTTON_1, &bDebouncedState);
	if ((pinState != GPIO_PIN_SET) || (bDebouncedState != true)) return false;

	printf("\r\n*** Force pin value to 0V ***\r\n");
	// Remove voltage from pin. Debounce counter = 3 (max).
	pinState = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN);
	GPIOGetDDI(GPIO_DDI_BUTTON_1, &bDebouncedState);
	if ((pinState != GPIO_PIN_RESET) || (bDebouncedState != true)) return false;
	
	// Force one cycle of debouncing.
	GPIODebounceDDI();
	
	// Remove voltage from pin. Debounce counter = 2.
	pinState = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN);
	GPIOGetDDI(GPIO_DDI_BUTTON_1, &bDebouncedState);
	if ((pinState != GPIO_PIN_RESET) || (bDebouncedState != true)) return false;
	
	// Force one cycle of debouncing.
	GPIODebounceDDI();
	
	// Remove voltage from pin. Debounce counter = 1.
	pinState = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN);
	GPIOGetDDI(GPIO_DDI_BUTTON_1, &bDebouncedState);
	if ((pinState != GPIO_PIN_RESET) || (bDebouncedState != true)) return false;
	
	// Force one cycle of debouncing.
	GPIODebounceDDI();
	
	// Remove voltage from pin. Debounce counter = 0.
	pinState = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN);
	GPIOGetDDI(GPIO_DDI_BUTTON_1, &bDebouncedState);
	if ((pinState != GPIO_PIN_RESET) || (bDebouncedState != true)) return false;
	
	// Force one cycle of debouncing.
	GPIODebounceDDI();
	
	// Remove voltage from pin. Debounce counter = 0 (min).
	pinState = HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN);
	GPIOGetDDI(GPIO_DDI_BUTTON_1, &bDebouncedState);
	if ((pinState != GPIO_PIN_RESET) || (bDebouncedState != false)) return false;
	
	return true;
}
#endif
