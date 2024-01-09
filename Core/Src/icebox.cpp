///
/// \file 		IceBoxx.c
/// \brief 		[Source file]
///
/// \author 	Xavier Boucher, eng. 5022396 - NOVO
///
////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include "IceBoxx.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "Framework.h"
#include "MCUMap.h"
#include "adc.h"
#include "gpio.h"
#include "LedDriver.h"
#include "tim.h"
#include "UTLookUpTable.h"
// #include "NoTouch.h"

// GPIO Pins Mapping

// Ice boxx mapping
//  Swap compressor and fan address for resolving TRIAC issues
#define ICEBOXX_GPIO_ICE_BOXX_COMPRESSOR_PORT (MCUMAP_DO_COMPRESSOR_PERIPH) ///< Mapping og the ice boxx compressor port (Old address: MCUMAP_DO_COMPRESSOR_PERIPH)
#define ICEBOXX_GPIO_ICE_BOXX_COMPRESSOR_PIN (MCUMAP_DO_COMPRESSOR_PIN)		///< Mapping og the ice boxx compressor pin (Old address: MCUMAP_DO_COMPRESSOR_PIN)

#define ICEBOXX_GPIO_PUMP_120V_OUT_PORT (MCUMAP_DO_PUMP_RECIR_PERIPH)
#define ICEBOXX_GPIO_PUMP_120V_OUT_PIN (MCUMAP_DO_PUMP_RECIR_PIN)

#define ICEBOXX_GPIO_ICE_BOXX_WATER_LEVEL_LED_PIN (MCUMAP_DO_LED_BLUE_PIN)
#define ICEBOXX_GPIO_ICE_BOXX_WATER_LEVEL_LED_PORT (MCUMAP_DO_LED_BLUE_PERIPH)

#define ICEBOXX_GPIO_ICE_BOXX_TEMPERATURE_AND_WATER_LEAK_LED_PIN (MCUMAP_DO_LED_YELLOW_PIN)
#define ICEBOXX_GPIO_ICE_BOXX_TEMPERATURE_AND_WATER_LEAK_LED_PORT (MCUMAP_DO_LED_YELLOW_PERIPH)

#define ICEBOXX_GPIO_ICE_BOXX_WATER_LEAK_LED_PIN (MCUMAP_DO_LED_GREEN_BI_PIN)
#define ICEBOXX_GPIO_ICE_BOXX_WATER_LEAK_LED_PORT (MCUMAP_DO_LED_GREEN_BI_PERIPH)

// #define ICEBOXX_GPIO_FAN_PIN													(MCUMAP_DO_COMPRESSOR_PIN)				///< Mapping of the ice boxx heater/fan pin (Old address: MCUMAP_DO_HEATBAND_PIN)
// #define ICEBOXX_GPIO_FAN_PORT													(MCUMAP_DO_COMPRESSOR_PERIPH)			///< Mapping of the ice boxx heater/fan port (Old address: MCUMAP_DO_HEATBAND_PERIPH)

// Water leak mapping
// #define ICEBOXX_GPIO_WATER_LEAK_DETECTION_BOTTOM								(GPIO_DDI_ICEBOXX_WATER_LEAK_BOTTOM)	///< Mapping of the water leak bottom detection
// #define ICEBOXX_GPIO_WATER_LEAK_DETECTION_TOP									(GPIO_DDI_ICEBOXX_WATER_LEAK_TOP)		///< Mapping of the water leak top detection

// Contrôle du temps de cycles de pompe
#define ICEBOXX_PUMP_TIME_ON (0)   ///< Time Pump is ON when Compressor is ON
#define ICEBOXX_PUMP_TIME_OFF (30) ///< Time Pump is OFF when Compressor is ON

// Cold water timeout
#define ICEBOXX_COLD_WATER_IR_WAIT_TIME_MS (3000)			///< Period for IR of cold water to be active in ms.
#define ICEBOXX_COLD_WATER_IR_ACTIVATION_TIMEOUT_MS (30000) ///< Period time IR water can be activated ins ms.

#define ICEBOXX_ADC_INPUT_ERROR_THRESHOLD_HIGH 4000 ///< ADC count value indicating a misconnection of the probe.
#define ICEBOXX_ADC_INPUT_ERROR_THRESHOLD_LOW 0		///< ADC count value indicating a misconnection of the probe.

// Temperature limits
#define ICEBOXX_TEMPERATURE_HIGH_C (55.0)	   // 55								///< High temperature threshold, in deg C.
#define ICEBOXX_TEMPERATURE_VERY_HIGH_C (65.0) // 65								///< Very high temperature threshold, in deg C.

// Potentiometers
#define ICEBOXX_POT_TEMP_AMBIANT_MIN_C (2.0) ///< Minimum adjustable temperature for the HOT water, in deg C.
#define ICEBOXX_POT_TEMP_AMBIANT_MAX_C (5.0) ///< Maximum adjustable temperature for the HOT water, in deg C.

#define ICEBOXX_POT_TEMP_COLD_WATER_MIN_C (2.0)		   ///< Minimum adjustable temperature for the COLD water, in deg C.
#define ICEBOXX_POT_TEMP_COLD_WATER_MAX_C (5.0)		   ///< Maximum adjustable temperature for the COLD water, in deg C.
#define ICEBOXX_TEMP_COLD_WATER_THRESHOLD_HYST_C (1.0) ///< Hysteresis threshold for COLD water control, in deg C.

#define ICEBOXX_TEMPERATURE_ACQUISITION_TIMER_MS (1000) ///< Delay of time for changing the value of cold temperature

#define ICEBOXX_POT_ADC_MIN_COUNT (0)	 ///< Minimum ADC raw value for the adjustable temperature ranges, in counts.
#define ICEBOXX_POT_ADC_MAX_COUNT (2047) ///< Maximum ADC raw value for the adjustable temperature ranges, in counts).

// Led status
#define ICEBOXX_TON_TOFF_TIME_MS 500 ///< Time led stays in a state when blinking
#define ICEBOXX_TON_TOFF_PAUSE_MS 0	 ///< Time led pause between blinking
#define ICEBOXX_NUMBER_BLINK 1		 ///< Number of time to blink before pause

#define ICEBOXX_COMPRESSOR_TIME_PROTECTION_MS 1000	// 180000									///< 3 min compressor time protection
#define ICEBOXX_COMPRESSOR_TEMP_REACHED_DELAY_MS 60 ///< 1 min before compressor stops after reaching target temperature

#define NB_CYCLES_SEQUENCING (1)			  ///< Number of refrigerating cycles in the sequence
#define NB_MAX_PUMP_CYCLES_SEQUENCE (3)		  ///< Maximal number of pump cycles during one refrigerating cycle during a sequence
#define ICEBOXX_SEQUENCE_DATA_ARRAY_SIZE (30) ///< Number of data in the memory array for sequencing
#define ICEBOXX_TEMPERATURE_OLD_DIFF_LIMIT (0.1)

// counter
int i = 0;

///
/// \enum	eIceBoxxWaterLeak_t
/// \brief	Enumeration of the possible states of water leak section.
///
typedef enum
{
	ICEBOXX_WATER_LEAK_STANDBY = 0,
	ICEBOXX_WATER_LEAKING,
} eIceBoxxWaterLeak_t;

///
/// \enum	eIceBoxxTemperature_t
/// \brief	Enumeration of the possible states of temperature section.
///
typedef enum
{
	ICEBOXX_TEMPERATURE_STANDBY = 0,
	ICEBOXX_TEMPERATURE_ABOVE_HIGH,
	ICEBOXX_TEMPERATURE_ABOVE_VERY_HIGH,
	ICEBOXX_TEMPERATURE_ABOVE_VERY_HIGH_ACTIVATION
} eIceBoxxTemperature_t;

///
/// \enum	eIceBoxxStateIcePoolWaterLevel_t
/// \brief	Enumeration of the possible states of the water level.
///
typedef enum
{
	ICEBOXX_ICE_POOL_WATER_LEVEL_STANDBY = 0, ///< Always the first state after reboot.
	ICEBOXX_ICE_POOL_WATER_LEVEL_LOW,		  ///< Low level of water state
	ICEBOXX_ICE_POOL_WATER_LEVEL_VERY_LOW	  ///< Very low level of water state
} eIceBoxxStateIcePoolWaterLevel_t;

///
/// \enum	eIceBoxxStateCompressor_t
/// \brief	Enumeration of the possible states of the compressor.
///
typedef enum
{
	ICEBOXX_ICE_COMPRESSOR_STANDBY = 0,
	ICEBOXX_ICE_COMPRESSOR_WORKING,
	ICEBOXX_ICE_COMPRESSOR_INACTIVE,
} eIceBoxxStateCompressor_t;

///
/// \enum	eIceBoxxStateCompressor_t
/// \brief	Enumeration of the possible states of the compressor.
///
typedef enum
{
	WATER_DISPENSOR_STANDBY = 0,
	WATER_DISPENSOR_NORMAL,
	WATER_DISPENSOR_COLD,
	WATER_DISPENSOR_BUBBLE,
	WATER_DISPENSOR_INACTIVE,
} eWaterDispensorState_t;

///
/// \enum	eIceBoxxStateCompressor_t
/// \brief	Enumeration of the possible states of the compressor.
///
typedef enum
{
	ICEBOXX_SEQUENCE_FINITE = 0,
	ICEBOXX_SEQUENCE_INFINITE
} eIceBoxxStateSequence;

///
/// \struct	oPumpsequence_t
/// \brief	Structure containing pump sequence
///
typedef struct
{
	eIceBoxxStateCompressor_t eSeqCompPreviousState;										  ///< State to see if the compressor just started.
	eIceBoxxStateSequence eSeqPumpState;													  ///< State to determine the type of sequence (finite or infinite)
	bool bPumpSeqFlag[(2 * NB_MAX_PUMP_CYCLES_SEQUENCE) + 1][NB_CYCLES_SEQUENCING];			  ///< Sequence flags. Each column represents the number of cycles which need sequencing
	bool bPumpStartFlag;																	  ///< Flag to stop the pump when at the end of its sequence during a refrigeration cycle
	bool bPumpHoldFlag;																		  ///< Flag to keep the pump at ON
	uint32_t u32PumpSeqDuration[(2 * NB_MAX_PUMP_CYCLES_SEQUENCE) + 1][NB_CYCLES_SEQUENCING]; ///< Sequence flags duration. Represents the duration of each sequence flag
	uint32_t indexSeq[2];																	  ///< Sequence index for choosing the right flag in in IceBoxxCompressorTask()
	float fTemperatureOld15Min[ICEBOXX_SEQUENCE_DATA_ARRAY_SIZE];							  ///< Variable that keeps an old temperature variable
	float fTemperatureOldMean;
	float fTemperatureOldDiff;
	float fTemperatureMax15;
	float fTemperatureMin15;
	uint8_t u8TemperatureOldArrayIndex;
} oPumpSequence_t;

///
/// \struct	oDDPsequence_t
/// \brief	Structure containing DDP sequence
///
typedef struct
{
	eIceBoxxStateCompressor_t eSeqCompPreviousState;										  ///< State to see if the compressor just started.
	eIceBoxxStateSequence eSeqPumpState;													  ///< State to determine the type of sequence (finite or infinite)
	bool bPumpSeqFlag[(2 * NB_MAX_PUMP_CYCLES_SEQUENCE) + 1][NB_CYCLES_SEQUENCING];			  ///< Sequence flags. Each column represents the number of cycles which need sequencing
	bool bPumpStartFlag;																	  ///< Flag to stop the pump when at the end of its sequence during a refrigeration cycle
	bool bPumpHoldFlag;																		  ///< Flag to keep the pump at ON
	uint32_t u32PumpSeqDuration[(2 * NB_MAX_PUMP_CYCLES_SEQUENCE) + 1][NB_CYCLES_SEQUENCING]; ///< Sequence flags duration. Represents the duration of each sequence flag
	uint32_t indexSeq[2];																	  ///< Sequence index for choosing the right flag in in IceBoxxCompressorTask()
	float fTemperatureOld15Min[ICEBOXX_SEQUENCE_DATA_ARRAY_SIZE];							  ///< Variable that keeps an old temperature variable
	float fTemperatureOldMean;
	float fTemperatureOldDiff;
	float fTemperatureMax15;
	float fTemperatureMin15;
	uint8_t u8TemperatureOldArrayIndex;
} oDDPSequence_t;

///
/// \struct	oIcePoolWaterLevel_t
/// \brief	Structure containing ice pool water level section
///
typedef struct
{
	eIceBoxxStateIcePoolWaterLevel_t eStateIcePoolWaterLevel; ///< Current diagnostics state.
	bool bLowLevelH2O;										  ///< H2O low level state
	bool bVeryLowLevelH2O;									  ///< H2O very low level state
} oIcePoolWaterLevel_t;

///
/// \struct	oWaterLeak_t
/// \brief	Structure containing water leak section
///
typedef struct
{
	eIceBoxxWaterLeak_t eStateWaterLeak; ///< Current diagnostics state.
} oWaterLeak_t;

///
/// \struct	oTemperature_t
/// \brief	Structure containing temperature section
///
typedef struct
{
	eIceBoxxTemperature_t eStateTemperature; ///< Current diagnostics state.
	float fHotSensor;						 ///< Hot temperature Reading
	float fColdSensor;						 ///< Cold water Sensor Reading
	float fAmbientTemperature;				 ///< Ambient temperature
} oTemperature_t;

///
/// \struct	oCompressor_t
/// \brief	Structure containing compressor section
///
typedef struct
{
	eIceBoxxStateCompressor_t eIceBoxxStateCompressor; ///< Current diagnostics state.

	uint32_t u32TimestampCompressorProtection; ///< Wait time on compressor protection

	bool bIsActivated; ///< Flag indicating if the pump is currently activated or not.
	bool bIsAllowToStart;
	uint32_t u32TimestampLastDeactivation; ///< Timestamp of the last deactivation, in ms.
} oCompressor_t;

///
/// \struct	oDispensor_t
/// \brief	Structure containing compressor section
///
typedef struct
{
	eWaterDispensorState_t eWaterDispensorState; ///< Current diagnostics state.

	uint32_t u32TimestampCompressorProtection; ///< Wait time on compressor protection

	bool bIsActivated; ///< Flag indicating if the pump is currently activated or not.
	bool bIsAllowToStart;

	bool bNormalButton;
	bool bColdButton;
	bool bBubbleButton;
	uint32_t u32TimestampLastDeactivation; ///< Timestamp of the last deactivation, in ms.
} oDispensor_t;

///
/// \struct	o_Pump_t
/// \brief	Structure containing Pump section
///
typedef struct
{
	bool bIsActivated;				   ///< Flag indicating if the pump is currently activated or not.
	bool bIsAllowToStart;			   ///< Flag indicating if the pump has the permission to start. (Useful at when errors)
	uint8_t u8RetryCounter;			   ///< Keep track of the number of retry done in the retry mechanism.
	uint32_t u32TimestampActivation;   ///< Timestamp of the last activation, in ms.
	uint32_t u32TimestampDeactivation; ///< Timestamp of the last deactivation, in ms.
	oPumpSequence_t g_oSequence;
} oPump_t;

///
/// \struct	o_DDP_t
/// \brief	Structure containing DDP section
///
typedef struct
{
	bool bIsActivated;				   ///< Flag indicating if the pump is currently activated or not.
	bool bIsAllowToStart;			   ///< Flag indicating if the pump has the permission to start. (Useful at when errors)
	uint8_t u8RetryCounter;			   ///< Keep track of the number of retry done in the retry mechanism.
	uint32_t u32TimestampActivation;   ///< Timestamp of the last activation, in ms.
	uint32_t u32TimestampDeactivation; ///< Timestamp of the last deactivation, in ms.
	oPumpSequence_t g_oSequence;
} oDDP_t;

///
/// \struct	o_DDP_t
/// \brief	Structure containing DDP section
///
typedef struct
{
	bool bNormalIsActivated;	///< Flag indicating if the pump is currently activated or not.
	bool bNormalIsAllowToStart; ///< Flag indicating if the pump has the permission to start. (Useful at when errors)
	bool bColdIsActivated;		///< Flag indicating if the pump is currently activated or not.
	bool bColdIsAllowToStart;	///< Flag indicating if the pump has the permission to start. (Useful at when errors)
	bool bBubbleIsActivated;	///< Flag indicating if the pump is currently activated or not.
	bool bBubbleIsAllowToStart; ///< Flag indicating if the pump has the permission to start. (Useful at when errors)
} oValves_t;

///
/// \struct	oIceBoxx_t
/// \brief	Structure containing ice boxx section
///
typedef struct
{
	oIcePoolWaterLevel_t g_oIcePoolWaterLevel;
	oWaterLeak_t g_oWaterLeak;
	oTemperature_t g_oTemperature;
	oCompressor_t g_oCompressor;
	oDispensor_t g_oDispensor;
	oPump_t g_oPump; ///< Pump peripheral of the Iceboxx model.
	oDDP_t g_oDDP;	 ///< DDP peripheral of the Iceboxx model.
	oValves_t g_oValves;
	oLedDriverLed_t g_oH2OWaterLevelLed;  ///< Led #6 Blue
	oLedDriverLed_t g_oH2OTemperatureLed; ///< Led #7 Yellow
	oLedDriverLed_t g_oH2OWaterLeakLed;	  ///< Led #1 Green

	uint8_t uCompteurPump;				  ///< Time between pump cycles
	uint32_t u32Compteur;				  ///< Counter to make 30 acquistion for an average
	uint32_t u32CounterSequence;		  ///< Sequence counter
	uint32_t u32CompteurDelai;			  ///< Time before stopping compressor
	uint32_t u32TimerCompressON;		  ///< Time compressor stayed ON
	uint32_t u32TimerCompressOFF;		  ///< Time compressor stayed OFF
	uint32_t u32TimerCompress;			  ///< Timer for compressor
	uint32_t u32TimestampWaterAquisition; ///< Timestamp used to delay the acquistion of cold temperature
	uint32_t u32TimestampDetectionSpeed;  ///< Speed of detection refresh

	float fTempColdDeactivComp; ///< Activation temperature of compressor
	float fTempColdMoyenne;		///< Acquisition of temperature each second
	float fTempColdResult;		///< Average updated every 30 seconds
	float fColdWaterTempThreshold;
	bool bColdWaterProbeOk;
	bool bColdWaterThresholdPotOk;
	bool bAmbiantProbeOk;
	bool bOverheated;			/// Flag to keep compressor INACTIVE after Overheat
	bool bColdWaterAcquisition; ///< Flag to allow the system to save cold temperature value

	bool bWaterLeakBottom;
	bool bWaterLeakTop;

	uint32_t u32Compteurrr; /// Compteur pour tests
	bool ModuleInError;		/// Flag pour gérer erreurs module (Water leak, overheat, water level)

} oIceBoxx_t;

////////////////////////////////////////////////////////////////////////////////
// Private functions
////////////////////////////////////////////////////////////////////////////////
bool IceBoxxIcePoolWaterLevelTask(void);
bool IceBoxxWaterLeakTask(void);
bool IceBoxxOverheatTask(void);
bool IceBoxxTemperatureUpdate(void);
bool IceBoxxCompressorTask(void);
bool WaterDispensorTask(void);
bool ButtonPressed(void);
bool IceBoxxCountersUpdateTask(void);
bool IceBoxxSequenceTask(void);
bool IceBoxxModuleInError(bool *pbState);
bool NoTouchTimeoutState(bool *noTouchState);

static void IceboxxActivateCompressor(bool bActivate, bool bForce);
static void IceboxxActivatePump(bool bActivate, bool bForce);
static void IceboxxActivateDDP(bool bActivate, bool bForce);
static void IceboxxActivateFan(bool bActivate);
static inline float IceboxxPotentiometer2Temp(uint16_t u16ADCValue, float i16MinTemp, float i16MaxTemp, uint16_t u16MinADC, uint16_t u16MaxADC);

////////////////////////////////////////////////////////////////////////////////
// Private variables
////////////////////////////////////////////////////////////////////////////////
oIceBoxx_t g_oIceBoxx; ///< Main Base Board object.
uint16_t u16ADCValue1; ///< Only for livewatch purpose (for updating temperature tables)
////////////////////////////////////////////////////////////////////////////////
/// \brief 		IceBoxxInit
/// \details	Initialize ice boxx section
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool IceBoxxInit(void)
{
	// Map leds
	if (LedDriverInitLed(&g_oIceBoxx.g_oH2OWaterLevelLed, ICEBOXX_GPIO_ICE_BOXX_WATER_LEVEL_LED_PORT, ICEBOXX_GPIO_ICE_BOXX_WATER_LEVEL_LED_PIN) == false)
		return false;
	if (LedDriverInitLed(&g_oIceBoxx.g_oH2OTemperatureLed, ICEBOXX_GPIO_ICE_BOXX_TEMPERATURE_AND_WATER_LEAK_LED_PORT, ICEBOXX_GPIO_ICE_BOXX_TEMPERATURE_AND_WATER_LEAK_LED_PIN) == false)
		return false;
	if (LedDriverInitLed(&g_oIceBoxx.g_oH2OWaterLeakLed, ICEBOXX_GPIO_ICE_BOXX_WATER_LEAK_LED_PORT, ICEBOXX_GPIO_ICE_BOXX_WATER_LEAK_LED_PIN) == false)
		return false;

	// Set leak/power led to ON at start
	if (LedDriverSetModeLed(&g_oIceBoxx.g_oH2OWaterLeakLed, LEDDRIVER_LED_MODE_ON) == false)
		return false;

	// Set to false 120V pump as it is not used
	HAL_GPIO_WritePin(ICEBOXX_GPIO_PUMP_120V_OUT_PORT, ICEBOXX_GPIO_PUMP_120V_OUT_PIN, GPIO_PIN_SET);

	g_oIceBoxx.g_oIcePoolWaterLevel.eStateIcePoolWaterLevel = ICEBOXX_ICE_POOL_WATER_LEVEL_STANDBY;
	g_oIceBoxx.g_oIcePoolWaterLevel.bLowLevelH2O = false;
	g_oIceBoxx.g_oIcePoolWaterLevel.bVeryLowLevelH2O = false;

	g_oIceBoxx.g_oTemperature.eStateTemperature = ICEBOXX_TEMPERATURE_STANDBY;
	g_oIceBoxx.g_oTemperature.fColdSensor = 0.0;
	g_oIceBoxx.g_oTemperature.fAmbientTemperature = 0.0;

	g_oIceBoxx.g_oCompressor.eIceBoxxStateCompressor = ICEBOXX_ICE_COMPRESSOR_STANDBY;
	g_oIceBoxx.g_oCompressor.bIsActivated = false;
	g_oIceBoxx.g_oCompressor.bIsAllowToStart = true;
	g_oIceBoxx.g_oCompressor.u32TimestampCompressorProtection = ICEBOXX_COMPRESSOR_TIME_PROTECTION_MS;

	g_oIceBoxx.g_oDispensor.eWaterDispensorState = WATER_DISPENSOR_STANDBY;
	g_oIceBoxx.g_oDispensor.bIsActivated = false;
	g_oIceBoxx.g_oDispensor.bIsAllowToStart = true;
	g_oIceBoxx.g_oDispensor.bNormalButton = false;
	g_oIceBoxx.g_oDispensor.bColdButton = false;
	g_oIceBoxx.g_oDispensor.bBubbleButton = false;

	g_oIceBoxx.g_oWaterLeak.eStateWaterLeak = ICEBOXX_WATER_LEAK_STANDBY;

	g_oIceBoxx.g_oPump.bIsActivated = true;
	g_oIceBoxx.g_oPump.bIsAllowToStart = true;
	g_oIceBoxx.g_oPump.u8RetryCounter = 0;
	g_oIceBoxx.g_oPump.u32TimestampActivation = 0;
	g_oIceBoxx.g_oPump.u32TimestampDeactivation = 0;
	g_oIceBoxx.u32TimestampDetectionSpeed = 0;

	g_oIceBoxx.g_oDDP.bIsActivated = false;
	g_oIceBoxx.g_oDDP.bIsAllowToStart = true;
	g_oIceBoxx.g_oDDP.u32TimestampActivation = 0;
	g_oIceBoxx.g_oDDP.u32TimestampDeactivation = 0;
	g_oIceBoxx.g_oDDP.u8RetryCounter = 0;

	g_oIceBoxx.g_oValves.bNormalIsActivated = 0;
	g_oIceBoxx.g_oValves.bColdIsActivated = 0;
	g_oIceBoxx.g_oValves.bBubbleIsActivated = 0;

	g_oIceBoxx.uCompteurPump = 0;
	g_oIceBoxx.u32Compteur = 0;
	g_oIceBoxx.u32CounterSequence = 0;
	g_oIceBoxx.u32CompteurDelai = 0;
	g_oIceBoxx.u32TimerCompressON = 0;
	g_oIceBoxx.u32TimerCompressOFF = 0;
	g_oIceBoxx.u32TimerCompress = 0;
	g_oIceBoxx.fColdWaterTempThreshold = 0.0;
	g_oIceBoxx.u32TimestampWaterAquisition = 0;
	g_oIceBoxx.fTempColdResult = 25.0;
	g_oIceBoxx.fTempColdMoyenne = 0.0;
	g_oIceBoxx.bAmbiantProbeOk = false;
	g_oIceBoxx.bOverheated = false;
	g_oIceBoxx.bColdWaterProbeOk = false;
	g_oIceBoxx.bColdWaterThresholdPotOk = false;
	g_oIceBoxx.bColdWaterAcquisition = false;

	g_oIceBoxx.bWaterLeakBottom = false;
	g_oIceBoxx.bWaterLeakTop = false;

	g_oIceBoxx.u32Compteurrr = 0;
	g_oIceBoxx.ModuleInError = false;

	g_oIceBoxx.g_oPump.g_oSequence.bPumpStartFlag = false;
	g_oIceBoxx.g_oPump.g_oSequence.bPumpHoldFlag = false;
	g_oIceBoxx.g_oPump.g_oSequence.u8TemperatureOldArrayIndex = 0;
	g_oIceBoxx.g_oPump.g_oSequence.fTemperatureOldDiff = 0.0;
	g_oIceBoxx.g_oPump.g_oSequence.fTemperatureMax15 = -20.0;
	g_oIceBoxx.g_oPump.g_oSequence.fTemperatureMin15 = 40.0;

	for (int indexArr = 0; indexArr < ICEBOXX_SEQUENCE_DATA_ARRAY_SIZE; indexArr++)
	{
		g_oIceBoxx.g_oPump.g_oSequence.fTemperatureOld15Min[indexArr] = 0.0;
	}

	for (int indexLine = 0; indexLine < 2 * NB_MAX_PUMP_CYCLES_SEQUENCE; indexLine++)
	{
		for (int indexCol = 0; indexCol < NB_MAX_PUMP_CYCLES_SEQUENCE; indexCol++)
		{
			g_oIceBoxx.g_oPump.g_oSequence.bPumpSeqFlag[indexLine][indexCol] = false;
			g_oIceBoxx.g_oPump.g_oSequence.u32PumpSeqDuration[indexLine][indexCol] = 0;
		}
	}

	g_oIceBoxx.g_oPump.g_oSequence.indexSeq[0] = 0;
	g_oIceBoxx.g_oPump.g_oSequence.indexSeq[1] = 0;
	g_oIceBoxx.g_oPump.g_oSequence.eSeqCompPreviousState = ICEBOXX_ICE_COMPRESSOR_STANDBY;

	// Modify sequence here.
	// Each line in the array g_oIceBoxx.g_oPump.g_oSequence.bPumpSeqFlag is a pump state in a cycle
	// Each column is a new cycle.
	// The array g_oIceBoxx.g_oPump.g_oSequence.fPumpSeqDuration is the duration of each pump state.

	// As an example, the following sequence represents the pump running twice for two minutes each time
	// with a 1 minutes delay between each pump cycle for the first refrigerating cycle only
	// For the time array, time is expressed in ms and the state array is in boolean
	// Each time the sequence index is changed, the sequence counter is reset
	/*g_oIceBoxx.g_oPump.g_oSequence.bPumpSeqFlag[0][0] = true;
	g_oIceBoxx.g_oPump.g_oSequence.u32PumpSeqDuration[0][0] = 120;

	g_oIceBoxx.g_oPump.g_oSequence.bPumpSeqFlag[1][0] = false;
	g_oIceBoxx.g_oPump.g_oSequence.u32PumpSeqDuration[1][0] = 60;

	g_oIceBoxx.g_oPump.g_oSequence.bPumpSeqFlag[2][0] = true;
	g_oIceBoxx.g_oPump.g_oSequence.u32PumpSeqDuration[2][0] = 120;*/

	// Sequence type
	g_oIceBoxx.g_oPump.g_oSequence.eSeqPumpState = ICEBOXX_SEQUENCE_INFINITE;

	// If sequence if finite, determine the sequence here
	if (g_oIceBoxx.g_oPump.g_oSequence.eSeqPumpState == ICEBOXX_SEQUENCE_FINITE)
	{
		g_oIceBoxx.g_oPump.g_oSequence.bPumpSeqFlag[0][0] = false;
		g_oIceBoxx.g_oPump.g_oSequence.u32PumpSeqDuration[0][0] = 900;

		g_oIceBoxx.g_oPump.g_oSequence.bPumpSeqFlag[1][0] = true;
		g_oIceBoxx.g_oPump.g_oSequence.u32PumpSeqDuration[1][0] = 900;

		g_oIceBoxx.g_oPump.g_oSequence.bPumpSeqFlag[2][0] = false;
		g_oIceBoxx.g_oPump.g_oSequence.u32PumpSeqDuration[2][0] = 900;

		g_oIceBoxx.g_oPump.g_oSequence.bPumpSeqFlag[3][0] = true;
		g_oIceBoxx.g_oPump.g_oSequence.u32PumpSeqDuration[3][0] = 900;

		g_oIceBoxx.g_oPump.g_oSequence.bPumpSeqFlag[4][0] = false;
		g_oIceBoxx.g_oPump.g_oSequence.u32PumpSeqDuration[4][0] = 2400;

		g_oIceBoxx.g_oPump.g_oSequence.bPumpSeqFlag[5][0] = true;
		g_oIceBoxx.g_oPump.g_oSequence.u32PumpSeqDuration[5][0] = 900;
	}

	// If the sequence is infinite, alternate between only two pump states
	if (g_oIceBoxx.g_oPump.g_oSequence.eSeqPumpState == ICEBOXX_SEQUENCE_INFINITE)
	{
		g_oIceBoxx.g_oPump.g_oSequence.bPumpSeqFlag[0][0] = false;
		g_oIceBoxx.g_oPump.g_oSequence.u32PumpSeqDuration[0][0] = 900;

		g_oIceBoxx.g_oPump.g_oSequence.bPumpSeqFlag[1][0] = true;
		g_oIceBoxx.g_oPump.g_oSequence.u32PumpSeqDuration[1][0] = 900;
	}

	g_oIceBoxx.g_oTemperature.eStateTemperature = ICEBOXX_TEMPERATURE_STANDBY;

	IceboxxActivateCompressor(false, true);
	IceboxxActivatePump(true, true);
	IceboxxActivateDDP(true, false);
	// Set FAN to OFF
	IceboxxActivateFan(false);
	// HAL_GPIO_WritePin(ICEBOXX_GPIO_FAN_PORT, ICEBOXX_GPIO_FAN_PIN, GPIO_PIN_SET);

	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		IceBoxxTask
/// \details	Execution of ice boxx task section
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool IceBoxxTask(void)
{
	/*
	 * The order of tasks that needs to be done goes as follows:
	 * 1. Update the LED depending of what happened during the last call of IceBoxxTask(), like leaks or low water level
	 * 2. Update counters. Instead of having counters in every functions, they will all be updated at the same time.
	 *    It is not a issue to do so because all the counter dependable updates need to count in milliseconds or seconds,
	 *    which is a lot slower than the MCU frequency.
	 * 3. Create and update sequences. The sequences will be used to have a better control over the water temperature without
	 * 	  having too much ice too quickly.
	 * 4. Check the water level and raise flags if needed to stop compressor
	 * 5. Check the ambient temperature to avoid overheating and raise flags if needed to stop compressor
	 * 6. Check the water temperature to determine if the compressor and/or the pump should be working or not
	 * 7. Check for leaks and raise flags to stop compressor if needed
	 * 8. Compressor and pump tasks. Depending of what happened in the other tasks earlier, the compressor and/or the
	 * 	  pump start/stop.
	 */

	// Update LED tasks
	if (LedDriverUpdateLed(&g_oIceBoxx.g_oH2OWaterLevelLed) == false)
		return false; // Blue LED
	if (LedDriverUpdateLed(&g_oIceBoxx.g_oH2OTemperatureLed) == false)
		return false; // Yellow LED
	if (LedDriverUpdateLed(&g_oIceBoxx.g_oH2OWaterLeakLed) == false)
		return false; // Green LED

	// Update counters
	if (IceBoxxCountersUpdateTask() == false)
		return false;
	// Create and update sequences
	if (IceBoxxSequenceTask() == false)
		return false;
	// Check if compressor must be stopped due to water level in ice pool
	if (IceBoxxIcePoolWaterLevelTask() == false)
		return false;
	// Check for overheating
	if (IceBoxxOverheatTask() == false)
		return false;
	// Check the water temperature
	if (IceBoxxTemperatureUpdate() == false)
		return false;
	// Check water leak
	if (IceBoxxWaterLeakTask() == false)
		return false;
	// Check compressor usage
	if (IceBoxxCompressorTask() == false)
		return false;
	// Check Dispensor usage
	if (WaterDispensorTask() == false)
		return false;

	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		IceBoxxCountersUpdateTask
/// \details	Update counters
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool IceBoxxCountersUpdateTask(void)
{
	// Counter limit. Set to 1 hour
	uint16_t u32CounterLimit = 3600;

	// Check if one second has passed since the last counter update
	if (FrameworkTimeIsDue(g_oIceBoxx.u32TimestampWaterAquisition, ICEBOXX_TEMPERATURE_ACQUISITION_TIMER_MS) == true)
	{
		// Update the time stamp for the next check
		g_oIceBoxx.u32TimestampWaterAquisition = FrameworkGetTime();

		// Update base counter. It will reset automatically every hour to avoid potential overflow
		g_oIceBoxx.u32Compteur = (uint32_t)((++g_oIceBoxx.u32Compteur) % u32CounterLimit);

		// Update sequence counter. To avoid potential overflow, it will reset after 10 hours if not changed.
		if ((g_oIceBoxx.g_oCompressor.eIceBoxxStateCompressor == ICEBOXX_ICE_COMPRESSOR_WORKING) && (g_oIceBoxx.g_oPump.bIsActivated == true))
			g_oIceBoxx.u32CounterSequence++;

		// Depending if the compressor is working or not, update adequate counter. To make the counter update, the compressor must have permission to start
		if ((g_oIceBoxx.g_oCompressor.bIsActivated == true) && (g_oIceBoxx.g_oCompressor.bIsAllowToStart == true))
			g_oIceBoxx.u32TimerCompressON++;
		if ((g_oIceBoxx.g_oCompressor.bIsActivated == false) && (g_oIceBoxx.g_oCompressor.bIsAllowToStart == true))
			g_oIceBoxx.u32TimerCompressOFF++;

		// Update pump counter. To make the counter update, the pump must have permission to start
		if (g_oIceBoxx.g_oPump.bIsAllowToStart == true)
			g_oIceBoxx.uCompteurPump++;

		// Update delay counter when the temperature has reached the threshold
		if (g_oIceBoxx.fTempColdResult < (g_oIceBoxx.fColdWaterTempThreshold - ICEBOXX_TEMP_COLD_WATER_THRESHOLD_HYST_C))
			g_oIceBoxx.u32CompteurDelai++;

		// Raise bColdWaterAcquisition flag to allow saving cold water data
		g_oIceBoxx.bColdWaterAcquisition = true;
	}

	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		IceBoxxSequenceTask
/// \details	Create or update sequences
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool IceBoxxSequenceTask(void)
{
	float upperLimitPump = 2.0;

	/*
	 * To do: Update temperature tables
	 */

	if (g_oIceBoxx.g_oPump.g_oSequence.fTemperatureOld15Min[ICEBOXX_SEQUENCE_DATA_ARRAY_SIZE - 1] != 0.0)
	{
		// Temporary maximum and minimum temperature values
		float tempMinTempo = 40.0;
		float tempMaxTempo = -20.0;

		// Flags that, if true, has found in the historic array the min/max value
		bool findRealMin = false;
		bool findRealMax = false;

		// Checkup of each value in the historic array
		for (int index = 0; index < ICEBOXX_SEQUENCE_DATA_ARRAY_SIZE; index++)
		{
			// Check if max value is in the historic array
			if (g_oIceBoxx.g_oPump.g_oSequence.fTemperatureOld15Min[index] == g_oIceBoxx.g_oPump.g_oSequence.fTemperatureMax15)
			{
				findRealMax = true;
			}

			// Check if min value is in the historic array
			if (g_oIceBoxx.g_oPump.g_oSequence.fTemperatureOld15Min[index] == g_oIceBoxx.g_oPump.g_oSequence.fTemperatureMin15)
			{
				findRealMin = true;
			}

			// Update temporary max value if lower
			if (g_oIceBoxx.g_oPump.g_oSequence.fTemperatureOld15Min[index] > tempMaxTempo)
			{
				tempMaxTempo = g_oIceBoxx.g_oPump.g_oSequence.fTemperatureOld15Min[index];
			}

			// Update temporary min value if higher
			if (g_oIceBoxx.g_oPump.g_oSequence.fTemperatureOld15Min[index] < tempMinTempo)
			{
				tempMinTempo = g_oIceBoxx.g_oPump.g_oSequence.fTemperatureOld15Min[index];
			}
		}

		// If flags true or temporary value real min/max, update min/max value
		if ((findRealMax == false) || (tempMaxTempo > g_oIceBoxx.g_oPump.g_oSequence.fTemperatureMax15))
		{
			g_oIceBoxx.g_oPump.g_oSequence.fTemperatureMax15 = tempMaxTempo;
		}

		if ((findRealMin == false) || (tempMinTempo < g_oIceBoxx.g_oPump.g_oSequence.fTemperatureMin15))
		{
			g_oIceBoxx.g_oPump.g_oSequence.fTemperatureMin15 = tempMinTempo;
		}

		// Calculate differential value of historic array (max - min)
		g_oIceBoxx.g_oPump.g_oSequence.fTemperatureOldDiff = g_oIceBoxx.g_oPump.g_oSequence.fTemperatureMax15 - g_oIceBoxx.g_oPump.g_oSequence.fTemperatureMin15;

		// If differential value lower than limit, start pump for 5 minutes
		if (g_oIceBoxx.g_oPump.g_oSequence.fTemperatureOldDiff <= ICEBOXX_TEMPERATURE_OLD_DIFF_LIMIT)
		{
			g_oIceBoxx.g_oPump.g_oSequence.bPumpStartFlag = true;
		}
		else
		{
			if (FrameworkTimeIsDue(g_oIceBoxx.g_oPump.u32TimestampActivation, 300000) == true)
			{
				g_oIceBoxx.g_oPump.g_oSequence.bPumpStartFlag = false;
			}
		}
	}

	// If temperature is higher than limit, start pump
	if (g_oIceBoxx.fTempColdResult > (g_oIceBoxx.fColdWaterTempThreshold + upperLimitPump))
	{
		// The temperature limit is not reached. Start pump.
		g_oIceBoxx.g_oPump.g_oSequence.bPumpHoldFlag = true;
	}
	else
	{
		g_oIceBoxx.g_oPump.g_oSequence.bPumpHoldFlag = false;
	}

	// Change the compressor previous state only if it is not the same as the compressor current state
	if (g_oIceBoxx.g_oCompressor.eIceBoxxStateCompressor != g_oIceBoxx.g_oPump.g_oSequence.eSeqCompPreviousState)
	{
		g_oIceBoxx.g_oPump.g_oSequence.eSeqCompPreviousState = g_oIceBoxx.g_oCompressor.eIceBoxxStateCompressor;
	}

	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		IceBoxxIcePoolWaterLevelTask
/// \details	Execution of ice water pool level task section
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool IceBoxxIcePoolWaterLevelTask(void)
{
	// Get ice pool water level detection debounced states
	// if (GPIOGetDDI(GPIO_DDI_ICEBOXX_LOW_WATER, &g_oIceBoxx.g_oIcePoolWaterLevel.bLowLevelH2O) == false) return false;
	// if (GPIOGetDDI(GPIO_DDI_ICEBOXX_VERY_LOW_WATER, &g_oIceBoxx.g_oIcePoolWaterLevel.bVeryLowLevelH2O) == false) return false;

	g_oIceBoxx.g_oIcePoolWaterLevel.eStateIcePoolWaterLevel = ICEBOXX_ICE_POOL_WATER_LEVEL_STANDBY;

	switch (g_oIceBoxx.g_oIcePoolWaterLevel.eStateIcePoolWaterLevel)
	{
	case (ICEBOXX_ICE_POOL_WATER_LEVEL_STANDBY):
	{
		// Update led blue OFF
		if (LedDriverSetModeLed(&g_oIceBoxx.g_oH2OWaterLevelLed, LEDDRIVER_LED_MODE_OFF) == false)
			return false;

		// Update led green ON
		if (LedDriverSetModeLed(&g_oIceBoxx.g_oH2OWaterLeakLed, LEDDRIVER_LED_MODE_ON) == false)
			return false;

		// Check if level is low
		if (g_oIceBoxx.g_oIcePoolWaterLevel.bLowLevelH2O == true)
		{
			// Change state to LOW
			g_oIceBoxx.g_oIcePoolWaterLevel.eStateIcePoolWaterLevel = ICEBOXX_ICE_POOL_WATER_LEVEL_LOW;
		}
		// Check if level is very low
		if (g_oIceBoxx.g_oIcePoolWaterLevel.bVeryLowLevelH2O == true)
		{
			// change state to VERY LOW
			g_oIceBoxx.g_oIcePoolWaterLevel.eStateIcePoolWaterLevel = ICEBOXX_ICE_POOL_WATER_LEVEL_VERY_LOW;
		}
	}
	break;

	case (ICEBOXX_ICE_POOL_WATER_LEVEL_LOW):
	{
		// Update led blue ON
		if (LedDriverSetModeLed(&g_oIceBoxx.g_oH2OWaterLevelLed, LEDDRIVER_LED_MODE_ON) == false)
			return false;

		// Check if level is very low
		if (g_oIceBoxx.g_oIcePoolWaterLevel.bVeryLowLevelH2O == true)
		{
			// Change state to VERY LOW
			g_oIceBoxx.g_oIcePoolWaterLevel.eStateIcePoolWaterLevel = ICEBOXX_ICE_POOL_WATER_LEVEL_VERY_LOW;
		}
		// Check if level is ok
		if (g_oIceBoxx.g_oIcePoolWaterLevel.bLowLevelH2O == false)
		{
			// Change state to STANDBY
			g_oIceBoxx.g_oIcePoolWaterLevel.eStateIcePoolWaterLevel = ICEBOXX_ICE_POOL_WATER_LEVEL_STANDBY;
		}
	}
	break;

	case (ICEBOXX_ICE_POOL_WATER_LEVEL_VERY_LOW):
	{
		// Update led blue BLINK
		if (LedDriverConfigLedContinuousBlinkPattern(&g_oIceBoxx.g_oH2OWaterLevelLed,
													 ICEBOXX_NUMBER_BLINK, ICEBOXX_TON_TOFF_TIME_MS, ICEBOXX_TON_TOFF_TIME_MS, ICEBOXX_TON_TOFF_PAUSE_MS, false) == false)
			return false;
		if (LedDriverSetModeLed(&g_oIceBoxx.g_oH2OWaterLevelLed, LEDDRIVER_LED_MODE_BLINK) == false)
			return false;
		// Update led green BLINK
		if (LedDriverConfigLedContinuousBlinkPattern(&g_oIceBoxx.g_oH2OWaterLeakLed,
													 ICEBOXX_NUMBER_BLINK, ICEBOXX_TON_TOFF_TIME_MS, ICEBOXX_TON_TOFF_TIME_MS, ICEBOXX_TON_TOFF_PAUSE_MS, false) == false)
			return false;
		if (LedDriverSetModeLed(&g_oIceBoxx.g_oH2OWaterLeakLed, LEDDRIVER_LED_MODE_BLINK) == false)
			return false;

		g_oIceBoxx.ModuleInError = true;
		g_oIceBoxx.g_oCompressor.eIceBoxxStateCompressor = ICEBOXX_ICE_COMPRESSOR_INACTIVE;

		// Check if water level is ok
		if (g_oIceBoxx.g_oIcePoolWaterLevel.bVeryLowLevelH2O == false)
		{
			// Change state to STANDBY
			g_oIceBoxx.ModuleInError = false;
			g_oIceBoxx.g_oIcePoolWaterLevel.eStateIcePoolWaterLevel = ICEBOXX_ICE_POOL_WATER_LEVEL_STANDBY;
		}
		// Check if water level is low
		else if (g_oIceBoxx.g_oIcePoolWaterLevel.bVeryLowLevelH2O == false)
		{
			// Change state to LOW
			g_oIceBoxx.ModuleInError = false;
			g_oIceBoxx.g_oIcePoolWaterLevel.eStateIcePoolWaterLevel = ICEBOXX_ICE_POOL_WATER_LEVEL_LOW;
		}
	}
	break;

	default:
	{
		return false;
	}
	break;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		IceBoxxIcePoolWaterLeakTask
/// \details	Execution of water leak task section
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool IceBoxxWaterLeakTask(void)
{
	// bool bWaterLeakBottom = false; 	// false
	// bool bWaterLeakTop = false;		// false

	// Get water leak debounced states
	// if (GPIOGetDDI(ICEBOXX_GPIO_WATER_LEAK_DETECTION_BOTTOM, &g_oIceBoxx.bWaterLeakBottom) == false) return false;
	// if (GPIOGetDDI(ICEBOXX_GPIO_WATER_LEAK_DETECTION_TOP, &g_oIceBoxx.bWaterLeakTop) == false) return false;
	// *******************************************
	switch (g_oIceBoxx.g_oWaterLeak.eStateWaterLeak)
	{
	case (ICEBOXX_WATER_LEAK_STANDBY):
	{
		// Check if a water leak is detected
		if ((g_oIceBoxx.bWaterLeakBottom == true) || (g_oIceBoxx.bWaterLeakTop == true))
		{
			// Activate buzzer
			MX_BUZZER_PWM_START();

			// Change state to LEAKING
			g_oIceBoxx.g_oWaterLeak.eStateWaterLeak = ICEBOXX_WATER_LEAKING;
		}
	}
	break;

	case (ICEBOXX_WATER_LEAKING):
	{
		// Water is leaking.
		// Update led blue flashing
		if (LedDriverConfigLedContinuousBlinkPattern(&g_oIceBoxx.g_oH2OWaterLevelLed,
													 ICEBOXX_NUMBER_BLINK, ICEBOXX_TON_TOFF_TIME_MS, ICEBOXX_TON_TOFF_TIME_MS, ICEBOXX_TON_TOFF_PAUSE_MS, false) == false)
			return false;
		if (LedDriverSetModeLed(&g_oIceBoxx.g_oH2OWaterLevelLed, LEDDRIVER_LED_MODE_BLINK) == false)
			return false;

		// Update led green flashing
		if (LedDriverConfigLedContinuousBlinkPattern(&g_oIceBoxx.g_oH2OWaterLeakLed,
													 ICEBOXX_NUMBER_BLINK, ICEBOXX_TON_TOFF_TIME_MS, ICEBOXX_TON_TOFF_TIME_MS, ICEBOXX_TON_TOFF_PAUSE_MS, false) == false)
			return false;
		if (LedDriverSetModeLed(&g_oIceBoxx.g_oH2OWaterLeakLed, LEDDRIVER_LED_MODE_BLINK) == false)
			return false;

		// Update led yellow flashing
		if (LedDriverConfigLedContinuousBlinkPattern(&g_oIceBoxx.g_oH2OTemperatureLed,
													 ICEBOXX_NUMBER_BLINK, ICEBOXX_TON_TOFF_TIME_MS, ICEBOXX_TON_TOFF_TIME_MS, ICEBOXX_TON_TOFF_PAUSE_MS, false) == false)
			return false;
		if (LedDriverSetModeLed(&g_oIceBoxx.g_oH2OTemperatureLed, LEDDRIVER_LED_MODE_BLINK) == false)
			return false;

		// Raise the module error flag and change the compressor state to inactive
		g_oIceBoxx.ModuleInError = true;
		g_oIceBoxx.g_oCompressor.eIceBoxxStateCompressor = ICEBOXX_ICE_COMPRESSOR_INACTIVE;

		// Cannot get out of this state. A manual reset is required.
	}
	break;

	default:
	{
		return false;
	}
	break;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		IceBoxxOverheatUpdate
/// \details	Execution of ambiant temperature task section
/// \private
///
/// \param		Update air sensor data and control Compressor states
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool IceBoxxOverheatTask(void)
{
	uint16_t u16ADCValue2 = 0; // Ambiant Sensor
	uint16_t u16TemperatureCount2 = 0;
	float hTemperature = 0.0; // Ambiant Sensor temperature
	FrameworkModelID_e modelID = FRAMEWORK_MODEL_ID_UNKNOWN;

	// *********************** Gestion Capteur Température Ambiante (J7) ************************** //
	//
	//
	// Lookup table function changed
	//
	//
	//
	// Get average analog input from both sensors
	if (HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_HOT, &u16ADCValue2) == false)
		return false;

	// Get temperature
	if (UTLookUpTableGetTemperratureC(HAL_ADC_AI_TEMP_WATER_HOT, &hTemperature, &u16TemperatureCount2) == false)
		return false;
	g_oIceBoxx.g_oTemperature.fAmbientTemperature = hTemperature;

	// Activate fan if compressor is working
	if (g_oIceBoxx.g_oCompressor.eIceBoxxStateCompressor == ICEBOXX_ICE_COMPRESSOR_WORKING)
	{
		// Activate FAN (median and full board only)
		if (modelID != FRAMEWORK_MODEL_ID_BASE)
		{
			IceboxxActivateFan(true);
			// AL_GPIO_WritePin(ICEBOXX_GPIO_FAN_PORT, ICEBOXX_GPIO_FAN_PIN, GPIO_PIN_RESET);
		}
	}

	// Validate water probes behavior.
	if (u16ADCValue2 >= ICEBOXX_ADC_INPUT_ERROR_THRESHOLD_HIGH)
	{
		g_oIceBoxx.bAmbiantProbeOk = false;
	}
	else
	{
		g_oIceBoxx.bAmbiantProbeOk = true;
	}

	switch (g_oIceBoxx.g_oTemperature.eStateTemperature)
	{
	case (ICEBOXX_TEMPERATURE_STANDBY):
	{
		if (g_oIceBoxx.g_oCompressor.eIceBoxxStateCompressor != ICEBOXX_ICE_COMPRESSOR_WORKING)
		{
			// Deactivate FAN (median and full board only)
			if (modelID != FRAMEWORK_MODEL_ID_BASE)
			{
				IceboxxActivateFan(false);
				// HAL_GPIO_WritePin(ICEBOXX_GPIO_FAN_PORT, ICEBOXX_GPIO_FAN_PIN, GPIO_PIN_SET);
			}
		}
		// Check if temperature is high
		if (g_oIceBoxx.g_oTemperature.fAmbientTemperature > ICEBOXX_TEMPERATURE_HIGH_C)
		{
			// Activate FAN (median and full board only)
			if (modelID != FRAMEWORK_MODEL_ID_BASE)
			{
				IceboxxActivateFan(true);
				// HAL_GPIO_WritePin(ICEBOXX_GPIO_FAN_PORT, ICEBOXX_GPIO_FAN_PIN, GPIO_PIN_RESET);
			}
			// Change state to very high activation
			g_oIceBoxx.g_oTemperature.eStateTemperature = ICEBOXX_TEMPERATURE_ABOVE_HIGH;
		}
		// Check if temperature is very high
		else if (g_oIceBoxx.g_oTemperature.fAmbientTemperature > ICEBOXX_TEMPERATURE_VERY_HIGH_C)
		{
			// Change state to very high activation
			g_oIceBoxx.g_oTemperature.eStateTemperature = ICEBOXX_TEMPERATURE_ABOVE_VERY_HIGH_ACTIVATION;
			g_oIceBoxx.g_oCompressor.eIceBoxxStateCompressor = ICEBOXX_ICE_COMPRESSOR_INACTIVE;
		}
	}
	break;

	case (ICEBOXX_TEMPERATURE_ABOVE_HIGH):
	{
		// Update led yellow on
		if (LedDriverSetModeLed(&g_oIceBoxx.g_oH2OTemperatureLed, LEDDRIVER_LED_MODE_ON) == false)
			return false;

		// Check if temperature is very high
		if (g_oIceBoxx.g_oTemperature.fAmbientTemperature > ICEBOXX_TEMPERATURE_VERY_HIGH_C)
		{
			// Chagne state to very high activation
			g_oIceBoxx.g_oTemperature.eStateTemperature = ICEBOXX_TEMPERATURE_ABOVE_VERY_HIGH_ACTIVATION;
		}
		// Check if temperature is not high
		else if (g_oIceBoxx.g_oTemperature.fAmbientTemperature < ICEBOXX_TEMPERATURE_HIGH_C)
		{
			// Deactivate FAN (median and full board only)
			if (modelID != FRAMEWORK_MODEL_ID_BASE)
			{
				IceboxxActivateFan(false);
				// HAL_GPIO_WritePin(ICEBOXX_GPIO_FAN_PORT, ICEBOXX_GPIO_FAN_PIN, GPIO_PIN_SET);
			}

			// Update led yellow off
			if (LedDriverSetModeLed(&g_oIceBoxx.g_oH2OTemperatureLed, LEDDRIVER_LED_MODE_OFF) == false)
				return false;

			// Change state to standby
			g_oIceBoxx.g_oTemperature.eStateTemperature = ICEBOXX_TEMPERATURE_STANDBY;
		}
	}
	break;

	case (ICEBOXX_TEMPERATURE_ABOVE_VERY_HIGH):
	{
		// Update led green flashing
		if (LedDriverConfigLedContinuousBlinkPattern(&g_oIceBoxx.g_oH2OWaterLeakLed,
													 ICEBOXX_NUMBER_BLINK, ICEBOXX_TON_TOFF_TIME_MS, ICEBOXX_TON_TOFF_TIME_MS, ICEBOXX_TON_TOFF_PAUSE_MS, false) == false)
			return false;
		if (LedDriverSetModeLed(&g_oIceBoxx.g_oH2OWaterLeakLed, LEDDRIVER_LED_MODE_BLINK) == false)
			return false;

		// Update led yellow flashing
		if (LedDriverConfigLedContinuousBlinkPattern(&g_oIceBoxx.g_oH2OTemperatureLed,
													 ICEBOXX_NUMBER_BLINK, ICEBOXX_TON_TOFF_TIME_MS, ICEBOXX_TON_TOFF_TIME_MS, ICEBOXX_TON_TOFF_PAUSE_MS, false) == false)
			return false;
		if (LedDriverSetModeLed(&g_oIceBoxx.g_oH2OTemperatureLed, LEDDRIVER_LED_MODE_BLINK) == false)
			return false;

		// Manual reset TBD
	}
	break;

	case (ICEBOXX_TEMPERATURE_ABOVE_VERY_HIGH_ACTIVATION):
	{
		// Activate FAN (median and full board only)
		if (modelID != FRAMEWORK_MODEL_ID_BASE)
		{
			IceboxxActivateFan(true);
			// HAL_GPIO_WritePin(ICEBOXX_GPIO_FAN_PORT, ICEBOXX_GPIO_FAN_PIN, GPIO_PIN_RESET);
		}
		// Change state to temperature very high
		g_oIceBoxx.ModuleInError = true;
		g_oIceBoxx.bOverheated = true;
		g_oIceBoxx.g_oTemperature.eStateTemperature = ICEBOXX_TEMPERATURE_ABOVE_VERY_HIGH;
	}
	break;

	default:
	{
		return false;
	}
	break;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		IceBoxxTemperatureUpdate
/// \details	Execution of ice temperature task section
/// \private
///
/// \param		Update water sensor data and control Compressor states
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool IceBoxxTemperatureUpdate(void)
{
	uint16_t u16TemperatureCount = 0;
	// uint16_t	u16ADCValue1 		= 0;		// Cold Sensor
	float fTemperature = 0.0; // Cold Sensor temperature
	FrameworkModelID_e modelID = FRAMEWORK_MODEL_ID_UNKNOWN;

	//*********************** Gestion Capteur Froid (J11) **************************************//
	// Get average analog input from sensor
	if (HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_COLD_1, &u16ADCValue1) == false)
		return false;

	// Perform a lookup with the tables to find value
	if (UTLookUpTableGetTemperratureC(HAL_ADC_AI_TEMP_WATER_COLD_1, &fTemperature, &u16TemperatureCount) == false)
		return false;
	// g_oIceBoxx.g_oTemperature.fColdSensor = fTemperature;
	g_oIceBoxx.g_oTemperature.fColdSensor = -0.022045293 * u16ADCValue1 + 71.316522510;

	// Get board type
	if (FrameworkGetModelID(&modelID) == false)
		return false;

	// If bColdWaterAcquisition flag is raised, update fTempColdMoyenne value and put flag to false
	if (g_oIceBoxx.bColdWaterAcquisition == true)
	{
		g_oIceBoxx.bColdWaterAcquisition = false;
		g_oIceBoxx.fTempColdMoyenne = g_oIceBoxx.fTempColdMoyenne + fTemperature;

		// Calculate average every 30 seconds and reset fTempColdMoyenne value
		if ((g_oIceBoxx.u32Compteur % 30) == 0)
		{
			g_oIceBoxx.fTempColdResult = g_oIceBoxx.fTempColdMoyenne / 30;
			g_oIceBoxx.fTempColdMoyenne = 0;
			g_oIceBoxx.g_oPump.g_oSequence.fTemperatureOld15Min[g_oIceBoxx.g_oPump.g_oSequence.u8TemperatureOldArrayIndex] = g_oIceBoxx.fTempColdResult;
			g_oIceBoxx.g_oPump.g_oSequence.u8TemperatureOldArrayIndex = (g_oIceBoxx.g_oPump.g_oSequence.u8TemperatureOldArrayIndex + 1) % ICEBOXX_SEQUENCE_DATA_ARRAY_SIZE;
		}
	}

	// Validate water probes behavior.
	if (u16ADCValue1 >= ICEBOXX_ADC_INPUT_ERROR_THRESHOLD_HIGH)
	{
		g_oIceBoxx.bColdWaterProbeOk = false;
	}
	else
	{
		g_oIceBoxx.bColdWaterProbeOk = true;
	}

	// Water temperature thresholds.
	if (HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_COLD_ADJUST, &u16ADCValue1) == false)
		return false;
	g_oIceBoxx.fColdWaterTempThreshold = IceboxxPotentiometer2Temp(u16ADCValue1, ICEBOXX_POT_TEMP_COLD_WATER_MIN_C, ICEBOXX_POT_TEMP_COLD_WATER_MAX_C,
																   ICEBOXX_POT_ADC_MIN_COUNT, ICEBOXX_POT_ADC_MAX_COUNT);
	// Validate water threshold input behavior.
	if (u16ADCValue1 >= ICEBOXX_ADC_INPUT_ERROR_THRESHOLD_HIGH)
	{
		g_oIceBoxx.bColdWaterThresholdPotOk = false;
	}
	else
	{
		g_oIceBoxx.bColdWaterThresholdPotOk = true;
	}

	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		IceBoxxCompressorTask
/// \details	Execution of compressor task section
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool IceBoxxCompressorTask(void)
{
	FrameworkModelID_e modelID = FRAMEWORK_MODEL_ID_UNKNOWN;

	/*
	 * This section is only for updating the temperature lookup tables
	 * Start fan, pump and compressor
	 */

	// If any error has been found in the other modules, change compressor state to inactive. It will stop pump and compressor
	if (/*(g_oIceBoxx.ModuleInError == true) ||*/
		(g_oIceBoxx.bColdWaterProbeOk == false) ||
		(g_oIceBoxx.bAmbiantProbeOk == false) ||
		(g_oIceBoxx.bOverheated == true) ||
		(g_oIceBoxx.bColdWaterThresholdPotOk == false))
	{
		g_oIceBoxx.g_oCompressor.eIceBoxxStateCompressor = ICEBOXX_ICE_COMPRESSOR_INACTIVE;
	}

	// Switch case for the different states of the compressor
	// Possible modes:
	// 1. Standby
	// 2. Working
	// 3. Inactive
	switch (g_oIceBoxx.g_oCompressor.eIceBoxxStateCompressor)
	{
	case (ICEBOXX_ICE_COMPRESSOR_STANDBY):
	{
		// In standby mode, the the water temperature has reached the threshold.
		// By doing so, the compressor needs to stop and the pump needs to start
		if ((g_oIceBoxx.g_oCompressor.bIsActivated == true))
		{
			// Start pump only if it's not already working
			if (g_oIceBoxx.g_oPump.bIsActivated == false)
				IceboxxActivatePump(true, false);

			g_oIceBoxx.u32TimerCompressOFF = 0;

			// Stop compressor
			IceboxxActivateCompressor(false, false);
		}

		// If the temperature is over the upper limit, change the compressor state to working.
		// By doing so, the compressor will start
		// The pump state (working or not) will depend of the sequencer
		if (g_oIceBoxx.fTempColdResult > (g_oIceBoxx.fColdWaterTempThreshold + ICEBOXX_TEMP_COLD_WATER_THRESHOLD_HYST_C))
		{
			g_oIceBoxx.g_oCompressor.eIceBoxxStateCompressor = ICEBOXX_ICE_COMPRESSOR_WORKING;
		}
	}
	break;

	case (ICEBOXX_ICE_COMPRESSOR_WORKING):
	{
		// START COMP | STOP PUMP
		// If the compressor is not ON, start.
		if ((g_oIceBoxx.g_oCompressor.bIsActivated == false))
		{
			// Activate FAN (median and full board only)
			if (modelID != FRAMEWORK_MODEL_ID_BASE)
			{
				IceboxxActivateFan(true);
			}
			g_oIceBoxx.u32TimerCompressON = 0;
			IceboxxActivateCompressor(true, false);
			// Get timestamp for protection
			g_oIceBoxx.g_oCompressor.u32TimestampCompressorProtection = FrameworkGetTime();
		}

		// The pump is controlled here by the sequencer
		if ((g_oIceBoxx.g_oPump.g_oSequence.bPumpHoldFlag == true) || (g_oIceBoxx.g_oPump.g_oSequence.bPumpStartFlag == true))
		{
			IceboxxActivatePump(true, false);
		}
		else
		{
			IceboxxActivatePump(false, false);
		}
		// Temperature < Treshold -> STANDBY
		if (g_oIceBoxx.fTempColdResult < (g_oIceBoxx.fColdWaterTempThreshold - ICEBOXX_TEMP_COLD_WATER_THRESHOLD_HYST_C))
		{
			if (g_oIceBoxx.u32CompteurDelai >= ICEBOXX_COMPRESSOR_TEMP_REACHED_DELAY_MS)
			{
				if (g_oIceBoxx.g_oPump.bIsActivated == false)
				{
					g_oIceBoxx.u32TimerCompressOFF = 0;
				}
				g_oIceBoxx.g_oCompressor.eIceBoxxStateCompressor = ICEBOXX_ICE_COMPRESSOR_STANDBY;
				g_oIceBoxx.u32CompteurDelai = 0;
			}
		}
	}
	break;

	// Inactive state, compressor can not work here and can not be started
	case (ICEBOXX_ICE_COMPRESSOR_INACTIVE):
	{
		IceboxxActivateCompressor(false, true);
		IceboxxActivatePump(false, true);
		g_oIceBoxx.g_oCompressor.bIsAllowToStart = false;
		g_oIceBoxx.g_oPump.bIsAllowToStart = false;
		// Check for no iceboxx error
		if ((g_oIceBoxx.ModuleInError == false || //(IceBoxxModuleInError(&bIceBoxxModuleInError) == false
			 (g_oIceBoxx.g_oIcePoolWaterLevel.bLowLevelH2O == false)) &&
			g_oIceBoxx.bOverheated == false)
		{
			// Change state machine to STANDBY
			g_oIceBoxx.g_oCompressor.eIceBoxxStateCompressor = ICEBOXX_ICE_COMPRESSOR_STANDBY;
			g_oIceBoxx.g_oCompressor.bIsAllowToStart = true;
			g_oIceBoxx.g_oPump.bIsAllowToStart = true;
			g_oIceBoxx.u32CompteurDelai = 0;
		}
	}
	break;

	default:
	{
		return false;
	}
	break;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		WaterDispensorTask
/// \details	Execution of Water dispensor task section
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool WaterDispensorTask(void)
{

	ButtonPressed();

	// If any error has been found in the other modules, change compressor state to inactive. It will stop pump and compressor
	if ((g_oIceBoxx.ModuleInError == true) ||
		(g_oIceBoxx.bColdWaterProbeOk == false) ||
		(g_oIceBoxx.bAmbiantProbeOk == false) ||
		(g_oIceBoxx.bOverheated == true) ||
		(g_oIceBoxx.bColdWaterThresholdPotOk == false))
	{
		//	g_oIceBoxx.g_oDispensor.eWaterDispensorState = ICEBOXX_ICE_COMPRESSOR_INACTIVE;
	}

	// Switch case for the different states of the compressor
	// Possible modes:
	// 1. Standby
	// 2. Normal
	// 3. Cold
	// 4. Bubble
	// 5. Inactive
	switch (g_oIceBoxx.g_oDispensor.eWaterDispensorState)
	{
	case (WATER_DISPENSOR_STANDBY):
	{
		// In standby mode, there is no water being dispensed.
		// By doing so, the compressor needs to stop and the pump needs to start
		if ((g_oIceBoxx.g_oDDP.bIsActivated == true))
		{
			if (g_oIceBoxx.g_oIcePoolWaterLevel.bLowLevelH2O == true)
			{
				i = 0;
			}

			// Stop DDP if Carbonator is full
			if (g_oIceBoxx.g_oIcePoolWaterLevel.bLowLevelH2O == false)
			{
				i++;
				if (i == 20000)
				{
					i = 0;
					IceboxxActivateDDP(false, false);
					HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, SET);
				}
			}
		}

		//
		if ((g_oIceBoxx.g_oDDP.bIsActivated == false))
		{
			// Start DDP if Carbonator not full
			if (g_oIceBoxx.g_oIcePoolWaterLevel.bLowLevelH2O == true)
			{
				IceboxxActivateDDP(true, false);
				HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, SET);
			}
		}

		///////////////////Normal Water
		if (g_oIceBoxx.g_oValves.bNormalIsActivated == true)
		{
			HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, SET);
			HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_HOT_PERIPH, MCUMAP_DO_SLND_WATER_HOT_PIN, RESET);
			g_oIceBoxx.g_oValves.bNormalIsActivated = false;
		}

		if ((g_oIceBoxx.g_oDispensor.bNormalButton == true))
		{
			// Dispense normal Water
			g_oIceBoxx.g_oDispensor.eWaterDispensorState = WATER_DISPENSOR_NORMAL;
		}

		///////////////////Cold Water
		if (g_oIceBoxx.g_oValves.bColdIsActivated == true)
		{
			HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, SET);
			HAL_GPIO_WritePin(MCUMAP_DO_SLND_CO2_ON_PERIPH, MCUMAP_DO_SLND_CO2_ON_PIN, RESET);
			g_oIceBoxx.g_oValves.bColdIsActivated = false;
		}

		if ((g_oIceBoxx.g_oDispensor.bColdButton == true))
		{
			// Dispense Cold Water
			g_oIceBoxx.g_oDispensor.eWaterDispensorState = WATER_DISPENSOR_COLD;
		}
		///////////////////Bubble Water
		if (g_oIceBoxx.g_oValves.bBubbleIsActivated == true)
		{
			HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, SET);
			HAL_GPIO_WritePin(MCUMAP_DO_3WAY_VALVE_PERIPH, MCUMAP_DO_3WAY_VALVE_PIN, RESET);
			g_oIceBoxx.g_oValves.bBubbleIsActivated = false;
		}

		if ((g_oIceBoxx.g_oDispensor.bBubbleButton == true))
		{
			// Dispense Cold Water
			g_oIceBoxx.g_oDispensor.eWaterDispensorState = WATER_DISPENSOR_BUBBLE;
		}
	}
	break;

	case (WATER_DISPENSOR_NORMAL):
	{
		if (g_oIceBoxx.g_oValves.bNormalIsActivated == false)
		{
			HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, SET);
			HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_HOT_PERIPH, MCUMAP_DO_SLND_WATER_HOT_PIN, SET);
			g_oIceBoxx.g_oValves.bNormalIsActivated = true;
		}

		if ((g_oIceBoxx.g_oDispensor.bNormalButton == false))
		{
			g_oIceBoxx.g_oDispensor.eWaterDispensorState = WATER_DISPENSOR_STANDBY;
		}
	}
	break;

	case (WATER_DISPENSOR_COLD):
	{
		if (g_oIceBoxx.g_oValves.bColdIsActivated == false)
		{
			HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, SET);
			HAL_GPIO_WritePin(MCUMAP_DO_SLND_CO2_ON_PERIPH, MCUMAP_DO_SLND_CO2_ON_PIN, SET);
			g_oIceBoxx.g_oValves.bColdIsActivated = true;
		}

		if ((g_oIceBoxx.g_oDispensor.bColdButton == false))
		{
			g_oIceBoxx.g_oDispensor.eWaterDispensorState = WATER_DISPENSOR_STANDBY;
		}
	}
	break;

	case (WATER_DISPENSOR_BUBBLE):
	{
		if (g_oIceBoxx.g_oValves.bBubbleIsActivated == false)
		{
			HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, SET);
			HAL_GPIO_WritePin(MCUMAP_DO_3WAY_VALVE_PERIPH, MCUMAP_DO_3WAY_VALVE_PIN, SET);
			g_oIceBoxx.g_oValves.bBubbleIsActivated = true;
		}

		if ((g_oIceBoxx.g_oDispensor.bBubbleButton == false))
		{
			g_oIceBoxx.g_oDispensor.eWaterDispensorState = WATER_DISPENSOR_STANDBY;
		}
	}
	break;

	default:
	{
		return false;
	}
	break;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		ButtonPressed
/// \details
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool ButtonPressed(void)
{

	if (HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_1_PERIPH, MCUMAP_DI_BUTTON_1_PIN) == 0)
	{
		IceboxxActivateDDP(false, false);
		HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, SET);
		g_oIceBoxx.g_oDispensor.bNormalButton = true;
		g_oIceBoxx.g_oDispensor.bColdButton = false;
		g_oIceBoxx.g_oDispensor.bBubbleButton = false;
		return true;
	}
	if (HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_2_PERIPH, MCUMAP_DI_BUTTON_2_PIN) == 0)
	{
		IceboxxActivateDDP(false, false);
		HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, SET);
		g_oIceBoxx.g_oDispensor.bNormalButton = false;
		g_oIceBoxx.g_oDispensor.bColdButton = true;
		g_oIceBoxx.g_oDispensor.bBubbleButton = false;
		return true;
	}
	if (HAL_GPIO_ReadPin(MCUMAP_DI_BUTTON_3_PERIPH, MCUMAP_DI_BUTTON_3_PIN) == 0)
	{
		IceboxxActivateDDP(false, false);
		HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, SET);
		g_oIceBoxx.g_oDispensor.bNormalButton = false;
		g_oIceBoxx.g_oDispensor.bColdButton = false;
		g_oIceBoxx.g_oDispensor.bBubbleButton = true;
		return true;
	}
	else
	{
		g_oIceBoxx.g_oDispensor.bNormalButton = false;
		g_oIceBoxx.g_oDispensor.bColdButton = false;
		g_oIceBoxx.g_oDispensor.bBubbleButton = false;
	}
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		IceboxxActivateCompressor
/// \details	Function that activates/deactivates the compressor peripheral.
/// \private
///
/// \param		bActivate: flag indicating if we activate the peripheral or not.
/// \param		bForce: Will activate/deactivate the compressor, no matter its current state.
/// \return		None
////////////////////////////////////////////////////////////////////////////////
static void IceboxxActivateCompressor(bool bActivate, bool bForce)
{
	// Active LOW.
	if (bActivate == true && ((g_oIceBoxx.g_oCompressor.bIsActivated == false) || (bForce == true)) && (FrameworkTimeIsDue(g_oIceBoxx.g_oCompressor.u32TimestampLastDeactivation, ICEBOXX_COMPRESSOR_TIME_PROTECTION_MS) == true))
	{
		if (g_oIceBoxx.g_oCompressor.bIsAllowToStart == true)
		{
			HAL_GPIO_WritePin(MCUMAP_DO_COMPRESSOR_PERIPH, MCUMAP_DO_COMPRESSOR_PIN, GPIO_PIN_RESET);
			g_oIceBoxx.g_oCompressor.bIsActivated = true;
			// g_oIceBoxx.u32CompteurDelai++;
			g_oIceBoxx.uCompteurPump = 0;
		}
	}
	else if (bActivate == false && ((g_oIceBoxx.g_oCompressor.bIsActivated == true) || (bForce == true)))
	{
		HAL_GPIO_WritePin(MCUMAP_DO_COMPRESSOR_PERIPH, MCUMAP_DO_COMPRESSOR_PIN, GPIO_PIN_SET);
		g_oIceBoxx.g_oCompressor.u32TimestampLastDeactivation = FrameworkGetTime();

		g_oIceBoxx.g_oCompressor.bIsActivated = false;
	}
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		IceboxxActivatePump
/// \details	Function that activates/deactivates the pump peripheral.
/// \private
///
/// \param		bActivate: flag indicating if we activate the peripheral or not.
/// \param		bForce: Will activate/deactivate the pump, no matter its current state.
/// \return		None
////////////////////////////////////////////////////////////////////////////////
static void IceboxxActivatePump(bool bActivate, bool bForce)
{
	// Active HIGH.
	if (bActivate == true && ((g_oIceBoxx.g_oPump.bIsActivated == false) || (bForce == true)))
	{
		if (g_oIceBoxx.g_oPump.bIsAllowToStart == true)
		{
			HAL_GPIO_WritePin(MCUMAP_DO_PUMP_LOW_PERIPH, MCUMAP_DO_PUMP_LOW_PIN, GPIO_PIN_SET);
			g_oIceBoxx.g_oPump.bIsActivated = true;
			g_oIceBoxx.g_oPump.u32TimestampActivation = FrameworkGetTime();
		}
	}
	else if (bActivate == false && ((g_oIceBoxx.g_oPump.bIsActivated == true) || bForce == true))
	{
		HAL_GPIO_WritePin(MCUMAP_DO_PUMP_LOW_PERIPH, MCUMAP_DO_PUMP_LOW_PIN, GPIO_PIN_RESET);
		g_oIceBoxx.g_oPump.bIsActivated = false;
		g_oIceBoxx.g_oPump.u32TimestampDeactivation = FrameworkGetTime();
	}
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		IceboxxActivateDDP
/// \details	Function that activates/deactivates the DDP peripheral.
/// \private
///
/// \param		bActivate: flag indicating if we activate the peripheral or not.
/// \param		bForce: Will activate/deactivate the DDP, no matter its current state.
/// \return		None
////////////////////////////////////////////////////////////////////////////////
static void IceboxxActivateDDP(bool bActivate, bool bForce)
{
	// turn on
	if (bActivate == true && ((g_oIceBoxx.g_oDDP.bIsActivated == false) || (bForce == true)))
	{
		if (g_oIceBoxx.g_oDDP.bIsAllowToStart == true)
		{
			HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, SET);
			HAL_GPIO_WritePin(MCUMAP_DO_DDP_PERIPH, MCUMAP_DO_DDP_PIN, GPIO_PIN_RESET);
			g_oIceBoxx.g_oDDP.bIsActivated = true;
			g_oIceBoxx.g_oDDP.u32TimestampActivation = FrameworkGetTime();
		}
	}

	// turn off
	else if (bActivate == false && ((g_oIceBoxx.g_oDDP.bIsActivated == true) || bForce == true))
	{
		HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, SET);
		HAL_GPIO_WritePin(MCUMAP_DO_DDP_PERIPH, MCUMAP_DO_DDP_PIN, GPIO_PIN_SET);
		g_oIceBoxx.g_oDDP.bIsActivated = false;
		g_oIceBoxx.g_oDDP.u32TimestampDeactivation = FrameworkGetTime();
	}
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		IceboxxActivateFan
/// \details	Function that activates/deactivates the fan.
/// \private
///
/// \param		bActivate: flag indicating if we activate the peripheral or not.
/// \return		None
////////////////////////////////////////////////////////////////////////////////
static void IceboxxActivateFan(bool bActivate)
{
	if (bActivate == true)
	{
		//	HAL_GPIO_WritePin(ICEBOXX_GPIO_FAN_PORT, ICEBOXX_GPIO_FAN_PIN, GPIO_PIN_RESET);
	}
	else if (bActivate == false)
	{
		//	HAL_GPIO_WritePin(ICEBOXX_GPIO_FAN_PORT, ICEBOXX_GPIO_FAN_PIN, GPIO_PIN_SET);
	}
}

////////////////////////////////////////////////////////////////////////////////
/// \brief		IceboxxLinearInterp
/// \details	Will perform a cross product to convert adc to temperature value.
///				If value too high or low will return max or min value, respectively.
/// \private
///
/// \param		u16ADCValue: The ADC value to convert
/// \param		i16MinTemp: The minimal temperature value
/// \param		i16MaxTemp: The maximal temperature value
/// \param		u16MinADC: The minimal ADC value
/// \param		u16MaxADC: The maximal ADC value
/// \return		float: The converted temperature value
////////////////////////////////////////////////////////////////////////////////
static inline float IceboxxPotentiometer2Temp(uint16_t u16ADCValue, float i16MinTemp, float i16MaxTemp, uint16_t u16MinADC, uint16_t u16MaxADC)
{
	float result = ((u16ADCValue * (i16MaxTemp - i16MinTemp) / (u16MaxADC - u16MinADC)) + i16MinTemp);
	if (result > i16MaxTemp)
		return i16MaxTemp; // gards against to high value
	else if (result < i16MinTemp)
		return i16MinTemp; // gards against to low value
	else
		return result;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		IceBoxxModuleInError
/// \details	Get the state of the iceBoxx module
/// \public
///
/// \param[out]	pbState: true if module is in error, false otherwise
///
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool IceBoxxModuleInError(bool *pbState)
{
	if (pbState == NULL)
		return false;
	if (g_oIceBoxx.ModuleInError == true)
	{
		*pbState = true;
	}
	else
	{
		*pbState = false;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// UNIT TESTS
////////////////////////////////////////////////////////////////////////////////
#if (ICEBOXX_UNIT_TEST == 1)
#warning "IceBoxx unit test enable"
////////////////////////////////////////////////////////////////////////////////
/// IceBoxxUnitTest
////////////////////////////////////////////////////////////////////////////////
bool IceBoxxUnitTest(void)
{
	printf("\r\n");
	printf("------------------------------------------------------- \r\n");
	printf("---- ICEBOXX UNIT TESTS\r\n");
	printf("------------------------------------------------------- \r\n");

	return true;
}
#endif
