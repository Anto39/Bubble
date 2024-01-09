///
/// \file 		BottomLoad.c
/// \brief 		[Source file]
///
/// \author 	NOVO
///
////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <gpio.h>
#include "BottomLoad.h"

#include <stdio.h>
#include <string.h>
#include "Framework.h"
#include "MCUMap.h"
#include "adc.h"
#include "LedDriver.h"
#include "tim.h"


////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
#define BOTTOMLOAD_CONTROL_PERIOD_MS								(30)		///< Period for Bottom Load control actions. In ms.
#define BOTTOMLOAD_INITIAL_DEBOUNCING_PERIOD_MS						(500)		///< Button should be debounded after that
#define BOTTOMLOAD_MAX_BOOST_TEMP_REACH_PERIOD_MS					(300000)	///< Maximal time that the flag `boost mode temperature reach` can be set, in ms).

#define BOTTOMLOAD_ADC_INPUT_ERROR_THRESHOLD						(4000)		///< ADC count value indicating a misconnection of the probe.

#define BOTTOMLOAD_DIAGNOSTIC_CHRONO_1_MS							(5000)		///< Time used in a phase of the diagnostics, in ms.
#define BOTTOMLOAD_DIAGNOSTIC_CHRONO_2_MS							(60000)		///< Time used in a phase of the diagnostics, in ms.
#define BOTTOMLOAD_DIAGNOSTIC_CHRONO_3_MS							(240000)	///< Time used in a phase of the diagnostics, in ms.
#define BOTTOMLOAD_DIAGNOSTIC_TEMP_THRESHOLD_COLD_C					(10.0)  	///< Cold temperature threshold for diagnostics, in deg C.
#define BOTTOMLOAD_DIAGNOSTIC_TEMP_THRESHOLD_HOT_C					(10.0) 		///< Hot temperature threshold for diagnostics, in deg C)

#define BOTTOMLOAD_TEMP_HOT_WATER_BOOST_THRESHOLD_HYST_C			(1.0)		///< Hysteresis threshold for hot water control in boost mode, in deg C.
#define BOTTOMLOAD_TEMP_HOT_WATER_THRESHOLD_C						(1.0)		///< Holding target temperature for the hot water, in deg C).

//Sensor utilisé pour le contrôle de température
#define BOTTOMLOAD_TEMPERATURE_SENSOR_CHOICE						(false)		///< Sensor Control : true = COLD | false = HOT

//Contrôle du temps de cycles de pompe
#define BOTTOMLOAD_PUMP_TIME_ON										(3)			///< Time Pump is ON when Compressor is ON
#define BOTTOMLOAD_PUMP_TIME_OFF									(30)		///< Time Pump is OFF when Compressor is ON

//Contrôle de la température HOT
#define BOTTOMLOAD_POT_TEMP_HOT_WATER_MIN_C 						(1.0) 		///< Minimum adjustable temperature for the HOT water, in deg C.
#define BOTTOMLOAD_POT_TEMP_HOT_WATER_MAX_C 						(5.0) 		///< Maximum adjustable temperature for the HOT water, in deg C.
#define BOTTOMLOAD_TEMP_HOT_WATER_THRESHOLD_HYST_C					(1.0)		///< Hysteresis threshold for HOT water control, in deg C.

//Contrôle de la température COLD
#define BOTTOMLOAD_POT_TEMP_COLD_WATER_MIN_C						(2.2) 		///< Minimum adjustable temperature for the COLD water, in deg C.
#define BOTTOMLOAD_POT_TEMP_COLD_WATER_MAX_C						(6.2) 		///< Maximum adjustable temperature for the COLD water, in deg C.
#define BOTTOMLOAD_TEMP_COLD_WATER_THRESHOLD_HYST_C					(1.0)		///< Hysteresis threshold for COLD water control, in deg C.

#define BOTTOMLOAD_POT_ADC_MIN_COUNT								(0) 		///< Minimum ADC raw value for the adjustable temperature ranges, in counts.
#define BOTTOMLOAD_POT_ADC_MAX_COUNT								(2047)		///< Maximum ADC raw value for the adjustable temperature ranges, in counts).

#define BOTTOMLOAD_COMPRESSOR_MIN_REST_DELAY_MS						(180000)	///< Minimal compressor rest delay (time between last desactivation and next activation), in ms.
#define BOTTOMLOAD_COMPRESSOR_TEMP_REACHED_DELAY_MS					(60)		///< Delay to keep the compressor active after the target temperature was reached, in s).

#define BOTTOMLOAD_PUMPING_ON_TIME_CHECK_MS							(10000)		///< When trying to start pump, on perdiod in ms.
#define BOTTOMLOAD_PUMPING_OFF_TIME_CHECK_MS						(10000)		///< When trying to start pump, off perdiod in ms.
#define BOTTOMLOAD_PUMPING_WATER_ABSENCE_CHECK_MS					(10000)		///< When trying to start pump, on perdiod in ms.
#define BOTTOMLOAD_PUMPING_NUMBER_OF_RETRY							(1)	     	///< Number of time pump will do a start/stop cycle.

#define BOTTOMLOAD_HEATER_FIRST_START_DELAY_MS						(90000)		///< Start delay for the heater, in case there is no water at all. In ms.

#define BOTTOMLOAD_TEMPERATURE_ACQUISITION_TIMER_MS					(1000)		///< Delay of time for changing the value of cold temperature

// Remapping
#define BOTTOMLOAD_GPIO_DDI_BUTTON_BOOST							(GPIO_DDI_BUTTON_4)		///< Mapping of the Boost Button on generic button input.
#define BOTTOMLOAD_GPIO_DDI_WATER_PRESENCE_AFTER_BOTTLE_PIPE		(GPIO_DDI_LEVEL_ICE)	///< In bottomload, the Level ice will be use to detect if there's water at the end of end of the bottle pump.

/*
// OLD TABLE SENSOR HOT
#define BOTTOMLOAD_LUT_HOT_WATER_SIZE 106					///< Look-up table size for the hot water ADC count to temperature conversion.
static const int16_t rg16BottomLoadLUTHotWater[BOTTOMLOAD_LUT_HOT_WATER_SIZE][2] = {	///< Look-up table for the hot water ADC count to temperature conversion.
	{3580, 0},   {3556, 1},   {3532, 2},   {3507, 3},   {3482, 4},
	{3455, 5},   {3428, 6},   {3400, 7},   {3372, 8},   {3342, 9},
	{3312, 10},  {3282, 11},  {3250, 12},  {3218, 13},  {3185, 14},
	{3152, 15},  {3118, 16},  {3083, 17},  {3048, 18},  {3012, 19},
	{2975, 20},  {2938, 21},  {2901, 22},  {2863, 23},  {2824, 24},
	{2785, 25},  {2746, 26},  {2707, 27},  {2667, 28},  {2626, 29},
	{2586, 30},  {2545, 31},  {2504, 32},  {2463, 33},  {2422, 34},
	{2381, 35},  {2340, 36},  {2298, 37},  {2257, 38},  {2216, 39},
	{2175, 40},  {2134, 41},  {2093, 42},  {2052, 43},  {2012, 44},
	{1972, 45},  {1932, 46},  {1893, 47},  {1853, 48},  {1815, 49},
	{1776, 50},  {1738, 51},  {1701, 52},  {1663, 53},  {1627, 54},
	{1590, 55},  {1555, 56},  {1519, 57},  {1485, 58},  {1450, 59},
	{1417, 60},  {1384, 61},  {1351, 62},  {1319, 63},  {1287, 64},
	{1257, 65},  {1226, 66},  {1197, 67},  {1168, 68},  {1139, 69},
	{1111, 70},  {1084, 71},  {1057, 72},  {1030, 73},  {1005, 74},
	{980,  75},  {956,  76},  {931,  77},  {908,  78},  {885,  79},
	{862,  80},  {840,  81},  {819,  82},  {799,  83},  {778,  84},
	{759,  85},  {739,  86},  {721,  87},  {702,  88},  {684,  89},
	{667,  90},  {650,  91},  {633,  92},  {617,  93},  {602,  94},
	{586,  95},  {571,  96},  {557,  97},  {543,  98},  {529,  99},
	{516,  100}, {503,  101}, {490,  102}, {478,  103}, {466,  104},
	{454,  105}
};
*/

// New Look-up Table Hot Sensor input (ADAPTED FOR COLD WATER)
#define BOTTOMLOAD_LUT_HOT_WATER_SIZE 66					///< Look-up table size for the hot water ADC count to temperature conversion.
static const int16_t rg16BottomLoadLUTHotWater[BOTTOMLOAD_LUT_HOT_WATER_SIZE][2] = {	///< Look-up table for the hot water ADC count to temperature conversion.
{3684, -5 }, {3665, -4 }, {3644, -3 }, {3623, -2 }, {3601, -1 },
{3578,	0 }, {3555,  1 }, {3531,  2 }, {3506,  3 }, {3480,  4 },
{3454,  5 }, {3427,  6 }, {3399,  7 }, {3371,  8 }, {3342,  9 },
{3312,  10}, {3281,  11}, {3250,  12}, {3218,  13}, {3185,  14},
{3152,  15}, {3118,  16}, {3083,  17}, {3048,  18}, {3012,  19},
{2975,  20}, {2938,  21}, {2901,  22}, {2863,  23}, {2824,  24},
{2785,  25}, {2746,  26}, {2706,  27}, {2666,  28}, {2626,  29},
{2585,  30}, {2544,  31}, {2503,  32}, {2462,  33}, {2421,  34},
{2380,  35}, {2338,  36}, {2297,  37}, {2255,  38}, {2214,  39},
{2173,  40}, {2132,  41}, {2091,  42}, {2050,  43}, {2009,  44},
{1969,  45}, {1929,  46}, {1889,  47}, {1850,  48}, {1811,  49},
{1772,  50}, {1734,  51}, {1696,  52}, {1659,  53}, {1622,  54},
{1586,  55}, {1550,  56}, {1514,  57}, {1479,  58}, {1445,  59},
{1412, 60}
};

// New Look-up Table Cold
#define BOTTOMLOAD_LUT_COLD_WATER_SIZE 	66		///< Look-up table size for the cold water ADC count to temperature conversion.
static const int16_t rg16BottomLoadLUTColdWater[BOTTOMLOAD_LUT_COLD_WATER_SIZE][2] = {	///< Look-up table for the cold water ADC count to temperature conversion.
{3310, -5 }, {3276, -4 }, {3242, -3 }, {3206, -2 }, {3170, -1 },
{3133,  0 }, {3095,  1 }, {3056,  2 }, {3017,  3 }, {2977,  4 },
{2936,  5 }, {2895,  6 }, {2853,  7 }, {2810,  8 }, {2768,  9 },
{2724,  10}, {2680,  11}, {2636,  12}, {2592,  13}, {2547,  14},
{2502,  15}, {2456,  16}, {2411,  17}, {2365,  18}, {2320,  19},
{2274,  20}, {2228,  21}, {2183,  22}, {2137,  23}, {2092,  24},
{2047,  25}, {2002,  26}, {1958,  27}, {1913,  28}, {1870,  29},
{1826,  30}, {1783,  31}, {1740,  32}, {1698,  33}, {1657,  34},
{1616,  35}, {1576,  36}, {1536,  37}, {1497,  38}, {1458,  39},
{1420,  40}, {1383,  41}, {1347,  42}, {1311,  43}, {1276,  44},
{1242,  45}, {1208,  46}, {1175,  47}, {1143,  48}, {1111,  49},
{1081,  50}, {1051,  51}, {1021,  52}, {992,  53},  {964,  54},
{937,  55},  {911,  56},  {885,  57},  {860,  58},  {835,  59},
{811,  60}
};

///
/// \enum	eBottomLoadState_t
/// \brief	Enumeration of the possible states of the module.
///
typedef enum
{
	BOTTOMLOAD_STATE_INIT = 0,				///< Always the first state after reboot. Will wait for debouncing data to be valid.
	BOTTOMLOAD_STATE_DIAGNOSTIC,			///< Special, user triggered state. Will do a diagnostic of the machine and wait for reboot.
	BOTTOMLOAD_STATE_NORMAL,				///< Usual state. Will handle normal functionalities.
	BOTTOMLOAD_STATE_BOTTLE_CHANGE,			///< No water left. Will handle the process of changing the water bottle.
	BOTTOMLOAD_STATE_ERROR_LEVEL_SWITCH,	///< Error state, when there is an incoherence between the water level switches.
	BOTTOMLOAD_STATE_ERROR_FLOOD,			///< Error state, when flooding.
} eBottomLoadState_t;

///
/// \enum	eBottomLoadDiagnosticState_t
/// \brief	Enumeration of the possible states of the diagnostics.
///
typedef enum
{
	BOTTOMLOAD_DIAGNOSTIC_STATE_INIT = 0,       	 ///< Initial diagnostic state. Prepare the firmware.
	BOTTOMLOAD_DIAGNOSTIC_STATE_TESTING_PHASE_1,	 ///< Testing phase
	BOTTOMLOAD_DIAGNOSTIC_STATE_TESTING_PHASE_2,	 ///< Testing phase
	BOTTOMLOAD_DIAGNOSTIC_STATE_TESTING_PHASE_3,	 ///< Testing phase
	BOTTOMLOAD_DIAGNOSTIC_STATE_TESTING_PHASE_4,	 ///< Testing phase
	BOTTOMLOAD_DIAGNOSTIC_STATE_WAIT_FOR_REBOOT,	 ///< Test is finished. Now system will wait for reboot.
} eBottomLoadDiagnosticState_t;

///
/// \enum	eBottomLoadDiagnosticResults_t
/// \brief	Enumeration of the possible diagnostic results.
///
typedef enum
{
	BOTTOMLOAD_DIAGNOSTIC_RESULT_UNKNOWN = 0,	              ///< Initial state: Still waiting for diagnostic results.
	BOTTOMLOAD_DIAGNOSTIC_RESULT_OK,                          ///< No problem detected.
	BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_COLD_WATER_PROBE_READING,
	BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_COLD_WATER_THRESHOLD_POT_READING,
	BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_HOT_WATER_PROBE_READING,
	BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_HOT_WATER_THRESHOLD_POT_READING,
	BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_SENSOR_READING,      ///< Problem detected. Cause: Sensor reading.
	BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_GAS_LEAK,            ///< Problem detected. Cause: Gas leak.
	BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_JUMPING_COMPRESSOR,  ///< Problem detected. Cause: Jumping compressor.
} eBottomLoadDiagnosticResults_t;

///
/// \enum	eBottomLoadPumRetryState_t
/// \brief	Enumeration of the possible pump retry state.
///
typedef enum
{
	BOTTOMLOAD_PUMP_RETRY_STATE_INIT = 0,     	  ///< Initial state: Prepare the retry counter and start.
	BOTTOMLOAD_PUMP_RETRY_STATE_TRY_TO_PUMP,   	  ///< Active the pump for a period of time.
	BOTTOMLOAD_PUMP_RETRY_STATE_COOL_OFF,      	  ///< Deactive the pump for a period of time. If the retry are exausted, go to RETRY_EXAUSTED.
	BOTTOMLOAD_PUMP_RETRY_STATE_RETRY_EXAUSTED,	  ///< Reset state machine and inform the caller that retry are exausted.
} eBottomLoadPumRetryState_t;

///
/// \struct	oBottomLoadInputData_t
/// \brief	Structure containing all the data retrieved from digital and/or analog inputs.
///
typedef struct
{
	bool	bHotWaterProbeOk;
	bool	bHotWaterThresholdPotOk;
	float	fHotWaterTemp;					///< Hot water tank temperature (deg. C). Set by a thermistor.
	float	fHotWaterTempThreshold;			///< Hot water temperature threshold. Set by a potentiometer.
	bool	bColdWaterProbeOk;
	bool	bColdWaterThresholdPotOk;
	float	fColdWaterTemp;					///< Cold water tank temperature (deg. C). Set by a thermistor.
	float	fColdWaterTempThreshold;		///< Cold water temperature threshold. Set by a potentiometer.
	bool	bWaterLevelLowState;			///< If there's water in lower part of the main tank. Set by a 2 probes connectivity detector.
	bool	bWaterLevelHighState;			///< If there's water in higher part of the main tank. Set by a 2 probes connectivity detector.
	bool	bDoorIsOpen;					///< If the door is open. No info on the sensor used.
	bool	bBottleIsEmpty;					///< If the bottle is empty. No info on the sensor used.
	bool	bWaterPresenceBottomFlood;		///< If there's water outside the tanks, at the bottom of the machine. No info on the sensor used.
	bool	bWaterPresenceTopFlood;			///< If there's water outside the tanks, at the top of the machine. No info on the sensor used.
	bool 	bWaterPresenceAfterBottlePipe;	///< If there's water at the end of the bottle water pipe.
} oBottomLoadInputData_t;

///
/// \struct	oBottomLoadStatus_t
/// \brief	Structure containing all the statuses of the module.
///
typedef struct
{
	bool 	bIsStarting;					///< This flag will be set for the first execution of bottomload task. After it will be cleared.
	bool	bInBoostMode;					///< Other hot water mode. This mode is user activated.
	bool 	bBoostModeTemperatureReached;	///< When boost mode reach wanted temperature, set this variable. It stay on until, temperature reach a threshold.
	bool    bInFillingTheTankMode;			///< When set, indicate the system is trying to pump water in the tank to full it.
	bool	bInDoorOpenMode;				///< Door is open.
} oBottomLoadStatus_t;

///
/// \struct	oBottomLoadCompressor_t
/// \brief	Structure containing compressor specific variables.
///
typedef struct
{
	bool		bIsActivated;					///< Flag indicating if the compressor is currently activated or not.
	bool 		bIsAllowToStart;				///< Flag indicating if the compressor has the permission to start. (Useful at when errors)
	bool		bTempReached;					///< Flag indicating if the target temperature has been reached or not.
	uint32_t	u32TimestampLastDeactivation;	///< Timestamp of the last deactivation, in ms.
	uint32_t	u32TimestampTempReached;		///< Timestamp of when the temperature was reached, in ms.
} oBottomLoadCompressor_t;

///
/// \struct	oBottomLoadPump_t
/// \brief	Structure containing pump specific variables.
///
typedef struct
{
	bool				bIsActivated;					///< Flag indicating if the pump is currently activated or not.
	bool 				bIsAllowToStart;				///< Flag indicating if the pump has the permission to start. (Useful at when errors)
	uint8_t				u8RetryCounter;					///< Keep track of the number of retry done in the retry mechanism.
	uint32_t			u32TimestampActivation;			///< Timestamp of the last activation, in ms.
	uint32_t			u32TimestampDeactivation;	    ///< Timestamp of the last deactivation, in ms.
} oBottomLoadPump_t;

///
/// \struct	oBottomLoadHeater_t
/// \brief	Structure containing heater specific variables.
///
typedef struct
{
	bool		bIsActivated;					///< Flag indicating if the heater is currently activated or not.
	bool 		bIsAllowToStart;				///< Flag indicating if the heater has the permission to start. (Useful at bootup)
	uint32_t	u32TimestampFirstStart;			///< Timestamp of the first start, in ms.

} oBottomLoadHeater_t;


///
/// \struct	oBottomLoadLeds_t
/// \brief	Structure containing all the LEDs.
///
typedef struct
{
	oLedDriverLed_t		oNight;		///< Night-light Led object
	oLedDriverLed_t		oBlue;		///< Blue Led object
	oLedDriverLed_t		oGreen;		///< Green Led object
	oLedDriverLed_t		oRed;		///< Red Led object
	oLedDriverLed_t		oYellow;	///< Yellow Led object
} oBottomLoadLeds_t;

///
/// \struct	oBottomLoadDiagnostic_t
/// \brief	Structure containing all the diags data.
///
typedef struct
{
	eBottomLoadDiagnosticState_t	eState;				///< Current diagnostics state.
	eBottomLoadDiagnosticResults_t  eResult;			///< Diagnostics result.
	uint32_t						u32TimestampTest;	///< Timestamp used on the diagnostics tests.
} oBottomLoadDiagnostic_t;

///
/// \struct	oBottomLoad_t
/// \brief	Structure containing this module specific variables.
///
typedef struct
{
	eBottomLoadState_t			eState;						///< Main state-machine state of the Bottom Load model.
	uint32_t					u32TimestampControl;		///< Timestamp used to control (periodically) the Bottom Load actions.
	uint32_t					u32TimestampStartup;		///< Timestamp used at startup to control/detect if the diagnostics mode must be activated.
	uint32_t					u32TimestampBoostTempReach;	///< Timestamp used to note when boost mode temperature has been reach.
	uint32_t					u32TimestampWaterPresenceToggle;
	uint32_t					u32TimestampWaterAquisition;///< Timestamp used to delay the acquistion of cold temperature
	uint8_t						u8Compteur;					///< Counter to make 30 acquistion for a moyenne
	uint32_t					u32CompteurDelaiComp;		///< Time before stopping compressor
	uint8_t						u32CompteurLevelSwitch;		///< Timer before going into BOTTOMLOAD_STATE_ERROR_LEVEL_SWITCH
	uint32_t					u32TimerCompressON;			///< Time compressor stayed ON
	uint32_t					u32TimerCompressOFF;		///< Time compressor stayed OFF

	float						fTempColdActivComp;				///< Activation temperature of compressor
	float						fTempHotActivComp;				///< Activation temperature of compressor
	float						fTempColdDeactivComp;				///< Activation temperature of compressor
	float						fTempHotDeactivComp;				///< Activation temperature of compressor
	float						fTempColdMoyenne;			///< Acquisition of temperature each second
	float						fTempHotMoyenne;			///< Acquisition of temperature each second
	float						fTempColdResult;			///< Average updated every 30 seconds
	float						fTempHotResult;				///< Average updated every 30 seconds

	oBottomLoadStatus_t			oStatus;					///< Contains all the statuses of the Bottom Load model.
	oBottomLoadInputData_t		oInputData;					///< Contains all the input data of the Bottom Load model.
	oBottomLoadHeater_t			oHeater;					///< Heater peripheral of the Bottom Load model.
	oBottomLoadCompressor_t		oCompressor;				///< Compressor peripheral of the Bottom Load model.
	oBottomLoadPump_t			oPump;						///< Pump peripheral of the Bottom Load model.
	oBottomLoadLeds_t			oLed;						///< Contains all the LEDs peripheral of the Bottom Load model.
	oBottomLoadDiagnostic_t 	oDiagnostic;				///< Diagnostics data.
} oBottomLoad_t;


// Usefull macro
#define TANK_IS_EMPTY() (((g_oBottomLoad.oInputData.bWaterLevelLowState == false) && (g_oBottomLoad.oInputData.bWaterLevelHighState == false)) ? true : false)
#define TANK_IS_FULL()  (((g_oBottomLoad.oInputData.bWaterLevelLowState == true)  && (g_oBottomLoad.oInputData.bWaterLevelHighState == true))  ? true : false)


////////////////////////////////////////////////////////////////////////////////
// Private functions
////////////////////////////////////////////////////////////////////////////////
static bool BottomLoadUpdateData(void);
static bool BottomLoadVerifyErrors(void);
static bool BottomLoadCheckStartupCondition(void);
static bool BottomLoadDiagnostic(void);
static bool BottomLoadProcHeating(void);
static bool BottomLoadProcChilling(void);
static bool BottomLoadProcPumping(void);
static bool BottomLoadUpdateLeds(void);
static bool BottomLoadUpdateLedDiagnostic(void);
static bool BottomLoadUpdateLedDefault(void);
static void BottomLoadActivatePump(bool bActivate, bool bForce);
static void BottomLoadActivateHeater(bool bActivate, bool bForce);
static void BottomLoadActivateCompressor(bool bActivate, bool bForce);
static void BottomLoadActivateBuzzer(bool bActivate);
static bool BottomLoadLUT2DLinearSearch(const int16_t rgu16LUT[][2], uint16_t u16LUTLen, uint16_t u16X, float *fY);
static inline float BottomLoadLinearInterp(float fX, float fXa, float fYa, float fXb, float fYb);
static inline float BottomLoadPotentiometer2Temp(uint16_t u16ADCValue, float i16MinTemp, float i16MaxTemp, uint16_t u16MinADC, uint16_t u16MaxADC);


////////////////////////////////////////////////////////////////////////////////
// Private variables
////////////////////////////////////////////////////////////////////////////////
oBottomLoad_t	g_oBottomLoad;					///< Main Bottom Load object.


////////////////////////////////////////////////////////////////////////////////
/// \brief 		BottomLoadInit
/// \details	Initialization of the module.
/// \public
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool BottomLoadInit(void)
{
	// Initialize main object.
	g_oBottomLoad.eState									= BOTTOMLOAD_STATE_INIT;
	g_oBottomLoad.u32TimestampControl						= 0;
	g_oBottomLoad.u32TimestampStartup						= 0;
	g_oBottomLoad.u32TimestampWaterPresenceToggle			= 0;

	g_oBottomLoad.oStatus.bInBoostMode						= false;
	g_oBottomLoad.oStatus.bBoostModeTemperatureReached		= false;
	g_oBottomLoad.oStatus.bInDoorOpenMode					= false;
	g_oBottomLoad.oStatus.bInFillingTheTankMode				= false;
	g_oBottomLoad.oStatus.bIsStarting						= true;

	g_oBottomLoad.oInputData.fHotWaterTemp					= 0.0;
	g_oBottomLoad.oInputData.fHotWaterTempThreshold			= 0.0;
	g_oBottomLoad.oInputData.fColdWaterTemp					= 0.0;
	g_oBottomLoad.oInputData.fColdWaterTempThreshold		= 0.0;
	g_oBottomLoad.oInputData.bWaterLevelLowState			= false;
	g_oBottomLoad.oInputData.bWaterLevelHighState			= false;
	g_oBottomLoad.oInputData.bDoorIsOpen					= false;
	g_oBottomLoad.oInputData.bBottleIsEmpty					= false;
	g_oBottomLoad.oInputData.bWaterPresenceBottomFlood		= false;
	g_oBottomLoad.oInputData.bWaterPresenceTopFlood			= false;
	g_oBottomLoad.oInputData.bColdWaterProbeOk				= false;
	g_oBottomLoad.oInputData.bColdWaterThresholdPotOk		= false;
	g_oBottomLoad.oInputData.bHotWaterProbeOk				= false;
	g_oBottomLoad.oInputData.bHotWaterThresholdPotOk		= false;
	
	g_oBottomLoad.oHeater.bIsActivated						= false;
	g_oBottomLoad.oHeater.bIsAllowToStart					= false;
	g_oBottomLoad.oHeater.u32TimestampFirstStart			= 0;
	g_oBottomLoad.oCompressor.bTempReached					= false;
	g_oBottomLoad.oCompressor.bIsAllowToStart				= true;
	g_oBottomLoad.oCompressor.bIsActivated					= false;
	g_oBottomLoad.oCompressor.u32TimestampLastDeactivation 	= 0;
	g_oBottomLoad.oCompressor.u32TimestampTempReached	   	= 0;
	g_oBottomLoad.oPump.bIsActivated						= true;
	g_oBottomLoad.oPump.bIsAllowToStart						= true;
	g_oBottomLoad.oPump.u8RetryCounter						= 0;
	g_oBottomLoad.oPump.u32TimestampActivation				= 0;
	g_oBottomLoad.oPump.u32TimestampDeactivation			= 0;

	g_oBottomLoad.u32TimestampWaterAquisition				= 0;
	g_oBottomLoad.u8Compteur								= 0;
	g_oBottomLoad.u32CompteurDelaiComp						= 0;
	g_oBottomLoad.u32CompteurLevelSwitch					= 0;
	g_oBottomLoad.u32TimerCompressON						= 0;
	g_oBottomLoad.u32TimerCompressOFF						= 0;
	g_oBottomLoad.fTempColdActivComp						= 0;
	g_oBottomLoad.fTempHotActivComp							= 0;
	g_oBottomLoad.fTempColdDeactivComp						= 0;
	g_oBottomLoad.fTempHotDeactivComp						= 0;
	g_oBottomLoad.fTempColdMoyenne							= 0;
	g_oBottomLoad.fTempHotMoyenne							= 0;
	g_oBottomLoad.fTempColdResult							= 0;
	g_oBottomLoad.fTempHotResult							= 0;

	// Init LED object
	LedDriverInitLed(&g_oBottomLoad.oLed.oNight, MCUMAP_DO_LED_NIGHT_PERIPH, MCUMAP_DO_LED_NIGHT_PIN);
	LedDriverInitLed(&g_oBottomLoad.oLed.oBlue, MCUMAP_DO_LED_BLUE_PERIPH, MCUMAP_DO_LED_BLUE_PIN);
	LedDriverInitLed(&g_oBottomLoad.oLed.oGreen, MCUMAP_DO_LED_GREEN_BI_PERIPH, MCUMAP_DO_LED_GREEN_BI_PIN);
	LedDriverInitLed(&g_oBottomLoad.oLed.oRed, MCUMAP_DO_LED_RED_PERIPH, MCUMAP_DO_LED_RED_PIN);
	LedDriverInitLed(&g_oBottomLoad.oLed.oYellow, MCUMAP_DO_LED_YELLOW_PERIPH, MCUMAP_DO_LED_YELLOW_PIN);

	// Open night light.
	LedDriverSetModeLed(&g_oBottomLoad.oLed.oNight, LEDDRIVER_LED_MODE_ON);

	// Force all peripherals to be in a known state.
	BottomLoadActivateHeater(false, true);
	BottomLoadActivateCompressor(false, true);
	BottomLoadActivatePump(true, true);
	BottomLoadActivateBuzzer(false);
	
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		BottomLoadTask
/// \details	Execution of the main task of the module.
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool BottomLoadTask(void)
{
	if (FrameworkTimeIsDue(g_oBottomLoad.u32TimestampControl, BOTTOMLOAD_CONTROL_PERIOD_MS) == true)
	{
		g_oBottomLoad.u32TimestampControl = FrameworkGetTime();

		// Retrive all information.
		if (BottomLoadUpdateData() == false) return false;

		// Verify errors and abberations.
		if (BottomLoadVerifyErrors() == false) return false;

		// Process according to the current state.
		switch (g_oBottomLoad.eState)
		{
			// Initial state (Always start here)
			case BOTTOMLOAD_STATE_INIT:
			{
				// Wait and check if its a normal start or a diagnostic start.
				if (BottomLoadCheckStartupCondition() == false) return false;
			} break;

			// Diagnostic state.
			case BOTTOMLOAD_STATE_DIAGNOSTIC:
			{
				if (BottomLoadDiagnostic() == false) return false;
			} break;

			// Normal state.
			case BOTTOMLOAD_STATE_NORMAL:
			{
				// Process the different behaviors.
				if (BottomLoadProcHeating() == false) return false;
				if (BottomLoadProcChilling() == false) return false;
				if (BottomLoadProcPumping() == false) return false;
				// Make sure system is not in starting mode
				g_oBottomLoad.oStatus.bIsStarting = false;
			} break;

			// Bottle change state. Limited in what we allow.
			case BOTTOMLOAD_STATE_BOTTLE_CHANGE:
			{
				// Process the different behaviors. Do not allow boost in bottle change.
				if (BottomLoadProcHeating() == false) return false;
				if (BottomLoadProcChilling() == false) return false;
				
				// Instead of processing the pump, special treatment is needed because we are changing the bottle.
				//**BottomLoadActivatePump(false, false);

				// Detect a door opening.
				if ((g_oBottomLoad.oStatus.bInDoorOpenMode == false) && (g_oBottomLoad.oInputData.bDoorIsOpen == true))
				{
					g_oBottomLoad.oStatus.bInDoorOpenMode = true;
				}
				// Detect a door closing. This will exit the bottle change mode.
				else if ((g_oBottomLoad.oStatus.bInDoorOpenMode == true) && (g_oBottomLoad.oInputData.bDoorIsOpen == false))
				{
					g_oBottomLoad.oStatus.bInDoorOpenMode = false;
					g_oBottomLoad.eState 			      = BOTTOMLOAD_STATE_NORMAL;
				}
			} break;
			// Error state
			case BOTTOMLOAD_STATE_ERROR_LEVEL_SWITCH:
			{
				// Only pump is deactivated in this error state.
				// Process the different behaviors.
				if (BottomLoadProcHeating() == false) return false;
				if (BottomLoadProcChilling() == false) return false;
			} break;
			
			// Error state
			case BOTTOMLOAD_STATE_ERROR_FLOOD:
			{
				// Nothing is allowed in this state. Reset the power to recover.
			} break;
			default: return false;
		}
	}

	// Update the LEDs.
	if (BottomLoadUpdateLeds() == false) return false;
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		BottomLoadUpdateData
/// \details	Function that updates all input data.
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
static bool BottomLoadUpdateData(void)
{
	uint16_t	u16ADCValue1 	= 0;
	uint16_t	u16ADCValue2 	= 0;
	float 		fTemperature 	= 0.0;
	float		hTemperature	= 0.0;
	bool 		bTempVal		= false;

//*********************** Gestion Capteur Chaud (J7) **************************************//
  	if(BOTTOMLOAD_TEMPERATURE_SENSOR_CHOICE == false)
	{
  		// Get average analog input of both Sensors
		if ((HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_HOT, &u16ADCValue2) == false) || (HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_COLD_1, &u16ADCValue1) == false)) return false;

		// Perform a lookup with the tables to find value
		if (BottomLoadLUT2DLinearSearch(rg16BottomLoadLUTColdWater, BOTTOMLOAD_LUT_COLD_WATER_SIZE, u16ADCValue1, &fTemperature) == false) return false;
		g_oBottomLoad.oInputData.fColdWaterTemp = fTemperature;
		if (BottomLoadLUT2DLinearSearch(rg16BottomLoadLUTHotWater, BOTTOMLOAD_LUT_HOT_WATER_SIZE, u16ADCValue2, &hTemperature) == false) return false;
		g_oBottomLoad.oInputData.fHotWaterTemp = hTemperature;

		// Accumulate average of both sensors
		if(FrameworkTimeIsDue(g_oBottomLoad.u32TimestampWaterAquisition, BOTTOMLOAD_TEMPERATURE_ACQUISITION_TIMER_MS) == true)
		{
			g_oBottomLoad.fTempColdMoyenne=g_oBottomLoad.fTempColdMoyenne+fTemperature;
			g_oBottomLoad.fTempHotMoyenne=g_oBottomLoad.fTempHotMoyenne+hTemperature;
			g_oBottomLoad.u8Compteur++;
			g_oBottomLoad.u32TimestampWaterAquisition = FrameworkGetTime();

			// Calcul un délai de X minutes après avoir atteint la température désirée avant de fermer le compresseur
			if ((g_oBottomLoad.oCompressor.bIsActivated == true))
			{
				g_oBottomLoad.u32TimerCompressON++;
				if(g_oBottomLoad.oPump.bIsActivated == false)
				{
					if(g_oBottomLoad.oPump.bIsAllowToStart == true)
					{
					BottomLoadActivatePump(true, false);
					}
				}
				if(g_oBottomLoad.oPump.bIsActivated == true)
				{
					BottomLoadActivatePump(false, false);
				}
				if(g_oBottomLoad.fTempHotResult < (g_oBottomLoad.oInputData.fHotWaterTempThreshold - BOTTOMLOAD_TEMP_HOT_WATER_THRESHOLD_HYST_C)) // temperature is reached !
				{
					g_oBottomLoad.u32CompteurDelaiComp++;
				}
			}
			else if ((g_oBottomLoad.oCompressor.bIsActivated == false))
			{
				g_oBottomLoad.u32TimerCompressOFF++;
			}
		}
	}
//*********************** Gestion Capteur Froid (J11) **************************************//
	if(BOTTOMLOAD_TEMPERATURE_SENSOR_CHOICE == true)
	{
		// Get average analog input of both Sensors
		if ((HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_HOT, &u16ADCValue2) == false) || (HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_COLD_1, &u16ADCValue1) == false)) return false;

		// Perform a lookup with the tables to find value
		if (BottomLoadLUT2DLinearSearch(rg16BottomLoadLUTColdWater, BOTTOMLOAD_LUT_COLD_WATER_SIZE, u16ADCValue1, &fTemperature) == false) return false;
		g_oBottomLoad.oInputData.fColdWaterTemp = fTemperature;
		if (BottomLoadLUT2DLinearSearch(rg16BottomLoadLUTHotWater, BOTTOMLOAD_LUT_HOT_WATER_SIZE, u16ADCValue2, &hTemperature) == false) return false;
		g_oBottomLoad.oInputData.fHotWaterTemp = hTemperature;

		// Accumulate average of both sensors
		if(FrameworkTimeIsDue(g_oBottomLoad.u32TimestampWaterAquisition, BOTTOMLOAD_TEMPERATURE_ACQUISITION_TIMER_MS) == true)
		{
			g_oBottomLoad.fTempColdMoyenne=g_oBottomLoad.fTempColdMoyenne+fTemperature;
			g_oBottomLoad.fTempHotMoyenne=g_oBottomLoad.fTempHotMoyenne+hTemperature;
			g_oBottomLoad.u8Compteur++;
			g_oBottomLoad.u32TimestampWaterAquisition = FrameworkGetTime();

			// Calcul un délai de X minutes après avoir atteint la température désirée avant de fermer le compresseur
			if ((g_oBottomLoad.oCompressor.bIsActivated == true))
			{
				g_oBottomLoad.u32TimerCompressON++;
				if(g_oBottomLoad.fTempColdResult < (g_oBottomLoad.oInputData.fColdWaterTempThreshold - BOTTOMLOAD_TEMP_COLD_WATER_THRESHOLD_HYST_C)) // temperature is reached !
				{
					g_oBottomLoad.u32CompteurDelaiComp++;
				}
			}
			else if ((g_oBottomLoad.oCompressor.bIsActivated == false))
			{
				g_oBottomLoad.u32TimerCompressOFF++;
			}
		}
	}

//*********************** Moyenne des deux capteurs **************************************//

	// Calculate average on 30 seconds
	if(g_oBottomLoad.u8Compteur==30)
	{
		g_oBottomLoad.fTempColdResult=g_oBottomLoad.fTempColdMoyenne/30;
		g_oBottomLoad.fTempHotResult=g_oBottomLoad.fTempHotMoyenne/30;
		g_oBottomLoad.fTempColdMoyenne=0;
		g_oBottomLoad.fTempHotMoyenne=0;
		g_oBottomLoad.u8Compteur=0;
	}

	// Validate water probes behavior.
	if (u16ADCValue2 >= BOTTOMLOAD_ADC_INPUT_ERROR_THRESHOLD) 	{g_oBottomLoad.oInputData.bHotWaterProbeOk = false;}
	else 														{g_oBottomLoad.oInputData.bHotWaterProbeOk = true;}
	if (u16ADCValue1 >= BOTTOMLOAD_ADC_INPUT_ERROR_THRESHOLD) 	{g_oBottomLoad.oInputData.bColdWaterProbeOk = false;}
	else														{g_oBottomLoad.oInputData.bColdWaterProbeOk = true;}

	// Water temperature thresholds.
	if (HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_COLD_ADJUST, &u16ADCValue1) == false) return false;
	g_oBottomLoad.oInputData.fColdWaterTempThreshold	= BottomLoadPotentiometer2Temp(u16ADCValue1,
																				   BOTTOMLOAD_POT_TEMP_COLD_WATER_MIN_C, BOTTOMLOAD_POT_TEMP_COLD_WATER_MAX_C,
																				   BOTTOMLOAD_POT_ADC_MIN_COUNT, BOTTOMLOAD_POT_ADC_MAX_COUNT);
	if (HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_HOT_ADJUST, &u16ADCValue2) == false) return false;
	g_oBottomLoad.oInputData.fHotWaterTempThreshold	= BottomLoadPotentiometer2Temp(u16ADCValue2,
																				   BOTTOMLOAD_POT_TEMP_HOT_WATER_MIN_C, BOTTOMLOAD_POT_TEMP_HOT_WATER_MAX_C,
																				   BOTTOMLOAD_POT_ADC_MIN_COUNT, BOTTOMLOAD_POT_ADC_MAX_COUNT);
	// Validate water threshold input behavior.
	if (u16ADCValue2 >= BOTTOMLOAD_ADC_INPUT_ERROR_THRESHOLD) 	{g_oBottomLoad.oInputData.bHotWaterThresholdPotOk = false;}
	else 														{g_oBottomLoad.oInputData.bHotWaterThresholdPotOk = true;}
	if (u16ADCValue1 >= BOTTOMLOAD_ADC_INPUT_ERROR_THRESHOLD) 	{g_oBottomLoad.oInputData.bColdWaterThresholdPotOk = false;}
	else 														{g_oBottomLoad.oInputData.bColdWaterThresholdPotOk = true;}
	

		// Digital inputs.
		if (GPIOGetDDI(GPIO_DDI_WATER_LEVEL_LOW, &g_oBottomLoad.oInputData.bWaterLevelLowState) == false) return false;
		if (GPIOGetDDI(GPIO_DDI_WATER_LEVEL_HIGH, &g_oBottomLoad.oInputData.bWaterLevelHighState) == false) return false;
		if (GPIOGetDDI(GPIO_DDI_DOOR_OPEN, &g_oBottomLoad.oInputData.bDoorIsOpen) == false) return false;
		if (GPIOGetDDI(GPIO_DDI_EMPTY_BOTTLE, &g_oBottomLoad.oInputData.bBottleIsEmpty) == false) return false;
		if (GPIOGetDDI(GPIO_DDI_PRESENCE_FLOOD_BOTTOM, &g_oBottomLoad.oInputData.bWaterPresenceBottomFlood) == false) return false;
		if (GPIOGetDDI(GPIO_DDI_PRESENCE_FLOOD_TOP, &g_oBottomLoad.oInputData.bWaterPresenceTopFlood) == false) return false;

		// Water presence after pipe has a special behavior attached.
		if (GPIOGetDDI(BOTTOMLOAD_GPIO_DDI_WATER_PRESENCE_AFTER_BOTTLE_PIPE, &bTempVal) == false) return false;
		if (bTempVal != g_oBottomLoad.oInputData.bWaterPresenceAfterBottlePipe)
		{
			g_oBottomLoad.oInputData.bWaterPresenceAfterBottlePipe 	= bTempVal;
			g_oBottomLoad.u32TimestampWaterPresenceToggle			= FrameworkGetTime();
		}

	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		BottomLoadVerifyErrors
/// \details	Function that verifies the possible errors in the Bottom Load module.
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
static bool BottomLoadVerifyErrors(void)
{
	// Verify that both water level are not in an impossible state.
	if ((g_oBottomLoad.oInputData.bWaterLevelLowState == false) && (g_oBottomLoad.oInputData.bWaterLevelHighState == true))
	{
		if ((g_oBottomLoad.eState != BOTTOMLOAD_STATE_ERROR_LEVEL_SWITCH) || (g_oBottomLoad.eState != BOTTOMLOAD_STATE_ERROR_FLOOD))
		{
			g_oBottomLoad.u32CompteurLevelSwitch++;
			if (g_oBottomLoad.u32CompteurLevelSwitch >= 3)
			{
				g_oBottomLoad.eState = BOTTOMLOAD_STATE_ERROR_LEVEL_SWITCH;
				BottomLoadActivatePump(false, true);
				g_oBottomLoad.oPump.bIsAllowToStart = false;
				g_oBottomLoad.u32CompteurLevelSwitch = 0;
			}
		}
	}

	// Verify flooding.
	if ((g_oBottomLoad.oInputData.bWaterPresenceBottomFlood == true) || (g_oBottomLoad.oInputData.bWaterPresenceTopFlood == true))
	{
		if (g_oBottomLoad.eState != BOTTOMLOAD_STATE_ERROR_FLOOD)
		{
			// Ensure that the buzzer is activated.
			BottomLoadActivateBuzzer(true);
			
			// Deactivate all the rest.
			BottomLoadActivateHeater(false, true);
			BottomLoadActivateCompressor(false, true);
			BottomLoadActivatePump(false, true);
			
			g_oBottomLoad.oHeater.bIsAllowToStart 				= false;
			g_oBottomLoad.oCompressor.bIsAllowToStart 			= false;
			g_oBottomLoad.oPump.bIsAllowToStart 				= false;
			
			// Reset boost status.
			g_oBottomLoad.oStatus.bBoostModeTemperatureReached	= false;
			g_oBottomLoad.oStatus.bInBoostMode					= false;

			// If device is in flooding state, a manual reboot is required by the user.
			g_oBottomLoad.eState = BOTTOMLOAD_STATE_ERROR_FLOOD;
		}
	}
	
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		BottomLoadCheckStartupCondition
/// \details	At startup, wait for the button to be debounced and check if
///				the user want to go inside the diagnostic state or normal state.
/// \private
///
/// \param
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
static bool BottomLoadCheckStartupCondition(void)
{
	// Wait the buttons to be fully debounced.
	if (FrameworkTimeIsDue(g_oBottomLoad.u32TimestampStartup, BOTTOMLOAD_INITIAL_DEBOUNCING_PERIOD_MS) == true)
	{
		bool bBoostPressed = false;
		if (GPIOGetDDI(BOTTOMLOAD_GPIO_DDI_BUTTON_BOOST, &bBoostPressed) == false) return false;

		// Boost Button was active when starting AND No water -> Diag mode
		if ((bBoostPressed == true) && (TANK_IS_EMPTY() == true))
		{
			g_oBottomLoad.eState = BOTTOMLOAD_STATE_DIAGNOSTIC;
		}
		else // Everything is normal
		{
			g_oBottomLoad.eState = BOTTOMLOAD_STATE_NORMAL;
		}
	}
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		BottomLoadDiagnostic
/// \details	Will use variation of temperature over time to identify possible
/// 		    problems. Will use led to display the results.
///
///   | LED UI      | Problem                    | Logic                                                 |
///   | ----------- | -------------------------- | ----------------------------------------------------- |
///   | Y Blink x1  | INCORRECT_SENSOR		     | T <  10°C before CHRONO_1                             |
///   | Y Blink x2  | GAS_LEAK             	     | T >= 18°C after  CHRONO_2                             |
///   | Y Blink x3  | JUMPING_COMPRESSOR         | T <  18°C after  CHRONO_2                             |
///   | G Solid     | OK                         | T <  10°C after  CHRONO_1 && T < 10°C before CHRONO_2 |
///
/// \private
///
/// \param
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
static bool BottomLoadDiagnostic(void)
{
	switch (g_oBottomLoad.oDiagnostic.eState)
	{
		// Initialising diagnostic
		case BOTTOMLOAD_DIAGNOSTIC_STATE_INIT:
		{
			// Configure and start LED parttern so user know that the test is starting.
			if (LedDriverConfigLedDiscontinuousBlinkPattern(&g_oBottomLoad.oLed.oBlue,   3, 3, 400, 400, 1000) == false) return false;
			if (LedDriverConfigLedDiscontinuousBlinkPattern(&g_oBottomLoad.oLed.oGreen,  3, 3, 400, 400, 1000) == false) return false;
			if (LedDriverConfigLedDiscontinuousBlinkPattern(&g_oBottomLoad.oLed.oRed, 	3, 3, 400, 400, 1000) == false) return false;
			if (LedDriverConfigLedDiscontinuousBlinkPattern(&g_oBottomLoad.oLed.oYellow, 3, 3, 400, 400, 1000) == false) return false;

			// turn off green led
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oGreen,  LEDDRIVER_LED_MODE_OFF) == false) return false;
			if (LedDriverUpdateLed(&g_oBottomLoad.oLed.oGreen) == false)	return false;

			// Start blinking at the same time
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oBlue,   LEDDRIVER_LED_MODE_BLINK) == false) return false;
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oGreen,  LEDDRIVER_LED_MODE_BLINK) == false) return false;
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oRed,    LEDDRIVER_LED_MODE_BLINK) == false) return false;
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oYellow, LEDDRIVER_LED_MODE_BLINK) == false) return false;

			// Night led will blink continuously in diagnostic mode.
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oNight,  LEDDRIVER_LED_MODE_BLINK) == false) return false;
			//if (LedDriverConfigLedContinuousBlinkPattern(&g_oBottomLoad.oLed.oNight, 1, 500, 500, 0) == false) return false;

			// Start testing
			g_oBottomLoad.oDiagnostic.eResult = BOTTOMLOAD_DIAGNOSTIC_RESULT_UNKNOWN;
			g_oBottomLoad.oDiagnostic.u32TimestampTest = FrameworkGetTime();
			g_oBottomLoad.oDiagnostic.eState = BOTTOMLOAD_DIAGNOSTIC_STATE_TESTING_PHASE_1;
		} break;

		// Phase 1: Before chrono 1
		case BOTTOMLOAD_DIAGNOSTIC_STATE_TESTING_PHASE_1:
		{
			if (FrameworkTimeIsDue(g_oBottomLoad.oDiagnostic.u32TimestampTest, BOTTOMLOAD_DIAGNOSTIC_CHRONO_1_MS) == true)
			{
				// Check readout validity.
				if (g_oBottomLoad.oInputData.bColdWaterProbeOk == false)
				{
					g_oBottomLoad.oDiagnostic.eResult = BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_COLD_WATER_PROBE_READING;
					g_oBottomLoad.oDiagnostic.eState = BOTTOMLOAD_DIAGNOSTIC_STATE_WAIT_FOR_REBOOT; // Diagnostic is done. Now wait for reboot.
					break;
				}
				else if (g_oBottomLoad.oInputData.bColdWaterThresholdPotOk == false)
				{
					g_oBottomLoad.oDiagnostic.eResult = BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_COLD_WATER_THRESHOLD_POT_READING;
					g_oBottomLoad.oDiagnostic.eState = BOTTOMLOAD_DIAGNOSTIC_STATE_WAIT_FOR_REBOOT; // Diagnostic is done. Now wait for reboot.
					break;
				}
				else if (g_oBottomLoad.oInputData.bHotWaterProbeOk == false)
				{
					g_oBottomLoad.oDiagnostic.eResult = BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_HOT_WATER_PROBE_READING;
					g_oBottomLoad.oDiagnostic.eState = BOTTOMLOAD_DIAGNOSTIC_STATE_WAIT_FOR_REBOOT; // Diagnostic is done. Now wait for reboot.
					break;
				}
				else if (g_oBottomLoad.oInputData.bHotWaterThresholdPotOk == false)
				{
					g_oBottomLoad.oDiagnostic.eResult = BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_HOT_WATER_THRESHOLD_POT_READING;
					g_oBottomLoad.oDiagnostic.eState = BOTTOMLOAD_DIAGNOSTIC_STATE_WAIT_FOR_REBOOT; // Diagnostic is done. Now wait for reboot.
					break;
				}
				
				g_oBottomLoad.oDiagnostic.u32TimestampTest = FrameworkGetTime();
				g_oBottomLoad.oDiagnostic.eState = BOTTOMLOAD_DIAGNOSTIC_STATE_TESTING_PHASE_2; // Next phase
				BottomLoadActivateCompressor(true, true);
			}
		} break;
		
		// Phase 2: Before chrono 1
		case BOTTOMLOAD_DIAGNOSTIC_STATE_TESTING_PHASE_2:
		{
			// if Temperature is too cold before chrono 1
			if (g_oBottomLoad.oInputData.fColdWaterTemp < BOTTOMLOAD_DIAGNOSTIC_TEMP_THRESHOLD_COLD_C)
			{
				g_oBottomLoad.oDiagnostic.eResult = BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_SENSOR_READING;
				g_oBottomLoad.oDiagnostic.eState = BOTTOMLOAD_DIAGNOSTIC_STATE_WAIT_FOR_REBOOT; // Diagnostic is done. Now wait for reboot.
				break;
			}
			if (FrameworkTimeIsDue(g_oBottomLoad.oDiagnostic.u32TimestampTest, BOTTOMLOAD_DIAGNOSTIC_CHRONO_2_MS) == true)
			{
				g_oBottomLoad.oDiagnostic.eState = BOTTOMLOAD_DIAGNOSTIC_STATE_TESTING_PHASE_3; // Next phase
			}
		} break;

		// Phase 3: After chrono 1, Before chrono 2
		case BOTTOMLOAD_DIAGNOSTIC_STATE_TESTING_PHASE_3:
		{
			// if Temperature is cold enough before chrono 2
			if (g_oBottomLoad.oInputData.fColdWaterTemp < BOTTOMLOAD_DIAGNOSTIC_TEMP_THRESHOLD_COLD_C)
			{
				g_oBottomLoad.oDiagnostic.eResult = BOTTOMLOAD_DIAGNOSTIC_RESULT_OK;
				g_oBottomLoad.oDiagnostic.eState = BOTTOMLOAD_DIAGNOSTIC_STATE_WAIT_FOR_REBOOT; // Diagnostic is done. Now wait for reboot.
				break;
			}
			if (FrameworkTimeIsDue(g_oBottomLoad.oDiagnostic.u32TimestampTest, BOTTOMLOAD_DIAGNOSTIC_CHRONO_3_MS) == true)
			{
				g_oBottomLoad.oDiagnostic.eState = BOTTOMLOAD_DIAGNOSTIC_STATE_TESTING_PHASE_4; // Next phase
			}
		} break;

		// Phase 4: After chrono 1, After chrono 2
		case BOTTOMLOAD_DIAGNOSTIC_STATE_TESTING_PHASE_4:
		{
			if (g_oBottomLoad.oInputData.fColdWaterTemp < BOTTOMLOAD_DIAGNOSTIC_TEMP_THRESHOLD_HOT_C)
			{
				g_oBottomLoad.oDiagnostic.eResult = BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_JUMPING_COMPRESSOR;
			}
			else
			{
				 g_oBottomLoad.oDiagnostic.eResult = BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_GAS_LEAK;
			}
			g_oBottomLoad.oDiagnostic.eState = BOTTOMLOAD_DIAGNOSTIC_STATE_WAIT_FOR_REBOOT; // Diagnostic is done. Now wait for reboot.
		} break;

		// Wait for reboot.
		case BOTTOMLOAD_DIAGNOSTIC_STATE_WAIT_FOR_REBOOT:
		{
			BottomLoadActivateCompressor(false, false);
			break;
		} break;

		default: return false;
	}
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		BottomLoadProcHeating
/// \details	Function that processes the heating mechanism of the Bottom Load
/// \private
///
/// \param		bAllowBoost	Allow boost mode or not.
/// \PARAMETRES
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
static bool BottomLoadProcHeating(void)
{
	if(BOTTOMLOAD_TEMPERATURE_SENSOR_CHOICE == false)
	{
		// If probes are not ok, do not allow anything.
		if ((g_oBottomLoad.oInputData.bHotWaterProbeOk == false) || (g_oBottomLoad.oInputData.bHotWaterThresholdPotOk == false))
		{
			BottomLoadActivateCompressor(false, false);
			return true;
		}
		// Temperature logic. Under threshold, activate the compressor. g_oBottomLoad.oInputData.fColdWaterTemp
		// Redefine the thresholds for cold mode.
		if (g_oBottomLoad.fTempHotResult > (g_oBottomLoad.oInputData.fHotWaterTempThreshold + BOTTOMLOAD_TEMP_HOT_WATER_THRESHOLD_HYST_C))
		{
			if ((g_oBottomLoad.oCompressor.bIsActivated == false))
			{
				if(g_oBottomLoad.oPump.bIsActivated == true)
				{
					g_oBottomLoad.u32TimerCompressON = 0;
				}
				BottomLoadActivateCompressor(true, false);
				g_oBottomLoad.fTempHotActivComp = g_oBottomLoad.fTempHotResult;
				BottomLoadActivatePump(false, false);
			}
		}
		else if(g_oBottomLoad.fTempHotResult < (g_oBottomLoad.oInputData.fHotWaterTempThreshold - BOTTOMLOAD_TEMP_HOT_WATER_THRESHOLD_HYST_C)) // temperature is reached !
		{
			if(g_oBottomLoad.u32CompteurDelaiComp>=BOTTOMLOAD_COMPRESSOR_TEMP_REACHED_DELAY_MS)
			{
				g_oBottomLoad.u32TimerCompressOFF = 0;
				BottomLoadActivateCompressor(false, false);
				g_oBottomLoad.fTempHotDeactivComp = g_oBottomLoad.fTempHotResult;
				g_oBottomLoad.u32CompteurDelaiComp = 0;
				BottomLoadActivatePump(true, false);
			}
		}
	}
	return true;
}
////////////////////////////////////////////////////////////////////////////////*******************************************///////////
/// \brief 		BottomLoadProcChilling
/// \details	Function that processes the chilling mechanism of the Bottom Load.
/// \private
///
/// \PARAMETRES PARAMETERS TO CHANGE TO CHANGE SENSOR USED
/// \param		bHotWaterProbeOk | bHotWaterThresholdPotOk | fTempHotResult | fHotWaterTempThreshold
/// \param		bColdWaterProbeOk | bColdWaterThresholdPotOk | fTempColdResult | fColdWaterTempThreshold
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////********************************************/////////////
static bool BottomLoadProcChilling(void)
{
	if(BOTTOMLOAD_TEMPERATURE_SENSOR_CHOICE == true)
	{
		// If probes are not ok, do not allow anything.
		if ((g_oBottomLoad.oInputData.bColdWaterProbeOk == false) || (g_oBottomLoad.oInputData.bColdWaterThresholdPotOk == false))
		{
			BottomLoadActivateCompressor(false, false);
			return true;
		}
		// Temperature logic. Under threshold, activate the compressor. g_oBottomLoad.oInputData.fColdWaterTemp
		// Redefine the thresholds for cold mode.
		if (g_oBottomLoad.fTempColdResult > (g_oBottomLoad.oInputData.fColdWaterTempThreshold + BOTTOMLOAD_TEMP_COLD_WATER_THRESHOLD_HYST_C))
		{
			if ((g_oBottomLoad.oCompressor.bIsActivated == false))
			{
				g_oBottomLoad.u32TimerCompressON = 0;
				BottomLoadActivateCompressor(true, false);
				g_oBottomLoad.fTempColdActivComp = g_oBottomLoad.fTempColdResult;
				BottomLoadActivatePump(false, false);
			}
		}

		else if(g_oBottomLoad.fTempColdResult < (g_oBottomLoad.oInputData.fColdWaterTempThreshold - BOTTOMLOAD_TEMP_COLD_WATER_THRESHOLD_HYST_C)) // temperature is reached !
		{
			if(g_oBottomLoad.u32CompteurDelaiComp>=BOTTOMLOAD_COMPRESSOR_TEMP_REACHED_DELAY_MS)
			{
				if(g_oBottomLoad.oPump.bIsActivated == false)
				{
					g_oBottomLoad.u32TimerCompressOFF = 0;
				}
				BottomLoadActivateCompressor(false, false);
				BottomLoadActivatePump(true, false);
				g_oBottomLoad.fTempColdDeactivComp = g_oBottomLoad.fTempColdResult;
				g_oBottomLoad.u32CompteurDelaiComp = 0;
			}
		}
	}
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		BottomLoadProcPumping
/// \details	Function that processes the pumping mechanism of the Bottom Load.
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
static bool BottomLoadProcPumping(void)
{
	// Pumping is allowed only if the door is closed.
	if ((g_oBottomLoad.oInputData.bDoorIsOpen == true) || (TANK_IS_FULL() == true))
	{
		// Pumping is allowed only if the water tank is not full.
		//BottomLoadActivatePump(false, false);
		g_oBottomLoad.oStatus.bInFillingTheTankMode = false;
		return true;
	}

	//  task management: Fulling the tank
	if (g_oBottomLoad.oStatus.bInFillingTheTankMode == false)
	{
		// If we detect that the tank is empty, start filling it.
		if (TANK_IS_EMPTY() == true)
		{
			// Activate the pump if needed.
			//**BottomLoadActivatePump(true, false);
			g_oBottomLoad.oStatus.bInFillingTheTankMode = true;
			g_oBottomLoad.oPump.u8RetryCounter 			= 0;
		}
	}
	else
	{
		// If we are filling, monitor the water level after some time to see if we are pumping correctly.
		if ((g_oBottomLoad.oPump.bIsActivated == true) && (FrameworkTimeIsDue(g_oBottomLoad.oPump.u32TimestampActivation, BOTTOMLOAD_PUMPING_ON_TIME_CHECK_MS)))
		{
			// If the tank is still empty. Stop for a retry.
			if ((g_oBottomLoad.oInputData.bWaterPresenceAfterBottlePipe == false) && (FrameworkTimeIsDue(g_oBottomLoad.u32TimestampWaterPresenceToggle, BOTTOMLOAD_PUMPING_WATER_ABSENCE_CHECK_MS)))
			{
				//**BottomLoadActivatePump(false, false);
				
				// Check if we tried enough time.
				if (g_oBottomLoad.oPump.u8RetryCounter >= BOTTOMLOAD_PUMPING_NUMBER_OF_RETRY)
				{
					// Tank is still empty and no retry left -> change bottle
					g_oBottomLoad.eState = BOTTOMLOAD_STATE_BOTTLE_CHANGE;
					g_oBottomLoad.oStatus.bInFillingTheTankMode = false;
				}
			}
			else if (g_oBottomLoad.oInputData.bWaterPresenceAfterBottlePipe == true)
			{
				// Water presence, reset the retry counter.
				g_oBottomLoad.oPump.u8RetryCounter = 0;
			}
		}
		// If we stopped filling, retry when the time has come.
		else if ((g_oBottomLoad.oPump.bIsActivated == false) && (FrameworkTimeIsDue(g_oBottomLoad.oPump.u32TimestampDeactivation, BOTTOMLOAD_PUMPING_OFF_TIME_CHECK_MS)))
		{
			// Activate the pump for a retry.
			//**BottomLoadActivatePump(true, false);
			++g_oBottomLoad.oPump.u8RetryCounter;
		}
	}
	
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		BottomLoadUpdateLeds
/// \details	Function that updates all the LEDs according to the current state
///				of the module.
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
static bool BottomLoadUpdateLeds(void)
{
	// Update led configuration
	if (g_oBottomLoad.eState == BOTTOMLOAD_STATE_DIAGNOSTIC)
	{
		if (BottomLoadUpdateLedDiagnostic() == false) return false;
	}
	else // All other states
	{
		if (BottomLoadUpdateLedDefault() == false) return false;
	}

	// Change physical led states
	if (LedDriverUpdateLed(&g_oBottomLoad.oLed.oBlue) == false)		return false;
	if (LedDriverUpdateLed(&g_oBottomLoad.oLed.oGreen) == false)	return false;
	if (LedDriverUpdateLed(&g_oBottomLoad.oLed.oRed) == false)		return false;
	if (LedDriverUpdateLed(&g_oBottomLoad.oLed.oYellow) == false)	return false;
	if (LedDriverUpdateLed(&g_oBottomLoad.oLed.oNight) == false)	return false;
	
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		BottomLoadUpdateLedDiagnostic
/// \details	Will update led state to display the diagnostic result
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
static bool BottomLoadUpdateLedDiagnostic(void)
{
	uint8_t u8NumberOfBlink = 0;

	switch (g_oBottomLoad.oDiagnostic.eResult)
	{
		// No results yet to display.
		case BOTTOMLOAD_DIAGNOSTIC_RESULT_UNKNOWN: return true;
		
		// Everything is OK -> Config green light
		case BOTTOMLOAD_DIAGNOSTIC_RESULT_OK:
		{
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oGreen, LEDDRIVER_LED_MODE_ON) == false) return false;
		} break;
		
		case BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_COLD_WATER_PROBE_READING:
		case BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_COLD_WATER_THRESHOLD_POT_READING:
		{
			if (g_oBottomLoad.oDiagnostic.eResult == BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_COLD_WATER_PROBE_READING) 				u8NumberOfBlink = 1;
			else if (g_oBottomLoad.oDiagnostic.eResult == BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_COLD_WATER_THRESHOLD_POT_READING)	u8NumberOfBlink = 2;
				
			//if (LedDriverConfigLedContinuousBlinkPattern(&g_oBottomLoad.oLed.oBlue, u8NumberOfBlink, 100, 100, 500) == false) return false;
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oBlue,  LEDDRIVER_LED_MODE_BLINK)   == false) return false;
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oRed,  LEDDRIVER_LED_MODE_OFF)   == false) return false;
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oYellow, LEDDRIVER_LED_MODE_OFF) == false) return false;
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oGreen,  LEDDRIVER_LED_MODE_OFF)   == false) return false;
		} break;
		
		case BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_HOT_WATER_PROBE_READING:
		case BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_HOT_WATER_THRESHOLD_POT_READING:
		{
			if (g_oBottomLoad.oDiagnostic.eResult == BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_HOT_WATER_PROBE_READING) 				u8NumberOfBlink = 1;
			else if (g_oBottomLoad.oDiagnostic.eResult == BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_HOT_WATER_THRESHOLD_POT_READING)	u8NumberOfBlink = 2;
			
			//if (LedDriverConfigLedContinuousBlinkPattern(&g_oBottomLoad.oLed.oRed, u8NumberOfBlink, 100, 100, 500) == false) return false;
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oBlue,  LEDDRIVER_LED_MODE_OFF)   == false) return false;
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oRed,  LEDDRIVER_LED_MODE_BLINK)   == false) return false;
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oYellow, LEDDRIVER_LED_MODE_OFF) == false) return false;
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oGreen,  LEDDRIVER_LED_MODE_OFF)   == false) return false;
		} break;
		
		// There's a problem -> Config yellow light
		case BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_SENSOR_READING:
		case BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_GAS_LEAK:
		case BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_JUMPING_COMPRESSOR:
		{
			if (g_oBottomLoad.oDiagnostic.eResult == BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_SENSOR_READING) 			u8NumberOfBlink = 1;
			else if (g_oBottomLoad.oDiagnostic.eResult == BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_GAS_LEAK)			u8NumberOfBlink = 2;
			else if (g_oBottomLoad.oDiagnostic.eResult == BOTTOMLOAD_DIAGNOSTIC_RESULT_PROBLEM_JUMPING_COMPRESSOR)	u8NumberOfBlink = 3;
			
			//if (LedDriverConfigLedContinuousBlinkPattern(&g_oBottomLoad.oLed.oYellow, u8NumberOfBlink, 100, 100, 1000) == false) return false;
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oBlue,  LEDDRIVER_LED_MODE_OFF)   == false) return false;
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oRed,  LEDDRIVER_LED_MODE_OFF)   == false) return false;
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oYellow, LEDDRIVER_LED_MODE_BLINK) == false) return false;
			if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oGreen,  LEDDRIVER_LED_MODE_OFF)   == false) return false;
		}
	}

	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		BottomLoadUpdateLedDefault
/// \details	Will update led state depending on the machine state.
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
static bool BottomLoadUpdateLedDefault(void)
{
	if (g_oBottomLoad.eState == BOTTOMLOAD_STATE_ERROR_FLOOD)
	{
		// Blink both GREEN and YELLOW LEDs, critical error.
		if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oGreen, LEDDRIVER_LED_MODE_BLINK) == false) return false;
		if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oYellow, LEDDRIVER_LED_MODE_BLINK) == false) return false;
	}
	else if (g_oBottomLoad.eState == BOTTOMLOAD_STATE_ERROR_LEVEL_SWITCH)
	{
		// GREEN: ON when MCU is powered; OFF otherwise.
		if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oGreen, LEDDRIVER_LED_MODE_ON) == false) return false;
		
		// YELLOW: ON when system detect an error; OFF otherwise.
		if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oYellow, LEDDRIVER_LED_MODE_BLINK) == false) return false;
	}
	else
	{
		// GREEN: ON when MCU is powered; OFF otherwise.
		if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oGreen, LEDDRIVER_LED_MODE_ON) == false) return false;

		// YELLOW: ON when system detect an error; OFF otherwise.
		if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oYellow, LEDDRIVER_LED_MODE_OFF) == false) return false;
	}
	
	// BLUE: ON when bottle almost empty; BLINK when door is open; OFF otherwise.
	LedDriverLedMode_e eBlueLedMode;
	if      (g_oBottomLoad.oInputData.bDoorIsOpen == true)              eBlueLedMode = LEDDRIVER_LED_MODE_BLINK;
	else if (g_oBottomLoad.oInputData.bBottleIsEmpty == true) 			eBlueLedMode = LEDDRIVER_LED_MODE_ON;
	else if (g_oBottomLoad.eState == BOTTOMLOAD_STATE_BOTTLE_CHANGE)    eBlueLedMode = LEDDRIVER_LED_MODE_ON;
	else                                                                eBlueLedMode = LEDDRIVER_LED_MODE_OFF;
	if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oBlue, eBlueLedMode) == false) return false;

	// RED: ON when boost temperature is reached ; BLINK when BOOST MODE; OFF otherwise
	LedDriverLedMode_e eRedLedMode;
	if      (g_oBottomLoad.oStatus.bInBoostMode == true) 				 eRedLedMode = LEDDRIVER_LED_MODE_BLINK;
	else if (g_oBottomLoad.oStatus.bBoostModeTemperatureReached == true) eRedLedMode = LEDDRIVER_LED_MODE_ON;
	else																 eRedLedMode = LEDDRIVER_LED_MODE_OFF;
	if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oRed, eRedLedMode) == false) return false;

	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		BottomLoadActivatePump
/// \details	Function that activates/deactivates the pump peripheral.
/// \private
///
/// \param		bActivate: flag indicating if we activate the peripheral or not.
/// \param		bForce: Will activate/deactivate the pump, no matter its current state.
/// \return		None
////////////////////////////////////////////////////////////////////////////////
static void BottomLoadActivatePump(bool bActivate, bool bForce)
{
	// Active HIGH.
	if ((bActivate == true) && ((g_oBottomLoad.oPump.bIsActivated == false) || (bForce == true)))
	{
		if (g_oBottomLoad.oPump.bIsAllowToStart == true)
		{
			HAL_GPIO_WritePin(MCUMAP_DO_PUMP_LOW_PERIPH, MCUMAP_DO_PUMP_LOW_PIN, GPIO_PIN_SET);
			g_oBottomLoad.oPump.bIsActivated 			= true;
			g_oBottomLoad.oPump.u32TimestampActivation	= FrameworkGetTime();
		}
	}
	else if ((bActivate == false) && ((g_oBottomLoad.oPump.bIsActivated == true) || bForce == true))
	{
		HAL_GPIO_WritePin(MCUMAP_DO_PUMP_LOW_PERIPH, MCUMAP_DO_PUMP_LOW_PIN, GPIO_PIN_RESET);
		g_oBottomLoad.oPump.bIsActivated 			 = false;
		g_oBottomLoad.oPump.u32TimestampDeactivation = FrameworkGetTime();
	}
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		BottomLoadActivateHeater
/// \details	Function that activates/deactivates the heater peripheral.
/// \private
///
/// \param		bActivate: flag indicating if we activate the peripheral or not.
/// \param		bForce: Will activate/deactivate the heater, no matter its current state.
/// \return		None
////////////////////////////////////////////////////////////////////////////////

static void BottomLoadActivateHeater(bool bActivate, bool bForce)
{
	// Active LOW.
	if ((bActivate == true) && ((g_oBottomLoad.oCompressor.bIsActivated == false) || (bForce == true)))
	{
		if (g_oBottomLoad.oCompressor.bIsAllowToStart == true)
		{
			HAL_GPIO_WritePin(MCUMAP_DO_COMPRESSOR_PERIPH, MCUMAP_DO_COMPRESSOR_PIN, GPIO_PIN_RESET);
			g_oBottomLoad.oCompressor.bIsActivated = true;
		}
	}
	else if ((bActivate == false) && ((g_oBottomLoad.oCompressor.bIsActivated == true) || (bForce == true)))
	{
		HAL_GPIO_WritePin(MCUMAP_DO_COMPRESSOR_PERIPH, MCUMAP_DO_COMPRESSOR_PIN, GPIO_PIN_SET);
		g_oBottomLoad.oCompressor.u32TimestampLastDeactivation 	= FrameworkGetTime();
		g_oBottomLoad.oCompressor.bIsActivated 				    = false;
	}
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		BottomLoadActivateCompressor
/// \details	Function that activates/deactivates the compressor peripheral.
/// \private
///
/// \param		bActivate: flag indicating if we activate the peripheral or not.
/// \param		bForce: Will activate/deactivate the compressor, no matter its current state.
/// \return		None
////////////////////////////////////////////////////////////////////////////////
//////////////////////BottomLoadActivateCompressor2
static void BottomLoadActivateCompressor(bool bActivate, bool bForce)
{
	// Active LOW.
	if ((bActivate == true) && ((g_oBottomLoad.oCompressor.bIsActivated == false) || (bForce == true)))
	{
		if (g_oBottomLoad.oCompressor.bIsAllowToStart == true)
		{
			HAL_GPIO_WritePin(MCUMAP_DO_COMPRESSOR_PERIPH, MCUMAP_DO_COMPRESSOR_PIN, GPIO_PIN_RESET);
			g_oBottomLoad.oCompressor.bIsActivated = true;
		}
	}
	else if ((bActivate == false) && ((g_oBottomLoad.oCompressor.bIsActivated == true) || (bForce == true)))
	{
		HAL_GPIO_WritePin(MCUMAP_DO_COMPRESSOR_PERIPH, MCUMAP_DO_COMPRESSOR_PIN, GPIO_PIN_SET);
		g_oBottomLoad.oCompressor.u32TimestampLastDeactivation 	= FrameworkGetTime();
		g_oBottomLoad.oCompressor.bIsActivated 				    = false;
	}
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		BottomLoadActivateBuzzer
/// \details	Function that activates/deactivates the buzzer peripheral.
/// \private
///
/// \param		bActivate: flag indicating if we activate the peripheral or not.
/// \return		None
////////////////////////////////////////////////////////////////////////////////
static void BottomLoadActivateBuzzer(bool bActivate)
{
	if (bActivate == true)
	{
		MX_BUZZER_PWM_START();
	}
	else
	{
		MX_BUZZER_PWM_STOP();
	}
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		BottomLoadLUT2DLinearSearch
/// \details	Will perform a linear search to find the value in the 2D lookup
///				table. If the value doesnt directly exist, will return a linear
///				interpolation with the nearest value.
///				This function assume that the `rgu16LUT` is a sorted high to low
///				2D lookup table.
///				Complexity approximation:  ~O(N/2)
/// \private
///
/// \param		rgu16LUT[][2]: A 2D lookup table with `u16LUTLen` row, and 2 columns.
///						   	   Each row is a data pair, where column 0 is the
///						  	   dependent variable and Column 1 is the independent variable.
/// \param		u16LUTLen: Number of rows in the lookuptable.
/// \param		u16DependentVar[in]: The input dependent variable
/// \param		*pfIndependentVar[out]: The output independent variable.
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
static bool BottomLoadLUT2DLinearSearch(const int16_t rgu16LUT[][2], uint16_t u16LUTLen, uint16_t u16DependentVar, float *pfIndependentVar)
{
	const uint8_t 	DEPENDENT_VAR   = 0;
	const uint8_t 	INDEPENDENT_VAR = 1;
	uint16_t 		u16SearchFrom	= 0;
	uint16_t 		u16SearchTo		= 0;

	// Guards against invalid ADC value
	if (u16DependentVar >= rgu16LUT[0][DEPENDENT_VAR])
	{
		*pfIndependentVar = (float) rgu16LUT[0][INDEPENDENT_VAR];
		return true;
	}
	if (u16DependentVar <= rgu16LUT[u16LUTLen-1][DEPENDENT_VAR])
	{
		*pfIndependentVar = (float) rgu16LUT[u16LUTLen-1][INDEPENDENT_VAR];
		return true;
	}

	// Split in half: Speed optimisation.
	if (u16DependentVar > rgu16LUT[u16LUTLen/2][0])
	{
		u16SearchFrom = 0;
		u16SearchTo = u16LUTLen/2;
	}
	else
	{
		u16SearchFrom = u16LUTLen/2 - 1;
		u16SearchTo = u16LUTLen;
	}

	// Linear search of the half of the table
	for (uint16_t i = u16SearchFrom; i < u16SearchTo; i++)
	{
		int16_t u16XUpperBound = rgu16LUT[i][0];
		int16_t u16XLowerBound = rgu16LUT[i+1][0];
		int16_t u16YUpperBound = rgu16LUT[i][1];
		int16_t u16YLowerBound = rgu16LUT[i+1][1];

		if (u16DependentVar == u16XUpperBound) // If input is directly in the table
		{
			*pfIndependentVar = (float) u16YUpperBound; // No need for interpolation
			return true;
		}
		else if (u16DependentVar < u16XUpperBound && u16DependentVar > u16XLowerBound) // In between table value
		{
			*pfIndependentVar = BottomLoadLinearInterp((float) u16DependentVar, (float) u16XUpperBound, (float) u16YUpperBound, (float) u16XLowerBound, (float) u16YLowerBound);
			return true;
		}
	}

	return false;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		BottomLoadLinearInterp
/// \details	Will perform a linear interpolation.
/// \private
///
/// \param		fX:  The point to interpolate
/// \param		fXa: The bound A x value
/// \param		fYa: The bound A y value
/// \param		fXb: The bound B x value
/// \param		fYb: The bound B y value
/// \return		float: The interpolated value
////////////////////////////////////////////////////////////////////////////////
static inline float BottomLoadLinearInterp(float fX, float fXa, float fYa, float fXb, float fYb)
{
	// First order Taylor-Young formula
	return  fYa + (fX - fXa) * ( (fYb - fYa)/(fXb - fXa) );
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		BottomLoadLinearInterp
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
static inline float BottomLoadPotentiometer2Temp(uint16_t u16ADCValue, float i16MinTemp, float i16MaxTemp, uint16_t u16MinADC, uint16_t u16MaxADC)
{
	float result = ( (u16ADCValue*(i16MaxTemp - i16MinTemp)/(u16MaxADC - u16MinADC)) + i16MinTemp);

	if 		(result > i16MaxTemp) return i16MaxTemp; // gards against to high value
	else if (result < i16MinTemp) return i16MinTemp; // gards against to low value
	else return result;
}

////////////////////////////////////////////////////////////////////////////////
/// UNIT TESTS
////////////////////////////////////////////////////////////////////////////////
#if (BOTTOMLOAD_UNIT_TEST == 1)
#warning "BottomLoad unit test enable"

bool BottomLoadUTUpdateLEDs(void);
static bool BottomLoadUTLookupTable(void);
oLedDriverLed_t oUTDefaultLED;

////////////////////////////////////////////////////////////////////////////////
/// BottomLoadUnitTest
////////////////////////////////////////////////////////////////////////////////
bool BottomLoadUnitTest(void)
{
	printf("\r\n");
	printf("------------------------------------------------------- \r\n");
	printf("---- BOTTOMLOAD UNIT TESTS\r\n");
	printf("------------------------------------------------------- \r\n");

	BottomLoadUTUpdateLEDs();
	BottomLoadUTLookupTable();
	return true;
}


////////////////////////////////////////////////////////////////////////////////
/// BottomLoadUTUpdateLEDs
////////////////////////////////////////////////////////////////////////////////
bool BottomLoadUTUpdateLEDs(void)
{
	bool bResult = false;

	// Building setup
	g_oBottomLoad.eState 						    = BOTTOMLOAD_STATE_NORMAL;
	g_oBottomLoad.oInputData.bDoorIsOpen            = false;
	g_oBottomLoad.oInputData.fHotWaterTemp          = 30.0;
	g_oBottomLoad.oInputData.fHotWaterTempThreshold = 60.0;
	g_oBottomLoad.oStatus.bInBoostMode              = false;
	g_oBottomLoad.oInputData.bWaterLevelLowState 	= true;

	if (LedDriverConfigLedContinuousBlinkPattern(&g_oBottomLoad.oLed.oBlue,   2, 400, 400, 2000) == false) goto END;
	if (LedDriverConfigLedContinuousBlinkPattern(&g_oBottomLoad.oLed.oGreen,  2, 400, 400, 2000) == false) goto END;
	if (LedDriverConfigLedContinuousBlinkPattern(&g_oBottomLoad.oLed.oRed, 	 2, 400, 400, 2000) == false) goto END;
	if (LedDriverConfigLedContinuousBlinkPattern(&g_oBottomLoad.oLed.oYellow, 2, 400, 400, 2000) == false) goto END;
	if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oBlue,   LEDDRIVER_LED_MODE_BLINK) == false) goto END;
	if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oGreen,  LEDDRIVER_LED_MODE_BLINK) == false) goto END;
	if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oRed,    LEDDRIVER_LED_MODE_BLINK) == false) goto END;
	if (LedDriverSetModeLed(&g_oBottomLoad.oLed.oYellow, LEDDRIVER_LED_MODE_BLINK) == false) goto END;

	// Testing
	printf("\r\n*** BottomLoadUTUpdateLEDs ***\r\n");

	// GREEN LED
	printf("- Default mode\r\n");
	BottomLoadUpdateLeds();
	if (g_oBottomLoad.oLed.oBlue.eMode    != LEDDRIVER_LED_MODE_OFF &&
		g_oBottomLoad.oLed.oGreen.eMode  != LEDDRIVER_LED_MODE_ON &&
		g_oBottomLoad.oLed.oRed.eMode    != LEDDRIVER_LED_MODE_OFF &&
		g_oBottomLoad.oLed.oYellow.eMode != LEDDRIVER_LED_MODE_OFF) goto END;


	// BLUE LED
	printf("- Door is open mode\r\n");
	g_oBottomLoad.oInputData.bDoorIsOpen = true;
	BottomLoadUpdateLeds();
	if (g_oBottomLoad.oLed.oBlue.eMode    != LEDDRIVER_LED_MODE_BLINK ||
		g_oBottomLoad.oLed.oGreen.eMode  != LEDDRIVER_LED_MODE_ON ||
		g_oBottomLoad.oLed.oRed.eMode    != LEDDRIVER_LED_MODE_OFF ||
		g_oBottomLoad.oLed.oYellow.eMode != LEDDRIVER_LED_MODE_OFF) goto END;
	g_oBottomLoad.oInputData.bDoorIsOpen = false; //reset

	printf("- Empty bottle\r\n");
	g_oBottomLoad.oInputData.bDoorIsOpen = false;
	g_oBottomLoad.oInputData.bWaterLevelLowState = false;
	BottomLoadUpdateLeds();
	if (g_oBottomLoad.oLed.oBlue.eMode    != LEDDRIVER_LED_MODE_ON ||
		g_oBottomLoad.oLed.oGreen.eMode  != LEDDRIVER_LED_MODE_ON ||
		g_oBottomLoad.oLed.oRed.eMode    != LEDDRIVER_LED_MODE_OFF ||
		g_oBottomLoad.oLed.oYellow.eMode != LEDDRIVER_LED_MODE_OFF) goto END;
	g_oBottomLoad.oInputData.bDoorIsOpen = false; //reset
	g_oBottomLoad.oInputData.bWaterLevelLowState = true; // reset

	printf("- Empty bottle and door open\r\n");
	g_oBottomLoad.oInputData.bDoorIsOpen = true;
	g_oBottomLoad.oInputData.bWaterLevelLowState = false;
	BottomLoadUpdateLeds();
	if (g_oBottomLoad.oLed.oBlue.eMode    != LEDDRIVER_LED_MODE_BLINK || //should have priority
		g_oBottomLoad.oLed.oGreen.eMode  != LEDDRIVER_LED_MODE_ON ||
		g_oBottomLoad.oLed.oRed.eMode    != LEDDRIVER_LED_MODE_OFF ||
		g_oBottomLoad.oLed.oYellow.eMode != LEDDRIVER_LED_MODE_OFF) goto END;
	g_oBottomLoad.oInputData.bDoorIsOpen = false; //reset
	g_oBottomLoad.oInputData.bWaterLevelLowState = true; // reset

	// RED LED
	printf("- Temperature is too low\r\n");
	g_oBottomLoad.oInputData.fHotWaterTemp = 26.0;
	BottomLoadUpdateLeds();
	if (g_oBottomLoad.oLed.oBlue.eMode    != LEDDRIVER_LED_MODE_OFF ||
		g_oBottomLoad.oLed.oGreen.eMode  != LEDDRIVER_LED_MODE_ON ||
		g_oBottomLoad.oLed.oRed.eMode    != LEDDRIVER_LED_MODE_OFF ||
		g_oBottomLoad.oLed.oYellow.eMode != LEDDRIVER_LED_MODE_OFF) goto END;

	printf("- Temperature is ok\r\n");
	g_oBottomLoad.oInputData.fHotWaterTemp = 80.0;
	BottomLoadUpdateLeds();
	if (g_oBottomLoad.oLed.oBlue.eMode    != LEDDRIVER_LED_MODE_OFF ||
		g_oBottomLoad.oLed.oGreen.eMode  != LEDDRIVER_LED_MODE_ON ||
		g_oBottomLoad.oLed.oRed.eMode    != LEDDRIVER_LED_MODE_ON ||
		g_oBottomLoad.oLed.oYellow.eMode != LEDDRIVER_LED_MODE_OFF) goto END;

	printf("- Boost mode is active\r\n");
	g_oBottomLoad.oStatus.bInBoostMode = true;
	g_oBottomLoad.oInputData.fHotWaterTemp = 26.0;
	BottomLoadUpdateLeds();
	if (g_oBottomLoad.oLed.oBlue.eMode    != LEDDRIVER_LED_MODE_OFF ||
		g_oBottomLoad.oLed.oGreen.eMode  != LEDDRIVER_LED_MODE_ON ||
		g_oBottomLoad.oLed.oRed.eMode    != LEDDRIVER_LED_MODE_BLINK ||
		g_oBottomLoad.oLed.oYellow.eMode != LEDDRIVER_LED_MODE_OFF) goto END;
	g_oBottomLoad.oStatus.bInBoostMode = false; //reset

	printf("- Boost mode is activated and the temperature go to 26 to 80\r\n");
	g_oBottomLoad.oInputData.fHotWaterTemp = 26.0;
	g_oBottomLoad.oStatus.bInBoostMode = false;
	BottomLoadUpdateLeds();
	if (g_oBottomLoad.oLed.oBlue.eMode    != LEDDRIVER_LED_MODE_OFF ||
		g_oBottomLoad.oLed.oGreen.eMode  != LEDDRIVER_LED_MODE_ON ||
		g_oBottomLoad.oLed.oRed.eMode    != LEDDRIVER_LED_MODE_OFF ||
		g_oBottomLoad.oLed.oYellow.eMode != LEDDRIVER_LED_MODE_OFF) goto END;
	g_oBottomLoad.oStatus.bInBoostMode = true;  // User activate boost mode
	BottomLoadUpdateLeds();
	if (g_oBottomLoad.oLed.oBlue.eMode    != LEDDRIVER_LED_MODE_OFF ||
		g_oBottomLoad.oLed.oGreen.eMode  != LEDDRIVER_LED_MODE_ON ||
		g_oBottomLoad.oLed.oRed.eMode    != LEDDRIVER_LED_MODE_BLINK ||
		g_oBottomLoad.oLed.oYellow.eMode != LEDDRIVER_LED_MODE_OFF) goto END;
	g_oBottomLoad.oInputData.fHotWaterTemp = 80.0; // temperature is now ok
	BottomLoadProcHeating(); //
	BottomLoadUpdateLeds();
	if (g_oBottomLoad.oLed.oBlue.eMode    != LEDDRIVER_LED_MODE_OFF ||
		g_oBottomLoad.oLed.oGreen.eMode  != LEDDRIVER_LED_MODE_ON ||
		g_oBottomLoad.oLed.oRed.eMode    != LEDDRIVER_LED_MODE_ON ||
		g_oBottomLoad.oLed.oYellow.eMode != LEDDRIVER_LED_MODE_OFF) goto END;
	g_oBottomLoad.oInputData.fHotWaterTemp = 26.0;	// reset
	g_oBottomLoad.oStatus.bInBoostMode = false;	// reset


	// YELLOW LED
	printf("- No H20 problem is detected\r\n");
	g_oBottomLoad.eState = BOTTOMLOAD_STATE_NORMAL;
	BottomLoadUpdateLeds();
	if (g_oBottomLoad.oLed.oBlue.eMode    != LEDDRIVER_LED_MODE_OFF ||
		g_oBottomLoad.oLed.oGreen.eMode  != LEDDRIVER_LED_MODE_ON ||
		g_oBottomLoad.oLed.oRed.eMode    != LEDDRIVER_LED_MODE_OFF ||
		g_oBottomLoad.oLed.oYellow.eMode != LEDDRIVER_LED_MODE_OFF) goto END;

	printf("- A H20 problem is detected\r\n");
	g_oBottomLoad.eState = BOTTOMLOAD_STATE_ERROR;
	BottomLoadUpdateLeds();
	if (g_oBottomLoad.oLed.oBlue.eMode    != LEDDRIVER_LED_MODE_OFF ||
		g_oBottomLoad.oLed.oGreen.eMode  != LEDDRIVER_LED_MODE_ON ||
		g_oBottomLoad.oLed.oRed.eMode    != LEDDRIVER_LED_MODE_OFF ||
		g_oBottomLoad.oLed.oYellow.eMode != LEDDRIVER_LED_MODE_ON) goto END;
	g_oBottomLoad.oStatus.bInBoostMode = false; //reset


	bResult = true;
END:
	if (bResult)
	{
		printf("----> BottomLoadUTUpdateLEDs - PASS.\r\n");
	}
	else
	{
		printf("----> BottomLoadUTUpdateLEDs - FAIL.\r\n");
	}
	return bResult;
}

////////////////////////////////////////////////////////////////////////////////
/// BottomLoadUTLookupTable
////////////////////////////////////////////////////////////////////////////////
static bool BottomLoadUTLookupTable(void)
{
	bool bResult = false;

	// Building setup
	float fTemperature = 0.0;

	// Testing
	printf("\r\n*** BottomLoadUTLookupTable ***\r\n");

	// Cold water temperature

	printf("-- TEST: exact input positive \r\n");
	if (BottomLoadLUT2DLinearSearch(rg16BottomLoadLUTColdWater,
									BOTTOMLOAD_LUT_COLD_WATER_SIZE,
									2085,
									&fTemperature) == false) goto END;
	if (fTemperature != (float) 24.0) goto END;

	printf("-- TEST: exact input negative \r\n");
	if (BottomLoadLUT2DLinearSearch(rg16BottomLoadLUTColdWater,
									BOTTOMLOAD_LUT_COLD_WATER_SIZE,
									3130,
									&fTemperature) == false) goto END;
	if (fTemperature != (float) -4.0) goto END;

	printf("-- TEST: Interpolation\r\n");
	if (BottomLoadLUT2DLinearSearch(rg16BottomLoadLUTColdWater,
									BOTTOMLOAD_LUT_COLD_WATER_SIZE,
									2600,
									&fTemperature) == false) goto END;
	if (fTemperature != (float) 10.789473684211) goto END;

	printf("-- TEST: Input too high\r\n");
	if (BottomLoadLUT2DLinearSearch(rg16BottomLoadLUTColdWater,
									BOTTOMLOAD_LUT_COLD_WATER_SIZE,
									3200,
									&fTemperature) == false) goto END;
	if (fTemperature != (float) -5.0) goto END;

	printf("-- TEST: Input too low\r\n");
	if (BottomLoadLUT2DLinearSearch(rg16BottomLoadLUTColdWater,
									BOTTOMLOAD_LUT_COLD_WATER_SIZE,
									360,
									&fTemperature) == false) goto END;
	if (fTemperature != (float) 28.0) goto END;

	printf("-- TEST: Near the max edge\r\n");
	if (BottomLoadLUT2DLinearSearch(rg16BottomLoadLUTColdWater,
									BOTTOMLOAD_LUT_COLD_WATER_SIZE,
									3162,
									&fTemperature) == false) goto END;
	if (fTemperature != (float) -4.969696969697) goto END;

	printf("-- TEST: Near the min edge\r\n");
	if (BottomLoadLUT2DLinearSearch(rg16BottomLoadLUTColdWater,
									BOTTOMLOAD_LUT_COLD_WATER_SIZE,
									1934,
									&fTemperature) == false) goto END;
	if (fTemperature != (float) 27.973684) goto END;

	printf("-- TEST: The exact center part 1\r\n");
	if (BottomLoadLUT2DLinearSearch(rg16BottomLoadLUTColdWater,
									BOTTOMLOAD_LUT_COLD_WATER_SIZE,
									2573,
									&fTemperature) == false) goto END;
	if (fTemperature != (float) 11.487179487179) goto END;

		printf("-- TEST: The exact center part 2\r\n");
	if (BottomLoadLUT2DLinearSearch(rg16BottomLoadLUTColdWater,
									BOTTOMLOAD_LUT_COLD_WATER_SIZE,
									2572,
									&fTemperature) == false) goto END;
	if (fTemperature != (float) 11.512820512821) goto END;

	// Evaluate
	bResult = 1;
END:
	if (bResult)
	{
		printf("----> BottomLoadUTLookupTable - PASS.\r\n");
	}
	else
	{
		printf("   [!]\t Incorrect answer: %f\r\n", fTemperature);
		printf("----> BottomLoadUTLookupTable - FAIL.\r\n");
	}
	return bResult;
}

#endif /*Unit tests*/
