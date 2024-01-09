///
/// \file 		Co2Bottle.c
/// \brief 		[Source file]
///				
/// \author 	NOVO
///
////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <gpio.h>
#include "Co2Bottle.h"

#include <stdio.h>
#include <string.h>
#include "MCUMap.h"
#include "LedDriver.h"
#include "tim.h"
#include "adc.h"
#include "Framework.h"

////////////////////////////////////////////////////////////////////////////////
// GPIO Pins Mapping
////////////////////////////////////////////////////////////////////////////////
#define CO2BOTTLE_HP_PUMP_PORT			  			(MCUMAP_DO_PUMP_HIGH_PERIPH)
#define CO2BOTTLE_HP_PUMP_PIN  						(MCUMAP_DO_PUMP_HIGH_PIN)

////////////////////////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////////////////////////

// Time constants
#define CO2BOTTLE_BOTTLE_LEVEL_PIN					(15000)		///< Max filling duration
#define Co2Bottle_CONTROL_PERIOD_MS					(30)		///< Period for Bottom Load control actions. In ms.


// Usefull macro
#define TANK_IS_EMPTY() (((g_oCo2Bottle.oInputData.bCO2WaterLevelLowState == false) && (g_oCo2Bottle.oInputData.bCO2WaterLevelHighState == false)) ? true : false)
#define TANK_IS_FULL()  (((g_oCo2Bottle.oInputData.bCO2WaterLevelLowState == true)  && (g_oCo2Bottle.oInputData.bCO2WaterLevelHighState == true))  ? true : false)

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
///
/// \enum	eCo2BottleState_t
/// \brief	Enumeration of the possible states of the module.
///
typedef enum
{
	Co2Bottle_STANDBY = 0,					///< Always the first state after reboot. Will wait for debouncing data to be valid.
	Co2Bottle_PUMPING,						///< PUMP ACTIVE
	Co2Bottle_STATE_ERROR_LEVEL_SWITCH,	///< Impossible state level switch
} eCo2BottleState_t;

///
/// \struct	oCo2BottleInputData_t
/// \brief	Structure containing all the data retrieved from digital and/or analog inputs.
///
typedef struct
{
	bool	bCO2WaterLevelLowState;				///< If there's water in lower part of the main tank. Set by a 2 probes connectivity detector.
	bool	bCO2WaterLevelHighState;			///< If there's water in higher part of the main tank. Set by a 2 probes connectivity detector.
} oCo2BottleInputData_t;

///
/// \struct	oCo2BottlePump_t
/// \brief	Structure containing pump specific variables.
///
typedef struct
{
	bool				bIsActivated;					///< Flag indicating if the pump is currently activated or not.
	bool 				bIsAllowToStart;				///< Flag indicating if the pump has the permission to start. (Useful at when errors)
	uint8_t				u8RetryCounter;					///< Keep track of the number of retry done in the retry mechanism.
	uint32_t			u32TimestampActivation;			///< Timestamp of the last activation, in ms.
	uint32_t			u32TimestampDeactivation;	    ///< Timestamp of the last deactivation, in ms.
} oCo2BottlePump_t;

///
/// \struct	oCo2Bottle_t
/// \brief	Structure containing this module specific variables.
///
typedef struct
{
	eCo2BottleState_t			eState;						///< Main state-machine state of the Bottom Load model.
	oCo2BottleInputData_t		oInputData;					///< Contains all the input data of the Bottom Load model.
	oCo2BottlePump_t			oPump;						///< Pump peripheral of the Bottom Load model.
	uint32_t					u32TimestampControl;		///< Timestamp used to control (periodically) the Bottom Load actions.
	uint32_t					Counter;
} oCo2Bottle_t;


////////////////////////////////////////////////////////////////////////////////
// Private functions
////////////////////////////////////////////////////////////////////////////////
static void Co2BottleActivatePump(bool bActivate, bool bForce);
static bool Co2BottleVerifyErrors(void);
static bool Co2BottleUpdateData(void);

////////////////////////////////////////////////////////////////////////////////
// Private variables
////////////////////////////////////////////////////////////////////////////////
oCo2Bottle_t	g_oCo2Bottle;					///< Main Bottom Load object.

////////////////////////////////////////////////////////////////////////////////
/// \brief 		Co2BottleInit
/// \details	Initialization of the module.
///           
/// \public
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool Co2BottleInit(void)
{
	g_oCo2Bottle.eState									= Co2Bottle_STANDBY;

	g_oCo2Bottle.u32TimestampControl					= 0;

	g_oCo2Bottle.oInputData.bCO2WaterLevelLowState		= false;
	g_oCo2Bottle.oInputData.bCO2WaterLevelHighState		= false;

	g_oCo2Bottle.oPump.bIsAllowToStart					= true;
	g_oCo2Bottle.oPump.bIsActivated						= false;
	g_oCo2Bottle.Counter								= 0;

	Co2BottleActivatePump(false, true);

	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		Co2BottleTask
/// \details	Execution of the main task of the module.
///           
/// \public
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool Co2BottleTask(void)
{
	// Retrive all information.
	if (Co2BottleUpdateData() == false) return false;
	//Verify errors and abherations.
	if (Co2BottleVerifyErrors() == false) return false;

	if (FrameworkTimeIsDue(g_oCo2Bottle.u32TimestampControl, Co2Bottle_CONTROL_PERIOD_MS) == true)
	{
		g_oCo2Bottle.u32TimestampControl = FrameworkGetTime();

		// Process according to the current state.
		switch (g_oCo2Bottle.eState)
		{
			// STANDBY state (Always start here)
			case Co2Bottle_STANDBY:
			{
				//if(TANK_IS_EMPTY() == true)
				if((g_oCo2Bottle.oInputData.bCO2WaterLevelLowState == false) && (g_oCo2Bottle.oInputData.bCO2WaterLevelHighState == false))
				{
					Co2BottleActivatePump(true, false);
					g_oCo2Bottle.eState = Co2Bottle_PUMPING;
				}
			} break;

			// PUMP ON state.
			case Co2Bottle_PUMPING:
			{
				//if(TANK_IS_FULL() == true)
				if((g_oCo2Bottle.oInputData.bCO2WaterLevelLowState == true) && (g_oCo2Bottle.oInputData.bCO2WaterLevelHighState == true))
				{
					Co2BottleActivatePump(false, false);
					g_oCo2Bottle.eState = Co2Bottle_STANDBY;
				}
			} break;

			// Error state
			case Co2Bottle_STATE_ERROR_LEVEL_SWITCH:
			{
				// Only pump is deactivated in this error state.
				g_oCo2Bottle.oPump.bIsAllowToStart = false;

				if ((g_oCo2Bottle.oInputData.bCO2WaterLevelLowState != false) && (g_oCo2Bottle.oInputData.bCO2WaterLevelHighState != true))
				{
					g_oCo2Bottle.oPump.bIsAllowToStart = true;
					g_oCo2Bottle.eState = Co2Bottle_STANDBY;
				}
			} break;

			default: return false;
		}
	}

	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		Co2BottleActivatePump
/// \details	Function that activates/deactivates the pump peripheral.
/// \private
///
/// \param		bActivate: flag indicating if we activate the peripheral or not.
/// \param		bForce: Will activate/deactivate the pump, no matter its current state.
/// \return		None
////////////////////////////////////////////////////////////////////////////////
static void Co2BottleActivatePump(bool bActivate, bool bForce)
{
	// Active HIGH.
	if ((bActivate == true) && ((g_oCo2Bottle.oPump.bIsActivated == false) || (bForce == true)))
	{
		if (g_oCo2Bottle.oPump.bIsAllowToStart == true)
		{
			HAL_GPIO_WritePin(CO2BOTTLE_HP_PUMP_PORT, CO2BOTTLE_HP_PUMP_PIN, GPIO_PIN_SET);
			g_oCo2Bottle.oPump.bIsActivated 			= true;
			g_oCo2Bottle.oPump.u32TimestampActivation	= FrameworkGetTime();
		}
	}
	else if ((bActivate == false) && ((g_oCo2Bottle.oPump.bIsActivated == true) || bForce == true))
	{
		HAL_GPIO_WritePin(CO2BOTTLE_HP_PUMP_PORT, CO2BOTTLE_HP_PUMP_PIN, GPIO_PIN_RESET);
		g_oCo2Bottle.oPump.bIsActivated 			 = false;
		g_oCo2Bottle.oPump.u32TimestampDeactivation = FrameworkGetTime();
	}
}

////////////////////////////////////////////////////////////////////////////////
/// \brief 		Co2BottleVerifyErrors
/// \details	Function that verifies the possible errors in the Bottom Load module.
/// \private
///
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
static bool Co2BottleVerifyErrors(void)
{
	// Verify that both water level are not in an impossible state.
	if ((g_oCo2Bottle.oInputData.bCO2WaterLevelLowState == false) && (g_oCo2Bottle.oInputData.bCO2WaterLevelHighState == true))
	{
		if (g_oCo2Bottle.eState != Co2Bottle_STATE_ERROR_LEVEL_SWITCH)
		{
			g_oCo2Bottle.eState = Co2Bottle_STATE_ERROR_LEVEL_SWITCH;
			Co2BottleActivatePump(false, true);
			g_oCo2Bottle.oPump.bIsAllowToStart = false;
		}
	}
	return true;
}

static bool Co2BottleUpdateData(void)
{
	// Digital inputs.
	if (GPIOGetDDI(GPIO_DDI_LEVEL_ICE, &g_oCo2Bottle.oInputData.bCO2WaterLevelLowState) == false) return false;
	if (GPIOGetDDI(GPIO_DDI_PRESENCE_ICE, &g_oCo2Bottle.oInputData.bCO2WaterLevelHighState) == false) return false;
	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// UNIT TESTS
////////////////////////////////////////////////////////////////////////////////
#if (CO2BOTTLE_UNIT_TEST == 1)
#warning "Co2Bottle unit test enable"

////////////////////////////////////////////////////////////////////////////////
/// Co2BottleUnitTest
////////////////////////////////////////////////////////////////////////////////
bool Co2BottleUnitTest(void)
{
	printf("\r\n");
	printf("------------------------------------------------------- \r\n");
	printf("---- CO2BOTTLE UNIT TESTS\r\n");
	printf("------------------------------------------------------- \r\n");
	
	return true;
}
#endif
