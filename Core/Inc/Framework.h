///
/// \file 		Framework.h
/// \brief 		[Header file]
///				
/// \author 	NOVO
///
#ifndef __FRAMEWORK_H_
#define __FRAMEWORK_H_


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
#define FRAMEWORK_INTERNAL_WATCHDOG_ENABLED		1	///< Enable switch for the internal watchdog.
#define FRAMEWORK_UNIT_TEST						0	///< Module unit test activation.

// Blocking Unit Tests
#if (FRAMEWORK_UNIT_TEST == 1)
#define FRAMEWORK_ADC_UNIT_TEST					0	///< Blocking test, mutually exclusive. ADC test.
#endif


///
/// \enum	FrameworkWDComp_e
/// \brief	Watchdog components that need to be signaled before a watchdog feed (using FrameworkSignalWDComp).
///
typedef enum
{
	FRAMEWORK_WD_COMPONENT_MAIN				= 0x0001,	///< Watchdog component, main task.
	
} FrameworkWDComp_e;

///
/// \enum	FrameworkModelID_e
/// \brief	Enumeration of the detected product model.
///
typedef enum
{
	FRAMEWORK_MODEL_ID_BOTTOMLOAD	= 0,	///< Product model, Bottom Load (hardware detected).
	FRAMEWORK_MODEL_ID_ICEBOXX,		   		///< Product model, Iceboxx (hardware detected).
	FRAMEWORK_MODEL_ID_HOTDEGREE,			///< Product model, Hot Degree (hardware detected).
	FRAMEWORK_MODEL_ID_CO2POU,				///< Product model, Co2 POU (hardware detected).
	FRAMEWORK_MODEL_ID_CO2BOTTLE,			///< Product model, Co2 Bottle (hardware detected).
	FRAMEWORK_MODEL_ID_BASE	= 0,			///< Product model, Bottom Load (hardware detected).
	FRAMEWORK_MODEL_ID_MEDIAN,		   		///< Product model, Iceboxx (hardware detected).
	FRAMEWORK_MODEL_ID_FULL,				///< Product model, Hot Degree (hardware detected).
	FRAMEWORK_MODEL_ID_NOT_USE_1,			///< Product model, Co2 POU (hardware detected).
	FRAMEWORK_MODEL_ID_NOT_USE_2,			///< Product model, Co2 Bottle (hardware detected).
	FRAMEWORK_MODEL_ID_UNKNOWN,				///< Product model, unknown (hardware detected).

} FrameworkModelID_e;

	
////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
bool		FrameworkInit(void);
bool		FrameworkTask(void);
void		FrameworkFatalErrorTask(void);
void		FrameworkSignalWDComp(FrameworkWDComp_e eWDCompToSignal);

// Version management
bool		FrameworkGetHWVersion(uint8_t* pu8HWVerMajor, uint8_t* pu8HWVerMinor);
bool		FrameworkGetFWVersion(uint8_t* pu8FWVerMajor, uint8_t* pu8FWVerMinor, uint8_t* pu8FWVerRev);
bool		FrameworkGetModelID(FrameworkModelID_e* pModelID);

// Data manipulation
bool		FrameworkExtract32bits(uint32_t * pu32Dst, uint8_t * pu8Src);

// Time Management
uint32_t	FrameworkGetTime(void);
void 		FrameworkTimeDelay(uint32_t u32DelayInMs);
uint32_t	FrameworkGetTimeDiff(uint32_t u32Compare);
bool		FrameworkTimeIsDue(uint32_t u32Compare, uint32_t u32Interval);
void		FrameworkSystemTimeISR(void);

// Unit test
#if (FRAMEWORK_UNIT_TEST == 1)
	bool FrameworkUnitTest(void);
#endif

#endif // __FRAMEWORK_H_
