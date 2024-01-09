///
/// \file 		CommManager.h
/// \brief 		[Header file]
///				
/// \author 	NOVO
///
#ifndef __COMMMANAGER_H_
#define __COMMMANAGER_H_


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
#define COMMMANAGER_UNIT_TEST				0		///< Module unit test activation.

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
bool	CommManagerInit(void);
bool	CommManagerTask(void);


// Unit test
#if (COMMMANAGER_UNIT_TEST == 1)
	bool CommManagerUnitTest(void);
#endif

#endif // __COMMMANAGER_H_
