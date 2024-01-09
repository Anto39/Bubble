///
/// \file 		CommManager.c
/// \brief 		[Source file]
///				
/// \author 	NOVO
///
////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include "CommManager.h"

#include <stdio.h>
#include <string.h>


////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Private functions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Private variables
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/// \brief 		CommManagerInit
/// \details	Initialization of the module.
///           
/// \public
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool CommManagerInit(void)
{
	
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		CommManagerTask
/// \details	Execution of the main task of the module.
///           
/// \public
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool CommManagerTask(void)
{

	return true;
}


////////////////////////////////////////////////////////////////////////////////
/// UNIT TESTS
////////////////////////////////////////////////////////////////////////////////
#if (COMMMANAGER_UNIT_TEST == 1)
#warning "CommManager unit test enable"

////////////////////////////////////////////////////////////////////////////////
/// CommManagerUnitTest
////////////////////////////////////////////////////////////////////////////////
bool CommManagerUnitTest(void)
{
	printf("\r\n");
	printf("------------------------------------------------------- \r\n");
	printf("---- COMM MANAGER UNIT TESTS\r\n");
	printf("------------------------------------------------------- \r\n");
	
	return true;
}
#endif
