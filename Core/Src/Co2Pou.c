///
/// \file 		Co2Pou.c
/// \brief 		[Source file]
///				
/// \author 	NOVO
///
////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include "Co2Pou.h"

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
/// \brief 		Co2PouInit
/// \details	Initialization of the module.
///           
/// \public
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool Co2PouInit(void)
{
	
	return true;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief 		Co2PouTask
/// \details	Execution of the main task of the module.
///           
/// \public
/// \param		None
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool Co2PouTask(void)
{

	return true;
}


////////////////////////////////////////////////////////////////////////////////
/// UNIT TESTS
////////////////////////////////////////////////////////////////////////////////
#if (CO2POU_UNIT_TEST == 1)
#warning "Co2Pou unit test enable"

////////////////////////////////////////////////////////////////////////////////
/// Co2PouUnitTest
////////////////////////////////////////////////////////////////////////////////
bool Co2PouUnitTest(void)
{
	printf("\r\n");
	printf("------------------------------------------------------- \r\n");
	printf("---- CO2POU UNIT TESTS\r\n");
	printf("------------------------------------------------------- \r\n");
	
	return true;
}
#endif