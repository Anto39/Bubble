///
/// \file 		UTLookUpTable.c
/// \brief 		[Source file]
///
/// \author 	NOVO - xavier Boucher, ing. 5022396
///
////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <gpio.h>
#include "UTLookUpTable.h"

#include <stdio.h>
#include <string.h>
#include "Framework.h"
#include "MCUMap.h"
#include "LedDriver.h"
#include "tim.h"


////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

// New Look-up Table Hot Sensor input (ADAPTED FOR COLD WATER)
#define UTLOOKUPTABLE_LUT_HOT_WATER_SIZE 	106					///< Look-up table size for the hot water ADC count to temperature conversion.
static const int16_t rg16BottomLoadLUTHotWater[UTLOOKUPTABLE_LUT_HOT_WATER_SIZE][2] = {		///< Look-up table for the hot water ADC count to temperature conversion.
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

#define UTLOOKUPTABLE_LUT_COLD_WATER_SIZE 	146		///< Look-up table size for the cold water ADC count to temperature conversion.
static const int16_t rg16BottomLoadLUTColdWater[UTLOOKUPTABLE_LUT_COLD_WATER_SIZE][2] = {	///< Look-up table for the cold water ADC count to temperature conversion.
	{3975, -40}, {3967, -39}, {3959, -38}, {3950, -37}, {3941, -36},
	{3931, -35}, {3921, -34}, {3910, -33}, {3899, -32}, {3887, -31},
	{3874, -30}, {3861, -29}, {3848, -28}, {3833, -27}, {3818, -26},
	{3802, -25}, {3786, -24}, {3769, -23}, {3750, -22}, {3732, -21},
	{3712, -20}, {3691, -19}, {3670, -18}, {3648, -17}, {3624, -16},
	{3600, -15}, {3575, -14}, {3550, -13}, {3523, -12}, {3495, -11},
	{3466, -10}, {3437,  -9}, {3407,  -8}, {3375,  -7}, {3343,  -6},
	{3310,  -5}, {3276,  -4}, {3242,  -3}, {3206,  -2}, {3170,  -1},
	{3133,   0}, {3095,   1}, {3056,   2}, {3017,   3}, {2977,   4},
	{2936,   5}, {2895,   6}, {2853,   7}, {2810,   8}, {2768,   9},
	{2724,  10}, {2680,  11}, {2636,  12}, {2592,  13}, {2547,  14},
	{2502,  15}, {2456,  16}, {2411,  17}, {2365,  18}, {2320,  19},
	{2274,  20}, {2228,  21}, {2183,  22}, {2137,  23}, {2092,  24},
	{2047,  25}, {2002,  26}, {1958,  27}, {1913,  28}, {1870,  29},
	{1826,  30}, {1783,  31}, {1741,  32}, {1699,  33}, {1657,  34},
	{1616,  35}, {1576,  36}, {1536,  37}, {1497,  38}, {1458,  39},
	{1420,  40}, {1383,  41}, {1347,  42}, {1311,  43}, {1276,  44},
	{1242,  45}, {1208,  46}, {1175,  47}, {1143,  48}, {1111,  49},
	{1081,  50}, {1051,  51}, {1021,  52}, {993 ,  53}, {965 ,  54},
	{937 ,  55}, {911 ,  56}, {885 ,  57}, {860 ,  58}, {835 ,  59},
	{811 ,  60}, {788 ,  61}, {766 ,  62}, {744 ,  63}, {722 ,  64},
	{701 ,  65}, {681 ,  66}, {662 ,  67}, {642 ,  68}, {624 ,  69},
	{606 ,  70}, {588 ,  71}, {571 ,  72}, {555 ,  73}, {539 ,  74},
	{523 ,  75}, {508 ,  76}, {494 ,  77}, {479 ,  78}, {466 ,  79},
	{452 ,  80}, {439 ,  81}, {427 ,  82}, {415 ,  83}, {403 ,  84},
	{391 ,  85}, {380 ,  86}, {369 ,  87}, {359 ,  88}, {349 ,  89},
	{339 ,  90}, {329 ,  91}, {320 ,  92}, {311 ,  93}, {302 ,  94},
	{294 ,  95}, {286 ,  96}, {278 ,  97}, {270 ,  98}, {262 ,  99},
	{255 , 100}, {248 , 101}, {241 , 102}, {235 , 103}, {228 , 104},
	{222 , 105}




//  Part of all the sensor range
//	{3310, -5 }, {3276, -4 }, {3242, -3 }, {3206, -2 }, {3170, -1 },
//	{3133,  0 }, {3095,  1 }, {3056,  2 }, {3017,  3 }, {2977,  4 },
//	{2936,  5 }, {2895,  6 }, {2853,  7 }, {2810,  8 }, {2768,  9 },
//	{2724,  10}, {2680,  11}, {2636,  12}, {2592,  13}, {2547,  14},
//	{2502,  15}, {2456,  16}, {2411,  17}, {2365,  18}, {2320,  19},
//	{2274,  20}, {2228,  21}, {2183,  22}, {2137,  23}, {2092,  24},
//	{2047,  25}, {2002,  26}, {1958,  27}, {1913,  28}, {1870,  29},
//    {1826,  30}, {1783,  31}, {1740,  32}, {1698,  33}, {1657,  34},
//    {1616,  35}, {1576,  36}, {1536,  37}, {1497,  38}, {1458,  39},
//    {1420,  40}, {1383,  41}, {1347,  42}, {1311,  43}, {1276,  44},
//    {1242,  45}, {1208,  46}, {1175,  47}, {1143,  48}, {1111,  49},
//    {1081,  50}, {1051,  51}, {1021,  52}, {992,  53},  {964,  54},
//    {937,  55},  {911,  56},  {885,  57},  {860,  58},  {835,  59},
//    {811,  60}
};


////////////////////////////////////////////////////////////////////////////////
// Private functions
////////////////////////////////////////////////////////////////////////////////
bool UTLookUpTableLUT2DLinearSearch(const int16_t rgu16LUT[][2], uint16_t u16LUTLen, uint16_t u16DependentVar, float *pfIndependentVar);
float UTLookUpTableLinearInterp(float fX, float fXa, float fYa, float fXb, float fYb);
float UTLookUpTablePotentiometer2Temp(uint16_t u16ADCValue, float i16MinTemp, float i16MaxTemp, uint16_t u16MinADC, uint16_t u16MaxADC);


////////////////////////////////////////////////////////////////////////////////
// Private variables
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/// \brief 		UTLookUpTableGetTemperratureC
/// \details	Get temperature in Celcius of selected ADC input
/// \public
///
/// \param[in]	eInput: ADC selection to get temperature from
///	\param[out]	pfTemperatureC: returned temperature in celcius
///	\param[out]	pu16TemperatureCount: returned temperature in count (0 - 4096)
/// \return		bool: true if success, false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool UTLookUpTableGetTemperratureC(HAL_ADC_AnalogInputs_e eInput, float * pfTemperatureC, uint16_t * pu16TemperatureCount)
{
  	uint16_t u16ADCValue = 0;
  
	// Hot water temperature.
	if (HAL_ADC_GetValueRaw(eInput, &u16ADCValue) == false) return false;
	if (HAL_ADC_GetValueAveraged(eInput, &u16ADCValue) == false) return false;
	
	//Set return data temperature count
	*pu16TemperatureCount = u16ADCValue;
	
	switch(eInput)
  	{
	  	case HAL_ADC_AI_TEMP_WATER_HOT:
		{
			if (UTLookUpTableLUT2DLinearSearch(rg16BottomLoadLUTHotWater, UTLOOKUPTABLE_LUT_HOT_WATER_SIZE, u16ADCValue, pfTemperatureC) == false) return false;
			//if (UTLookUpTableLUT2DLinearSearch(rg16BottomLoadLUTColdWater, UTLOOKUPTABLE_LUT_COLD_WATER_SIZE, u16ADCValue, pfTemperatureC) == false) return false;
		}break;
		
		case HAL_ADC_AI_TEMP_WATER_COLD_1:
		{
			if (UTLookUpTableLUT2DLinearSearch(rg16BottomLoadLUTColdWater, UTLOOKUPTABLE_LUT_COLD_WATER_SIZE, u16ADCValue, pfTemperatureC) == false) return false;
		}break;
  
		default:
	  	{
	  		return false;
	  	}break;
  	}
	
	return true;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief		UTLookUpTableLUT2DLinearSearch
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
bool UTLookUpTableLUT2DLinearSearch(const int16_t rgu16LUT[][2], uint16_t u16LUTLen, uint16_t u16DependentVar, float *pfIndependentVar)
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
			*pfIndependentVar = UTLookUpTableLinearInterp((float) u16DependentVar, (float) u16XUpperBound, (float) u16YUpperBound, (float) u16XLowerBound, (float) u16YLowerBound);
			return true;
		}
	}

	return false;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief		UTLookUpTableLinearInterp
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
float UTLookUpTableLinearInterp(float fX, float fXa, float fYa, float fXb, float fYb)
{
	// First order Taylor-Young formula
	return  fYa + (fX - fXa) * ( (fYb - fYa)/(fXb - fXa) );
}

////////////////////////////////////////////////////////////////////////////////
/// \brief		UTLookUpTablePotentiometer2Temp
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
float UTLookUpTablePotentiometer2Temp(uint16_t u16ADCValue, float i16MinTemp, float i16MaxTemp, uint16_t u16MinADC, uint16_t u16MaxADC)
{
	float result = ( (u16ADCValue*(i16MaxTemp - i16MinTemp)/(u16MaxADC - u16MinADC)) + i16MinTemp);

	if 		(result > i16MaxTemp) return i16MaxTemp; // gards against to high value
	else if (result < i16MinTemp) return i16MinTemp; // gards against to low value
	else return result;
}



////////////////////////////////////////////////////////////////////////////////
/// UNIT TESTS
////////////////////////////////////////////////////////////////////////////////
#if (UTLOOKUPTABLE_UNIT_TEST == 1)
#warning "Look Up Table unit test enable"

////////////////////////////////////////////////////////////////////////////////
/// UTLookUpTableUnitTest
////////////////////////////////////////////////////////////////////////////////
bool UTLookUpTableUnitTest(void)
{
	printf("\r\n");
	printf("------------------------------------------------------- \r\n");
	printf("---- LOOK UP TABLE UNIT TESTS\r\n");
	printf("------------------------------------------------------- \r\n");
	
	return true;
}
#endif
