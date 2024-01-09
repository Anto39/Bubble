
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "time.h"

/* USER CODE BEGIN Includes */
#include "Framework.h"
#include "CommManager.h"
#include "LedDriver.h"
#include "IceBoxx.h"
#include "NoTouch.h"
#include "UVLamp.h"
#include "Heater.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define APPLICATION_HW_TEST			0		///< Activate specific hardware startup tests.

#if (APPLICATION_HW_TEST == 1)
#warning "*** Application HW Test is active ***"
#include "MCUMap.h"
uint16_t	u16ADCValueRaw1, u16ADCValueRaw2, u16ADCValueRaw3, u16ADCValueRaw4, u16ADCValueRaw5, u16ADCValueRaw6, u16ADCValueRaw7, u16ADCValueRaw8, u16ADCValueRaw9;
uint16_t	u16ADCValueAvg1, u16ADCValueAvg2, u16ADCValueAvg3, u16ADCValueAvg4, u16ADCValueAvg5, u16ADCValueAvg6, u16ADCValueAvg7, u16ADCValueAvg8, u16ADCValueAvg9;
bool		bState1, bState2, bState3, bState4, bState5, bState6, bState7, bState8, bState9, bState10, bState11, bState12, bState13;
#endif


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void ApplicationFatalError(void);

#if (APPLICATION_HW_TEST == 1)
void ApplicationHWTest(void);
#endif


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
////////////////////////////////////////////////////////////////////////////////
/// \brief		ApplicationFatalError
/// \details	Function managing the fatal error mode of the application.
///	\warning	This function is blocking and should never return.
///           
/// \private
/// \param		None.
/// \return		void.
////////////////////////////////////////////////////////////////////////////////
void ApplicationFatalError(void)
{
	// Infinite loop to prevent malfunctions.
	while (1)
	{
		FrameworkFatalErrorTask();
	}
}
////////////////////////////////////////////////////////////////////////////////
/// \brief		ApplicationHWTest
/// \details	Function reading input data to validate the hardware.
///           
/// \private
/// \param		None.
/// \return		void.
////////////////////////////////////////////////////////////////////////////////
#if (APPLICATION_HW_TEST == 1)
void ApplicationHWTest(void)
{
	// Read raw analogic values.
	HAL_ADC_GetValueRaw(HAL_ADC_AI_TEMP_WATER_HOT_ADJUST, &u16ADCValueRaw1);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_TEMP_WATER_COLD_ADJUST, &u16ADCValueRaw2);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_MODEL_ID_CH, &u16ADCValueRaw3);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_HW_VER_MAJOR, &u16ADCValueRaw4);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_HW_VER_MINOR, &u16ADCValueRaw5);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_TEMP_WATER_COLD_1, &u16ADCValueRaw6);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_TEMP_WATER_COLD_2, &u16ADCValueRaw7);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_INTERNAL_C02_CORE_TEMP, &u16ADCValueRaw8);
	HAL_ADC_GetValueRaw(HAL_ADC_AI_TEMP_WATER_HOT, &u16ADCValueRaw9);
	
	// Read averaged analogic values.
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_HOT_ADJUST, &u16ADCValueAvg1);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_COLD_ADJUST, &u16ADCValueAvg2);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_MODEL_ID_CH, &u16ADCValueAvg3);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_HW_VER_MAJOR, &u16ADCValueAvg4);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_HW_VER_MINOR, &u16ADCValueAvg5);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_COLD_1, &u16ADCValueAvg6);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_COLD_2, &u16ADCValueAvg7);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_INTERNAL_C02_CORE_TEMP, &u16ADCValueAvg8);
	HAL_ADC_GetValueAveraged(HAL_ADC_AI_TEMP_WATER_HOT, &u16ADCValueAvg9);
	
	// Read digital input.
	GPIOGetDDI(GPIO_DDI_PRESENCE_ICE, &bState1);
	GPIOGetDDI(GPIO_DDI_PRESENCE_FLOOD_BOTTOM, &bState2);
	GPIOGetDDI(GPIO_DDI_PRESENCE_FLOOD_TOP, &bState3);
	GPIOGetDDI(GPIO_DDI_WATER_LEVEL_LOW, &bState4);
	GPIOGetDDI(GPIO_DDI_WATER_LEVEL_HIGH, &bState5);
	GPIOGetDDI(GPIO_DDI_PRESSURE_CO2, &bState6);
	GPIOGetDDI(GPIO_DDI_DOOR_OPEN, &bState7);
	GPIOGetDDI(GPIO_DDI_BUTTON_1, &bState8);
	GPIOGetDDI(GPIO_DDI_BUTTON_2, &bState9);
	GPIOGetDDI(GPIO_DDI_BUTTON_3, &bState10);
	GPIOGetDDI(GPIO_DDI_BUTTON_4, &bState11);
	GPIOGetDDI(GPIO_DDI_EMPTY_BOTTLE, &bState12);
	GPIOGetDDI(GPIO_DDI_CAP_BUT_INTERRUPT, &bState13);
	
	// Set digital outputs.
	/*
	HAL_GPIO_WritePin(MCUMAP_DO_LED_NIGHT_PERIPH, MCUMAP_DO_LED_NIGHT_PIN, GPIO_PIN_SET);			// White Led
	HAL_GPIO_WritePin(MCUMAP_DO_LED_NIGHT_PERIPH, MCUMAP_DO_LED_NIGHT_PIN, GPIO_PIN_RESET);			// White Led
	HAL_GPIO_WritePin(MCUMAP_DO_LED_BLUE_PERIPH, MCUMAP_DO_LED_BLUE_PIN, GPIO_PIN_SET);				// Not Initialized
	HAL_GPIO_WritePin(MCUMAP_DO_LED_BLUE_PERIPH, MCUMAP_DO_LED_BLUE_PIN, GPIO_PIN_RESET);			// Not Initialized
	HAL_GPIO_WritePin(MCUMAP_DO_LED_GREEN_BI_PERIPH, MCUMAP_DO_LED_GREEN_BI_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_LED_GREEN_BI_PERIPH, MCUMAP_DO_LED_GREEN_BI_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_LED_RED_PERIPH, MCUMAP_DO_LED_RED_PIN, GPIO_PIN_SET);				// Blue Led
	HAL_GPIO_WritePin(MCUMAP_DO_LED_RED_PERIPH, MCUMAP_DO_LED_RED_PIN, GPIO_PIN_RESET);				// Blue Led
	HAL_GPIO_WritePin(MCUMAP_DO_LED_YELLOW_PERIPH, MCUMAP_DO_LED_YELLOW_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_LED_YELLOW_PERIPH, MCUMAP_DO_LED_YELLOW_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_PUMP_RECIR_PERIPH, MCUMAP_DO_PUMP_RECIR_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_PUMP_RECIR_PERIPH, MCUMAP_DO_PUMP_RECIR_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_HEATBAND_PERIPH, MCUMAP_DO_HEATBAND_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_HEATBAND_PERIPH, MCUMAP_DO_HEATBAND_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_PUMP_LOW_PERIPH, MCUMAP_DO_PUMP_LOW_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_PUMP_LOW_PERIPH, MCUMAP_DO_PUMP_LOW_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_PUMP_HIGH_PERIPH, MCUMAP_DO_PUMP_HIGH_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_PUMP_HIGH_PERIPH, MCUMAP_DO_PUMP_HIGH_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_VENTILATOR_PERIPH, MCUMAP_DO_VENTILATOR_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_VENTILATOR_PERIPH, MCUMAP_DO_VENTILATOR_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_3WAY_VALVE_PERIPH, MCUMAP_DO_3WAY_VALVE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_3WAY_VALVE_PERIPH, MCUMAP_DO_3WAY_VALVE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_WIFI_BLE_ENABLE_PERIPH, MCUMAP_DO_WIFI_BLE_ENABLE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_WIFI_BLE_ENABLE_PERIPH, MCUMAP_DO_WIFI_BLE_ENABLE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_WIFI_BLE_RESET_PERIPH, MCUMAP_DO_WIFI_BLE_RESET_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_WIFI_BLE_RESET_PERIPH, MCUMAP_DO_WIFI_BLE_RESET_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_COLD_PERIPH, MCUMAP_DO_SLND_WATER_COLD_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_HOT_PERIPH, MCUMAP_DO_SLND_WATER_HOT_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_HOT_PERIPH, MCUMAP_DO_SLND_WATER_HOT_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_TEMPERATE_PERIPH, MCUMAP_DO_SLND_WATER_TEMPERATE_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_WATER_TEMPERATE_PERIPH, MCUMAP_DO_SLND_WATER_TEMPERATE_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_AQUEDUCT_PERIPH, MCUMAP_DO_SLND_AQUEDUCT_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_AQUEDUCT_PERIPH, MCUMAP_DO_SLND_AQUEDUCT_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_CO2_ON_PERIPH, MCUMAP_DO_SLND_CO2_ON_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_SLND_CO2_ON_PERIPH, MCUMAP_DO_SLND_CO2_ON_PIN, GPIO_PIN_RESET);

	// For the compressor, if it is re-activated too fast, the internal fuse will blow and reset itself after some time.
	HAL_GPIO_WritePin(MCUMAP_DO_COMPRESSOR_PERIPH, MCUMAP_DO_COMPRESSOR_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MCUMAP_DO_COMPRESSOR_PERIPH, MCUMAP_DO_COMPRESSOR_PIN, GPIO_PIN_RESET);
*/
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //FrameworkModelID_e modelID = FRAMEWORK_MODEL_ID_UNKNOWN;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();
  MX_SPI1_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  
	// Modules initialization.
	if (FrameworkInit() == false) ApplicationFatalError();
	if (CommManagerInit() == false) ApplicationFatalError();
	if (NoTouchInit() == false) ApplicationFatalError();
	if (IceBoxxInit() == false) ApplicationFatalError();
	if (UVLampInit() == false) ApplicationFatalError();
	if (HeaterInit() == false) ApplicationFatalError();
	
	// Allow time for electronics to startup correctly.
  	//HAL_Delay(100);

	// Unit tests.
#if (FRAMEWORK_UNIT_TEST == 1)
	if (FrameworkUnitTest() == false) ApplicationFatalError();
#endif
#if (BOTTOMLOAD_UNIT_TEST == 1)
	if (BottomLoadUnitTest() == false) ApplicationFatalError();
#endif
#if (LEDDRIVER_UNIT_TEST == 1)
	if (LedDriverUnitTest() == false) ApplicationFatalError();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	// Start the probe polarity toggle.
	MX_PROBE_POL_PWM_START();
	//MX_BUZZER_PWM_START();
	// Forever loop.
	while (1)
	{
		//if (NoTouchTask() 	== false) ApplicationFatalError();
		if (FrameworkTask() == false) ApplicationFatalError();
		if (IceBoxxTask() 	== false) ApplicationFatalError();
		//if (UVLampTask() 	== false) ApplicationFatalError();
		//if (HeaterTask() 	== false) ApplicationFatalError();

#if (APPLICATION_HW_TEST == 1)
		// Hardware testing.
		ApplicationHWTest();
		continue;
#endif
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
