/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *
  * SMART Home
  *
  * author: Horvath Adam
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

// User driver
#include "user_BME280.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */


// Handle
SemaphoreHandle_t xSemaphore;

// Processed data
int32_t temperature;  		// °C *100
uint32_t pressure;			// hPa  (avg @ sea-level is 1013.25 hPa)
uint32_t humidity;			// rel %
uint32_t temperature_low;
uint32_t temperature_high;
uint32_t pressure_hPa;
uint32_t humidity_low;
uint32_t humidity_high;

// Config data
uint8_t device_id_data;
uint8_t ctrl_meas_data;
uint8_t config_data;

// Raw sensor data
uint8_t sensor_raw_data[8];
uint8_t temperature_raw_data[3];
uint8_t pressure_raw_data[3];
uint8_t humidity_raw_data[2];

// Calibration data
uint8_t calib_data[32];

// Error status
Error_StatusTypeDef BME280_ErrorStatus;
int *myptr = NULL;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */


// Port printf to USART thread safe
int _write(int file, char *ptr, int len)
{
	xSemaphoreTake(xSemaphore,portMAX_DELAY); // Take mutex to check if shared resource is available
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr,len,HAL_MAX_DELAY);
	xSemaphoreGive(xSemaphore);

return len;
}


/*-------------------------------------Task functions*------------------------------------*/

// Process raw sensor data, then send to console
void vTaskWrite(void * pvParameters){

	TickType_t ALastWakeTime; // create tick value for wakeup
	ALastWakeTime = xTaskGetTickCount(); // store when was the wakeup

	for( ;; )
	{
		// Process raw data
		bme280_measure_temperature_int32(sensor_raw_data,&temperature);
		bme280_measure_pressure_int32(sensor_raw_data,&pressure);
		bme280_measure_humidity_int32(sensor_raw_data,&humidity);

		// Convert to printf %d
		temperature_low = temperature % 100;
		temperature_high = temperature / 100;

		// Convert to hPA
		pressure_hPa = pressure / 100;

		// Humidity to printf %d
		humidity_low = humidity % 100;
		humidity_high = humidity / 1000;

		// Error checking
		if(BME280_ErrorStatus != No_Error)
		{
			printf("Error: 0x%x \r\n", BME280_ErrorStatus);
		}

		// Send data to console
		printf("Temperature:  %ld,%ld °C \r\nPressure: %ld hPa\r\nHumidity: %ld,%ld" "%%" "\r\n",temperature_high,temperature_low,pressure_hPa,humidity_high,humidity_low);




		xTaskDelayUntil(&ALastWakeTime, 1000);
	}
}

// Read raw sensor values
void vTaskRead(void * pvParameters){

	TickType_t BLastWakeTime;
	BLastWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		// Read raw sensor data in one burst
		bme280_I2C_read(&hi2c1,BME280_PRESS_MSB_ADDR, sensor_raw_data, 8);

		// Status LED
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

		xTaskDelayUntil(&BLastWakeTime, 100);
	}
}

void vTaskConfig(void * pvParameters){
	// Write status data
	printf(__DATE__" "__TIME__"\r\n");
	printf("Configuring sensor....\r\n");

	// Identify sensor
	bme280_I2C_read(&hi2c1,BME280_ID_ADDR, &device_id_data,1);

	// Config Sensor
	bme280_config(CONFIG_T_SB_05, CONFIG_IIR_FILTER_2, CONFIG_SPI3W_DISABLE,
			CTRL_MEAS_OSRS_T_8, CTRL_MEAS_OSRS_P_4, CTRL_HUM_OSRS_H_1,
			CTRL_MEAS_MODE_NORMAL);

	// Read configs and calib data (debug)
	bme280_I2C_read(&hi2c1,BME280_CONFIG_ADDR, &config_data,1);
	bme280_I2C_read(&hi2c1,BME280_CTRL_MEAS_ADDR, &ctrl_meas_data,1);

	// Save calibrate data
	bme280_I2C_read(&hi2c1, BME280_CALIB_00_ADDR, calib_data,26);
	bme280_I2C_read(&hi2c1, BME280_CALIB_26_ADDR, &calib_data[25], 7);

	// Parse calib data
	parse_compensate(calib_data);

	// Delete task
	vTaskDelete( NULL );
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Create tasks
  TaskHandle_t xHandleA = NULL; // Handler to store task data
  TaskHandle_t xHandleB = NULL;
  TaskHandle_t xHandleC = NULL;
  BaseType_t xRetruned; // to check create return value
  

  xRetruned = xTaskCreate(vTaskWrite,
		  "Task Write",
		  1024,
		  NULL,
		  tskIDLE_PRIORITY+1,  // lowest+1 prio
		  &xHandleA);

  xRetruned = xTaskCreate(vTaskRead,
  		  "Task Read",
		  1024,
		  NULL,
  		  tskIDLE_PRIORITY+2,  // lowest+2 prio
  		  &xHandleB);

  xRetruned = xTaskCreate(vTaskConfig,
  		  "Task C",
  		  1024,
  		  NULL,
  		  tskIDLE_PRIORITY+3,  // lowest+3 prio
  		  &xHandleC);

  // Check if task creation was successful
  if(xRetruned != pdPASS)
    {
  	 return -1;
    }

  // Create Mutex for protecting shared resource
  xSemaphore = xSemaphoreCreateMutex();
  //Check if semaphore creation was successful
  if( xSemaphore == NULL )
	  {
	 return -2;
	  }



  // -------------Start OS--------------------------

  vTaskStartScheduler();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

