/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "bmp280.h"
#include "GPS.h"
#include "AQI_thread.h"

#define NODE 11;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

 
osPoolDef(message, 1, data);							//define memory pool
osPoolId  message_id;
data *message_pointer;

osThreadId GPS_id,PMS_id,BME_id,OLED_id,ESP_id,NBIOT_id;			//Define the task ID variables
osSemaphoreId Turnstile;									
osSemaphoreDef(Turnstile);
osSemaphoreId Turnstile2;									
osSemaphoreDef(Turnstile2);
osSemaphoreId Mutex;									
osSemaphoreDef(Mutex);
unsigned int count=0;
int test =0;

osSemaphoreId Mutex_read;									
osSemaphoreDef(Mutex_read);

osSemaphoreId Mutex_send;									
osSemaphoreDef(Mutex_send);

void GPS_read_task(void const *argument) 			
{
	while(1)					
	{
		osSemaphoreWait(Mutex_read, osWaitForever);
		mux_enable();
		mux_sel(1);			
		GPS_Process();
		mux_enable();
		osSemaphoreRelease(Mutex_read);
//-----------------------------------Entry Turnstile-------------------------------							

		osSemaphoreWait(Mutex,0xffff);			 				//Allow one task at a time to access the first turnstile
		count = count+1;					 									// Increment count
		if( count == 5) 					 									//When last section of code reaches this point run his code
		{
			osSemaphoreWait (Turnstile2,0xffff);	 		//Lock the second turnstile
			osSemaphoreRelease(Turnstile);				 		//Unlock the first turnstile
		}			
		osSemaphoreRelease(Mutex);									//Allow other tasks to access the turnstile
		osSemaphoreWait(Turnstile,0xFFFF);					//Turnstile Gate			
		osSemaphoreRelease(Turnstile);

//-----------------------------------Entry Turnstile-----------------------------------

		
		  // write to message
//----------------------------------Exit Turnstile------------------------------------

		osSemaphoreWait(Mutex,0xffff);			 				//Allow one task at a time to access the turnstile
		count = count - 1;
		if(count ==0)						 										//When last section of code reaches this point run his code
		{
			osSemaphoreWait(Turnstile,0xffff);		 		//Lock the second turnstile
			osSemaphoreRelease(Turnstile2);			 			//Unlock the first turnstile
		}
		osSemaphoreRelease(Mutex);					 				//Allow other tasks to access the turnstile
		osSemaphoreWait(Turnstile2,0xffff);		 			//Turnstile Gate
		osSemaphoreRelease(Turnstile2);

//---------------------------------Exit Turnstile-------------------------------------		

	}
}	;


void PMS7003_read_task(void const *argument) 			
{
	while(1)					
	{
		osSemaphoreWait(Mutex_read, osWaitForever);
		PMS_Init();
		if(readPMS()){
		message_pointer->pm01= ((uint16_t)pms_rx_data[4] << 8) | (uint16_t)pms_rx_data[5];
		message_pointer->pm25= ((uint16_t)pms_rx_data[6] << 8) | (uint16_t)pms_rx_data[7];	
		message_pointer->pm10= ((uint16_t)pms_rx_data[8] << 8) | (uint16_t)pms_rx_data[9];
		}
		else{
		message_pointer->pm01= 99999;
		message_pointer->pm25= 99999;	
		message_pointer->pm10= 99999;	
		}
		osSemaphoreRelease(Mutex_read);
		
//-----------------------------------Entry Turnstile-------------------------------							

		osSemaphoreWait(Mutex,0xffff);			 				//Allow one task at a time to access the first turnstile
		count = count+1;					 									// Increment count
		if( count == 5) 					 									//When last section of code reaches this point run his code
		{
			osSemaphoreWait (Turnstile2,0xffff);	 		//Lock the second turnstile
			osSemaphoreRelease(Turnstile);				 		//Unlock the first turnstile
		}			
		osSemaphoreRelease(Mutex);									//Allow other tasks to access the turnstile
		osSemaphoreWait(Turnstile,0xFFFF);					//Turnstile Gate			
		osSemaphoreRelease(Turnstile);

//-----------------------------------Entry Turnstile-----------------------------------
    

		
		// write to message
//----------------------------------Exit Turnstile------------------------------------

		osSemaphoreWait(Mutex,0xffff);			 				//Allow one task at a time to access the turnstile
		count = count - 1;
		if(count ==0)						 										//When last section of code reaches this point run his code
		{
			osSemaphoreWait(Turnstile,0xffff);		 		//Lock the second turnstile
			osSemaphoreRelease(Turnstile2);			 			//Unlock the first turnstile
		}
		osSemaphoreRelease(Mutex);					 				//Allow other tasks to access the turnstile
		osSemaphoreWait(Turnstile2,0xffff);		 			//Turnstile Gate
		osSemaphoreRelease(Turnstile2);

//---------------------------------Exit Turnstile-------------------------------------		

	}
}	;

void BME_read_task(void const *argument) 			
{
	while(1)					
	{
		osSemaphoreWait(Mutex_read, osWaitForever);
	  if(BME280_Init()){
	  
	  	if(!readBME280()){
	    message_pointer->humid = 99999.0;
	  	message_pointer->temp = 99999.0;
	  	message_pointer->pressure = 99999.0;
	  	HAL_I2C_DeInit(&hi2c1);
	  	HAL_I2C_Init(&hi2c1);
	  	}
	  }
	  else{
	  	message_pointer->humid = 99999.0;
	  	message_pointer->temp = 99999.0;
	  	message_pointer->pressure = 99999.0;
	  	HAL_I2C_DeInit(&hi2c1);
	  	HAL_I2C_Init(&hi2c1);
	  	BME280_Init();
	  }
		osSemaphoreRelease(Mutex_read);
		
//-----------------------------------Entry Turnstile-------------------------------							

		osSemaphoreWait(Mutex,0xffff);			 				//Allow one task at a time to access the first turnstile
		count = count+1;					 									// Increment count
		if( count == 5) 					 									//When last section of code reaches this point run his code
		{
			osSemaphoreWait (Turnstile2,0xffff);	 		//Lock the second turnstile
			osSemaphoreRelease(Turnstile);				 		//Unlock the first turnstile
		}			
		osSemaphoreRelease(Mutex);									//Allow other tasks to access the turnstile
		osSemaphoreWait(Turnstile,0xFFFF);					//Turnstile Gate			
		osSemaphoreRelease(Turnstile);

//-----------------------------------Entry Turnstile-----------------------------------

			



		// write to message
//----------------------------------Exit Turnstile------------------------------------

		osSemaphoreWait(Mutex,0xffff);			 				//Allow one task at a time to access the turnstile
		count = count - 1;
		if(count ==0)						 										//When last section of code reaches this point run his code
		{
			osSemaphoreWait(Turnstile,0xffff);		 		//Lock the second turnstile
			osSemaphoreRelease(Turnstile2);			 			//Unlock the first turnstile
		}
		osSemaphoreRelease(Mutex);					 				//Allow other tasks to access the turnstile
		osSemaphoreWait(Turnstile2,0xffff);		 			//Turnstile Gate
		osSemaphoreRelease(Turnstile2);

//---------------------------------Exit Turnstile-------------------------------------		

	}
}	;
void OLED_disp_task(void const *argument) 			
{
	while(1)					
	{
		
//-----------------------------------Entry Turnstile-------------------------------							
	
		osSemaphoreWait(Mutex,0xffff);			 				//Allow one task at a time to access the first turnstile
		count = count+1;					 									// Increment count
		if( count == 5) 					 									//When last section of code reaches this point run his code
		{
			osSemaphoreWait (Turnstile2,0xffff);	 		//Lock the second turnstile
			osSemaphoreRelease(Turnstile);				 		//Unlock the first turnstile
		}			
		osSemaphoreRelease(Mutex);									//Allow other tasks to access the turnstile
		osSemaphoreWait(Turnstile,0xFFFF);					//Turnstile Gate			
		osSemaphoreRelease(Turnstile);

//-----------------------------------Entry Turnstile-----------------------------------
			
		  osSemaphoreWait(Mutex_send, osWaitForever);
		  ssd1306_Init();		 
 		  oled_disp();
			osSemaphoreRelease(Mutex_send);

//----------------------------------Exit Turnstile------------------------------------

		osSemaphoreWait(Mutex,0xffff);			 				//Allow one task at a time to access the turnstile
		count = count - 1;
		if(count ==0)						 										//When last section of code reaches this point run his code
		{
			osSemaphoreWait(Turnstile,0xffff);		 		//Lock the second turnstile
			osSemaphoreRelease(Turnstile2);			 			//Unlock the first turnstile
		}
		osSemaphoreRelease(Mutex);					 				//Allow other tasks to access the turnstile
		osSemaphoreWait(Turnstile2,0xffff);		 			//Turnstile Gate
		osSemaphoreRelease(Turnstile2);

//---------------------------------Exit Turnstile-------------------------------------		

	}
}	;


void NBIOT_send_task(void const *argument) 			
{
	while(1)					
	{
//-----------------------------------Entry Turnstile-------------------------------							

		osSemaphoreWait(Mutex,0xffff);			 				//Allow one task at a time to access the first turnstile
		count = count+1;					 									// Increment count
		if( count == 5) 					 									//When last section of code reaches this point run his code
		{
			osSemaphoreWait (Turnstile2,0xffff);	 		//Lock the second turnstile
			osSemaphoreRelease(Turnstile);				 		//Unlock the first turnstile
		}			
		osSemaphoreRelease(Mutex);									//Allow other tasks to access the turnstile
		osSemaphoreWait(Turnstile,0xFFFF);					//Turnstile Gate			
		osSemaphoreRelease(Turnstile);

//-----------------------------------Entry Turnstile-----------------------------------
	  
		osSemaphoreWait(Mutex_send, osWaitForever);
		sendData_NBIOT();
		osSemaphoreRelease(Mutex_send);
		
//----------------------------------Exit Turnstile------------------------------------

		osSemaphoreWait(Mutex,0xffff);			 				//Allow one task at a time to access the turnstile
		count = count - 1;
		if(count ==0)						 										//When last section of code reaches this point run his code
		{
			osSemaphoreWait(Turnstile,0xffff);		 		//Lock the second turnstile
			osSemaphoreRelease(Turnstile2);			 			//Unlock the first turnstile
		}
		osSemaphoreRelease(Mutex);					 				//Allow other tasks to access the turnstile
		osSemaphoreWait(Turnstile2,0xffff);		 			//Turnstile Gate
		osSemaphoreRelease(Turnstile2);

//---------------------------------Exit Turnstile-------------------------------------		

	}
}	;

osThreadDef(BME_read_task, osPriorityNormal, 1, 0);
osThreadDef(GPS_read_task, osPriorityNormal, 1, 0);
osThreadDef(PMS7003_read_task, osPriorityNormal, 1, 0);
osThreadDef(NBIOT_send_task, osPriorityNormal, 1, 0);
osThreadDef(OLED_disp_task, osPriorityNormal, 1, 0);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;



/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

//TIM:tick=1ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

//OLED
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
	osKernelInitialize ();
	Turnstile =	osSemaphoreCreate(osSemaphore(Turnstile), 0);
	Turnstile2 = osSemaphoreCreate(osSemaphore(Turnstile2), 1);
	Mutex =	osSemaphoreCreate(osSemaphore(Mutex), 1);	
	Mutex_read =	osSemaphoreCreate(osSemaphore(Mutex_read), 1);	
	Mutex_send =	osSemaphoreCreate(osSemaphore(Mutex_send), 1);	
	PMS_id = osThreadCreate(osThread(PMS7003_read_task),(void *) 1U);  //Create the tasks 
	BME_id = osThreadCreate(osThread(BME_read_task),(void *) 2U); 
	GPS_id = osThreadCreate(osThread(GPS_read_task),(void *) 3U);
	OLED_id = osThreadCreate(osThread(OLED_disp_task),(void *) 4U);
	NBIOT_id = osThreadCreate(osThread(NBIOT_send_task),(void *) 5U);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	osKernelStart (); 
	message_id= osPoolCreate(osPool(message));
	message_pointer= (data *)osPoolAlloc(message_id);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	
  
	
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 3;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 3999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PMS_SET_Pin|PMS_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, S0_Pin|S1_Pin|S2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Enable_Pin */
  GPIO_InitStruct.Pin = Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_WK_Pin */
  GPIO_InitStruct.Pin = OLED_WK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(OLED_WK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NBIOT_WK_Pin */
  GPIO_InitStruct.Pin = NBIOT_WK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NBIOT_WK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMS_SET_Pin PMS_RESET_Pin */
  GPIO_InitStruct.Pin = PMS_SET_Pin|PMS_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : S0_Pin S1_Pin S2_Pin */
  GPIO_InitStruct.Pin = S0_Pin|S1_Pin|S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

// TIM 6: tick=1ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim6.Instance) {
		if(sys_cnt==SYS_CNT_MAX) {
		  sys_cnt=0;
		} else {
			sys_cnt++;
		}
		HAL_IncTick();
	}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 if(GPIO_Pin == GPIO_PIN_0) {
		 oled_cnt_test++;
		 led_on=1;
     led_off=0;		 
	 }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
