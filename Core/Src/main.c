/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//VERSION DATA
#define SYS_VERSION			0


//SERIAL COMMUNICATION
#ifndef	PC_UART_COM
#define PC_UART_COM
#define PC_UART_PORT		&huart1
#define PC_UART_INSTANCE	USART1
#endif

#define MAX_PC_BUFFER_SIZE	50

#define PC_READ_CMD			0x55
#define PC_WRITE_CMD		0xAA

//SPI COMMUNICATION
#ifndef SPI_COM
#define SPI_COM
#define SPI_PORT			&hspi3
#define CS_PORT				GPIOA
#define CS_PIN				GPIO_PIN_0
#endif

//SYSTEM
/*Status*/
#define DISCONNECT		0
#define CONNECT			1
/*Mode*/
#define STOP_MODE		0
#define START_MODE		1
/*Curve type*/
#define VCE_CURVE		0
#define VBE_CURVE		1

//MEMORY ADDRESS
#define VERSION_OFF			0x00
#define STATUS_OFF			0x01
#define MODE_OFF			0x02
#define CURVE_T_OFF			0x03
#define POT_VAL_OFF			0x04
#define V_DAC_MAX_OFF		0x05

//DAC
#define DAC_TIM_PORT	&htim7
#define DAC_TIM_INS		TIM7

#define DAC_VBE_CH		DAC_CHANNEL_1
#define DAC_VCE_CH		DAC_CHANNEL_2

#define TIM_PERIOD_US	200
#define TIM_GEN_US		40000
#define DAC_SAMPLES		(TIM_GEN_US/TIM_PERIOD_US)+1

//ADC
#define ADC_SAMPLES		19000

#define ADC_VCE_CH		ADC_CHANNEL_14
#define ADC_IC_CH		ADC_CHANNEL_15
#define ADC_VBE_CH		ADC_CHANNEL_16

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
/*Semaphores*/
xSemaphoreHandle serial_sem;

//SYSTEM
#pragma pack(1)
struct{
	uint8_t version;
	uint8_t status;
	uint8_t mode;

	uint8_t curve_type;

	uint8_t pot_value;
	float v_dac_max;				//Range 0 to 1 for 0 to 10V

}system_data;

//PC COMMUNICATION
/*	[0]		:	PC COMMAND (Write or Read).
 * 	[1-2]	:	Size of data for read or write.
 * 	[3-4]	:	Initial address for write or read.
 * 	[5-..]	:	Data (in the case of write).
 */
uint8_t pc_buffer_rx[MAX_PC_BUFFER_SIZE];
uint8_t pc_byte_rx;

//DAC
volatile uint16_t dac_buffer[DAC_SAMPLES];
volatile uint16_t dac_i=0;

//ADC
volatile uint16_t adc_buffer[ADC_SAMPLES];

ADC_ChannelConfTypeDef adc_ch_config = {//For channel configure
		.Rank = ADC_REGULAR_RANK_2,
		.SamplingTime = ADC_SAMPLETIME_24CYCLES_5,
		.SingleDiff = ADC_SINGLE_ENDED,
		.OffsetNumber = ADC_OFFSET_NONE,
		.Offset = 0
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

//SERIAL COMMUNICATION
void uart_rx_handler(uint8_t data);

//SPI COMMUNICATION
void set_pot(uint8_t value);

//TEST TYPE
void vce_test_handler();
void vbe_test_handler();

//DAC
void dac_gen();

//TASK
void user_att_task(void *args);
void deb_att_task(void *args);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//SERIAL COMMUNICATION
void uart_rx_handler(uint8_t data)
{
	static uint16_t i=0;
	static uint16_t size_rx=0;

	pc_buffer_rx[i] = data;
	i++;

	if(i==1)
		if(pc_buffer_rx[0] != PC_READ_CMD || pc_buffer_rx[0] != PC_WRITE_CMD)
		{
			i=0;
			size_rx = 0;
		}
	if(i==5)
	{
		size_rx = pc_buffer_rx[1]<<8 | pc_buffer_rx[2];

		if(pc_buffer_rx[0] == PC_READ_CMD)
		{
			i=0;
			size_rx = 0;
			xSemaphoreGiveFromISR(serial_sem, NULL);
		}
	}
	if(size_rx == i+5)
	{
		i=0;
		size_rx = 0;
		xSemaphoreGiveFromISR(serial_sem, NULL);
	}
}

//SPI COMMUNICATION
void set_pot(uint8_t value)
{
	uint8_t buffer[2]={0x11, value};

	HAL_GPIO_WritePin(CS_PORT, CS_PIN, 0);
	HAL_SPI_Transmit(SPI_PORT, buffer, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, 1);
}

//TEST TYPE
void vce_test_handler()
{

	HAL_DAC_SetValue(&hdac1, DAC_VCE_CH,
			DAC_ALIGN_12B_R, dac_buffer[dac_i]);
	if(dac_i == 0)
	{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer,
				ADC_SAMPLES);
		set_pot(system_data.pot_value);
	}
}

void vbe_test_handler()
{
	HAL_DAC_SetValue(&hdac1, DAC_VBE_CH, DAC_ALIGN_12B_R,
			dac_buffer[dac_i]);
	if(dac_i == 0)
	{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer,
				ADC_SAMPLES);
	}
}

//DAC
void dac_gen()
{
	float t=0;

    for(uint16_t i=0; i<DAC_SAMPLES; i++)
    {
        dac_buffer[i] = (((float)system_data.v_dac_max/(float)TIM_GEN_US)*t)*4095;
        t += TIM_PERIOD_US;
    }
}


//TASK
void user_att_task(void *args)
{
	uint8_t *sys_ptr = (uint8_t *)args;
	uint16_t mem_addr;

	for(;;)
	{
		if(xSemaphoreTake(serial_sem, 1000/portTICK_PERIOD_MS)
				== pdTRUE)
		{

			mem_addr = pc_buffer_rx[3]<<8 | pc_buffer_rx[4];
			if(pc_buffer_rx[0] == PC_WRITE_CMD)
			{
				memcpy(&sys_ptr[mem_addr], &pc_buffer_rx[5],
						pc_buffer_rx[1]<<8 | pc_buffer_rx[2]);

				switch(mem_addr)
				{
				case MODE_OFF:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, system_data.curve_type);//Enable (VBE) or disable(VCE) rele
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, system_data.curve_type);//Enalbe TSCSEL PIN for GAIN
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);//Enable LED2 on board

					/*	Configure second channel by curve type
					 */
					if(system_data.curve_type)
					{
						adc_ch_config.Channel = ADC_VBE_CH;
						system_data.v_dac_max = 0.1;
						dac_gen();
					}
					else
					{
						adc_ch_config.Channel = ADC_VCE_CH;
					}
					if (HAL_ADC_ConfigChannel(&hadc1, &adc_ch_config) != HAL_OK)
						Error_Handler();

					HAL_TIM_Base_Start_IT(DAC_TIM_PORT);
					break;

				case V_DAC_MAX_OFF:
					dac_gen();
					break;

				default:
					break;
				}


			}
			else if(pc_buffer_rx[0] == PC_READ_CMD)
			{
				while(HAL_UART_Transmit_DMA(&huart1, &sys_ptr[mem_addr],
						pc_buffer_rx[1]<<8 | pc_buffer_rx[2]) != HAL_OK)
					vTaskDelay(10);
			}

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
		}
		else
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
			system_data.status = DISCONNECT;
		}
	}
}

/*
void deb_att_task(void *args)
{
	for(;;)
	{
		while(!start_t)
			vTaskDelay(1000/portTICK_PERIOD_MS);
		system_data.curve_type = VCE_CURVE;
		dac_gen();
		set_pot(pot_val);
		adc_ch_config.Channel = ADC_VCE_CH;
		if (HAL_ADC_ConfigChannel(&hadc1, &adc_ch_config) != HAL_OK)
			Error_Handler();
		system_data.mode = START_MODE;
		HAL_TIM_Base_Start_IT(DAC_TIM_PORT);
		while(system_data.mode == START_MODE);
		start_t = 0;

	}
}
*/

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
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_SPI3_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  system_data.version = SYS_VERSION;
  for(uint8_t i=0; i<10; i++)
  {
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1); //Toggle LED1 on board
	  HAL_Delay(100);
  }

  //INIT PPERIPHERALS
  HAL_UART_Receive_DMA(&huart1, &pc_byte_rx, 1);
  HAL_DAC_Start(&hdac1, DAC_VCE_CH);
  HAL_DAC_Start(&hdac1, DAC_VBE_CH);
  HAL_DAC_SetValue(&hdac1, DAC_VCE_CH, DAC_ALIGN_12B_R, 0);
  HAL_DAC_SetValue(&hdac1, DAC_VBE_CH, DAC_ALIGN_12B_R, 0);
  set_pot(255);

  //FREERTOS
  /*Semaphores*/
  serial_sem = xSemaphoreCreateBinary();

  /*Task*/
  xTaskCreate(user_att_task, "USER", 128, (void*)&system_data, 5, NULL);

  /*Init FreeRTOS*/
  vTaskStartScheduler();
  while(1);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV12;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_1LINE;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 79;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 199;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == PC_UART_INSTANCE)
		uart_rx_handler(pc_byte_rx);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == DAC_TIM_INS)
	{
		if(system_data.curve_type)
			vbe_test_handler();
		else
			vce_test_handler();

		dac_i++;

		if(dac_i==DAC_SAMPLES)
		{
			dac_i=0;
			system_data.mode = STOP_MODE;
			HAL_TIM_Base_Stop_IT(DAC_TIM_PORT);
			HAL_DAC_SetValue(&hdac1, DAC_VCE_CH, DAC_ALIGN_12B_R, 0);
			HAL_DAC_SetValue(&hdac1, DAC_VBE_CH, DAC_ALIGN_12B_R, 0);
			set_pot(255);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);//Disable LED2 on board
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
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
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
	  HAL_Delay(500);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
