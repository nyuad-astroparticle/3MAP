/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "string.h"
#include <math.h>
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
 ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// Buffers

uint8_t DPPrxBuffer[2] = {0};

uint16_t ADCBuffer;

bool AutoFocusEn = false;
uint16_t dist = 2048;

uint16_t max_pos = 20000; // The maximum possible extent of movement along the linear stages
bool MtrStop = false;


// Motor control registers

// GCONF 		- 0x00
//0b00000000000000000000000000000000;
const char GCONF[5] = {0x80, 0x00, 0x00, 0x00, 0x00};

// IHOLD_IRUN 	- 0x10
//0b00000000000001010001110000011100;
const char IHOLD_IRUN[5] = {0x90, 0x00, 0x05, 0x1C, 0x1C};

// TPOWERDOWN 	- 0x11
//0b00000000000000000000000010000000;
const char TPOWERDOWN[5] = {0x91, 0x00, 0x00, 0x00, 0x80};

// CHOPCONF		- 0x6C
//0b00001000000000001010010101000010;
const char CHOPCONF[5] = {0xEC, 0x08, 0x00, 0xA5, 0x42};

// COOLCONF		- 0x6D
//0b00000000010100000010100000100010;
const char COOLCONF[5] = {0xED, 0x00, 0x50, 0x28, 0x22};

// READSTAT		- 0x6F
//0b00000000000000000000000000000000;
const char READSTAT[5] = {0x6F, 0x00, 0x00, 0x00, 0x00};

// Error codes




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* Task Handles **********************************/

TaskHandle_t MotorCtrlXHandle;
TaskHandle_t MotorCtrlYHandle;
TaskHandle_t HeliumCtrlHandle;
TaskHandle_t UARTComHandle;
TaskHandle_t AutoFocusHandle;
//TaskHandle_t DPPComHandle;

/* Queue Handles *********************************/

QueueHandle_t UARTtxQueue;
QueueHandle_t DPPrxQueue;
QueueHandle_t MtrCtrlXrxQueue;
QueueHandle_t MtrCtrlYrxQueue;
QueueHandle_t HelCtrlrxQueue;
//QueueHandle_t AutoFocusrxQueue;

/* Semaphore Handles *****************************/

SemaphoreHandle_t MotorPulseSem;
SemaphoreHandle_t AutoFocusSem;
SemaphoreHandle_t UARTComSem;
SemaphoreHandle_t AutoFocusPulseSem;


/* Task Functions *********************************/
void AutoFocus (void *argument);
void MotorCtrlX (void *argument);
void MotorCtrlY (void *argument);
void HeliumCtrl (void *argument);
void UARTCom (void *argument);
//void DPPCom (void *argument);


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

  /* Initialise Pins ****************************/
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);		// EN X and Y
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);		// CS X and Y
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); 	// DIR X
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); 	// STEP X
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); 	// DIR Y
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); 	// STEP Y


  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); 	// EN Z
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); 		// CS Z
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); 	// DIR Z
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); 	// STEP Z

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_DMA_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  while(HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK);

  /* Create Queues *********************************/

  UARTtxQueue = xQueueCreate(10, 2);
  DPPrxQueue = xQueueCreate(50, 2); // Send and receive 2 bytes to/from the MCA
  MtrCtrlXrxQueue = xQueueCreate(10, 8);
  MtrCtrlYrxQueue = xQueueCreate(10, 8);
  HelCtrlrxQueue = xQueueCreate(1, 2); // Just an atomic variable, not really a "queue" per se. Could probably just use a semaphore and a global variable as I did in the ISRs
  //AutoFocusrxQueue = xQueueCreate(1, 4);

  /* Create Semaphores *****************************/

  MotorPulseSem = xSemaphoreCreateBinary();
  AutoFocusSem = xSemaphoreCreateBinary();
  UARTComSem = xSemaphoreCreateBinary();
  AutoFocusPulseSem = xSemaphoreCreateBinary();

  /* Create Tasks **********************************/

  xTaskCreate(AutoFocus, 	"Auto_Focus", 			256, NULL, 	4, &AutoFocusHandle);
  xTaskCreate(MotorCtrlX, 	"Motor_Controller_X", 	128, NULL, 	3, &MotorCtrlXHandle);
  xTaskCreate(MotorCtrlY, 	"Motor_Controller_Y", 	128, NULL, 	3, &MotorCtrlYHandle);
  xTaskCreate(HeliumCtrl, 	"Helium_Controller", 	128, NULL, 	1, &HeliumCtrlHandle);
  xTaskCreate(UARTCom, 		"UART_Communication", 	128, NULL, 	2, &UARTComHandle);

  /* Initialise Peripherals with HAL **********************************/

  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_SPI_Receive_DMA(&hspi2, DPPrxBuffer, 2);

 /* Platform initialisation ***********************************/

  // Being that we send the same information to all the motor drivers, we set the CS for all three low prior to sending a message
  vTaskDelay(pdMS_TO_TICKS(1));
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&GCONF, 5, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  vTaskDelay(pdMS_TO_TICKS(1));
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&IHOLD_IRUN, 5, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  vTaskDelay(pdMS_TO_TICKS(1));
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&TPOWERDOWN, 5, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  vTaskDelay(pdMS_TO_TICKS(1));
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&CHOPCONF, 5, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  vTaskDelay(pdMS_TO_TICKS(1));
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&COOLCONF, 5, 100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);



  vTaskStartScheduler();


  /* USER CODE END 2 */

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 70;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 280000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 280-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1500-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 280-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1500-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|GPIO_PIN_12|GPIO_PIN_13|LD3_Pin
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin PB12 PB13 LD3_Pin
                           PB15 PB3 PB4 PB5
                           PB8 PB9 */
  GPIO_InitStruct.Pin = LD1_Pin|GPIO_PIN_12|GPIO_PIN_13|LD3_Pin
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_N_Pin USB_FS_P_Pin */
  GPIO_InitStruct.Pin = USB_FS_N_Pin|USB_FS_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PG9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


// Tasks

void UARTCom(void *argument){

	char UARTtxBuffer[2] = {0};
	char UARTrxBuffer[9] = {0};

	char MtrCtrlXrx_temp[8] = {0};
	char MtrCtrlYrx_temp[8] = {0};

	char HelCtrlrx_temp[2] = {0};

	char AutoFocusrx_temp[4] = {0};


	while(1){


		// Send data from the tx UART queue over uart
		// Receive from UARTtxQueue, buffer into UARTtxBuffer, and send over UART
		if (xQueueReceive(UARTtxQueue, &UARTtxBuffer, 100) == pdTRUE){
			HAL_UART_Transmit(&huart3, (uint8_t *)UARTtxBuffer, sizeof(UARTtxBuffer), 100);
		}



		HAL_UART_Receive_IT(&huart3, (uint8_t *)UARTrxBuffer, 9);
		if(xSemaphoreTake(UARTComSem, 100) == pdTRUE){


			// Process data received over UART
			switch(UARTrxBuffer[0]){

//			case '0':
//				MtrStop = true;
//				break;

			case '1':
//
				if (UARTrxBuffer[1] == '0'){
					MtrStop = true;
				}

				memcpy(MtrCtrlXrx_temp, UARTrxBuffer + 1, 8);
				xQueueSend(MtrCtrlXrxQueue, &MtrCtrlXrx_temp, 100);
				//memset(&MtrCtrlXrx_temp[0], '0', 8);
				break;



			case '2':

				if (UARTrxBuffer[1] == '0'){
					MtrStop = true;
				}

				memcpy(MtrCtrlYrx_temp, UARTrxBuffer + 1, 8);
				xQueueSend(MtrCtrlYrxQueue, &MtrCtrlYrx_temp, 100);
				//memset(&MtrCtrlYrx_temp[0], '0', 8);
				break;



			case '3':
				memcpy(AutoFocusrx_temp, UARTrxBuffer + 1, 4);
				dist = atoi(AutoFocusrx_temp);

				if (atoi(AutoFocusrx_temp) == 0){
					AutoFocusEn = false;
				} else {
					AutoFocusEn = true;
				}

				//memset(&AutoFocusrx_temp[0], '0', 4);
				break;



			case '4':
				memcpy(HelCtrlrx_temp, UARTrxBuffer + 1, 2);
				xQueueSend(HelCtrlrxQueue, &HelCtrlrx_temp, 100);
				//memset(&HelCtrlrx_temp[0], '0', 2);
				break;
			}




//			if (UARTrxBuffer[0] != '0'){
//				HAL_UART_Transmit(&huart3, (uint8_t *)UARTrxBuffer, 9, 100);
//				memset(&UARTrxBuffer[0], '0', 9);
//			}

		} //else {

//			vTaskDelay(pdMS_TO_TICKS(100));
//		}

	}

}

void AutoFocus(void *argument){



	const uint8_t num_steps = 10; // Number of steps taken between each ADC sample - roughly corresponds to 5 ms. Determines how frequently ADC is sampled

	uint8_t i = 0;
	TIM3 -> ARR = 0;

	while(1){

		if (AutoFocusEn){

			// ADC sampling
			HAL_ADC_Start_IT(&hadc1);
			if (xSemaphoreTake(AutoFocusSem, 1) == pdTRUE){


				// Setting direction of rotation
				if (ADCBuffer < dist - 300){
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // This needs to be checked
				} else if (ADCBuffer > dist + 300){
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
				}


				// Setting the motor speed through the timer frequency
				TIM3 -> ARR = -abs(ADCBuffer - dist) + (dist + 500);

//				HAL_UART_Transmit(&huart3, (uint8_t *)itoa(ADCBuffer, test_buff, 10), sizeof(test_buff), 100);
//				HAL_UART_Transmit(&huart3, (uint8_t *)"\n", 1, 100);



				// The actual movement
				// Nothing is done if the distance is approximately correct
				// Additionally, due to what appears to be a glitch in the internal gain settings, when the value of ADCBuffer is around 2048
				// it sometimes skips to 1535 (2^11 - 2^9) so we must also account for this to ensure the autofocus doesn't skip around unpredictably
				if ( (ADCBuffer > (dist - 300) && ADCBuffer < (dist + 300)) || (ADCBuffer == 1535) || MtrStop == true ){
					vTaskDelay(pdMS_TO_TICKS(5));
				} else {
					for (i = 0; i<= num_steps; i++){
						if (xSemaphoreTake(AutoFocusPulseSem, 1) == pdTRUE){
							HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
						}
					}
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

				}
				HAL_ADC_Stop_IT(&hadc1);

			}

		} else { vTaskDelay(pdMS_TO_TICKS(500)); } // end if(AutoFocusEn)
	} // end while
}

void MotorCtrlX (void *argument){

	int i;
	char MtrCtrlXrxQueueBuffer[8] = {0};
	char steps[7]; // Local variable into which the number of steps is buffered from MtrCtrlXrxQueueBuffer
	char speed[4]; // Local variable into which the speed is buffered from MtrCtrlXrxQueueBuffer

	static uint16_t posX = 0; // The position along the side
	static int dir; // The direction, encoded by 1 or -1



	while(1){

		if (xQueueReceive(MtrCtrlXrxQueue, &MtrCtrlXrxQueueBuffer, 100) == pdTRUE){


			switch(MtrCtrlXrxQueueBuffer[0]) {

				/*
				 * Case 0 is handled in UARTCom - that is, the setting of the stop flag
				 */

				/*
				 * Set the direction, keep moving while the switch has not been triggered
				 * When the switch is triggered, change direction and move 200 steps off the switch.
				 * Register position
				 */

				case '1': // INIT
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); 			// The direction pin set to backwards

					while(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9) == 1 && !MtrStop){	// Keep moving while the switch has not been pushed and a stop flag has not been issued
						xSemaphoreTake(MotorPulseSem, 100);
						HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
					}

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);			// Once hit, set the pin low and wait 100 ms
					vTaskDelay(pdMS_TO_TICKS(500));

					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);				// Change to forwards motion and move 200 steps
					for (i=0; i < 200; i++){
						xSemaphoreTake(MotorPulseSem, 100);
						HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
					}
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);

					posX = 200;														// Set the X position to 200
					break;


				/*
				 *  There is a bit of work to do here in the case of the motor reaching the end of it's range
				 * 	Sending the position for example, so the host computer can sort it
				 * 	Further, if the motor reaches the maximum position, it is impossible to release it
				 *
				 * 	Buffer the number of steps to be moved into a local variable to remove command bytes
				 * 	Take 'steps' number of steps, only do so if the switch is untouched, the max position is not reached,
				 * 	or a stop command has not been received
				 */

				case '2': // MOVE
					memcpy(steps, MtrCtrlXrxQueueBuffer + 1, 7);			// Copy the useful information out of the queue buffer

					for (i=0; i < atoi(steps); i++){						// Move 'steps' number of steps

						if (!MtrStop){										// Every time, check that a stop command has not been received

							xSemaphoreTake(MotorPulseSem, 100);				// Wait for the motor pulse
							HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);			// Pulse the step pin
							posX += dir;									// Increment/decrement the x position

							if (i%10 == 0){ // Add this modulo operation to lessen the load on the processor
								if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9) == 0 || (posX >= max_pos)){MtrStop = true;}	// Check that the max position has not been reached and the switch has not been pressed
							}

						} else {											// If the stop flag is triggered, set it to false and break out of the loop
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
							MtrStop = false;
							break;
							//xQueueSend(MtrCtrlXrxQueue, , 100); SEND THE POSITION OR SOMETHING
						}
					}
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
					memset(&steps[0], 0, sizeof(steps));
					break;

				/*
				 * This one is pretty simple. The direction is selected by applying the appropriate state to the direction pin
				 * and the sign to the 'dir' variable
				 */

				case '3': // SELECT DIRECTION
					if (MtrCtrlXrxQueueBuffer[1] == '0'){
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
						dir = -1;
					} else if (MtrCtrlXrxQueueBuffer[1] == '1') {
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
						dir = 1;
					}
					break;

				/*
				 * Again, nice and simple. Copy the useful information from the queue buffer and
				 * assign it to the ARR register which governs the speed of the timer
				 */

				case '4': // CHANGE SPEED
					memcpy(speed, MtrCtrlXrxQueueBuffer + 1, 4);
					TIM5 -> ARR = atoi(speed);
					memset(&speed, 0, sizeof(speed));
					break;


			}

			memset(&MtrCtrlXrxQueueBuffer[0], 0, sizeof(MtrCtrlXrxQueueBuffer)); // Clear the queue buffer
		}
		vTaskDelay(pdMS_TO_TICKS(500)); // Wait half a second between commands

	}
}

void MotorCtrlY (void *argument){
	int i;
	char MtrCtrlYrxQueueBuffer[8] = {0};
	char steps[7];
	char speed[4];

	static uint16_t posY = 0;
	static int dir;



	while(1){

		if (xQueueReceive(MtrCtrlYrxQueue, &MtrCtrlYrxQueueBuffer, 100) == pdTRUE){


			switch(MtrCtrlYrxQueueBuffer[0]) {

				/*
				 * Case 0 is handled in UARTCom - that is, the setting of the stop flag
				 */

				/*
				 * Set the direction, keep moving while the switch has not been triggered
				 * When the switch is triggered, change direction and move 200 steps off the switch.
				 * Register position
				 */

				case '1': // INIT
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); 			// The direction pin set to backwards

					while(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9) == 1 && !MtrStop){	// Keep moving while the switch has not been pushed and a stop flag has not been issued
						xSemaphoreTake(MotorPulseSem, 100);
						HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
					}

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);			// Once hit, set the pin low and wait 100 ms
					vTaskDelay(pdMS_TO_TICKS(500));

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);				// Change to forwards motion and move 200 steps
					for (i=0; i < 200; i++){
						xSemaphoreTake(MotorPulseSem, 100);
						HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
					}
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

					posY = 200;														// Set the X position to 200
					break;


				/*
				 *  There is a bit of work to do here in the case of the motor reaching the end of it's range
				 * 	Sending the position for example, so the host computer can sort it
				 *
				 * 	Buffer the number of steps to be moved into a local variable to remove command bytes
				 * 	Take 'steps' number of steps, only do so if the switch is untouched, the max position is not reached,
				 * 	or a stop command has not been received
				 */

				case '2': // MOVE
					memcpy(steps, MtrCtrlYrxQueueBuffer + 1, 7);			// Copy the useful information out of the queue buffer

					for (i=0; i < atoi(steps); i++){						// Move 'steps' number of steps

						if (!MtrStop){										// Every time, check that a stop command has not been received

							xSemaphoreTake(MotorPulseSem, 100);				// Wait for the motor pulse
							HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);			// Pulse the step pin
							posY += dir;									// Increment/decrement the x position

							if (i%10 == 0){ // Add this modulo operation to lessen the load on the processor
								if (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9) == 0 || (posY >= max_pos)){MtrStop = true;}	// Check that the max position has not been reached and the switch has not been pressed
							}

						} else {													// If the stop flag is triggered, set it to false and break out of the loop
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
							MtrStop = false;
							break;
							//xQueueSend(MtrCtrlXrxQueue, , 100); SEND THE POSITION OR SOMETHING
						}
					}
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
					memset(&steps[0], 0, sizeof(steps));
					break;

				/*
				 * This one is pretty simple. The direction is selected by applying the appropriate state to the direction pin
				 * and the sign to the 'dir' variable
				 */

				case '3': // SELECT DIRECTION
					if (MtrCtrlYrxQueueBuffer[1] == '0'){
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
						dir = -1;
					} else if (MtrCtrlYrxQueueBuffer[1] == '1') {
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
						dir = 1;
					}
					break;

				/*
				 * Again, nice and simple. Copy the useful information from the queue buffer and
				 * assign it to the ARR register which governs the speed of the timer
				 */

				case '4': // CHANGE SPEED
					memcpy(speed, MtrCtrlYrxQueueBuffer + 1, 4);
					TIM5 -> ARR = atoi(speed);
					memset(&speed, 0, sizeof(speed));
					break;


			}

			memset(&MtrCtrlYrxQueueBuffer[0], 0, sizeof(MtrCtrlYrxQueueBuffer)); // Clear the queue buffer
		}
		vTaskDelay(pdMS_TO_TICKS(500)); // Wait half a second between commands

	}

}

void HeliumCtrl(void *argument){

	char CCRrxBuffer[2] = {0};
	TIM2 -> CCR1 = 0;

	while(1){

		if (xQueueReceive(HelCtrlrxQueue, &CCRrxBuffer, 100) == pdTRUE){
			TIM2 -> CCR1 = round( (TIM2 -> ARR) * (atoi(CCRrxBuffer)/100.0) );
		}


		vTaskDelay(pdMS_TO_TICKS(1000));

	}
}



// Non-task functions

//void MtrCtrl(GPIO_TypeDef* pin_group, uint16_t pin, char *QueueBuffer){
//
//}


// Interrupt Service Routines

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (huart->Instance == USART3){
		xSemaphoreGiveFromISR( UARTComSem, &xHigherPriorityTaskWoken);
	}

	if(xHigherPriorityTaskWoken){
	    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	HAL_SPI_Receive_DMA(&hspi2, DPPrxBuffer, sizeof(DPPrxBuffer));
    xQueueSendToBackFromISR(UARTtxQueue, &DPPrxBuffer, &xHigherPriorityTaskWoken);

    if(xHigherPriorityTaskWoken){
    	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	ADCBuffer = HAL_ADC_GetValue(&hadc1);
	xSemaphoreGiveFromISR( AutoFocusSem, &xHigherPriorityTaskWoken);

	if(xHigherPriorityTaskWoken){
	    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */


  if(htim == &htim3){ // tim3 is our autofocus motor pulse timer, triggers an interrupt on rollover
		xSemaphoreGiveFromISR( AutoFocusPulseSem, &xHigherPriorityTaskWoken);
	}


  if(htim == &htim5){ // tim5 is our motor pulse timer, triggers an interrupt on rollover
    	xSemaphoreGiveFromISR( MotorPulseSem, &xHigherPriorityTaskWoken);
    }

  if(xHigherPriorityTaskWoken){
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
   	}

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
