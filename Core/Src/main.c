/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include "stm32l475e_iot01_qspi.h"
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
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//Constants & Variables associated with tone generation-------
const int MAX_TONE_AMPLITUDE = 255;
const float OUTPUT_SAMPLE_RATE = 9400.0;
const float TAU = 6.2831853072;
const int TONE_FREQUENCIES[] = {524, 659, 784, 1047};
const int BLOCK_SIZE = 65536;
int REC_START;
const int NUM_TONES = 4;
const int TONE_LEN = 8192;
uint32_t currentTone[8192];

//Variables for sequencing tone------------------------------
const int MAX_TONE_SEQUENCE_LENGTH = 100;
int toneSequence[100];
int toneSequenceSize = 0;
int toneIndex = 0;

//Variables for processing tones------------------------------
arm_rfft_instance_q31 fftInstance;
uint32_t fftOutputBuffer[8192];
const int WINDOW_SIZE = 50;
const int MIN_FREQ = 150;
const int MAX_FREQ = 3000;

//Constants & Variables for High Scores
int HIGH_SCORE_START;
const int MAX_SCORES = 1000;
const int FORMAT_START = 0xA0FA0FEE;

//Flags for state --------------------------------------------
enum GAME_STATE {
	READY_TO_PLAY_TONE = 0,
	PLAYING_TONE,
	READY_TO_RECORD,
	RECORDING,
	START_GAME
};

enum GAME_STATE gameState = START_GAME;

//Other constants --------------------------------------------
const int UART_TIMEOUT_MS = 100;
const int UART_RETRIES = 3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_DFSDM1_Init(void);
/* USER CODE BEGIN PFP */

/// Adds an additional tone to the sequence to be played and expected from user
void addToneToSequence();

/// Starts audio output of tone sequence
void playSequence();

/// Get tone from flash of specified frequency index to currentTone buffer
///
/// @param frequency_index Index of frequency to retrieve from QSPI flash memory
void loadToneFromFlash(int frequency_index);

/// Starts microphone recording DMA
void startRecording();

/// Determine frequency of specific note
///
/// @param noteIndex Index of the note that frequency should be analyzed of
/// @return Frequency of user tone
int analyzeNote(int noteIndex);

/// Checks that the user tone sequence is correct
///
/// @return 0 if answer false, any other uint8_t if true
uint8_t checkAnswer();

/// Prints score and restarts game loop
void lose();

/// Transmits all scores through UART
 void printScoreToUart();

 /// Transmits message to UART to be displayed on screen of connected device
 ///
 /// @param buffer Pointer to null-terminated string to be sent through UART
 void transmitToUart(char * buffer);

 // Receives message from UART
 //
 // @param buffer Pointer to memory to write to
 // @param size Number of bytes to read
 // @return Bytes read if successful, -1 otherwise
 int receiveFromUart(char * buffer, uint16_t size);

 /// Loads scores from Flash Memory
 ///
 /// @param scores Pointer to where to copy scores
 /// @return Integer representing number of scores stored in Flash
 int loadScoresFromMemory(int * scores);

 /// Stores score in Flash Memory
 ///
 /// @param initials Character buffer with 2 characters
 /// @param score Score to store in memory
 void storeScoreInMemory(char * initials, int score);

 /// Formats the Flash Memory for store
 /// Note: Should only use to reset system
 void formatScoreMemory();

 /// Clears all scores in Flash Memory
 void clearScoresInMemory();

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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_DAC1_Init();
  MX_QUADSPI_Init();
  MX_DFSDM1_Init();
  /* USER CODE BEGIN 2 */
    HAL_UART_Init(&huart1);
	HAL_TIM_Base_Start(&htim2);
	BSP_QSPI_Init();
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);

	//erase flash except high scores
	int numBlocks = (int)(NUM_TONES*sizeof(currentTone)/(float)BLOCK_SIZE + 1);
	for (int i = 0; i < numBlocks; i++) {
		if (BSP_QSPI_Erase_Block(BLOCK_SIZE*i) != QSPI_OK) {
			Error_Handler();
		}
	}

	REC_START = BLOCK_SIZE * (numBlocks + 1);
	HIGH_SCORE_START = REC_START + BLOCK_SIZE * (numBlocks + 1);

	//format high score memory if necessary
	int check_format = 0;
	if (BSP_QSPI_Read(&check_format, HIGH_SCORE_START, sizeof(check_format)) != QSPI_OK) {
		Error_Handler();
	}

	// if constant start byte not there, we know we must format
	if(check_format != FORMAT_START)
	{
		formatScoreMemory();
	}

	//program flash
	for (int i = NUM_TONES-1; i >= 0; i--) {
		//calculate a single tone
		float w = TAU * TONE_FREQUENCIES[i];
		for (int j = 0; j < TONE_LEN; j++) {
			float theta = w * j/OUTPUT_SAMPLE_RATE;
			float s = (1 + arm_sin_f32(theta)) / 2.0;

			currentTone[j] = (uint32_t) (0.9 * MAX_TONE_AMPLITUDE * s);
		}
		//write tone to flash
		if (BSP_QSPI_Write(currentTone, i * sizeof(currentTone), sizeof(currentTone)) != QSPI_OK) {
			Error_Handler();
		}
	}

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_DFSDM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 250;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 34;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8500;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RED_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLUE_BUTTON_Pin */
  GPIO_InitStruct.Pin = BLUE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUE_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin != BLUE_BUTTON_Pin) {
		return;
	}
	if (gameState == READY_TO_PLAY_TONE || gameState == START_GAME) {
		if (gameState == START_GAME) {
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
			transmitToUart("   ______________ Starting New Game! ______________\n");
			transmitToUart("\n");
		}
		addToneToSequence();
		playSequence();
	} else if (gameState == READY_TO_RECORD) {
		startRecording();
	}
}

void addToneToSequence(void) {
	//If we exceed buffer size, pattern resets anew -- unlikely to matter
	if (toneSequenceSize == MAX_TONE_SEQUENCE_LENGTH) {
		toneSequenceSize = 0;
	}
	toneSequence[toneSequenceSize] = rand() % NUM_TONES;
	toneSequenceSize++;
}

void loadToneFromFlash(int frequency) {
	if (BSP_QSPI_Read(currentTone, sizeof(currentTone) * frequency, sizeof(currentTone)) != QSPI_OK) {
		Error_Handler();
	}
}

void printSequence() {

	transmitToUart("   Expected Sequence - ");
	for (int i = 0; i < toneSequenceSize; i++) {
		char buf[20];
		memset(buf, 0, sizeof(buf));
		if (i != toneSequenceSize - 1) {
			sprintf(buf, "%d, ", TONE_FREQUENCIES[toneSequence[i]]);
		} else {
			sprintf(buf, "%d\n", TONE_FREQUENCIES[toneSequence[i]]);
		}

		transmitToUart(buf);
	}

}

/*
 * non-blocking function that plays the sequence to the user
 */
void playSequence() {
	gameState = PLAYING_TONE;
	int numBlocks = (int)(toneSequenceSize*sizeof(currentTone)/(float)BLOCK_SIZE  + 1);

	// Erase all user recordings in flash
	for (int i = 0; i < numBlocks; i++) {
		if (BSP_QSPI_Erase_Block(REC_START + BLOCK_SIZE*i) != QSPI_OK) {
			Error_Handler();
		}
	}

	// Get tons from flash and start DMA
	toneIndex = 0;
	loadToneFromFlash(toneSequence[toneIndex]);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, currentTone, TONE_LEN, DAC_ALIGN_8B_R);

	//Turn on LED to start
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
}

//DAC callback
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac) {
	//toggles LED to help time
	HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

	toneIndex++;
	if (toneIndex >= toneSequenceSize) {
		//Turn off LED when done playing
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
		gameState = READY_TO_RECORD;
	} else {
		loadToneFromFlash(toneSequence[toneIndex]);
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, currentTone, TONE_LEN, DAC_ALIGN_8B_R);
	}
}

void startRecording() {
	toneIndex = 0;
	gameState = RECORDING;
	HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, currentTone, TONE_LEN);

	//Turn on LED to start
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
}

void lose() {
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
	transmitToUart("   _________________ GAME OVER! ___________________ \n");
	transmitToUart("\n");
	transmitToUart("\n");
	char buf[50];
	memset(buf, 0, sizeof(buf));
	sprintf(buf, "   __Your Score is %d Points\n", toneSequenceSize);
	transmitToUart(buf);
	transmitToUart("\n");
	transmitToUart("   ___Enter Initials:\n");

	//Ensure all characters transmitted
	char initials[2];
	receiveFromUart(initials, sizeof(initials));

	storeScoreInMemory(initials, toneSequenceSize);
	printScoreToUart();

	gameState = START_GAME;
	toneSequenceSize = 0;
	toneIndex = 0;
}

// callback for microphone
void HAL_DFSDM_FilterRegConvCpltCallback (DFSDM_Filter_HandleTypeDef * hdfsdm_filter) {
	// check to make sure correct callback
	if(hdfsdm_filter == &hdfsdm1_filter0) {
		HAL_DFSDM_FilterRegularStop_DMA(hdfsdm_filter);

		// scale 24 bits by removing 8 least sig bits
		for(size_t i = 0; i < TONE_LEN; i++) {
			currentTone[i] = (currentTone[i] >> 8);
		}

		//write microphone to flash
		if(BSP_QSPI_Write(currentTone, sizeof(currentTone) * toneIndex + REC_START, sizeof(currentTone)) != QSPI_OK) {
			Error_Handler();
		}

		//check if end of user recording
		if(++toneIndex < toneSequenceSize) {
			HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
			HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, currentTone, TONE_LEN);
		} else {
			if (checkAnswer()) {
				gameState = READY_TO_PLAY_TONE;
			} else {
				lose();
			}
			HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
		}
	}
}

int analyzeNote(int noteIndex) {
	//Load note from flash
	if (BSP_QSPI_Read(currentTone, REC_START + noteIndex*sizeof(currentTone), sizeof(currentTone)) != QSPI_OK) {
		Error_Handler();
	}

	//Initialize FFT instance
	if (arm_rfft_init_q31(&fftInstance, TONE_LEN/2, 0, 1) != ARM_MATH_SUCCESS) {
		Error_Handler();
	}

	//Call FFT (assumes currentTone buffer already formatted in q31 format)
	arm_rfft_q31(&fftInstance, &currentTone[TONE_LEN/4], fftOutputBuffer);

	//Find magnitude values
	arm_cmplx_mag_q31(fftOutputBuffer, currentTone, TONE_LEN/2);

	const float DELTA_F = OUTPUT_SAMPLE_RATE / (TONE_LEN / 2);
	int maxCenterFreq = MIN_FREQ;
	int maxWindowSum = 0;
	unsigned long currentSum = 0;
	for (int i = 0; i < TONE_LEN / 2; i++) {
		if (i - WINDOW_SIZE/2 >= 0) {
			currentSum -= currentTone[i - WINDOW_SIZE/2];
		}
		if (i + WINDOW_SIZE/2 < TONE_LEN / 2) {
			currentSum += currentTone[i + WINDOW_SIZE / 2];
		}
		if (currentSum > maxWindowSum && i*DELTA_F > MIN_FREQ && i*DELTA_F < MAX_FREQ) {
			maxWindowSum = currentSum;
			maxCenterFreq = (int)(i * DELTA_F) ;
		}

	}
	return maxCenterFreq;
}

uint8_t checkAnswer() {
       int maxValues[NUM_TONES];
       int minValues[NUM_TONES];
       uint8_t includedTones = 0x0;
       for (int i = 0; i < NUM_TONES; i++) {
		   maxValues[i] = 0;
		   minValues[i] = INT_MAX;
       }

       printSequence();

       transmitToUart("   User Inputed Frequencies - ");

       for (int i = 0; i < toneSequenceSize; i++) {
		   int tone = toneSequence[i];
		   int freq = analyzeNote(i);

		   char buf[20];
		   memset(buf, 0, sizeof(buf));
		   if (i != toneSequenceSize - 1) {
			   sprintf(buf, "%d, ", freq);
		   } else {
			   sprintf(buf, "%d\n", freq);
		   }
		   transmitToUart(buf);

		   if (freq > maxValues[tone]) {
			   maxValues[tone] = freq;
		   }
		   if (freq < minValues[tone]) {
			   minValues[tone] = freq;
		   }

		   // create bit field for whether tone used
		   includedTones |= 1 << tone;
       }
       transmitToUart("\n");
       //Checks tone ranges are reasonable
       for (int i = 0; i < NUM_TONES; i++) {
		   //Don't need to compare tones not involved in sequence
		   if (includedTones & (1 << i))
		   {
			   for (int j = i + 1; j < NUM_TONES; j++) {
				   if ((includedTones & (1 << j)) && maxValues[i] > minValues[j]) {
					   return 0; // failure :(
				   }
			   }
		   }
       }
       return 1; // success :)
}

void printScoreToUart() {
 	const int BUFFER_SIZE = 100;
 	char buffer[BUFFER_SIZE];
 	memset(buffer, 0, BUFFER_SIZE);

 	int scores[MAX_SCORES];
 	int num_scores = loadScoresFromMemory(scores);

 	int chars_written = sprintf(buffer, "   ________________ LEADERBOARD __________________ \n");
 	for(int i = 0; i < num_scores; i++) {
 		// Should flush buffer if mostly full
 		if(chars_written >= BUFFER_SIZE - BUFFER_SIZE / 5) {
 			chars_written = 0;
 			transmitToUart(buffer);
 		}
 		int raw_score = scores[i] & 0xFFFF; // Last 16 bits are actual score
 		char initials[3];
 		initials[0] = (scores[i] >> 3*8) & 0xFF;
 		initials[1] = (scores[i] >> 2*8) & 0xFF;
 		initials[2] = '\0';
 		chars_written += sprintf(buffer + chars_written, "%d. %s - %d\n", i + 1, initials, raw_score);
 	}

 	if(chars_written > 0) {
 		transmitToUart(buffer);
 	}
 	transmitToUart("\n");
 	transmitToUart("\n");
 	transmitToUart("\n");
 }

int loadScoresFromMemory(int * scores_to_load) {
	int num_scores = 0;
	// Read number of scores from memory
	if (BSP_QSPI_Read(&num_scores, HIGH_SCORE_START + sizeof(FORMAT_START), sizeof(num_scores)) != QSPI_OK) {
		Error_Handler();
	}

	if(num_scores > 0) {
		if (BSP_QSPI_Read(scores_to_load, HIGH_SCORE_START + sizeof(FORMAT_START) + sizeof(num_scores), num_scores*sizeof(int)) != QSPI_OK) {
			Error_Handler();
		}
	}
	return num_scores;
 }

 void storeScoreInMemory(char * initials, int score) {
	int scores[MAX_SCORES];
	int num_scores = loadScoresFromMemory(scores);

	// Must clear before rewrite
	clearScoresInMemory();

	// Allocate space for number of scores, previous scores, and new score to store
	int new_scores[MAX_SCORES + 3];
	new_scores[0] = FORMAT_START;
	new_scores[1] = num_scores + 1;

	score |= initials[0] << 3*8; // Add initials to score
	score |= initials[1] << 2*8;

	int new_score_added = 0;

	// Add element sorted
	for(int i = 0; i < num_scores; i++) {
		if(new_score_added == 0 && (score & 0xFFFF) >= (scores[i] & 0xFFFF))
		{
			new_scores[i+2] = score;
			new_score_added = 1;
			i--;
		}
		else if(new_score_added == 1)
		{
			new_scores[i+3] = scores[i];
		}
		else
		{
			new_scores[i+2] = scores[i];
		}
	}

	if(new_score_added == 0) {
		new_scores[num_scores + 2] = score;
	}



	// Write Scores to Memory
	if (BSP_QSPI_Write(new_scores, HIGH_SCORE_START, (2 + num_scores + 1)*sizeof(int)) != QSPI_OK) {
		Error_Handler();
	}
 }

 /// Clears all scores in Flash Memory
 void clearScoresInMemory() {
	if (BSP_QSPI_Erase_Block(HIGH_SCORE_START) != QSPI_OK) {
		Error_Handler();
	}
 }

 void formatScoreMemory()  {
	clearScoresInMemory();

	// Must set initial number of scores to 0 (since clearing writes all 1s)
	int num_score = 0;
	if (BSP_QSPI_Write(&FORMAT_START, HIGH_SCORE_START, sizeof(int)) != QSPI_OK ||
		BSP_QSPI_Write(&num_score, HIGH_SCORE_START + sizeof(int), sizeof(int)) != QSPI_OK) {
			Error_Handler();
	}
 }


void transmitToUart(char * buffer) {
	for(int i = 0; i < UART_RETRIES; i++) {
		if(HAL_OK == HAL_UART_Transmit(&huart1, buffer, strlen(buffer) + 1, UART_TIMEOUT_MS)) {
			break;
		}
	}
}

int receiveFromUart(char * buffer, uint16_t size) {
	for(int i = 0; i < UART_RETRIES; i++) {
		while(1) {

			int hal_status = HAL_UART_Receive(&huart1, buffer, size, UART_TIMEOUT_MS);
			if (hal_status == HAL_OK) {
				return size;
			} else if (hal_status == HAL_TIMEOUT) {
				// Do nothing, wait for input
			} else {
				break; // Reset for another retry
			}
		}
	}
	return -1;
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

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
