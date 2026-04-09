/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_TIMEOUT 500

#define FDCAN_FILTER_ID1 0x000
#define FDCAN_FILTER_ID2 0x7FF

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_FilterTypeDef sFilterConfig;
uint8_t TxData[8] = { 0 };

void FDCAN_Restart() {
	if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK) {
	}
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
	}
}

HAL_StatusTypeDef FDCAN_SendMessage(uint32_t id, uint8_t *TxData) {
	TxHeader.Identifier = id;
	HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,
			&TxHeader, TxData);
	if (status != HAL_OK) {
		FDCAN_Restart();
	}
	HAL_Delay(5);
	return status;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE BEGIN 0 */
uint8_t rx_buff[MAX_NMEA_LEN];

// inicializa a estrutura que salva os dados do gnss
gnss_data_t gnss_data;

// inicializa maquina de estados
system_state_t system_state = STATE_IDLE;

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
  MX_FDCAN1_Init();
  MX_ICACHE_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	// set wakeup and reset pins as HIGH
	HAL_GPIO_WritePin(RESET_GNSS_GPIO_Port, RESET_GNSS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(WAKEUP_GNSS_GPIO_Port, WAKEUP_GNSS_Pin, GPIO_PIN_SET);

	HAL_UART_Receive_IT(&huart1, rx_buff, MAX_NMEA_LEN);
	print_init(&huart3);

//	uint8_t change_baud[MAX_NMEA_LEN] = "$PSTMSETPAR,3102,0xA*12\r\n";
//	uint8_t change_sample_rate[MAX_NMEA_LEN] = "$PSTMSETPAR,1303,0.2*36\r\n";
//	uint8_t save_par[MAX_NMEA_LEN] = "$PSTMSAVEPAR*58\r\n";
//	uint8_t reset[MAX_NMEA_LEN] = "$PSTMSRR*49\r\n";
//	HAL_UART_Transmit(&huart1, change_sample_rate, MAX_NMEA_LEN, 1000);
//	HAL_UART_Transmit(&huart1, save_par, MAX_NMEA_LEN, 1000);
//	HAL_UART_Transmit(&huart1, reset, MAX_NMEA_LEN, 1000);

//	HAL_UART_Transmit(&huart1, change_baud, MAX_NMEA_LEN, 1000);
//	HAL_UART_Transmit(&huart1, save_par, MAX_NMEA_LEN, 1000);
//	HAL_UART_Transmit(&huart1, reset, MAX_NMEA_LEN, 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim6);
	int counter = 0;

	while (1) {
		counter++;

		switch (system_state) {

		case STATE_RECEIVE_UART: {
			// novos dados chegaram no DMA
			// identifica os inicios e fins das mensagens
			const uint8_t MAX_NUMBER_OF_SENTENCES = 20;
			int16_t list_of_start_idxs[MAX_NUMBER_OF_SENTENCES];
			int16_t list_of_end_idxs[MAX_NUMBER_OF_SENTENCES];

			// inicializa com -1
			for (int k = 0; k < MAX_NUMBER_OF_SENTENCES; ++k) {
				list_of_start_idxs[k] = -1;
				list_of_end_idxs[k] = -1;
			}

			// varre o buffer inteiro e vai salvandos os inicios e fins
			uint8_t sentence_idx = 0;
			for (int k = 0; k < MAX_NMEA_LEN; ++k) {
				if (sentence_idx >= MAX_NUMBER_OF_SENTENCES) {
					break;
				}

				if (rx_buff[k] == '$') {
					list_of_start_idxs[sentence_idx] = k;
					sentence_idx++;
				}

				if (rx_buff[k] == '\n' && sentence_idx > 0
						&& list_of_start_idxs[sentence_idx - 1] != -1) {
					// so pega o final se comecou a sentenca antes
					list_of_end_idxs[sentence_idx - 1] = k;
				}
			}

			// verifica se esta igual o numero de start e end
			uint8_t checked_started = 0, checked_end = 0;
			for (int k = 0; k < MAX_NUMBER_OF_SENTENCES; ++k) {
				if (list_of_start_idxs[k] != -1) {
					checked_started++;
				}
				if (list_of_end_idxs[k] != -1) {
					checked_end++;
				}
			}

			if (checked_started != checked_end) {
				// falha na separação das sentenças. break na FSM e descarta esse buffer
				change_state(&system_state, STATE_IDLE);
				break;
			}

			if (checked_started == 0 || checked_end == 0) {
				// sem sentencas NMEA. descarta
				change_state(&system_state, STATE_IDLE);
				break;
			}

			// para cada sentenca, parseia
			for (int k = 0; k < checked_started; ++k) {
				int16_t start_idx = list_of_start_idxs[k];
				int16_t end_idx = list_of_end_idxs[k];

				if (start_idx >= end_idx) {
					// vai para a proxima sentenca, ignora essa
					continue;
				}

				if (start_idx >= MAX_NMEA_LEN || end_idx >= MAX_NMEA_LEN) {
					// vai para a proxima sentenca, ignora essa
					continue;
				}

				// index de inicio e fim determinados. salva no nmea_sentence
				uint8_t sentence_size = end_idx - start_idx + 1; // inclui o \n
				char *nmea_sentence = (char*) malloc(
						sentence_size * sizeof(char));

				int16_t nmea_sentence_idx = 0;
				for (int16_t k = start_idx; k <= end_idx; ++k) {
					nmea_sentence[nmea_sentence_idx] = rx_buff[k];
					nmea_sentence_idx++;
				}

				// nmea_sentence agora contem uma sentença nmea. joga para os parsers
				NMEA_Type nmea_type = NMEA_UNKNOWN;

				if (classify_nmea(&nmea_type, nmea_sentence) != NO_ERROR) {
					continue; // falha na classificacao: vai para a proxima
				}

				switch (nmea_type) {
				case NMEA_GPGGA: {
					if (parse_gpgga(&gnss_data, nmea_sentence) != NO_ERROR) {
						continue; // falha no parse: vai para a proxima
					}
					break;
				}

				case NMEA_GPRMC: {
					if (parse_gprmc(&gnss_data, nmea_sentence) != NO_ERROR) {
						continue; // falha no parse: vai para a proxima
					}
					break;
				}
				case NMEA_UNKNOWN:
					break;
				}

				free(nmea_sentence);
				nmea_sentence = NULL;
			}

			// finalizei o parse do buffer
			change_state(&system_state, STATE_IDLE);
			break;

		}

		case STATE_SEND_UART: {
			// timer disparou: faz o envio
			char uart_output_message[MAX_NMEA_LEN] = { '\0' };
			if (save_to_message(&gnss_data, uart_output_message, MAX_NMEA_LEN)
					!= NO_ERROR) {
				continue; // messaging failed
			}

			print_message(uart_output_message);

			// formata os dados para a CAN
			uint8_t lat_buf[8];
			save_lat_to_buffer(&gnss_data, lat_buf);

			uint8_t lon_buf[8];
			save_lon_to_buffer(&gnss_data, lon_buf);

			uint8_t utc_buf[8]; // gets unix timestamp in miliseconds
			save_utc_to_buffer(&gnss_data, utc_buf);

			uint8_t general_buf[8];
			save_general_to_buffer(&gnss_data, general_buf);

			// envia para a CAN
			FDCAN_SendMessage(LATITUDE_CAN_ID, lat_buf);
			FDCAN_SendMessage(LONGITUDE_CAN_ID, lon_buf);
			FDCAN_SendMessage(UTC_CAN_ID, utc_buf);
			FDCAN_SendMessage(GENERAL_GNSS_CAN_ID, general_buf);

			change_state(&system_state, STATE_IDLE);

			break;
		}
		case STATE_IDLE:
		default: {
			break;
		}
		}
//		for (uint8_t k = 1; k < 60; ++k) {
//			uint8_t can_tx_data[8] = { k, k, k, k, k, k, k, k };
//			FDCAN_SendMessage(262, can_tx_data);
//			HAL_Delay(5);
//		}
//
//		HAL_Delay(5);
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV1;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 8;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = FDCAN_FILTER_ID1;
	sFilterConfig.FilterID2 = FDCAN_FILTER_ID2;

	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);

	TxHeader.Identifier = 0x000;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache (default 2-ways set associative cache)
  */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

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
  htim3.Init.Prescaler = 1787;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1787;
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
  htim6.Init.Prescaler = 1999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7999;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_DEBUG1_GPIO_Port, LED_DEBUG1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, WAKEUP_GNSS_Pin|RESET_GNSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : DEBUG_BTN_Pin */
  GPIO_InitStruct.Pin = DEBUG_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DEBUG_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_DEBUG1_Pin */
  GPIO_InitStruct.Pin = LED_DEBUG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_DEBUG1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WAKEUP_GNSS_Pin RESET_GNSS_Pin */
  GPIO_InitStruct.Pin = WAKEUP_GNSS_Pin|RESET_GNSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	uint16_t sz = Size;
//  HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buff, MAX_NMEA_LEN);

	if (huart->Instance == USART1) {
		// Size = number of received bytes before IDLE
		change_state(&system_state, STATE_RECEIVE_UART);

		// Restart reception
		HAL_UART_Receive_IT(&huart1, rx_buff, MAX_NMEA_LEN);

		// blink LED
		HAL_GPIO_TogglePin(LED_DEBUG1_GPIO_Port, LED_DEBUG1_Pin);

		// reset the watchdog timer
		TIM4->CNT = 0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		change_state(&system_state, STATE_SEND_UART);
	}

	if (htim->Instance == TIM6) {
		// watchdog timer: reset the UART communication
		HAL_UART_Receive_IT(&huart1, rx_buff, MAX_NMEA_LEN);
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
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
