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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
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
int main(void) {
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
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
//  HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buff, MAX_NMEA_LEN);
//  HAL_UART_Receive_DMA(&huart1, rx_buff, MAX_NMEA_LEN);
//  HAL_UART_Receive_IT(&huart1, rx_buff, MAX_NMEA_LEN);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buff, MAX_NMEA_LEN);
	print_init(&huart2);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);

	while (1) {
		// continue;
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

			// finalizei o parse do bufer
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

//			// envia para a CAN
//			uint8_t can_tx_data[8] = { 10, 20, 30, 40, 50, 60, 70, 80 };
//			FDCAN_SendMessage(262, can_tx_data);

			change_state(&system_state, STATE_IDLE);

			break;
		}
		case STATE_IDLE:
		default: {
			break;
		}
		}
//	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//	  print_message("Hello World! \r\n");

//	  HAL_Delay(100);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 64;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1787;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1787;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 1999;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 7999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	/* DMA2_Stream7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  HAL_UART_Receive_IT(&huart1, rx_buff, MAX_NMEA_LEN); //You need to toggle a breakpoint on this line!
//}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  HAL_UART_Receive_DMA(&huart1, rx_buff, MAX_NMEA_LEN); //You need to toggle a breakpoint on this line!
//}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
//	uint16_t sz = Size;
//  HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buff, MAX_NMEA_LEN);

	if (huart->Instance == USART1) {
		// Size = number of received bytes before IDLE
		change_state(&system_state, STATE_RECEIVE_UART);

		// Restart reception
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buff, MAX_NMEA_LEN);

		// blink LED
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		// reset the watchdog timer
		TIM4->CNT = 0;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		change_state(&system_state, STATE_SEND_UART);
	}

	if (htim->Instance == TIM4) {
		// watchdog timer: reset the UART communication
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, rx_buff, MAX_NMEA_LEN);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
