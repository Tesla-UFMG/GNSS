#include "print.h"

static UART_HandleTypeDef *communicationUart;

/**
 * @brief initializes UART module to communicate with FTDI
 * @param huart pointer to handler of UART
 */
void print_init(UART_HandleTypeDef *huart) {
	communicationUart = huart;
}

/**
 * @brief sends message to FTDI module via UART.
 * @param message string in sprintf format
 * @param ... variable arguments from sprintf
 */
void print_message(char* message, ...) {
	va_list args;
	va_start(args, message);

	char buffer[MSG_SIZE] = {0};
	vsnprintf(buffer, MSG_SIZE, message, args);
	HAL_UART_Transmit(communicationUart, (uint8_t*) buffer, MSG_SIZE, UART_TIMEOUT);
}
