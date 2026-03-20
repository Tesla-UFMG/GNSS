#ifndef PRINT_H
#define PRINT_H

#include "stm32u5xx_hal.h"
#include <stdio.h>
#include <stdarg.h>

#define UART_TIMEOUT 100
#define MSG_SIZE 512

void print_init(UART_HandleTypeDef *huart);
void print_message(char* message, ...);

#endif /* PRINT_H */
