/*
 * fdcan_config.h
 *
 *  Created on: Feb 7, 2026
 *      Author: rapha
 */

#ifndef INC_FDCAN_CONFIG_H_
#define INC_FDCAN_CONFIG_H_

// INCLUDES ------------------------------------------------------------------
#include "stm32u5xx.h"
#include "main.h"

// DEFINES -------------------------------------------------------------------
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_TxHeaderTypeDef TxHeader; // Header do envio
extern uint32_t TxMailbox; // Caixa de envio

// IDs -----------------------------------------------------------------------
#define LATITUDE_CAN_ID 262
#define LONGITUDE_CAN_ID 263
#define GENERAL_GNSS_CAN_ID 264

#define FDCAN_FILTER_ID1 0x000
#define FDCAN_FILTER_ID2 0x7FF
extern FDCAN_FilterTypeDef sFilterConfig;
extern uint8_t RxData[8];
extern FDCAN_RxHeaderTypeDef RxHeader;

// SENDING
HAL_StatusTypeDef FDCAN_SendMessage(uint16_t id, uint8_t *TxData);

// RESTART CAN
void FDCAN_Restart();

// RECEPÇÃO
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

#endif /* INC_FDCAN_CONFIG_H_ */
