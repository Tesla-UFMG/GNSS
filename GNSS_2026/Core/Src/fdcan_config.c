#include "fdcan_config.h"

// VARIAVEIS PARA O ENVIO
FDCAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;

// VARIAVEIS PARA TESTAR A RECEPÇÃO
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

// Functions --------------------------------------------------
HAL_StatusTypeDef FDCAN_SendMessage(uint16_t id, uint8_t *TxData) {
    TxHeader.Identifier = id;
    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
    return status;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
            return;
        }
    }
}
