/* fdcan.c - FDCAN control
   Created: Jan 14, 2025
   Author: Flori (refactored per GNU standards)
*/


#include <stdio.h>
#include "fdcan.h"

void fdcan_start(FDCAN_HandleTypeDef *hfdcan1, FDCAN_HandleTypeDef *hfdcan2)
{
    if (HAL_FDCAN_Start(hfdcan1) != HAL_OK) {
        printf("Error: FDCAN1 Start failed\r\n");
    }
    if (HAL_FDCAN_Start(hfdcan2) != HAL_OK) {
        printf("Error: FDCAN2 Start failed\r\n");
    }
}

void fdcan_prepare_tx_header(FDCAN_TxHeaderTypeDef *TxHeader)
{
    TxHeader->Identifier = 0x123;
    TxHeader->IdType = FDCAN_STANDARD_ID;
    TxHeader->TxFrameType = FDCAN_DATA_FRAME;
    TxHeader->DataLength = FDCAN_DLC_BYTES_8;
    TxHeader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader->BitRateSwitch = FDCAN_BRS_OFF;
    // FDCAN_CLASSIC_CAN: Standard CAN 2.0 frame (no CAN FD features)
    // FDCAN_FD_CAN: CAN FD frame (allows flexible data rate and larger payloads)
    TxHeader->FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader->MessageMarker = 0;
}

void fdcan_send_message(FDCAN_HandleTypeDef *hfdcan1, FDCAN_TxHeaderTypeDef *TxHeader, uint8_t *TxData, const char* (*get_fdcan_error_string)(FDCAN_HandleTypeDef *))
{
    printf("FDCAN1 sending message to FDCAN2...\r\n");
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, TxHeader, TxData) != HAL_OK) {
        printf("Error: Send failed (%s)\r\n", get_fdcan_error_string(hfdcan1));
    }
}

void fdcan_read_message(FDCAN_HandleTypeDef *hfdcan2, FDCAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData)
{
    HAL_Delay(100); // Wait for internal loopback processing
    if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan2, FDCAN_RX_FIFO0) > 0) {
        if (HAL_FDCAN_GetRxMessage(hfdcan2, FDCAN_RX_FIFO0, RxHeader, RxData) == HAL_OK) {
            printf("FDCAN2 received message:\r\n");
            printf("ID: 0x%03lX\r\n", (unsigned long)RxHeader->Identifier);
            printf("Data: %.8s\r\n", (char*)RxData);
        }
    } else {
        printf("Error: No message received on FDCAN2\r\n");
    }
}