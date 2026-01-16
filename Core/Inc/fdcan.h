/* fdcan.c - FDCAN control
   Created: Jan 14, 2025
   Author: Flori (refactored per GNU standards)
*/

#ifndef INC_FDCAN_H_
#define INC_FDCAN_H_

#include <stdint.h>
#include <main.h>

void fdcan_start(FDCAN_HandleTypeDef *hfdcan1, FDCAN_HandleTypeDef *hfdcan2);
void fdcan_prepare_tx_header(FDCAN_TxHeaderTypeDef *TxHeader);
void fdcan_send_message(FDCAN_HandleTypeDef *hfdcan1, FDCAN_TxHeaderTypeDef *TxHeader, uint8_t *TxData, const char* (*get_fdcan_error_string)(FDCAN_HandleTypeDef *));
void fdcan_read_message(FDCAN_HandleTypeDef *hfdcan2, FDCAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData);
#endif // INC_FDCAN_H_