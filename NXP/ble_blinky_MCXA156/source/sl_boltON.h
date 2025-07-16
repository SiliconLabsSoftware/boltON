/*******************************************************************************
 * @file
 * @brief boltON by Silicon Labs
 *******************************************************************************
 * # License
 * <b>Copyright 2025 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#ifndef SL_BOLTON_H
#define SL_BOLTON_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "sl_bt_api.h"
#include "sl_bt_ncp_host.h"
#include "fsl_lpuart.h"

#define BOLTON_UART_PERIPHERAL LPUART2

extern uint8_t bolton_usart_rx_byte;
void sl_bt_on_event(sl_bt_msg_t *evt);

// Receive function exposed to BGAPI
int32_t sl_bt_api_rx(uint32_t dataLength, uint8_t* data);

// Transmit function exposed to BGAPI
void sl_bt_api_tx(uint32_t msg_len, uint8_t* msg_data);

// Peek function exposed to BGAPI
// Returns the number of received bytes in the Rx buffer
int32_t sl_bt_api_peek_rx();

// Initializes the boltON module and the BLE API
bool sl_bolton_init();

// Processes incoming events from the Bluetooth stack
void sl_bolton_process();

// Stores the bytes receive on UART in an internal buffer
void sl_bolton_buffer_received_data(const uint32_t len, const uint8_t *data);

#endif // SL_BOLTON_H
