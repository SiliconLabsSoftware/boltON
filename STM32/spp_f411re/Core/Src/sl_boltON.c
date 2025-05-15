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

#include "sl_boltON.h"
#include "sl_bt_api.h"
#include "sl_bt_ncp_host.h"
#include "main.h"

extern UART_HandleTypeDef BOLTON_UART_HANDLE;
uint8_t bolton_usart_rx_byte;

// Checks if we have valid data in the incoming buffer
// If yes this function will copy it into the BGAPI buffer
static bool sl_check_for_valid_message_in_buf();

// Gets bytes from the Rx buffer without consuming them
static int32_t peek_rx_bytes(uint32_t dataLength, uint32_t offset, uint8_t* data);

// Buffer for receiving the data
static uint8_t rx_buf[256];
static uint32_t rx_buf_len = 0u;
// Buffer for presenting data for the BGAPI
static uint8_t rx_buf_out[256];
static uint32_t rx_buf_out_len = 0u;

int32_t sl_bt_api_rx(uint32_t dataLength, uint8_t* data)
{
  if (!data) {
    return -1;
  }

  // Check if we have valid data in the incoming buffer
  // If yes this function will copy it into the BGAPI buffer
  sl_check_for_valid_message_in_buf();

  if (dataLength <= rx_buf_out_len) {
    memcpy((void *)data, (void *)rx_buf_out, (size_t)dataLength);
    rx_buf_out_len -= dataLength;
    memmove((void *)rx_buf_out, (void *)&rx_buf_out[dataLength], rx_buf_out_len);
  } else {
    dataLength = -1;
  }
  return dataLength;
}

void sl_bt_api_tx(uint32_t msg_len, uint8_t* msg_data)
{
  HAL_UART_Transmit(&BOLTON_UART_HANDLE, msg_data, msg_len, HAL_MAX_DELAY);
}

int32_t sl_bt_api_peek_rx()
{
  sl_check_for_valid_message_in_buf();
  return rx_buf_out_len;
}

bool sl_bolton_init()
{
  HAL_UART_Receive_IT(&BOLTON_UART_HANDLE, &bolton_usart_rx_byte, 1);
  sl_status_t sc = sl_bt_api_initialize_nonblock(sl_bt_api_tx, sl_bt_api_rx, sl_bt_api_peek_rx);
  if (sc == SL_STATUS_OK) {
    // Reset the BLE board
    sl_bt_system_reboot();
    return true;
  }
  return false;
}

void sl_bolton_process()
{
  sl_bt_msg_t event;
  if (sl_bt_pop_event(&event) == SL_STATUS_OK) {
    sl_bt_on_event(&event);
  }
}

void sl_bolton_buffer_received_data(const uint32_t len, const uint8_t *data)
{
  if (!data) {
    return;
  }
  if (len <= (sizeof(rx_buf) - rx_buf_len)) {
    memcpy((void *)&rx_buf[rx_buf_len], (void *)data, (size_t)len);
    rx_buf_len += len;
  }
}

static bool sl_check_for_valid_message_in_buf()
{
  uint32_t msg_length;
  uint32_t header = 0;
  int ret;

  // check if a whole message is present in the input buffer
  // sync to a header start byte
  ret = peek_rx_bytes(1, 0, (uint8_t*)&header);
  // Failed to read header byte
  if (ret < 0) {
    return false;
  }

  // check header start byte
  // Check if it's a BT type event or response
  if (((header & 0xf8) != ( (uint32_t)(sl_bgapi_dev_type_bt) | (uint32_t)sl_bgapi_msg_type_evt)) && ((header & 0xf8) != (uint32_t)(sl_bgapi_dev_type_bt))) {
    // Consume the invalid header start byte
    HAL_NVIC_DisableIRQ(BOLTON_UART_IRQN);
    memmove((void *)rx_buf, (void *)&rx_buf[1], rx_buf_len);
    rx_buf_len--;
    HAL_NVIC_EnableIRQ(BOLTON_UART_IRQN);
    return false;
  }
  // peek the full header
  ret = peek_rx_bytes(SL_BGAPI_MSG_HEADER_LEN - 1, 1, &((uint8_t*)&header)[1]);
  if (ret < 0) {
    return 0;
  }
  // check payload length
  msg_length = SL_BT_MSG_LEN(header);
  if (msg_length > SL_BGAPI_MAX_PAYLOAD_SIZE) {
    // Consume the header start byte to invalidate the message
    //(void)sl_bt_api_input(1, (uint8_t*)&header);
    return false;
  }
  // check if we have enough bytes buffered for the payload
  uint32_t bytes_available = rx_buf_len;
  uint32_t bytes_needed = SL_BGAPI_MSG_HEADER_LEN + msg_length;
  if (bytes_available < bytes_needed) {
    return false;
  }

  memcpy((void *)&rx_buf_out[rx_buf_out_len], rx_buf, rx_buf_len);
  rx_buf_out_len += rx_buf_len;
  rx_buf_len = 0u;
  return true;
}

static int32_t peek_rx_bytes(uint32_t dataLength, uint32_t offset, uint8_t* data)
{
  if (!data) {
    return -1;
  }

  if (dataLength + offset <= rx_buf_len) {
    memcpy((void *)data, (void *)rx_buf + offset, (size_t)dataLength);
  } else {
    dataLength = -1;
  }
  return dataLength;
}
