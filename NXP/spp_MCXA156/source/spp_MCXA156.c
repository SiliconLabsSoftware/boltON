/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    spp_MCXA156.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_clock.h"
#include "fsl_common.h"
#include "fsl_lpuart.h"

#include "sl_boltON.h"
#include "sl_ring_buffer.h"

void sl_bt_on_event(sl_bt_msg_t *evt);
static void ble_initialize_gatt_db();
uint8_t advertising_set_handle = SL_BT_INVALID_ADVERTISING_SET_HANDLE;

static const uint8_t advertised_name[] = "SPP Example";
static uint16_t gattdb_session_id;
static uint16_t service_handle;
static uint16_t characteristic_handle;

uint8_t usart2_rx_byte;
sl_ring_buffer uart2_rx_buf = { 0 };

typedef enum {
  STATE_DISCONNECTED = 0,
  STATE_ADVERTISING,
  STATE_CONNECTED,
  STATE_SPP_MODE
} spp_state_t;
spp_state_t spp_main_state = STATE_DISCONNECTED;

uint16_t spp_local_gatt_service_handle = 0u;
uint16_t spp_local_data_gatt_characteristic_handle = 0u;
uint8_t spp_connection_handle = SL_BT_INVALID_CONNECTION_HANDLE;
const uint8_t spp_default_max_packet_size = 20u;
uint8_t spp_max_packet_size = spp_default_max_packet_size;

// Bookkeeping struct for storing amount of sent/received data
typedef struct {
  uint32_t num_pack_sent;
  uint32_t num_bytes_sent;
  uint32_t num_pack_received;
  uint32_t num_bytes_received;
} spp_ts_counters;
spp_ts_counters spp_connection_stat_counter = {};

static void spp_write(uint8_t* data, size_t size);
static void spp_print_stats(spp_ts_counters *p_counters);
static void spp_reset_state();

// SPP GATT service UUID: 4880c12c-fdcb-4077-8920-a450d7f9b907
const uuid_128 spp_service_uuid = {
  .data = { 0x07, 0xb9, 0xf9, 0xd7, 0x50, 0xa4, 0x20, 0x89, 0x77, 0x40, 0xcb, 0xfd, 0x2c, 0xc1, 0x80, 0x48 }
};

// SPP data GATT characteristic UUID: fec26ec4-6d71-4442-9f81-55bc21d658d6
const uuid_128 spp_data_characteristic_uuid = {
  .data = { 0xd6, 0x58, 0xd6, 0x21, 0xbc, 0x55, 0x81, 0x9f, 0x42, 0x44, 0x71, 0x6d, 0xc4, 0x6e, 0xc2, 0xfe }
};

#define BOARD_LED_GPIO     BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN

#define BOARD_BOLTON_LED_GPIO     GPIO3
#define BOARD_BOLTON_LED_GPIO_PIN 17U

void LPUART2_Init(void);
void init_led_gpio(void);
void set_led_on(bool state);

/*
 * @brief   Application entry point.
 */
int main(void)
{
  /* Init board hardware. */
  BOARD_InitBootPins();
  BOARD_InitBootClocks();
  BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
  /* Init FSL debug console. */
  BOARD_InitDebugConsole();
#endif

  LPUART2_Init();
  LPUART_EnableInterrupts(LPUART0, kLPUART_RxDataRegFullInterruptEnable);
  EnableIRQ(LPUART0_IRQn);
  init_led_gpio();

  PRINTF("boltON by Silicon Labs - BLE Serial Port Profile\n");
  PRINTF("Initializing BLE...\n");
  bool res = sl_bolton_init();
  assert(res);

  set_led_on(false);

  while (1) {
    sl_bolton_process();

    // Handle BLE transmission of the data received from UART
    uint8_t data;
    static uint8_t data_buf[256];
    static uint16_t data_buf_idx = 0u;
    while (sl_ringbuffer_read(&uart2_rx_buf, &data)) {
      // Buffer the received data
      data_buf[data_buf_idx] = data;
      data_buf_idx++;
      // If it's the end of a line - or we reach the max allowed packet size - or we would overrun our buffer - send the data out
      if (data == '\n' || data_buf_idx >= spp_max_packet_size || data_buf_idx >= sizeof(data_buf)) {
        spp_write(data_buf, data_buf_idx);
        data_buf_idx = 0u;
      }
    }
  }
  return 0;
}

/**************************************************************************//**
 * Bluetooth stack event handler
 * This overrides the default weak implementation
 *
 * @param[in] evt Event coming from the Bluetooth stack
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      PRINTF("BLE stack booted\n");
      ble_initialize_gatt_db();

      // Extract unique ID from BT Address
      sc = sl_bt_gap_get_identity_address(&address, &address_type);
      assert(sc == SL_STATUS_OK);

      // Pad and reverse unique ID to get System ID
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = address.addr[2];
      system_id[4] = address.addr[1];
      system_id[5] = address.addr[0];
      (void)system_id;
      PRINTF("System identity address: %02x:%02x:%02x:%02x:%02x:%02x\n",
             system_id[0],
             system_id[1],
             system_id[2],
             system_id[3],
             system_id[4],
             system_id[5]);

      // Create an advertising set
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      assert(sc == SL_STATUS_OK);

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      assert(sc == SL_STATUS_OK);

      // Set advertising interval to 100ms
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      assert(sc == SL_STATUS_OK);
      // Start advertising and enable connections
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      assert(sc == SL_STATUS_OK);
      PRINTF("Started advertising...\n");
      PRINTF("Go to 'https://siliconlabssoftware.github.io/web-bluetooth-spp-application/' to connect your computer\n");
      break;

    // -------------------------------
    // This event indicates that a new connection was opened
    case sl_bt_evt_connection_opened_id:
      PRINTF("Connection opened\n");
      spp_main_state = STATE_CONNECTED;
      spp_connection_handle = evt->data.evt_connection_opened.connection;
      break;

    // -------------------------------
    // This event indicates that a connection was closed
    case sl_bt_evt_connection_closed_id:
      PRINTF("---------------------------\r\n");
      PRINTF("Connection closed\n");
      spp_print_stats(&spp_connection_stat_counter);
      spp_reset_state();
      set_led_on(false);
      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      assert(sc == SL_STATUS_OK);

      // Restart advertising after the client has disconnected
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      assert(sc == SL_STATUS_OK);
      PRINTF("Restarted advertising...\n");
      break;

    case sl_bt_evt_gatt_mtu_exchanged_id:
      // Calculate maximum data per one notification / write-without-response,
      // this depends on the MTU. Up to ATT_MTU-3 bytes can be sent at once.
      spp_max_packet_size = evt->data.evt_gatt_mtu_exchanged.mtu - 3;
      PRINTF("MTU exchanged, max packet size set to: %d bytes\r\n", evt->data.evt_gatt_mtu_exchanged.mtu);
      break;

    case sl_bt_evt_gatt_server_characteristic_status_id:
    {
      sl_bt_evt_gatt_server_characteristic_status_t char_status;
      char_status = evt->data.evt_gatt_server_characteristic_status;
      if (char_status.characteristic == spp_local_data_gatt_characteristic_handle) {
        if (char_status.status_flags == sl_bt_gatt_server_client_config) {
          // Characteristic client configuration (CCC) for spp_data has been changed
          if (char_status.client_config_flags == sl_bt_gatt_server_notification) {
            spp_main_state = STATE_SPP_MODE;
            PRINTF("Ready for SPP communication\r\n");
            PRINTF("---------------------------\r\n");
            set_led_on(true);
          } else {
            PRINTF("SPP communication stopped\r\n");
            spp_main_state = STATE_CONNECTED;
            set_led_on(false);
          }
        }
      }
    }
    break;

    case sl_bt_evt_gatt_server_attribute_value_id:
    {
      if (evt->data.evt_gatt_server_attribute_value.value.len != 0u) {
        for (uint8_t i = 0u; i < evt->data.evt_gatt_server_attribute_value.value.len; i++) {
          uint8_t received_char = evt->data.evt_gatt_server_attribute_value.value.data[i];
          PRINTF("%c", received_char);
        }
        spp_connection_stat_counter.num_pack_received++;
        spp_connection_stat_counter.num_bytes_received += evt->data.evt_gatt_server_attribute_value.value.len;
      }
    }
    break;

    // -------------------------------
    // Default event handler
    default:
      PRINTF("BLE event: 0x%lx\n", SL_BT_MSG_ID(evt->header));
      break;
  }
  (void)sc;
}

static void spp_write(uint8_t* data, size_t size)
{
  if (spp_main_state != STATE_SPP_MODE) {
    PRINTF("Write error - SPP not connected\r\n");
    return;
  }
  sl_status_t sc = sl_bt_gatt_server_send_notification(spp_connection_handle,
                                                       spp_local_data_gatt_characteristic_handle,
                                                       size,
                                                       data);
  if (sc == SL_STATUS_OK) {
    spp_connection_stat_counter.num_pack_sent++;
    spp_connection_stat_counter.num_bytes_sent += size;
  }
}

static void spp_reset_state()
{
  spp_connection_handle = SL_BT_INVALID_CONNECTION_HANDLE;
  spp_main_state = STATE_ADVERTISING;
  spp_max_packet_size = spp_default_max_packet_size;
  memset(&spp_connection_stat_counter, 0u, sizeof(spp_ts_counters));
}

static void spp_print_stats(spp_ts_counters *p_counters)
{
  PRINTF("Bytes sent: %lu (%lu packets)\r\n",
         p_counters->num_bytes_sent,
         p_counters->num_pack_sent);
  PRINTF("Bytes received: %lu (%lu packets)\r\n",
         p_counters->num_bytes_received,
         p_counters->num_pack_received);
}

static void ble_initialize_gatt_db()
{
  sl_status_t sc;
  (void)sc;
  // Create a new GATT database
  sc = sl_bt_gattdb_new_session(&gattdb_session_id);
  assert(sc == SL_STATUS_OK);

  // Add the Generic Access service to the GATT DB
  const uint8_t generic_access_service_uuid[] = { 0x00, 0x18 };
  sc = sl_bt_gattdb_add_service(gattdb_session_id,
                                sl_bt_gattdb_primary_service,
                                SL_BT_GATTDB_ADVERTISED_SERVICE,
                                sizeof(generic_access_service_uuid),
                                generic_access_service_uuid,
                                &service_handle);
  assert(sc == SL_STATUS_OK);

  // Add the Device Name characteristic to the Generic Access service
  // The value of the Device Name characteristic will be advertised
  const sl_bt_uuid_16_t device_name_characteristic_uuid = { .data = { 0x00, 0x2A } };
  sc = sl_bt_gattdb_add_uuid16_characteristic(gattdb_session_id,
                                              service_handle,
                                              SL_BT_GATTDB_CHARACTERISTIC_READ,
                                              0x00,
                                              0x00,
                                              device_name_characteristic_uuid,
                                              sl_bt_gattdb_fixed_length_value,
                                              sizeof(advertised_name) - 1,
                                              sizeof(advertised_name) - 1,
                                              advertised_name,
                                              &characteristic_handle);
  assert(sc == SL_STATUS_OK);

  // Start the Generic Access service
  sc = sl_bt_gattdb_start_service(gattdb_session_id, service_handle);
  assert(sc == SL_STATUS_OK);

  // Add the SPP service to the GATT DB
  sc = sl_bt_gattdb_add_service(gattdb_session_id,
                                sl_bt_gattdb_primary_service,
                                SL_BT_GATTDB_ADVERTISED_SERVICE,
                                sizeof(spp_service_uuid),
                                spp_service_uuid.data,
                                &spp_local_gatt_service_handle);
  assert(sc == SL_STATUS_OK);

  // Add the 'SPP data' characteristic to the SPP service
  uint8_t spp_data_char_init_value = 0;
  sc = sl_bt_gattdb_add_uuid128_characteristic(gattdb_session_id,
                                               spp_local_gatt_service_handle,
                                               SL_BT_GATTDB_CHARACTERISTIC_WRITE_NO_RESPONSE | SL_BT_GATTDB_CHARACTERISTIC_NOTIFY,
                                               0x00,
                                               0x00,
                                               spp_data_characteristic_uuid,
                                               sl_bt_gattdb_fixed_length_value,
                                               250,                                  // max length
                                               sizeof(spp_data_char_init_value),     // initial value length
                                               &spp_data_char_init_value,            // initial value
                                               &spp_local_data_gatt_characteristic_handle);
  assert(sc == SL_STATUS_OK);

  // Start the SPP service
  sc = sl_bt_gattdb_start_service(gattdb_session_id, spp_local_gatt_service_handle);
  assert(sc == SL_STATUS_OK);

  // Commit the GATT DB changes
  sc = sl_bt_gattdb_commit(gattdb_session_id);
  assert(sc == SL_STATUS_OK);
}

void LPUART2_Init(void)
{
  lpuart_config_t config;

  LPUART_GetDefaultConfig(&config);
  config.baudRate_Bps = 115200;
  config.enableTx = true;
  config.enableRx = true;
  config.txFifoWatermark = 0;
  config.rxFifoWatermark = 0;

  LPUART_Init(LPUART2, &config, BOARD_DEBUG_UART_CLK_FREQ);

  LPUART_EnableInterrupts(LPUART2, kLPUART_RxDataRegFullInterruptEnable);
  EnableIRQ(LPUART2_IRQn);
}

void LPUART2_IRQHandler(void)
{
  if (LPUART_GetStatusFlags(LPUART2) & kLPUART_RxDataRegFullFlag) {
    uint8_t received = LPUART_ReadByte(LPUART2);
    sl_bolton_buffer_received_data(1, &received);
  }
}

void LPUART0_IRQHandler(void)
{
  if (LPUART_GetStatusFlags(LPUART0) & kLPUART_RxDataRegFullFlag) {
    uint8_t received = LPUART_ReadByte(LPUART0);
    sl_ringbuffer_write(&uart2_rx_buf, received);
  }
}

void init_led_gpio(void)
{
  LED_RED_INIT(LOGIC_LED_OFF);

  GPIO_PinWrite(BOARD_BOLTON_LED_GPIO, BOARD_BOLTON_LED_GPIO_PIN, 0U);
  BOARD_LED_RED_GPIO->PDDR |= (1U << BOARD_BOLTON_LED_GPIO_PIN);
}

void set_led_on(bool state)
{
  if (state) {
    GPIO_PortClear(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
    GPIO_PortSet(BOARD_BOLTON_LED_GPIO, 1u << BOARD_BOLTON_LED_GPIO_PIN);
  } else {
    GPIO_PortSet(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
    GPIO_PortClear(BOARD_BOLTON_LED_GPIO, 1u << BOARD_BOLTON_LED_GPIO_PIN);
  }
}
