/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    ncp_host_MCXA156.c
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
/* TODO: insert other include files here. */
#include "sl_boltON.h"

/* TODO: insert other definitions and declarations here. */
void sl_bt_on_event(sl_bt_msg_t *evt);
static void ble_initialize_gatt_db();

// The advertising set handle allocated by the Bluetooth stack
static uint8_t advertising_set_handle = 0xff;

static const uint8_t advertised_name[] = "boltON NXP";
static uint16_t gattdb_session_id;
static uint16_t generic_access_service_handle;
static uint16_t device_name_characteristic_handle;

#include "fsl_common.h"
#include "fsl_lpuart.h"

#define DEMO_LPUART       LPUART2
//#define DEMO_LPUART_CLK_FREQ CLOCK_GetFreq(kCLOCK_LPUART2)

#define BOARD_LED_GPIO     BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN

#define BOARD_BOLTON_LED_GPIO     GPIO3
#define BOARD_BOLTON_LED_GPIO_PIN 17U

void LPUART2_Init(void)
{
  lpuart_config_t config;

  LPUART_GetDefaultConfig(&config);
  config.baudRate_Bps = 115200;
  config.enableTx = true;
  config.enableRx = true;
  config.txFifoWatermark = 0;
  config.rxFifoWatermark = 0;

  LPUART_Init(DEMO_LPUART, &config, BOARD_DEBUG_UART_CLK_FREQ);

  LPUART_EnableInterrupts(DEMO_LPUART, kLPUART_RxDataRegFullInterruptEnable);
  EnableIRQ(LPUART2_IRQn);
}

void init_led_gpio(void)
{
  LED_RED_INIT(LOGIC_LED_OFF);

  GPIO_PinWrite(BOARD_BOLTON_LED_GPIO, BOARD_BOLTON_LED_GPIO_PIN, 0U);
  BOARD_LED_RED_GPIO->PDDR |= (1U << BOARD_BOLTON_LED_GPIO_PIN);
  /*
   #define LED_GPIO GPIO3
   #define LED_PORT 3U
   #define LED_PIN 17U

     // Enable clock for GPIO
     CLOCK_EnableClock(kCLOCK_Gpio3);

     PORT_SetPinMux(GPIO3, 17, kPORT_MuxAsGpio);

     gpio_pin_config_t led_config = {
     kGPIO_DigitalOutput,  // Set as output
     0,                    // Initial output value: 0 = off
     };

     // Initialize the GPIO pin
     GPIO_PinInit(LED_GPIO, LED_PORT, LED_PIN, &led_config);
   */
}

void LPUART2_IRQHandler(void)
{
  if (LPUART_GetStatusFlags(DEMO_LPUART) & kLPUART_RxDataRegFullFlag) {
    uint8_t received = LPUART_ReadByte(DEMO_LPUART);
    sl_bolton_buffer_received_data(1, &received);
  }
}

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
  init_led_gpio();

  PRINTF("boltON by Silicon Labs\n");
  PRINTF("Initializing BLE...\n");
  bool res = sl_bolton_init();
  assert(res);

  /* Enter an infinite loop, just incrementing a counter. */
  while (1) {
    sl_bolton_process();
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

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
    {
      bd_addr address;
      uint8_t address_type;
      uint8_t system_id[8];
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
    }
    break;

    // -------------------------------
    // This event indicates that a new connection was opened
    case sl_bt_evt_connection_opened_id:
      PRINTF("Connection opened\n");
      GPIO_PortClear(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
      GPIO_PortSet(BOARD_BOLTON_LED_GPIO, 1u << BOARD_BOLTON_LED_GPIO_PIN);
      //HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
      //HAL_GPIO_WritePin(BOLTON_LED_PORT, BOLTON_LED_PIN, GPIO_PIN_SET);
      break;

    // -------------------------------
    // This event indicates that a connection was closed
    case sl_bt_evt_connection_closed_id:
      PRINTF("Connection closed\n");
      GPIO_PortSet(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
      GPIO_PortClear(BOARD_BOLTON_LED_GPIO, 1u << BOARD_BOLTON_LED_GPIO_PIN);
      //HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
      //HAL_GPIO_WritePin(BOLTON_LED_PORT, BOLTON_LED_PIN, GPIO_PIN_RESET);
      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      assert(sc == SL_STATUS_OK);

      // Restart advertising after client has disconnected
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      assert(sc == SL_STATUS_OK);
      printf("Restarted advertising...\n");
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler
    default:
      PRINTF("BLE event: 0x%lx\n", SL_BT_MSG_ID(evt->header));
      break;
  }
}

static void ble_initialize_gatt_db()
{
  sl_status_t sc;
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
                                &generic_access_service_handle);
  assert(sc == SL_STATUS_OK);

  // Add the Device Name characteristic to the Generic Access service
  // The value of the Device Name characteristic will be advertised
  const sl_bt_uuid_16_t device_name_characteristic_uuid = { .data = { 0x00, 0x2A } };
  sc = sl_bt_gattdb_add_uuid16_characteristic(gattdb_session_id,
                                              generic_access_service_handle,
                                              SL_BT_GATTDB_CHARACTERISTIC_READ,
                                              0x00,
                                              0x00,
                                              device_name_characteristic_uuid,
                                              sl_bt_gattdb_fixed_length_value,
                                              sizeof(advertised_name) - 1,
                                              sizeof(advertised_name) - 1,
                                              advertised_name,
                                              &device_name_characteristic_handle);
  assert(sc == SL_STATUS_OK);

  // Start the Generic Access service
  sc = sl_bt_gattdb_start_service(gattdb_session_id, generic_access_service_handle);
  assert(sc == SL_STATUS_OK);

  // Commit the GATT DB changes
  sc = sl_bt_gattdb_commit(gattdb_session_id);
  assert(sc == SL_STATUS_OK);
}
