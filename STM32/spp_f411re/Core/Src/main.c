/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "sl_boltON.h"
#include "sl_ring_buffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define LED_PORT  GPIOA
#define LED_PIN   GPIO_PIN_5

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void sl_bt_on_event(sl_bt_msg_t *evt);
static void ble_initialize_gatt_db();
uint8_t advertising_set_handle = SL_BT_INVALID_ADVERTISING_SET_HANDLE;

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("boltON BLE Serial Port Profile by Silicon Labs\n");
  printf("Initializing BLE...\n");
  bool res = sl_bolton_init();
  assert(res);
  // Initialize UART reception for the USB serial
  HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1);
  // Reset the BLE board
  sl_bt_system_reboot();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// UART ISR handler
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == BOLTON_UART_INSATNCE) {
    sl_bolton_buffer_received_data(1, &bolton_usart_rx_byte);
    HAL_UART_Receive_IT(&BOLTON_UART_HANDLE, &bolton_usart_rx_byte, 1);
  }
  if (huart->Instance == USART2) {
    sl_ringbuffer_write(&uart2_rx_buf, usart2_rx_byte);
    HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1);
  }
}

// printf
int _write(int file, char *data, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)data, len, HAL_MAX_DELAY);
  return len;
}

static void spp_write(uint8_t* data, size_t size)
{
  if (spp_main_state != STATE_SPP_MODE) {
    printf("Write error - SPP not connected\r\n");
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
  printf("Bytes sent: %lu (%lu packets)\r\n",
         p_counters->num_bytes_sent,
         p_counters->num_pack_sent);
  printf("Bytes received: %lu (%lu packets)\r\n",
         p_counters->num_bytes_received,
         p_counters->num_pack_received);
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
      printf("BLE stack booted\n");
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
      printf("System identity address: %02x:%02x:%02x:%02x:%02x:%02x\n",
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
      printf("Started advertising...\n");
      printf("Go to 'https://siliconlabssoftware.github.io/web-bluetooth-spp-application/' to connect your computer\n");
      break;

    // -------------------------------
    // This event indicates that a new connection was opened
    case sl_bt_evt_connection_opened_id:
      printf("Connection opened\n");
      spp_main_state = STATE_CONNECTED;
      spp_connection_handle = evt->data.evt_connection_opened.connection;
      break;

    // -------------------------------
    // This event indicates that a connection was closed
    case sl_bt_evt_connection_closed_id:
      printf("---------------------------\r\n");
      printf("Connection closed\n");
      spp_print_stats(&spp_connection_stat_counter);
      spp_reset_state();
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      assert(sc == SL_STATUS_OK);

      // Restart advertising after the client has disconnected
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      assert(sc == SL_STATUS_OK);
      printf("Restarted advertising...\n");
      break;

    case sl_bt_evt_gatt_mtu_exchanged_id:
      // Calculate maximum data per one notification / write-without-response,
      // this depends on the MTU. Up to ATT_MTU-3 bytes can be sent at once.
      spp_max_packet_size = evt->data.evt_gatt_mtu_exchanged.mtu - 3;
      printf("MTU exchanged, max packet size set to: %d bytes\r\n", evt->data.evt_gatt_mtu_exchanged.mtu);
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
            printf("Ready for SPP communication\r\n");
            printf("---------------------------\r\n");
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
          } else {
            printf("SPP communication stopped\r\n");
            spp_main_state = STATE_CONNECTED;
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
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
          printf("%c", received_char);
        }
        spp_connection_stat_counter.num_pack_received++;
        spp_connection_stat_counter.num_bytes_received += evt->data.evt_gatt_server_attribute_value.value.len;
      }
    }
    break;

    // -------------------------------
    // Default event handler
    default:
      printf("BLE event: 0x%lx\n", SL_BT_MSG_ID(evt->header));
      break;
  }
  (void)sc;
}

static const uint8_t advertised_name[] = "SPP Example";
static uint16_t gattdb_session_id;
static uint16_t service_handle;
static uint16_t characteristic_handle;

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

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
