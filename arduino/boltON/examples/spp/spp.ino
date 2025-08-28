/*
   boltON Serial Port Profile (SPP) example

   The Serial Port Profile (SPP) example allows the users to send data through BLE in a bidirectional way similar to UART.
   The example allows the hardware to connect to a web app running in the user's browser and
   exchange messages between with the hardware wirelessly.

   Functionality of this example can be tested with the WEB Bluetooth app:
   https://siliconlabssoftware.github.io/web-bluetooth-spp-application/

   Author: Tamas Jozsi (Silicon Labs)
 */

#include <boltON.h>
#include <sl_ring_buffer.h>

void sl_bt_on_event(sl_bt_msg_t *evt);
static void ble_initialize_gatt_db();

void sl_bt_on_event(sl_bt_msg_t *evt);
static void ble_initialize_gatt_db();
uint8_t advertising_set_handle = SL_BT_INVALID_ADVERTISING_SET_HANDLE;

static const uint8_t advertised_name[] = "SPP Example";
static uint16_t gattdb_session_id;
static uint16_t service_handle;
static uint16_t characteristic_handle;

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

sl_ring_buffer serial_rx_buf;

void setup()
{
  Serial.begin(115200);
  sl_ringbuffer_reset(&serial_rx_buf);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_BOLTON, OUTPUT);
  digitalWrite(LED_BOLTON, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("boltON by Silicon Labs - BLE Serial Port Profile");
  Serial.println("Initializing BLE...");
  sl_bolton_init(&Serial1);
}

void loop()
{
  sl_bolton_process();

  // Buffer all data received on Serial
  while (Serial.available()) {
    uint8_t rx_byte = Serial.read();
    sl_ringbuffer_write(&serial_rx_buf, rx_byte);
  }

  // Handle BLE transmission of the data buffered from Serial
  uint8_t data;
  static uint8_t data_buf[256];
  static uint16_t data_buf_idx = 0u;
  while (sl_ringbuffer_read(&serial_rx_buf, &data)) {
    // Copy one byte into the BLE Tx buffer
    data_buf[data_buf_idx] = data;
    data_buf_idx++;
    // If it's the end of a line - or we reach the max allowed packet size - or we would overrun our buffer - send the data out
    if (data == '\n' || data_buf_idx >= spp_max_packet_size || data_buf_idx >= sizeof(data_buf)) {
      spp_write(data_buf, data_buf_idx);
      data_buf_idx = 0u;
    }
  }
}

/**************************************************************************//**
 * Bluetooth stack event handler
 * This overrides the default weak implementation
 *
 * @param[in] evt Event coming from the Bluetooth stack
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t * evt)
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
      Serial.println("BLE stack booted");
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

      Serial.print("System identity address: ");
      for (int i = 0; i < 6; i++) {
        Serial.print(system_id[i], HEX);
        if (i < 5) {
          Serial.print(":");
        }
      }
      Serial.println();

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
        160,     // min. adv. interval (milliseconds * 1.6)
        160,     // max. adv. interval (milliseconds * 1.6)
        0,     // adv. duration
        0);     // max. num. adv. events
      assert(sc == SL_STATUS_OK);
      // Start advertising and enable connections
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      assert(sc == SL_STATUS_OK);
      Serial.println("Started advertising...");
      Serial.println("Go to 'https://siliconlabssoftware.github.io/web-bluetooth-spp-application/' to connect your computer");
    }
    break;

    // -------------------------------
    // This event indicates that a new connection was opened
    case sl_bt_evt_connection_opened_id:
      Serial.println("Connection opened");
      spp_main_state = STATE_CONNECTED;
      spp_connection_handle = evt->data.evt_connection_opened.connection;
      break;

    // -------------------------------
    // This event indicates that a connection was closed
    case sl_bt_evt_connection_closed_id:
      Serial.println("---------------------------");
      Serial.println("Connection closed");
      spp_print_stats(&spp_connection_stat_counter);
      spp_reset_state();
      set_led_on(false);
      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      assert(sc == SL_STATUS_OK);

      // Restart advertising after client has disconnected
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      assert(sc == SL_STATUS_OK);
      Serial.println("Restarted advertising...");
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
            Serial.println("Ready for SPP communication");
            Serial.println("---------------------------");
            set_led_on(true);
          } else {
            Serial.println("SPP communication stopped");
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
          Serial.print((char)received_char);
        }
        spp_connection_stat_counter.num_pack_received++;
        spp_connection_stat_counter.num_bytes_received += evt->data.evt_gatt_server_attribute_value.value.len;
      }
    }
    break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler
    default:
      Serial.print("BLE event: 0x");
      Serial.println(SL_BT_MSG_ID(evt->header), HEX);
      break;
  }
}

static void spp_write(uint8_t* data, size_t size)
{
  if (spp_main_state != STATE_SPP_MODE) {
    Serial.println("Write error - SPP not connected");
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
  Serial.print("Bytes sent: ");
  Serial.print(p_counters->num_bytes_sent);
  Serial.print(" (");
  Serial.print(p_counters->num_pack_sent);
  Serial.println(" packets)");

  Serial.print("Bytes received: ");
  Serial.print(p_counters->num_bytes_received);
  Serial.print(" (");
  Serial.print(p_counters->num_pack_received);
  Serial.println(" packets)");
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

void set_led_on(bool state)
{
  if (state) {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BOLTON, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_BOLTON, LOW);
  }
}
