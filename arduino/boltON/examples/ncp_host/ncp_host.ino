/*
   boltON NCP host example

   This example serves as a starting point for users to develop BLE solutions with boltON.

   The example sets up the boltON board and starts a BLE advertisement.
   It also accepts connections and turns on the onboard LED when a connection is established,
   and turns it off on disconnection. The example also logs state changes on Serial.

   Functionality of this example can be tested with the Simplicity Connect mobile app.

   Get the Simplicity Connect app:
    - https://play.google.com/store/apps/details?id=com.siliconlabs.bledemo
    - https://apps.apple.com/us/app/efr-connect-ble-mobile-app/id1030932759

   Author: Tamas Jozsi (Silicon Labs)
 */

#include <boltON.h>

void sl_bt_on_event(sl_bt_msg_t *evt);
static void ble_initialize_gatt_db();

static uint8_t advertising_set_handle = 0xff; // The advertising set handle allocated by the Bluetooth stack
static const uint8_t advertised_name[] = "boltON Arduino";
static uint16_t gattdb_session_id;
static uint16_t generic_access_service_handle;
static uint16_t device_name_characteristic_handle;

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_BOLTON, OUTPUT);
  digitalWrite(LED_BOLTON, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("boltON by Silicon Labs");
  Serial.println("Initializing BLE...");
  sl_bolton_init(&Serial1);
}

void loop()
{
  sl_bolton_process();
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
    }
    break;

    // -------------------------------
    // This event indicates that a new connection was opened
    case sl_bt_evt_connection_opened_id:
      Serial.println("Connection opened");
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(LED_BOLTON, HIGH);
      break;

    // -------------------------------
    // This event indicates that a connection was closed
    case sl_bt_evt_connection_closed_id:
      Serial.println("Connection closed");
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(LED_BOLTON, LOW);
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
