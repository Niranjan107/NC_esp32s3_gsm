/**
 * @file ble_spp.c
 * @brief BLE SPP (Serial Port Profile) implementation using Nordic UART Service (NUS)
 *
 * PURPOSE:
 * This module provides Bluetooth Low Energy (BLE) serial communication between
 * the ESP32-S3 connector and mobile apps (Android/iOS). It emulates a serial port
 * over BLE using the industry-standard Nordic UART Service (NUS) protocol.
 *
 * WHY NUS (Nordic UART Service)?
 * - Industry standard - supported by most BLE serial apps
 * - Compatible with nRF Connect, Adafruit Bluefruit, and custom apps
 * - Two-way communication: RX (receive from phone) and TX (send to phone)
 * - Well-documented 128-bit UUIDs recognized by mobile frameworks
 *
 * ARCHITECTURE:
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                          Mobile App (Phone)                             │
 * │  - Scans for BLE devices with NUS service UUID                          │
 * │  - Connects to "NCLite-ESP32"                                           │
 * │  - Writes JSON commands to RX characteristic (6E400002...)              │
 * │  - Subscribes to TX characteristic notifications (6E400003...)          │
 * └─────────────────────────────────────────────────────────────────────────┘
 *                                    │
 *                            BLE Connection
 *                                    │
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                      ESP32-S3 BLE GATT Server                           │
 * │  ┌───────────────────────────────────────────────────────────────────┐  │
 * │  │ Nordic UART Service (6E400001-B5A3-F393-E0A9-E50E24DCCA9E)        │  │
 * │  │   ├── RX Characteristic (6E400002...) - Write, Write No Response  │  │
 * │  │   │   └── Phone writes JSON commands here → process_ble_rx_data() │  │
 * │  │   └── TX Characteristic (6E400003...) - Notify, Read              │  │
 * │  │       └── ESP32 sends responses here → ble_spp_send()             │  │
 * │  └───────────────────────────────────────────────────────────────────┘  │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * DATA FLOW:
 * 1. PHONE → ESP32 (Commands):
 *    Phone writes to RX characteristic → ESP_GATTS_WRITE_EVT triggered
 *    → process_ble_rx_data() accumulates data → parse_and_process_commands()
 *
 * 2. ESP32 → PHONE (Responses):
 *    WM/MA data arrives → wm_ble_data_callback()/ma_ble_data_callback()
 *    → ble_spp_send() → esp_ble_gatts_send_indicate() → Phone receives notification
 *
 * JSON COMMAND PROTOCOL (received from phone):
 *   {"C":"GETCONFIG"}#          - Get device configuration
 *   {"C":"SETWM","D":"9600,0"}# - Set WM baud/parity
 *   {"C":"PRINT","D":"..."}#    - Print receipt
 *   {"C":"REPRINT"}#            - Reprint last receipt
 *   {"C":"STATUS"}#             - Get device status
 *
 * Based on ESP-IDF GATT server example, adapted for NCLite dairy connector.
 * Compatible with Pico2W Bluetooth connector behavior.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "ble_spp.h"
#include "cmd_parser.h"

static const char *TAG = "BLE_SPP";

/*******************************************************************************
 * Private Defines
 *
 * PURPOSE: Constants for BLE GATT server configuration
 ******************************************************************************/

/**
 * GATTS_NUM_HANDLE: Number of attribute handles to allocate for GATT service
 * Each attribute (service, characteristic, descriptor) needs a handle.
 * NUS service needs: 1 service + 2 chars (RX/TX) * 2 handles each + 1 CCCD = ~7
 * We allocate 10 for safety margin.
 */
#define GATTS_NUM_HANDLE        10

/**
 * PROFILE_APP_ID: Application ID for GATT profile registration
 * ESP-IDF allows multiple GATT profiles; we only need one for NUS service
 */
#define PROFILE_APP_ID          0

/**
 * PREPARE_BUF_MAX_SIZE: Buffer size for BLE "prepared writes" (long writes)
 * When phone sends data larger than MTU (typically 20 bytes default),
 * it uses the prepared write protocol which fragments data across multiple writes.
 * This buffer accumulates fragments before the final execute write.
 */
#define PREPARE_BUF_MAX_SIZE    1024

/*******************************************************************************
 * Nordic UART Service UUIDs (128-bit) - little endian format
 *
 * PURPOSE: Standard Nordic UART Service UUIDs recognized by mobile BLE apps
 *
 * WHY LITTLE ENDIAN?
 * BLE transmits UUIDs in little-endian byte order, so we store them reversed.
 * Human-readable UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * Stored as bytes:     9E CA DC 24 0E E5 A9 E0 93 F3 A3 B5 01 00 40 6E
 *
 * UUID BREAKDOWN (for 6E400001...):
 * - 6E4000XX = Nordic Semiconductor vendor-specific base
 * - XX=01 = Service UUID
 * - XX=02 = RX Characteristic (phone writes to this)
 * - XX=03 = TX Characteristic (phone reads/subscribes to this)
 ******************************************************************************/

/**
 * NUS Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * This is the primary service that mobile apps scan for when looking
 * for NUS-compatible devices. When phone sees this UUID, it knows the
 * device supports serial-over-BLE communication.
 */
static uint8_t nus_service_uuid128[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

/**
 * NUS RX Characteristic UUID: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
 * PURPOSE: Receive data FROM mobile phone (phone writes here)
 * PROPERTIES: Write, Write Without Response
 *
 * Phone → ESP32 data flow:
 * Phone app calls writeCharacteristic(rx_uuid, json_command)
 * → ESP_GATTS_WRITE_EVT triggered → process_ble_rx_data()
 */
static uint8_t nus_rx_uuid128[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
};

/**
 * NUS TX Characteristic UUID: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
 * PURPOSE: Send data TO mobile phone (ESP32 notifies here)
 * PROPERTIES: Notify, Read
 *
 * ESP32 → Phone data flow:
 * WM/MA data arrives → ble_spp_send() → esp_ble_gatts_send_indicate()
 * → Phone receives BLE notification with data
 *
 * NOTE: Phone must enable notifications (write 0x0001 to CCCD descriptor)
 * before it can receive data from ESP32.
 */
static uint8_t nus_tx_uuid128[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
};

/**
 * Advertising Service UUID (same as NUS service)
 * PURPOSE: Included in BLE advertising packets so phones can filter/find us
 * When phone scans for devices, it can filter by this UUID to find NUS devices.
 */
static uint8_t adv_service_uuid128[16] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

/*******************************************************************************
 * Private Variables
 *
 * PURPOSE: Module-level state for BLE communication
 ******************************************************************************/

/**
 * Prepared Write Buffer
 * PURPOSE: Accumulate data fragments during BLE "prepared/long writes"
 *
 * WHY NEEDED?
 * BLE has limited MTU (Maximum Transmission Unit), typically 20-512 bytes.
 * When phone sends data larger than MTU, it uses "prepared write" protocol:
 * 1. Phone sends multiple ESP_GATTS_WRITE_EVT with is_prep=true
 * 2. Each fragment is accumulated in prepare_buf at specified offset
 * 3. Phone sends ESP_GATTS_EXEC_WRITE_EVT to commit all fragments
 * 4. We then process the complete data from prepare_buf
 */
static uint8_t prepare_buf[PREPARE_BUF_MAX_SIZE];
static uint16_t prepare_len = 0;

/**
 * JSON Command Buffer
 * PURPOSE: Accumulate JSON command characters until terminator '#' is received
 *
 * WHY NEEDED?
 * Mobile app sends JSON commands like: {"C":"GETCONFIG"}#
 * BLE may fragment this across multiple write events.
 * We accumulate characters until we see '#' or newline, then parse the JSON.
 *
 * PROTOCOL:
 * - Commands start with '{' (JSON object)
 * - Commands end with '#' terminator
 * - Only printable ASCII (32-126) is accumulated
 */
static char s_cmd_buffer[BLE_RX_BUFFER_SIZE];
static int s_cmd_index = 0;

/**
 * User-registered RX callback
 * PURPOSE: Allow other modules to receive raw BLE data
 * Set via ble_spp_register_rx_callback()
 */
static ble_spp_rx_callback_t s_rx_callback = NULL;

/**
 * BLE Connection State Machine
 * PURPOSE: Track current BLE state for status queries
 * States: IDLE → ADVERTISING → CONNECTED → DISCONNECTED → ADVERTISING
 */
static ble_spp_state_t s_ble_state = BLE_STATE_IDLE;

/**
 * BLE Advertising Data
 * PURPOSE: Define what data is broadcast in BLE advertising packets
 *
 * WHY THESE SETTINGS?
 * - include_name=true: Broadcast device name so phones can identify us
 * - include_txpower=true: Help phones estimate distance
 * - p_service_uuid: Include NUS UUID so apps can filter for our device
 * - flag: General discoverable + BLE only (no classic Bluetooth)
 *
 * When phone scans for BLE devices, it receives this advertising data.
 */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,          // This is main advertising data, not scan response
    .include_name = true,           // Include device name "NCLite-ESP32"
    .include_txpower = true,        // Include TX power level for distance estimation
    .min_interval = 0x0006,         // Min connection interval (7.5ms units) = 7.5ms
    .max_interval = 0x0010,         // Max connection interval = 20ms
    .appearance = 0x00,             // Generic device appearance
    .manufacturer_len = 0,          // No manufacturer-specific data
    .p_manufacturer_data = NULL,
    .service_data_len = 0,          // No service-specific data
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),  // Include NUS service UUID
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),  // Discoverable, BLE only
};

/**
 * Scan Response Data
 * PURPOSE: Additional data sent when phone actively scans (vs passive scan)
 * Contains device name for phones that request more info after seeing adv packet.
 */
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,           // This is scan response data
    .include_name = true,           // Include device name
    .include_txpower = true,        // Include TX power
};

/**
 * Advertising Parameters
 * PURPOSE: Control how often and on which channels we advertise
 *
 * WHY THESE VALUES?
 * - adv_int_min/max: 20ms-40ms interval = good balance of discoverability vs power
 * - ADV_TYPE_IND: Connectable, scannable advertising (allow connections)
 * - ADV_CHNL_ALL: Advertise on all 3 BLE advertising channels (37, 38, 39)
 * - ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY: Allow any device to connect
 */
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,            // Min advertising interval (0.625ms units) = 20ms
    .adv_int_max = 0x40,            // Max advertising interval = 40ms
    .adv_type = ADV_TYPE_IND,       // Connectable undirected advertising
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,  // Use public Bluetooth address
    .channel_map = ADV_CHNL_ALL,    // Advertise on channels 37, 38, 39
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,  // Accept all connections
};

/**
 * GATT Profile Instance Structure
 * PURPOSE: Store all state for our NUS GATT service
 *
 * This structure holds:
 * - Interface and connection IDs assigned by Bluedroid stack
 * - Handles for service, characteristics, and descriptors
 * - Connection state flags
 *
 * HANDLE EXPLANATION:
 * When we create GATT attributes, ESP-IDF assigns 16-bit handles.
 * These handles are used to identify attributes in all GATT operations.
 */
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;        // Callback function for GATT events
    uint16_t gatts_if;              // GATT interface ID (assigned on registration)
    uint16_t app_id;                // Application ID (we use PROFILE_APP_ID=0)
    uint16_t conn_id;               // Connection ID (assigned when phone connects)
    uint16_t service_handle;        // Handle for NUS service
    esp_gatt_srvc_id_t service_id;  // Service ID structure with UUID
    uint16_t char_rx_handle;        // Handle for RX characteristic (phone writes here)
    uint16_t char_tx_handle;        // Handle for TX characteristic (we notify here)
    uint16_t descr_tx_handle;       // Handle for TX CCCD descriptor (notification enable)
    esp_bt_uuid_t char_rx_uuid;     // UUID for RX characteristic
    esp_bt_uuid_t char_tx_uuid;     // UUID for TX characteristic
    bool is_connected;              // True when phone is connected
    bool notify_enabled;            // True when phone has enabled notifications
};

/* Forward declaration of profile event handler */
static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param);

/**
 * Global Profile Instance
 * PURPOSE: Single instance of our NUS GATT profile
 * Initialized with defaults, populated during BLE initialization
 */
static struct gatts_profile_inst gl_profile = {
    .gatts_cb = gatts_profile_event_handler,  // Event handler callback
    .gatts_if = ESP_GATT_IF_NONE,             // No interface yet
    .is_connected = false,                     // Not connected initially
    .notify_enabled = false,                   // Notifications not enabled
};

/*******************************************************************************
 * Private Functions
 *
 * PURPOSE: Internal helper functions for BLE GATT server operation
 ******************************************************************************/

/**
 * @brief Process received BLE data (JSON commands from mobile app)
 *
 * PURPOSE:
 * This function is called when phone writes data to the RX characteristic.
 * It accumulates bytes into s_cmd_buffer until a command terminator is found,
 * then parses and executes the JSON command.
 *
 * INPUT:
 * @param data - Raw bytes received from phone (may be partial command)
 * @param len  - Number of bytes received
 *
 * OUTPUT:
 * No return value. Side effects:
 * - Accumulates data in s_cmd_buffer
 * - When complete command found: calls parse_and_process_commands()
 * - Calls s_rx_callback if registered
 *
 * PROTOCOL:
 * Commands are JSON objects terminated with '#':
 *   {"C":"GETCONFIG"}#
 *   {"C":"SETWM","D":"9600,0"}#
 *
 * DATA FLOW:
 * Phone writes data → ESP_GATTS_WRITE_EVT → process_ble_rx_data()
 * → accumulate in buffer → find '#' → parse_and_process_commands()
 */
static void process_ble_rx_data(const uint8_t *data, uint16_t len)
{
    for (int i = 0; i < len; i++) {
        char ch = (char)data[i];

        // Command terminator '#' or newline - process command
        if (ch == '#' || ch == '\r' || ch == '\n') {
            if (s_cmd_index > 0) {
                s_cmd_buffer[s_cmd_index] = '\0';

                // Only process if it looks like JSON (starts with '{')
                if (s_cmd_buffer[0] == '{') {
                    ESP_LOGI(TAG, "BLE RX Command: %s", s_cmd_buffer);
                    parse_and_process_commands(s_cmd_buffer, s_cmd_index, CMD_SRC_BLE);
                }
            }
            // Reset buffer
            s_cmd_index = 0;
            memset(s_cmd_buffer, 0, sizeof(s_cmd_buffer));
            continue;
        }

        // Accumulate printable characters
        if (ch >= 32 && ch <= 126) {
            if (s_cmd_index < BLE_RX_BUFFER_SIZE - 1) {
                s_cmd_buffer[s_cmd_index++] = ch;
            }
        }
    }

    // Also call user callback if registered
    if (s_rx_callback != NULL) {
        s_rx_callback((const char *)data, len);
    }
}

/**
 * @brief Compare two 128-bit UUIDs for equality
 *
 * PURPOSE:
 * Helper function to check if a received UUID matches our expected UUID.
 * Used to identify which characteristic is being accessed in GATT events.
 *
 * INPUT:
 * @param uuid1 - First UUID (16 bytes)
 * @param uuid2 - Second UUID (16 bytes)
 *
 * OUTPUT:
 * @return true if UUIDs match, false otherwise
 */
static bool uuid128_cmp(uint8_t *uuid1, uint8_t *uuid2)
{
    return memcmp(uuid1, uuid2, 16) == 0;
}

/**
 * @brief GAP (Generic Access Profile) Event Handler
 *
 * PURPOSE:
 * Handle BLE GAP events related to advertising and connection management.
 * GAP controls how devices discover each other and establish connections.
 *
 * INPUT:
 * @param event - GAP event type (advertising complete, connection params, etc.)
 * @param param - Event-specific parameters
 *
 * OUTPUT:
 * No return value. Side effects:
 * - Starts advertising when scan response is set
 * - Updates s_ble_state when advertising starts
 *
 * EVENT FLOW:
 * 1. ADV_DATA_SET_COMPLETE_EVT: Advertising data configured
 * 2. SCAN_RSP_DATA_SET_COMPLETE_EVT: Scan response configured → start advertising
 * 3. ADV_START_COMPLETE_EVT: Advertising started (device now discoverable)
 * 4. ADV_STOP_COMPLETE_EVT: Advertising stopped (usually after connection)
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        // Main advertising data has been set in BLE controller
        ESP_LOGD(TAG, "ADV data set complete");
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        // Scan response data set - now we can start advertising
        ESP_LOGI(TAG, "Scan response set, starting advertising...");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // Advertising has started (or failed)
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed");
        } else {
            s_ble_state = BLE_STATE_ADVERTISING;
            ESP_LOGI(TAG, "Advertising started - Device: %s", BLE_DEVICE_NAME);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        // Advertising stopped (typically when phone connects)
        ESP_LOGI(TAG, "Advertising stopped");
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        // Connection parameters updated (interval, latency, timeout)
        ESP_LOGD(TAG, "Connection params updated");
        break;

    default:
        break;
    }
}

/**
 * @brief GATT Server Profile Event Handler
 *
 * PURPOSE:
 * Handle all GATT server events for our NUS profile. This is the main
 * event handler that processes service creation, characteristic operations,
 * client connections, and data transfers.
 *
 * INPUT:
 * @param event    - GATT event type
 * @param gatts_if - GATT server interface ID
 * @param param    - Event-specific parameters
 *
 * OUTPUT:
 * No return value. Side effects depend on event type:
 * - REG_EVT: Creates NUS service
 * - CREATE_EVT: Adds RX characteristic
 * - ADD_CHAR_EVT: Adds TX characteristic and CCCD descriptor
 * - CONNECT_EVT: Stores connection ID, sends "nclite_connected"
 * - DISCONNECT_EVT: Restarts advertising
 * - WRITE_EVT: Processes received data from phone
 *
 * EVENT SEQUENCE (initialization):
 * 1. REG_EVT → Set device name, configure advertising, create service
 * 2. CREATE_EVT → Service created, add RX characteristic
 * 3. ADD_CHAR_EVT (RX) → Add TX characteristic
 * 4. ADD_CHAR_EVT (TX) → Add CCCD descriptor
 * 5. ADD_CHAR_DESCR_EVT → Start service
 * 6. START_EVT → Service running, ready for connections
 *
 * EVENT SEQUENCE (connection):
 * 1. CONNECT_EVT → Phone connected, update params, send welcome message
 * 2. WRITE_EVT (CCCD) → Phone enables notifications
 * 3. WRITE_EVT (RX) → Phone sends JSON commands
 * 4. DISCONNECT_EVT → Phone disconnected, restart advertising
 */
static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "GATT server registered");

        // Setup service with 128-bit UUID
        gl_profile.service_id.is_primary = true;
        gl_profile.service_id.id.inst_id = 0x00;
        gl_profile.service_id.id.uuid.len = ESP_UUID_LEN_128;
        memcpy(gl_profile.service_id.id.uuid.uuid.uuid128, nus_service_uuid128, 16);

        esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);
        esp_ble_gap_config_adv_data(&scan_rsp_data);
        esp_ble_gatts_create_service(gatts_if, &gl_profile.service_id, GATTS_NUM_HANDLE);
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(TAG, "Service created, handle=%d", param->create.service_handle);
        gl_profile.service_handle = param->create.service_handle;

        // Add RX characteristic (Write - receive data from phone)
        gl_profile.char_rx_uuid.len = ESP_UUID_LEN_128;
        memcpy(gl_profile.char_rx_uuid.uuid.uuid128, nus_rx_uuid128, 16);

        esp_ble_gatts_add_char(gl_profile.service_handle,
                               &gl_profile.char_rx_uuid,
                               ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
                               NULL, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.char_uuid.len == ESP_UUID_LEN_128 &&
            uuid128_cmp(param->add_char.char_uuid.uuid.uuid128, nus_rx_uuid128)) {
            ESP_LOGI(TAG, "RX Characteristic added, handle=%d", param->add_char.attr_handle);
            gl_profile.char_rx_handle = param->add_char.attr_handle;

            // Add TX characteristic (Notify - send data to phone)
            gl_profile.char_tx_uuid.len = ESP_UUID_LEN_128;
            memcpy(gl_profile.char_tx_uuid.uuid.uuid128, nus_tx_uuid128, 16);

            esp_ble_gatts_add_char(gl_profile.service_handle,
                                   &gl_profile.char_tx_uuid,
                                   ESP_GATT_PERM_READ,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                   NULL, NULL);
        } else if (param->add_char.char_uuid.len == ESP_UUID_LEN_128 &&
                   uuid128_cmp(param->add_char.char_uuid.uuid.uuid128, nus_tx_uuid128)) {
            ESP_LOGI(TAG, "TX Characteristic added, handle=%d", param->add_char.attr_handle);
            gl_profile.char_tx_handle = param->add_char.attr_handle;

            // Add CCCD descriptor for TX characteristic (required for notifications)
            esp_bt_uuid_t descr_uuid;
            descr_uuid.len = ESP_UUID_LEN_16;
            descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
            esp_ble_gatts_add_char_descr(gl_profile.service_handle,
                                          &descr_uuid,
                                          ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                          NULL, NULL);
        }
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        ESP_LOGI(TAG, "CCCD Descriptor added, handle=%d", param->add_char_descr.attr_handle);
        gl_profile.descr_tx_handle = param->add_char_descr.attr_handle;
        // Start service after descriptor is added
        esp_ble_gatts_start_service(gl_profile.service_handle);
        break;

    case ESP_GATTS_START_EVT:
        ESP_LOGI(TAG, "Service started");
        break;

    case ESP_GATTS_CONNECT_EVT:
        gl_profile.conn_id = param->connect.conn_id;
        gl_profile.is_connected = true;
        s_ble_state = BLE_STATE_CONNECTED;
        ESP_LOGI(TAG, "BLE Client connected, conn_id=%d", param->connect.conn_id);

        // Update connection parameters
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency = 0;
        conn_params.max_int = 0x20;
        conn_params.min_int = 0x10;
        conn_params.timeout = 400;
        esp_ble_gap_update_conn_params(&conn_params);

        // Send connection notification to app (like Pico2W)
        vTaskDelay(pdMS_TO_TICKS(500));
        send_response("nclite_connected", 0, NULL);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        gl_profile.is_connected = false;
        gl_profile.notify_enabled = false;
        s_ble_state = BLE_STATE_DISCONNECTED;
        ESP_LOGI(TAG, "BLE Client disconnected, reason=0x%x", param->disconnect.reason);

        // Reset buffers
        prepare_len = 0;
        memset(prepare_buf, 0, PREPARE_BUF_MAX_SIZE);
        s_cmd_index = 0;
        memset(s_cmd_buffer, 0, sizeof(s_cmd_buffer));

        // Restart advertising
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_WRITE_EVT:
        if (param->write.is_prep) {
            // Handle prepared/long write - accumulate data in buffer
            if (param->write.offset + param->write.len <= PREPARE_BUF_MAX_SIZE) {
                memcpy(prepare_buf + param->write.offset, param->write.value, param->write.len);
                if (param->write.offset + param->write.len > prepare_len) {
                    prepare_len = param->write.offset + param->write.len;
                }
                ESP_LOGD(TAG, "Prepared write: offset=%d, len=%d", param->write.offset, param->write.len);
            }

            // Send response for prepared write
            if (param->write.need_rsp) {
                esp_gatt_rsp_t rsp;
                memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
                rsp.attr_value.handle = param->write.handle;
                rsp.attr_value.len = param->write.len;
                rsp.attr_value.offset = param->write.offset;
                memcpy(rsp.attr_value.value, param->write.value, param->write.len);
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, &rsp);
            }
        } else {
            // Regular write - check if this is data to RX characteristic
            if (param->write.handle == gl_profile.char_rx_handle) {
                // Process received data (JSON commands)
                process_ble_rx_data(param->write.value, param->write.len);
            }

            // Check if this is CCCD write (enable/disable notifications)
            if (param->write.handle == gl_profile.descr_tx_handle && param->write.len == 2) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001) {
                    gl_profile.notify_enabled = true;
                    ESP_LOGI(TAG, "Notifications enabled");
                } else if (descr_value == 0x0000) {
                    gl_profile.notify_enabled = false;
                    ESP_LOGI(TAG, "Notifications disabled");
                }
            }

            // Send response if needed
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                            param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }
        break;

    case ESP_GATTS_EXEC_WRITE_EVT:
        // Execute prepared write - process accumulated data
        ESP_LOGD(TAG, "Execute write, flag=%d, len=%d", param->exec_write.exec_write_flag, prepare_len);

        if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_len > 0) {
            // Process the complete long write data
            process_ble_rx_data(prepare_buf, prepare_len);
        }

        // Reset buffer for next long write
        prepare_len = 0;
        memset(prepare_buf, 0, PREPARE_BUF_MAX_SIZE);

        // Send response
        esp_ble_gatts_send_response(gatts_if, param->exec_write.conn_id,
                                    param->exec_write.trans_id, ESP_GATT_OK, NULL);
        break;

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "MTU set to %d", param->mtu.mtu);
        break;

    default:
        break;
    }
}

/**
 * @brief Main GATT Server Event Handler (dispatcher)
 *
 * PURPOSE:
 * Top-level dispatcher for all GATT server events. Routes events to the
 * appropriate profile event handler (gatts_profile_event_handler).
 *
 * WHY NEEDED?
 * ESP-IDF's Bluedroid stack sends all GATT events to a single callback.
 * This dispatcher stores the interface ID on registration and routes
 * events to the correct profile handler.
 *
 * INPUT:
 * @param event    - GATT event type
 * @param gatts_if - GATT server interface (assigned by Bluedroid)
 * @param param    - Event-specific parameters
 *
 * OUTPUT:
 * No return value. Routes events to gatts_profile_event_handler().
 *
 * FLOW:
 * 1. On REG_EVT: Store assigned gatts_if in gl_profile
 * 2. For all events: If gatts_if matches our profile, call profile handler
 */
static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    // On registration, store the assigned interface ID
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile.gatts_if = gatts_if;
        } else {
            ESP_LOGE(TAG, "Reg app failed, status=%d", param->reg.status);
            return;
        }
    }

    // Route events to profile handler if interface matches
    // ESP_GATT_IF_NONE means event applies to all interfaces
    if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile.gatts_if) {
        if (gl_profile.gatts_cb) {
            gl_profile.gatts_cb(event, gatts_if, param);
        }
    }
}

/*******************************************************************************
 * Public Functions
 *
 * PURPOSE: External API for BLE SPP module
 * These functions are called by other modules (main.c, cmd_parser.c) to:
 * - Initialize/deinitialize BLE
 * - Send data to mobile phone
 * - Query connection state
 ******************************************************************************/

/**
 * @brief Initialize the BLE SPP module
 *
 * PURPOSE:
 * Initialize the entire BLE stack (controller, Bluedroid, GATT server)
 * and start advertising so mobile phones can discover and connect.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return ESP_OK on success, error code on failure
 *
 * INITIALIZATION SEQUENCE:
 * 1. Release classic BT memory (BLE-only device saves ~30KB RAM)
 * 2. Initialize BT controller with default config
 * 3. Enable BT controller in BLE mode
 * 4. Initialize Bluedroid (ESP-IDF's BT host stack)
 * 5. Enable Bluedroid
 * 6. Register GATT server callback → triggers service creation
 * 7. Register GAP callback → handles advertising events
 * 8. Register GATT application → triggers REG_EVT
 * 9. Set MTU size for larger data transfers
 *
 * AFTER INIT:
 * Device starts advertising as "NCLite-ESP32" with NUS service UUID.
 * Mobile apps can scan, connect, and exchange data.
 *
 * CALLED FROM: app_main() in main.c
 */
esp_err_t ble_spp_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing BLE SPP...");
    ESP_LOGI(TAG, "Device Name: %s", BLE_DEVICE_NAME);

    // Release classic BT memory (we only use BLE)
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGW(TAG, "BT controller mem release failed (may be already released)");
    }

    // Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register callbacks
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GATTS register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GAP register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register GATT application
    ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
    if (ret) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set MTU size
    esp_ble_gatt_set_local_mtu(BLE_MTU_SIZE);

    ESP_LOGI(TAG, "BLE SPP initialized successfully");
    return ESP_OK;
}

/**
 * @brief Deinitialize the BLE SPP module
 *
 * PURPOSE:
 * Cleanly shut down the BLE stack, freeing resources.
 * Used for power management or mode switching.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return ESP_OK always
 *
 * SHUTDOWN SEQUENCE (reverse of init):
 * 1. Disable Bluedroid
 * 2. Deinitialize Bluedroid
 * 3. Disable BT controller
 * 4. Deinitialize BT controller
 * 5. Reset state variables
 */
esp_err_t ble_spp_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing BLE SPP...");

    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    s_ble_state = BLE_STATE_IDLE;
    gl_profile.is_connected = false;
    gl_profile.notify_enabled = false;

    return ESP_OK;
}

/**
 * @brief Send data to connected mobile phone via BLE notification
 *
 * PURPOSE:
 * This is the PRIMARY FUNCTION for sending data to the mobile app.
 * Used to send WM data, MA data, responses, and status updates.
 *
 * INPUT:
 * @param data - Pointer to data buffer (typically JSON string)
 * @param len  - Length of data in bytes
 *
 * OUTPUT:
 * @return ESP_OK on success
 * @return ESP_ERR_INVALID_STATE if not connected or notifications disabled
 * @return ESP_ERR_INVALID_ARG if data is NULL or len <= 0
 *
 * PREREQUISITES:
 * 1. Phone must be connected (is_connected = true)
 * 2. Phone must have enabled notifications on TX characteristic
 *    (wrote 0x0001 to CCCD descriptor)
 *
 * DATA FLOW:
 * ble_spp_send(json_data) → esp_ble_gatts_send_indicate()
 * → BLE stack sends notification → Phone receives data
 *
 * EXAMPLE USAGE:
 * - send_response() calls this to send JSON responses
 * - wm_ble_data_callback() calls this to forward WM data
 * - ma_ble_data_callback() calls this to forward MA data
 */
esp_err_t ble_spp_send(const char *data, int len)
{
    // Check if phone is connected
    if (!gl_profile.is_connected) {
        return ESP_ERR_INVALID_STATE;
    }

    // Check if phone has enabled notifications
    if (!gl_profile.notify_enabled) {
        ESP_LOGD(TAG, "Notifications not enabled, data not sent");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate input parameters
    if (data == NULL || len <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // Send data as BLE notification (false = notify, not indicate)
    esp_err_t ret = esp_ble_gatts_send_indicate(gl_profile.gatts_if,
                                                 gl_profile.conn_id,
                                                 gl_profile.char_tx_handle,
                                                 len, (uint8_t *)data, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send notify: %s", esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief Register a callback for raw BLE data reception
 *
 * PURPOSE:
 * Allow other modules to receive raw BLE data before JSON parsing.
 * The callback is called in addition to normal JSON command processing.
 *
 * INPUT:
 * @param callback - Function pointer: void callback(const char *data, int len)
 *
 * OUTPUT: None
 *
 * NOTE: Currently not used in this project but available for extensibility.
 */
void ble_spp_register_rx_callback(ble_spp_rx_callback_t callback)
{
    s_rx_callback = callback;
}

/**
 * @brief Check if a mobile phone is connected
 *
 * PURPOSE:
 * Query whether BLE connection is active. Used to conditionally
 * send data or display connection status.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return true if phone is connected, false otherwise
 *
 * USED BY: cmd_parser.c for STATUS command, main.c for status display
 */
bool ble_spp_is_connected(void)
{
    return gl_profile.is_connected;
}

/**
 * @brief Check if phone has enabled notifications
 *
 * PURPOSE:
 * Check whether phone has subscribed to TX characteristic notifications.
 * Data can only be sent if notifications are enabled.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return true if notifications enabled, false otherwise
 *
 * NOTE: Phone enables notifications by writing 0x0001 to CCCD descriptor.
 */
bool ble_spp_notify_enabled(void)
{
    return gl_profile.notify_enabled;
}

/**
 * @brief Get current BLE state
 *
 * PURPOSE:
 * Query the BLE state machine for status display.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return Current state: BLE_STATE_IDLE, BLE_STATE_ADVERTISING,
 *         BLE_STATE_CONNECTED, or BLE_STATE_DISCONNECTED
 */
ble_spp_state_t ble_spp_get_state(void)
{
    return s_ble_state;
}

/**
 * @brief Get the BLE device name
 *
 * PURPOSE:
 * Return the configured BLE device name for display purposes.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return Pointer to device name string (e.g., "NCLite-ESP32")
 */
const char *ble_spp_get_device_name(void)
{
    return BLE_DEVICE_NAME;
}

/**
 * @brief Convenience callback for sending data to BLE
 *
 * PURPOSE:
 * Wrapper function that can be used as a callback for other modules.
 * Checks connection state before attempting to send.
 *
 * INPUT:
 * @param data - Data to send
 * @param len  - Length of data
 *
 * OUTPUT: None (silently fails if not connected)
 *
 * NOTE: This is used as a callback function pointer, compatible
 * with output callback signature expected by some modules.
 */
void ble_spp_output_callback(const char *data, unsigned int len)
{
    if (gl_profile.is_connected && gl_profile.notify_enabled) {
        ble_spp_send(data, (int)len);
    }
}
