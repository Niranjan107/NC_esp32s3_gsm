/**
 * @file ota.c
 * @brief OTA (Over-The-Air) firmware update implementation for NCLite ESP32-S3
 *
 * PURPOSE:
 * Implements BLE-based OTA firmware updates using ESP-IDF's OTA APIs.
 * Receives firmware in chunks from mobile app, writes to OTA partition,
 * verifies, and reboots to new firmware.
 *
 * ESP-IDF OTA FLOW:
 * 1. esp_ota_begin() - Erase and prepare OTA partition
 * 2. esp_ota_write() - Write firmware data (can be called multiple times)
 * 3. esp_ota_end() - Validate written data
 * 4. esp_ota_set_boot_partition() - Set next boot partition
 * 5. esp_restart() - Reboot to new firmware
 *
 * ROLLBACK PROTECTION:
 * ESP-IDF supports automatic rollback if new firmware fails to boot.
 * Call ota_mark_valid() after successful startup to confirm firmware.
 *
 * DATA FLOW:
 * +---------------------------------------------------------------------------+
 * |                           OTA Update Flow                                  |
 * |                                                                            |
 * |  Mobile App                    ESP32-S3                                    |
 * |  ----------                    --------                                    |
 * |      |                             |                                       |
 * |      | {"command":"ota_begin","size":524288}#                              |
 * |      |----------------------------->|                                       |
 * |      |                             | esp_ota_begin()                       |
 * |      |<-----------------------------|                                       |
 * |      | {"response":"ota_begin","status":0}                                 |
 * |      |                             |                                       |
 * |      | {"command":"ota_write","seq":0,"data":"<base64>"}#                  |
 * |      |----------------------------->|                                       |
 * |      |                             | decode + esp_ota_write()              |
 * |      |<-----------------------------|                                       |
 * |      | {"response":"ota_write","status":0,"pct":10}                        |
 * |      |                             |                                       |
 * |      |     ... (repeat for all chunks) ...                                 |
 * |      |                             |                                       |
 * |      | {"command":"ota_end"}#                                              |
 * |      |----------------------------->|                                       |
 * |      |                             | esp_ota_end()                         |
 * |      |                             | esp_ota_set_boot_partition()          |
 * |      |<-----------------------------|                                       |
 * |      | {"response":"ota_end","status":0}                                   |
 * |      |                             |                                       |
 * |      |                             | esp_restart()                         |
 * |      |                             |                                       |
 * +---------------------------------------------------------------------------+
 */

#include "ota.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "OTA";

/*******************************************************************************
 * Internal State
 ******************************************************************************/

/**
 * OTA handle from esp_ota_begin()
 */
static esp_ota_handle_t s_ota_handle = 0;

/**
 * Target OTA partition
 */
static const esp_partition_t *s_ota_partition = NULL;

/**
 * Current OTA state
 */
static ota_state_t s_state = OTA_STATE_IDLE;

/**
 * Last error
 */
static ota_error_t s_last_error = OTA_ERR_NONE;

/**
 * Total firmware size
 */
static uint32_t s_total_size = 0;

/**
 * Bytes received so far
 */
static uint32_t s_received_size = 0;

/**
 * Expected next chunk sequence number
 */
static uint32_t s_expected_seq = 0;

/**
 * Module initialized flag
 */
static bool s_initialized = false;

/*******************************************************************************
 * Internal Functions
 ******************************************************************************/

/**
 * @brief Reset OTA state
 *
 * PURPOSE:
 * Reset all OTA state variables to initial values.
 */
static void reset_ota_state(void)
{
    s_ota_handle = 0;
    s_ota_partition = NULL;
    s_state = OTA_STATE_IDLE;
    s_last_error = OTA_ERR_NONE;
    s_total_size = 0;
    s_received_size = 0;
    s_expected_seq = 0;
}

/**
 * @brief Set error state
 *
 * PURPOSE:
 * Set error state and log the error.
 *
 * @param error - Error code
 */
static void set_error(ota_error_t error)
{
    s_last_error = error;
    s_state = OTA_STATE_ERROR;
    ESP_LOGE(TAG, "OTA error: %s", ota_error_to_string(error));
}

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

esp_err_t ota_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing OTA module");

    // Get running partition info
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == NULL) {
        ESP_LOGE(TAG, "Failed to get running partition");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Running partition: %s (addr=0x%lx, size=%lu)",
             running->label, running->address, running->size);

    // Reset state
    reset_ota_state();
    s_initialized = true;

    return ESP_OK;
}

esp_err_t ota_begin(uint32_t firmware_size)
{
    ESP_LOGI(TAG, "OTA begin: firmware size = %lu bytes", firmware_size);

    if (!s_initialized) {
        set_error(OTA_ERR_NOT_INITIALIZED);
        return ESP_ERR_INVALID_STATE;
    }

    if (s_state != OTA_STATE_IDLE && s_state != OTA_STATE_ERROR) {
        ESP_LOGE(TAG, "OTA already in progress");
        set_error(OTA_ERR_ALREADY_STARTED);
        return ESP_ERR_INVALID_STATE;
    }

    // Validate firmware size
    if (firmware_size == 0 || firmware_size > OTA_MAX_FIRMWARE_SIZE) {
        ESP_LOGE(TAG, "Invalid firmware size: %lu (max=%d)",
                 firmware_size, OTA_MAX_FIRMWARE_SIZE);
        set_error(OTA_ERR_SIZE_MISMATCH);
        return ESP_ERR_INVALID_SIZE;
    }

    // Get next OTA partition
    s_ota_partition = esp_ota_get_next_update_partition(NULL);
    if (s_ota_partition == NULL) {
        ESP_LOGE(TAG, "No OTA partition found");
        set_error(OTA_ERR_PARTITION_NOT_FOUND);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "OTA partition: %s (addr=0x%lx, size=%lu)",
             s_ota_partition->label, s_ota_partition->address,
             s_ota_partition->size);

    // Check partition size
    if (firmware_size > s_ota_partition->size) {
        ESP_LOGE(TAG, "Firmware too large for partition");
        set_error(OTA_ERR_PARTITION_TOO_SMALL);
        return ESP_ERR_INVALID_SIZE;
    }

    // Begin OTA (erases partition)
    esp_err_t err = esp_ota_begin(s_ota_partition, firmware_size, &s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        set_error(OTA_ERR_WRITE_FAILED);
        return err;
    }

    // Initialize state
    s_total_size = firmware_size;
    s_received_size = 0;
    s_expected_seq = 0;
    s_last_error = OTA_ERR_NONE;
    s_state = OTA_STATE_READY;

    ESP_LOGI(TAG, "OTA ready to receive firmware");
    return ESP_OK;
}

esp_err_t ota_write_chunk(const uint8_t *data, size_t len, uint32_t seq)
{
    if (!s_initialized) {
        set_error(OTA_ERR_NOT_INITIALIZED);
        return ESP_ERR_INVALID_STATE;
    }

    if (s_state != OTA_STATE_READY && s_state != OTA_STATE_RECEIVING) {
        ESP_LOGE(TAG, "OTA not ready for writing (state=%d)", s_state);
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0) {
        ESP_LOGE(TAG, "Invalid chunk data");
        set_error(OTA_ERR_INVALID_CHUNK);
        return ESP_ERR_INVALID_ARG;
    }

    // Check sequence number (allow idempotent re-sends)
    if (seq == s_expected_seq) {
        // Normal case: this is the next expected chunk
    } else if (seq == s_expected_seq - 1 && s_expected_seq > 0) {
        // Idempotent retry: client is re-sending the previous chunk
        // This happens when our response was lost but we already wrote the data
        ESP_LOGW(TAG, "Idempotent retry: seq %lu already written, returning success", seq);
        return ESP_OK;  // Return success without re-writing
    } else {
        ESP_LOGE(TAG, "Sequence error: expected %lu, got %lu", s_expected_seq, seq);
        set_error(OTA_ERR_SEQUENCE_ERROR);
        return ESP_ERR_INVALID_ARG;
    }

    // Check if we're receiving too much data
    if (s_received_size + len > s_total_size) {
        ESP_LOGE(TAG, "Too much data: %lu + %d > %lu",
                 s_received_size, len, s_total_size);
        set_error(OTA_ERR_SIZE_MISMATCH);
        return ESP_ERR_INVALID_SIZE;
    }

    // Write chunk to OTA partition
    esp_err_t err = esp_ota_write(s_ota_handle, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
        set_error(OTA_ERR_WRITE_FAILED);
        return err;
    }

    // Update state
    s_received_size += len;
    s_expected_seq++;
    s_state = OTA_STATE_RECEIVING;

    // Log progress every 10%
    uint8_t progress = (s_received_size * 100) / s_total_size;
    static uint8_t last_logged_progress = 0;
    if (progress >= last_logged_progress + 10 || progress == 100) {
        ESP_LOGI(TAG, "OTA progress: %d%% (%lu/%lu bytes)",
                 progress, s_received_size, s_total_size);
        last_logged_progress = progress;
    }

    return ESP_OK;
}

esp_err_t ota_end(void)
{
    ESP_LOGI(TAG, "OTA end: received %lu/%lu bytes", s_received_size, s_total_size);

    if (!s_initialized) {
        set_error(OTA_ERR_NOT_INITIALIZED);
        return ESP_ERR_INVALID_STATE;
    }

    if (s_state != OTA_STATE_RECEIVING) {
        ESP_LOGE(TAG, "OTA not in receiving state");
        return ESP_ERR_INVALID_STATE;
    }

    // Check if we received all data
    if (s_received_size != s_total_size) {
        ESP_LOGE(TAG, "Size mismatch: received %lu, expected %lu",
                 s_received_size, s_total_size);
        set_error(OTA_ERR_SIZE_MISMATCH);
        return ESP_ERR_INVALID_SIZE;
    }

    s_state = OTA_STATE_VERIFYING;

    // End OTA (validates firmware)
    esp_err_t err = esp_ota_end(s_ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            set_error(OTA_ERR_VERIFY_FAILED);
        } else {
            set_error(OTA_ERR_WRITE_FAILED);
        }
        return err;
    }

    ESP_LOGI(TAG, "Firmware verified successfully");

    // Set boot partition
    err = esp_ota_set_boot_partition(s_ota_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        set_error(OTA_ERR_SET_BOOT_FAILED);
        return err;
    }

    ESP_LOGI(TAG, "Boot partition set to: %s", s_ota_partition->label);

    s_state = OTA_STATE_COMPLETE;
    ESP_LOGI(TAG, "OTA complete! Ready to reboot.");

    return ESP_OK;
}

esp_err_t ota_abort(void)
{
    ESP_LOGW(TAG, "OTA abort");

    if (s_ota_handle != 0) {
        esp_ota_abort(s_ota_handle);
    }

    reset_ota_state();
    s_last_error = OTA_ERR_ABORTED;

    return ESP_OK;
}

void ota_reboot(void)
{
    ESP_LOGI(TAG, "Rebooting to new firmware...");
    vTaskDelay(pdMS_TO_TICKS(500));  // Allow logs to flush
    esp_restart();
}

esp_err_t ota_get_progress(ota_progress_t *progress)
{
    if (progress == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    progress->state = s_state;
    progress->error = s_last_error;
    progress->total_size = s_total_size;
    progress->received_size = s_received_size;
    progress->expected_seq = s_expected_seq;

    if (s_total_size > 0) {
        progress->progress_percent = (s_received_size * 100) / s_total_size;
    } else {
        progress->progress_percent = 0;
    }

    return ESP_OK;
}

ota_state_t ota_get_state(void)
{
    return s_state;
}

bool ota_is_in_progress(void)
{
    return (s_state == OTA_STATE_READY ||
            s_state == OTA_STATE_RECEIVING ||
            s_state == OTA_STATE_VERIFYING);
}

const char* ota_error_to_string(ota_error_t error)
{
    switch (error) {
        case OTA_ERR_NONE:              return "none";
        case OTA_ERR_NOT_INITIALIZED:   return "not_initialized";
        case OTA_ERR_ALREADY_STARTED:   return "already_started";
        case OTA_ERR_PARTITION_NOT_FOUND: return "partition_not_found";
        case OTA_ERR_PARTITION_TOO_SMALL: return "partition_too_small";
        case OTA_ERR_WRITE_FAILED:      return "write_failed";
        case OTA_ERR_INVALID_CHUNK:     return "invalid_chunk";
        case OTA_ERR_SEQUENCE_ERROR:    return "sequence_error";
        case OTA_ERR_SIZE_MISMATCH:     return "size_mismatch";
        case OTA_ERR_VERIFY_FAILED:     return "verify_failed";
        case OTA_ERR_SET_BOOT_FAILED:   return "set_boot_failed";
        case OTA_ERR_TIMEOUT:           return "timeout";
        case OTA_ERR_ABORTED:           return "aborted";
        default:                        return "unknown";
    }
}

esp_err_t ota_mark_valid(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    esp_err_t err = esp_ota_get_state_partition(running, &ota_state);
    if (err != ESP_OK) {
        // Not an OTA partition or factory partition - OK
        ESP_LOGI(TAG, "Not running from OTA partition (factory boot)");
        return ESP_OK;
    }

    if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_LOGI(TAG, "Marking OTA partition as valid");
        err = esp_ota_mark_app_valid_cancel_rollback();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to mark app valid: %s", esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(TAG, "OTA partition marked as valid - rollback disabled");
    } else {
        ESP_LOGI(TAG, "OTA partition already valid (state=%d)", ota_state);
    }

    return ESP_OK;
}

esp_err_t ota_get_running_partition_info(char *label, uint32_t *address, uint32_t *size)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running == NULL) {
        return ESP_FAIL;
    }

    if (label != NULL) {
        strncpy(label, running->label, 16);
    }
    if (address != NULL) {
        *address = running->address;
    }
    if (size != NULL) {
        *size = running->size;
    }

    return ESP_OK;
}
