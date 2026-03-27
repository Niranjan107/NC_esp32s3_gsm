/**
 * @file ota.h
 * @brief OTA (Over-The-Air) firmware update header for NCLite ESP32-S3
 *
 * PURPOSE:
 * Defines the API for BLE-based OTA firmware updates. Allows the mobile app
 * to update device firmware without USB connection.
 *
 * OTA FLOW:
 * 1. Mobile app sends "ota_begin" with firmware size
 * 2. Device prepares OTA partition, responds with ready status
 * 3. Mobile app sends firmware chunks via "ota_write" (Base64 encoded)
 * 4. Device decodes and writes each chunk to OTA partition
 * 5. Mobile app sends "ota_end" when complete
 * 6. Device verifies firmware, sets boot partition, reboots
 *
 * PARTITION LAYOUT (with OTA):
 * +-----------+----------+------+
 * | Name      | Offset   | Size |
 * +-----------+----------+------+
 * | nvs       | 0x9000   | 24K  |
 * | otadata   | 0xd000   | 8K   |
 * | phy_init  | 0xf000   | 4K   |
 * | ota_0     | 0x10000  | 1.5M |
 * | ota_1     | 0x190000 | 1.5M |
 * | nvs_key   | 0x310000 | 4K   |
 * +-----------+----------+------+
 *
 * SECURITY:
 * - Firmware is validated before boot switch
 * - Automatic rollback on boot failure
 * - CRC/SHA256 verification supported
 */

#ifndef OTA_H
#define OTA_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration Constants
 ******************************************************************************/

/**
 * OTA chunk size for BLE transfer
 * Base64 encoded chunk should fit in BLE MTU (~500 bytes)
 * 400 bytes Base64 = ~300 bytes binary
 */
#define OTA_CHUNK_SIZE          300

/**
 * Maximum firmware size (must fit in OTA partition)
 * Default: 1.5MB per partition
 */
#define OTA_MAX_FIRMWARE_SIZE   (1536 * 1024)

/**
 * OTA timeout for receiving chunks (ms)
 * If no chunk received within timeout, OTA is aborted
 */
#define OTA_RECEIVE_TIMEOUT_MS  30000

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/**
 * OTA state enumeration
 */
typedef enum {
    OTA_STATE_IDLE = 0,         // Not in OTA mode
    OTA_STATE_READY,            // OTA initialized, waiting for chunks
    OTA_STATE_RECEIVING,        // Receiving firmware chunks
    OTA_STATE_VERIFYING,        // Verifying firmware
    OTA_STATE_COMPLETE,         // OTA complete, ready to reboot
    OTA_STATE_ERROR             // Error occurred
} ota_state_t;

/**
 * OTA error codes
 */
typedef enum {
    OTA_ERR_NONE = 0,
    OTA_ERR_NOT_INITIALIZED,
    OTA_ERR_ALREADY_STARTED,
    OTA_ERR_PARTITION_NOT_FOUND,
    OTA_ERR_PARTITION_TOO_SMALL,
    OTA_ERR_WRITE_FAILED,
    OTA_ERR_INVALID_CHUNK,
    OTA_ERR_SEQUENCE_ERROR,
    OTA_ERR_SIZE_MISMATCH,
    OTA_ERR_VERIFY_FAILED,
    OTA_ERR_SET_BOOT_FAILED,
    OTA_ERR_TIMEOUT,
    OTA_ERR_ABORTED
} ota_error_t;

/**
 * OTA progress information
 */
typedef struct {
    ota_state_t state;          // Current OTA state
    ota_error_t error;          // Last error (if any)
    uint32_t total_size;        // Total firmware size
    uint32_t received_size;     // Bytes received so far
    uint32_t expected_seq;      // Expected next chunk sequence
    uint8_t progress_percent;   // Progress percentage (0-100)
} ota_progress_t;

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

/**
 * @brief Initialize OTA module
 *
 * PURPOSE:
 * Initialize OTA subsystem. Must be called before any OTA operations.
 *
 * @return ESP_OK on success
 */
esp_err_t ota_init(void);

/**
 * @brief Begin OTA update
 *
 * PURPOSE:
 * Start a new OTA update session. Prepares the OTA partition for writing.
 *
 * @param firmware_size - Total size of firmware to be received
 * @return ESP_OK on success, error code on failure
 *
 * CALLED FROM: cmd_parser when "ota_begin" command received
 */
esp_err_t ota_begin(uint32_t firmware_size);

/**
 * @brief Write firmware chunk
 *
 * PURPOSE:
 * Write a chunk of firmware data to the OTA partition.
 * Chunks must be received in sequence.
 *
 * @param data - Binary firmware data (already decoded from Base64)
 * @param len - Length of data
 * @param seq - Chunk sequence number (0-based)
 * @return ESP_OK on success, error code on failure
 *
 * CALLED FROM: cmd_parser when "ota_write" command received
 */
esp_err_t ota_write_chunk(const uint8_t *data, size_t len, uint32_t seq);

/**
 * @brief End OTA update
 *
 * PURPOSE:
 * Finalize OTA update. Verifies firmware and prepares for reboot.
 *
 * @return ESP_OK on success, error code on failure
 *
 * CALLED FROM: cmd_parser when "ota_end" command received
 */
esp_err_t ota_end(void);

/**
 * @brief Abort OTA update
 *
 * PURPOSE:
 * Cancel ongoing OTA update and clean up.
 *
 * @return ESP_OK on success
 */
esp_err_t ota_abort(void);

/**
 * @brief Reboot to new firmware
 *
 * PURPOSE:
 * Reboot the device to run the newly flashed firmware.
 * Call this after ota_end() returns success.
 *
 * NOTE: This function does not return!
 */
void ota_reboot(void);

/**
 * @brief Get OTA progress
 *
 * PURPOSE:
 * Get current OTA progress and state information.
 *
 * @param progress - Pointer to ota_progress_t structure to fill
 * @return ESP_OK on success
 */
esp_err_t ota_get_progress(ota_progress_t *progress);

/**
 * @brief Get OTA state
 *
 * PURPOSE:
 * Get current OTA state.
 *
 * @return Current ota_state_t value
 */
ota_state_t ota_get_state(void);

/**
 * @brief Check if OTA is in progress
 *
 * PURPOSE:
 * Quick check if an OTA update is currently active.
 *
 * @return true if OTA is in progress
 */
bool ota_is_in_progress(void);

/**
 * @brief Get error string
 *
 * PURPOSE:
 * Convert OTA error code to human-readable string.
 *
 * @param error - OTA error code
 * @return Error string
 */
const char* ota_error_to_string(ota_error_t error);

/**
 * @brief Mark current firmware as valid
 *
 * PURPOSE:
 * Mark the running firmware as valid after successful boot.
 * Prevents automatic rollback on next reboot.
 * Should be called from app_main() after successful initialization.
 *
 * @return ESP_OK on success
 */
esp_err_t ota_mark_valid(void);

/**
 * @brief Get running partition info
 *
 * PURPOSE:
 * Get information about the currently running partition.
 *
 * @param label - Buffer for partition label (at least 16 bytes)
 * @param address - Pointer to store partition address (can be NULL)
 * @param size - Pointer to store partition size (can be NULL)
 * @return ESP_OK on success
 */
esp_err_t ota_get_running_partition_info(char *label, uint32_t *address, uint32_t *size);

#ifdef __cplusplus
}
#endif

#endif /* OTA_H */
