/**
 * @file battery.h
 * @brief Battery voltage monitoring header for NCLite ESP32-S3
 *
 * PURPOSE:
 * Defines the API for monitoring battery voltage via ADC on GPIO6.
 * Used by mobile app to display battery status and warn on low battery.
 *
 * HARDWARE:
 * - GPIO6 connected to battery voltage divider
 * - ESP32-S3 ADC1_CHANNEL_5 (GPIO6)
 * - Typical voltage divider: 100K/100K for 2:1 ratio
 *
 * USAGE:
 * 1. battery_init() - Initialize ADC for GPIO6
 * 2. battery_get_voltage_mv() - Read battery voltage in millivolts
 * 3. battery_get_percentage() - Get estimated battery percentage
 * 4. battery_get_json() - Get battery status as JSON for mobile app
 */

#ifndef BATTERY_H
#define BATTERY_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Configuration Constants (from Kconfig menuconfig)
 ******************************************************************************/

/**
 * ADC Pin Configuration
 * Configured via menuconfig: NCLite CLEV4 → Battery Monitor
 * GPIO1-10 maps to ADC1_CH0-9, so GPIO6 = ADC1_CH5
 */
#ifdef CONFIG_NCLE_BATTERY_ADC_GPIO
#define BATTERY_ADC_GPIO        CONFIG_NCLE_BATTERY_ADC_GPIO
#else
#define BATTERY_ADC_GPIO        6   // Default: GPIO6
#endif

// Calculate ADC channel from GPIO (GPIO1=CH0, GPIO2=CH1, ..., GPIO10=CH9)
#define BATTERY_ADC_CHANNEL     (BATTERY_ADC_GPIO - 1)
#define BATTERY_ADC_UNIT        ADC_UNIT_1

/**
 * Voltage Divider Configuration (from menuconfig)
 * Kconfig stores as integer x100 (e.g., 200 = 2.0 ratio)
 * - For 100K/100K: ratio = 2.0 (enter 200)
 * - For 200K/100K: ratio = 3.0 (enter 300)
 */
#ifdef CONFIG_NCLE_BATTERY_DIVIDER_RATIO
#define BATTERY_DIVIDER_RATIO   (CONFIG_NCLE_BATTERY_DIVIDER_RATIO / 100.0f)
#else
#define BATTERY_DIVIDER_RATIO   2.0f    // Default: 2:1 divider
#endif

/**
 * Battery Voltage Thresholds (from menuconfig, in millivolts)
 * Default values for single Li-Ion cell:
 * - Full: 4200mV (4.2V)
 * - Empty: 3000mV (3.0V cutoff)
 */
#ifdef CONFIG_NCLE_BATTERY_FULL_MV
#define BATTERY_VOLTAGE_FULL_MV     CONFIG_NCLE_BATTERY_FULL_MV
#else
#define BATTERY_VOLTAGE_FULL_MV     4200
#endif

#ifdef CONFIG_NCLE_BATTERY_EMPTY_MV
#define BATTERY_VOLTAGE_EMPTY_MV    CONFIG_NCLE_BATTERY_EMPTY_MV
#else
#define BATTERY_VOLTAGE_EMPTY_MV    3000
#endif

// Low battery warning threshold (20% above empty)
#define BATTERY_VOLTAGE_LOW_MV      (BATTERY_VOLTAGE_EMPTY_MV + \
                                     ((BATTERY_VOLTAGE_FULL_MV - BATTERY_VOLTAGE_EMPTY_MV) / 5))

/**
 * ADC Sampling Configuration
 */
#define BATTERY_ADC_SAMPLES         64      // Number of samples to average
#define BATTERY_ADC_ATTEN           ADC_ATTEN_DB_12  // 0-3.3V range (ESP32-S3)

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/**
 * Battery status enumeration
 */
typedef enum {
    BATTERY_STATUS_UNKNOWN = 0,
    BATTERY_STATUS_FULL,        // > 80%
    BATTERY_STATUS_GOOD,        // 50-80%
    BATTERY_STATUS_LOW,         // 20-50%
    BATTERY_STATUS_CRITICAL,    // < 20%
    BATTERY_STATUS_CHARGING     // If charging detection is available
} battery_status_t;

/**
 * Battery information structure
 */
typedef struct {
    uint32_t voltage_mv;        // Battery voltage in millivolts
    uint8_t percentage;         // Estimated percentage (0-100)
    battery_status_t status;    // Current status
    uint32_t raw_adc;           // Raw ADC value (for debugging)
} battery_info_t;

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

/**
 * @brief Initialize battery voltage monitoring
 *
 * PURPOSE:
 * Configure ADC1_CHANNEL_5 (GPIO6) for battery voltage measurement.
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t battery_init(void);

/**
 * @brief Deinitialize battery monitoring
 *
 * PURPOSE:
 * Release ADC resources.
 *
 * @return ESP_OK on success
 */
esp_err_t battery_deinit(void);

/**
 * @brief Get battery voltage in millivolts
 *
 * PURPOSE:
 * Read ADC and calculate actual battery voltage accounting for divider.
 *
 * @return Battery voltage in millivolts
 */
uint32_t battery_get_voltage_mv(void);

/**
 * @brief Get battery percentage
 *
 * PURPOSE:
 * Calculate estimated battery percentage based on voltage.
 *
 * @return Battery percentage (0-100)
 */
uint8_t battery_get_percentage(void);

/**
 * @brief Get battery status
 *
 * PURPOSE:
 * Get current battery status (full, good, low, critical).
 *
 * @return Battery status enumeration value
 */
battery_status_t battery_get_status(void);

/**
 * @brief Get complete battery information
 *
 * PURPOSE:
 * Get all battery metrics in one call.
 *
 * @param info - Pointer to battery_info_t structure to fill
 * @return ESP_OK on success
 */
esp_err_t battery_get_info(battery_info_t *info);

/**
 * @brief Get battery status as JSON string
 *
 * PURPOSE:
 * Format battery information as JSON for mobile app.
 *
 * @param buffer - Output buffer for JSON string
 * @param buffer_size - Size of output buffer
 * @return Number of characters written
 */
int battery_get_json(char *buffer, size_t buffer_size);

/**
 * @brief Check if battery is low
 *
 * PURPOSE:
 * Quick check for low battery condition.
 *
 * @return true if battery voltage is below BATTERY_VOLTAGE_LOW_MV
 */
bool battery_is_low(void);

/**
 * @brief Set voltage divider ratio
 *
 * PURPOSE:
 * Override the default voltage divider ratio at runtime.
 * Useful for calibration or different hardware configurations.
 *
 * @param ratio - New divider ratio (e.g., 2.0 for 100K/100K divider)
 */
void battery_set_divider_ratio(float ratio);

/**
 * @brief Get raw ADC value (for debugging)
 *
 * PURPOSE:
 * Read raw ADC value without voltage conversion.
 *
 * @return Raw ADC value (0-4095 for 12-bit ADC)
 */
uint32_t battery_get_raw_adc(void);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */
