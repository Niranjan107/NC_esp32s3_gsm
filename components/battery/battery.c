/**
 * @file battery.c
 * @brief Battery voltage monitoring implementation for NCLite ESP32-S3
 *
 * PURPOSE:
 * This module monitors battery voltage using the ESP32-S3's ADC on GPIO6.
 * It provides voltage readings, percentage estimates, and JSON output for
 * the mobile app to display battery status.
 *
 * WHY THIS MODULE EXISTS:
 * Dairy collection centers often use battery-powered connectors.
 * Operators need to know battery status to avoid unexpected shutdowns
 * during milk collection operations. This module:
 * 1. Reads battery voltage via ADC
 * 2. Accounts for voltage divider ratio
 * 3. Calculates percentage estimate
 * 4. Provides JSON data for mobile app
 *
 * HARDWARE SETUP:
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                        Battery Voltage Divider                          │
 * │                                                                          │
 * │    Battery +  ────┬──── R1 (100K) ────┬──── GPIO6 (ADC1_CH5)            │
 * │                   │                    │                                 │
 * │                   │              R2 (100K)                               │
 * │                   │                    │                                 │
 * │    Battery -  ────┴────────────────────┴──── GND                        │
 * │                                                                          │
 * │    Vout = Vbat × R2/(R1+R2) = Vbat × 0.5                                │
 * │    Example: 4.2V battery → 2.1V at GPIO6                                │
 * └─────────────────────────────────────────────────────────────────────────┘
 *
 * ESP32-S3 ADC CHARACTERISTICS:
 * - ADC1_CHANNEL_5 = GPIO6
 * - 12-bit resolution (0-4095)
 * - With ADC_ATTEN_DB_12: 0-3.3V range (approximately)
 * - Non-linear at extremes, best accuracy in 150-2450mV range
 *
 * CALIBRATION:
 * ESP-IDF provides ADC calibration based on eFuse values burned at factory.
 * This module uses esp_adc_cal for more accurate voltage readings.
 *
 * DATA FLOW:
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                         battery.c (THIS FILE)                           │
 * │                                                                          │
 * │  1. battery_init() - Configure ADC, load calibration                    │
 * │  2. battery_get_voltage_mv() - Read ADC, apply calibration & divider    │
 * │  3. battery_get_percentage() - Map voltage to 0-100%                    │
 * │  4. battery_get_json() - Format for mobile app                          │
 * └─────────────────────────────────────────────────────────────────────────┘
 *                                    │
 *                              JSON output
 *                                    │
 *                                    ▼
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  {"device":"battery","voltage_mv":3850,"percentage":72,"status":"good"} │
 * │                                                                          │
 * │  → cmd_parser (self_diagnosis includes battery status)                  │
 * │  → BLE (s_output_callback → ble_spp_send)                               │
 * └─────────────────────────────────────────────────────────────────────────┘
 */

#include "battery.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "BATTERY";

// ============================================================================
// Internal State
//
// PURPOSE: Module-level variables for battery monitoring
// ============================================================================

/**
 * ADC Handle
 * ESP-IDF 5.x uses oneshot ADC driver with handle-based API.
 */
static adc_oneshot_unit_handle_t s_adc_handle = NULL;

/**
 * ADC Calibration Handle
 * Provides voltage-accurate readings based on factory eFuse calibration.
 */
static adc_cali_handle_t s_cali_handle = NULL;

/**
 * Calibration Available Flag
 * True if calibration scheme was successfully initialized.
 * If false, we fall back to raw ADC values with linear approximation.
 */
static bool s_cali_available = false;

/**
 * Initialization Flag
 */
static bool s_initialized = false;

/**
 * Runtime Voltage Divider Ratio
 * Can be adjusted via battery_set_divider_ratio() for calibration.
 */
static float s_divider_ratio = BATTERY_DIVIDER_RATIO;

// ============================================================================
// Internal Functions
//
// PURPOSE: Helper functions for ADC operations
// ============================================================================

/**
 * @brief Initialize ADC calibration
 *
 * PURPOSE:
 * Set up ADC calibration using ESP-IDF's calibration scheme.
 * ESP32-S3 supports curve fitting calibration for better accuracy.
 *
 * INPUT:
 * @param unit - ADC unit (ADC_UNIT_1 for GPIO6)
 * @param atten - Attenuation setting
 * @param out_handle - Output calibration handle
 *
 * OUTPUT:
 * @return true if calibration initialized successfully
 *
 * CALIBRATION SCHEMES:
 * ESP32-S3 supports:
 * - Line fitting: Simple linear correction
 * - Curve fitting: Polynomial correction (more accurate)
 *
 * Factory eFuse values provide per-chip calibration data.
 */
static bool init_adc_calibration(adc_unit_t unit, adc_atten_t atten,
                                  adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    // Try curve fitting first (most accurate)
    if (!calibrated) {
        ESP_LOGI(TAG, "Attempting curve fitting calibration");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
            ESP_LOGI(TAG, "Curve fitting calibration enabled");
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    // Fall back to line fitting
    if (!calibrated) {
        ESP_LOGI(TAG, "Attempting line fitting calibration");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
            ESP_LOGI(TAG, "Line fitting calibration enabled");
        }
    }
#endif

    *out_handle = handle;
    if (!calibrated) {
        ESP_LOGW(TAG, "No calibration scheme available, using raw values");
    }

    return calibrated;
}

/**
 * @brief Deinitialize ADC calibration
 *
 * PURPOSE:
 * Free calibration resources when shutting down.
 *
 * INPUT:
 * @param handle - Calibration handle to free
 */
static void deinit_adc_calibration(adc_cali_handle_t handle)
{
    if (handle == NULL) return;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_delete_scheme_curve_fitting(handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_delete_scheme_line_fitting(handle);
#endif
}

/**
 * @brief Read ADC with averaging
 *
 * PURPOSE:
 * Read multiple ADC samples and return the average.
 * Averaging reduces noise for more stable readings.
 *
 * INPUT:
 * @param num_samples - Number of samples to average
 *
 * OUTPUT:
 * @return Average raw ADC value
 *
 * WHY AVERAGING?
 * ADC readings have inherent noise. Taking multiple samples and
 * averaging them reduces random noise by sqrt(N) factor.
 * 64 samples → 8x noise reduction.
 */
static uint32_t read_adc_averaged(int num_samples)
{
    if (!s_initialized || s_adc_handle == NULL) {
        return 0;
    }

    uint32_t sum = 0;
    int valid_samples = 0;
    int raw_value;

    for (int i = 0; i < num_samples; i++) {
        esp_err_t ret = adc_oneshot_read(s_adc_handle, BATTERY_ADC_CHANNEL, &raw_value);
        if (ret == ESP_OK) {
            sum += raw_value;
            valid_samples++;
        }
    }

    if (valid_samples == 0) {
        ESP_LOGE(TAG, "No valid ADC samples");
        return 0;
    }

    return sum / valid_samples;
}

/**
 * @brief Convert raw ADC value to voltage in millivolts
 *
 * PURPOSE:
 * Convert raw ADC reading to calibrated voltage, then apply
 * voltage divider ratio to get actual battery voltage.
 *
 * INPUT:
 * @param raw_adc - Raw ADC value (0-4095)
 *
 * OUTPUT:
 * @return Battery voltage in millivolts
 *
 * CALCULATION:
 * 1. If calibration available: Use calibrated voltage
 * 2. Else: Linear approximation (raw * 3300 / 4095 for 12-bit)
 * 3. Multiply by divider ratio to get battery voltage
 */
static uint32_t raw_to_battery_voltage_mv(uint32_t raw_adc)
{
    int voltage_mv = 0;

    if (s_cali_available && s_cali_handle != NULL) {
        // Use calibrated conversion
        esp_err_t ret = adc_cali_raw_to_voltage(s_cali_handle, (int)raw_adc, &voltage_mv);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Calibration conversion failed, using linear");
            voltage_mv = (raw_adc * 3300) / 4095;
        }
    } else {
        // Linear approximation for 12-bit ADC with 3.3V reference
        voltage_mv = (raw_adc * 3300) / 4095;
    }

    // Apply voltage divider ratio to get actual battery voltage
    uint32_t battery_mv = (uint32_t)(voltage_mv * s_divider_ratio);

    return battery_mv;
}

/**
 * @brief Calculate battery percentage from voltage
 *
 * PURPOSE:
 * Map battery voltage to 0-100% estimate.
 *
 * INPUT:
 * @param voltage_mv - Battery voltage in millivolts
 *
 * OUTPUT:
 * @return Percentage (0-100)
 *
 * NOTE:
 * This is a simple linear mapping. For more accurate percentage,
 * a lookup table based on battery discharge curve would be better.
 * Li-Ion discharge is non-linear (flat in middle, steep at ends).
 */
static uint8_t voltage_to_percentage(uint32_t voltage_mv)
{
    if (voltage_mv >= BATTERY_VOLTAGE_FULL_MV) {
        return 100;
    }
    if (voltage_mv <= BATTERY_VOLTAGE_EMPTY_MV) {
        return 0;
    }

    // Linear interpolation between empty and full
    uint32_t range = BATTERY_VOLTAGE_FULL_MV - BATTERY_VOLTAGE_EMPTY_MV;
    uint32_t offset = voltage_mv - BATTERY_VOLTAGE_EMPTY_MV;

    return (uint8_t)((offset * 100) / range);
}

/**
 * @brief Determine battery status from percentage
 *
 * PURPOSE:
 * Map percentage to status category.
 *
 * INPUT:
 * @param percentage - Battery percentage (0-100)
 *
 * OUTPUT:
 * @return battery_status_t enumeration value
 */
static battery_status_t percentage_to_status(uint8_t percentage)
{
    if (percentage > 80) return BATTERY_STATUS_FULL;
    if (percentage > 50) return BATTERY_STATUS_GOOD;
    if (percentage > 20) return BATTERY_STATUS_LOW;
    return BATTERY_STATUS_CRITICAL;
}

/**
 * @brief Get status string from status enum
 *
 * PURPOSE:
 * Convert status enum to human-readable string for JSON output.
 *
 * INPUT:
 * @param status - Battery status enumeration
 *
 * OUTPUT:
 * @return Status string
 */
static const char* status_to_string(battery_status_t status)
{
    switch (status) {
        case BATTERY_STATUS_FULL:     return "full";
        case BATTERY_STATUS_GOOD:     return "good";
        case BATTERY_STATUS_LOW:      return "low";
        case BATTERY_STATUS_CRITICAL: return "critical";
        case BATTERY_STATUS_CHARGING: return "charging";
        default:                      return "unknown";
    }
}

// ============================================================================
// Public API Implementation
//
// PURPOSE: External functions called by main.c and cmd_parser.c
// ============================================================================

/**
 * @brief Initialize battery voltage monitoring
 *
 * PURPOSE:
 * Configure ADC1_CHANNEL_5 (GPIO6) for battery voltage measurement.
 * Sets up oneshot ADC driver and calibration.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return ESP_OK on success, error code on failure
 *
 * INITIALIZATION:
 * 1. Create ADC oneshot unit for ADC1
 * 2. Configure channel (GPIO6) with attenuation
 * 3. Initialize calibration scheme
 *
 * CALLED FROM: app_main() in main.c
 */
esp_err_t battery_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing battery monitor on GPIO%d", BATTERY_ADC_GPIO);

    // Create ADC oneshot unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = BATTERY_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &s_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure ADC channel
    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = BATTERY_ADC_ATTEN,
    };
    ret = adc_oneshot_config_channel(s_adc_handle, BATTERY_ADC_CHANNEL, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    // Initialize calibration
    s_cali_available = init_adc_calibration(BATTERY_ADC_UNIT, BATTERY_ADC_ATTEN,
                                             &s_cali_handle);

    s_initialized = true;
    ESP_LOGI(TAG, "Battery monitor initialized (divider ratio: %.2f)", s_divider_ratio);

    // Log initial reading
    uint32_t voltage = battery_get_voltage_mv();
    uint8_t percentage = battery_get_percentage();
    ESP_LOGI(TAG, "Initial battery: %lumV (%d%%)", voltage, percentage);

    return ESP_OK;
}

/**
 * @brief Deinitialize battery monitoring
 *
 * PURPOSE:
 * Release ADC resources.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return ESP_OK on success
 */
esp_err_t battery_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing battery monitor");

    // Free calibration
    if (s_cali_handle != NULL) {
        deinit_adc_calibration(s_cali_handle);
        s_cali_handle = NULL;
    }

    // Delete ADC unit
    if (s_adc_handle != NULL) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
    }

    s_cali_available = false;
    s_initialized = false;

    return ESP_OK;
}

/**
 * @brief Get battery voltage in millivolts
 *
 * PURPOSE:
 * Read ADC and calculate actual battery voltage accounting for divider.
 * Uses averaging and calibration for accurate readings.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return Battery voltage in millivolts
 *
 * EXAMPLE:
 * If divider ratio is 2.0 and ADC reads 2000mV:
 * Battery voltage = 2000 * 2.0 = 4000mV (4.0V)
 */
uint32_t battery_get_voltage_mv(void)
{
    if (!s_initialized) {
        ESP_LOGW(TAG, "Not initialized");
        return 0;
    }

    // Read averaged raw ADC value
    uint32_t raw_adc = read_adc_averaged(BATTERY_ADC_SAMPLES);

    // Convert to battery voltage
    uint32_t voltage_mv = raw_to_battery_voltage_mv(raw_adc);

    ESP_LOGD(TAG, "ADC raw=%lu, voltage=%lumV", raw_adc, voltage_mv);

    return voltage_mv;
}

/**
 * @brief Get battery percentage
 *
 * PURPOSE:
 * Calculate estimated battery percentage based on voltage.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return Battery percentage (0-100)
 *
 * NOTE:
 * Uses linear interpolation. For better accuracy with specific
 * battery chemistry, implement a voltage-to-SoC lookup table.
 */
uint8_t battery_get_percentage(void)
{
    uint32_t voltage_mv = battery_get_voltage_mv();
    return voltage_to_percentage(voltage_mv);
}

/**
 * @brief Get battery status
 *
 * PURPOSE:
 * Get current battery status (full, good, low, critical).
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return Battery status enumeration value
 */
battery_status_t battery_get_status(void)
{
    uint8_t percentage = battery_get_percentage();
    return percentage_to_status(percentage);
}

/**
 * @brief Get complete battery information
 *
 * PURPOSE:
 * Get all battery metrics in one call. More efficient than
 * calling individual functions when multiple values needed.
 *
 * INPUT:
 * @param info - Pointer to battery_info_t structure to fill
 *
 * OUTPUT:
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if info is NULL
 *
 * FILLED FIELDS:
 * - voltage_mv: Battery voltage in millivolts
 * - percentage: Estimated percentage (0-100)
 * - status: FULL, GOOD, LOW, or CRITICAL
 * - raw_adc: Raw ADC value for debugging
 */
esp_err_t battery_get_info(battery_info_t *info)
{
    if (info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        ESP_LOGW(TAG, "Not initialized");
        memset(info, 0, sizeof(battery_info_t));
        info->status = BATTERY_STATUS_UNKNOWN;
        return ESP_ERR_INVALID_STATE;
    }

    // Read raw ADC once
    info->raw_adc = read_adc_averaged(BATTERY_ADC_SAMPLES);

    // Calculate voltage
    info->voltage_mv = raw_to_battery_voltage_mv(info->raw_adc);

    // Calculate percentage
    info->percentage = voltage_to_percentage(info->voltage_mv);

    // Determine status
    info->status = percentage_to_status(info->percentage);

    return ESP_OK;
}

/**
 * @brief Get battery status as JSON string
 *
 * PURPOSE:
 * Format battery information as JSON for mobile app.
 * Used by cmd_parser for battery status responses.
 *
 * INPUT:
 * @param buffer - Output buffer for JSON string
 * @param buffer_size - Size of output buffer
 *
 * OUTPUT:
 * @return Number of characters written
 *
 * JSON FORMAT:
 * {"device":"battery","voltage_mv":3850,"percentage":72,"status":"good"}
 */
int battery_get_json(char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return 0;
    }

    battery_info_t info;
    battery_get_info(&info);

    return snprintf(buffer, buffer_size,
        "{\"device\":\"battery\",\"voltage_mv\":%lu,\"percentage\":%d,\"status\":\"%s\"}",
        info.voltage_mv,
        info.percentage,
        status_to_string(info.status)
    );
}

/**
 * @brief Check if battery is low
 *
 * PURPOSE:
 * Quick check for low battery condition.
 * Used to trigger warnings or low-power mode.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return true if battery voltage is below BATTERY_VOLTAGE_LOW_MV
 */
bool battery_is_low(void)
{
    uint32_t voltage_mv = battery_get_voltage_mv();
    return voltage_mv < BATTERY_VOLTAGE_LOW_MV;
}

/**
 * @brief Set voltage divider ratio
 *
 * PURPOSE:
 * Override the default voltage divider ratio at runtime.
 * Useful for calibration or different hardware configurations.
 *
 * INPUT:
 * @param ratio - New divider ratio (e.g., 2.0 for 100K/100K divider)
 *
 * OUTPUT: None
 *
 * EXAMPLE:
 * If your actual divider is slightly off, you can calibrate:
 * 1. Measure actual battery voltage with multimeter
 * 2. Call battery_get_voltage_mv() to get reported voltage
 * 3. Calculate: actual_ratio = actual_voltage / reported_voltage * current_ratio
 * 4. Call battery_set_divider_ratio(actual_ratio)
 */
void battery_set_divider_ratio(float ratio)
{
    if (ratio > 0.0f && ratio < 100.0f) {
        s_divider_ratio = ratio;
        ESP_LOGI(TAG, "Divider ratio set to %.3f", ratio);
    } else {
        ESP_LOGW(TAG, "Invalid divider ratio: %.3f (must be 0-100)", ratio);
    }
}

/**
 * @brief Get raw ADC value (for debugging)
 *
 * PURPOSE:
 * Read raw ADC value without voltage conversion.
 * Useful for debugging and calibration.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return Raw ADC value (0-4095 for 12-bit ADC)
 */
uint32_t battery_get_raw_adc(void)
{
    return read_adc_averaged(BATTERY_ADC_SAMPLES);
}
