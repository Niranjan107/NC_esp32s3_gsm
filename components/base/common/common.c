/**
 * @file common.c
 * @brief Common utilities implementation for NCLite ESP32-S3
 *
 * PURPOSE:
 * Provides shared utility functions and global resources used across
 * all modules in the NCLite connector firmware.
 *
 * KEY FUNCTIONS:
 * - get_unique_id(): Returns device's unique hardware ID (from BT MAC)
 * - get_device_type_string(): Convert device type enum to string
 *
 * GLOBAL RESOURCES:
 * - cmd_queue: FreeRTOS queue for command messages (if used)
 * - response_queue: FreeRTOS queue for responses (if used)
 *
 * UNIQUE ID:
 * The device's unique ID is derived from the ESP32's Bluetooth MAC address
 * stored in eFuse (factory-programmed, cannot be changed).
 * Format: 48-bit MAC converted to 64-bit integer
 * Used by mobile app to identify and track individual devices.
 */

#include "common.h"
#include <stdio.h>          /* snprintf in timing_format */
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_timer.h"      /* esp_timer_get_time for the cycle timing */

static const char *TAG = "COMMON";

// Global queues (defined in main.c)
QueueHandle_t cmd_queue = NULL;
QueueHandle_t response_queue = NULL;

/**
 * @brief Initialize common module
 */
void common_init(void)
{
    ESP_LOGI(TAG, "Common module initialized");
}

/**
 * @brief Get string representation of device type
 */
const char* get_device_type_string(device_type_t type)
{
    switch (type) {
        case DEVICE_TYPE_MA:
            return "ma";
        case DEVICE_TYPE_WM:
            return "wm";
        case DEVICE_TYPE_PRINTER:
            return "printer";
        default:
            return "unknown";
    }
}

/**
 * @brief Get unique device ID (based on Bluetooth MAC address)
 *
 * PURPOSE:
 * Return a unique hardware identifier for this device.
 * Used by mobile app to identify and track individual connectors.
 *
 * INPUT: None
 *
 * OUTPUT:
 * @return 64-bit unique ID (48-bit MAC in lower 6 bytes)
 *
 * ID SOURCE:
 * Uses ESP32's Bluetooth MAC address from eFuse.
 * This is factory-programmed and unique per chip.
 *
 * EXAMPLE:
 * MAC: AA:BB:CC:DD:EE:FF → ID: 0x0000AABBCCDDEEFF
 * Displayed as: "AABBCCDDEEFF" (12 hex digits)
 */
uint64_t get_unique_id(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);

    uint64_t id = 0;
    for (int i = 0; i < 6; i++) {
        id = (id << 8) | mac[i];
    }
    return id;
}

// ============================================================================
// Cycle Timing Instrumentation
//
// PURPOSE:
// Answer "how many seconds does one collection take, and where do they go?"
// without an RTC. esp_timer_get_time() counts microseconds since boot, which
// is all we need - every reported value is a difference, never a wall clock.
//
// The cycle starts when the MA packet is decoded and ends when the last
// receipt byte has clocked out to the printer:
//
//   T0 first MA byte  ──┐
//   T1 MA complete      │
//   T2 sent to app      │  differences between adjacent marks
//   T3 print data back  │  show which stage is slow
//   T4 printing starts  │
//   T5 printing done  ──┘  total task time = T5 - T0
// ============================================================================

/** Uptime (us) at which the current cycle started - the T=0 reference */
static int64_t s_epoch_us = 0;

/** Absolute timestamp (us) of each marker, -1 when not yet stamped */
static int64_t s_marks[TIMING_MARK_MAX] = { -1, -1, -1, -1, -1, -1, -1 };

/**
 * True while the once-per-power-cycle receipt timing line is still owed.
 *
 * Armed at boot, cleared by the printer after the line goes out - so each
 * power cycle produces exactly one timing line on paper, and every receipt
 * after that is clean. Re-arm at any time with
 *   {"command":"set_timing_debug","enable":1}#
 * The console log and the app push are unaffected by this and run every cycle.
 */
static bool s_receipt_print = true;

void timing_reset(void)
{
    s_epoch_us = esp_timer_get_time();
    for (int i = 0; i < TIMING_MARK_MAX; i++) {
        s_marks[i] = -1;
    }
}

void timing_mark(timing_mark_t mark)
{
    if (mark >= TIMING_MARK_MAX) {
        return;
    }

    // T0 - the analyser starting to talk begins a new cycle. Everything else
    // is measured from that instant, so the framing wait is visible too.
    if (mark == TIMING_T0_MA_FIRST) {
        timing_reset();
    } else if (s_epoch_us == 0) {
        // Cycle started part-way through (e.g. a reprint with no MA reading).
        // Anchor T=0 here so the numbers stay meaningful instead of huge.
        s_epoch_us = esp_timer_get_time();
    }

    s_marks[mark] = esp_timer_get_time();
}

void timing_mark_once(timing_mark_t mark)
{
    if (mark >= TIMING_MARK_MAX) {
        return;
    }
    if (s_marks[mark] >= 0) {
        return;  // already stamped this cycle - keep the first value
    }
    timing_mark(mark);
}

int32_t timing_get_ms(timing_mark_t mark)
{
    if (mark >= TIMING_MARK_MAX || s_marks[mark] < 0) {
        return -1;  // never stamped this cycle
    }
    return (int32_t)((s_marks[mark] - s_epoch_us) / 1000);
}

int32_t timing_get_epoch_ms(void)
{
    return (int32_t)(s_epoch_us / 1000);
}

int timing_format(char *buf, size_t size)
{
    if (buf == NULL || size == 0) {
        return 0;
    }

    int32_t t4 = timing_get_ms(TIMING_T4_PRINT_FIRST);
    int32_t t5 = timing_get_ms(TIMING_T5_PRINT_DONE);

    // All values are ms from T0. PRN is the print duration (T5-T4) and
    // TOT is the whole task (T5, since T0 is zero by definition).
    return snprintf(buf, size,
                    "T0=%ld T1=%ld T2=%ld T3=%ld T4=%ld T5=%ld PRN=%ld TOT=%ld",
                    (long)timing_get_ms(TIMING_T0_MA_FIRST),
                    (long)timing_get_ms(TIMING_T1_MA_DONE),
                    (long)timing_get_ms(TIMING_T2_APP_SENT),
                    (long)timing_get_ms(TIMING_T3_CMD_RX),
                    (long)t4,
                    (long)t5,
                    (long)((t4 >= 0 && t5 >= 0) ? (t5 - t4) : -1),
                    (long)t5);
}

void timing_set_receipt_print(bool enable)
{
    s_receipt_print = enable;
    ESP_LOGI(TAG, "Receipt timing print: %s", enable ? "ON" : "OFF");
}

bool timing_get_receipt_print(void)
{
    return s_receipt_print;
}
