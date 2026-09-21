/**
 * @file link_mode.h
 * @brief Which connectivity stack is running: gsm, wifi, or neither.
 *
 * Exactly one is ever initialised. That is a memory constraint, not a
 * preference: a spike measured both stacks up together leaving 43 KB of heap
 * free, against the ~45 KB FOTA needs to download an image. GSM alone leaves
 * ~124 KB. So this does not choose a PREFERRED link among several that are
 * running - it chooses which one is brought up at all.
 *
 * WiFi is therefore initialised lazily. esp_wifi_init() allocates its buffers
 * the moment it is called and holds them whether or not the radio ever
 * associates, so it runs only on entering wifi mode, never at boot.
 *
 * The mode is chosen by the operator from the app and stored in NVS. Field
 * machines are powered off between the morning and evening collection
 * sessions; a mode that reset each boot would mean an operator reconnecting
 * over BLE and re-switching twice a day, on a device whose SIM they already
 * know has failed.
 *
 * BLE is unaffected by all three modes and is always the way back - including
 * out of a wifi mode whose router has been replaced. That is what makes
 * persisting the mode safe.
 *
 * This is the only file that knows both GSM and WiFi exist. Neither
 * connectivity component references the other.
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Bring up the stored mode's stack. Call once during startup.
 *
 * Replaces the unconditional gsm_task_start() in main.c. A device with no
 * stored mode gets LINK_MODE_GSM and behaves exactly as it did before this
 * setting existed.
 *
 * Registers the chosen link with net_link too, so this must run before MQTT
 * starts: the MQTT task asks net_link_is_up() as soon as it runs, and a link
 * registered late reads as permanently offline until the next poll.
 *
 * If the stored mode fails to start, falls back to gsm rather than leaving
 * the device with no link at all.
 */
esp_err_t link_mode_start(void);

/**
 * @brief Switch modes: tear the current stack down, bring the new one up.
 *
 * Blocking, and deliberately so - the teardown waits for the outgoing task to
 * exit and release its memory before the incoming stack asks for any. Expect
 * several seconds, mostly powering the modem down.
 *
 * Persists the new mode only after the switch succeeds, so a mode that would
 * not start never becomes what the device boots into.
 *
 * @param mode LINK_MODE_GSM, LINK_MODE_WIFI or LINK_MODE_OFF.
 * @return ESP_OK on success; the current mode is unchanged on failure.
 */
esp_err_t link_mode_switch(uint8_t mode);

/**
 * @brief The mode currently running.
 *
 * Not necessarily the one stored: a failed switch leaves the old stack up and
 * the old mode stored.
 */
uint8_t link_mode_current(void);
