#ifndef WM_UART_VALIDATOR_H_
#define WM_UART_VALIDATOR_H_

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int      shared_gpio;
    int      rmt_channel;
    int      baud_rate;
    uint32_t report_interval_ms;
    uint32_t packet_quiet_ms;
} wm_uart_validator_config_t;

typedef struct {
    int      baud_rate;
    uint64_t hw_bytes;
    uint64_t sw_bytes;
    uint64_t matched_bytes;
    uint64_t frame_errs;
    uint32_t hw_packets;
    uint32_t sw_packets;
    uint32_t json_matches;
    uint32_t json_mismatches;
    int64_t  avg_skew_us;
    int64_t  max_skew_us;
    uint64_t rmt_overflows;
    uint64_t queue_drops;
} wm_uart_validator_stats_t;

esp_err_t wm_uart_validator_init(const wm_uart_validator_config_t *cfg);
esp_err_t wm_uart_validator_start(void);
esp_err_t wm_uart_validator_stop(void);
esp_err_t wm_uart_validator_deinit(void);

void wm_uart_validator_get_stats(wm_uart_validator_stats_t *out);
void wm_uart_validator_reset_stats(void);

/**
 * Sweep across baud rates, logging stats per step. BLOCKS the calling task
 * for (seconds_per_step * n) seconds. Operator must change the WM device
 * baud out-of-band between steps (this function does not control the WM
 * hardware).
 */
esp_err_t wm_uart_validator_run_baud_sweep(const int *bauds, size_t n,
                                           uint32_t seconds_per_step);

#ifdef __cplusplus
}
#endif
#endif
