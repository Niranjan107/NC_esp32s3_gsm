#ifndef SOFT_UART_RMT_H_
#define SOFT_UART_RMT_H_

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct soft_uart_rmt_obj *soft_uart_rmt_handle_t;

typedef struct {
    int      gpio_num;              /* RX pin */
    int      baud_rate;             /* 9600..115200 */
    uint8_t  data_bits;             /* must be 8 in v1 */
    uint8_t  stop_bits;             /* must be 1 in v1 */
    uint8_t  parity;                /* must be 0 (none) in v1 */
    uint32_t resolution_hz;         /* RMT tick rate; 0 = use Kconfig default */
    int      rmt_mem_block_symbols; /* RMT buffer words; 0 = use Kconfig default */
    size_t   byte_queue_depth;      /* FreeRTOS queue size; 0 = use 256 */
} soft_uart_rmt_config_t;

typedef struct {
    uint8_t  byte;
    int64_t  ts_us;                 /* esp_timer_get_time() at frame completion */
    bool     frame_err;
} soft_uart_rmt_rx_t;

typedef void (*soft_uart_rmt_byte_cb_t)(const soft_uart_rmt_rx_t *rx,
                                        void *user_ctx);

typedef struct {
    uint64_t bytes_ok;
    uint64_t bytes_frame_err;
    uint64_t rmt_overflows;
    uint64_t queue_drops;
} soft_uart_rmt_stats_t;

esp_err_t soft_uart_rmt_init(const soft_uart_rmt_config_t *cfg,
                             soft_uart_rmt_handle_t *out);
esp_err_t soft_uart_rmt_start(soft_uart_rmt_handle_t h);
esp_err_t soft_uart_rmt_stop(soft_uart_rmt_handle_t h);
esp_err_t soft_uart_rmt_deinit(soft_uart_rmt_handle_t h);

esp_err_t soft_uart_rmt_register_byte_cb(soft_uart_rmt_handle_t h,
                                         soft_uart_rmt_byte_cb_t cb,
                                         void *user_ctx);

esp_err_t soft_uart_rmt_set_baud(soft_uart_rmt_handle_t h, int baud_rate);

void soft_uart_rmt_get_stats(soft_uart_rmt_handle_t h,
                             soft_uart_rmt_stats_t *out);
void soft_uart_rmt_reset_stats(soft_uart_rmt_handle_t h);

#ifdef __cplusplus
}
#endif
#endif /* SOFT_UART_RMT_H_ */
