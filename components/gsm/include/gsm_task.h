#ifndef GSM_TASK_H
#define GSM_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "gsm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool                  alive;
    gsm_network_status_t  net_status;
    bool                  registered;     /* HOME or ROAMING */
    uint8_t               rssi;           /* 0-31, 99 = unknown */
    uint8_t               ber;            /* 0-7,  99 = unknown */
    uint8_t               bars;           /* 0-5, derived from rssi */
    char                  module_info[64];
} gsm_status_t;

typedef void (*gsm_status_cb_t)(const gsm_status_t *s, void *ctx);

/* Lifecycle */
esp_err_t gsm_task_start(void);
esp_err_t gsm_task_stop(void);
bool      gsm_task_is_running(void);

/* Status delivery */
void      gsm_task_set_status_callback(gsm_status_cb_t cb, void *ctx);
void      gsm_task_get_last_status(gsm_status_t *out);   /* cached, no AT roundtrip */

#ifdef __cplusplus
}
#endif
#endif /* GSM_TASK_H */
