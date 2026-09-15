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

/* Connectivity is reported as SEPARATE stages, never collapsed into one
 * "connected" flag - the field team needs to know WHICH stage failed:
 *
 *   alive       modem responding        -> power / wiring / PWRKEY
 *   sim_status  SIM present & unlocked  -> insert SIM / remove PIN
 *   registered  found a tower           -> coverage / antenna
 *   data_up     IP assigned via PPP     -> data plan / APN
 *
 * A SIM can be registered and still carry no data (expired plan), which is
 * exactly the case that a single boolean would misreport.
 */
typedef struct {
    bool                  alive;
    gsm_sim_status_t      sim_status;     /* SIM presence (AT+CPIN?)   */
    gsm_network_status_t  net_status;
    bool                  registered;     /* HOME or ROAMING */
    bool                  data_up;        /* PPP has an IP - real data link */
    uint8_t               rssi;           /* 0-31, 99 = unknown */
    uint8_t               ber;            /* 0-7,  99 = unknown */
    uint8_t               bars;           /* 0-5, derived from rssi */
    char                  module_info[64];
    char                  iccid[24];      /* SIM serial, "" if unread */
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
