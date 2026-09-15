#include "wm_capture.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"        /* monotonic clock for weight freshness (no RTC) */

/* App-team-aligned weight capture (mirrors the mobile app's BLE flow exactly):
 *  - every VALID (non-zero) WM value during the window is stored in a small
 *    temporary list (numbers + value strings)
 *  - when the MA arrives we wait WM_GRACE_MS so any reading still in flight
 *    joins the list (the app does the same over BLE)
 *  - then ONE weight is selected with the app's findRepeatingNumber() rule:
 *      1) the OLDEST value that repeats >= WM_REPEAT_MIN times
 *      2) else the FIRST value within (average +/- WM_AVG_TOL)
 *      3) else (safety, connector-only) the last value
 *    and the list is cleared for the next collection. */

/* Tunables. The defaults match the mobile app; all three are exposed in
 * menuconfig (Nitara CLV4 -> WM UART) so a site can adapt without a code change.
 * Raising NCLE_WM_GRACE_MS is how you make CONTINUOUS analysers collect real
 * samples: their frame ends in ~40ms, so a longer grace is the only way the
 * window sees more than one weight and the repeat rule can run. */
#ifndef CONFIG_NCLE_WM_GRACE_MS
#define CONFIG_NCLE_WM_GRACE_MS   200
#endif
#ifndef CONFIG_NCLE_WM_GRACE_CONTINUOUS_MS
#define CONFIG_NCLE_WM_GRACE_CONTINUOUS_MS 2000
#endif
#ifndef CONFIG_NCLE_WM_STALE_MS
#define CONFIG_NCLE_WM_STALE_MS   5000
#endif
#ifndef CONFIG_NCLE_WM_REPEAT_MIN
#define CONFIG_NCLE_WM_REPEAT_MIN 4
#endif

#define WM_VAL_MAX    24
#define WM_LIST_MAX   32      /* most recent values kept (ring buffer) */
/* Grace actually used. Defaults to the single-shot value; the MA driver calls
 * wm_capture_set_continuous() when a terminator-framed model (3xxx/4xxx) is
 * configured, switching it to the longer value so the window can collect real
 * samples instead of falling back to the last known weight. */
static volatile int s_grace_ms = CONFIG_NCLE_WM_GRACE_MS;

void wm_capture_set_continuous(bool continuous)
{
    s_grace_ms = continuous ? CONFIG_NCLE_WM_GRACE_CONTINUOUS_MS
                            : CONFIG_NCLE_WM_GRACE_MS;
}
#define WM_REPEAT_MIN CONFIG_NCLE_WM_REPEAT_MIN /* app rule: value must appear >= N times */
#define WM_AVG_TOL    0.5f    /* app rule: fallback = first within avg +/- 0.5 */
#define WM_STALE_US   ((int64_t)CONFIG_NCLE_WM_STALE_MS * 1000)  /* last-weight validity */

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static bool  s_active = false;
static char  s_vals[WM_LIST_MAX][WM_VAL_MAX];  /* value strings (verbatim) */
static float s_nums[WM_LIST_MAX];              /* parsed numeric values    */
static int   s_count = 0;                      /* total fed (ring may wrap) */

/* Last known non-zero weight, kept ACROSS capture windows (not cleared by
 * wm_capture_start). The scale value physically persists between the meter's
 * periodic reports, so when a short per-frame window catches no fresh sample -
 * as happens on continuous MA modes (3xxx/4xxx), whose frames re-open the
 * window faster than the WM reports - we fall back to this. Guarded by a 5 s
 * freshness window so a disconnected/stopped WM never attaches a stale weight. */
static char    s_last_weight[WM_VAL_MAX] = {0};
static int64_t s_last_weight_us = 0;

/* Parse the numeric value out of a WM string, honouring the decimal point:
 * "+0008.50Kg" -> 8.50, "N0001.25=lt" -> 1.25, "L01234" -> 1234, "00012" -> 12.
 * Returns 0 for all-zero / no-digit strings. */
static float wm_to_val(const char *s)
{
    const char *p = s;
    while (*p && (*p < '0' || *p > '9')) {
        p++;                                   /* skip prefix (N, L, +, spaces) */
    }
    if (p > s && *(p - 1) == '-') {
        p--;                                   /* keep a leading minus sign */
    }
    if (*p == '\0') {
        return 0.0f;
    }
    return strtof(p, NULL);
}

void wm_capture_start(void)
{
    portENTER_CRITICAL(&s_mux);
    s_active = true;
    s_count  = 0;   /* clear the list - new collection window */
    portEXIT_CRITICAL(&s_mux);
}

void wm_capture_feed(const char *wm_clean)
{
    if (!wm_clean) {
        return;
    }
    float v = wm_to_val(wm_clean);
    if (v <= 0.0f) {
        return;   /* zero/empty scale or invalid - never stored (app rule) */
    }
    int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&s_mux);
    /* Remember the latest non-zero weight regardless of whether a capture
     * window is open, so it is ready as the fallback for the next MA reading. */
    strncpy(s_last_weight, wm_clean, WM_VAL_MAX - 1);
    s_last_weight[WM_VAL_MAX - 1] = '\0';
    s_last_weight_us = now;
    if (s_active) {
        int idx = s_count % WM_LIST_MAX;
        strncpy(s_vals[idx], wm_clean, WM_VAL_MAX - 1);
        s_vals[idx][WM_VAL_MAX - 1] = '\0';
        s_nums[idx] = v;
        s_count++;
    }
    portEXIT_CRITICAL(&s_mux);
}

/* >>> SELECTION - matches the app team's findRepeatingNumber() <<<
 * 1) oldest value repeated >= WM_REPEAT_MIN times
 * 2) else first value within (average +/- WM_AVG_TOL)
 * 3) else last value (connector-only safety so a measured weight is not lost)
 * Must be called with s_mux held. Returns 1 and fills out, or 0 if empty. */
static int wm_select_locked(char *out, size_t sz)
{
    int n = (s_count < WM_LIST_MAX) ? s_count : WM_LIST_MAX;
    if (n == 0) {
        /* No fresh sample in this window - fall back to the last known weight
         * if it is still recent (continuous MA modes re-open the window faster
         * than the WM reports, so the weight lives here between reports). */
        if (s_last_weight[0] != '\0' &&
            (esp_timer_get_time() - s_last_weight_us) <= WM_STALE_US) {
            if (out && sz) {
                strncpy(out, s_last_weight, sz - 1);
                out[sz - 1] = '\0';
            }
            return 1;
        }
        if (out && sz) out[0] = '\0';
        return 0;
    }
    int start = (s_count > WM_LIST_MAX) ? (s_count % WM_LIST_MAX) : 0;
    int pick = -1;

    /* Rule 1: oldest value that repeats >= WM_REPEAT_MIN times (exact repeat) */
    for (int k = 0; k < n && pick < 0; k++) {           /* oldest -> newest */
        int i = (start + k) % WM_LIST_MAX;
        int cnt = 0;
        for (int m = 0; m < n; m++) {
            int j = (start + m) % WM_LIST_MAX;
            if (fabsf(s_nums[j] - s_nums[i]) < 0.001f) {
                cnt++;
            }
        }
        if (cnt >= WM_REPEAT_MIN) {
            pick = i;
        }
    }

    /* Rule 2: first value within (average +/- WM_AVG_TOL) */
    if (pick < 0) {
        float sum = 0.0f;
        for (int m = 0; m < n; m++) {
            sum += s_nums[(start + m) % WM_LIST_MAX];
        }
        float avg = sum / (float)n;
        for (int k = 0; k < n && pick < 0; k++) {       /* oldest -> newest */
            int i = (start + k) % WM_LIST_MAX;
            if (fabsf(s_nums[i] - avg) <= WM_AVG_TOL) {
                pick = i;
            }
        }
    }

    /* Rule 3 (safety): nothing matched -> take the last value */
    if (pick < 0) {
        pick = (start + n - 1) % WM_LIST_MAX;
    }

    if (out && sz) {
        strncpy(out, s_vals[pick], sz - 1);
        out[sz - 1] = '\0';
    }
    return 1;
}

bool wm_capture_result(char *out, size_t sz)
{
    char r[WM_VAL_MAX];
    portENTER_CRITICAL(&s_mux);
    s_active = false;
    int got = wm_select_locked(r, sizeof(r));
    s_count = 0;                     /* clear for the next collection */
    portEXIT_CRITICAL(&s_mux);

    if (!got) {
        if (out && sz) out[0] = '\0';
        return false;
    }
    if (out && sz) {
        strncpy(out, r, sz - 1);
        out[sz - 1] = '\0';
    }
    return true;
}

int wm_capture_merge_into_ma(const char *ma_json, int ma_len, char *out, size_t out_sz)
{
    if (!ma_json || ma_len <= 0 || !out || out_sz == 0) {
        return 0;
    }

    /* MA message -> app-team flow: wait the grace period so any WM reading
     * still in transit joins the list, then select + merge + clear. The
     * {"device":"ma"} 14-char prefix is preserved so downstream MA detection
     * (store-and-forward) still matches. No weight collected -> "wm":"". */
    if (ma_len >= 14 && ma_json[0] == '{' &&
        memcmp(ma_json, "{\"device\":\"ma\"", 14) == 0) {
        vTaskDelay(pdMS_TO_TICKS(s_grace_ms));    /* grace: collect stragglers */

        char wm[WM_VAL_MAX];
        wm_capture_result(wm, sizeof(wm));        /* "" if nothing measured */
        int n = snprintf(out, out_sz, "%.14s,\"wm\":\"%s\"%.*s",
                         ma_json, wm, ma_len - 14, ma_json + 14);
        if (n < 0) {
            return 0;
        }
        if ((size_t)n >= out_sz) {
            n = (int)out_sz - 1;
        }
        return n;
    }

    /* Not an MA message -> copy verbatim (don't consume the list). */
    int n = snprintf(out, out_sz, "%.*s", ma_len, ma_json);
    if (n < 0) {
        return 0;
    }
    if ((size_t)n >= out_sz) {
        n = (int)out_sz - 1;
    }
    return n;
}
