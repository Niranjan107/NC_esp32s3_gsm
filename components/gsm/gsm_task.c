#include "gsm_task.h"

#ifdef CONFIG_NCLE_GSM_ENABLE

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "GSM_TASK";

#define GSM_POLL_INTERVAL_MS   CONFIG_NCLE_GSM_TASK_POLL_INTERVAL_MS

/* Init retry back-off: quick first retries so a slow-booting modem is picked up
 * within seconds, growing to 5 minutes so a genuinely absent one does not spam
 * the log forever. */
#define GSM_INIT_RETRY_MIN_MS  10000     /* 10 s */
#define GSM_INIT_RETRY_MAX_MS  300000    /* 5 min */

static TaskHandle_t       s_task_handle  = NULL;
static volatile bool      s_running      = false;
static gsm_status_cb_t    s_status_cb    = NULL;
static void              *s_status_ctx   = NULL;
static gsm_status_t       s_last_status  = {0};

static uint8_t rssi_to_bars(uint8_t rssi)
{
    if (rssi == 99 || rssi == 0) return 0;
    if (rssi >= 25) return 5;
    if (rssi >= 19) return 4;
    if (rssi >= 13) return 3;
    if (rssi >= 7)  return 2;
    return 1;
}

static void poll_and_report(void)
{
    gsm_status_t s = {0};

    /* While PPP is up the modem is in DATA mode: the UART carries PPP frames,
     * not AT commands. Issuing AT here would corrupt the data link (or, with
     * a +++ escape, drop it every poll). So when the link is up, report it
     * from the netif and carry the last known signal forward.
     *
     * This is the trade for dropping CMUX - see gsm_ppp_try_apn(). Signal
     * polling resumes automatically whenever the data link is down, which is
     * exactly when it matters for diagnosing why. */
    if (gsm_pdp_is_active()) {
        s.data_up    = true;
        s.alive      = true;                      /* PPP traffic proves it */
        s.rssi       = s_last_status.rssi;        /* last reading before DATA */
        s.ber        = s_last_status.ber;
        s.net_status = s_last_status.net_status;
        s.bars       = rssi_to_bars(s.rssi);
        memcpy(s.module_info, s_last_status.module_info, sizeof(s.module_info));
        memcpy(s.iccid,       s_last_status.iccid,       sizeof(s.iccid));
        s.sim_status = s_last_status.sim_status;

        /* A live data session is proof of BOTH a working SIM and network
         * registration - neither is possible without the other. Cached values
         * read while the modem was still booting can say otherwise (observed:
         * "sim=absent reg=0" alongside data=1), and a status that contradicts
         * itself is worse than no status: it sends the field team to check a
         * SIM that is demonstrably fine. Trust the data link. */
        if (s.sim_status != GSM_SIM_READY) {
            ESP_LOGI(TAG, "data link up - correcting stale sim=%s to ready",
                     gsm_sim_status_str(s.sim_status));
            s.sim_status = GSM_SIM_READY;
            s_last_status.sim_status = GSM_SIM_READY;
        }
        s.registered = true;
        if (s.net_status != GSM_NET_REGISTERED_HOME &&
            s.net_status != GSM_NET_REGISTERED_ROAMING) {
            s.net_status = GSM_NET_REGISTERED_HOME;
            s_last_status.net_status = GSM_NET_REGISTERED_HOME;
        }

        s_last_status = s;
        if (s_status_cb) s_status_cb(&s, s_status_ctx);
        ESP_LOGI(TAG, "alive=1 sim=%s reg=%d rssi=%d bars=%d net=%d data=1 (DATA mode)",
                 gsm_sim_status_str(s.sim_status), s.registered,
                 s.rssi, s.bars, (int)s.net_status);
        return;
    }

    s.alive = gsm_is_alive();

    if (s.alive) {
        if (gsm_get_signal_strength(&s.rssi, &s.ber) != ESP_OK) {
            s.rssi = 99;
            s.ber  = 99;
        }
        if (gsm_get_network_status(&s.net_status) != ESP_OK) {
            s.net_status = GSM_NET_UNKNOWN;
        }
    } else {
        s.rssi       = 99;
        s.ber        = 99;
        s.net_status = GSM_NET_UNKNOWN;
    }

    s.registered = (s.net_status == GSM_NET_REGISTERED_HOME ||
                    s.net_status == GSM_NET_REGISTERED_ROAMING);
    s.bars       = rssi_to_bars(s.rssi);

    /* The data link is a SEPARATE question from registration: a SIM can be
     * registered on the tower and still carry no data (expired plan, wrong
     * APN). Reporting them as one "connected" flag is what sends technicians
     * to healthy machines - see docs/GSM_PORT_PLAN.md Step 6. */
    s.data_up = gsm_pdp_is_active();

    /* Carry forward values captured once at startup */
    memcpy(s.module_info, s_last_status.module_info, sizeof(s.module_info));
    memcpy(s.iccid,       s_last_status.iccid,       sizeof(s.iccid));
    s.sim_status = s_last_status.sim_status;

    /* Registration is proof the SIM works: a modem cannot attach to a network
     * without one. If an earlier +CPIN? failed transiently (SIM busy right
     * after a reset) the cached status can say "error" while the modem is
     * plainly registered - a self-contradictory report that would send the
     * field team hunting a SIM fault that does not exist. Trust the stronger
     * evidence and re-read the real status. */
    if (s.registered && s.sim_status != GSM_SIM_READY) {
        ESP_LOGI(TAG, "registered but sim=%s - re-reading SIM status",
                 gsm_sim_status_str(s.sim_status));
        s.sim_status = gsm_get_sim_status();
        if (s.sim_status != GSM_SIM_READY) {
            /* Still not READY yet registered: the SIM is demonstrably working,
             * so report READY rather than a contradiction. */
            ESP_LOGW(TAG, "SIM reports %s while registered - treating as ready",
                     gsm_sim_status_str(s.sim_status));
            s.sim_status = GSM_SIM_READY;
        }
        s_last_status.sim_status = s.sim_status;

        /* ICCID may have been skipped when the SIM looked bad - fetch it now. */
        if (s_last_status.iccid[0] == '\0') {
            char iccid[24] = {0};
            if (gsm_get_iccid(iccid, sizeof(iccid)) == ESP_OK) {
                ESP_LOGI(TAG, "ICCID: %s", iccid);
                strncpy(s_last_status.iccid, iccid,
                        sizeof(s_last_status.iccid) - 1);
                memcpy(s.iccid, s_last_status.iccid, sizeof(s.iccid));
            }
        }
    }

    s_last_status = s;

    if (s_status_cb) s_status_cb(&s, s_status_ctx);

    ESP_LOGI(TAG, "alive=%d sim=%s reg=%d rssi=%d bars=%d net=%d data=%d",
             s.alive, gsm_sim_status_str(s.sim_status), s.registered,
             s.rssi, s.bars, (int)s.net_status, s.data_up);
}

static void gsm_task_body(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "GSM task starting");

    /* Retry init forever rather than exiting on failure.
     *
     * A modem that does not answer is usually a RECOVERABLE condition: it may
     * power up more slowly than the ESP32 (separate regulators), reboot on its
     * own, brown out briefly, or sit behind a connector that reseats. Exiting
     * here would leave the device permanently without GSM until someone sends
     * gsm_enable by hand or power-cycles it - a site visit for a fault that
     * fixes itself in seconds.
     *
     * Back-off grows so a genuinely absent modem does not fill the log, but
     * recovery stays quick when the modem is merely late. Status is reported
     * honestly as alive=0 throughout - never as a fake "connected". */
    uint32_t retry_delay_ms = GSM_INIT_RETRY_MIN_MS;
    uint32_t attempt        = 0;

    while (s_running && gsm_init() != ESP_OK) {
        attempt++;
        ESP_LOGE(TAG, "gsm_init failed (attempt %u) - retrying in %u s",
                 (unsigned)attempt, (unsigned)(retry_delay_ms / 1000));

        /* Publish the failure so the app shows "no modem" rather than nothing */
        gsm_status_t bad = {
            .alive = false, .registered = false, .data_up = false,
            .sim_status = GSM_SIM_ERROR,
            .rssi = 99, .ber = 99, .bars = 0,
            .net_status = GSM_NET_UNKNOWN,
        };
        s_last_status = bad;
        if (s_status_cb) s_status_cb(&bad, s_status_ctx);

        /* Sleep in 1s slices so gsm_task_stop() is still observed promptly */
        for (uint32_t waited = 0; waited < retry_delay_ms && s_running;
             waited += 1000) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        if (retry_delay_ms < GSM_INIT_RETRY_MAX_MS) {
            retry_delay_ms *= 2;
            if (retry_delay_ms > GSM_INIT_RETRY_MAX_MS) {
                retry_delay_ms = GSM_INIT_RETRY_MAX_MS;
            }
        }
    }

    /* Stopped while retrying - leave cleanly without touching the modem. */
    if (!s_running) {
        ESP_LOGI(TAG, "GSM task stopped during init retry");
        s_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    if (attempt > 0) {
        ESP_LOGI(TAG, "Modem came up after %u retries", (unsigned)attempt);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    char info[64] = {0};
    if (gsm_get_module_info(info, sizeof(info)) == ESP_OK) {
        ESP_LOGI(TAG, "Module: %s", info);
        strncpy(s_last_status.module_info, info,
                sizeof(s_last_status.module_info) - 1);
    }

    /* SIM presence and ICCID: read once, they do not change while powered.
     * Both go through esp_modem now, so a successful read here also proves the
     * UART hand-over worked.
     *
     * The "RDY" URC in the ATI reply means the module has only just finished
     * booting: its SIM interface needs another moment before +CPIN? is
     * meaningful. Without this pause the first read returns "not inserted" on
     * a SIM that is present. */
    vTaskDelay(pdMS_TO_TICKS(1500));

    s_last_status.sim_status = gsm_get_sim_status();
    ESP_LOGI(TAG, "SIM: %s", gsm_sim_status_str(s_last_status.sim_status));

    if (s_last_status.sim_status == GSM_SIM_READY) {
        char iccid[24] = {0};
        if (gsm_get_iccid(iccid, sizeof(iccid)) == ESP_OK) {
            ESP_LOGI(TAG, "ICCID: %s", iccid);
            strncpy(s_last_status.iccid, iccid, sizeof(s_last_status.iccid) - 1);
        }
    }

    poll_and_report();

    /* Bring the data link up. Blocking - worst case around two minutes when
     * every APN fails - which is why it runs here on gsm_task and not in
     * app_main: WM, MA, printer and BLE keep working throughout.
     *
     * Failure is NOT fatal. The task carries on polling signal and
     * registration, so diagnostics keep working on a SIM with no data plan,
     * and gsm_pdp_is_active() simply stays false. */
    ESP_LOGI(TAG, "Bringing data link up...");
    esp_err_t ppp_err = gsm_ppp_start();
    if (ppp_err == ESP_OK) {
        ESP_LOGI(TAG, "=== DATA LINK UP ===");

        /* Prove the link actually carries traffic. An IP address alone does
         * not: a PDP context can be assigned while nothing routes. Ping shows
         * packets flow; the HTTP GET additionally exercises DNS and TCP, which
         * is what MQTT and FOTA will need. Both use the standard stack, so
         * success here means the netif is genuinely socket-capable. */
        vTaskDelay(pdMS_TO_TICKS(2000));   /* let the link settle */
        gsm_test_ping(NULL, 4);
        gsm_test_http_get(NULL);
    } else {
        ESP_LOGW(TAG, "Data link not available (0x%x) - diagnostics continue",
                 ppp_err);
    }
    poll_and_report();

    /* Wake every 1s so gsm_task_stop() is observed within ~1s */
    uint32_t accumulated_ms = 0;
    uint32_t dead_polls     = 0;
    while (s_running) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        accumulated_ms += 1000;
        if (accumulated_ms >= GSM_POLL_INTERVAL_MS) {
            accumulated_ms = 0;
            poll_and_report();

            /* The modem can disappear AFTER a good init - a brown-out, a
             * self-reboot, a connector that moves. Two consecutive dead polls
             * (not one, so a single missed reply does not trigger it) means
             * re-running the whole init sequence, which power-cycles and resets
             * the module. Without this the task would poll a dead modem
             * forever, reporting alive=0 but never trying to recover. */
            /* Never tear the modem down while the data link is up. The crash
             * this guards against: PPP connects, the link later degrades, this
             * loop calls gsm_deinit() -> esp_modem_destroy() while esp_modem's
             * own worker is still running, and it touches freed memory ->
             * InstructionFetchError panic and reboot.
             *
             * If the link is up, gsm_pdp_is_active() is true and poll_and_report
             * has already returned early, so alive is true and we never get
             * here. Bring PPP down first if it is still nominally up. */
            if (!s_last_status.alive && !gsm_pdp_is_active()) {
                if (++dead_polls >= 2) {
                    ESP_LOGW(TAG, "Modem stopped responding - reinitialising");

                    /* Stop PPP cleanly first so esp_modem's worker is idle
                     * before anything is destroyed. */
                    gsm_ppp_stop();
                    vTaskDelay(pdMS_TO_TICKS(500));

                    gsm_deinit();
                    vTaskDelay(pdMS_TO_TICKS(500));   /* let tasks unwind */
                    dead_polls = 0;

                    if (gsm_init() == ESP_OK) {
                        ESP_LOGI(TAG, "Modem recovered");
                        s_last_status.sim_status = gsm_get_sim_status();
                        if (s_last_status.sim_status == GSM_SIM_READY) {
                            char iccid[24] = {0};
                            if (gsm_get_iccid(iccid, sizeof(iccid)) == ESP_OK) {
                                strncpy(s_last_status.iccid, iccid,
                                        sizeof(s_last_status.iccid) - 1);
                            }
                        }
                        /* Data link needs re-establishing too */
                        if (gsm_ppp_start() == ESP_OK) {
                            ESP_LOGI(TAG, "=== DATA LINK RESTORED ===");
                        }
                    } else {
                        ESP_LOGW(TAG, "Reinit failed - will retry next poll");
                    }
                }
            } else {
                dead_polls = 0;
            }
        }
    }

    ESP_LOGI(TAG, "GSM task stopping");
    gsm_deinit();
    s_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t gsm_task_start(void)
{
    if (s_running || s_task_handle) {
        ESP_LOGW(TAG, "Task already running");
        return ESP_OK;
    }

    s_running = true;
    BaseType_t ok = xTaskCreate(gsm_task_body, "gsm_task",
                                4096, NULL, 5, &s_task_handle);
    if (ok != pdPASS) {
        s_running = false;
        ESP_LOGE(TAG, "xTaskCreate failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t gsm_task_stop(void)
{
    if (!s_running) return ESP_OK;
    s_running = false;
    return ESP_OK;
}

bool gsm_task_is_running(void)
{
    return s_running;
}

void gsm_task_set_status_callback(gsm_status_cb_t cb, void *ctx)
{
    s_status_cb  = cb;
    s_status_ctx = ctx;
}

void gsm_task_get_last_status(gsm_status_t *out)
{
    if (out) *out = s_last_status;
}

#endif /* CONFIG_NCLE_GSM_ENABLE */
