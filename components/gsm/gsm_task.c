#include "gsm_task.h"

#ifdef CONFIG_NCLE_GSM_ENABLE

#include <string.h>
#include "esp_log.h"
#include "esp_system.h"      /* esp_restart - SIM recovery backstop */
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
static gsm_fault_t        s_last_fault   = GSM_FAULT_NONE;

/* False until gsm_ppp_start() has run at least once. Before that, data=0 is
 * the normal starting state rather than a fault worth reporting. */
static bool               s_ppp_attempted = false;

/* Set when a SIM that was absent/locked becomes READY again, so the poll loop
 * can re-establish the data link instead of waiting for a reboot. */
static bool               s_sim_reinserted = false;

/* Consecutive polls where the modem did not answer. One missed reply during
 * PPP teardown is normal and must not be reported as a dead modem. */
static uint8_t            s_unanswered_polls = 0;

/* Consecutive polls reporting the SIM absent. The EC200U only reads the SIM
 * slot at power-up, so after this many polls the modem is reset to make it
 * look again. Rate-limited so an empty holder does not cause a reset loop:
 * 3 polls at 10s = one reset per ~30s. */
static uint8_t            s_sim_absent_polls = 0;
#define GSM_SIM_RESET_AFTER_POLLS  3

/* How many modem power cycles to try before rebooting the ESP32 itself. The
 * reboot is the backstop for a SIM swapped while running: it drives the whole
 * board through the same power-up path a human would, so nobody has to switch
 * the machine off. Deliberately generous, so an empty SIM holder spends about
 * 2 minutes failing rather than entering a reboot loop. */
static uint8_t            s_sim_recovery_attempts = 0;
#define GSM_SIM_REBOOT_AFTER_ATTEMPTS  4

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

        /* Link is up and carrying traffic, so no fault to report. A stale
         * fault from before the link came up would contradict data=1. */
        s.fault = GSM_FAULT_NONE;

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
        s_unanswered_polls = 0;    /* answered, so clear the strike count */

        /* Re-read the SIM when it is not currently READY, so a card put back in
         * is noticed without a reboot.
         *
         * IMPORTANT: the EC200U only reads the SIM slot at power-up. Simply
         * asking +CPIN? again after re-insertion returns "not inserted"
         * forever - observed here as 25 consecutive CME ERROR 10 replies with
         * the card physically seated. The module must be RESET before it will
         * look at the slot again.
         *
         * So: ask a few times (cheap, and covers a SIM that was merely slow to
         * initialise), and if it still reports absent, reset the modem once and
         * ask again. Reset is rate-limited so a genuinely empty holder does not
         * put the device into a reset loop. */
        if (s_last_status.sim_status != GSM_SIM_READY) {
            gsm_sim_status_t now = gsm_get_sim_status();

            if (now != GSM_SIM_READY &&
                ++s_sim_absent_polls >= GSM_SIM_RESET_AFTER_POLLS) {
                s_sim_absent_polls = 0;
                s_sim_recovery_attempts++;

                /* POWER CYCLE, not gsm_reset(). The RST line restarts the
                 * module's firmware but does not necessarily drop power to the
                 * SIM interface, so the slot is never re-read and the card
                 * still reports "not inserted" - which is what we saw: a reset
                 * changed nothing while a manual power cycle worked.
                 * PWRKEY removes power properly, which is what makes the
                 * module look at the slot again on the way back up. */
                ESP_LOGW(TAG, "SIM still absent - power-cycling modem (attempt %u) "
                              "so it re-reads the SIM slot",
                         (unsigned)s_sim_recovery_attempts);
                gsm_power_off();
                vTaskDelay(pdMS_TO_TICKS(2000));   /* let the rail discharge */
                gsm_power_on();
                gsm_sim_cache_invalidate();
                now = gsm_get_sim_status();

                /* Last resort: if power-cycling the modem never recovers the
                 * SIM, reboot the ESP32. That drives the whole board through
                 * the same power-up path a human would, so a SIM swapped in
                 * the field recovers without anyone having to switch the
                 * machine off. Only after several failed attempts, and only
                 * when the modem is otherwise healthy - a genuinely empty
                 * holder must not put the device in a reboot loop. */
                if (now != GSM_SIM_READY &&
                    s_sim_recovery_attempts >= GSM_SIM_REBOOT_AFTER_ATTEMPTS) {
                    ESP_LOGE(TAG, "SIM not recovered after %u power cycles - "
                                  "restarting device",
                             (unsigned)s_sim_recovery_attempts);
                    vTaskDelay(pdMS_TO_TICKS(1000));   /* let the log flush */
                    esp_restart();
                }
            }

            if (now != s_last_status.sim_status) {
                ESP_LOGI(TAG, "SIM state changed: %s -> %s",
                         gsm_sim_status_str(s_last_status.sim_status),
                         gsm_sim_status_str(now));
            }
            s_last_status.sim_status = now;

            /* Freshly re-inserted: pick the ICCID up again and retry the data
             * link, rather than waiting for a reboot. */
            if (now == GSM_SIM_READY) {
                s_sim_absent_polls      = 0;
                s_sim_recovery_attempts = 0;   /* recovered - start afresh */
                char iccid[24] = {0};
                if (gsm_get_iccid(iccid, sizeof(iccid)) == ESP_OK) {
                    ESP_LOGI(TAG, "ICCID: %s", iccid);
                    strncpy(s_last_status.iccid, iccid,
                            sizeof(s_last_status.iccid) - 1);
                }
                s_sim_reinserted = true;
            }
        }
    } else {
        s.rssi       = 99;
        s.ber        = 99;
        s.net_status = GSM_NET_UNKNOWN;
    }

    s.registered = (s.net_status == GSM_NET_REGISTERED_HOME ||
                    s.net_status == GSM_NET_REGISTERED_ROAMING);
    s.bars       = rssi_to_bars(s.rssi);

    /* Derive the fault from what this poll just measured, so the reported
     * cause tracks reality instead of whatever the last full diagnostic found.
     * Same order as gsm_diagnose(): first failing stage wins. */
    /* SIM faults are checked BEFORE signal and registration, and use the value
     * just refreshed above rather than the stale copy in s. Getting this order
     * wrong produced the worst message this diagnostic has printed: with the
     * SIM physically removed it reported "no coverage - check the antenna"
     * while showing rssi=31 on the same line, because an absent SIM cannot
     * register and the registration check fired first. Someone would go looking
     * for coverage while the SIM sat on the bench. */
    s.sim_status = s_last_status.sim_status;

    /* One unanswered poll is not a dead modem: the modem is briefly busy while
     * PPP tears down, and reporting "check module power and wiring" for that
     * sends the user to inspect hardware that is working. Require two in a row,
     * matching the threshold the recovery path already uses. */
    if (!s.alive) {
        if (++s_unanswered_polls < 2) {
            ESP_LOGI(TAG, "modem did not answer (1st) - waiting before judging");
            s.fault = GSM_FAULT_NONE;
        } else {
            s.fault = GSM_FAULT_MODEM_DEAD;
        }
    }
    else if (s.sim_status == GSM_SIM_ABSENT)       s.fault = GSM_FAULT_SIM_ABSENT;
    else if (s.sim_status == GSM_SIM_PIN_REQUIRED) s.fault = GSM_FAULT_SIM_LOCKED;
    else if (s.sim_status == GSM_SIM_ERROR)        s.fault = GSM_FAULT_SIM_FAILURE;
    else if (s.rssi == 99)                 s.fault = GSM_FAULT_NO_SIGNAL;
    else if (s.rssi < GSM_RSSI_WEAK_THRESHOLD) s.fault = GSM_FAULT_WEAK_SIGNAL;
    else if (s.net_status == GSM_NET_DENIED)   s.fault = GSM_FAULT_SIM_BARRED;
    else if (!s.registered)                s.fault = GSM_FAULT_NO_COVERAGE;
    /* Only call "no data link" a fault once we have actually TRIED to connect.
     * The first poll runs before gsm_ppp_start(), where data=0 is simply the
     * normal starting state - reporting it as a fault told the user to change
     * the APN moments before the existing APN connected perfectly. */
    else if (!s.data_up && s_ppp_attempted)  s.fault = GSM_FAULT_NO_DATA_LINK;
    else                                   s.fault = GSM_FAULT_NONE;

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
    /* ONLY when genuinely registered (HOME or ROAMING). This originally tested
     * a looser condition and fired while the modem was merely SEARCHING
     * (CREG=2), overwriting a correctly-detected "absent" with "ready" - after
     * which the fault fell through to no_coverage and told the user to check
     * the antenna for a SIM that was sitting on the bench. Searching proves
     * nothing about the SIM; only a completed registration does. */
    bool truly_registered = (s.net_status == GSM_NET_REGISTERED_HOME ||
                             s.net_status == GSM_NET_REGISTERED_ROAMING);

    if (truly_registered && s.sim_status != GSM_SIM_READY) {
        ESP_LOGI(TAG, "registered but sim=%s - re-reading SIM status",
                 gsm_sim_status_str(s.sim_status));
        s.sim_status = gsm_get_sim_status();
        if (s.sim_status != GSM_SIM_READY) {
            /* Still not READY yet genuinely attached to a network: the SIM is
             * demonstrably working, so report READY rather than a
             * contradiction. */
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

    /* Re-derive the fault AFTER the SIM re-read above, which can change
     * sim_status. Computing it earlier meant the reported fault described the
     * value before the correction - the status line and the fault disagreed. */
    if (!s.alive) {
        s.fault = (s_unanswered_polls >= 2) ? GSM_FAULT_MODEM_DEAD
                                            : GSM_FAULT_NONE;
    } else if (s.sim_status == GSM_SIM_ABSENT)       s.fault = GSM_FAULT_SIM_ABSENT;
    else if (s.sim_status == GSM_SIM_PIN_REQUIRED)   s.fault = GSM_FAULT_SIM_LOCKED;
    else if (s.sim_status == GSM_SIM_ERROR)          s.fault = GSM_FAULT_SIM_FAILURE;
    else if (s.rssi == 99)                           s.fault = GSM_FAULT_NO_SIGNAL;
    else if (s.rssi < GSM_RSSI_WEAK_THRESHOLD)       s.fault = GSM_FAULT_WEAK_SIGNAL;
    else if (s.net_status == GSM_NET_DENIED)         s.fault = GSM_FAULT_SIM_BARRED;
    else if (!s.registered)                          s.fault = GSM_FAULT_NO_COVERAGE;
    else if (!s.data_up && s_ppp_attempted)          s.fault = GSM_FAULT_NO_DATA_LINK;
    else                                             s.fault = GSM_FAULT_NONE;

    s_last_status = s;

    if (s_status_cb) s_status_cb(&s, s_status_ctx);

    if (s.fault == GSM_FAULT_NONE) {
        ESP_LOGI(TAG, "alive=%d sim=%s reg=%d rssi=%d bars=%d net=%d data=%d",
                 s.alive, gsm_sim_status_str(s.sim_status), s.registered,
                 s.rssi, s.bars, (int)s.net_status, s.data_up);
    } else {
        /* Name the fault on the status line itself, so a log skimmed weeks
         * later still says what was wrong without cross-referencing numbers. */
        ESP_LOGW(TAG, "alive=%d sim=%s reg=%d rssi=%d bars=%d net=%d data=%d "
                      "FAULT=%s (%s)",
                 s.alive, gsm_sim_status_str(s.sim_status), s.registered,
                 s.rssi, s.bars, (int)s.net_status, s.data_up,
                 gsm_fault_name(s.fault), gsm_fault_action(s.fault));
    }
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
    s_ppp_attempted = true;
    bool ping_ok = false;
    esp_err_t ppp_err = gsm_ppp_start();
    if (ppp_err == ESP_OK) {
        ESP_LOGI(TAG, "=== DATA LINK UP ===");

        /* Prove the link actually carries traffic. An IP address alone does
         * not: a PDP context can be assigned while nothing routes. Ping shows
         * packets flow; the HTTP GET additionally exercises DNS and TCP, which
         * is what MQTT and FOTA will need. Both use the standard stack, so
         * success here means the netif is genuinely socket-capable. */
        vTaskDelay(pdMS_TO_TICKS(2000));   /* let the link settle */
        ping_ok = (gsm_test_ping(NULL, 4) == ESP_OK);
        gsm_test_http_get(NULL);
    } else {
        ESP_LOGW(TAG, "Data link not available (0x%x)", ppp_err);
    }

    /* Print the stage-by-stage summary. On success it confirms every stage; on
     * failure it names the ONE thing to fix, so the log is usable by whoever is
     * in front of the machine rather than only by someone who can read AT
     * traffic.
     *
     * The internet stage is skipped when the ping above already proved it -
     * repeating it would cost another 15s and a second round of data for an
     * answer we have. */
    s_last_fault = gsm_diagnose(false);
    if (ping_ok) {
        ESP_LOGI(TAG, "(internet already verified by the ping above)");
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
                        /* SIM status is NOT read here: gsm_ppp_start() checks
                         * it as a precondition, and reading it twice ran the
                         * 5-attempt retry loop twice - ten AT commands and ten
                         * seconds to answer one question. */
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

            /* SIM put back in: bring the data link up again rather than making
             * the user power-cycle a device they have already fixed. */
            if (s_sim_reinserted && !gsm_pdp_is_active()) {
                s_sim_reinserted = false;
                ESP_LOGI(TAG, "SIM re-inserted - reconnecting data link");
                if (gsm_ppp_start() == ESP_OK) {
                    ESP_LOGI(TAG, "=== DATA LINK RESTORED ===");
                    gsm_test_ping(NULL, 3);
                }
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
