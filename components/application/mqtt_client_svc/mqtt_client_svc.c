/**
 * @file mqtt_client_svc.c
 * @brief MQTT cloud service — connect, online/offline status, publish, commands.
 */
#include "mqtt_client_svc.h"
#include "sdkconfig.h"

#ifdef CONFIG_NCLE_MQTT_ENABLE
/* ==== MQTT enabled: full implementation ================================== */

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "mqtt_client.h"          /* ESP-IDF esp-mqtt */
#include "esp_crt_bundle.h"       /* public-CA trust for mqtts:// (TLS) */
#include "esp_log.h"
#include "net_link.h"             /* "can I send right now?" - transport-agnostic */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "store_forward.h"        /* offline buffer (LittleFS) */
#include "wm_capture.h"           /* merge settled WM weight into the MA message */

/* Store-and-forward on/off, from base's `config`. Forward-declared rather than
 * including device_config.h, which pulls in the UART port types this layer has
 * no business knowing about. */
bool config_get_store_forward(void);

/* From the `common` component (MAC-based device id). Forward-declared rather
 * than including common.h, whose include guard collides with a header pulled
 * in transitively by mqtt_client.h, which would skip common.h here. */
extern uint64_t get_unique_id(void);

/* ---- Broker address + credentials ----------------------------------------
 * Real broker details live in `mqtt_secrets.h` (gitignored, copied from
 * mqtt_secrets.h.example) so they never reach the repository. That file is
 * OPTIONAL - without it the build falls back to the menuconfig values, which
 * default to the public test broker with no authentication. Nothing depends on
 * the file existing. */
#if defined(__has_include)
#  if __has_include("mqtt_secrets.h")
#    include "mqtt_secrets.h"
#  endif
#endif
#ifndef NCLE_MQTT_URI
#define NCLE_MQTT_URI   CONFIG_NCLE_MQTT_BROKER_URI
#endif
#ifndef NCLE_MQTT_USER
#define NCLE_MQTT_USER  CONFIG_NCLE_MQTT_USERNAME
#endif
#ifndef NCLE_MQTT_PASS
#define NCLE_MQTT_PASS  CONFIG_NCLE_MQTT_PASSWORD
#endif

static const char *TAG = "MQTT_SVC";

/* Presence payloads for clv4/<id>/status. JSON so every topic the server
 * consumes (data / resp / status) has the same shape and one parser handles
 * all three. `offline` is the MQTT Last-Will: its payload is fixed when the
 * client connects, so the broker publishes exactly this if the device drops. */
#define STATUS_ONLINE_JSON   "{\"status\":\"online\"}"
#define STATUS_OFFLINE_JSON  "{\"status\":\"offline\"}"

/* Topics/identity */
static char s_device_id[16];
static char s_client_id[24];
static char s_data_topic[48];
static char s_cmd_topic[48];
static char s_resp_topic[48];
static char s_status_topic[48];

static esp_mqtt_client_handle_t s_client = NULL;
static volatile bool s_connected = false;
static mqtt_cmd_cb_t s_cmd_cb = NULL;

/* Diagnostics counters (read by the BLE `diag` command in a later step). */
static volatile uint32_t s_published = 0;   /* readings actually sent to broker */
static volatile uint32_t s_dropped   = 0;   /* readings not sent (offline/queue full) */
static volatile uint32_t s_deduped   = 0;   /* continuous-meter repeats skipped     */
static volatile uint32_t s_to_app    = 0;   /* handed to the app over BLE, link down */
static bool s_buffer_warned = false;        /* "nearly full" warning already sent   */

/* Message buffers / queues.
 *  - s_out_queue: readings to publish (producer = WM/MA callback, Task 2)
 *  - s_cmd_queue: commands received from clv4/<id>/cmd (Task 3). Processed on
 *    a dedicated task so a slow command (e.g. printing) never blocks the MQTT
 *    event task / connection. */
#define MQTT_MSG_MAX   1200
#define MQTT_QUEUE_LEN 8
#define MQTT_CMD_QUEUE_LEN 4
typedef struct { int len; char data[MQTT_MSG_MAX]; } mqtt_msg_t;
static QueueHandle_t s_out_queue = NULL;
static QueueHandle_t s_cmd_queue = NULL;

/* ---- Continuous-meter de-duplication --------------------------------------
 * A continuous MA (models 3xxx/4xxx) repeats the SAME frame many times per
 * second. Without this guard every repeat would be published AND (when offline)
 * written to flash, flooding both the broker and LittleFS. We publish the FIRST
 * occurrence of a reading and skip byte-identical repeats until the content
 * changes (next farmer / new test). The check is on the RAW MA json (before the
 * WM merge) so a skipped duplicate also costs no 200 ms WM-grace delay.
 * BLE/USB are unaffected - they output every frame (live display).
 * Single-threaded: only the MA RX task calls mqtt_svc_publish_data(), so the
 * baseline needs no lock. Assumes distinct milk tests never produce byte-
 * identical readings - the same assumption the server's MD5 dedup already makes. */
#define MA_PREFIX      "{\"device\":\"ma\""
#define MA_PREFIX_LEN  14
static char s_last_ma[MQTT_MSG_MAX];        /* last accepted MA reading (raw) */
static int  s_last_ma_len = 0;

/* Store-and-forward flush uses delete-after-ACK: we wait for the broker's
 * PUBACK before removing a buffered record from flash (so nothing is lost on a
 * reboot). The MQTT event task sets SF_ACK_BIT when MQTT_EVENT_PUBLISHED fires
 * for the msg_id the flush loop is waiting on. */
#define SF_ACK_BIT   (1 << 0)
static EventGroupHandle_t s_sf_evt = NULL;
static volatile int s_sf_pending_msgid = -1;

void mqtt_svc_set_cmd_callback(mqtt_cmd_cb_t cb) { s_cmd_cb = cb; }
bool mqtt_svc_is_connected(void) { return s_connected; }
const char *mqtt_svc_broker_uri(void) { return NCLE_MQTT_URI; }

/* Suspend/resume for FOTA - see the header for why this exists. s_suspended
 * stops mqtt_task restarting the client behind our back the moment it notices
 * the link is up. */
static volatile bool s_suspended = false;

void mqtt_svc_suspend(void)
{
    if (!s_client || s_suspended) {
        return;
    }
    s_suspended = true;
    esp_mqtt_client_stop(s_client);
    s_connected = false;
    ESP_LOGI(TAG, "MQTT suspended (freeing TLS session)");
}

void mqtt_svc_resume(void)
{
    if (!s_client || !s_suspended) {
        return;
    }
    s_suspended = false;
    esp_err_t err = esp_mqtt_client_start(s_client);
    ESP_LOGI(TAG, "MQTT resumed (%s)", esp_err_to_name(err));
}

void mqtt_svc_get_stats(mqtt_svc_stats_t *out)
{
    if (!out) {
        return;
    }
    out->connected = s_connected;
    out->published = s_published;
    out->dropped   = s_dropped;
    out->deduped   = s_deduped;
    out->buffered  = sf_count();
}

static void build_ids(void)
{
    unsigned long long uid = (unsigned long long)get_unique_id();
    snprintf(s_device_id,   sizeof(s_device_id),   "%012llx", uid);
    snprintf(s_client_id,   sizeof(s_client_id),   "clv4-%s", s_device_id);
    snprintf(s_data_topic,  sizeof(s_data_topic),  "clv4/%s/data",   s_device_id);
    snprintf(s_cmd_topic,   sizeof(s_cmd_topic),   "clv4/%s/cmd",    s_device_id);
    snprintf(s_resp_topic,  sizeof(s_resp_topic),  "clv4/%s/resp",   s_device_id);
    snprintf(s_status_topic,sizeof(s_status_topic),"clv4/%s/status", s_device_id);
    ESP_LOGI(TAG, "Device id=%s  data=%s  cmd=%s", s_device_id, s_data_topic, s_cmd_topic);
}

static void mqtt_event_handler(void *args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            s_connected = true;
            ESP_LOGI(TAG, "Connected to broker");
            /* Announce online (retained) + subscribe to commands */
            esp_mqtt_client_publish(s_client, s_status_topic, STATUS_ONLINE_JSON, 0, 1, 1);
            esp_mqtt_client_subscribe(s_client, s_cmd_topic, 1);

            /* Publish startup version notification to response topic */
            #ifdef NCLE_FIRMWARE_VERSION
            {
                char boot_msg[128];
                snprintf(boot_msg, sizeof(boot_msg),
                         "{\"response_message\":\"device_boot\",\"status_code\":0,\"data\":{\"version\":\"%s\"}}",
                         NCLE_FIRMWARE_VERSION);
                esp_mqtt_client_publish(s_client, s_resp_topic, boot_msg, 0, 1, 0);
            }
            #endif
            break;
        case MQTT_EVENT_DISCONNECTED:
            s_connected = false;
            ESP_LOGW(TAG, "Disconnected from broker");
            break;
        case MQTT_EVENT_DATA:
            /* Incoming command on clv4/<id>/cmd. Copy it onto the command queue
             * and return fast — the actual command runs on mqtt_cmd_task so a
             * slow action (e.g. printing) never blocks the MQTT event task.
             * Single-message commands only (payload <= MQTT buffer size). */
            if (s_cmd_queue && event->data_len > 0 &&
                event->data_len == event->total_data_len) {
                mqtt_msg_t cmd;
                int n = event->data_len;
                if (n > MQTT_MSG_MAX) {
                    n = MQTT_MSG_MAX;   /* guard oversized payloads */
                }
                cmd.len = n;
                memcpy(cmd.data, event->data, (size_t)n);
                if (xQueueSend(s_cmd_queue, &cmd, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Command queue full, dropped incoming command");
                }
            } else if (event->data_len != event->total_data_len) {
                ESP_LOGW(TAG, "Chunked command payload not supported (len=%d/%d)",
                         event->data_len, event->total_data_len);
            }
            break;
        case MQTT_EVENT_PUBLISHED:
            /* Broker ACKed a QoS-1 publish. If it's the store-and-forward
             * record the flush loop is waiting on, signal it to delete it. */
            if (s_sf_evt && event->msg_id == s_sf_pending_msgid) {
                xEventGroupSetBits(s_sf_evt, SF_ACK_BIT);
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGW(TAG, "MQTT error event");
            break;
        default:
            break;
    }
}

/* Outgoing publisher task (Task 2 gives it work) */
static void mqtt_task(void *arg)
{
    (void)arg;
    mqtt_msg_t msg;
    static char sfbuf[MQTT_MSG_MAX];
    static char sfpath[48];
    bool client_started = false;

    while (1) {
        /* Start the MQTT client once the network link comes up (once), so we
         * don't spam DNS/connect errors while it is still coming up. Which
         * transport that is - WiFi here, GSM in the GSM product - is not this
         * layer's business; net_link answers only "is it usable now?".
         * IMPORTANT: this is a non-blocking check - the queue drain below runs
         * from boot so an offline reading is written to FLASH within one loop,
         * never left in the RAM queue (power-loss safe, and the 8-deep queue
         * can't overflow during a long outage). */
        if (!client_started && !s_suspended && net_link_is_up()) {
            esp_err_t err = esp_mqtt_client_start(s_client);
            ESP_LOGI(TAG, "MQTT client started (%s)", esp_err_to_name(err));
            client_started = true;
        }

        /* Take a reading, or wake every 500 ms to flush the offline backlog. */
        if (xQueueReceive(s_out_queue, &msg, pdMS_TO_TICKS(500)) == pdTRUE) {
            /* Store-and-forward currently buffers MA readings ONLY. WM is a
             * continuous stream (many near-identical ticks) - buffering every
             * tick would thrash flash, so WM offline handling is deferred
             * pending a team decision (see roadmap: WM store-and-forward).
             * WM: publish if online, else drop (BLE + console still deliver). */
            bool is_ma = (msg.len >= 14 &&
                          memcmp(msg.data, "{\"device\":\"ma\"", 14) == 0);

            /* Buffer only when the cloud is actually wanted here.
             *
             * net_link_is_provisioned() is "have credentials been entered",
             * NOT "is the link up". A site whose WiFi is merely down must keep
             * buffering - that outage is exactly what the buffer is for. A site
             * that was never given an SSID is app-only, and buffering there
             * would fill the ~2800-record buffer over a few weeks and then
             * evict in a loop for nothing.
             *
             * config_get_store_forward() stays as a manual override for a site
             * that needs the opposite of the default.
             *
             * When not buffering, the else-branch below publishes if the broker
             * is up and drops if it is not - the app already has the reading. */
            /* Store only while the LINK itself is healthy.
             *
             * net_link_is_up() is the whole six-stage GSM chain in one call -
             * modem, SIM, signal, registration, IP, traffic. If it is up and we
             * are here, the reading could not be delivered for some other
             * reason (broker down, certificate, credentials), and it must be
             * kept until the broker returns.
             *
             * If the link is DOWN, ble_data_mode=auto has already opened the
             * BLE feed and the app is receiving the reading, exactly as the
             * pre-GSM CLV4 behaved: the connector hands the reading to the app
             * and the app owns delivery from there. Buffering as well would
             * have the server receive the same reading twice - once forwarded
             * by the app, once flushed from flash when the SIM is replaced -
             * and over a long outage that is the whole ~2500-record buffer.
             *
             * KNOWN GAP, accepted deliberately: with the link down and NO phone
             * listening, the reading is neither sent nor stored. Revisit if
             * field use shows readings taken with no app connected. */
            if (is_ma && sf_available() && config_get_store_forward() &&
                net_link_is_provisioned() && net_link_is_up()) {
                /* EVERY MA reading goes to flash first - online or not - and is
                 * delivered by the flush loop below, which removes the file only
                 * after the broker's PUBACK.
                 *
                 * Publishing directly when "online" used to skip that guarantee:
                 * esp_mqtt_client_publish() can fail (or be accepted into an
                 * outbox that never drains) on a link that LOOKS up but is dead,
                 * and the reading was then neither retried nor buffered - lost,
                 * while the log claimed success. Milk readings are payment data,
                 * so the flash round-trip is worth it: one delivery path, and a
                 * record only disappears once the broker has acknowledged it. */
                sf_buffer(msg.data, msg.len);

                /* Warn once when the buffer is nearly full, so the operator and
                 * the server hear about it BEFORE the oldest readings start
                 * being evicted. Hysteresis (90% up / 80% down) stops it
                 * chattering around the threshold. */
                sf_info_t si;
                sf_get_info(&si);
                if (!s_buffer_warned && si.used_pct >= 90) {
                    s_buffer_warned = true;
                    char warn[160];
                    int n = snprintf(warn, sizeof(warn),
                                     "{\"response_message\":\"buffer_nearly_full\",\"status_code\":-1,"
                                     "\"data\":{\"used_pct\":%u,\"records\":%lu,\"capacity_left\":%lu}}",
                                     (unsigned)si.used_pct, (unsigned long)si.records,
                                     (unsigned long)si.capacity_left);
                    ESP_LOGW(TAG, "Offline buffer %u%% full (%lu readings, ~%lu left) - oldest will be evicted",
                             (unsigned)si.used_pct, (unsigned long)si.records,
                             (unsigned long)si.capacity_left);
                    mqtt_svc_publish_resp(warn, n);
                } else if (s_buffer_warned && si.used_pct < 80) {
                    s_buffer_warned = false;    /* drained -> re-arm the warning */
                }
            } else {
                /* WM (not buffered - a continuous stream would thrash flash), or
                 * store-and-forward unavailable: publish if online, else drop.
                 * The return IS checked here so the counters tell the truth. */
                if (s_connected && s_client) {
                    int mid = esp_mqtt_client_publish(s_client, s_data_topic,
                                                      msg.data, msg.len, 1, 0);
                    if (mid >= 0) {
                        s_published++;
                        ESP_LOGI(TAG, "Published reading #%lu (%d bytes) -> %s",
                                 (unsigned long)s_published, msg.len, s_data_topic);
                    } else {
                        s_dropped++;
                        ESP_LOGW(TAG, "Publish REJECTED by client, dropped reading (dropped=%lu)",
                                 (unsigned long)s_dropped);
                    }
                } else if (is_ma && !net_link_is_up()) {
                    /* Link down: the BLE feed is open and the app has this
                     * reading (see the buffering comment above). Not a loss, so
                     * it is counted separately and logged calmly - a "dropped"
                     * warning here would have the operator chasing a fault that
                     * is really the designed hand-off to the app. */
                    s_to_app++;
                    ESP_LOGI(TAG, "Link down - reading handed to the app over BLE (app=%lu)",
                             (unsigned long)s_to_app);
                } else {
                    s_dropped++;
                    ESP_LOGW(TAG, "Broker offline, dropped reading (dropped=%lu)",
                             (unsigned long)s_dropped);
                }
            }
        }

        /* Flush ONE buffered record per loop when connected (keeps the queue
         * drained between flushes). Delete only after the broker's PUBACK. */
        if (s_connected && s_client && s_sf_evt && sf_count() > 0) {
            int len = 0;
            if (sf_oldest(sfbuf, sizeof(sfbuf), &len, sfpath, sizeof(sfpath))) {
                xEventGroupClearBits(s_sf_evt, SF_ACK_BIT);
                int mid = esp_mqtt_client_publish(s_client, s_data_topic, sfbuf, len, 1, 0);
                if (mid >= 0) {
                    s_sf_pending_msgid = mid;
                    EventBits_t b = xEventGroupWaitBits(s_sf_evt, SF_ACK_BIT,
                                                        pdTRUE, pdTRUE, pdMS_TO_TICKS(5000));
                    s_sf_pending_msgid = -1;
                    if (b & SF_ACK_BIT) {
                        sf_delete(sfpath);          /* confirmed -> remove from flash */
                        s_published++;
                        ESP_LOGI(TAG, "Flushed %s (published=%lu, buffered left=%lu)",
                                 sfpath, (unsigned long)s_published,
                                 (unsigned long)sf_count());
                    } else {
                        ESP_LOGW(TAG, "Flush ACK timeout for %s - retry later", sfpath);
                    }
                }
                /* mid < 0 -> publish failed (disconnected); retry next loop */
            }
        }
    }
}

/* Incoming command processor. Runs OFF the MQTT event task, so executing a
 * command (which may print, reconfigure a UART, etc.) can take as long as it
 * needs without blocking the MQTT connection. Delivers each command to the
 * registered callback (main.c -> parse_and_process_commands). */
static void mqtt_cmd_task(void *arg)
{
    (void)arg;
    mqtt_msg_t cmd;
    while (1) {
        if (xQueueReceive(s_cmd_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            if (s_cmd_cb) {
                ESP_LOGI(TAG, "Command from cloud (%d bytes)", cmd.len);
                s_cmd_cb(cmd.data, cmd.len);
            }
        }
    }
}

esp_err_t mqtt_svc_start(void)
{
    build_ids();

    /* Mount the offline store-and-forward buffer (LittleFS on `storage`) and
     * create the event group used to confirm flush publishes. */
    sf_init();
    s_sf_evt = xEventGroupCreate();

    s_out_queue = xQueueCreate(MQTT_QUEUE_LEN, sizeof(mqtt_msg_t));
    if (!s_out_queue) {
        ESP_LOGE(TAG, "queue create failed");
        return ESP_FAIL;
    }

    s_cmd_queue = xQueueCreate(MQTT_CMD_QUEUE_LEN, sizeof(mqtt_msg_t));
    if (!s_cmd_queue) {
        ESP_LOGE(TAG, "cmd queue create failed");
        return ESP_FAIL;
    }

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri     = NCLE_MQTT_URI,
        .credentials.client_id  = s_client_id,
        .session.keepalive      = 60,
        /* NOTE: Persistent session (.session.disable_clean_session = true) is
         * deferred to Stage 3 - its effect (broker queuing commands during an
         * outage) needs the internal broker to honour it and can't be tested
         * reliably on the public broker. Enabled there, alongside TLS. */
        .session.last_will = {
            .topic  = s_status_topic,
            .msg    = STATUS_OFFLINE_JSON,
            .msg_len = 0,       /* 0 = strlen */
            .qos    = 1,
            .retain = 1,
        },
        /* 2 KB RX/TX buffer so a full receipt arrives in ONE message (no
         * chunking to reassemble). Commands/receipts are well under this. */
        .buffer.size = 2048,
        /* TLS trust anchor for mqtts:// URIs: the ESP-IDF certificate bundle
         * (same one FOTA uses for HTTPS) validates any broker with a public-CA
         * certificate - HiveMQ Cloud (Let's Encrypt), AWS, Azure, etc. Ignored
         * for plain mqtt:// . A broker with a PRIVATE CA needs its CA PEM here
         * instead (.broker.verification.certificate). */
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
    };

    /* Username/password auth - only sent when a username is configured, so
     * anonymous brokers (public test broker) are unaffected. */
    if (sizeof(NCLE_MQTT_USER) > 1) {
        cfg.credentials.username = NCLE_MQTT_USER;
        cfg.credentials.authentication.password = NCLE_MQTT_PASS;
        ESP_LOGI(TAG, "Broker auth: user '%s'", NCLE_MQTT_USER);
    } else {
        ESP_LOGI(TAG, "Broker auth: anonymous");
    }

    s_client = esp_mqtt_client_init(&cfg);
    if (!s_client) {
        ESP_LOGE(TAG, "client init failed");
        return ESP_FAIL;
    }
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    /* The task waits for the network link, then starts the client (see mqtt_task). */
    /* 8192: the task keeps a ~1.2KB mqtt_msg_t on its stack plus the esp-mqtt
     * publish path; 4096 overflowed during a QoS1 flush burst (HW-seen 2026-07-22). */
    if (xTaskCreate(mqtt_task, "mqtt_svc", 8192, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        return ESP_FAIL;
    }
    /* Command processor (runs commands off the MQTT event task). Larger stack
     * because it runs the full command parser (cJSON + printer + response). */
    if (xTaskCreate(mqtt_cmd_task, "mqtt_cmd", 8192, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "cmd task create failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "MQTT service started (will connect once the link is up)");
    return ESP_OK;
}

/**
 * Hand a copy of an already-formed reading JSON to the MQTT queue.
 * NON-BLOCKING: called from the WM/MA data callback, so it must never stall
 * the data path. If the queue is full (MQTT slow/offline) the reading is
 * dropped for MQTT only — BLE + console have already delivered it.
 */
void mqtt_svc_publish_data(const char *json, int len)
{
    if (!s_out_queue || !json || len <= 0) {
        return;
    }

    /* Continuous-meter de-dup (see s_last_ma note): skip a byte-identical
     * repeat of the last MA reading we accepted. Only MA messages are deduped;
     * anything else passes straight through. Done here - before the WM merge -
     * so a skipped duplicate incurs no WM-grace delay and never touches flash. */
    if (len >= MA_PREFIX_LEN && memcmp(json, MA_PREFIX, MA_PREFIX_LEN) == 0) {
        if (len == s_last_ma_len && (size_t)len <= sizeof(s_last_ma) &&
            memcmp(json, s_last_ma, len) == 0) {
            s_deduped++;
            return;                         /* identical repeat -> skip silently */
        }
        if ((size_t)len <= sizeof(s_last_ma)) {
            memcpy(s_last_ma, json, len);   /* new reading -> new baseline */
            s_last_ma_len = len;
        } else {
            s_last_ma_len = 0;              /* too big to cache -> never dedup it */
        }
    }

    mqtt_msg_t msg;
    /* Merge this transaction's settled WM weight into MA messages (the single
     * cloud-format function in wm_capture). Non-MA messages are copied verbatim.
     * Output fits msg.data (MQTT_MSG_MAX). */
    int n = wm_capture_merge_into_ma(json, len, msg.data, sizeof(msg.data));
    if (n <= 0) {
        return;
    }
    msg.len = n;

    if (xQueueSend(s_out_queue, &msg, 0) != pdTRUE) {
        s_dropped++;          /* queue full -> drop (never block the caller) */
        ESP_LOGW(TAG, "Queue full, dropped reading (total dropped=%lu)",
                 (unsigned long)s_dropped);
    }
}

/**
 * Publish a command reply/ack to clv4/<id>/resp. Best-effort: if the broker is
 * not connected the reply is simply not sent (BLE + console already have it).
 * Called from a normal task (mqtt_cmd_task or the BLE/console task via the
 * combined output callback), never from the MQTT event handler.
 */
void mqtt_svc_publish_resp(const char *json, int len)
{
    if (s_client && s_connected && json && len > 0) {
        esp_mqtt_client_publish(s_client, s_resp_topic, json, len, 1, 0);
    }
}

#else /* !CONFIG_NCLE_MQTT_ENABLE */
/* ==== MQTT disabled: no-op stubs ========================================= */
/* Callers already guard on CONFIG_NCLE_MQTT_ENABLE; these keep the component
 * compiling and linking so the firmware behaves identically to a non-MQTT
 * build. */

esp_err_t mqtt_svc_start(void) { return ESP_OK; }
bool mqtt_svc_is_connected(void) { return false; }
const char *mqtt_svc_broker_uri(void) { return "disabled"; }
void mqtt_svc_suspend(void) { }
void mqtt_svc_resume(void) { }
void mqtt_svc_get_stats(mqtt_svc_stats_t *out)
{
    if (out) {
        out->connected = false; out->published = 0;
        out->dropped = 0; out->deduped = 0; out->buffered = 0;
    }
}
void mqtt_svc_publish_data(const char *json, int len) { (void)json; (void)len; }
void mqtt_svc_publish_resp(const char *json, int len) { (void)json; (void)len; }
void mqtt_svc_set_cmd_callback(mqtt_cmd_cb_t cb) { (void)cb; }

#endif /* CONFIG_NCLE_MQTT_ENABLE */
