#include "gsm.h"

#ifdef CONFIG_NCLE_GSM_ENABLE

#include <string.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

static const char *TAG = "GSM";

#define GSM_UART_NUM        CONFIG_NCLE_GSM_UART_NUM
#define GSM_UART_TX_PIN     CONFIG_NCLE_GSM_UART_TX_PIN
#define GSM_UART_RX_PIN     CONFIG_NCLE_GSM_UART_RX_PIN
#define GSM_PWRKEY_PIN      CONFIG_NCLE_GSM_PWRKEY_PIN
#define GSM_RST_PIN         CONFIG_NCLE_GSM_RST_PIN
#define GSM_UART_BAUD_RATE  CONFIG_NCLE_GSM_UART_BAUD_RATE

#define GSM_RX_BUFFER_SIZE  2048
#define GSM_TX_BUFFER_SIZE  1024

/* Reset polarity: by default Conn_plus board has an inverter on RESET,
 * so HIGH = assert, LOW = release. Direct (datasheet-spec) boards disable
 * the flag in menuconfig. */
#ifdef CONFIG_NCLE_GSM_RST_INVERTED
#define GSM_RST_ASSERT      1
#define GSM_RST_RELEASE     0
#else
#define GSM_RST_ASSERT      0
#define GSM_RST_RELEASE     1
#endif

static bool s_initialized = false;
static SemaphoreHandle_t s_uart_mutex = NULL;

static void gsm_gpio_init(void)
{
    gpio_config_t io_conf = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GSM_PWRKEY_PIN) | (1ULL << GSM_RST_PIN),
        .pull_down_en = 0,
        .pull_up_en   = 0,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(GSM_RST_PIN, GSM_RST_RELEASE);
    gpio_set_level(GSM_PWRKEY_PIN, 1);

    ESP_LOGI(TAG, "GPIO: PWRKEY=%d, RST=%d (inverted=%d)",
             GSM_PWRKEY_PIN, GSM_RST_PIN,
#ifdef CONFIG_NCLE_GSM_RST_INVERTED
             1
#else
             0
#endif
            );
}

static void gsm_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate  = GSM_UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(GSM_UART_NUM,
                                        GSM_RX_BUFFER_SIZE,
                                        GSM_TX_BUFFER_SIZE,
                                        0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GSM_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GSM_UART_NUM,
                                 GSM_UART_TX_PIN, GSM_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART: TX=%d, RX=%d, Baud=%d",
             GSM_UART_TX_PIN, GSM_UART_RX_PIN, GSM_UART_BAUD_RATE);
}

esp_err_t gsm_power_on(void)
{
    ESP_LOGI(TAG, "Power ON sequence...");

    vTaskDelay(pdMS_TO_TICKS(500));

    gpio_set_level(GSM_PWRKEY_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(600));
    gpio_set_level(GSM_PWRKEY_PIN, 1);

    ESP_LOGI(TAG, "Waiting for module boot...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    return ESP_OK;
}

esp_err_t gsm_power_off(void)
{
    ESP_LOGI(TAG, "Power OFF sequence...");

    gpio_set_level(GSM_PWRKEY_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(800));
    gpio_set_level(GSM_PWRKEY_PIN, 1);

    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Power OFF complete");

    return ESP_OK;
}

esp_err_t gsm_reset(void)
{
    ESP_LOGI(TAG, "Hardware RESET (assert=%d, release=%d)...",
             GSM_RST_ASSERT, GSM_RST_RELEASE);

    gpio_set_level(GSM_RST_PIN, GSM_RST_ASSERT);
    vTaskDelay(pdMS_TO_TICKS(100));

    gpio_set_level(GSM_RST_PIN, GSM_RST_RELEASE);
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "RESET complete");
    return ESP_OK;
}

static int gsm_read_response(char *buffer, size_t buffer_size, uint32_t timeout_ms)
{
    int total = 0;
    TickType_t start = xTaskGetTickCount();

    while (total < (int)(buffer_size - 1)) {
        TickType_t elapsed = (xTaskGetTickCount() - start) * portTICK_PERIOD_MS;
        if (elapsed >= timeout_ms) break;

        int len = uart_read_bytes(GSM_UART_NUM,
                                  (uint8_t *)(buffer + total),
                                  buffer_size - 1 - total,
                                  pdMS_TO_TICKS(200));
        if (len > 0) {
            total += len;
        } else {
            break;
        }
    }
    buffer[total] = '\0';
    return total;
}

static bool gsm_check_at_response(void)
{
    char buf[128] = {0};
    bool result = false;

    bool have_mutex = (s_uart_mutex != NULL);
    if (have_mutex) {
        if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
            ESP_LOGW(TAG, "Could not acquire mutex for AT check");
            return false;
        }
    }

    uart_flush_input(GSM_UART_NUM);
    uart_write_bytes(GSM_UART_NUM, "AT\r\n", 4);
    vTaskDelay(pdMS_TO_TICKS(500));

    int len = gsm_read_response(buf, sizeof(buf), 1000);
    if (len > 0) {
        ESP_LOGI(TAG, "RX: %s", buf);
        result = (strstr(buf, "OK") != NULL);
    }

    if (have_mutex) xSemaphoreGive(s_uart_mutex);
    return result;
}

bool gsm_is_alive(void)
{
    return gsm_check_at_response();
}

esp_err_t gsm_send_at_command(const char *command, char *response,
                              size_t response_size, uint32_t timeout_ms)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (command == NULL) return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire UART mutex");
        return ESP_ERR_TIMEOUT;
    }

    uart_flush_input(GSM_UART_NUM);   /* clear stale RX before sending */

    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT%s\r\n", command);
    ESP_LOGI(TAG, "TX: AT%s", command);
    uart_write_bytes(GSM_UART_NUM, cmd, strlen(cmd));

    esp_err_t ret = ESP_OK;

    if (response && response_size > 0) {
        vTaskDelay(pdMS_TO_TICKS(100));
        int len = gsm_read_response(response, response_size, timeout_ms);
        if (len > 0) {
            ESP_LOGI(TAG, "RX: %s", response);
        } else {
            ESP_LOGW(TAG, "RX: No response");
            ret = ESP_ERR_TIMEOUT;
        }
    }

    xSemaphoreGive(s_uart_mutex);
    return ret;
}

esp_err_t gsm_init(void)
{
    if (s_initialized) return ESP_OK;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "GSM Module Initialization");
    ESP_LOGI(TAG, "========================================");

    if (s_uart_mutex == NULL) {
        s_uart_mutex = xSemaphoreCreateMutex();
        if (s_uart_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create UART mutex");
            return ESP_FAIL;
        }
    }

    gsm_gpio_init();
    gsm_uart_init();

    ESP_LOGI(TAG, "Waiting 2s for stabilization...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Probe-1: module already running? */
    ESP_LOGI(TAG, "Checking if module already running...");
    for (int i = 0; i < 3; i++) {
        if (gsm_check_at_response()) {
            ESP_LOGI(TAG, "Module already ON!");
            s_initialized = true;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Probe-2: try power-on */
    ESP_LOGI(TAG, "Module not responding, trying power-on...");
    gsm_power_on();

    for (int i = 0; i < 5; i++) {
        ESP_LOGI(TAG, "AT test %d/5 after power-on", i + 1);
        if (gsm_check_at_response()) {
            ESP_LOGI(TAG, "Module responding after power-on!");
            s_initialized = true;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Probe-3: try hardware reset */
    ESP_LOGW(TAG, "Still no response, trying hardware reset...");
    gsm_reset();

    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TAG, "AT test %d/3 after reset", i + 1);
        if (gsm_check_at_response()) {
            ESP_LOGI(TAG, "Module responding after reset!");
            s_initialized = true;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGE(TAG, "========================================");
    ESP_LOGE(TAG, "FAILED to communicate with module!");
    ESP_LOGE(TAG, "Check wiring:");
    ESP_LOGE(TAG, "  ESP TX  (GPIO%d) -> Module RX",     GSM_UART_TX_PIN);
    ESP_LOGE(TAG, "  ESP RX  (GPIO%d) -> Module TX",     GSM_UART_RX_PIN);
    ESP_LOGE(TAG, "  ESP PWR (GPIO%d) -> Module PWRKEY", GSM_PWRKEY_PIN);
    ESP_LOGE(TAG, "  ESP RST (GPIO%d) -> Module RESET",  GSM_RST_PIN);
    ESP_LOGE(TAG, "  Common GND, Module power: 3.8-4.2V");
    ESP_LOGE(TAG, "  If reset doesn't pulse, try flipping NCLE_GSM_RST_INVERTED");
    ESP_LOGE(TAG, "========================================");

    uart_driver_delete(GSM_UART_NUM);
    if (s_uart_mutex) {
        vSemaphoreDelete(s_uart_mutex);
        s_uart_mutex = NULL;
    }
    return ESP_FAIL;
}

esp_err_t gsm_deinit(void)
{
    if (!s_initialized) return ESP_OK;

    gsm_power_off();
    uart_driver_delete(GSM_UART_NUM);
    s_initialized = false;

    if (s_uart_mutex) {
        vSemaphoreDelete(s_uart_mutex);
        s_uart_mutex = NULL;
    }
    return ESP_OK;
}

esp_err_t gsm_get_module_info(char *info, size_t info_size)
{
    if (info == NULL || info_size == 0) return ESP_ERR_INVALID_ARG;

    char response[256];
    if (gsm_send_at_command("I", response, sizeof(response), 2000) != ESP_OK) {
        return ESP_FAIL;
    }

    char *start = strchr(response, '\n');
    if (start) {
        start++;
        char *end = strstr(start, "\r\n");
        if (end) {
            size_t len = end - start;
            if (len < info_size) {
                memcpy(info, start, len);
                info[len] = '\0';
                return ESP_OK;
            }
        }
    }
    strncpy(info, response, info_size - 1);
    info[info_size - 1] = '\0';
    return ESP_OK;
}

esp_err_t gsm_get_signal_strength(uint8_t *rssi, uint8_t *ber)
{
    if (rssi == NULL || ber == NULL) return ESP_ERR_INVALID_ARG;

    char response[64];
    if (gsm_send_at_command("+CSQ", response, sizeof(response), 2000) != ESP_OK) {
        return ESP_FAIL;
    }

    char *p = strstr(response, "+CSQ:");
    if (p && sscanf(p, "+CSQ: %hhu,%hhu", rssi, ber) == 2) {
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t gsm_get_network_status(gsm_network_status_t *status)
{
    if (status == NULL) return ESP_ERR_INVALID_ARG;

    char response[64];
    if (gsm_send_at_command("+CREG?", response, sizeof(response), 2000) != ESP_OK) {
        return ESP_FAIL;
    }

    char *p = strstr(response, "+CREG:");
    int n, stat;
    if (p && sscanf(p, "+CREG: %d,%d", &n, &stat) == 2) {
        *status = (gsm_network_status_t)stat;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t gsm_get_phone_number(char *number, size_t number_size)
{
    if (number == NULL || number_size == 0) return ESP_ERR_INVALID_ARG;
    number[0] = '\0';

    char response[128];
    if (gsm_send_at_command("+CNUM", response, sizeof(response), 2000) != ESP_OK) {
        return ESP_FAIL;
    }

    /* Format: +CNUM: "<alpha>","<number>",<type>
     * Examples: +CNUM: "","+918123456789",145
     *           +CNUM: "","918123456789",129
     * If the SIM has no MSISDN, response is just "OK" with no +CNUM line. */
    char *p = strstr(response, "+CNUM:");
    if (p == NULL) {
        ESP_LOGW(TAG, "CNUM: SIM has no MSISDN provisioned");
        return ESP_FAIL;
    }

    /* Skip the first quoted field (alpha tag), find the second quoted field (number) */
    char *first_comma = strchr(p, ',');
    if (first_comma == NULL) return ESP_FAIL;
    char *num_start = strchr(first_comma, '"');
    if (num_start == NULL) return ESP_FAIL;
    num_start++;
    char *num_end = strchr(num_start, '"');
    if (num_end == NULL) return ESP_FAIL;

    size_t len = num_end - num_start;
    if (len >= number_size) len = number_size - 1;
    memcpy(number, num_start, len);
    number[len] = '\0';
    return ESP_OK;
}

esp_err_t gsm_get_iccid(char *iccid, size_t iccid_size)
{
    if (iccid == NULL || iccid_size == 0) return ESP_ERR_INVALID_ARG;
    iccid[0] = '\0';

    char response[128];
    if (gsm_send_at_command("+QCCID", response, sizeof(response), 2000) != ESP_OK) {
        return ESP_FAIL;
    }

    /* Format: +QCCID: <iccid_digits>
     * Example: +QCCID: 8991100123456789012F     */
    char *p = strstr(response, "+QCCID:");
    if (p == NULL) return ESP_FAIL;

    /* Skip "+QCCID:" prefix and any whitespace */
    p += strlen("+QCCID:");
    while (*p == ' ' || *p == '\t') p++;

    /* Read digits/hex chars until \r or \n */
    size_t i = 0;
    while (*p && *p != '\r' && *p != '\n' && i < iccid_size - 1) {
        iccid[i++] = *p++;
    }
    iccid[i] = '\0';
    return (i > 0) ? ESP_OK : ESP_FAIL;
}

/* ============================================================================
 * Data session (PDP context) + HTTP — verification helpers
 * ============================================================================
 * These functions use longer timeouts (PDP activation can take 5-15 seconds)
 * and a multi-stage AT flow (CONNECT prompt + raw data upload).
 * ===========================================================================*/

static bool s_pdp_active = false;

/* Read until a needle string appears, or timeout. Returns total bytes read. */
static int gsm_read_until(const char *needle, char *buf, size_t bufsz, uint32_t timeout_ms)
{
    int total = 0;
    TickType_t start = xTaskGetTickCount();
    size_t needle_len = strlen(needle);

    while (total < (int)(bufsz - 1)) {
        TickType_t elapsed = (xTaskGetTickCount() - start) * portTICK_PERIOD_MS;
        if (elapsed >= timeout_ms) break;

        int len = uart_read_bytes(GSM_UART_NUM,
                                  (uint8_t *)(buf + total),
                                  bufsz - 1 - total,
                                  pdMS_TO_TICKS(200));
        if (len > 0) {
            total += len;
            buf[total] = '\0';
            if (total >= (int)needle_len && strstr(buf, needle) != NULL) {
                return total;
            }
        }
    }
    buf[total] = '\0';
    return total;
}

esp_err_t gsm_pdp_activate(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    char cmd[128];
    char resp[256];

    /* Step 1: Check if context 1 is already active (e.g. from a previous
     * session that left the modem powered with PDP up). The modem persists
     * PDP state across ESP reboots if it stays powered.
     *
     * Expected format when active:
     *   +QIACT: <ctx>,<state>,<ctx_type>,"<ip>"
     * where state=1 means activated. */
    if (gsm_send_at_command("+QIACT?", resp, sizeof(resp), 2000) == ESP_OK) {
        if (strstr(resp, "+QIACT: 1,1,") != NULL) {
            ESP_LOGI(TAG, "PDP context 1 already active, reusing");
            s_pdp_active = true;
            return ESP_OK;
        }
    }

    /* Step 2: Best-effort cleanup of any half-state on context 1.
     * If it was never active this is a no-op; if it was in a stale state
     * this clears it. We ignore the result either way. */
    gsm_send_at_command("+QIDEACT=1", resp, sizeof(resp), 5000);

    /* Step 3: Configure PDP context 1: IPv4, APN, no auth */
    snprintf(cmd, sizeof(cmd), "+QICSGP=1,1,\"%s\",\"\",\"\",1", CONFIG_NCLE_GSM_APN);
    ESP_LOGI(TAG, "Configuring PDP context with APN=%s", CONFIG_NCLE_GSM_APN);
    if (gsm_send_at_command(cmd, resp, sizeof(resp), 2000) != ESP_OK) {
        ESP_LOGE(TAG, "QICSGP: no response");
        return ESP_FAIL;
    }
    if (strstr(resp, "OK") == NULL) {
        ESP_LOGE(TAG, "QICSGP returned: %s", resp);
        return ESP_FAIL;
    }

    /* Step 4: Activate PDP context (can take up to 150s per Quectel doc; cap at 30s) */
    ESP_LOGI(TAG, "Activating PDP context (up to 30s)...");
    if (gsm_send_at_command("+QIACT=1", resp, sizeof(resp), 30000) != ESP_OK) {
        ESP_LOGE(TAG, "QIACT: no response");
        return ESP_FAIL;
    }
    if (strstr(resp, "OK") == NULL) {
        ESP_LOGE(TAG, "QIACT did not return OK: %s", resp);
        return ESP_FAIL;
    }

    /* Step 5: Confirm with IP query */
    if (gsm_send_at_command("+QIACT?", resp, sizeof(resp), 2000) == ESP_OK) {
        ESP_LOGI(TAG, "PDP active: %s", resp);
    }

    s_pdp_active = true;
    return ESP_OK;
}

esp_err_t gsm_pdp_deactivate(void)
{
    if (!s_initialized || !s_pdp_active) return ESP_OK;
    char resp[64];
    gsm_send_at_command("+QIDEACT=1", resp, sizeof(resp), 5000);
    s_pdp_active = false;
    return ESP_OK;
}

/* Configure HTTP context. Detects HTTPS from URL scheme and sets SSL appropriately. */
static esp_err_t gsm_http_configure(bool is_https)
{
    char resp[128];

    if (gsm_send_at_command("+QHTTPCFG=\"contextid\",1", resp, sizeof(resp), 2000) != ESP_OK)
        return ESP_FAIL;
    if (gsm_send_at_command("+QHTTPCFG=\"responseheader\",0", resp, sizeof(resp), 2000) != ESP_OK)
        return ESP_FAIL;

    if (is_https) {
        if (gsm_send_at_command("+QHTTPCFG=\"sslctxid\",1", resp, sizeof(resp), 2000) != ESP_OK)
            return ESP_FAIL;
        /* TLS 1.2 */
        if (gsm_send_at_command("+QSSLCFG=\"sslversion\",1,4", resp, sizeof(resp), 2000) != ESP_OK)
            return ESP_FAIL;
        /* All ciphers */
        if (gsm_send_at_command("+QSSLCFG=\"ciphersuite\",1,0xFFFF", resp, sizeof(resp), 2000) != ESP_OK)
            return ESP_FAIL;
#ifdef CONFIG_NCLE_GSM_HTTP_INSECURE
        /* seclevel=0: no certificate verification (testing only) */
        if (gsm_send_at_command("+QSSLCFG=\"seclevel\",1,0", resp, sizeof(resp), 2000) != ESP_OK)
            return ESP_FAIL;
#else
        /* seclevel=1: verify server cert against installed CA */
        if (gsm_send_at_command("+QSSLCFG=\"seclevel\",1,1", resp, sizeof(resp), 2000) != ESP_OK)
            return ESP_FAIL;
#endif
    }
    return ESP_OK;
}

/* Set the URL for the next HTTP operation.
 * QHTTPURL flow: AT+QHTTPURL=<len>,80 -> "CONNECT" -> raw URL bytes -> "OK". */
static esp_err_t gsm_http_set_url(const char *url)
{
    char cmd[64];
    char buf[512];
    int url_len = (int)strlen(url);

    snprintf(cmd, sizeof(cmd), "AT+QHTTPURL=%d,80\r\n", url_len);
    ESP_LOGI(TAG, "TX: AT+QHTTPURL=%d,80 (url='%s')", url_len, url);
    uart_flush_input(GSM_UART_NUM);    /* clear stale RX bytes */
    uart_write_bytes(GSM_UART_NUM, cmd, strlen(cmd));

    /* Wait for CONNECT prompt (up to 5s) */
    int len = gsm_read_until("CONNECT", buf, sizeof(buf), 5000);
    if (len <= 0 || strstr(buf, "CONNECT") == NULL) {
        ESP_LOGE(TAG, "QHTTPURL: no CONNECT (got %d bytes: %s)", len, buf);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "QHTTPURL got CONNECT, sending %d URL bytes", url_len);

    /* Send the URL bytes */
    uart_write_bytes(GSM_UART_NUM, url, url_len);

    /* Wait for OK (give modem time to validate URL — up to 10s) */
    len = gsm_read_until("OK", buf, sizeof(buf), 10000);
    if (len <= 0 || strstr(buf, "OK") == NULL) {
        ESP_LOGE(TAG, "QHTTPURL: no OK after URL (got %d bytes: %s)", len, buf);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "QHTTPURL OK (response: %s)", buf);
    return ESP_OK;
}

/* Execute POST. body may be NULL (then a 0-length body is sent — useful for GET-like POST).
 * QHTTPPOST flow: AT+QHTTPPOST=<len>,80,80 -> "CONNECT" -> body bytes -> "+QHTTPPOST: <err>,<http>,<rxlen>" */
static esp_err_t gsm_http_do_post(const char *body, int *http_code_out)
{
    char cmd[64];
    char buf[1024];
    int body_len = body ? (int)strlen(body) : 0;

    snprintf(cmd, sizeof(cmd), "AT+QHTTPPOST=%d,80,80\r\n", body_len);
    ESP_LOGI(TAG, "TX: AT+QHTTPPOST=%d,80,80 (body_len=%d)", body_len, body_len);
    uart_flush_input(GSM_UART_NUM);
    uart_write_bytes(GSM_UART_NUM, cmd, strlen(cmd));

    int len = gsm_read_until("CONNECT", buf, sizeof(buf), 5000);
    if (len <= 0 || strstr(buf, "CONNECT") == NULL) {
        ESP_LOGE(TAG, "QHTTPPOST: no CONNECT (got %d bytes: %s)", len, buf);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "QHTTPPOST got CONNECT, sending %d body bytes", body_len);

    if (body_len > 0) {
        uart_write_bytes(GSM_UART_NUM, body, body_len);
    }

    /* Wait for the +QHTTPPOST: line within 80s (server response timeout) */
    len = gsm_read_until("+QHTTPPOST:", buf, sizeof(buf), 80000);
    if (len <= 0) {
        ESP_LOGE(TAG, "QHTTPPOST: no response (got %d bytes: %s)", len, buf);
        return ESP_FAIL;
    }
    char *p = strstr(buf, "+QHTTPPOST:");
    int err = -1, http_code = -1, rxlen = -1;
    if (p && sscanf(p, "+QHTTPPOST: %d,%d,%d", &err, &http_code, &rxlen) >= 2) {
        ESP_LOGI(TAG, "QHTTPPOST result: err=%d http=%d len=%d", err, http_code, rxlen);
        if (http_code_out) *http_code_out = http_code;
        return (err == 0) ? ESP_OK : ESP_FAIL;
    }
    ESP_LOGE(TAG, "QHTTPPOST: parse failed (%s)", buf);
    return ESP_FAIL;
}

/* QHTTPGET flow: AT+QHTTPGET=80 -> "+QHTTPGET: <err>,<http>,<rxlen>" */
static esp_err_t gsm_http_do_get(int *http_code_out)
{
    char buf[1024];
    ESP_LOGI(TAG, "TX: AT+QHTTPGET=80");
    uart_flush_input(GSM_UART_NUM);
    const char *cmd = "AT+QHTTPGET=80\r\n";
    uart_write_bytes(GSM_UART_NUM, cmd, strlen(cmd));

    int len = gsm_read_until("+QHTTPGET:", buf, sizeof(buf), 80000);
    if (len <= 0) {
        ESP_LOGE(TAG, "QHTTPGET: no response (got: %s)", buf);
        return ESP_FAIL;
    }
    char *p = strstr(buf, "+QHTTPGET:");
    int err = -1, http_code = -1, rxlen = -1;
    if (p && sscanf(p, "+QHTTPGET: %d,%d,%d", &err, &http_code, &rxlen) >= 2) {
        ESP_LOGI(TAG, "QHTTPGET result: err=%d http=%d len=%d", err, http_code, rxlen);
        if (http_code_out) *http_code_out = http_code;
        return (err == 0) ? ESP_OK : ESP_FAIL;
    }
    ESP_LOGE(TAG, "QHTTPGET: parse failed (%s)", buf);
    return ESP_FAIL;
}

/* Read the response body via QHTTPREAD.
 *
 * Modem response shape:
 *   AT+QHTTPREAD=80\r\n              (echo)
 *   CONNECT\r\n                      (ready prompt)
 *   <body bytes>\r\n
 *   +QHTTPREAD: 0\r\n                (terminator with result code)
 *   OK\r\n
 *
 * Single-pass read until "+QHTTPREAD:" appears, then parse the body
 * out of the accumulated buffer. Avoids the two-step bug where the
 * first read can consume everything, leaving the second read empty. */
static esp_err_t gsm_http_read_body(char *out, size_t out_size)
{
    if (out == NULL || out_size == 0) return ESP_OK;
    out[0] = '\0';

    static char buf[2048];   /* static — keep off task stack */
    ESP_LOGI(TAG, "TX: AT+QHTTPREAD=80");
    uart_flush_input(GSM_UART_NUM);
    const char *cmd = "AT+QHTTPREAD=80\r\n";
    uart_write_bytes(GSM_UART_NUM, cmd, strlen(cmd));

    /* Single read: keep reading until the +QHTTPREAD: terminator appears
     * or we hit a 10s timeout. This captures the entire CONNECT+body+
     * terminator sequence in one buffer. */
    int len = gsm_read_until("+QHTTPREAD:", buf, sizeof(buf), 10000);
    if (len <= 0) {
        ESP_LOGW(TAG, "QHTTPREAD: timeout, no terminator received");
        return ESP_OK;
    }

    /* Locate the body between "CONNECT\r\n" and "\r\n+QHTTPREAD:" */
    char *body_start = strstr(buf, "CONNECT\r\n");
    if (body_start == NULL) {
        ESP_LOGW(TAG, "QHTTPREAD: no CONNECT prompt in response");
        return ESP_OK;
    }
    body_start += strlen("CONNECT\r\n");

    char *body_end = strstr(body_start, "\r\n+QHTTPREAD:");
    if (body_end == NULL || body_end <= body_start) {
        ESP_LOGW(TAG, "QHTTPREAD: empty body");
        return ESP_OK;
    }

    size_t body_len = body_end - body_start;
    if (body_len >= out_size) body_len = out_size - 1;
    memcpy(out, body_start, body_len);
    out[body_len] = '\0';
    ESP_LOGI(TAG, "QHTTPREAD: body=%d bytes", (int)body_len);
    /* Print the actual response body content so we can verify what came
     * from the server. Multi-line — server reply often has \n separators. */
    ESP_LOGI(TAG, "============ Server response begin ============");
    ESP_LOGI(TAG, "%s", out);
    ESP_LOGI(TAG, "============= Server response end =============");
    return ESP_OK;
}

esp_err_t gsm_http_post(const char *url, const char *body,
                        int *http_code_out,
                        char *resp_body, size_t resp_body_size)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (url == NULL)    return ESP_ERR_INVALID_ARG;

    /* Activate PDP if not already (uses mutex internally) */
    if (!s_pdp_active) {
        if (gsm_pdp_activate() != ESP_OK) return ESP_FAIL;
    }

    bool is_https = (strncmp(url, "https://", 8) == 0);
    if (gsm_http_configure(is_https) != ESP_OK) return ESP_FAIL;

    /* The URL upload + POST body upload + response body read ALL need raw
     * UART access. Take the mutex once and hold it through the whole flow,
     * otherwise the periodic gsm_task poll can grab the UART between the
     * POST and the QHTTPREAD, corrupting the response body read. */
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(15000)) != pdTRUE) {
        ESP_LOGE(TAG, "http_post: mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = gsm_http_set_url(url);
    if (ret == ESP_OK) ret = gsm_http_do_post(body, http_code_out);

    /* Read response body BEFORE releasing the mutex */
    if (ret == ESP_OK && resp_body && resp_body_size > 0) {
        gsm_http_read_body(resp_body, resp_body_size);
    }

    xSemaphoreGive(s_uart_mutex);
    return ret;
}

esp_err_t gsm_http_get(const char *url,
                       int *http_code_out,
                       char *resp_body, size_t resp_body_size)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (url == NULL)    return ESP_ERR_INVALID_ARG;

    if (!s_pdp_active) {
        if (gsm_pdp_activate() != ESP_OK) return ESP_FAIL;
    }

    bool is_https = (strncmp(url, "https://", 8) == 0);
    if (gsm_http_configure(is_https) != ESP_OK) return ESP_FAIL;

    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(15000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = gsm_http_set_url(url);
    if (ret == ESP_OK) ret = gsm_http_do_get(http_code_out);

    /* Read response body BEFORE releasing the mutex (same fix as POST) */
    if (ret == ESP_OK && resp_body && resp_body_size > 0) {
        gsm_http_read_body(resp_body, resp_body_size);
    }

    xSemaphoreGive(s_uart_mutex);
    return ret;
}

/* ============================================================================
 * Ping — uses AT+QPING to verify IP reachability (proves data plan works)
 * ===========================================================================*/

/* Returns the LAST "+QPING:" line that has 7 comma-separated fields.
 * That's the SUMMARY line:    +QPING: <result>,<sent>,<rcvd>,<lost>,<min>,<max>,<avg>
 * Per-reply lines have either 6 fields (with quoted IP) or 1 field (timeout). */
static char *gsm_ping_find_summary(char *buf, int buf_len)
{
    char *p = buf;
    char *summary = NULL;
    while ((p = strstr(p, "+QPING:")) != NULL) {
        int r, s, rc, l, mn, mx, avg;
        if (sscanf(p, "+QPING: %d,%d,%d,%d,%d,%d,%d",
                   &r, &s, &rc, &l, &mn, &mx, &avg) == 7) {
            summary = p;
        }
        p++;
    }
    (void)buf_len;
    return summary;
}

esp_err_t gsm_ping(const char *host, uint8_t count, uint16_t timeout_s,
                   gsm_ping_result_t *out)
{
    if (!s_initialized || !out) return ESP_ERR_INVALID_STATE;
    if (host == NULL || host[0] == '\0') host = "8.8.8.8";
    if (count == 0)     count = 4;
    if (timeout_s == 0) timeout_s = 5;

    memset(out, 0, sizeof(*out));
    out->loss_pct = 100;

    if (!s_pdp_active) {
        if (gsm_pdp_activate() != ESP_OK) return ESP_FAIL;
    }

    /* AT+QPING returns "OK" immediately, then emits +QPING: lines asynchronously
     * over the next (count * timeout_s) seconds. We can't use gsm_send_at_command
     * because it returns on the first read gap. Instead, take the mutex and read
     * directly until the summary line appears or the time budget elapses. */
    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
        ESP_LOGE(TAG, "ping: mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    char cmd[128];
    static char resp[2048];   /* static — keep off task stack */
    snprintf(cmd, sizeof(cmd), "AT+QPING=1,\"%s\",%u,%u\r\n",
             host, (unsigned)timeout_s, (unsigned)count);

    uint32_t budget_ms = (uint32_t)count * (uint32_t)timeout_s * 1000u + 5000u;
    ESP_LOGI(TAG, "Pinging %s (%u pings, %us each, total budget %lus)",
             host, count, timeout_s, (unsigned long)(budget_ms / 1000));
    ESP_LOGI(TAG, "TX: AT+QPING=1,\"%s\",%u,%u", host, timeout_s, count);

    uart_flush_input(GSM_UART_NUM);
    uart_write_bytes(GSM_UART_NUM, cmd, strlen(cmd));

    int total = 0;
    TickType_t start_tick = xTaskGetTickCount();
    bool have_summary = false;

    while (total < (int)(sizeof(resp) - 1) && !have_summary) {
        uint32_t elapsed_ms =
            (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
        if (elapsed_ms >= budget_ms) break;

        int len = uart_read_bytes(GSM_UART_NUM,
                                  (uint8_t *)(resp + total),
                                  sizeof(resp) - 1 - total,
                                  pdMS_TO_TICKS(500));
        if (len > 0) {
            total += len;
            resp[total] = '\0';
            /* Early exit when summary line appears */
            if (gsm_ping_find_summary(resp, total) != NULL) {
                have_summary = true;
            }
        }
        /* If len <= 0 we just loop and check the time budget — the modem may
         * still be doing pings between replies. Don't bail on a gap. */
    }

    xSemaphoreGive(s_uart_mutex);

    char *summary = gsm_ping_find_summary(resp, total);
    if (summary == NULL) {
        ESP_LOGE(TAG, "QPING: no summary line after %lu ms (got %d bytes):\n%s",
                 (unsigned long)((xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS),
                 total, resp);
        return ESP_FAIL;
    }

    int result, sent, rcvd, lost, mn, mx, avg;
    if (sscanf(summary, "+QPING: %d,%d,%d,%d,%d,%d,%d",
               &result, &sent, &rcvd, &lost, &mn, &mx, &avg) != 7) {
        ESP_LOGE(TAG, "QPING: summary parse failed: %s", summary);
        return ESP_FAIL;
    }

    out->sent       = (uint8_t)sent;
    out->received   = (uint8_t)rcvd;
    out->loss_pct   = (sent > 0) ? (uint8_t)((lost * 100) / sent) : 100;
    out->rtt_min_ms = (uint16_t)mn;
    out->rtt_avg_ms = (uint16_t)avg;
    out->rtt_max_ms = (uint16_t)mx;
    out->reachable  = (rcvd > 0);

    ESP_LOGI(TAG, "PING result: sent=%d recv=%d loss=%d%% rtt avg=%dms max=%dms",
             sent, rcvd, out->loss_pct, avg, mx);

    return ESP_OK;
}

#endif /* CONFIG_NCLE_GSM_ENABLE */
