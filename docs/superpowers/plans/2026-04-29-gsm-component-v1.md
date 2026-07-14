# GSM Component V1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a thin GSM connectivity component for the CLv4 ESP32-S3 firmware that brings up a Quectel EC200U-CN modem on UART0, verifies AT-command response, polls registration/signal periodically, and reports status over BLE — triggered by BLE commands `gsm_enable` / `gsm_disable`.

**Architecture:** Two-layer component (`gsm.c` driver + `gsm_task.c` orchestration) ported from Conn_plus master with naming and Kconfig adjustments. Synchronous AT command engine with internal UART mutex. Status delivered to upper layers via callback. Compiles cleanly with `CONFIG_NCLE_GSM_ENABLE=n` (default) and `=y` (production).

**Tech Stack:** ESP-IDF v5.5, FreeRTOS, ESP32-S3, Quectel EC200U-CN AT command set.

**Reference spec:** [`docs/superpowers/specs/2026-04-29-gsm-component-v1-design.md`](../specs/2026-04-29-gsm-component-v1-design.md)

**Reference implementation:** `D:/ESP32S3-Conn_plus/Conn_plus/master.esp32s3/components/gsm/` (port from this).

---

## File Structure

**Create:**
- `components/gsm/CMakeLists.txt` — IDF component registration
- `components/gsm/Kconfig.projbuild` — menu options (UART/pins/baud/polarity)
- `components/gsm/include/gsm.h` — driver public API
- `components/gsm/include/gsm_task.h` — task + status callback API
- `components/gsm/gsm.c` — UART, GPIO, AT engine
- `components/gsm/gsm_task.c` — orchestration task

**Modify:**
- `components/cmd_parser/include/cmd_parser.h` — add `CMD_GSM_*` and `RESP_GSM_*` defines
- `components/cmd_parser/cmd_parser.c` — add three command handlers (gated by `CONFIG_NCLE_GSM_ENABLE`)
- `main/main.c` — define `gsm_ble_status_callback`, register it after BLE init

---

## Phase 1 — Component skeleton (compiles with ENABLE=n)

### Task 1: Create component folder, CMakeLists.txt, Kconfig.projbuild

**Files:**
- Create: `components/gsm/CMakeLists.txt`
- Create: `components/gsm/Kconfig.projbuild`

- [ ] **Step 1.1: Create the component directory**

```bash
mkdir -p components/gsm/include
```

- [ ] **Step 1.2: Write `components/gsm/CMakeLists.txt`**

```cmake
idf_component_register(
    SRCS "gsm.c" "gsm_task.c"
    INCLUDE_DIRS "include"
    REQUIRES driver freertos esp_timer
)
```

- [ ] **Step 1.3: Write `components/gsm/Kconfig.projbuild`**

```
menu "GSM Module (EC200U-CN)"

    config NCLE_GSM_ENABLE
        bool "Enable GSM Module"
        default n
        help
            Enable GSM/LTE module support (Quectel EC200U-CN).
            Requires hardware UART0 to be free — WM must be on soft UART
            (CONFIG_NCLE_WM_USE_SOFT_UART=y).

    if NCLE_GSM_ENABLE

        config NCLE_GSM_UART_NUM
            int "UART controller number"
            range 0 2
            default 0

        config NCLE_GSM_UART_TX_PIN
            int "GSM UART TX GPIO (ESP TX -> Module RX)"
            range 0 48
            default 16

        config NCLE_GSM_UART_RX_PIN
            int "GSM UART RX GPIO (ESP RX <- Module TX)"
            range 0 48
            default 15

        config NCLE_GSM_PWRKEY_PIN
            int "GSM PWRKEY GPIO"
            range 0 48
            default 7

        config NCLE_GSM_RST_PIN
            int "GSM RESET GPIO"
            range 0 48
            default 8

        config NCLE_GSM_RST_INVERTED
            bool "Inverted RESET polarity (HIGH = assert)"
            default y
            help
                Default (on) matches the Conn_plus board design which uses
                an inverter on the RESET line: HIGH = assert reset,
                LOW = release reset.
                Disable only if your CLv4 board is wired directly to the
                EC200U RESET pin per datasheet (LOW = assert, HIGH idle).

        config NCLE_GSM_UART_BAUD_RATE
            int "GSM UART baud rate"
            default 115200

        config NCLE_GSM_TASK_POLL_INTERVAL_MS
            int "Status poll interval (ms)"
            range 1000 60000
            default 10000

    endif

endmenu
```

- [ ] **Step 1.4: Commit**

```bash
git add components/gsm/CMakeLists.txt components/gsm/Kconfig.projbuild
git commit -m "feat(gsm): add empty component skeleton (CMakeLists, Kconfig)"
```

---

### Task 2: Create stub source/header files; verify build with ENABLE=n

**Files:**
- Create: `components/gsm/include/gsm.h`
- Create: `components/gsm/include/gsm_task.h`
- Create: `components/gsm/gsm.c`
- Create: `components/gsm/gsm_task.c`

- [ ] **Step 2.1: Write minimal `components/gsm/include/gsm.h`**

```c
#ifndef GSM_H
#define GSM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Public API will be filled in Task 3 */

#ifdef __cplusplus
}
#endif
#endif /* GSM_H */
```

- [ ] **Step 2.2: Write minimal `components/gsm/include/gsm_task.h`**

```c
#ifndef GSM_TASK_H
#define GSM_TASK_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Public API will be filled in Task 11 */

#ifdef __cplusplus
}
#endif
#endif /* GSM_TASK_H */
```

- [ ] **Step 2.3: Write minimal `components/gsm/gsm.c`**

```c
#include "gsm.h"

/* Implementation added in Tasks 4-10 */
```

- [ ] **Step 2.4: Write minimal `components/gsm/gsm_task.c`**

```c
#include "gsm_task.h"

/* Implementation added in Task 12 */
```

- [ ] **Step 2.5: Build the project (ENABLE=n is the default)**

```bash
idf.py reconfigure
idf.py build
```

Expected: Build completes successfully. The `gsm` component compiles to a (mostly empty) object file. No warnings about missing symbols. No new linker errors.

- [ ] **Step 2.6: Commit**

```bash
git add components/gsm/include/gsm.h components/gsm/include/gsm_task.h components/gsm/gsm.c components/gsm/gsm_task.c
git commit -m "feat(gsm): add stub headers and source files"
```

---

## Phase 2 — Driver layer (gsm.c)

### Task 3: Define full public API in `gsm.h`

**Files:**
- Modify: `components/gsm/include/gsm.h`

- [ ] **Step 3.1: Replace `components/gsm/include/gsm.h` with the full API**

```c
#ifndef GSM_H
#define GSM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Network registration status (matches AT+CREG result codes) */
typedef enum {
    GSM_NET_NOT_REGISTERED      = 0,
    GSM_NET_REGISTERED_HOME     = 1,
    GSM_NET_SEARCHING           = 2,
    GSM_NET_DENIED              = 3,
    GSM_NET_UNKNOWN             = 4,
    GSM_NET_REGISTERED_ROAMING  = 5,
} gsm_network_status_t;

/* ===== Lifecycle ===== */
esp_err_t gsm_init(void);
esp_err_t gsm_deinit(void);

/* ===== Power control ===== */
esp_err_t gsm_power_on(void);
esp_err_t gsm_power_off(void);
esp_err_t gsm_reset(void);

/* ===== AT command engine ===== */
bool      gsm_is_alive(void);
esp_err_t gsm_send_at_command(const char *cmd, char *response,
                              size_t response_size, uint32_t timeout_ms);

/* ===== Helpers (parse common AT replies) ===== */
esp_err_t gsm_get_module_info(char *info, size_t info_size);
esp_err_t gsm_get_signal_strength(uint8_t *rssi, uint8_t *ber);
esp_err_t gsm_get_network_status(gsm_network_status_t *status);

#ifdef __cplusplus
}
#endif
#endif /* GSM_H */
```

- [ ] **Step 3.2: Build to verify the header compiles**

```bash
idf.py build
```

Expected: Build still succeeds. No new errors (unused declarations are fine).

- [ ] **Step 3.3: Commit**

```bash
git add components/gsm/include/gsm.h
git commit -m "feat(gsm): define public driver API in gsm.h"
```

---

### Task 4: Add GPIO init helper to `gsm.c`

**Files:**
- Modify: `components/gsm/gsm.c`

- [ ] **Step 4.1: Replace `components/gsm/gsm.c` with includes + GPIO init**

```c
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
 * so HIGH = assert, LOW = release. Inverted boards leave the flag as default.
 * Direct (datasheet-spec) boards disable the flag in menuconfig.            */
#ifdef CONFIG_NCLE_GSM_RST_INVERTED
#define GSM_RST_ASSERT      1   /* drive HIGH to assert reset */
#define GSM_RST_RELEASE     0
#else
#define GSM_RST_ASSERT      0   /* drive LOW to assert reset (datasheet) */
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

    /* Idle state: PWRKEY HIGH (idle), RST in RELEASE */
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

#endif /* CONFIG_NCLE_GSM_ENABLE */
```

- [ ] **Step 4.2: Build**

```bash
idf.py build
```

Expected: Build succeeds. With `CONFIG_NCLE_GSM_ENABLE=n` the entire body is `#ifdef`'d out — `gsm_gpio_init` does not exist as a symbol. Compiler warns about no public symbols in `gsm.c`; this is acceptable (will go away in later tasks).

- [ ] **Step 4.3: Commit**

```bash
git add components/gsm/gsm.c
git commit -m "feat(gsm): add GPIO init helper"
```

---

### Task 5: Add UART init helper to `gsm.c`

**Files:**
- Modify: `components/gsm/gsm.c`

- [ ] **Step 5.1: Append `gsm_uart_init` to `gsm.c` (insert just before `#endif /* CONFIG_NCLE_GSM_ENABLE */`)**

```c
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
```

- [ ] **Step 5.2: Build**

```bash
idf.py build
```

Expected: Build succeeds. Both `gsm_gpio_init` and `gsm_uart_init` are unused statics → "defined but not used" warning is fine; will be consumed in Task 9.

- [ ] **Step 5.3: Commit**

```bash
git add components/gsm/gsm.c
git commit -m "feat(gsm): add UART init helper"
```

---

### Task 6: Implement `gsm_power_on`, `gsm_power_off`, `gsm_reset`

**Files:**
- Modify: `components/gsm/gsm.c`

- [ ] **Step 6.1: Append the three public power-control functions (insert before `#endif /* CONFIG_NCLE_GSM_ENABLE */`)**

```c
esp_err_t gsm_power_on(void)
{
    ESP_LOGI(TAG, "Power ON sequence...");

    vTaskDelay(pdMS_TO_TICKS(500));   /* Power rail stable */

    /* EC200U: pull PWRKEY LOW for >= 500ms to power on */
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

    /* EC200U: pull PWRKEY LOW for >= 650ms to power off */
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
    vTaskDelay(pdMS_TO_TICKS(5000));   /* allow modem to reboot */

    ESP_LOGI(TAG, "RESET complete");
    return ESP_OK;
}
```

- [ ] **Step 6.2: Build**

```bash
idf.py build
```

Expected: Build succeeds with no new warnings (these are public symbols, no longer "unused").

- [ ] **Step 6.3: Commit**

```bash
git add components/gsm/gsm.c
git commit -m "feat(gsm): add power_on, power_off, reset (polarity-aware)"
```

---

### Task 7: Implement `gsm_read_response` + `gsm_check_at_response` + `gsm_is_alive`

**Files:**
- Modify: `components/gsm/gsm.c`

- [ ] **Step 7.1: Append response-reader and alive-check helpers (insert before `#endif`)**

```c
/* Read up to (buffer_size-1) bytes from the modem within timeout_ms.
 * Returns the number of bytes read (0 on timeout). Always NUL-terminates. */
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
            break;  /* no more data this chunk */
        }
    }
    buffer[total] = '\0';
    return total;
}

/* Internal alive probe — sends "AT\r\n" and looks for "OK" in the reply.
 * Mutex-aware: takes the mutex if it exists (it may not during init). */
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

    uart_flush(GSM_UART_NUM);
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
```

- [ ] **Step 7.2: Build**

```bash
idf.py build
```

Expected: Build succeeds.

- [ ] **Step 7.3: Commit**

```bash
git add components/gsm/gsm.c
git commit -m "feat(gsm): add read_response, check_at_response, is_alive"
```

---

### Task 8: Implement `gsm_send_at_command` (mutex-protected)

**Files:**
- Modify: `components/gsm/gsm.c`

- [ ] **Step 8.1: Append `gsm_send_at_command` (insert before `#endif`)**

```c
esp_err_t gsm_send_at_command(const char *command, char *response,
                              size_t response_size, uint32_t timeout_ms)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (command == NULL) return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire UART mutex");
        return ESP_ERR_TIMEOUT;
    }

    uart_flush(GSM_UART_NUM);

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
```

- [ ] **Step 8.2: Build**

```bash
idf.py build
```

Expected: Build succeeds.

- [ ] **Step 8.3: Commit**

```bash
git add components/gsm/gsm.c
git commit -m "feat(gsm): add send_at_command with mutex protection"
```

---

### Task 9: Implement `gsm_init` and `gsm_deinit` (three-stage probe sequence)

**Files:**
- Modify: `components/gsm/gsm.c`

- [ ] **Step 9.1: Append `gsm_init` + `gsm_deinit` (insert before `#endif`)**

```c
esp_err_t gsm_init(void)
{
    if (s_initialized) return ESP_OK;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "GSM Module Initialization");
    ESP_LOGI(TAG, "========================================");

    /* Step 1: UART mutex */
    if (s_uart_mutex == NULL) {
        s_uart_mutex = xSemaphoreCreateMutex();
        if (s_uart_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create UART mutex");
            return ESP_FAIL;
        }
    }

    /* Step 2: GPIO (PWRKEY high, RST in release per polarity flag) */
    gsm_gpio_init();

    /* Step 3: UART driver */
    gsm_uart_init();

    /* Step 4: Stabilization wait */
    ESP_LOGI(TAG, "Waiting 2s for stabilization...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Step 5: Probe-1 — module already running? */
    ESP_LOGI(TAG, "Checking if module already running...");
    for (int i = 0; i < 3; i++) {
        if (gsm_check_at_response()) {
            ESP_LOGI(TAG, "Module already ON!");
            s_initialized = true;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Step 6: PWRKEY power-on, then probe-2 */
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

    /* Step 7: Hardware reset, then probe-3 */
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

    /* Failure path: log diagnostic wiring help, clean up */
    ESP_LOGE(TAG, "========================================");
    ESP_LOGE(TAG, "FAILED to communicate with module!");
    ESP_LOGE(TAG, "Check wiring:");
    ESP_LOGE(TAG, "  ESP TX  (GPIO%d) -> Module RX",  GSM_UART_TX_PIN);
    ESP_LOGE(TAG, "  ESP RX  (GPIO%d) -> Module TX",  GSM_UART_RX_PIN);
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
```

- [ ] **Step 9.2: Build**

```bash
idf.py build
```

Expected: Build succeeds. All previously-unused statics (`gsm_gpio_init`, `gsm_uart_init`, `gsm_check_at_response`, `gsm_read_response`) are now consumed.

- [ ] **Step 9.3: Commit**

```bash
git add components/gsm/gsm.c
git commit -m "feat(gsm): add init/deinit with 3-stage probe sequence"
```

---

### Task 10: Implement `gsm_get_module_info`, `gsm_get_signal_strength`, `gsm_get_network_status`

**Files:**
- Modify: `components/gsm/gsm.c`

- [ ] **Step 10.1: Append the three helpers (insert before `#endif`)**

```c
esp_err_t gsm_get_module_info(char *info, size_t info_size)
{
    if (info == NULL || info_size == 0) return ESP_ERR_INVALID_ARG;

    char response[256];
    if (gsm_send_at_command("I", response, sizeof(response), 2000) != ESP_OK) {
        return ESP_FAIL;
    }

    /* Response shape:
     *   "ATI\r\nQuectel\r\nEC200U\r\n...\r\nOK\r\n"
     * Skip the echoed "ATI" line and pick the next non-empty line as the model.
     */
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
    /* Fallback: dump whole response */
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

    /* Response shape: "+CSQ: <rssi>,<ber>\r\nOK\r\n" */
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

    /* Response shape: "+CREG: <n>,<stat>\r\nOK\r\n" */
    char *p = strstr(response, "+CREG:");
    int n, stat;
    if (p && sscanf(p, "+CREG: %d,%d", &n, &stat) == 2) {
        *status = (gsm_network_status_t)stat;
        return ESP_OK;
    }
    return ESP_FAIL;
}
```

- [ ] **Step 10.2: Build**

```bash
idf.py build
```

Expected: Build succeeds. Driver layer is complete.

- [ ] **Step 10.3: Commit**

```bash
git add components/gsm/gsm.c
git commit -m "feat(gsm): add module_info, signal_strength, network_status helpers"
```

---

## Phase 3 — Task layer (gsm_task.c)

### Task 11: Define `gsm_task.h` API and status struct

**Files:**
- Modify: `components/gsm/include/gsm_task.h`

- [ ] **Step 11.1: Replace `components/gsm/include/gsm_task.h` with the full API**

```c
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
```

- [ ] **Step 11.2: Build**

```bash
idf.py build
```

Expected: Build succeeds.

- [ ] **Step 11.3: Commit**

```bash
git add components/gsm/include/gsm_task.h
git commit -m "feat(gsm): define gsm_task public API and gsm_status_t"
```

---

### Task 12: Implement `gsm_task.c` (init + 10s poll loop + status callback)

**Files:**
- Modify: `components/gsm/gsm_task.c`

- [ ] **Step 12.1: Replace `components/gsm/gsm_task.c` with the full implementation**

```c
#include "gsm_task.h"

#ifdef CONFIG_NCLE_GSM_ENABLE

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "GSM_TASK";

#define GSM_POLL_INTERVAL_MS   CONFIG_NCLE_GSM_TASK_POLL_INTERVAL_MS

static TaskHandle_t       s_task_handle  = NULL;
static volatile bool      s_running      = false;
static gsm_status_cb_t    s_status_cb    = NULL;
static void              *s_status_ctx   = NULL;
static gsm_status_t       s_last_status  = {0};

/* Convert AT+CSQ rssi (0-31, 99=unknown) to display bars (0-5). */
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

    /* Carry over module_info captured at start */
    memcpy(s.module_info, s_last_status.module_info, sizeof(s.module_info));

    s_last_status = s;

    if (s_status_cb) s_status_cb(&s, s_status_ctx);

    ESP_LOGI(TAG, "alive=%d reg=%d rssi=%d bars=%d net=%d",
             s.alive, s.registered, s.rssi, s.bars, (int)s.net_status);
}

static void gsm_task_body(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "GSM task starting");

    if (gsm_init() != ESP_OK) {
        ESP_LOGE(TAG, "gsm_init failed");
        gsm_status_t bad = {
            .alive = false, .registered = false,
            .rssi = 99, .ber = 99, .bars = 0,
            .net_status = GSM_NET_UNKNOWN,
        };
        s_last_status = bad;
        if (s_status_cb) s_status_cb(&bad, s_status_ctx);
        s_running = false;
        s_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Capture module info once (static for a given hardware unit) */
    char info[64] = {0};
    if (gsm_get_module_info(info, sizeof(info)) == ESP_OK) {
        ESP_LOGI(TAG, "Module: %s", info);
        strncpy(s_last_status.module_info, info,
                sizeof(s_last_status.module_info) - 1);
    }

    /* Initial poll right away */
    poll_and_report();

    /* Main loop: poll every GSM_POLL_INTERVAL_MS, but wake every 1s
     * so gsm_task_stop() is observed within ~1s. */
    uint32_t accumulated_ms = 0;
    while (s_running) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        accumulated_ms += 1000;
        if (accumulated_ms >= GSM_POLL_INTERVAL_MS) {
            accumulated_ms = 0;
            poll_and_report();
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
    /* Task self-deletes after observing s_running=false (within ~1s) +
     * gsm_deinit (~2s). No join here; caller is fire-and-forget. */
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
```

- [ ] **Step 12.2: Build**

```bash
idf.py build
```

Expected: Build succeeds. With ENABLE=n the body compiles to nothing.

- [ ] **Step 12.3: Commit**

```bash
git add components/gsm/gsm_task.c
git commit -m "feat(gsm): add gsm_task with poll loop and status callback"
```

---

## Phase 4 — Integration

### Task 13: Add `gsm_enable` / `gsm_disable` / `gsm_status` command handlers to `cmd_parser`

**Files:**
- Modify: `components/cmd_parser/include/cmd_parser.h`
- Modify: `components/cmd_parser/cmd_parser.c`
- Modify: `components/cmd_parser/CMakeLists.txt`

- [ ] **Step 13.1: Read the existing `cmd_parser` CMakeLists to confirm REQUIRES**

```bash
cat components/cmd_parser/CMakeLists.txt
```

Note any current `REQUIRES` list. The `gsm` component must be added to it for the command handlers to use `gsm_task_*` functions.

- [ ] **Step 13.2: Add `gsm` to `REQUIRES` in `components/cmd_parser/CMakeLists.txt`**

If the file currently looks like:
```cmake
idf_component_register(
    SRCS "cmd_parser.c"
    INCLUDE_DIRS "include"
    REQUIRES wm_uart ma_uart printer_uart battery ota json
)
```

Append `gsm` to REQUIRES so it becomes:
```cmake
idf_component_register(
    SRCS "cmd_parser.c"
    INCLUDE_DIRS "include"
    REQUIRES wm_uart ma_uart printer_uart battery ota json gsm
)
```

(Match the actual REQUIRES list from your file — just add `gsm` at the end.)

- [ ] **Step 13.3: Add new command and response defines to `components/cmd_parser/include/cmd_parser.h`**

Insert these near the existing `CMD_*` defines (after `CMD_GET_BATTERY_STATUS`):

```c
/* GSM commands (only meaningful when CONFIG_NCLE_GSM_ENABLE=y) */
#define CMD_GSM_ENABLE          "gsm_enable"
#define CMD_GSM_DISABLE         "gsm_disable"
#define CMD_GSM_STATUS          "gsm_status"
```

Insert near the existing `RESP_*` defines:

```c
#define RESP_GSM_ENABLE_STARTED    "gsm_enable_started"
#define RESP_GSM_ENABLE_FAILED     "gsm_enable_failed"
#define RESP_GSM_DISABLE_STOPPED   "gsm_disable_stopped"
#define RESP_GSM_NOT_ENABLED       "gsm_not_enabled"
#define RESP_GSM_STATUS_OK         "gsm_status"
```

- [ ] **Step 13.4: Add `#include "gsm_task.h"` to the top of `components/cmd_parser/cmd_parser.c`**

Add the include guarded by the GSM-enabled flag near the existing `#include "wm_uart.h"` block:

```c
#ifdef CONFIG_NCLE_GSM_ENABLE
#include "gsm_task.h"
#endif
```

- [ ] **Step 13.5: Add three command branches in the dispatch chain in `cmd_parser.c`**

Insert just before the final `else` that returns `RESP_UNDEFINED_COMMAND` (find the dispatch chain — it's the long `if/else if` ladder around line 332). Add:

```c
    else if (strcmp(cmd, CMD_GSM_ENABLE) == 0) {
#ifdef CONFIG_NCLE_GSM_ENABLE
        esp_err_t err = gsm_task_start();
        send_response(err == ESP_OK ? RESP_GSM_ENABLE_STARTED
                                    : RESP_GSM_ENABLE_FAILED,
                      err == ESP_OK ? STATUS_OK : STATUS_ERR, NULL);
#else
        send_response(RESP_GSM_NOT_ENABLED, STATUS_ERR, NULL);
#endif
    }
    else if (strcmp(cmd, CMD_GSM_DISABLE) == 0) {
#ifdef CONFIG_NCLE_GSM_ENABLE
        gsm_task_stop();
        send_response(RESP_GSM_DISABLE_STOPPED, STATUS_OK, NULL);
#else
        send_response(RESP_GSM_NOT_ENABLED, STATUS_ERR, NULL);
#endif
    }
    else if (strcmp(cmd, CMD_GSM_STATUS) == 0) {
#ifdef CONFIG_NCLE_GSM_ENABLE
        gsm_status_t s;
        gsm_task_get_last_status(&s);
        char buf[160];
        snprintf(buf, sizeof(buf),
                 "{\"alive\":%d,\"registered\":%d,\"rssi\":%d,\"bars\":%d,\"net\":%d}",
                 s.alive, s.registered, s.rssi, s.bars, (int)s.net_status);
        send_response(RESP_GSM_STATUS_OK, STATUS_OK, buf);
#else
        send_response(RESP_GSM_NOT_ENABLED, STATUS_ERR, NULL);
#endif
    }
```

- [ ] **Step 13.6: Build**

```bash
idf.py build
```

Expected: Build succeeds. With ENABLE=n the GSM branches return `RESP_GSM_NOT_ENABLED`.

- [ ] **Step 13.7: Commit**

```bash
git add components/cmd_parser/include/cmd_parser.h components/cmd_parser/cmd_parser.c components/cmd_parser/CMakeLists.txt
git commit -m "feat(cmd_parser): add gsm_enable/gsm_disable/gsm_status commands"
```

---

### Task 14: Wire GSM status callback to BLE in `main/main.c`

**Files:**
- Modify: `main/main.c`
- Modify: `main/CMakeLists.txt` (if `gsm` not already inherited)

- [ ] **Step 14.1: Verify `main/CMakeLists.txt` will see the gsm component**

```bash
cat main/CMakeLists.txt
```

If main's CMakeLists declares its REQUIRES explicitly, add `gsm` to it. Otherwise (most ESP-IDF projects), main automatically depends on all components and no change is needed.

If a change is needed, append `gsm` to the REQUIRES line.

- [ ] **Step 14.2: Add includes near the top of `main/main.c`**

Find the section where `wm_uart.h` is included (around line 80-90) and add:

```c
#ifdef CONFIG_NCLE_GSM_ENABLE
#include "gsm.h"
#include "gsm_task.h"
#endif
```

- [ ] **Step 14.3: Add the GSM-to-BLE status callback function**

Insert near the existing `wm_ble_data_callback` (around line 262), guarded by `CONFIG_NCLE_GSM_ENABLE`:

```c
#ifdef CONFIG_NCLE_GSM_ENABLE
/**
 * @brief Status callback wired to BLE TX
 * Fires once per gsm_task poll interval (default 10s).
 */
static void gsm_ble_status_callback(const gsm_status_t *s, void *ctx)
{
    (void)ctx;
    char buf[160];
    int n = snprintf(buf, sizeof(buf),
        "{\"device\":\"gsm\",\"alive\":%d,\"registered\":%d,"
        "\"rssi\":%d,\"bars\":%d,\"net\":%d}\n",
        s->alive, s->registered, s->rssi, s->bars, (int)s->net_status);
    if (n > 0 && n < (int)sizeof(buf)) {
        ble_spp_output_callback(buf, (unsigned int)n);
    }
}
#endif
```

- [ ] **Step 14.4: Register the callback after BLE init**

Find the existing block in `app_main()` (around line 856-865) where `wm_uart_set_data_callback(...)` is called. Immediately below the MA callback registration block, add:

```c
        // Register GSM status callback - sends GSM signal/registration to mobile app via BLE
#ifdef CONFIG_NCLE_GSM_ENABLE
        gsm_task_set_status_callback(gsm_ble_status_callback, NULL);
        ESP_LOGI(TAG, "GSM -> BLE callback registered");
        // NOTE: gsm_task_start() is NOT called here; waits for BLE command
#endif
```

- [ ] **Step 14.5: Build**

```bash
idf.py build
```

Expected: Build succeeds with ENABLE=n (the GSM blocks compile to nothing).

- [ ] **Step 14.6: Commit**

```bash
git add main/main.c
git commit -m "feat(main): wire GSM status callback to BLE TX"
```

---

## Phase 5 — Acceptance

### Task 15: Build with `CONFIG_NCLE_GSM_ENABLE=y` and verify

**Files:**
- Modify: `sdkconfig` (via menuconfig, will be rebuilt)

- [ ] **Step 15.1: Enable GSM in menuconfig**

```bash
idf.py menuconfig
```

Navigate to:
- `Component config` → `GSM Module (EC200U-CN)` → `[*] Enable GSM Module`

Verify defaults:
- UART controller number = 0
- TX = 16, RX = 15, PWRKEY = 7, RST = 8
- `[*] Inverted RESET polarity (HIGH = assert)` (default y, matches Conn_plus)
- Baud = 115200
- Poll interval = 10000

Save and exit (`Q`, then `Y`).

- [ ] **Step 15.2: Build**

```bash
idf.py build
```

Expected:
- Build succeeds
- No warnings related to `gsm` component (or only benign ones)
- `gsm.c` and `gsm_task.c` show as compiled in the build summary
- Final binary size increases by ~3-5KB

- [ ] **Step 15.3: Disable and re-build to verify symmetry**

```bash
idf.py menuconfig
# Disable [ ] Enable GSM Module
idf.py build
```

Expected: Build succeeds again. Binary size returns to original. Confirms `#ifdef CONFIG_NCLE_GSM_ENABLE` gating is correct.

- [ ] **Step 15.4: Re-enable for the bench test**

```bash
idf.py menuconfig
# Re-enable [*] Enable GSM Module
idf.py build
```

- [ ] **Step 15.5: Commit the sdkconfig change**

```bash
git add sdkconfig
git commit -m "build: enable GSM module in sdkconfig (CONFIG_NCLE_GSM_ENABLE=y)"
```

---

### Task 16: Manual bench test on hardware

This task is the V1 acceptance criterion. It cannot be automated — it requires the EC200U module physically wired to the ESP32-S3 board. Run all steps below and write down results inline (replace the `<...>` placeholders).

- [ ] **Step 16.1: Hardware wiring**

Wire the EC200U-CN to the ESP32-S3:

| ESP32 GPIO | Modem pin |
|---|---|
| GPIO 16 | RX (modem's RX) |
| GPIO 15 | TX (modem's TX) |
| GPIO 7 | PWRKEY |
| GPIO 8 | RESET |
| GND | GND |

Power the modem from a 3.8-4.2V supply (NOT from the ESP32 3.3V rail — EC200U draws peak 1.5A which the ESP regulator cannot supply).

- [ ] **Step 16.2: Flash and monitor**

```bash
idf.py -p <COM_PORT> flash monitor
```

Expected boot sequence:
- Standard NCLE_MAIN init lines
- `WM_UART: Initialized: SOFT-UART RX=44 (UART peripheral unused)` — confirms WM is on soft UART (precondition met)
- `GSM -> BLE callback registered` — confirms callback is wired
- **No** other GSM activity at this point (component dormant by design)

- [ ] **Step 16.3: Connect a BLE client**

Use any BLE SPP-capable client (e.g., the project's mobile app, "Serial Bluetooth Terminal" Android app, or a Python `bleak` script). Connect to device named `NitaraCLE4`.

Expect log: `BLE_SPP: BLE Client connected, conn_id=0` and `Notifications enabled`.

- [ ] **Step 16.4: Send `gsm_enable` and observe**

From the BLE client send the JSON line (followed by `#` per the project's command-framing convention):

```
{"command":"gsm_enable"}#
```

Expected logs (in order):
```
GSM: ========================================
GSM: GSM Module Initialization
GSM: ========================================
GSM: GPIO: PWRKEY=7, RST=8 (inverted=1)
GSM: UART: TX=16, RX=15, Baud=115200
GSM: Waiting 2s for stabilization...
GSM: Checking if module already running...
GSM: TX: AT             # might already be on; otherwise:
GSM: Module not responding, trying power-on...
GSM: Power ON sequence...
GSM: Waiting for module boot...
GSM: AT test 1/5 after power-on
GSM: TX: AT
GSM: RX: ...OK...
GSM: Module responding after power-on!
GSM_TASK: Module: <module model string>
GSM_TASK: alive=1 reg=0 rssi=99 bars=0 net=2     # net=2 = SEARCHING
```

Expected BLE TX (every 10s):
```
{"device":"gsm","alive":1,"registered":0,"rssi":99,"bars":0,"net":2}
```

- [ ] **Step 16.5: Wait for SIM to register (10-30s)**

Log line eventually shifts to:
```
GSM_TASK: alive=1 reg=1 rssi=<N> bars=<B> net=1     # or net=5 for roaming
```

BLE TX:
```
{"device":"gsm","alive":1,"registered":1,"rssi":<N>,"bars":<B>,"net":1}
```

Record observed values — RSSI should be 1-31, bars 1-5, registered=1.

**Observed:** RSSI=`<...>`, bars=`<...>`, net=`<...>` after `<...>s`.

- [ ] **Step 16.6: Send `gsm_status` for instant snapshot**

```
{"command":"gsm_status"}#
```

Expected immediate response (cached, no AT roundtrip):
```
{"response_message":"gsm_status","status_code":0,"data":{"alive":1,"registered":1,"rssi":<N>,"bars":<B>,"net":1}}
```

- [ ] **Step 16.7: Send `gsm_disable` and verify clean stop**

```
{"command":"gsm_disable"}#
```

Expected logs:
```
GSM_TASK: GSM task stopping
GSM: Power OFF sequence...
GSM: Power OFF complete
```

Expected: No more periodic BLE status updates within 1-2s. Response:
```
{"response_message":"gsm_disable_stopped","status_code":0}
```

- [ ] **Step 16.8: Re-send `gsm_enable` to verify clean restart**

Same as Step 16.4. This time, the "Module already ON" branch may trigger if PWRKEY was a long pulse — note in the log either path is OK.

- [ ] **Step 16.9: Edge — pull modem TX wire mid-run**

While `gsm_enable` is active and registered, physically disconnect the wire from GPIO 15. Wait one poll cycle (~10s).

Expected log:
```
GSM_TASK: alive=0 reg=0 rssi=99 bars=0 net=4
```

Expected BLE TX shows `alive:0`. Reconnect the wire — next poll recovers.

- [ ] **Step 16.10: Verify no UART conflicts**

Confirm that during all of the above, **WM, MA, and Printer continue working normally** (send a weight to the WM and observe the `{"device":"wm",...}` JSON on BLE; same for MA if applicable). Print a test receipt if Printer is wired.

- [ ] **Step 16.11: Document results + commit**

If all steps pass, commit any minor adjustments (log message tweaks etc.) made during bench testing. If a step fails, capture the log, debug, fix in code, repeat from step 16.4 — do NOT mark this task complete on partial success.

```bash
# If any source changes were needed during debug:
git add <files>
git commit -m "fix(gsm): <whatever was discovered on bench>"
```

If no changes were needed, no commit at this step. The component is ready for handoff to the senior.

---

## Self-review checklist

Run through this after the plan is implemented, before declaring V1 done:

- [ ] All 16 tasks committed individually with clear messages
- [ ] `idf.py build` succeeds with both `CONFIG_NCLE_GSM_ENABLE=y` and `=n`
- [ ] Bench test (Task 16) recorded observed RSSI/bars/net values for at least one full registration cycle
- [ ] Periodic BLE status updates fire every ~10s while task is running
- [ ] `gsm_disable` cleanly stops the task within 1-2s, modem powers off
- [ ] WM, MA, Printer behavior unchanged when GSM is enabled and running
- [ ] No `uart_driver_install(0, ...)` conflicts (proves WM-on-soft-UART precondition holds)

---

## Out of scope reminder (do NOT attempt in V1)

These belong to V2 or to the senior — do not creep into V1:

- `gsm_ping()` / `gsm_get_connection_quality()` (V2)
- TCP/HTTP/MQTT/SMS layers (senior)
- PDP context configuration / APN management (senior)
- Asynchronous URC dispatcher (senior)
- BLE-vs-GSM transport selection logic (senior)
- Power management / sleep mode (out of scope entirely for now)
