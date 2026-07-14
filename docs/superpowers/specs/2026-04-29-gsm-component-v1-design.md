# GSM Component V1 — Design Spec

**Date:** 2026-04-29
**Status:** Draft, awaiting user review
**Author:** collaborative (user + Claude)
**Target hardware:** ESP32-S3 (CLv4 GSM variant)
**GSM module:** Quectel EC200U-CN (LTE Cat 1, 115200 baud, AT commands)
**Branch:** `feat/wm-on-soft-uart` (built on top of completed Phase 2 cutover)

---

## 1. Problem

The CLv4 product needs GSM/LTE connectivity to send weighing-machine data to a remote server. The Quectel EC200U-CN module is the chosen hardware. The firmware needs a connectivity layer that:

1. Brings the modem up reliably from any state (off, already-on, hung)
2. Verifies the modem responds to AT commands
3. Reports network registration status and signal strength
4. Exposes BLE-driven enable/disable so the user app controls when GSM is active
5. Provides a clean handoff to a senior developer who will layer SMS / TCP / HTTP / MQTT features on top

This spec covers V1 only: connectivity establishment plus the BLE trigger flow. Higher-level features (data path, fallback to BLE, ping-based quality verdict) are deferred to V2.

## 2. Strategy

Port the proven `gsm_module` + `gsm_task` pattern from the Conn_plus master project (`D:\ESP32S3-Conn_plus\Conn_plus\master.esp32s3\components\gsm\`) into CLv4, with two adjustments:

1. **Transport for trigger / status:** SPI (Conn_plus) → BLE (CLv4). Conn_plus has a touchscreen slave that sends SPI commands; CLv4 uses BLE SPP commands routed through the existing `cmd_parser` component.
2. **Naming and Kconfig surface:** match CLv4 conventions (`gsm_*` not `gsm_module_*`, all hardware choices Kconfig-configurable).

The internal state machine (init retry sequence, periodic poll loop, status reporting) is unchanged because it is already proven on hardware.

## 3. Prerequisites

- **WM is on soft UART** (CONFIG_NCLE_WM_USE_SOFT_UART=y) so hardware UART0 is free. Verified in branch `feat/wm-on-soft-uart` (boot log: `WM_UART: Initialized: SOFT-UART RX=44 (UART peripheral unused)`).
- Existing components in CLv4: `wm_uart`, `ma_uart`, `printer_uart`, `ble_spp`, `cmd_parser`, `battery`, `ota`, `soft_uart_rmt`. None of these are modified by this work except `cmd_parser` which gets three new command handlers.

## 4. Hardware

### 4.1 GPIO assignment (defaults)

| Signal | Direction | GPIO | Kconfig key |
|---|---|---|---|
| UART TX (ESP→Modem) | output | 16 | `NCLE_GSM_UART_TX_PIN` |
| UART RX (ESP←Modem) | input | 15 | `NCLE_GSM_UART_RX_PIN` |
| PWRKEY | output | 7 | `NCLE_GSM_PWRKEY_PIN` |
| RESET | output | 8 | `NCLE_GSM_RST_PIN` |

Pins selected to avoid conflicts with: WM (44), Printer (17/18), MA (39/40), Status LED (36), Battery ADC (6), Soft UART loopback (15 — overlaps but loopback is dev-only and inactive in production builds), Flash (26-32), strapping pins (0/3/45/46), USB (19/20).

GPIO 15 is shared with the (development-only) soft UART loopback test default. The GSM scaffold takes precedence in production; the loopback Kconfig default may be moved to a different pin in a follow-up change if both are ever needed simultaneously.

### 4.2 Reset polarity

The EC200U datasheet specifies active-low reset (drive LOW to assert, HIGH idle). However, the Conn_plus board (which CLv4's GSM hardware is derived from) inserts an inverter on the reset line, requiring active-HIGH to assert. The proven, working design is therefore the *inverted* one.

Resolved via Kconfig flag `NCLE_GSM_RST_INVERTED`:

| Value | Behavior | When to use |
|---|---|---|
| `y` (default) | HIGH = assert, LOW = release | Conn_plus-derived boards (default — likely the CLv4 hardware) |
| `n` | LOW = assert, HIGH = release | Boards wired direct to the EC200U RESET pin per datasheet |

If bring-up confirms the CLv4 board uses direct (datasheet) wiring instead of the Conn_plus inverter circuit, flip the flag in `menuconfig` — no code change needed.

### 4.3 UART controller

UART0 is the target controller (freed by the WM-on-soft-UART cutover). Exposed as Kconfig `NCLE_GSM_UART_NUM` (default 0, range 0-2) for testing flexibility.

## 5. Architecture

### 5.1 Component layout

```
components/gsm/
├── CMakeLists.txt
├── Kconfig.projbuild
├── include/
│   ├── gsm.h           # Driver API: init, power, AT, helpers
│   └── gsm_task.h      # Orchestration: task lifecycle + status callback
├── gsm.c               # UART, GPIO, AT command engine (mutex-protected)
└── gsm_task.c          # FreeRTOS task: init → periodic poll → status callback
```

Two-layer split mirrors Conn_plus:

- **`gsm.c`** — synchronous low-level driver. Single-threaded behavior enforced via internal UART mutex.
- **`gsm_task.c`** — long-running orchestration. Owns the connection lifecycle and produces status updates.

### 5.2 Data flow

```
   BLE client (phone app)
        │
        │ BLE SPP RX:
        │ {"command":"gsm_enable"}
        ▼
   ble_spp                ──►   cmd_parser
                                    │
                                    │ gsm_task_start()
                                    ▼
                          ┌─────────────────────────────────┐
                          │  gsm_task (FreeRTOS task)       │
                          │  ────────────────────────────── │
                          │  gsm_init()  ──► gsm.c          │
                          │     │  power on / AT verify     │
                          │     ▼                            │
                          │  Loop every 10s:                 │
                          │    gsm_is_alive()                │
                          │    gsm_get_signal_strength()     │
                          │    gsm_get_network_status()      │
                          │     │                            │
                          │     ▼ status_callback(s, ctx)    │
                          └─────────────────────────────────┘
                                    │
                                    │ on_gsm_status() (in main.c)
                                    ▼
                              Build JSON: {"device":"gsm", ...}
                                    │
                                    ▼
                              ble_spp_send(...)
                                    │
                                    ▼
                          BLE client receives status update
```

The gsm_task is the **only** owner of the modem after `gsm_task_start()` is called. The `cmd_parser` sees gsm_status snapshots through `gsm_task_get_last_status()` (cached, no AT roundtrip).

## 6. Public API

### 6.1 `gsm.h` — driver

```c
typedef enum {
    GSM_NET_NOT_REGISTERED = 0,
    GSM_NET_REGISTERED_HOME = 1,
    GSM_NET_SEARCHING = 2,
    GSM_NET_DENIED = 3,
    GSM_NET_UNKNOWN = 4,
    GSM_NET_REGISTERED_ROAMING = 5,
} gsm_network_status_t;

esp_err_t gsm_init(void);
esp_err_t gsm_deinit(void);

esp_err_t gsm_power_on(void);
esp_err_t gsm_power_off(void);
esp_err_t gsm_reset(void);

bool      gsm_is_alive(void);
esp_err_t gsm_send_at_command(const char *cmd, char *response,
                              size_t response_size, uint32_t timeout_ms);

esp_err_t gsm_get_module_info(char *info, size_t info_size);
esp_err_t gsm_get_signal_strength(uint8_t *rssi, uint8_t *ber);
esp_err_t gsm_get_network_status(gsm_network_status_t *status);
```

Semantics match Conn_plus exactly. `gsm_send_at_command()` is mutex-protected; concurrent callers are serialized.

### 6.2 `gsm_task.h` — orchestration

```c
typedef struct {
    bool                  alive;
    gsm_network_status_t  net_status;
    bool                  registered;     // home or roaming
    uint8_t               rssi;           // 0-31, 99=unknown
    uint8_t               ber;            // 0-7, 99=unknown
    uint8_t               bars;           // 0-5, derived from rssi
    char                  module_info[64];
} gsm_status_t;

typedef void (*gsm_status_cb_t)(const gsm_status_t *s, void *ctx);

esp_err_t gsm_task_start(void);
esp_err_t gsm_task_stop(void);
bool      gsm_task_is_running(void);

void gsm_task_set_status_callback(gsm_status_cb_t cb, void *ctx);
void gsm_task_get_last_status(gsm_status_t *out);
```

`gsm_task_start()` is idempotent: returns `ESP_OK` if already running.
`gsm_task_stop()` is idempotent: returns `ESP_OK` if not running.

Bars are derived from RSSI per Conn_plus convention:
| RSSI | Bars |
|---|---|
| 25-31 | 5 |
| 19-24 | 4 |
| 13-18 | 3 |
| 7-12 | 2 |
| 1-6 | 1 |
| 0 or 99 (unknown) | 0 |

## 7. Kconfig

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

## 8. Init flow (`gsm_init()`)

1. Create UART mutex (`xSemaphoreCreateMutex`)
2. Configure GPIO: PWRKEY as output (HIGH idle), RST as output (release-state per polarity flag)
3. Configure UART: 115200 8N1, no flow control, default APB clock source
4. Install UART driver with 2KB RX / 1KB TX buffers
5. Wait 2s for power rails / modem stabilization
6. **Probe-1:** Try AT/OK x3 (500ms apart). If any responds → already on, return `ESP_OK`
7. **Power-on:** PWRKEY pulse LOW for 600ms, return HIGH, wait 5s for boot
8. **Probe-2:** Try AT/OK x5 (500ms apart). If any responds → return `ESP_OK`
9. **Hardware reset:** RST pulse per polarity flag (assert 100ms, release, wait 5s)
10. **Probe-3:** Try AT/OK x3 (500ms apart). If any responds → return `ESP_OK`
11. **Failure:** log diagnostic wiring help, `uart_driver_delete()`, free mutex, return `ESP_FAIL`

This sequence handles three real-world startup states: modem already running (warm boot), modem off (cold boot), modem hung (needs reset).

## 9. Task flow (`gsm_task`)

1. `gsm_init()` — if `ESP_FAIL`, fire `status_cb({alive=false, registered=false, rssi=99, bars=0, ...})` and self-delete
2. Sleep 2s for post-init settle
3. `gsm_get_module_info()` (sends `ATI`) → cache in `gsm_status_t.module_info`
4. Set `gsm_task_running = true`
5. Loop while `gsm_task_running`:
   - On every `poll_interval_ms` boundary (default 10s):
     - `gsm_is_alive()`
     - `gsm_get_signal_strength()`
     - `gsm_get_network_status()`
     - Build `gsm_status_t`, derive `bars`, derive `registered` (HOME or ROAMING)
     - Cache as `g_last_status` (atomic copy under brief critsec)
     - Fire `status_callback(&g_last_status, ctx)`
   - `vTaskDelay(1s)` so `gsm_task_stop()` is observed within ≤1s
6. On exit: `gsm_deinit()` (powers off modem, deletes UART driver, frees mutex), `vTaskDelete(NULL)`

## 10. BLE command integration (`cmd_parser`)

Three new commands added to the existing JSON command parser. Each returns a JSON response over the same channel.

```c
{"command":"gsm_enable"}
    → gsm_task_start()
    → response: {"response_message":"gsm_enable_started","status_code":0}
                or {"response_message":"gsm_enable_failed","status_code":<errno>}

{"command":"gsm_disable"}
    → gsm_task_stop()
    → response: {"response_message":"gsm_disable_stopped","status_code":0}

{"command":"gsm_status"}
    → gsm_task_get_last_status()    # cached, instant
    → response: {"gsm":{"alive":1,"registered":1,"rssi":15,"bars":3,"net":1}}
```

Periodic status updates (separate from the on-demand `gsm_status` command) are pushed via the status callback wired to `ble_spp_send()` in `main.c` (see §11). These appear over BLE TX once per `poll_interval_ms`.

## 11. Wiring in `main.c`

```c
#ifdef CONFIG_NCLE_GSM_ENABLE
#include "gsm.h"
#include "gsm_task.h"

static void on_gsm_status(const gsm_status_t *s, void *ctx) {
    char buf[160];
    int n = snprintf(buf, sizeof(buf),
        "{\"device\":\"gsm\",\"alive\":%d,\"registered\":%d,"
        "\"rssi\":%d,\"bars\":%d,\"net\":%d}\n",
        s->alive, s->registered, s->rssi, s->bars, s->net_status);
    if (n > 0 && n < (int)sizeof(buf)) {
        ble_spp_send((uint8_t*)buf, (size_t)n);
    }
}
#endif

void app_main(void) {
    /* ... existing init ... */

#ifdef CONFIG_NCLE_GSM_ENABLE
    gsm_task_set_status_callback(on_gsm_status, NULL);
    /* gsm_task_start() is NOT called here; it starts on BLE command */
#endif

    /* ... rest of init ... */
}
```

At boot the GSM component is compiled in but dormant. No UART traffic, no GPIO toggling. Activation is fully under user control via BLE.

## 12. Error handling

| Failure mode | Behavior |
|---|---|
| `gsm_init()` reaches step 11 (no AT response after retries) | Log wiring help; return `ESP_FAIL`; task fires all-bad status_cb and self-deletes; subsequent `gsm_task_start()` is allowed (re-runs init from scratch) |
| AT command timeout | Returns `ESP_ERR_TIMEOUT`; caller (helper or task) logs and continues |
| UART driver install failure | `ESP_ERROR_CHECK` aborts (treated as a programming bug, must be fixed) |
| Mutex contention | 5s timeout for alive checks, 10s for general AT — returns `ESP_ERR_TIMEOUT` |
| `gsm_task_start()` called twice | Second call returns `ESP_OK` immediately (idempotent) |
| `gsm_task_stop()` called when not running | Returns `ESP_OK` immediately (idempotent) |
| `gsm_task_stop()` mid-AT | Mutex held by AT call completes first (≤10s), then task observes `running=false` and exits cleanly within 1s of mutex release |
| Reset polarity flag wrong | Modem never responds → init falls through to FAIL → diagnostic logs prompt operator to flip Kconfig flag |
| GSM enabled without WM on soft UART | Both drivers call `uart_driver_install(UART_NUM_0, ...)` → second one fails with `ESP_ERR_INVALID_ARG` → that subsystem's `ESP_ERROR_CHECK` aborts. Documented as user error; no defensive code |

## 13. Testing plan

### 13.1 Bench test (manual, on hardware)

```
1. Wire EC200U:
     ESP TX (GPIO16) -> Module RX
     ESP RX (GPIO15) -> Module TX
     ESP PWR (GPIO7) -> Module PWRKEY
     ESP RST (GPIO8) -> Module RESET
     Common GND
     Module power 3.8-4.2V
2. menuconfig:
     - NCLE_GSM_ENABLE=y
     - Confirm pin defaults match wiring
     - NCLE_GSM_RST_INVERTED: leave default (y, matches Conn_plus); flip to n only if reset doesn't work
3. Build & flash. Verify boot:
     - WM_UART: SOFT-UART RX=44 (UART peripheral unused)   <- precondition
     - No GSM activity on boot (component dormant)
     - All other peripherals (WM, MA, Printer, BLE, Battery) initialize normally
4. Connect BLE client (phone app or BLE explorer). MTU should negotiate to ~500.
5. Send {"command":"gsm_enable"}
     Expect logs:
     - GSM: GPIO: PWRKEY=7, RST=8
     - GSM: UART: TX=16, RX=15, Baud=115200
     - GSM: Waiting 2s for stabilization...
     - GSM: Checking if module already running... (or power-on path)
     - GSM: Module: <model string>  (after ATI)
     Expect BLE TX every 10s:
     - {"device":"gsm","alive":1,"registered":0,"rssi":99,"bars":0,"net":2}
6. Wait for SIM registration (10-30s).
     Expect rssi 1-31, registered=1, bars 1-5.
7. Send {"command":"gsm_status"} → instant cached response.
8. Send {"command":"gsm_disable"}
     Expect:
     - Status updates stop within 1-2s
     - Logs: GSM: Power OFF sequence... / Power OFF complete
9. Re-send {"command":"gsm_enable"} → re-init succeeds, module already-on path triggers.
```

### 13.2 Edge cases

- **No SIM:** registered stays 0, rssi may still report (radio hears towers). Expected.
- **Wrong RST polarity:** init fails after 13 AT probes (~25s). Operator flips `NCLE_GSM_RST_INVERTED` and retries.
- **Pull modem TX wire mid-run:** next poll cycle reports alive=false, registered=false, rssi=99. Status updates continue until `gsm_disable` is sent.
- **Power-cycle modem manually mid-run:** alive=false for one poll cycle (~10s), then init's already-on path recovers without firmware reset.
- **All four UART subsystems active simultaneously** (WM-soft, MA-UART2, Printer-UART1, GSM-UART0): no controller conflicts. Validate with `idf.py size --diff` for sanity on RAM/IRAM headroom.

### 13.3 Non-goals for V1

- No BLE-vs-GSM transport selection logic (V2)
- No ping-based connection quality verdict (V2)
- No data path (TCP/HTTP/MQTT/SMS) — senior owns this layer
- No automated regression tests; manual bench validation is the V1 acceptance bar

## 14. Out of scope (deferred to V2 or senior)

- **`gsm_ping()` / `gsm_get_connection_quality()`** — V2. Required to make GSM-vs-BLE fallback decisions on real signal.
- **Data path** — senior. Includes `AT+QIOPEN`, `AT+QISEND`, `AT+QIRD` for TCP, or higher-level HTTP/MQTT.
- **PDP context management** — senior. `AT+QICSGP`, `AT+QIACT`, APN configuration.
- **URC dispatcher** — senior. Asynchronous server messages (`+CMTI`, `+QIURC`, `+CEREG` unsolicited) need a separate URC parser; the synchronous `gsm_send_at_command` does not handle URCs.
- **GPS/GNSS** — separate component if needed (Conn_plus has `gps_module.c` / `gps_task.c` as a model).
- **Power management** — currently no sleep/wake support. The modem stays powered as long as `gsm_task` runs.
- **Multi-modem support** — single instance only. Conn_plus pattern; sufficient for CLv4.

## 15. Open decisions (defer to implementation plan)

- Whether `cmd_parser` JSON response format includes the gsm status snapshot inline with `gsm_enable_started` (saves one round-trip) or only on explicit `gsm_status` query. Implementation choice; UX impact only.
- Whether to log every periodic poll at INFO or DEBUG level. INFO matches Conn_plus; could become spammy. Implementation may prefer DEBUG with INFO only on state changes (registered→not registered, etc.).
- Module identification storage: `module_info` field captured on init, never refreshed. Acceptable because `ATI` output is static for a given hardware unit.

These are minor and don't affect architecture.

## 16. Success outcome

When V1 is complete:

- A new `components/gsm/` exists with `gsm.h`, `gsm.c`, `gsm_task.h`, `gsm_task.c`, `Kconfig.projbuild`, `CMakeLists.txt`.
- Building with `CONFIG_NCLE_GSM_ENABLE=y` compiles without warnings; building with `=n` (default) excludes the component entirely.
- Sending `{"command":"gsm_enable"}` over BLE on a board with EC200U wired powers up the modem, verifies AT/OK, and begins emitting status JSON over BLE every 10s.
- Sending `{"command":"gsm_disable"}` cleanly stops the task and powers off the modem.
- The senior can call `gsm_send_at_command()` from new code (e.g. `AT+QIOPEN` for TCP) without modifying `gsm.c`.
- WM, MA, Printer, BLE, Battery, and OTA continue working unchanged.

Phase 2 cutover (WM on soft UART, UART0 free) is a hard prerequisite and is assumed complete before this work begins.
