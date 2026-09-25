# WiFi as an Operator-Selectable Alternative Link — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the operator switch the connector between GSM, WiFi, and off from the mobile app, with exactly one radio running at a time and the choice remembered across reboots.

**Architecture:** A new `link_mode` setting (gsm | wifi | off, default gsm) stored in NVS decides which connectivity stack is initialised. GSM keeps today's boot path when the mode is `gsm`. WiFi is brought up **lazily** — `esp_wifi_init()` runs only when the mode becomes `wifi`, never at boot — because a spike measured that starting it at boot costs 81 KB of heap (124 KB → 43 KB), which is below what FOTA needs to download an image. Switching modes tears the outgoing stack down before bringing the incoming one up, so the two never coexist. The application layer is untouched: it asks `net_link_is_up()` and does not care which link answers.

**Tech Stack:** ESP-IDF v5.5, ESP32-S3, esp_modem (PPP over UART), esp_wifi, esp_netif, lwIP, NVS, BLE SPP command parser.

**Spec:** No separate spec document — the design was agreed in conversation and is restated in full under "Design Decisions" below.

---

## Global Constraints

- **One radio at a time.** GSM and WiFi are never both initialised. This is a memory requirement, not a preference: both together leaves 43 KB free and FOTA needs ~45 KB to download.
- **Default is `gsm`.** A device with no stored mode boots exactly as it does today.
- **Switching is manual only.** No automatic failover, no priority logic, no background probing of the inactive link.
- **The mode persists across reboot.** Field machines are powered off between the morning and evening collection sessions; a mode that reset each boot would mean re-switching twice a day.
- **BLE is always available** and is the recovery path out of any mode, including a WiFi that can no longer reach its router.
- **Buffering rule is already implemented and must not change.** `mqtt_client_svc.c:306` buffers only while `net_link_is_up()`; with the link down the reading is handed to the app over BLE. This works unchanged for whichever link is active.
- **No new heap cost in `gsm` mode.** Free heap at boot in `gsm` mode must stay within a few KB of today's ~124 KB. Any task that regresses this has failed.
- Firmware version stays `2.1.0.1000` until Task 9, which bumps it for the FOTA test.
- **WiFi must be fully removable at build time.** `CONFIG_NCLE_WIFI_ENABLE=n` must produce a firmware byte-for-byte equivalent in behaviour to today's: no WiFi code linked, no flash cost, no heap cost, GSM and BLE untouched, `link_mode` reduced to gsm-or-off. Every file this plan touches guards its WiFi references with `#ifdef CONFIG_NCLE_WIFI_ENABLE`, and Task 11 proves it by building both ways and comparing. This is not a nicety: it is how a GSM-only product line ships without carrying 294 KB of flash and the risk of a radio it never uses.

---

## Design Decisions

Recorded here because the plan argues from them and there is no separate spec.

**Why lazy WiFi init rather than start-and-stop.** The obvious approach — start both at boot and stop the unused one — was measured and rejected. `esp_wifi_init()` allocates its buffers immediately and holds them whether or not the radio is associated; the spike showed 43 KB free with WiFi idle and never connected. Deferring `esp_wifi_init()` until the mode is actually `wifi` is what keeps `gsm` mode at today's headroom.

**Why the mode lives in NVS rather than in RAM.** See the reboot constraint above.

**Why `net_link` gains a mode concept rather than a priority list.** `net_link_is_up()` returns true if *any* registered link is up, and `net_link_active()` returns the first registered link that is up. With only one link ever initialised, both already give the right answer — no priority logic is needed. What *is* needed is that the inactive link's descriptor does not linger in the table claiming to be up; Task 3 handles that by registering the link as part of bringing it up, not at boot.

**Known hazards inherited from the WiFi product's `wifi_sta` component**, each addressed by a named task:
- `net_link_register()` is called from inside `ncle_wifi_sta_init()` (`wifi_sta.c:412`). `NET_LINK_MAX` is 2 and `net_link.c` has no unregister. Re-initialising WiFi twice would exhaust the table. → Task 3.
- The task's reconnect loop (`wifi_sta_task.c:179-195`) retries every 15 s forever with no off switch, so "switch to GSM" would leave WiFi associating in the background. → Task 5.
- `ncle_wifi_sta_task_stop()` uses `vTaskDelete()` on a task that may hold `s_status_mutex`, permanently leaking it. → Task 5.
- `esp_coex_preference_set(ESP_COEX_PREFER_WIFI)` (`wifi_sta.c:382`) biases the shared 2.4 GHz radio away from BLE. In this product BLE is the primary control channel. → Task 4. **Revised during implementation:** the setting was kept. The code comment records a *measured* failure with BALANCE - BLE starving the WPA2 handshake - and the line only runs while WiFi is initialised, which the lazy-init design already confines to wifi mode. Changing it would have traded a known problem for an assumed one. The residual risk (BLE responsiveness in wifi mode) is checked in Task 8 step 6.

---

## File Structure

| File | Responsibility | Task |
|---|---|---|
| `components/connectivity/wifi_sta/**` | Copied verbatim from the WiFi product, then amended by Tasks 3–5. Already present in the working tree from the spike. | 1 |
| `components/base/config/device_config.{c,h}` | Add `link_mode` field, default, NVS key, getter/setter. | 2 |
| `components/connectivity/wifi_sta/wifi_sta.c` | Split `net_link_register` out of `init()`; gate coex preference. | 3, 4 |
| `components/connectivity/wifi_sta/wifi_sta_task.c` | Cooperative stop flag replacing `vTaskDelete`. | 5 |
| `components/connectivity/link_mode/**` | **New.** Owns the mode: reads it at boot, brings the right stack up, tears the other down on switch. The only file that knows both GSM and WiFi exist. | 6 |
| `components/base/cmd_parser/cmd_parser.c` | Add `set_link_mode` to the MQTT risky-command block. | 7 |
| `main/main.c` | Replace the unconditional GSM start with a call into `link_mode`. | 6 |
| `main/Kconfig.projbuild` | `rsource` the wifi_sta Kconfig. | 1 |
| `main/CMakeLists.txt` | Add `wifi_sta` and `link_mode` to `PRIV_REQUIRES`. | 1, 6 |

A separate `link_mode` component is deliberate: it is the only place that references both `gsm.h` and `wifi_sta.h`, which keeps the switching logic in one readable file and leaves both connectivity components unaware of each other.

---

### Task 1: Land the wifi_sta component and prove it builds

The spike already copied the component and edited three files. This task makes that state deliberate and committed, minus the spike's boot-time WiFi start.

**Files:**
- Already present (untracked): `components/connectivity/wifi_sta/`
- Modify: `main/Kconfig.projbuild` (already edited by the spike — verify)
- Modify: `main/CMakeLists.txt` (already edited by the spike — verify)
- Revert: `main/main.c`, `sdkconfig`

**Interfaces:**
- Consumes: nothing.
- Produces: the `wifi_sta` component compiles in this tree. Public API as in `components/connectivity/wifi_sta/include/wifi_sta.h`, notably `ncle_wifi_sta_init(void)`, `ncle_wifi_sta_deinit(void)`, `ncle_wifi_sta_connect(const char *ssid, const char *password)`, `ncle_wifi_sta_is_connected(void)`, `ncle_wifi_sta_task_start(void)`.

- [ ] **Step 1: Undo the spike's edits to main.c and sdkconfig**

The spike added a boot-time WiFi start to `main.c` and `CONFIG_NCLE_WIFI_ENABLE=y` to `sdkconfig`. Both must go — WiFi is started by `link_mode` in Task 6, never at boot.

```bash
git checkout -- main/main.c sdkconfig
```

- [ ] **Step 2: Verify the two intended edits survived**

```bash
grep -n 'wifi_sta/Kconfig.in' main/Kconfig.projbuild
grep -n 'wifi_sta' main/CMakeLists.txt
```

Expected: one match each. If either is missing, re-add:
- `main/Kconfig.projbuild`, after the `mqtt_client_svc/Kconfig.in` rsource line:
  `    rsource "../components/connectivity/wifi_sta/Kconfig.in"`
- `main/CMakeLists.txt`, in `PRIV_REQUIRES` after `gsm`:
  `        wifi_sta`

- [ ] **Step 3: Enable the component in sdkconfig.defaults, not sdkconfig**

`sdkconfig` is generated. The setting belongs in the checked-in defaults.

Append to `sdkconfig.defaults`:

```
# WiFi is compiled in as a selectable alternative link. It is NOT started at
# boot - see components/connectivity/link_mode. Starting it at boot costs 81 KB
# of heap (measured: 124 KB -> 43 KB free) which is below what FOTA needs.
CONFIG_NCLE_WIFI_ENABLE=y
```

- [ ] **Step 4: Build**

```bash
idf.py build
```

Expected: PASS. Binary grows to roughly 1,760 KB (from 1,466 KB) — the WiFi stack's flash cost. Still 43% free in a 3 MB partition.

- [ ] **Step 5: Confirm no boot-time WiFi start leaked through**

```bash
grep -n 'ncle_wifi_sta_task_start\|ncle_wifi_sta_init' main/main.c
```

Expected: **no matches.** `main.c` must not reference WiFi at all after this task.

- [ ] **Step 6: Commit**

```bash
git add components/connectivity/wifi_sta main/Kconfig.projbuild main/CMakeLists.txt sdkconfig.defaults
git commit -m "feat(wifi): land wifi_sta from the WiFi product, compiled but not started

Copied verbatim from D:/ESP32S3-CLV4-WIFI so the two products keep one
implementation. Compiled in but deliberately NOT started at boot: a spike
measured esp_wifi_init() costing 81 KB of heap (124 KB -> 43 KB free) whether
or not the radio ever associates, and FOTA needs ~45 KB to download an image.
Bring-up moves to the link_mode component in a later task."
```

---

### Task 2: Store the link mode in NVS

**Files:**
- Modify: `components/base/config/device_config.h`
- Modify: `components/base/config/device_config.c`

**Interfaces:**
- Consumes: the existing `device_config_t` struct and its NVS load/save pattern — follow `ble_data_mode` (`device_config.h:101`, `device_config.c:110/166/223/265/274`) exactly; it is the closest analogue.
- Produces:
  - `#define LINK_MODE_GSM 0`, `LINK_MODE_WIFI 1`, `LINK_MODE_OFF 2`
  - `#define DEFAULT_LINK_MODE LINK_MODE_GSM`
  - `esp_err_t config_set_link_mode(uint8_t mode);`
  - `uint8_t config_get_link_mode(void);`
  - `const char *config_link_mode_name(uint8_t mode);` — returns `"gsm"` / `"wifi"` / `"off"`

- [ ] **Step 1: Add the constants and the struct field**

In `device_config.h`, beside the existing `BLE_DATA_*` constants:

```c
/* Which connectivity stack is running. Exactly one, ever: GSM and WiFi both
 * initialised costs 81 KB of heap and breaks FOTA, so the mode is not a
 * preference but a hard selection. Chosen by the operator from the app;
 * survives reboot because field machines are powered off between the morning
 * and evening sessions and re-switching twice a day is not acceptable. */
#define LINK_MODE_GSM       0
#define LINK_MODE_WIFI      1
#define LINK_MODE_OFF       2

#define DEFAULT_LINK_MODE   LINK_MODE_GSM
```

In `device_config_t`, beside `uint8_t ble_data_mode;`:

```c
    uint8_t link_mode;                  // LINK_MODE_GSM / _WIFI / _OFF
```

Declare the accessors beside the `ble_data_mode` ones:

```c
esp_err_t config_set_link_mode(uint8_t mode);
uint8_t   config_get_link_mode(void);
const char *config_link_mode_name(uint8_t mode);
```

- [ ] **Step 2: Add the NVS key and defaults**

In `device_config.c`, beside `KEY_BLE_DATA_MODE`:

```c
static const char *KEY_LINK_MODE = "link_mode";
```

In the defaults function, beside `config->ble_data_mode = DEFAULT_BLE_DATA_MODE;`:

```c
    config->link_mode = DEFAULT_LINK_MODE;
```

In the NVS load function, mirroring the `ble_data_mode` block — note the `err != ESP_OK` branch must fall back to the default so an upgrade from a build without this key does not read garbage:

```c
    err = nvs_get_u8(handle, KEY_LINK_MODE, &mode);
    if (err == ESP_OK) {
        config->link_mode = mode;
    } else {
        config->link_mode = DEFAULT_LINK_MODE;
    }
```

In the NVS save function, beside the `ble_data_mode` set:

```c
    err = nvs_set_u8(handle, KEY_LINK_MODE, config->link_mode);
```

- [ ] **Step 3: Implement the accessors**

```c
esp_err_t config_set_link_mode(uint8_t mode)
{
    if (mode > LINK_MODE_OFF) {
        return ESP_ERR_INVALID_ARG;
    }
    g_device_config.link_mode = mode;
    return config_save();
}

uint8_t config_get_link_mode(void)
{
    return g_device_config.link_mode;
}

const char *config_link_mode_name(uint8_t mode)
{
    switch (mode) {
        case LINK_MODE_GSM:  return "gsm";
        case LINK_MODE_WIFI: return "wifi";
        case LINK_MODE_OFF:  return "off";
        default:             return "unknown";
    }
}
```

Match the existing `config_save()` spelling in this file — check whether it is `config_save()` or takes an argument before writing.

- [ ] **Step 4: Build**

```bash
idf.py build
```

Expected: PASS.

- [ ] **Step 5: Flash and confirm the default on a device with no stored mode**

```bash
idf.py -p COM4 flash monitor
```

Expected: the device behaves exactly as before (nothing reads `link_mode` yet). This step is a regression check, not a feature check: free heap at boot must still be ~124 KB.

- [ ] **Step 6: Commit**

```bash
git add components/base/config/device_config.c components/base/config/device_config.h
git commit -m "feat(config): store the selected link mode in NVS

gsm | wifi | off, defaulting to gsm so an unconfigured device boots exactly
as it does today. Persisted because field machines are powered off between
the morning and evening sessions - a mode that reset each boot would mean an
operator re-switching twice a day."
```

---

### Task 3: Make WiFi's net_link registration one-shot

`ncle_wifi_sta_init()` calls `net_link_register()` on every init. `NET_LINK_MAX` is 2 and `net_link.c` has no unregister, so initialising WiFi a second time exhausts the table and the link silently disappears from the registry. Mirror the GSM pattern (`gsm.c:832`, `gsm_net_link_register()`), which registers once from outside init.

**Files:**
- Modify: `components/connectivity/wifi_sta/wifi_sta.c:406-412`
- Modify: `components/connectivity/wifi_sta/include/wifi_sta.h`

**Interfaces:**
- Consumes: `net_link_register()` from `net_link.h`; `ncle_wifi_sta_is_connected()` and the file-static `wifi_link_status_json` / `wifi_link_is_provisioned`.
- Produces: `void wifi_net_link_register(void);` — idempotent, safe to call more than once, registers the WiFi descriptor at most once for the lifetime of the process.

- [ ] **Step 1: Remove the registration from init**

Delete this block from the end of `ncle_wifi_sta_init()` (`wifi_sta.c:406-412`):

```c
    static const net_link_t wifi_link = {
        .name           = "wifi",
        .is_up          = ncle_wifi_sta_is_connected,
        .status_json    = wifi_link_status_json,
        .is_provisioned = wifi_link_is_provisioned,
    };
    net_link_register(&wifi_link);
```

- [ ] **Step 2: Add the one-shot registration function**

Add near the end of `wifi_sta.c`, mirroring `gsm_net_link_register()`:

```c
void wifi_net_link_register(void)
{
    /* One-shot: net_link has NET_LINK_MAX == 2 slots and no unregister, so a
     * second registration would consume the last slot and then fail silently
     * for every link after it. WiFi can be initialised and de-initialised many
     * times over a device's life as the operator switches modes, so this must
     * not live inside init(). */
    static bool s_registered = false;
    if (s_registered) {
        return;
    }

    /* Static storage: net_link keeps the pointer, so a stack copy would dangle. */
    static const net_link_t wifi_link = {
        .name           = "wifi",
        .is_up          = ncle_wifi_sta_is_connected,
        .status_json    = wifi_link_status_json,
        .is_provisioned = wifi_link_is_provisioned,
    };

    if (net_link_register(&wifi_link)) {
        s_registered = true;
        ESP_LOGI(TAG, "registered with net_link as '%s'", wifi_link.name);
    } else {
        ESP_LOGE(TAG, "net_link registration FAILED - the application layer "
                      "will believe the device is permanently offline");
    }
}
```

- [ ] **Step 3: Declare it in the header**

In `wifi_sta.h`, beside `ncle_wifi_sta_init`:

```c
/**
 * @brief Register WiFi with net_link. Idempotent; call before starting MQTT.
 *
 * Separate from init() because init/deinit run repeatedly as the operator
 * switches link modes, while net_link registration must happen at most once.
 */
void wifi_net_link_register(void);
```

- [ ] **Step 4: Build**

```bash
idf.py build
```

Expected: PASS. Nothing calls `wifi_net_link_register()` yet — that is Task 6.

- [ ] **Step 5: Commit**

```bash
git add components/connectivity/wifi_sta/wifi_sta.c components/connectivity/wifi_sta/include/wifi_sta.h
git commit -m "fix(wifi): register with net_link once, not on every init

net_link has 2 slots and no unregister. WiFi init/deinit now runs repeatedly
as the operator switches modes, so registering from inside init() would
exhaust the table on the second switch and every later link would fail to
register - silently, because the application only ever asks 'is anything up'.
Mirrors gsm_net_link_register()."
```

---

### Task 4: Stop WiFi biasing the shared radio away from BLE

`wifi_sta.c:382` calls `esp_coex_preference_set(ESP_COEX_PREFER_WIFI)` during init, with a comment explaining that BLE is "only an occasional config channel". That is true of the WiFi product; it is false here, where BLE is the primary control channel and the only way to recover a device from a bad WiFi.

**Files:**
- Modify: `components/connectivity/wifi_sta/wifi_sta.c:382-385`

**Interfaces:**
- Consumes: nothing new.
- Produces: no API change.

- [ ] **Step 1: Change the preference to balanced**

Replace the `ESP_COEX_PREFER_WIFI` call with:

```c
#ifdef CONFIG_ESP_COEX_SW_COEXIST_ENABLE
    /* BALANCE, not PREFER_WIFI as the WiFi product uses. There, BLE is an
     * occasional config channel and starving it costs little. Here BLE is the
     * primary control channel AND the only way back from a WiFi that can no
     * longer reach its router - an operator who cannot connect over BLE cannot
     * switch the device back to GSM. */
    esp_err_t coex_ret = esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
    ESP_LOGI(TAG, "Coexistence preference: balanced (%s)", esp_err_to_name(coex_ret));
#endif
```

- [ ] **Step 2: Build**

```bash
idf.py build
```

Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add components/connectivity/wifi_sta/wifi_sta.c
git commit -m "fix(wifi): balance radio coexistence instead of preferring WiFi

The WiFi product biases the shared 2.4 GHz radio toward WiFi because BLE
there is an occasional config channel. In this product BLE is the primary
control channel and the only way to switch a device back to GSM when its
WiFi can no longer reach the router, so starving it is the one failure that
cannot be recovered in the field."
```

---

### Task 5: Give the WiFi task a cooperative off switch

Two defects for this use case, both in `wifi_sta_task.c`:
1. The reconnect loop (`:179-195`) retries every 15 s forever with no way to say "stop" — after switching to GSM, WiFi would keep associating in the background.
2. `ncle_wifi_sta_task_stop()` (`:219-227`) calls `vTaskDelete()` on a task that may be blocked inside `ncle_wifi_sta_connect()` holding `s_status_mutex`. Killing it there leaks the mutex permanently, and every later `ncle_wifi_sta_get_status()` blocks forever on `portMAX_DELAY`.

**Files:**
- Modify: `components/connectivity/wifi_sta/wifi_sta_task.c`

**Interfaces:**
- Consumes: nothing new.
- Produces: `esp_err_t ncle_wifi_sta_task_stop(void)` keeps its signature but now stops the task cooperatively and blocks until it has exited. Safe to call when the task is not running.

- [ ] **Step 1: Add the stop flag**

Beside the existing statics at the top of `wifi_sta_task.c`:

```c
/* Cooperative stop. vTaskDelete() on this task is unsafe: it can be blocked
 * inside ncle_wifi_sta_connect() holding s_status_mutex, and killing it there
 * leaks the mutex - after which every ncle_wifi_sta_get_status() blocks
 * forever on portMAX_DELAY. The task checks this flag each second and exits
 * its own loop instead. */
static volatile bool s_stop_request = false;
static volatile bool s_task_exited  = false;
```

- [ ] **Step 2: Check the flag in the loop**

Change the loop header from `while (1) {` to:

```c
    while (!s_stop_request) {
```

and immediately after the loop's closing brace, before the function ends:

```c
    ESP_LOGI(TAG, "WiFi task stopping on request");
    s_task_exited  = true;
    s_task_handle  = NULL;
    vTaskDelete(NULL);      /* self-delete: the only safe delete for this task */
```

- [ ] **Step 3: Also break out of the reconnect burst**

The reconnect block calls `ncle_wifi_sta_connect()` then falls through to a 1 s delay. Guard the reconnect itself so a stop request during a burst does not start one more association:

Immediately inside `if (++disconnected_secs >= WIFI_RECONNECT_PERIOD_S) {`, add as the first statement:

```c
                if (s_stop_request) {
                    break;      /* leaving the loop; do not start another join */
                }
```

- [ ] **Step 4: Rewrite the stop function**

Replace the body of `ncle_wifi_sta_task_stop()`:

```c
esp_err_t ncle_wifi_sta_task_stop(void)
{
    if (s_task_handle == NULL) {
        return ESP_OK;              /* not running */
    }

    s_stop_request = true;
    s_task_exited  = false;

    /* The task checks the flag once a second, so 3 s is generous. If it has
     * not exited by then something is wedged and tearing WiFi down anyway
     * would be worse than leaving it running - report the failure up. */
    for (int i = 0; i < 30 && !s_task_exited; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    s_stop_request = false;

    if (!s_task_exited) {
        ESP_LOGE(TAG, "WiFi task did not stop within 3 s - leaving it running");
        return ESP_ERR_TIMEOUT;
    }

    return ncle_wifi_sta_deinit();
}
```

- [ ] **Step 5: Reset the flag on start**

In `ncle_wifi_sta_task_start()`, before `xTaskCreate`:

```c
    s_stop_request = false;
    s_task_exited  = false;
```

- [ ] **Step 6: Build**

```bash
idf.py build
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add components/connectivity/wifi_sta/wifi_sta_task.c
git commit -m "fix(wifi): stop the task cooperatively instead of deleting it

The task was written for a device where WiFi is the only way home, so it
reconnects every 15 s forever and the stop path was never exercised. Two
problems for mode switching: after switching to GSM the task would keep
associating in the background, and vTaskDelete() could kill it while it held
s_status_mutex inside connect(), permanently wedging every later
get_status() on portMAX_DELAY. Now it checks a flag each second and
self-deletes, and stop() waits for that before de-initialising."
```

---

### Task 6: The link_mode component — bring up exactly one stack

The heart of the feature. A new component that owns the mode, brings the right stack up at boot, and switches between them at runtime. It is the only file that includes both `gsm.h` and `wifi_sta.h`.

**Files:**
- Create: `components/connectivity/link_mode/CMakeLists.txt`
- Create: `components/connectivity/link_mode/include/link_mode.h`
- Create: `components/connectivity/link_mode/link_mode.c`
- Modify: `main/CMakeLists.txt`
- Modify: `main/main.c`

**Interfaces:**
- Consumes: `config_get_link_mode()`, `config_set_link_mode()`, `config_link_mode_name()` (Task 2); `gsm_task_start()`, `gsm_deinit()` from `gsm.h`/`gsm_task.h`; `ncle_wifi_sta_task_start()`, `ncle_wifi_sta_task_stop()`, `wifi_net_link_register()` (Task 3); `gsm_net_link_register()`.
- Produces:
  - `esp_err_t link_mode_start(void);` — read the stored mode and bring that stack up. Call once from `main.c` where `gsm_task_start()` is called today.
  - `esp_err_t link_mode_switch(uint8_t mode);` — tear down the current stack, bring up the requested one, persist the choice.
  - `uint8_t link_mode_current(void);`

- [ ] **Step 1: Check the exact GSM teardown API before writing**

The plan assumes `gsm_deinit()` exists and is safe to call while the modem is in data mode. Verify:

```bash
grep -n 'gsm_deinit\|gsm_task_stop\|esp_modem_destroy' components/connectivity/gsm/include/gsm.h components/connectivity/gsm/include/gsm_task.h components/connectivity/gsm/gsm.c
```

If there is no safe teardown, **stop and report** — GSM teardown while PPP is up previously crashed the device (`InstructionFetchError`, `esp_modem_destroy` racing its worker) and that is recorded in the project memory as a hazard. A teardown that stops the netif before switching mode, and never destroys the DCE while the link is up, is a prerequisite for this task.

- [ ] **Step 2: Write the component CMakeLists**

`components/connectivity/link_mode/CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS "link_mode.c"
    INCLUDE_DIRS "include"
    REQUIRES common config
    PRIV_REQUIRES gsm wifi_sta cmd_parser json
)
```

- [ ] **Step 3: Write the header**

`components/connectivity/link_mode/include/link_mode.h`:

```c
/**
 * @file link_mode.h
 * @brief Which connectivity stack is running: gsm, wifi, or neither.
 *
 * Exactly one is ever initialised. This is a memory requirement, not a
 * preference: a spike measured both stacks together leaving 43 KB of heap free
 * against the ~45 KB FOTA needs to download an image. GSM alone leaves ~124 KB.
 *
 * The mode is chosen by the operator from the app and stored in NVS, so a
 * device powered off between the morning and evening collection sessions comes
 * back in the mode it was left in.
 *
 * BLE is unaffected by the mode and is always the way back - including out of
 * a wifi mode whose router has been replaced.
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>

/**
 * @brief Bring up the stored mode's stack. Call once during startup.
 *
 * Replaces the unconditional gsm_task_start() in main.c. A device with no
 * stored mode gets LINK_MODE_GSM and behaves exactly as before this feature.
 */
esp_err_t link_mode_start(void);

/**
 * @brief Switch modes: tear the current stack down, bring the new one up.
 *
 * Blocking - the teardown waits for the outgoing task to exit. Persists the
 * new mode to NVS only after the switch succeeds, so a failed switch does not
 * leave the device booting into a mode that did not work.
 *
 * @param mode LINK_MODE_GSM, LINK_MODE_WIFI or LINK_MODE_OFF.
 */
esp_err_t link_mode_switch(uint8_t mode);

/** @brief The mode currently running (not necessarily the one stored). */
uint8_t link_mode_current(void);
```

- [ ] **Step 4: Write the implementation**

`components/connectivity/link_mode/link_mode.c`:

```c
#include "link_mode.h"
#include "device_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef CONFIG_NCLE_GSM_ENABLE
#include "gsm.h"
#include "gsm_task.h"
#endif
#ifdef CONFIG_NCLE_WIFI_ENABLE
#include "wifi_sta.h"
#include "wifi_sta_task.h"
#endif

static const char *TAG = "LINK_MODE";

/* What is actually running right now, which is not always what is stored: a
 * failed switch leaves the old stack up and the old mode stored. */
static uint8_t s_running = LINK_MODE_OFF;

uint8_t link_mode_current(void)
{
    return s_running;
}

/* --- bring up ---------------------------------------------------------- */

static esp_err_t bring_up(uint8_t mode)
{
    switch (mode) {
#ifdef CONFIG_NCLE_GSM_ENABLE
    case LINK_MODE_GSM: {
        gsm_net_link_register();
        esp_err_t err = gsm_task_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "GSM start failed: %s", esp_err_to_name(err));
            return err;
        }
        s_running = LINK_MODE_GSM;
        ESP_LOGI(TAG, "mode: gsm");
        return ESP_OK;
    }
#endif
#ifdef CONFIG_NCLE_WIFI_ENABLE
    case LINK_MODE_WIFI: {
        /* esp_wifi_init() runs inside here, which is where the 81 KB goes.
         * Nothing above this line has allocated it. */
        wifi_net_link_register();
        esp_err_t err = ncle_wifi_sta_task_start();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "WiFi start failed: %s", esp_err_to_name(err));
            return err;
        }
        s_running = LINK_MODE_WIFI;
        ESP_LOGI(TAG, "mode: wifi");
        return ESP_OK;
    }
#endif
    case LINK_MODE_OFF:
        s_running = LINK_MODE_OFF;
        ESP_LOGI(TAG, "mode: off - readings go to the app over BLE");
        return ESP_OK;

    default:
        ESP_LOGE(TAG, "unknown mode %u", (unsigned)mode);
        return ESP_ERR_INVALID_ARG;
    }
}

/* --- tear down --------------------------------------------------------- */

static esp_err_t tear_down(uint8_t mode)
{
    switch (mode) {
#ifdef CONFIG_NCLE_GSM_ENABLE
    case LINK_MODE_GSM:
        ESP_LOGI(TAG, "stopping GSM");
        return gsm_task_stop();     /* verified in Step 1 */
#endif
#ifdef CONFIG_NCLE_WIFI_ENABLE
    case LINK_MODE_WIFI:
        ESP_LOGI(TAG, "stopping WiFi");
        return ncle_wifi_sta_task_stop();
#endif
    case LINK_MODE_OFF:
    default:
        return ESP_OK;
    }
}

/* --- public ------------------------------------------------------------ */

esp_err_t link_mode_start(void)
{
    uint8_t mode = config_get_link_mode();
    ESP_LOGI(TAG, "stored mode: %s", config_link_mode_name(mode));

    esp_err_t err = bring_up(mode);
    if (err != ESP_OK && mode != LINK_MODE_GSM) {
        /* The stored mode would not start. Fall back to GSM rather than leave
         * the device with no link at all: BLE still works either way, but a
         * device that can reach the cloud is worth more than one that cannot,
         * and the operator can switch again from the app. */
        ESP_LOGW(TAG, "stored mode failed to start - falling back to gsm");
        err = bring_up(LINK_MODE_GSM);
    }
    return err;
}

esp_err_t link_mode_switch(uint8_t mode)
{
    if (mode > LINK_MODE_OFF) {
        return ESP_ERR_INVALID_ARG;
    }
    if (mode == s_running) {
        ESP_LOGI(TAG, "already in %s", config_link_mode_name(mode));
        return ESP_OK;
    }

    ESP_LOGI(TAG, "switching %s -> %s",
             config_link_mode_name(s_running), config_link_mode_name(mode));

    /* Down before up, always: the two stacks must never be initialised at the
     * same time (81 KB, see the header). */
    esp_err_t err = tear_down(s_running);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "teardown of %s failed (%s) - staying put",
                 config_link_mode_name(s_running), esp_err_to_name(err));
        return err;
    }
    s_running = LINK_MODE_OFF;

    /* Let the outgoing stack's memory actually come back before asking for the
     * incoming one's. */
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "free heap after teardown: %u bytes",
             (unsigned)esp_get_free_heap_size());

    err = bring_up(mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bring-up of %s failed - trying to return to gsm",
                 config_link_mode_name(mode));
        bring_up(LINK_MODE_GSM);
        return err;
    }

    /* Persist only on success: a mode that would not start must not be what
     * the device boots into next time. */
    config_set_link_mode(mode);
    return ESP_OK;
}
```

- [ ] **Step 5: Add the component to main's dependencies**

In `main/CMakeLists.txt`, `PRIV_REQUIRES`, after `wifi_sta`:

```
        link_mode
```

- [ ] **Step 6: Replace the GSM start in main.c**

Find the GSM auto-start block (`main.c` around line 1120, `"GSM Module: auto-start enabled, connecting..."`) and the `gsm_net_link_register()` call (around line 1156). Replace both with a single call to `link_mode_start()`, placed where `gsm_net_link_register()` is today — before `mqtt_svc_start()`, because the MQTT task asks `net_link_is_up()` as soon as it runs.

```c
#if defined(CONFIG_NCLE_GSM_ENABLE) || defined(CONFIG_NCLE_WIFI_ENABLE)
    /* Brings up whichever stack the operator last selected - gsm by default.
     * Registers that link with net_link too, so this must run before MQTT
     * starts: the MQTT task asks net_link_is_up() as soon as it runs and a
     * link registered late reads as permanently offline until the next poll. */
    link_mode_start();
#endif
```

Add `#include "link_mode.h"` to the include block.

- [ ] **Step 7: Build**

```bash
idf.py build
```

Expected: PASS.

- [ ] **Step 8: Flash and verify gsm mode is unchanged**

```bash
idf.py -p COM4 flash monitor
```

Expected in the log:
```
I (xxxx) LINK_MODE: stored mode: gsm
I (xxxx) LINK_MODE: mode: gsm
I (xxxx) NCLE_MAIN: Memory: Free heap: 12xxxx bytes
```

**Free heap must be ~124 KB**, within a few KB of today's figure. If it is near 43 KB, WiFi is being initialised at boot — find out where and fix it before going further. This is the check the whole design rests on.

Then confirm the GSM link still reaches the broker and a reading still publishes, exactly as before.

- [ ] **Step 9: Commit**

```bash
git add components/connectivity/link_mode main/CMakeLists.txt main/main.c
git commit -m "feat: link_mode component - bring up exactly one connectivity stack

Owns the gsm|wifi|off selection: reads the stored mode at boot, brings that
stack up, and switches at runtime by tearing the outgoing one down before
bringing the incoming one up. The only file that knows both GSM and WiFi
exist; neither component references the other.

Down-before-up is not tidiness - both stacks initialised leaves 43 KB of heap
against the ~45 KB FOTA needs. A failed switch returns to gsm and does NOT
persist the failed mode, so a device cannot be left booting into a link that
would not start."
```

---

### Task 7: The set_link_mode command

**Files:**
- Modify: `components/connectivity/link_mode/link_mode.c`
- Modify: `components/base/cmd_parser/include/cmd_parser.h`
- Modify: `components/base/cmd_parser/cmd_parser.c:420-437`

**Interfaces:**
- Consumes: `cmd_parser_register()`, `send_response()` — follow the pattern in `wifi_sta.c:189-235`, where the WiFi product registers its own handlers from inside its component rather than adding them to cmd_parser.
- Produces: BLE/USB command `{"command":"set_link_mode","mode":"gsm"|"wifi"|"off"}#` and `{"command":"get_link_mode"}#`.

- [ ] **Step 1: Add the command name macros**

In `cmd_parser.h`, beside `CMD_WIFI_CONFIG`:

```c
#define CMD_SET_LINK_MODE       "set_link_mode"
#define CMD_GET_LINK_MODE       "get_link_mode"
```

- [ ] **Step 2: Write the handlers in link_mode.c**

```c
#include "cmd_parser.h"
#include "cJSON.h"

static void handle_set_link_mode(const cJSON *root, cmd_source_t source)
{
    const cJSON *m = cJSON_GetObjectItem(root, "mode");
    if (!cJSON_IsString(m) || m->valuestring[0] == '\0') {
        send_response("set_link_mode_failed", STATUS_ERR,
                      "{\"reason\":\"missing_mode\"}");
        return;
    }

    uint8_t mode;
    if      (strcmp(m->valuestring, "gsm")  == 0) mode = LINK_MODE_GSM;
    else if (strcmp(m->valuestring, "wifi") == 0) mode = LINK_MODE_WIFI;
    else if (strcmp(m->valuestring, "off")  == 0) mode = LINK_MODE_OFF;
    else {
        send_response("set_link_mode_failed", STATUS_ERR,
                      "{\"reason\":\"unknown_mode\"}");
        return;
    }

    /* Answer BEFORE switching: switching to wifi tears GSM down, and if the
     * command arrived over MQTT the reply would never reach anyone. It also
     * takes seconds, and the app should not be left waiting. */
    char data[48];
    snprintf(data, sizeof(data), "{\"mode\":\"%s\",\"state\":\"switching\"}",
             m->valuestring);
    send_response("set_link_mode", STATUS_OK, data);

    link_mode_switch(mode);
}

static void handle_get_link_mode(const cJSON *root, cmd_source_t source)
{
    (void)root; (void)source;
    char data[64];
    snprintf(data, sizeof(data), "{\"mode\":\"%s\",\"stored\":\"%s\"}",
             config_link_mode_name(link_mode_current()),
             config_link_mode_name(config_get_link_mode()));
    send_response("get_link_mode", STATUS_OK, data);
}
```

Check the exact `cmd_parser_register` handler signature and `send_response` spelling in `wifi_sta.c:189-235` and match them — the sketch above follows that pattern but the real signature governs.

- [ ] **Step 3: Register them in link_mode_start()**

At the top of `link_mode_start()`, before reading the mode:

```c
    cmd_parser_register(CMD_SET_LINK_MODE, handle_set_link_mode);
    cmd_parser_register(CMD_GET_LINK_MODE, handle_get_link_mode);
```

- [ ] **Step 4: Block set_link_mode over MQTT**

In `cmd_parser.c`, the risky-command block at `:420-437` already refuses `wifi_config` and `wifi_erase` from `CMD_SRC_MQTT`. Add `set_link_mode` to the same list:

```c
            strcmp(cmd, CMD_SET_LINK_MODE) == 0 ||
```

The reason is concrete: switching to WiFi over MQTT tears down the GSM link that carried the command, and if the WiFi credentials are wrong the device is left unreachable from the cloud with nobody on site expecting it. `get_link_mode` stays allowed remotely.

- [ ] **Step 5: Build**

```bash
idf.py build
```

Expected: PASS.

- [ ] **Step 6: Flash and test the command over BLE**

```bash
idf.py -p COM4 flash monitor
```

Send over BLE:
```
{"command":"get_link_mode"}#
```
Expected: `{"response_message":"get_link_mode","status_code":0,"data":{"mode":"gsm","stored":"gsm"}}`

```
{"command":"set_link_mode","mode":"off"}#
```
Expected: the reply arrives, then `LINK_MODE: stopping GSM` and `LINK_MODE: mode: off`. Free heap should rise as GSM's stack is released — note the figure.

Then take an MA reading. Expected: `Link down - reading handed to the app over BLE (app=1)` and **no** `Buffered reading` line, because `net_link_is_up()` is false. This is the Task-from-last-week behaviour, now reachable deliberately.

```
{"command":"set_link_mode","mode":"gsm"}#
```
Expected: GSM comes back up and reaches the broker.

- [ ] **Step 7: Commit**

```bash
git add components/connectivity/link_mode/link_mode.c components/base/cmd_parser/include/cmd_parser.h components/base/cmd_parser/cmd_parser.c
git commit -m "feat: set_link_mode / get_link_mode commands

Refused over MQTT alongside wifi_config and wifi_erase: switching to wifi
tears down the GSM link the command arrived on, and wrong credentials would
leave the device unreachable from the cloud with nobody on site expecting it.
The reply is sent before the switch starts, for the same reason."
```

---

### Task 8: End-to-end WiFi mode on hardware

No new code unless this finds something. This is the task that proves the feature.

**Files:** none expected.

**Interfaces:** none.

- [ ] **Step 1: Provision WiFi credentials over BLE**

With the device in `gsm` mode, send:
```
{"command":"wifi_config","ssid":"<your SSID>","password":"<your password>"}#
```

Expected: `config_update_failed` or similar — **WiFi is not running in gsm mode**, so provisioning cannot test the credentials. If that is what happens, it is a real gap: the operator must be able to enter credentials *before* switching, or the switch will always fail the first time.

If it fails, **stop and report**. The fix is to allow `wifi_config` to store credentials without requiring the stack to be up, deferring the trial connection until the mode switches. That is a small change to `handle_wifi_config` but it is a design decision, not a detail.

- [ ] **Step 2: Switch to WiFi**

```
{"command":"set_link_mode","mode":"wifi"}#
```

Expected in the log:
```
LINK_MODE: switching gsm -> wifi
LINK_MODE: stopping GSM
LINK_MODE: free heap after teardown: <figure>
LINK_MODE: mode: wifi
WIFI_STA: STA started
NET_LINK: link registered: wifi
```
then an association and an IP.

**Record the free heap figure after teardown and after WiFi is up.** This is the number the whole plan has been working toward.

- [ ] **Step 3: Verify the cloud path over WiFi**

Expected: `MQTT_SVC: Connected to broker`.

Take an MA reading with a WM weight. Expected: `SF: Buffered reading` then `MQTT_SVC: Flushed ... (published=N, buffered left=0)`, and the reading appears on `clv4/<id>/data` in MQTTX with its `"wm"` field. Identical to the GSM path — the application layer does not know which link carried it.

- [ ] **Step 4: Verify the buffer rule over WiFi**

Switch the router off, or move out of range. Expected: MQTT disconnects, `net_link_is_up()` goes false, and readings are handed to the app rather than buffered — the same rule as GSM, no special case.

Restore the router. Expected: reconnect, and no flush because nothing was buffered.

- [ ] **Step 5: Verify the mode survives a reboot**

Power-cycle the device while in `wifi` mode.

Expected:
```
LINK_MODE: stored mode: wifi
LINK_MODE: mode: wifi
```
and **no GSM initialisation at all** — free heap at boot should reflect WiFi only, not both.

- [ ] **Step 6: Verify BLE recovery from a bad WiFi**

With the device in `wifi` mode, change the router's password so the device can no longer associate. Confirm the operator can still connect over BLE and send:
```
{"command":"set_link_mode","mode":"gsm"}#
```
and that GSM comes back.

This is the failure mode that makes persistence safe. If BLE is unreachable here, the coexistence change in Task 4 needs revisiting.

- [ ] **Step 7: Commit any fixes found**

If Steps 1–6 required code changes, commit them with a message naming which step found the problem.

---

### Task 9: FOTA over WiFi

The memory question the operator raised, answered on hardware rather than by calculation.

**Files:**
- Modify: `CMakeLists.txt` (version bump only)

**Interfaces:** none.

- [ ] **Step 1: Bump the version**

In `CMakeLists.txt`, change `NCLE_FIRMWARE_VERSION` from `"2.1.0.1000"` to `"2.1.0.1001"`.

- [ ] **Step 2: Build and publish the image**

```bash
idf.py build
```

Upload `build/NCLite_ESP32S3.bin` to a GitHub release, as in the earlier GSM FOTA tests.

- [ ] **Step 3: Put the device in WiFi mode and record the heap**

```
{"command":"set_link_mode","mode":"wifi"}#
{"command":"get_status"}#
```

Record free heap before starting the download.

- [ ] **Step 4: Trigger the update**

Send the FOTA command with the release URL, as in the GSM tests.

Expected: `{"fota":"downloading"}` with progress in 10% steps, MQTT suspended for the download (freeing ~45 KB), then `{"fota":"success"}` and a reboot into the new version.

**If it fails with a memory error**, that is the answer to the operator's question and it is not a defect in this plan — report the figures. The fallback is to suspend more aggressively during download, or to accept that FOTA runs in GSM mode only.

- [ ] **Step 5: Confirm the new version and slot**

Expected after reboot:
```
NCLE_MAIN:    Firmware version: 2.1.0.1001
OTA: Running partition: ota_1
LINK_MODE: stored mode: wifi
```

The mode must survive the update — an OTA that reset the device to GSM mode would strand a site that has no SIM.

- [ ] **Step 6: Commit**

```bash
git add CMakeLists.txt
git commit -m "chore: bump to 2.1.0.1001 for the WiFi-mode FOTA test

Verifies the memory question raised when this feature was designed: whether a
firmware update completes with the WiFi stack resident rather than GSM's.
Records the free-heap figures during download."
```

---

### Task 10: Documentation

**Files:**
- Modify: `docs/GSM_PORT_PLAN.md` or create `docs/link-modes.md`
- Modify: `README.md` if it lists commands

- [ ] **Step 1: Document the three modes**

Cover: what each mode does, the command to switch, that the choice persists, that BLE is always the way back, and the measured heap figures for each mode. The heap figures are the reason the design is shaped this way and will not be obvious to whoever reads this next.

- [ ] **Step 2: Document the app-facing commands**

`set_link_mode`, `get_link_mode`, and the existing `wifi_config` / `wifi_status` / `wifi_erase`, with their JSON shapes — the app team needs this.

- [ ] **Step 3: Commit**

```bash
git add docs
git commit -m "docs: link modes and the app-facing switch commands"
```

---

### Task 11: Prove WiFi is fully removable

The whole feature must be deletable at build time. A GSM-only product line ships with `CONFIG_NCLE_WIFI_ENABLE=n` and gets today's firmware back: no WiFi code, no flash cost, no heap cost, no behaviour change.

This is the last task because it can only be checked once everything else exists — but the `#ifdef` guards it tests are written as each earlier task goes in, not retrofitted here.

**Files:**
- Verify (and fix if needed): `components/connectivity/link_mode/link_mode.c`, `main/main.c`, `components/base/config/device_config.c`, `components/base/cmd_parser/cmd_parser.c`

**Interfaces:**
- Produces: a build that is behaviourally identical to the pre-WiFi firmware when the symbol is off.

- [ ] **Step 1: Audit every WiFi reference for a guard**

```bash
grep -rn 'wifi' --include=*.c --include=*.h --include=*.txt   main components/connectivity/link_mode components/base/config components/base/cmd_parser   | grep -v '^components/connectivity/wifi_sta/'
```

Every hit outside the `wifi_sta` component itself must sit inside `#ifdef CONFIG_NCLE_WIFI_ENABLE`. Three places need care:

- `link_mode.c` — the `LINK_MODE_WIFI` cases in `bring_up()` and `tear_down()` are already guarded. Also guard the *command handler's* `"wifi"` branch in `handle_set_link_mode`, so a WiFi-less build answers `unknown_mode` rather than accepting a mode it cannot enter.
- `main/CMakeLists.txt` — `wifi_sta` in `PRIV_REQUIRES`. CMake has no `#ifdef`, so wrap it:
  ```cmake
  if(CONFIG_NCLE_WIFI_ENABLE)
      list(APPEND priv_requires wifi_sta)
  endif()
  ```
  or leave it listed and rely on the component's own Kconfig gating its sources — check which pattern the project already uses for `gsm` before choosing.
- `device_config.c` — `LINK_MODE_WIFI` is just a constant, harmless to keep. But `config_set_link_mode()` must reject it when WiFi is not compiled in, or a stale NVS value from a WiFi build would leave a GSM-only device trying to enter a mode that does not exist.

  ```c
  esp_err_t config_set_link_mode(uint8_t mode)
  {
      if (mode > LINK_MODE_OFF) {
          return ESP_ERR_INVALID_ARG;
      }
  #ifndef CONFIG_NCLE_WIFI_ENABLE
      if (mode == LINK_MODE_WIFI) {
          return ESP_ERR_NOT_SUPPORTED;   /* not built with WiFi */
      }
  #endif
      g_device_config.link_mode = mode;
      return config_save();
  }
  ```

- [ ] **Step 2: Handle a stored wifi mode in a WiFi-less build**

A device flashed with a WiFi build, switched to `wifi`, then re-flashed with a GSM-only build would read `LINK_MODE_WIFI` from NVS and find no stack to start. `link_mode_start()`'s fallback already catches this — `bring_up()` returns `ESP_ERR_INVALID_ARG` for an unbuilt mode and the function falls back to gsm — but make it explicit rather than incidental:

In `link_mode_start()`, after reading the stored mode:

```c
#ifndef CONFIG_NCLE_WIFI_ENABLE
    if (mode == LINK_MODE_WIFI) {
        ESP_LOGW(TAG, "stored mode is wifi but this build has no WiFi - using gsm");
        mode = LINK_MODE_GSM;
        config_set_link_mode(LINK_MODE_GSM);   /* stop asking every boot */
    }
#endif
```

- [ ] **Step 3: Build with WiFi off**

```bash
idf.py -D CONFIG_NCLE_WIFI_ENABLE=n build
```

Or set it in `sdkconfig.defaults` and rebuild. Expected: PASS, with no reference to any `esp_wifi_*` symbol.

- [ ] **Step 4: Compare the binary against the pre-WiFi baseline**

```bash
idf.py size
```

Expected: **~1,466 KB**, matching commit `4db6383` (the last build before this feature). If it is materially larger, WiFi code is still being linked — find it with:

```bash
xtensa-esp32s3-elf-nm build/NCLite_ESP32S3.elf | grep -i 'esp_wifi\|wifi_sta'
```

Expected: no output.

- [ ] **Step 5: Flash and verify behaviour is unchanged**

```bash
idf.py -p COM4 flash monitor
```

Expected:
- Free heap at boot **~124 KB**
- `LINK_MODE: mode: gsm`
- No `WIFI_STA` lines at all
- GSM connects, MQTT connects, a reading publishes with its `"wm"` weight
- `{"command":"set_link_mode","mode":"wifi"}#` answers `unknown_mode`
- `{"command":"set_link_mode","mode":"off"}#` still works — off is not a WiFi feature

- [ ] **Step 6: Restore the WiFi-enabled default and rebuild**

Leave the shipping default at `CONFIG_NCLE_WIFI_ENABLE=y`. Confirm the WiFi build still passes Task 8's checks after any guards added here.

- [ ] **Step 7: Commit**

```bash
git add -A
git commit -m "feat: make WiFi fully removable with CONFIG_NCLE_WIFI_ENABLE=n

A GSM-only build gets today's firmware back: 1,466 KB, ~124 KB free heap, no
esp_wifi symbols linked, GSM and BLE untouched. Verified by building both
ways and comparing size, symbols and boot heap.

Also handles the awkward case of a device whose NVS says 'wifi' being
re-flashed with a GSM-only build: link_mode_start rewrites the stored mode to
gsm rather than failing to bring anything up on every boot."
```

---

## Self-Review

**Spec coverage.** Every point of the agreed design maps to a task: three modes (2, 6), default gsm (2, 6), BLE switch command (7), persistence across reboot (2, 8 step 5), one radio at a time (6, verified in 8), BLE-only in off mode (7 step 6), FOTA under WiFi (9), WiFi fully removable at build time (11). The lazy-init constraint that the spike established is enforced by Task 6 step 8 and re-checked in Task 8 step 5.

**Known gaps deliberately left.**
- Task 6 step 1 may find that GSM has no safe teardown. That is a genuine unknown — the project memory records a crash tearing the modem down mid-session — and the plan stops rather than guessing.
- Task 8 step 1 may find that WiFi credentials cannot be entered while in gsm mode. Also a real gap, also flagged to stop rather than improvise.
- The accepted gap from the previous change stands: with no link up and no phone listening, a reading is neither sent nor stored. Unchanged by this work.

**Type consistency.** `config_get_link_mode` / `config_set_link_mode` / `config_link_mode_name` are defined in Task 2 and used with those exact names in Tasks 6 and 7. `wifi_net_link_register` is defined in Task 3 and called in Task 6. `link_mode_start` / `link_mode_switch` / `link_mode_current` are defined in Task 6 and used in Tasks 6 and 7. `LINK_MODE_GSM` / `_WIFI` / `_OFF` are defined once in Task 2.

**Sketched, not final.** The handler bodies in Task 7 follow the pattern in `wifi_sta.c:189-235` but the real `cmd_parser_register` signature and `send_response` spelling govern; the task says so explicitly. Task 6's `gsm_task_stop()` is assumed and verified in step 1 before use.
