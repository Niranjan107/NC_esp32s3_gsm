# GSM Port Plan — Sharing the Cloud Layer with the WiFi Product

**Project:** `D:\ESp32S3_CLV4_GSM\nitara.connector.esp32s3.gsm`
**Source project:** `D:\ESP32S3-CLV4-WIFI\nitara.connector.esp32s3`
**Target:** ESP32-S3 · ESP-IDF v5.5 · Quectel EC200U-class modem

---

## Goal

Run both products from one shared codebase where **only the connectivity layer
differs**. The WiFi product is already organised in three layers:

```
base/           meters, printer, BLE, commands, settings, OTA
                -> identical in every product, never edited per variant
connectivity/   how the device reaches the internet
                -> THE ONLY LAYER THAT DIFFERS  (wifi_sta | gsm)
application/    MQTT publish, store-and-forward, weight merge, HTTPS FOTA
                -> portable, moves to the GSM product unchanged
```

The GSM project's components are currently flat and older. This plan brings it onto
the same structure so the hardware-proven cloud layer can be reused as-is.

## Why Step 1 blocks everything

`components/gsm` currently performs HTTP **itself** using AT commands
(`AT+QHTTPURL`, `AT+QHTTPGET`, `AT+QHTTPPOST`), and PPP is disabled.

The cloud layer being reused uses **TCP sockets** (`esp-mqtt`, `esp_http_client`).
It cannot run over AT-command HTTP. So the modem must first become a normal
ESP-IDF network interface.

> **Do not start Steps 2–6 before Step 1 is demonstrated.** They are easier, and
> they will appear to succeed while failing silently later.

---

## The golden rule for testing

**Flash and test between every single step.**

If Steps 2–4 are done in one go and the device goes quiet, there are three
candidates and no way to separate them. One step → one flash → one known-good
state to fall back to.

Where a result is ambiguous, **change one variable and re-run** rather than
reasoning about noisy data. (See the 5000 ms re-run in Step 1.)

---

## Step 1 — PPP: give the modem a real IP interface

**This is the only real engineering work. Everything after it is moving files.**

### What to implement

1. **`CONFIG_LWIP_PPP_SUPPORT=y`** — and verify it actually took effect.
   ⚠️ `sdkconfig:2154` currently reads `# CONFIG_LWIP_PPP_SUPPORT is not set`. The
   committed `sdkconfig` **overrides** `sdkconfig.defaults`, so setting only the
   defaults file changes nothing. Delete and regenerate, or use `menuconfig`.

2. **Add `espressif/esp_modem`**, bring the modem into data mode
   (`AT+CGDATA="PPP",1` or `ATD*99#`) so it creates an `esp_netif` and obtains an IP.

3. **⚠️ esp_modem must OWN the UART.**
   `gsm.c:76-81` currently calls `uart_driver_install` / `uart_param_config` /
   `uart_set_pin` itself, and the AT helpers use raw `uart_write_bytes` /
   `uart_read_bytes` under `s_uart_mutex`. Two drivers on one UART either fail at
   install or silently corrupt RX — and once PPP is up, those raw reads will consume
   PPP frames. Keep the four diagnostic functions' signatures and behaviour
   unchanged, but route their transport through `esp_modem_at()`.

4. **Use CMUX (`ESP_MODEM_MODE_CMUX`).**
   `gsm_task` polls status every 10 s. If each RSSI read drops the data link into
   command mode and back, the PPP session flaps every 10 seconds. CMUX gives a
   command channel alongside the data channel.

5. **Add the networking prerequisites** — neither exists in this project today:
   ```c
   esp_netif_init();
   esp_event_loop_create_default();
   ```

6. **Add `gsm` to `main/CMakeLists.txt` `PRIV_REQUIRES`** — currently missing, so
   `main.c` cannot call the GSM component at all.

7. **APN configurable** — read from NVS, falling back to the existing
   `CONFIG_NCLE_GSM_APN` (`"airtelgprs.com"`). Do not introduce a second hard-coded
   default that can disagree with the Kconfig one.

8. **Expose** `bool gsm_pdp_is_active(void);`

### Constraints

- **Keep** `gsm_get_signal_strength()`, `gsm_get_iccid()`, `gsm_get_network_status()`,
  `gsm_get_phone_number()` — still needed for diagnostics. Only the transport
  underneath changes.
- **Do not delete** `gsm_http_get` / `gsm_http_post` until PPP is proven.
- **Keep registration and data-link state separate** (`gsm_get_network_status()` and
  `gsm_pdp_is_active()`) rather than collapsing them into one "connected" boolean.
  A SIM can be registered and still have no working data connection — Step 6 depends
  on this distinction.

### ✅ How Step 1 is tested

All five must be demonstrated on hardware.

| # | Test | Pass |
|---|---|---|
| 1 | `CONFIG_LWIP_PPP_SUPPORT=y` in **`build/config/sdkconfig.h`** | present (not just in `sdkconfig.defaults`) |
| 2 | Boot log | shows IP **plus gateway and DNS** via `esp_netif` |
| 3 | `ping 8.8.8.8` using **standard lwIP** — not `AT+QPING` | succeeds |
| 4 | **Stock `esp_http_client`** GET to `http://example.com` | succeeds with no custom transport, no modem special-casing, no `AT+Q…` anywhere in that path |
| 5 | Continuous ping ≥3 min while `gsm_task` polls at its normal 10 s cadence | **no loss at ~10 s boundaries** |

**Report back:** boot log (IP + gateway + DNS), the GET code **verbatim**, the full
unedited ping output, and the list of files changed.

#### Reading test #5 correctly

The signature is **periodicity, not presence**. Mobile links drop packets normally.

| Pattern | Reading |
|---|---|
| 3 scattered losses at random offsets | ordinary cellular — not evidence of anything |
| Loss clustered at ~10 s, 20 s, 30 s… | the poll boundary — **mode-flapping, CMUX not used** |
| Several consecutive packets lost, repeating | link genuinely dropping and re-establishing |

**If ambiguous:** set `CONFIG_NCLE_GSM_TASK_POLL_INTERVAL_MS` to `5000` and re-run.
If the loss pattern follows the poll interval, it is the polling. If it stays put, it
is the network. One rebuild, no interpretation needed.

#### Why tests #2, #4 and #5 are the decisive ones

They guard against an AT-command wrapper that *looks* like a socket interface:

- A wrapper can trivially print "got IP" — it cannot produce a populated **gateway
  and DNS** entry that stock `esp_http_client` then resolves against.
- If sockets are genuinely real, **unmodified** `esp_http_client` just works.
- Periodic ping loss exposes mode-flapping that a short demo would hide.

Faking all three convincingly is more work than doing the port properly.

---

## Step 2 — Layer restructure

```bat
mkdir components\base components\connectivity components\application
git mv components\ble_spp components\ma_uart components\wm_uart components\printer_uart ^
       components\battery components\config components\cmd_parser components\common ^
       components\ota components\soft_uart_rmt components\wm_uart_validator components\base\
git mv components\gsm components\connectivity\
```

Root `CMakeLists.txt`:

```cmake
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/components/base"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/connectivity"
    "${CMAKE_CURRENT_SOURCE_DIR}/components/application"
)
set(EXCLUDE_COMPONENTS esp_lcd)
```

Also in this step:

- ⚠️ **`soft_uart_rmt` must go into `base/`** — it is **not** an optional experiment.
  It drives WM RX on GPIO44 in the current build (`CONFIG_NCLE_WM_USE_SOFT_UART=y`)
  and is a hard dependency of `main`.
- **`EXCLUDE_COMPONENTS esp_lcd`** — same board as WiFi; its RGB panel driver crashes
  the xtensa GCC 14.2 build (internal compiler error in the `ira` pass).
- **`add_compile_definitions` → `idf_build_set_property`**, and move the GSM version
  from `2.0.0.1000` onto the WiFi scheme.
- ⚠️ **Fix `rsource` paths in `main/Kconfig.projbuild`.** The GSM project has none
  today, but three arrive with the components copied in Steps 3–4. This is what broke
  the WiFi build during its own restructure.
- Extend `main/CMakeLists.txt` `PRIV_REQUIRES` for the new component locations.

### ✅ How Step 2 is tested

**This step moves files. It must change no behaviour at all.**

| Test | Pass |
|---|---|
| `idf.py fullclean build` | succeeds — a **clean** build, not incremental (an incremental build can succeed on stale paths and hide a broken `EXTRA_COMPONENT_DIRS`) |
| Build output component list | every expected component present — a silently missing component still links |
| `idf.py menuconfig` | opens and shows the **NCLite CLEV4** menu → proves `rsource` paths resolve |
| Flash → weigh something | WM weight still arrives over BLE → proves the `soft_uart_rmt` / GPIO44 path survived |
| Flash → MA reading | still arrives |
| Flash → print a receipt | still works |

---

## Step 3 — Sync `base/` from the WiFi project

**Must happen BEFORE Step 4** — `application/` depends on these.

| Copy from WiFi | Why |
|---|---|
| `base/common/` | `net_link.h/.c` — required by the whole application layer |
| `base/cmd_parser/` | `cmd_parser_register()` — needed for Step 5 |
| `base/ma_uart/`, `base/wm_uart/` | frame-start and value callbacks — needed for the weight merge |
| `base/printer_uart/` | timing marks |

Also: `ble_spp.h` device name — the WiFi variant uses a `"W"` suffix so both don't
advertise the same name. Pick a distinct GSM suffix.

### ✅ How Step 3 is tested

**You are replacing working code with newer working code — the risk is silent
behavioural drift, so test what already worked.**

| Test | Pass |
|---|---|
| Every existing BLE command | returns what it used to — check `diag` especially |
| WM value callback | fires on each weight |
| MA frame-start callback | fires on each reading |
| Printer | timing marks unchanged, receipt still correct |
| Flash **between Step 3 and Step 4** | device fully working before application/ arrives |

> If you sync base and copy application together and something breaks, the fault
> could be in either. Flash in between.

---

## Step 4 — Copy the application layer

```bat
xcopy /E /I "D:\ESP32S3-CLV4-WIFI\nitara.connector.esp32s3\components\application" ^
            "D:\ESp32S3_CLV4_GSM\nitara.connector.esp32s3.gsm\components\application"
```

Then create `mqtt_secrets.h` from `mqtt_secrets.h.example`.

### ✅ How Step 4 is tested

**The test IS that you don't edit anything.**

| Test | Pass |
|---|---|
| Build with **zero edits** inside `components/application/` | succeeds |
| If any edit is needed | ❌ **STOP** — `net_link` or the Step 3 base sync is wrong. Fix that instead; do not patch `application/`. |
| `partitions.csv` has a LittleFS partition | present — store-and-forward needs it, and it fails at **runtime**, not build time |

This is the whole point of the architecture. If `application/` needs editing, the
layering has been broken somewhere upstream.

---

## Step 5 — Wire GSM in

In `gsm.c`, at the end of init:

```c
static bool gsm_link_is_up(void) { return gsm_pdp_is_active(); }

static int gsm_link_status_json(char *buf, size_t size) {
    gsm_status_t s; gsm_task_get_last_status(&s);
    return snprintf(buf, size,
        "{\"connected\":%s,\"rssi\":%d,\"operator\":\"%s\",\"iccid\":\"%s\"}",
        s.connected ? "true":"false", s.rssi, s.operator_name, s.iccid);
}

static const net_link_t gsm_link = {
    .name = "gsm", .is_up = gsm_link_is_up, .status_json = gsm_link_status_json };
net_link_register(&gsm_link);

cmd_parser_register("apn_config",  handle_apn_config);
cmd_parser_register("sim_status",  handle_sim_status);
cmd_parser_register("gsm_status",  handle_gsm_status);
```

⚠️ **`gsm_status_t` must be extended first.** It currently has `alive, net_status,
registered, rssi, ber, bars, module_info[64]` — there is **no** `connected`,
`operator_name` or `iccid`. Operator needs `AT+COPS?` wrapped (not currently wrapped);
ICCID can use the existing `gsm_get_iccid()`. Map `connected` to **PDP-active**, not
to `registered`.

In `main.c`:

```c
static void on_ma_frame_start(bool terminator_framed) {
    wm_capture_set_continuous(terminator_framed);
    wm_capture_start();
}
static void on_wm_value(const char *v) { wm_capture_feed(v); }

ma_uart_set_frame_start_callback(on_ma_frame_start);
wm_uart_set_value_callback(on_wm_value);
```

Also replace the WiFi BLE-status timer block with the GSM equivalent — this is real
work, not a copy, since `gsm_task` has its own callback shape.

### ✅ How Step 5 is tested

| Test | Pass |
|---|---|
| `diag` command | returns `"link":"gsm"` with SIM and signal — **no SSID** |
| Take a milk reading | MQTT publish arrives at the broker |
| Weight merge | the MQTT message contains the WM weight |
| **Store-and-forward** | pull the SIM (or disable PDP) mid-run → send several readings → restore → **buffered readings arrive** |

> The store-and-forward test is the one people skip, and it is the one that matters
> in the field. Do not sign off Step 5 without it.

---

## Step 6 — `net_probe`: honest connectivity

Report **four** states instead of one "connected":

```
Registered  ->  Link Up  ->  Internet Up  ->  Cloud Up
```

| State | Meaning | If it fails |
|---|---|---|
| Registered | found the mobile tower (`AT+CREG`) | SIM / antenna / coverage |
| Link up | PDP active, got an IP | wrong APN |
| Internet up | a **public** host replied | data pack finished / carrier issue |
| Cloud up | **our** MQTT broker replied | our server — **the device is fine** |

### Why this exists

`is_up()` today means *"got an address"*, not *"the internet works"*. A router with
no WAN, or a SIM with no balance, still reports `connected: true`.

**The probe must ask a stranger, not our own server.** If "internet up" were defined
as "MQTT connected", then whenever the broker went down every device in the field
would report a SIM fault and technicians would be sent to healthy machines.

- Probe = DNS query to `8.8.8.8` (~80 bytes) — proves PDP + routing + DNS in one
  round trip. Ping alone is weaker: it does not prove DNS, and some carriers
  deprioritise ICMP.
- `net_link_t.is_up()` returns **Internet up**. `Cloud up` is reported in
  `status_json` but **never gates anything**.
- Probe on link-up, on send failure, and a slow 15-min background check — **not** a
  fast timer (metered SIM). Successful application traffic counts as proof, so a busy
  device almost never probes.
- Require **2 consecutive failures** before declaring down, so one lost packet does
  not flap the state.

### ✅ How Step 6 is tested

**Test on the WiFi bench first** — you can create the failure with your hands, whereas
staging an exhausted SIM data pack is awkward.

| Test | Pass |
|---|---|
| Unplug the **WAN cable** from the router, router still running | `connected:true, internet:false` (today it wrongly says `connected:true`) |
| Stop the MQTT broker, internet fine | `internet:true, cloud:false` — **device not blamed** |
| Plug the cable back in | recovers **without a reboot** |
| Busy device sending readings | probe count stays near zero — traffic is counted as proof |

Then port to GSM, where it arrives already validated.

---

## Two decisions to settle before Step 2

**1. One repo, or two?**
Copying `base/` and `application/` between repos means two copies that diverge the
moment someone fixes a bug in one. Since `connectivity/` is the only difference, a
single repo where a Kconfig flag selects the connectivity component is genuinely
"one codebase, two builds". Cheap to decide now, expensive after the folders are
copied. If two repos are kept, make `base/` + `application/` a git submodule.

**2. A uniform connectivity entry point.**
`main.c` is currently the only file that differs structurally between products, so it
is the one that will drift. If `wifi_sta` and `gsm` both expose `conn_start()` /
`conn_register_ble_status_cb()`, `main.c` becomes identical and the per-product diff
shrinks to `PRIV_REQUIRES`. Decide before Step 5 wires GSM in its current shape.

---

## Reference — verified repo facts

Confirmed by reading the code on 2026-08-24 (branch `feat/wm-on-soft-uart`):

- `sdkconfig:2154` — `# CONFIG_LWIP_PPP_SUPPORT is not set`
- `components/gsm/gsm.c:76-81` — installs the UART driver directly
- `esp_netif_init()` / `esp_event_loop_create_default()` — **absent** project-wide
- `main/CMakeLists.txt` — no `gsm` in `PRIV_REQUIRES`
- `components/gsm/include/gsm_task.h` — `gsm_status_t` has no `connected`,
  `operator_name` or `iccid`
- WiFi `main/Kconfig.projbuild:346-348` — three `rsource` lines
- WiFi `application/` and `base/` — **no** `esp_wifi` dependency (every match is a
  comment or a command-name string), so "copy `application/` unedited" is valid

### Pin assignments (unchanged by this plan)

| Function | GPIO | Interface |
|---|---|---|
| WM RX | 44 | RMT **soft UART** (not a hardware UART) |
| WM TX | 43 | disabled (`WM_UART_TX_PIN` = `-1`) |
| MA TX / RX | 40 / 39 | UART2 |
| Printer TX / RX | 17 / 18 | UART1 |
| GSM TX / RX | 16 / 15 | **UART0** |
| GSM PWRKEY / RST | 7 / 8 | GPIO |
| Status LED | 36 | GPIO |
| Battery ADC | 6 | ADC1_CH5 |

> WM uses the **soft UART**, which is what frees UART0 for GSM. The soft-UART
> loopback test defaults its TX to GPIO15 — the same pin as GSM RX. It is disabled;
> do not enable it while GSM is running.
