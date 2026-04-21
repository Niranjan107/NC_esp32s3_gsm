# GSM UART Expansion — Design Spec

**Date:** 2026-04-21
**Status:** Draft, awaiting user review
**Author:** collaborative (user + Claude)
**Target hardware:** ESP32-S3 (`CONFIG_SOC_UART_NUM=3`)
**GSM module:** Quectel EC200U-CN (LTE Cat 1, default 115200 baud, AT commands)

---

## 1. Problem

The product needs a fourth UART peripheral — for the Quectel EC200U-CN GSM/LTE module — on an ESP32-S3 that has **only three hardware UART controllers**, all currently allocated:

| UART | Peripheral | Pins | Baud | Direction |
|------|------------|------|------|-----------|
| UART0 | WM (Weighing Machine) | RX=44 | 9600 | RX-only |
| UART1 | Printer | TX=17, RX=18 | 9600 | bidirectional |
| UART2 | MA (Main App) | TX=40, RX=39 | 9600 | bidirectional |

The product requirement is to add GSM **without dropping any of the three existing peripherals** and without redesigning the PCB as a first move.

## 2. Strategy

Staged, risk-managed plan:

1. **Phase 1 — Parallel validation.** Build a generic RMT-based soft UART driver. Run it on the same GPIO as the WM hardware UART (via GPIO matrix fan-out, no hardware change). Both decoders see identical bytes. Compare live.
2. **Phase 1.5 — Baud sweep.** Characterize soft UART reliability across {9600, 19200, 38400, 57600, 115200} baud with BLE/WiFi load active.
3. **Phase 2 — Cutover.** If validation passes, migrate WM to the soft UART permanently, free UART0, assign UART0 to the GSM (EC200U-CN) at 115200.
4. **Phase 3 — Fallback (contingency).** If validation fails at required baud rates, add an I²C-to-UART expander (SC16IS752 preferred) to host the GSM externally.

Phase 1 is pure firmware and fully reversible. Phase 3 is only entered if Phase 1 objectively fails defined criteria.

**Scope of the first implementation plan:** Phase 1 + Phase 1.5 only. Phase 2 (cutover) and Phase 3 (fallback) each get their own follow-up spec + implementation plan, written only after the prior phase concludes. This spec is the root document that both follow-ups will reference.

## 3. Hardware constraint rationale

- ESP32-S3 has exactly 3 UART controllers. No software trick creates a 4th controller.
- Soft UART at 115200 on a BLE-active ESP32-S3 is unreliable with interrupt-based bit-banging. GSM asynchronous URCs (`+CMTI`, `+QIURC`, `+CEREG`) plus large TCP bursts make dropped bytes unacceptable.
- Conversely, the WM is **RX-only at 9600 baud**, sending short ASCII lines with `\n`/`\r` terminators. It is the single peripheral best suited to soft UART: low baud, one direction, forgiving line-oriented protocol, and a dropped reading is self-healing because the WM sends continuously.
- Therefore the migration target is WM → soft UART, freeing hardware UART0 for GSM.
- RMT is chosen over bit-bang because the RMT peripheral has hardware timing + FIFO + its own ISR, making it immune to BLE interrupt jitter. No other project code uses RMT (verified via grep).

## 4. Architecture

### 4.1 New components

```
components/
├── soft_uart_rmt/          NEW — generic, reusable RMT-based soft UART driver
│   ├── include/soft_uart_rmt.h
│   ├── soft_uart_rmt.c
│   ├── Kconfig.projbuild
│   └── CMakeLists.txt
├── wm_uart_validator/      NEW — Phase 1-only A/B comparison harness
│   ├── include/wm_uart_validator.h
│   ├── wm_uart_validator.c
│   ├── Kconfig.projbuild
│   └── CMakeLists.txt
└── wm_uart/                UNCHANGED during Phase 1
```

Existing components (`wm_uart`, `ma_uart`, `printer_uart`, `ble_spp`, etc.) are **not modified** during Phase 1. This guarantees zero regression risk for the shipping WM path while validation runs.

### 4.2 Runtime data flow during Phase 1

```
                                ┌──────────────────────────────┐
                                │    ESP32-S3 GPIO Matrix      │
                                │                              │
   WM TX ──────► GPIO 44 ──────►│  ├──► UART0_RXD_IN           │──► wm_uart (HW)      ─┐
                                │  └──► RMT_SIG_IN0_IDX        │──► soft_uart_rmt (SW)─┤
                                └──────────────────────────────┘                       │
                                                                                        ▼
                                                       ┌──────────────────────────────────┐
                                                       │  wm_uart_validator task          │
                                                       │  - aligns bytes by timestamp     │
                                                       │  - compares bytes, packets, JSON │
                                                       │  - emits periodic stats          │
                                                       └──────────────────────────────────┘
```

Single GPIO, two listeners, zero hardware modification. The WM sees a normal high-impedance input.

### 4.3 Component: `soft_uart_rmt`

**Purpose:** Generic RMT-based software UART RX (initial scope: RX-only; TX is out of scope for Phase 1). WM-agnostic.

**Public API:**

```c
typedef struct {
    int      gpio_num;         // RX pin
    int      baud_rate;        // 9600..115200
    uint8_t  data_bits;        // 8 (only 8 supported in v1)
    uint8_t  stop_bits;        // 1 (only 1 supported in v1)
    uint8_t  parity;           // 0=none (only none supported in v1)
    int      rmt_channel;      // RMT RX channel index (0..3 on S3)
    size_t   symbol_mem_size;  // RMT symbol buffer size (words); default 64
    size_t   byte_queue_depth; // FreeRTOS queue depth for decoded bytes
} soft_uart_rmt_config_t;

typedef struct {
    uint8_t  byte;
    int64_t  ts_us;            // esp_timer_get_time() at frame completion
    bool     frame_err;        // true = start/stop violation
} soft_uart_rmt_rx_t;

typedef void (*soft_uart_rmt_byte_cb_t)(const soft_uart_rmt_rx_t *rx, void *user_ctx);

esp_err_t soft_uart_rmt_init(const soft_uart_rmt_config_t *cfg, soft_uart_rmt_handle_t *out);
esp_err_t soft_uart_rmt_start(soft_uart_rmt_handle_t h);
esp_err_t soft_uart_rmt_stop(soft_uart_rmt_handle_t h);
esp_err_t soft_uart_rmt_register_byte_cb(soft_uart_rmt_handle_t h,
                                         soft_uart_rmt_byte_cb_t cb,
                                         void *user_ctx);
esp_err_t soft_uart_rmt_set_baud(soft_uart_rmt_handle_t h, int baud_rate);
esp_err_t soft_uart_rmt_deinit(soft_uart_rmt_handle_t h);

// Statistics for validation
typedef struct {
    uint64_t bytes_ok;
    uint64_t bytes_frame_err;
    uint64_t rmt_overflows;
    uint64_t queue_drops;
} soft_uart_rmt_stats_t;
void soft_uart_rmt_get_stats(soft_uart_rmt_handle_t h, soft_uart_rmt_stats_t *out);
void soft_uart_rmt_reset_stats(soft_uart_rmt_handle_t h);
```

**Internal decoder pipeline:**

1. `rmt_new_rx_channel` with `resolution_hz = max(baud_rate * 16, 1_000_000)` so tick resolution is finer than bit time.
2. `rmt_receive` called in a loop with:
   - `signal_range_min_ns = bit_time_ns / 4` (filter glitches < ¼ bit)
   - `signal_range_max_ns = bit_time_ns * 12` (end-of-frame idle detection; > full 10-bit frame)
3. Decoder task consumes RMT symbols via queue. For each symbol (`level`, `duration`):
   ```c
   uint32_t bits = (duration + bit_time / 2) / bit_time;   // rounded integer division
   if (bits == 0) bits = 1;
   if (bits > 10) bits = 10;
   // shift `bits` copies of `level` into a frame-building shift register
   ```
4. When shift register contains a full 10-bit frame (start=0, 8 data, stop=1), emit byte with `frame_err` flag set if start/stop violate.
5. Handle inter-frame idle by flushing any partial frame when RMT reports `done` after `signal_range_max_ns`.

**Kconfig options:**

- `CONFIG_SOFT_UART_RMT_DEFAULT_RESOLUTION_HZ` (default 1_000_000)
- `CONFIG_SOFT_UART_RMT_LOG_LEVEL`

### 4.4 Component: `wm_uart_validator`

**Purpose:** Phase 1-only harness that runs HW and soft WM drivers in parallel, compares outputs, and emits telemetry. Deleted or disabled via Kconfig after Phase 2.

**Public API:**

```c
typedef struct {
    int      shared_gpio;         // GPIO shared with wm_uart (default 44)
    int      rmt_channel;         // RMT channel for soft UART
    int      baud_rate;           // Must match wm_uart's active baud
    uint32_t report_interval_ms;  // Default 10_000
    uint32_t packet_quiet_ms;     // Default 50 (WM packet end gap)
} wm_uart_validator_config_t;

esp_err_t wm_uart_validator_init(const wm_uart_validator_config_t *cfg);
esp_err_t wm_uart_validator_start(void);
esp_err_t wm_uart_validator_stop(void);

typedef struct {
    int      baud_rate;
    uint64_t hw_bytes;
    uint64_t sw_bytes;
    uint64_t matched_bytes;
    uint64_t frame_errs;
    uint32_t hw_packets;
    uint32_t sw_packets;
    uint32_t json_matches;
    uint32_t json_mismatches;
    int64_t  avg_skew_us;
    int64_t  max_skew_us;
} wm_uart_validator_stats_t;

void wm_uart_validator_get_stats(wm_uart_validator_stats_t *out);
void wm_uart_validator_reset_stats(void);

// Test automation: sweep across baud rates, emit a report per step
esp_err_t wm_uart_validator_run_baud_sweep(const int *bauds, size_t n, uint32_t seconds_per_step);
```

**Wiring in `main.c` (Phase 1 only):**

```c
wm_uart_init();                // existing HW driver; owns UART0 on GPIO 44
wm_uart_start();

soft_uart_rmt_config_t cfg = {
    .gpio_num       = 44,      // SAME GPIO as wm_uart
    .baud_rate      = 9600,
    .data_bits      = 8,
    .stop_bits      = 1,
    .parity         = 0,
    .rmt_channel    = 0,
};
// soft_uart_rmt_init internally calls:
//   esp_rom_gpio_connect_in_signal(44, RMT_SIG_IN0_IDX, false);
// without changing the UART0_RXD_IN routing already established by wm_uart.

wm_uart_validator_config_t vcfg = { .shared_gpio=44, .rmt_channel=0,
                                    .baud_rate=9600, .report_interval_ms=10000,
                                    .packet_quiet_ms=50 };
wm_uart_validator_init(&vcfg);
wm_uart_validator_start();
```

**Hook points in existing `wm_uart`:** Validator needs per-byte timestamps from the HW driver. The `wm_uart` component currently exposes JSON-level and activity callbacks but no raw-byte hook. Phase 1 adds **one** additive function to `wm_uart` — the only modification to that component during Phase 1:

```c
// Added to wm_uart.h:
typedef void (*wm_uart_raw_byte_cb_t)(uint8_t byte, int64_t ts_us, void *ctx);
void wm_uart_set_raw_byte_callback(wm_uart_raw_byte_cb_t cb, void *ctx);
```

The default callback is NULL. When NULL, the HW driver executes its existing code path unchanged (zero regression risk). When set, the driver invokes the callback once per received byte, with `ts_us = esp_timer_get_time()` captured immediately after `uart_read_bytes` returns. Validator registers this callback during Phase 1 and deregisters (NULL) during Phase 2 cutover.

### 4.5 Comparison algorithm (validator)

Both sources push `tagged_byte_t` into a shared FreeRTOS queue:

```c
typedef struct {
    uint8_t  byte;
    int64_t  ts_us;
    uint8_t  source;    // 0=HW, 1=SOFT
    uint8_t  frame_err; // SOFT only
} tagged_byte_t;
```

Validator task maintains two rolling byte buffers (HW and SOFT), each keyed by time window. A byte is "paired" with the other source's byte if:
- Same logical position in their respective packets, AND
- `|ts_hw - ts_soft| < 2 × bit_time_us` (allow decoder latency skew)

Per packet (packet = bytes bracketed by `packet_quiet_ms` of line-silence):

- **Byte match**: `matched / max(|hw|, |sw|)`
- **Length delta**: `|sw| - |hw|` (negative = SW drops, positive = SW spurious)
- **Skew**: average/max `ts_soft - ts_hw` over paired bytes

Per packet-ending, the validator also feeds HW-bytes and SW-bytes through a minimal re-export of the WM packet parser (prefix stripping, non-printable filter, duplicate filter) and compares the resulting JSON strings.

### 4.6 Phase 1.5 — Baud sweep procedure

`wm_uart_validator_run_baud_sweep()` executes this sequence:

1. Stop HW + SW drivers.
2. Set WM device to baud B (manually documented per baud via physical WM DIP switch or serial command, depending on WM model — requires operator).
3. `wm_uart_set_baud(B)` + `soft_uart_rmt_set_baud(h, B)`.
4. Start HW + SW.
5. Run for `seconds_per_step` (default 600 seconds = 10 minutes) with BLE SPP enabled and a representative BLE client connected to simulate real RF load.
6. Collect `wm_uart_validator_stats_t` snapshot.
7. Log a row in the report table.

Bauds to sweep: `{9600, 19200, 38400, 57600, 115200}`. Primary target is 9600 (current WM config); higher bauds characterize soft UART ceiling.

### 4.7 Pass/fail criteria for Phase 2 cutover

**PASS** if, at the target baud (9600) over a 10-minute run under the BLE load defined in test T2 (§7.2):

- Byte match ≥ 99.9%
- Frame errors ≤ 1 per 10,000 bytes
- JSON mismatches = 0
- Max skew ≤ 2 × bit_time_us
- RMT overflows = 0

A secondary PASS at 19200 is required to prove headroom (WM will always be at 9600, but soft UART must have margin).

**FAIL** triggers Phase 3 (I²C-UART expander). No borderline "mostly works" cutover.

## 5. Phase 2 — Cutover plan

Executed only after Phase 1 passes.

1. Delete (or Kconfig-disable) `wm_uart_validator` component.
2. Refactor `wm_uart` to source its bytes from `soft_uart_rmt` instead of `uart_driver_install` / `uart_read_bytes`. Keep the existing public API (`wm_uart_init`, `_start`, `_set_baud`, JSON callback, etc.) unchanged so the rest of the firmware is untouched. Internal implementation swaps HW driver for a thin adapter that feeds bytes from `soft_uart_rmt` through the existing parser.
3. Remove `UART_NUM_0` usage from `wm_uart`. UART0 controller becomes free.
4. Add new `components/gsm_uart/` (separate future task) using the now-free UART0 for the EC200U-CN at 115200 baud. RX/TX pin selection and full GSM driver design are **out of scope for this document** — covered by a follow-up spec created when Phase 2 kicks off.
5. Update `sdkconfig` defaults:
   - Remove `CONFIG_NCLE_WM_UART_NUM` (no longer an HW UART controller number).
   - Add `CONFIG_NCLE_WM_SOFT_UART_RMT_CHANNEL = 0` (and related RMT config keys).
   - GSM-specific Kconfig keys (`CONFIG_NCLE_GSM_UART_*`) are added as part of the follow-up GSM spec, not this one.

Rollback: revert the refactor commit. `soft_uart_rmt` component stays in the tree as a dormant reusable asset.

## 6. Phase 3 — Fallback plan (I²C-to-UART expander)

Triggered only if Phase 1 objectively fails.

- **Candidate part:** NXP SC16IS752 (dual UART, I²C or SPI, 64-byte FIFOs, RTS/CTS support). Alternative: SC16IS762 (SPI) or WCH CH9434 (4 UART, SPI).
- **Assignment:** GSM on expander UART A at 115200. Expander UART B is spare.
- **Pin budget:** Adds 2 pins (I²C SDA/SCL) + 1 IRQ pin. I²C can be shared with other future peripherals.
- **Driver:** Existing community ESP-IDF drivers for SC16IS752 exist; evaluate or write thin wrapper presenting same interface shape as `ma_uart` / `printer_uart` for consistency.
- **PCB impact:** New component, small rework. Design it so the expander footprint is DNP on current PCB variant and populated on GSM-variant PCB.

Detailed Phase 3 hardware/firmware spec is **out of scope for this document** (future spec if triggered).

## 7. Testing

### 7.1 Unit tests (Phase 1)

Host-side unit tests for the decoder math in `soft_uart_rmt`:

- `bits_from_duration()`: table of (duration, bit_time, expected_bits) covering rounding edge cases, clock drift ±2%, ±3%.
- Frame assembler: feed known bit patterns, assert emitted bytes and frame_err flags.
- These tests run on host (not target) via a small extracted pure-C unit of the decoder logic.

### 7.2 Integration tests (Phase 1)

On-target tests driven by `wm_uart_validator`:

- **T1 — Bench idle test.** No BLE client, WM at 9600. Run 10 min. Expect: zero mismatches. Smoke test.
- **T2 — BLE-loaded test.** BLE SPP connected, client streaming chatter. WM at 9600. Run 10 min. Expect: pass criteria (§4.7).
- **T3 — Baud sweep.** `wm_uart_validator_run_baud_sweep({9600, 19200, 38400, 57600, 115200}, 600)`.
- **T4 — Stress test.** All three peripherals active (WM+Printer+MA), BLE connected, validator running on WM. Run 30 min. Expect: pass criteria hold.
- **T5 — Thermal test.** Run T4 at ambient ~50°C (or closest available) to catch marginal timing.

Each test emits a JSON report captured via serial log. Results archived alongside this spec for Phase 2 decision-making.

### 7.3 Post-cutover regression (Phase 2)

After cutover, rerun existing WM end-to-end scenarios (weight capture → JSON → BLE delivery) with the refactored `wm_uart` to confirm no behavior change.

## 8. Error handling

- **RMT symbol buffer overflow:** Logged, `rmt_overflows` counter increments, affected frame dropped. Validator reports as decoder-side loss.
- **Decoder framing error:** Byte emitted with `frame_err=true`; validator counts separately.
- **Queue overflow to validator:** Byte dropped, `queue_drops` incremented. This is a validator-infrastructure issue, not decoder quality.
- **Baud change during operation:** `soft_uart_rmt_set_baud` stops the channel, reconfigures, restarts. Not atomic; caller must quiesce traffic.
- **Phase 2 GSM init failure:** Does not affect WM/Printer/MA. GSM subsystem fails independently.

## 9. Out of scope

- **Soft UART TX.** RX-only in v1. GSM does not need soft UART; WM only needs RX.
- **Parity / multi-stop-bit support.** Not required by WM.
- **GSM driver design.** This spec covers UART *assignment* only. Full GSM (EC200U-CN) driver, AT command layer, SMS/TCP/PPP, PWRKEY/STATUS pins, network management, etc., are the subject of a separate future spec after Phase 2 completes.
- **Phase 3 hardware design.** Detailed only at concept level; full expander integration is a separate spec if triggered.

## 10. Open decisions (defer to implementation plan)

- Which RMT channel number to use (default 0; confirm no conflict in any optional component).
- Exact symbol memory size for RMT (default 64 symbols; may tune during testing).
- Whether validator runs in a dedicated task or piggybacks on existing monitoring task.
- Exact hook mechanism in `wm_uart` for per-byte timestamps (single raw-byte callback vs shared ring buffer).

These are implementation-detail choices, not architectural, and will be decided in the implementation plan.

## 11. Success outcome

After all phases complete:

- **WM** runs on `soft_uart_rmt` (RMT channel, GPIO 44), behavior indistinguishable from today's HW UART0.
- **GSM (EC200U-CN)** runs on hardware UART0 at 115200 baud, serving AT commands and data traffic for the product.
- **Printer** and **MA** unchanged on UART1 and UART2.
- `soft_uart_rmt` component remains as a reusable asset for any future low-speed peripheral.
