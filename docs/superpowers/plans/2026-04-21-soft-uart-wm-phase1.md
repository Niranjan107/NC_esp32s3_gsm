# Soft UART for WM — Phase 1 + 1.5 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build and validate an RMT-based software UART driver for the Weighing Machine (WM), running in parallel with the existing hardware UART0 driver, to prove we can free UART0 for a future GSM (EC200U-CN) peripheral.

**Architecture:** Two new ESP-IDF components. `soft_uart_rmt` is a generic RMT-based RX soft UART driver with pure-C decoder math (host-unit-testable). `wm_uart_validator` runs the existing hardware `wm_uart` and the new `soft_uart_rmt` on the **same GPIO (44)** via GPIO matrix fan-out, tags each received byte with source + timestamp, and compares bytes/packets/JSON in real time. Existing `wm_uart` is modified only by adding one additive raw-byte callback (NULL by default = zero behavior change).

**Tech Stack:** ESP-IDF v5.x (RMT v2 API), FreeRTOS, ESP32-S3, C11. Host unit tests use plain `gcc` + assert macros (no framework dependency).

**Reference spec:** [../specs/2026-04-21-gsm-uart-expansion-design.md](../specs/2026-04-21-gsm-uart-expansion-design.md)

---

## File Structure

**New files:**

```
components/soft_uart_rmt/
├── CMakeLists.txt                          NEW — component build
├── Kconfig.projbuild                       NEW — menuconfig entries
├── include/soft_uart_rmt.h                 NEW — public API
├── soft_uart_rmt.c                         NEW — ESP-IDF integration (RMT + tasks)
├── rmt_decoder.h                           NEW — pure-C decoder API (no ESP-IDF deps)
├── rmt_decoder.c                           NEW — pure-C decoder math
└── test/
    ├── Makefile                            NEW — host gcc build
    └── test_rmt_decoder.c                  NEW — host unit tests

components/wm_uart_validator/
├── CMakeLists.txt                          NEW
├── Kconfig.projbuild                       NEW
├── include/wm_uart_validator.h             NEW — public API
└── wm_uart_validator.c                     NEW — comparator task + stats
```

**Modified files:**

```
components/wm_uart/wm_uart.h                +1 typedef, +1 function decl
components/wm_uart/wm_uart.c                +1 static var, +1 function impl, +3 lines in RX task
main/main.c                                 +10 lines (conditional init of soft_uart_rmt + validator)
main/CMakeLists.txt                         +2 lines (add to PRIV_REQUIRES)
main/Kconfig.projbuild                      +1 submenu entry reference
```

**Responsibility split:**

- `rmt_decoder.[ch]` → pure decoder math (duration→bits, frame assembly). Fully host-testable, zero ESP-IDF deps.
- `soft_uart_rmt.c` → ESP-IDF glue (RMT channel config, task, GPIO matrix routing, callbacks).
- `wm_uart_validator.c` → subscribes to both UART byte streams, aligns by timestamp, reports metrics.

This split lets us TDD the dangerous math cleanly, while the ESP-IDF bits are simple integration code.

---

## Build / Flash / Test Commands (reference)

```bash
# Full firmware build
idf.py build

# Flash + monitor (replace COMx with actual port)
idf.py -p COMx flash monitor

# Menuconfig
idf.py menuconfig

# Host unit tests (decoder math only)
cd components/soft_uart_rmt/test && make clean && make && ./test_rmt_decoder
```

**Expected git commit prefix:** match the existing project style. Initial commit is `"Initial commit: ESP32S3 VLC connector firmware"`, design spec uses `"Add GSM UART expansion design spec"`. Use imperative mood, no conventional-commits prefix.

---

## PHASE A — Pure-C Decoder (Host-Testable)

### Task 1: Create `soft_uart_rmt` component skeleton + host test harness

**Files:**
- Create: `components/soft_uart_rmt/CMakeLists.txt`
- Create: `components/soft_uart_rmt/Kconfig.projbuild`
- Create: `components/soft_uart_rmt/include/soft_uart_rmt.h` (stub)
- Create: `components/soft_uart_rmt/soft_uart_rmt.c` (stub)
- Create: `components/soft_uart_rmt/rmt_decoder.h` (stub)
- Create: `components/soft_uart_rmt/rmt_decoder.c` (stub)
- Create: `components/soft_uart_rmt/test/Makefile`
- Create: `components/soft_uart_rmt/test/test_rmt_decoder.c` (empty test)

- [ ] **Step 1: Create `components/soft_uart_rmt/CMakeLists.txt`**

```cmake
idf_component_register(
    SRCS "soft_uart_rmt.c" "rmt_decoder.c"
    INCLUDE_DIRS "include" "."
    REQUIRES driver esp_timer
)
```

- [ ] **Step 2: Create `components/soft_uart_rmt/Kconfig.projbuild`**

```
menu "Soft UART (RMT-based)"

    config SOFT_UART_RMT_LOG_LEVEL
        int "Soft UART RMT log level (0=none,1=err,2=warn,3=info,4=debug,5=verbose)"
        default 3
        range 0 5
        help
            Verbosity for soft_uart_rmt component.

    config SOFT_UART_RMT_DEFAULT_RESOLUTION_HZ
        int "Default RMT resolution (Hz) for soft UART"
        default 1000000
        range 100000 10000000
        help
            RMT tick resolution. Must be > 16 * max baud rate for good sampling.
            1 MHz (default) gives 1 us ticks, adequate up to ~115200 baud.

    config SOFT_UART_RMT_DEFAULT_SYMBOL_MEM_WORDS
        int "Default RMT symbol memory size (words)"
        default 64
        range 48 256
        help
            RMT receive buffer in 32-bit symbol words. Larger = more tolerant
            to decoder task scheduling latency under load.

endmenu
```

- [ ] **Step 3: Create `components/soft_uart_rmt/rmt_decoder.h` (stub)**

```c
#ifndef RMT_DECODER_H_
#define RMT_DECODER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Stub — API is added by Tasks 2 & 4 driven by failing tests. */

#ifdef __cplusplus
}
#endif
#endif
```

- [ ] **Step 4: Create `components/soft_uart_rmt/rmt_decoder.c` (stub)**

```c
#include "rmt_decoder.h"
/* Implementation added by Tasks 3 & 5 driven by failing tests. */
```

- [ ] **Step 5: Create `components/soft_uart_rmt/include/soft_uart_rmt.h` (stub)**

```c
#ifndef SOFT_UART_RMT_H_
#define SOFT_UART_RMT_H_

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public ESP-IDF API — added in Phase B. */

#ifdef __cplusplus
}
#endif
#endif
```

- [ ] **Step 6: Create `components/soft_uart_rmt/soft_uart_rmt.c` (stub)**

```c
#include "soft_uart_rmt.h"
/* ESP-IDF integration added in Phase B. */
```

- [ ] **Step 7: Create `components/soft_uart_rmt/test/Makefile`**

```makefile
CC      = gcc
CFLAGS  = -std=c11 -Wall -Wextra -Werror -O0 -g -I.. -I../include
TARGET  = test_rmt_decoder
SRCS    = test_rmt_decoder.c ../rmt_decoder.c

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRCS)

test: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe

.PHONY: all test clean
```

- [ ] **Step 8: Create `components/soft_uart_rmt/test/test_rmt_decoder.c` (empty test driver)**

```c
#include <stdio.h>
#include <assert.h>
#include "rmt_decoder.h"

/* Tests added in Task 2 and onward. Each test function prints its name
 * and asserts; main() calls every test in order. */

int main(void) {
    printf("=== rmt_decoder tests ===\n");
    printf("ALL TESTS PASSED\n");
    return 0;
}
```

- [ ] **Step 9: Verify firmware build is clean (component compiles)**

```bash
idf.py build
```
Expected: build succeeds, no errors. New component `soft_uart_rmt` shown in component list.

- [ ] **Step 10: Verify host test harness builds and runs**

```bash
cd components/soft_uart_rmt/test && make clean && make && ./test_rmt_decoder
```
Expected output:
```
=== rmt_decoder tests ===
ALL TESTS PASSED
```

- [ ] **Step 11: Commit**

```bash
git add components/soft_uart_rmt/
git commit -m "Add soft_uart_rmt component skeleton and host test harness"
```

---

### Task 2: Write failing tests for `bits_from_duration`

**Files:**
- Modify: `components/soft_uart_rmt/rmt_decoder.h` — add function declaration
- Modify: `components/soft_uart_rmt/test/test_rmt_decoder.c` — add test cases

- [ ] **Step 1: Declare `bits_from_duration` in `rmt_decoder.h`**

Replace the stub comment block with:
```c
/**
 * Convert an RMT pulse duration (in ticks) into a rounded bit count.
 *
 * Implements: bits = (duration + bit_time/2) / bit_time
 *
 * @param duration_ticks Measured pulse duration, in RMT ticks.
 * @param bit_time_ticks Ticks per one UART bit at the configured baud.
 * @return Rounded bit count, clamped to [1, 10]. Returns 1 when duration is 0
 *         to ensure forward progress in the decoder.
 */
uint8_t rmt_decoder_bits_from_duration(uint32_t duration_ticks,
                                       uint32_t bit_time_ticks);
```

- [ ] **Step 2: Add test cases in `test_rmt_decoder.c`**

Replace the file body with:
```c
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include "rmt_decoder.h"

static int tests_run = 0;

#define RUN(name) do { printf("  [%2d] %s\n", ++tests_run, #name); name(); } while (0)

/* bit_time_ticks for 9600 baud at 1 MHz resolution = 104 (≈104.17 us/bit). */
static void test_bits_from_duration_exact_1bit(void) {
    assert(rmt_decoder_bits_from_duration(104, 104) == 1);
}
static void test_bits_from_duration_exact_2bit(void) {
    assert(rmt_decoder_bits_from_duration(208, 104) == 2);
}
static void test_bits_from_duration_exact_10bit(void) {
    assert(rmt_decoder_bits_from_duration(1040, 104) == 10);
}
static void test_bits_from_duration_rounding_up(void) {
    /* 103 ticks should round to 1 bit (within half-bit of 104). */
    assert(rmt_decoder_bits_from_duration(103, 104) == 1);
    /* 155 ticks (exactly halfway between 1 and 2 bits) — rounded div rounds up. */
    assert(rmt_decoder_bits_from_duration(156, 104) == 2);
}
static void test_bits_from_duration_rounding_down(void) {
    /* 51 ticks should round to 0 bits nominally, but clamp to minimum 1. */
    assert(rmt_decoder_bits_from_duration(51, 104) == 1);
}
static void test_bits_from_duration_clamp_max(void) {
    /* 11-bit-long pulse is impossible in UART; clamp to 10. */
    assert(rmt_decoder_bits_from_duration(1144, 104) == 10);
    assert(rmt_decoder_bits_from_duration(100000, 104) == 10);
}
static void test_bits_from_duration_zero_duration(void) {
    /* Defensive: 0 ticks should return 1 (minimum), never 0. */
    assert(rmt_decoder_bits_from_duration(0, 104) == 1);
}
static void test_bits_from_duration_clock_drift_plus_2pct(void) {
    /* Sender 2% fast: 1 bit = 102 ticks. Expect 1. */
    assert(rmt_decoder_bits_from_duration(102, 104) == 1);
    /* 5 bits @ 2% fast = 510. Expect 5. */
    assert(rmt_decoder_bits_from_duration(510, 104) == 5);
}
static void test_bits_from_duration_clock_drift_minus_2pct(void) {
    /* Sender 2% slow: 1 bit = 106 ticks. Expect 1. */
    assert(rmt_decoder_bits_from_duration(106, 104) == 1);
    /* 5 bits @ 2% slow = 530. Expect 5. */
    assert(rmt_decoder_bits_from_duration(530, 104) == 5);
}
static void test_bits_from_duration_115200_baud(void) {
    /* bit_time at 115200 baud, 1 MHz res = 8.68 → round to 9. Test with 9. */
    const uint32_t bt = 9;
    assert(rmt_decoder_bits_from_duration(9, bt) == 1);
    assert(rmt_decoder_bits_from_duration(90, bt) == 10);
}

int main(void) {
    printf("=== rmt_decoder tests ===\n");
    RUN(test_bits_from_duration_exact_1bit);
    RUN(test_bits_from_duration_exact_2bit);
    RUN(test_bits_from_duration_exact_10bit);
    RUN(test_bits_from_duration_rounding_up);
    RUN(test_bits_from_duration_rounding_down);
    RUN(test_bits_from_duration_clamp_max);
    RUN(test_bits_from_duration_zero_duration);
    RUN(test_bits_from_duration_clock_drift_plus_2pct);
    RUN(test_bits_from_duration_clock_drift_minus_2pct);
    RUN(test_bits_from_duration_115200_baud);
    printf("ALL %d TESTS PASSED\n", tests_run);
    return 0;
}
```

- [ ] **Step 3: Verify the test FAILS to link**

```bash
cd components/soft_uart_rmt/test && make clean && make
```
Expected: **link error** — `undefined reference to 'rmt_decoder_bits_from_duration'`. This confirms the test actually exercises the function we're about to write.

- [ ] **Step 4: Commit the failing test**

```bash
git add components/soft_uart_rmt/rmt_decoder.h components/soft_uart_rmt/test/test_rmt_decoder.c
git commit -m "Add failing host tests for rmt_decoder_bits_from_duration"
```

---

### Task 3: Implement `bits_from_duration` to make tests pass

**Files:**
- Modify: `components/soft_uart_rmt/rmt_decoder.c`

- [ ] **Step 1: Implement the function in `rmt_decoder.c`**

Replace the file contents with:
```c
#include "rmt_decoder.h"

uint8_t rmt_decoder_bits_from_duration(uint32_t duration_ticks,
                                       uint32_t bit_time_ticks)
{
    if (bit_time_ticks == 0) {
        return 1;
    }
    uint32_t bits = (duration_ticks + bit_time_ticks / 2u) / bit_time_ticks;
    if (bits < 1u) bits = 1u;
    if (bits > 10u) bits = 10u;
    return (uint8_t)bits;
}
```

- [ ] **Step 2: Run host tests**

```bash
cd components/soft_uart_rmt/test && make clean && make && ./test_rmt_decoder
```
Expected output ends with:
```
ALL 10 TESTS PASSED
```

- [ ] **Step 3: Verify ESP-IDF build still clean**

```bash
idf.py build
```
Expected: build succeeds.

- [ ] **Step 4: Commit**

```bash
git add components/soft_uart_rmt/rmt_decoder.c
git commit -m "Implement rmt_decoder_bits_from_duration with rounding + clamping"
```

---

### Task 4: Write failing tests for the frame assembler

**Files:**
- Modify: `components/soft_uart_rmt/rmt_decoder.h` — add assembler API
- Modify: `components/soft_uart_rmt/test/test_rmt_decoder.c` — add assembler tests

**Design:** The frame assembler consumes `(level, bits)` pairs (from the `bits_from_duration` output) and emits complete 10-bit UART frames: 1 start bit (low) + 8 data bits LSB-first + 1 stop bit (high). It reports each byte + `frame_err` flag.

- [ ] **Step 1: Add assembler types and API to `rmt_decoder.h`** (append before the closing `#endif`)

```c
/* ---- Frame assembler ---- */

typedef struct {
    uint8_t  byte;
    bool     frame_err;   /* true if start/stop bits were violated */
} rmt_decoder_frame_t;

typedef void (*rmt_decoder_byte_cb_t)(const rmt_decoder_frame_t *frame,
                                      void *user_ctx);

typedef struct {
    uint32_t bit_shift_reg;  /* bits accumulated (LSB = next bit) */
    uint8_t  bits_collected; /* 0..10 */
    bool     in_frame;       /* true after start bit detected */
    rmt_decoder_byte_cb_t cb;
    void    *cb_ctx;
} rmt_decoder_state_t;

/**
 * Initialize an assembler state. cb may be NULL (bytes then silently dropped,
 * useful for tests that only check internal state).
 */
void rmt_decoder_init(rmt_decoder_state_t *s,
                      rmt_decoder_byte_cb_t cb,
                      void *user_ctx);

/**
 * Reset the assembler to idle (forget any partial frame).
 * Called when the decoder sees end-of-RMT-reception (line idle).
 */
void rmt_decoder_reset(rmt_decoder_state_t *s);

/**
 * Feed `n_bits` copies of `level` (0 or 1) into the assembler. When a full
 * 10-bit frame is assembled, `cb` is invoked once. Multiple frames from a
 * single call are supported (rare but possible near pulse boundaries).
 */
void rmt_decoder_feed(rmt_decoder_state_t *s, uint8_t level, uint8_t n_bits);
```

- [ ] **Step 2: Add assembler tests in `test_rmt_decoder.c`** (insert before `main`, add to `main`)

```c
/* ---- Frame assembler tests ---- */

typedef struct {
    uint8_t  bytes[16];
    bool     errs[16];
    size_t   count;
} capture_t;

static void capture_cb(const rmt_decoder_frame_t *f, void *ctx) {
    capture_t *c = (capture_t*)ctx;
    if (c->count < sizeof(c->bytes)) {
        c->bytes[c->count] = f->byte;
        c->errs[c->count]  = f->frame_err;
        c->count++;
    }
}

/* Feed a complete frame for byte 0x55 ('U') — start=0, 0x55=01010101 LSB first,
 * stop=1. Bits, LSB-first in time: 0 1 0 1 0 1 0 1 0 1 — alternating, trivial. */
static void test_assembler_ascii_U(void) {
    capture_t cap = {0};
    rmt_decoder_state_t s; rmt_decoder_init(&s, capture_cb, &cap);
    /* Bit sequence in temporal order for 0x55 w/ start+stop:
     * 0 (start), 1 (d0), 0 (d1), 1 (d2), 0 (d3), 1 (d4), 0 (d5), 1 (d6), 0 (d7), 1 (stop)
     * Feed them one bit at a time. */
    uint8_t bits[10] = {0,1,0,1,0,1,0,1,0,1};
    for (int i = 0; i < 10; i++) rmt_decoder_feed(&s, bits[i], 1);
    assert(cap.count == 1);
    assert(cap.bytes[0] == 0x55);
    assert(cap.errs[0] == false);
}

/* Same byte fed as multi-bit runs (simulating real RMT pulse duration output).
 * For 0x55 the runs are all length-1, so identical. Try 0x0F instead:
 * temporal: 0 (start), 1111 (d0-d3), 0000 (d4-d7), 1 (stop).
 * As runs: (0,1), (1,4), (0,4), (1,1). */
static void test_assembler_ascii_0x0F_runs(void) {
    capture_t cap = {0};
    rmt_decoder_state_t s; rmt_decoder_init(&s, capture_cb, &cap);
    rmt_decoder_feed(&s, 0, 1);
    rmt_decoder_feed(&s, 1, 4);
    rmt_decoder_feed(&s, 0, 4);
    rmt_decoder_feed(&s, 1, 1);
    assert(cap.count == 1);
    assert(cap.bytes[0] == 0x0F);
    assert(cap.errs[0] == false);
}

/* Frame error: stop bit is 0 instead of 1.
 * Temporal: 0 (start), 1 (d0), 0 (d1..d7), 0 (stop — BAD). */
static void test_assembler_bad_stop_bit(void) {
    capture_t cap = {0};
    rmt_decoder_state_t s; rmt_decoder_init(&s, capture_cb, &cap);
    uint8_t bits[10] = {0,1,0,0,0,0,0,0,0,0}; /* stop=0, bad */
    for (int i = 0; i < 10; i++) rmt_decoder_feed(&s, bits[i], 1);
    assert(cap.count == 1);
    assert(cap.errs[0] == true);
}

/* Frame error: start bit is 1 instead of 0 — the assembler should NOT
 * enter a frame. No byte emitted. */
static void test_assembler_no_start_bit(void) {
    capture_t cap = {0};
    rmt_decoder_state_t s; rmt_decoder_init(&s, capture_cb, &cap);
    rmt_decoder_feed(&s, 1, 10); /* idle line — should emit nothing */
    assert(cap.count == 0);
}

/* Two consecutive frames, back-to-back (stop of first flows into start of
 * second after one bit gap). Byte 'A' (0x41) then 'B' (0x42). */
static void test_assembler_back_to_back_frames(void) {
    capture_t cap = {0};
    rmt_decoder_state_t s; rmt_decoder_init(&s, capture_cb, &cap);
    /* 'A' = 0x41 = 0100 0001 -> LSB first: 1 0 0 0 0 0 1 0
     * temporal: 0 1 0 0 0 0 0 1 0 1 */
    uint8_t a[10] = {0,1,0,0,0,0,0,1,0,1};
    /* 'B' = 0x42 = 0100 0010 -> LSB first: 0 1 0 0 0 0 1 0
     * temporal: 0 0 1 0 0 0 0 1 0 1 */
    uint8_t b[10] = {0,0,1,0,0,0,0,1,0,1};
    for (int i = 0; i < 10; i++) rmt_decoder_feed(&s, a[i], 1);
    for (int i = 0; i < 10; i++) rmt_decoder_feed(&s, b[i], 1);
    assert(cap.count == 2);
    assert(cap.bytes[0] == 'A');
    assert(cap.bytes[1] == 'B');
    assert(cap.errs[0] == false && cap.errs[1] == false);
}

/* Reset in mid-frame discards partial state. */
static void test_assembler_reset_midframe(void) {
    capture_t cap = {0};
    rmt_decoder_state_t s; rmt_decoder_init(&s, capture_cb, &cap);
    rmt_decoder_feed(&s, 0, 1);     /* start */
    rmt_decoder_feed(&s, 1, 3);     /* 3 data bits */
    rmt_decoder_reset(&s);          /* forget */
    rmt_decoder_feed(&s, 1, 10);    /* idle, no frame */
    assert(cap.count == 0);
}
```

Add to `main()` after the existing `RUN(...)` lines:
```c
    RUN(test_assembler_ascii_U);
    RUN(test_assembler_ascii_0x0F_runs);
    RUN(test_assembler_bad_stop_bit);
    RUN(test_assembler_no_start_bit);
    RUN(test_assembler_back_to_back_frames);
    RUN(test_assembler_reset_midframe);
```

- [ ] **Step 3: Verify tests FAIL to link**

```bash
cd components/soft_uart_rmt/test && make clean && make
```
Expected: link errors — `undefined reference to 'rmt_decoder_init'`, `'rmt_decoder_reset'`, `'rmt_decoder_feed'`.

- [ ] **Step 4: Commit the failing tests**

```bash
git add components/soft_uart_rmt/rmt_decoder.h components/soft_uart_rmt/test/test_rmt_decoder.c
git commit -m "Add failing host tests for rmt_decoder frame assembler"
```

---

### Task 5: Implement the frame assembler

**Files:**
- Modify: `components/soft_uart_rmt/rmt_decoder.c`

- [ ] **Step 1: Implement the assembler in `rmt_decoder.c`** (append after `bits_from_duration`)

```c
#include <string.h>

void rmt_decoder_init(rmt_decoder_state_t *s,
                      rmt_decoder_byte_cb_t cb,
                      void *user_ctx)
{
    memset(s, 0, sizeof(*s));
    s->cb     = cb;
    s->cb_ctx = user_ctx;
}

void rmt_decoder_reset(rmt_decoder_state_t *s)
{
    s->bit_shift_reg  = 0;
    s->bits_collected = 0;
    s->in_frame       = false;
}

static void emit_frame(rmt_decoder_state_t *s)
{
    /* After collecting 10 bits (start + 8 data + stop), verify & emit.
     * bit_shift_reg holds bits in time order: bit 0 = start (oldest). */
    uint32_t reg = s->bit_shift_reg;
    bool start_ok = ((reg & 0x1u) == 0u);
    bool stop_ok  = (((reg >> 9) & 0x1u) == 1u);
    uint8_t data  = (uint8_t)((reg >> 1) & 0xFFu);
    rmt_decoder_frame_t f = {
        .byte      = data,
        .frame_err = !(start_ok && stop_ok),
    };
    if (s->cb) s->cb(&f, s->cb_ctx);
    rmt_decoder_reset(s);
}

void rmt_decoder_feed(rmt_decoder_state_t *s, uint8_t level, uint8_t n_bits)
{
    level = level ? 1u : 0u;
    while (n_bits-- > 0) {
        if (!s->in_frame) {
            if (level == 0u) {
                /* Saw potential start bit. Enter frame. */
                s->in_frame       = true;
                s->bit_shift_reg  = 0; /* start bit = 0 at position 0 */
                s->bits_collected = 1;
            }
            /* else: still idle, stay idle. */
        } else {
            /* Append bit to shift register at position bits_collected. */
            if (level) {
                s->bit_shift_reg |= (1u << s->bits_collected);
            }
            s->bits_collected++;
            if (s->bits_collected >= 10u) {
                emit_frame(s);
            }
        }
    }
}
```

- [ ] **Step 2: Run host tests**

```bash
cd components/soft_uart_rmt/test && make clean && make && ./test_rmt_decoder
```
Expected output ends with:
```
ALL 16 TESTS PASSED
```

- [ ] **Step 3: Verify ESP-IDF build still clean**

```bash
idf.py build
```
Expected: build succeeds.

- [ ] **Step 4: Commit**

```bash
git add components/soft_uart_rmt/rmt_decoder.c
git commit -m "Implement rmt_decoder frame assembler (start/data/stop, frame_err)"
```

---

### Task 6: Add integration tests combining `bits_from_duration` + assembler

**Files:**
- Modify: `components/soft_uart_rmt/test/test_rmt_decoder.c`

**Design rationale:** The real decoder feeds RMT-measured durations through `bits_from_duration` and then into the assembler. These integration tests catch bugs where the two pieces compose incorrectly under clock drift.

- [ ] **Step 1: Add integration tests** (insert before `main`, add to `main`)

```c
/* ---- Integration: duration → bits → assembler ---- */

/* Simulate RMT input for byte 'A' at 9600 baud, 1 MHz resolution (bt=104).
 * 'A' temporal bit pattern: 0 1 0 0 0 0 0 1 0 1
 * Runs: (0,1), (1,1), (0,5), (1,1), (0,1), (1,1).
 * Simulate realistic duration noise: ±3 ticks per pulse. */
static void test_integration_byte_A_with_jitter(void) {
    const uint32_t bt = 104;
    capture_t cap = {0};
    rmt_decoder_state_t s; rmt_decoder_init(&s, capture_cb, &cap);

    struct { uint8_t level; uint32_t dur; } pulses[] = {
        {0, 104 - 2},      /* 1 bit, 2 ticks short */
        {1, 104 + 3},      /* 1 bit, 3 ticks long */
        {0, 5 * 104 - 4},  /* 5 bits, 4 ticks short */
        {1, 104 + 2},      /* 1 bit, 2 ticks long */
        {0, 104 - 1},      /* 1 bit, 1 tick short */
        {1, 104 + 1},      /* 1 bit, 1 tick long */
    };
    for (size_t i = 0; i < sizeof(pulses)/sizeof(pulses[0]); i++) {
        uint8_t n = rmt_decoder_bits_from_duration(pulses[i].dur, bt);
        rmt_decoder_feed(&s, pulses[i].level, n);
    }
    assert(cap.count == 1);
    assert(cap.bytes[0] == 'A');
    assert(cap.errs[0] == false);
}

/* Extreme drift: -4% on a 5-bit run should still resolve correctly. */
static void test_integration_4pct_slow_5bit_run(void) {
    const uint32_t bt = 104;
    uint32_t dur_5bit_slow = (uint32_t)(5 * 104 * 1.04); /* 540 */
    assert(rmt_decoder_bits_from_duration(dur_5bit_slow, bt) == 5);
    /* -4% fast: 499 ticks should still be 5 bits. */
    uint32_t dur_5bit_fast = (uint32_t)(5 * 104 * 0.96); /* 499 */
    assert(rmt_decoder_bits_from_duration(dur_5bit_fast, bt) == 5);
}
```

Add to `main()`:
```c
    RUN(test_integration_byte_A_with_jitter);
    RUN(test_integration_4pct_slow_5bit_run);
```

- [ ] **Step 2: Run tests**

```bash
cd components/soft_uart_rmt/test && make clean && make && ./test_rmt_decoder
```
Expected: `ALL 18 TESTS PASSED`.

- [ ] **Step 3: Commit**

```bash
git add components/soft_uart_rmt/test/test_rmt_decoder.c
git commit -m "Add integration tests for duration-to-bits-to-assembler pipeline"
```

---

## PHASE B — ESP-IDF RMT Integration (On-Target)

### Task 7: Define public `soft_uart_rmt` API (header + stub)

**Files:**
- Modify: `components/soft_uart_rmt/include/soft_uart_rmt.h`
- Modify: `components/soft_uart_rmt/soft_uart_rmt.c` (stub functions)

- [ ] **Step 1: Fill in `soft_uart_rmt.h` with the full public API**

Replace the file body with:
```c
#ifndef SOFT_UART_RMT_H_
#define SOFT_UART_RMT_H_

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct soft_uart_rmt_obj *soft_uart_rmt_handle_t;

typedef struct {
    int      gpio_num;          /* RX pin */
    int      baud_rate;         /* 9600..115200 */
    uint8_t  data_bits;         /* must be 8 in v1 */
    uint8_t  stop_bits;         /* must be 1 in v1 */
    uint8_t  parity;            /* must be 0 (none) in v1 */
    uint32_t resolution_hz;     /* RMT tick rate; 0 = use Kconfig default */
    int      rmt_mem_block_symbols; /* RMT buffer words; 0 = use Kconfig default */
    size_t   byte_queue_depth;  /* FreeRTOS queue size; 0 = use 256 */
} soft_uart_rmt_config_t;

typedef struct {
    uint8_t  byte;
    int64_t  ts_us;             /* esp_timer_get_time() at frame completion */
    bool     frame_err;
} soft_uart_rmt_rx_t;

typedef void (*soft_uart_rmt_byte_cb_t)(const soft_uart_rmt_rx_t *rx,
                                        void *user_ctx);

typedef struct {
    uint64_t bytes_ok;
    uint64_t bytes_frame_err;
    uint64_t rmt_overflows;
    uint64_t queue_drops;
} soft_uart_rmt_stats_t;

esp_err_t soft_uart_rmt_init(const soft_uart_rmt_config_t *cfg,
                             soft_uart_rmt_handle_t *out);
esp_err_t soft_uart_rmt_start(soft_uart_rmt_handle_t h);
esp_err_t soft_uart_rmt_stop(soft_uart_rmt_handle_t h);
esp_err_t soft_uart_rmt_deinit(soft_uart_rmt_handle_t h);

esp_err_t soft_uart_rmt_register_byte_cb(soft_uart_rmt_handle_t h,
                                         soft_uart_rmt_byte_cb_t cb,
                                         void *user_ctx);

esp_err_t soft_uart_rmt_set_baud(soft_uart_rmt_handle_t h, int baud_rate);

void soft_uart_rmt_get_stats(soft_uart_rmt_handle_t h,
                             soft_uart_rmt_stats_t *out);
void soft_uart_rmt_reset_stats(soft_uart_rmt_handle_t h);

#ifdef __cplusplus
}
#endif
#endif /* SOFT_UART_RMT_H_ */
```

- [ ] **Step 2: Stub all functions in `soft_uart_rmt.c`** (they return `ESP_ERR_NOT_SUPPORTED`, so linking works and we can wire the validator later even before finishing impl)

Replace the file contents with:
```c
#include "soft_uart_rmt.h"
#include <string.h>

struct soft_uart_rmt_obj { int placeholder; };

esp_err_t soft_uart_rmt_init(const soft_uart_rmt_config_t *cfg,
                             soft_uart_rmt_handle_t *out) {
    (void)cfg; (void)out;
    return ESP_ERR_NOT_SUPPORTED;
}
esp_err_t soft_uart_rmt_start(soft_uart_rmt_handle_t h) { (void)h; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t soft_uart_rmt_stop(soft_uart_rmt_handle_t h)  { (void)h; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t soft_uart_rmt_deinit(soft_uart_rmt_handle_t h){ (void)h; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t soft_uart_rmt_register_byte_cb(soft_uart_rmt_handle_t h,
                                         soft_uart_rmt_byte_cb_t cb,
                                         void *ctx) { (void)h;(void)cb;(void)ctx; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t soft_uart_rmt_set_baud(soft_uart_rmt_handle_t h, int b) { (void)h;(void)b; return ESP_ERR_NOT_SUPPORTED; }
void soft_uart_rmt_get_stats(soft_uart_rmt_handle_t h, soft_uart_rmt_stats_t *o){ (void)h; if (o) memset(o,0,sizeof(*o)); }
void soft_uart_rmt_reset_stats(soft_uart_rmt_handle_t h){ (void)h; }
```

- [ ] **Step 3: Verify ESP-IDF build**

```bash
idf.py build
```
Expected: succeeds.

- [ ] **Step 4: Commit**

```bash
git add components/soft_uart_rmt/include/soft_uart_rmt.h components/soft_uart_rmt/soft_uart_rmt.c
git commit -m "Define public soft_uart_rmt API with stub implementation"
```

---

### Task 8: Implement `soft_uart_rmt_init` / `_deinit` (RMT channel lifecycle, no reception yet)

**Files:**
- Modify: `components/soft_uart_rmt/soft_uart_rmt.c`
- Modify: `components/soft_uart_rmt/CMakeLists.txt` — add `esp_driver_rmt` requirement (ESP-IDF v5.x split the driver components)

- [ ] **Step 1: Update `CMakeLists.txt`**

```cmake
idf_component_register(
    SRCS "soft_uart_rmt.c" "rmt_decoder.c"
    INCLUDE_DIRS "include" "."
    REQUIRES driver esp_driver_rmt esp_timer esp_rom freertos
)
```

- [ ] **Step 2: Implement `init` / `deinit`** in `soft_uart_rmt.c`

Replace the file with:
```c
#include "soft_uart_rmt.h"
#include "rmt_decoder.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>
#include <stdlib.h>

#define TAG "soft_uart_rmt"

#ifndef CONFIG_SOFT_UART_RMT_DEFAULT_RESOLUTION_HZ
#define CONFIG_SOFT_UART_RMT_DEFAULT_RESOLUTION_HZ 1000000
#endif
#ifndef CONFIG_SOFT_UART_RMT_DEFAULT_SYMBOL_MEM_WORDS
#define CONFIG_SOFT_UART_RMT_DEFAULT_SYMBOL_MEM_WORDS 64
#endif

struct soft_uart_rmt_obj {
    soft_uart_rmt_config_t     cfg;
    rmt_channel_handle_t       rx_chan;
    uint32_t                   bit_time_ticks;
    uint32_t                   frame_min_ticks;  /* ~1 bit, glitch filter */
    uint32_t                   frame_max_ticks;  /* > full frame idle */
    bool                       running;
    soft_uart_rmt_byte_cb_t    byte_cb;
    void                      *byte_cb_ctx;
    soft_uart_rmt_stats_t      stats;
};

static void recalc_timings(soft_uart_rmt_handle_t h)
{
    uint32_t res = h->cfg.resolution_hz;
    uint32_t baud = (uint32_t)h->cfg.baud_rate;
    h->bit_time_ticks   = (res + baud / 2u) / baud;              /* rounded */
    h->frame_min_ticks  = h->bit_time_ticks / 4u;                /* glitch filter */
    h->frame_max_ticks  = h->bit_time_ticks * 12u;               /* >10 bits */
    if (h->frame_min_ticks == 0) h->frame_min_ticks = 1;
}

esp_err_t soft_uart_rmt_init(const soft_uart_rmt_config_t *cfg,
                             soft_uart_rmt_handle_t *out)
{
    if (!cfg || !out) return ESP_ERR_INVALID_ARG;
    if (cfg->data_bits != 8 || cfg->stop_bits != 1 || cfg->parity != 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (cfg->baud_rate < 1200 || cfg->baud_rate > 230400) {
        return ESP_ERR_INVALID_ARG;
    }

    soft_uart_rmt_handle_t h = (soft_uart_rmt_handle_t)calloc(1, sizeof(*h));
    if (!h) return ESP_ERR_NO_MEM;
    h->cfg = *cfg;
    if (h->cfg.resolution_hz == 0) h->cfg.resolution_hz = CONFIG_SOFT_UART_RMT_DEFAULT_RESOLUTION_HZ;
    if (h->cfg.rmt_mem_block_symbols == 0) h->cfg.rmt_mem_block_symbols = CONFIG_SOFT_UART_RMT_DEFAULT_SYMBOL_MEM_WORDS;
    if (h->cfg.byte_queue_depth == 0) h->cfg.byte_queue_depth = 256;

    recalc_timings(h);

    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num       = h->cfg.gpio_num,
        .clk_src        = RMT_CLK_SRC_DEFAULT,
        .resolution_hz  = h->cfg.resolution_hz,
        .mem_block_symbols = h->cfg.rmt_mem_block_symbols,
        .flags.invert_in = false,
        .flags.with_dma  = false,
    };
    esp_err_t err = rmt_new_rx_channel(&rx_cfg, &h->rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_rx_channel failed: %d", err);
        free(h);
        return err;
    }

    ESP_LOGI(TAG, "init OK: gpio=%d baud=%d res=%lu bit_ticks=%lu",
             h->cfg.gpio_num, h->cfg.baud_rate,
             (unsigned long)h->cfg.resolution_hz, (unsigned long)h->bit_time_ticks);
    *out = h;
    return ESP_OK;
}

esp_err_t soft_uart_rmt_deinit(soft_uart_rmt_handle_t h)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    if (h->running) soft_uart_rmt_stop(h);
    if (h->rx_chan) rmt_del_channel(h->rx_chan);
    free(h);
    return ESP_OK;
}

/* _start, _stop, _register_byte_cb, _set_baud, stats: added in Task 9 */
esp_err_t soft_uart_rmt_start(soft_uart_rmt_handle_t h) { (void)h; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t soft_uart_rmt_stop(soft_uart_rmt_handle_t h)  { (void)h; return ESP_OK; }
esp_err_t soft_uart_rmt_register_byte_cb(soft_uart_rmt_handle_t h,
                                         soft_uart_rmt_byte_cb_t cb, void *ctx) {
    if (!h) return ESP_ERR_INVALID_ARG;
    h->byte_cb = cb; h->byte_cb_ctx = ctx; return ESP_OK;
}
esp_err_t soft_uart_rmt_set_baud(soft_uart_rmt_handle_t h, int b) {
    if (!h || b < 1200 || b > 230400) return ESP_ERR_INVALID_ARG;
    h->cfg.baud_rate = b; recalc_timings(h); return ESP_OK;
}
void soft_uart_rmt_get_stats(soft_uart_rmt_handle_t h, soft_uart_rmt_stats_t *o) {
    if (h && o) *o = h->stats;
}
void soft_uart_rmt_reset_stats(soft_uart_rmt_handle_t h) {
    if (h) memset(&h->stats, 0, sizeof(h->stats));
}
```

- [ ] **Step 3: Build**

```bash
idf.py build
```
Expected: succeeds. RMT channel creation compiles; no actual RX yet.

- [ ] **Step 4: Commit**

```bash
git add components/soft_uart_rmt/CMakeLists.txt components/soft_uart_rmt/soft_uart_rmt.c
git commit -m "Implement soft_uart_rmt_init/_deinit: RMT RX channel lifecycle"
```

---

### Task 9: Implement `_start` / `_stop` with RMT RX loop + decoder dispatch

**Files:**
- Modify: `components/soft_uart_rmt/soft_uart_rmt.c`

**Design:** A dedicated FreeRTOS task calls `rmt_receive` in a loop. On `on_recv_done`, symbols are delivered via a FreeRTOS queue. The task walks each `rmt_symbol_word_t` (which carries two level/duration pairs per word), feeds each into `bits_from_duration` then `rmt_decoder_feed`, and publishes emitted bytes via the registered callback.

- [ ] **Step 1: Add receive loop infrastructure** — replace the existing stub `_start` / `_stop` and add helpers. Modify the struct and add fields.

Edit the struct at top of `soft_uart_rmt.c`:
```c
struct soft_uart_rmt_obj {
    soft_uart_rmt_config_t     cfg;
    rmt_channel_handle_t       rx_chan;
    uint32_t                   bit_time_ticks;
    uint32_t                   frame_min_ticks;
    uint32_t                   frame_max_ticks;
    bool                       running;
    soft_uart_rmt_byte_cb_t    byte_cb;
    void                      *byte_cb_ctx;
    soft_uart_rmt_stats_t      stats;

    /* ---- new fields ---- */
    QueueHandle_t              symbol_queue;    /* carries rmt_rx_done_event_data_t */
    TaskHandle_t               task_handle;
    rmt_symbol_word_t         *rx_buffer;       /* RMT receive buffer */
    size_t                     rx_buffer_size;  /* bytes */
    rmt_decoder_state_t        dec_state;
};
```

Add an event-data wrapper type at file scope (above functions):
```c
typedef struct {
    rmt_symbol_word_t symbols[CONFIG_SOFT_UART_RMT_DEFAULT_SYMBOL_MEM_WORDS];
    size_t            num_symbols;
} rx_batch_t;
```

- [ ] **Step 2: Add the ISR-context receive-done callback and decoder task**

Add before `soft_uart_rmt_init`:
```c
static void on_decoded_byte(const rmt_decoder_frame_t *frame, void *ctx)
{
    soft_uart_rmt_handle_t h = (soft_uart_rmt_handle_t)ctx;
    if (frame->frame_err) h->stats.bytes_frame_err++;
    else                  h->stats.bytes_ok++;

    if (h->byte_cb) {
        soft_uart_rmt_rx_t rx = {
            .byte      = frame->byte,
            .ts_us     = esp_timer_get_time(),
            .frame_err = frame->frame_err,
        };
        h->byte_cb(&rx, h->byte_cb_ctx);
    }
}

static bool IRAM_ATTR rmt_rx_done_cb(rmt_channel_handle_t chan,
                                     const rmt_rx_done_event_data_t *edata,
                                     void *user_ctx)
{
    soft_uart_rmt_handle_t h = (soft_uart_rmt_handle_t)user_ctx;
    BaseType_t higher = pdFALSE;
    rx_batch_t batch;
    size_t n = edata->num_symbols;
    if (n > CONFIG_SOFT_UART_RMT_DEFAULT_SYMBOL_MEM_WORDS) {
        n = CONFIG_SOFT_UART_RMT_DEFAULT_SYMBOL_MEM_WORDS;
        h->stats.rmt_overflows++;
    }
    memcpy(batch.symbols, edata->received_symbols, n * sizeof(rmt_symbol_word_t));
    batch.num_symbols = n;
    if (xQueueSendFromISR(h->symbol_queue, &batch, &higher) != pdTRUE) {
        h->stats.queue_drops++;
    }
    return higher == pdTRUE;
}

static void decoder_task(void *arg)
{
    soft_uart_rmt_handle_t h = (soft_uart_rmt_handle_t)arg;
    rmt_receive_config_t rx_cfg = {
        .signal_range_min_ns = (uint32_t)((uint64_t)h->frame_min_ticks * 1000000000ull / h->cfg.resolution_hz),
        .signal_range_max_ns = (uint32_t)((uint64_t)h->frame_max_ticks * 1000000000ull / h->cfg.resolution_hz),
    };
    /* Kick off first receive */
    rmt_receive(h->rx_chan, h->rx_buffer, h->rx_buffer_size, &rx_cfg);

    rx_batch_t batch;
    while (h->running) {
        if (xQueueReceive(h->symbol_queue, &batch, pdMS_TO_TICKS(100)) == pdTRUE) {
            for (size_t i = 0; i < batch.num_symbols; i++) {
                rmt_symbol_word_t *w = &batch.symbols[i];
                /* Each word has two level/duration pairs */
                uint8_t b0 = rmt_decoder_bits_from_duration(w->duration0, h->bit_time_ticks);
                rmt_decoder_feed(&h->dec_state, (uint8_t)w->level0, b0);
                if (w->duration1 > 0) {
                    uint8_t b1 = rmt_decoder_bits_from_duration(w->duration1, h->bit_time_ticks);
                    rmt_decoder_feed(&h->dec_state, (uint8_t)w->level1, b1);
                } else {
                    /* duration1==0 means end-of-item; line idle → reset partial frame */
                    rmt_decoder_reset(&h->dec_state);
                }
            }
            /* Re-arm receive */
            rmt_receive(h->rx_chan, h->rx_buffer, h->rx_buffer_size, &rx_cfg);
        }
    }
    vTaskDelete(NULL);
}
```

- [ ] **Step 3: Implement `_start` and `_stop`**

Replace the stub `soft_uart_rmt_start` and `soft_uart_rmt_stop`:
```c
esp_err_t soft_uart_rmt_start(soft_uart_rmt_handle_t h)
{
    if (!h || h->running) return ESP_ERR_INVALID_STATE;

    /* Allocate symbol buffer (2 * mem_block_symbols is a safe receive chunk) */
    h->rx_buffer_size = h->cfg.rmt_mem_block_symbols * sizeof(rmt_symbol_word_t) * 2;
    h->rx_buffer = (rmt_symbol_word_t *)malloc(h->rx_buffer_size);
    if (!h->rx_buffer) return ESP_ERR_NO_MEM;

    h->symbol_queue = xQueueCreate(8, sizeof(rx_batch_t));
    if (!h->symbol_queue) { free(h->rx_buffer); h->rx_buffer = NULL; return ESP_ERR_NO_MEM; }

    rmt_decoder_init(&h->dec_state, on_decoded_byte, h);

    rmt_rx_event_callbacks_t cbs = { .on_recv_done = rmt_rx_done_cb };
    esp_err_t err = rmt_rx_register_event_callbacks(h->rx_chan, &cbs, h);
    if (err != ESP_OK) goto fail;

    err = rmt_enable(h->rx_chan);
    if (err != ESP_OK) goto fail;

    h->running = true;
    BaseType_t ok = xTaskCreate(decoder_task, "sw_uart_dec", 4096, h, 6, &h->task_handle);
    if (ok != pdPASS) { h->running = false; err = ESP_ERR_NO_MEM; goto fail; }
    ESP_LOGI(TAG, "started on GPIO %d at %d baud", h->cfg.gpio_num, h->cfg.baud_rate);
    return ESP_OK;

fail:
    if (h->symbol_queue) { vQueueDelete(h->symbol_queue); h->symbol_queue = NULL; }
    if (h->rx_buffer)    { free(h->rx_buffer); h->rx_buffer = NULL; }
    return err;
}

esp_err_t soft_uart_rmt_stop(soft_uart_rmt_handle_t h)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    if (!h->running) return ESP_OK;
    h->running = false;
    vTaskDelay(pdMS_TO_TICKS(150)); /* let task exit */
    rmt_disable(h->rx_chan);
    if (h->symbol_queue) { vQueueDelete(h->symbol_queue); h->symbol_queue = NULL; }
    if (h->rx_buffer)    { free(h->rx_buffer); h->rx_buffer = NULL; }
    h->task_handle = NULL;
    return ESP_OK;
}
```

- [ ] **Step 4: Build**

```bash
idf.py build
```
Expected: succeeds. If errors about missing `driver/rmt_rx.h`, verify `esp_driver_rmt` is in REQUIRES (added in Task 8).

- [ ] **Step 5: Commit**

```bash
git add components/soft_uart_rmt/soft_uart_rmt.c
git commit -m "Implement soft_uart_rmt RX loop with RMT symbol decoder task"
```

---

### Task 10: On-target loopback smoke test (bench, no WM needed)

**Files:**
- Create: `components/soft_uart_rmt/test_apps/loopback_main.c` (Phase-1-only test harness; deleted later)
- Modify: `main/main.c` — call the test under a Kconfig guard

**Design:** Use any free ESP32-S3 GPIO as a simulated UART TX (bit-banged in test code), wired externally via a jumper to GPIO 44 (WM RX). Start `soft_uart_rmt` on GPIO 44; verify decoder receives the bit-banged bytes correctly. This validates the full pipeline without needing the WM.

**Hardware setup for this test:** With the device unpowered, jumper **GPIO 15 → GPIO 44** on the dev board. Power on. No WM should be connected during this test (otherwise signals fight).

- [ ] **Step 1: Add smoke-test Kconfig entry** — in `main/Kconfig.projbuild`, inside the `menu "NCLite CLEV4"` block, add a new submenu:

```
    menu "Development: Soft UART Smoke Test"
        config NCLE_SOFT_UART_LOOPBACK_TEST
            bool "Enable soft UART GPIO loopback smoke test"
            default n
            help
                When enabled, main.c runs a one-shot test that bit-bangs
                the ASCII string "HELLO\n" on GPIO 15 and verifies that
                soft_uart_rmt receives it on GPIO 44. Requires a jumper
                wire from GPIO 15 to GPIO 44. Disable for production.

        config NCLE_SOFT_UART_LOOPBACK_TX_GPIO
            int "Loopback TX GPIO"
            default 15
            depends on NCLE_SOFT_UART_LOOPBACK_TEST

        config NCLE_SOFT_UART_LOOPBACK_RX_GPIO
            int "Loopback RX GPIO"
            default 44
            depends on NCLE_SOFT_UART_LOOPBACK_TEST
    endmenu
```

- [ ] **Step 2: Add the smoke test function to `main.c`** — find the existing function block after the BLE callback wrappers and add:

```c
#ifdef CONFIG_NCLE_SOFT_UART_LOOPBACK_TEST
#include "soft_uart_rmt.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"

static volatile int s_loopback_rx_count = 0;
static char s_loopback_rx_buf[32];

static void loopback_byte_cb(const soft_uart_rmt_rx_t *rx, void *ctx)
{
    (void)ctx;
    if (s_loopback_rx_count < (int)sizeof(s_loopback_rx_buf) - 1) {
        s_loopback_rx_buf[s_loopback_rx_count++] = (char)rx->byte;
        s_loopback_rx_buf[s_loopback_rx_count] = '\0';
    }
}

static void bitbang_byte(int gpio, uint8_t b, int bit_us)
{
    /* Start bit */
    gpio_set_level(gpio, 0); ets_delay_us(bit_us);
    for (int i = 0; i < 8; i++) {
        gpio_set_level(gpio, (b >> i) & 1); ets_delay_us(bit_us);
    }
    /* Stop bit */
    gpio_set_level(gpio, 1); ets_delay_us(bit_us);
}

static void run_loopback_test(void)
{
    const int tx_gpio = CONFIG_NCLE_SOFT_UART_LOOPBACK_TX_GPIO;
    const int rx_gpio = CONFIG_NCLE_SOFT_UART_LOOPBACK_RX_GPIO;
    const int baud = 9600;
    const int bit_us = 1000000 / baud; /* 104 */

    /* Configure TX */
    gpio_config_t cfg = { .pin_bit_mask = 1ULL << tx_gpio, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&cfg);
    gpio_set_level(tx_gpio, 1); /* idle high */

    /* Init soft UART on RX */
    soft_uart_rmt_config_t ucfg = {
        .gpio_num = rx_gpio, .baud_rate = baud, .data_bits = 8,
        .stop_bits = 1, .parity = 0,
    };
    soft_uart_rmt_handle_t h = NULL;
    ESP_ERROR_CHECK(soft_uart_rmt_init(&ucfg, &h));
    ESP_ERROR_CHECK(soft_uart_rmt_register_byte_cb(h, loopback_byte_cb, NULL));
    ESP_ERROR_CHECK(soft_uart_rmt_start(h));

    vTaskDelay(pdMS_TO_TICKS(100)); /* settle */

    const char *msg = "HELLO\n";
    ESP_LOGI(TAG, "LOOPBACK: sending '%s' on GPIO %d", msg, tx_gpio);
    for (const char *p = msg; *p; p++) {
        bitbang_byte(tx_gpio, (uint8_t)*p, bit_us);
        ets_delay_us(bit_us); /* extra idle between chars */
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "LOOPBACK: received %d bytes: '%s'",
             s_loopback_rx_count, s_loopback_rx_buf);

    if (s_loopback_rx_count == (int)strlen(msg) &&
        strncmp(s_loopback_rx_buf, msg, strlen(msg)) == 0) {
        ESP_LOGI(TAG, "LOOPBACK: *** PASS ***");
    } else {
        ESP_LOGE(TAG, "LOOPBACK: *** FAIL *** (expected '%s')", msg);
    }

    soft_uart_rmt_stop(h);
    soft_uart_rmt_deinit(h);
}
#endif
```

- [ ] **Step 3: Call the test from `app_main`** — add at the start of `app_main`, before any other init:

```c
#ifdef CONFIG_NCLE_SOFT_UART_LOOPBACK_TEST
    run_loopback_test();
#endif
```

(Find where `app_main` is defined in `main.c`; insert the call before `nvs_flash_init()` call. If you're unsure where it goes, insert right after `ESP_LOGI(TAG, "...starting...")` style first log.)

- [ ] **Step 4: Update `main/CMakeLists.txt`** — add `soft_uart_rmt` to `PRIV_REQUIRES`:

Change:
```cmake
    PRIV_REQUIRES
        wm_uart
        ma_uart
        cmd_parser
        printer_uart
        ble_spp
```
to:
```cmake
    PRIV_REQUIRES
        wm_uart
        ma_uart
        cmd_parser
        printer_uart
        ble_spp
        soft_uart_rmt
```

- [ ] **Step 5: Enable the test in menuconfig**

```bash
idf.py menuconfig
# Navigate: NCLite CLEV4 → Development: Soft UART Smoke Test → Enable [Y]
# Save, exit
```

- [ ] **Step 6: Build, flash, monitor**

With the jumper wire GPIO15↔GPIO44 in place and no WM connected:

```bash
idf.py build
idf.py -p COMx flash monitor
```
(Replace `COMx` with the actual port. Use `Ctrl+]` to exit monitor.)

**Expected log output (early in boot):**
```
I (xxx) NCLE_MAIN: LOOPBACK: sending 'HELLO
' on GPIO 15
I (xxx) NCLE_MAIN: LOOPBACK: received 6 bytes: 'HELLO
'
I (xxx) NCLE_MAIN: LOOPBACK: *** PASS ***
```

If FAIL: double-check jumper wire, check soft_uart_rmt init logs for errors, verify RMT resolution (1 MHz default).

- [ ] **Step 7: Disable the smoke test in menuconfig before committing**

```bash
idf.py menuconfig
# Navigate: NCLite CLEV4 → Development: Soft UART Smoke Test → Disable
# Save, exit
```

- [ ] **Step 8: Commit**

```bash
git add main/Kconfig.projbuild main/main.c main/CMakeLists.txt sdkconfig
git commit -m "Add soft UART GPIO loopback smoke test (Kconfig-gated)"
```

---

## PHASE C — Hook raw-byte callback into `wm_uart`

### Task 11: Add `wm_uart_set_raw_byte_callback` API + invocation

**Files:**
- Modify: `components/wm_uart/wm_uart.h`
- Modify: `components/wm_uart/wm_uart.c`

- [ ] **Step 1: Add API declaration to `wm_uart.h`** — append before the closing `#endif // _WM_UART_H_`:

```c
// ============================================================================
// Raw byte callback (for validation/diagnostics only)
// ============================================================================

/**
 * @brief Raw per-byte callback, invoked for every byte read from the HW UART.
 *
 * Primarily used by wm_uart_validator during Phase 1 to compare hardware
 * and software UART byte streams. If cb is NULL (default), the RX task
 * runs exactly as before with no callback overhead.
 *
 * @param byte   The received byte
 * @param ts_us  Timestamp (esp_timer_get_time) captured immediately after
 *               uart_read_bytes() returned
 * @param ctx    User context pointer (see wm_uart_set_raw_byte_callback)
 */
typedef void (*wm_uart_raw_byte_cb_t)(uint8_t byte, int64_t ts_us, void *ctx);

/**
 * @brief Register a raw-byte callback. Pass NULL to disable.
 * @param cb   Callback function (NULL disables)
 * @param ctx  User context, passed through to cb
 */
void wm_uart_set_raw_byte_callback(wm_uart_raw_byte_cb_t cb, void *ctx);
```

- [ ] **Step 2: Add the static variables and setter to `wm_uart.c`** — near the other `static` state definitions (around line 150, after `s_task_running`):

```c
/* Raw-byte callback (NULL = disabled, zero overhead path). */
static wm_uart_raw_byte_cb_t s_raw_byte_cb = NULL;
static void                 *s_raw_byte_ctx = NULL;

void wm_uart_set_raw_byte_callback(wm_uart_raw_byte_cb_t cb, void *ctx)
{
    s_raw_byte_cb  = cb;
    s_raw_byte_ctx = ctx;
}
```

- [ ] **Step 3: Invoke the callback in the RX task** — in `uart_rx_task` (around line 502), change:

```c
        if (len > 0) {
            s_last_rx_time = esp_timer_get_time();

            for (int i = 0; i < len; i++) {
                uint8_t byte = rx_buffer[i];
```

to:

```c
        if (len > 0) {
            s_last_rx_time = esp_timer_get_time();

            /* Invoke raw-byte callback if registered (validator hook).
             * Capture timestamp ONCE for the whole chunk — all bytes in a
             * chunk arrived within the same uart_read_bytes call so share
             * approximately the same timestamp within UART DMA granularity. */
            int64_t chunk_ts = s_last_rx_time;

            for (int i = 0; i < len; i++) {
                uint8_t byte = rx_buffer[i];
                if (s_raw_byte_cb) {
                    s_raw_byte_cb(byte, chunk_ts, s_raw_byte_ctx);
                }
```

- [ ] **Step 4: Build**

```bash
idf.py build
```
Expected: succeeds.

- [ ] **Step 5: Smoke test — existing WM path still works with NULL callback**

```bash
idf.py -p COMx flash monitor
```

With the WM physically connected to GPIO 44 and the smoke test disabled, verify the normal WM logs appear (weight packets being received and JSON'd) exactly as before this task. No regressions.

- [ ] **Step 6: Commit**

```bash
git add components/wm_uart/wm_uart.h components/wm_uart/wm_uart.c
git commit -m "Add wm_uart_set_raw_byte_callback hook for validation (NULL-safe default)"
```

---

## PHASE D — `wm_uart_validator` Component

### Task 12: Create `wm_uart_validator` skeleton

**Files:**
- Create: `components/wm_uart_validator/CMakeLists.txt`
- Create: `components/wm_uart_validator/Kconfig.projbuild`
- Create: `components/wm_uart_validator/include/wm_uart_validator.h`
- Create: `components/wm_uart_validator/wm_uart_validator.c`

- [ ] **Step 1: Create `CMakeLists.txt`**

```cmake
idf_component_register(
    SRCS "wm_uart_validator.c"
    INCLUDE_DIRS "include"
    REQUIRES wm_uart soft_uart_rmt esp_timer esp_rom freertos driver log
)
```

- [ ] **Step 2: Create `Kconfig.projbuild`**

```
menu "WM UART Validator (Phase 1 only)"

    config NCLE_WM_VALIDATOR_ENABLE
        bool "Enable WM UART Validator"
        default n
        depends on NCLE_WM_ENABLE
        help
            Phase 1 validation harness. Runs the RMT-based soft UART in
            parallel with the hardware WM UART on the same GPIO and
            compares byte streams. Disable for production.

    config NCLE_WM_VALIDATOR_RMT_CHANNEL
        int "RMT channel for soft UART"
        default 0
        range 0 3
        depends on NCLE_WM_VALIDATOR_ENABLE

    config NCLE_WM_VALIDATOR_REPORT_INTERVAL_MS
        int "Stats report interval (ms)"
        default 10000
        range 1000 60000
        depends on NCLE_WM_VALIDATOR_ENABLE

    config NCLE_WM_VALIDATOR_PACKET_QUIET_MS
        int "Packet end quiet-time (ms)"
        default 50
        range 10 500
        depends on NCLE_WM_VALIDATOR_ENABLE
endmenu
```

- [ ] **Step 3: Create `include/wm_uart_validator.h`**

```c
#ifndef WM_UART_VALIDATOR_H_
#define WM_UART_VALIDATOR_H_

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int      shared_gpio;
    int      rmt_channel;
    int      baud_rate;
    uint32_t report_interval_ms;
    uint32_t packet_quiet_ms;
} wm_uart_validator_config_t;

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
    uint64_t rmt_overflows;
    uint64_t queue_drops;
} wm_uart_validator_stats_t;

esp_err_t wm_uart_validator_init(const wm_uart_validator_config_t *cfg);
esp_err_t wm_uart_validator_start(void);
esp_err_t wm_uart_validator_stop(void);
esp_err_t wm_uart_validator_deinit(void);

void wm_uart_validator_get_stats(wm_uart_validator_stats_t *out);
void wm_uart_validator_reset_stats(void);

/**
 * Sweep across baud rates, logging stats per step. BLOCKS the calling task
 * for (seconds_per_step * n) seconds. Operator must change the WM device
 * baud out-of-band between steps (this function does not control the WM
 * hardware).
 */
esp_err_t wm_uart_validator_run_baud_sweep(const int *bauds, size_t n,
                                           uint32_t seconds_per_step);

#ifdef __cplusplus
}
#endif
#endif
```

- [ ] **Step 4: Create `wm_uart_validator.c` (skeleton only — stats struct, init/deinit, state)**

```c
#include "wm_uart_validator.h"
#include "wm_uart.h"
#include "soft_uart_rmt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/gpio_sig_map.h"
#include <string.h>

#define TAG "wm_validate"

typedef struct {
    uint8_t  byte;
    int64_t  ts_us;
    uint8_t  source;      /* 0 = HW, 1 = SW */
    uint8_t  frame_err;
} tagged_byte_t;

static wm_uart_validator_config_t   s_cfg;
static wm_uart_validator_stats_t    s_stats;
static soft_uart_rmt_handle_t       s_sw_handle = NULL;
static QueueHandle_t                s_byte_queue = NULL;
static TaskHandle_t                 s_task = NULL;
static volatile bool                s_running = false;

static void hw_byte_cb(uint8_t byte, int64_t ts_us, void *ctx) {
    (void)ctx;
    tagged_byte_t t = { .byte = byte, .ts_us = ts_us, .source = 0, .frame_err = 0 };
    if (s_byte_queue) xQueueSend(s_byte_queue, &t, 0);
}
static void sw_byte_cb(const soft_uart_rmt_rx_t *rx, void *ctx) {
    (void)ctx;
    tagged_byte_t t = { .byte = rx->byte, .ts_us = rx->ts_us,
                        .source = 1, .frame_err = rx->frame_err ? 1 : 0 };
    if (s_byte_queue) xQueueSend(s_byte_queue, &t, 0);
}

static void validator_task(void *arg); /* added in Task 13 */

esp_err_t wm_uart_validator_init(const wm_uart_validator_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    s_cfg = *cfg;
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.baud_rate = cfg->baud_rate;

    soft_uart_rmt_config_t uc = {
        .gpio_num = cfg->shared_gpio,
        .baud_rate = cfg->baud_rate,
        .data_bits = 8, .stop_bits = 1, .parity = 0,
    };
    esp_err_t err = soft_uart_rmt_init(&uc, &s_sw_handle);
    if (err != ESP_OK) return err;

    /* Route the same GPIO input ALSO into the RMT peripheral. wm_uart already
     * routed it into UART0_RXD_IN; GPIO matrix permits fan-out. */
    esp_rom_gpio_connect_in_signal(cfg->shared_gpio,
                                   RMT_SIG_IN0_IDX + cfg->rmt_channel,
                                   false);

    s_byte_queue = xQueueCreate(1024, sizeof(tagged_byte_t));
    if (!s_byte_queue) { soft_uart_rmt_deinit(s_sw_handle); s_sw_handle = NULL; return ESP_ERR_NO_MEM; }
    return ESP_OK;
}

esp_err_t wm_uart_validator_deinit(void)
{
    if (s_running) wm_uart_validator_stop();
    if (s_sw_handle) { soft_uart_rmt_deinit(s_sw_handle); s_sw_handle = NULL; }
    if (s_byte_queue) { vQueueDelete(s_byte_queue); s_byte_queue = NULL; }
    return ESP_OK;
}

esp_err_t wm_uart_validator_start(void) { return ESP_ERR_NOT_SUPPORTED; } /* Task 13 */
esp_err_t wm_uart_validator_stop(void)  { return ESP_OK; } /* Task 13 */
void wm_uart_validator_get_stats(wm_uart_validator_stats_t *out) { if (out) *out = s_stats; }
void wm_uart_validator_reset_stats(void) { memset(&s_stats, 0, sizeof(s_stats)); s_stats.baud_rate = s_cfg.baud_rate; }
esp_err_t wm_uart_validator_run_baud_sweep(const int *b, size_t n, uint32_t s) {
    (void)b;(void)n;(void)s; return ESP_ERR_NOT_SUPPORTED; /* Task 15 */
}
```

- [ ] **Step 5: Build**

```bash
idf.py build
```
Expected: succeeds, `wm_uart_validator` appears in components.

- [ ] **Step 6: Commit**

```bash
git add components/wm_uart_validator/
git commit -m "Add wm_uart_validator component skeleton with HW/SW byte hooks"
```

---

### Task 13: Implement validator task — byte-level comparison + periodic stats

**Files:**
- Modify: `components/wm_uart_validator/wm_uart_validator.c`

**Design:** The task drains the shared queue. For each byte it maintains two rolling buffers (HW and SW). When a `packet_quiet_ms` gap is seen on the HW side, the packet closes: compare HW vs SW bytes, update stats. Every `report_interval_ms`, emit a log summary.

- [ ] **Step 1: Implement the validator task** — replace the placeholder line `static void validator_task(void *arg); /* added in Task 13 */` and the two stubs `_start` / `_stop` with:

```c
#define MAX_PACKET_BYTES 256

typedef struct {
    uint8_t  bytes[MAX_PACKET_BYTES];
    int64_t  ts[MAX_PACKET_BYTES];
    uint16_t count;
    bool     truncated;
} packet_buf_t;

static packet_buf_t s_hw_pkt;
static packet_buf_t s_sw_pkt;
static int64_t      s_last_hw_ts = 0;
static int64_t      s_last_sw_ts = 0;
static int64_t      s_last_report_us = 0;

static void close_packet(void)
{
    if (s_hw_pkt.count == 0 && s_sw_pkt.count == 0) return;

    s_stats.hw_packets++;
    if (s_sw_pkt.count > 0) s_stats.sw_packets++;

    /* Byte-wise match */
    uint16_t n = s_hw_pkt.count < s_sw_pkt.count ? s_hw_pkt.count : s_sw_pkt.count;
    int64_t skew_sum = 0;
    int64_t skew_max = 0;
    int64_t skew_count = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (s_hw_pkt.bytes[i] == s_sw_pkt.bytes[i]) s_stats.matched_bytes++;
        int64_t sk = s_sw_pkt.ts[i] - s_hw_pkt.ts[i];
        if (sk < 0) sk = -sk;
        skew_sum += sk;
        if (sk > skew_max) skew_max = sk;
        skew_count++;
    }
    s_stats.hw_bytes += s_hw_pkt.count;
    s_stats.sw_bytes += s_sw_pkt.count;
    if (skew_count > 0) {
        s_stats.avg_skew_us = skew_sum / skew_count;
        if (skew_max > s_stats.max_skew_us) s_stats.max_skew_us = skew_max;
    }

    memset(&s_hw_pkt, 0, sizeof(s_hw_pkt));
    memset(&s_sw_pkt, 0, sizeof(s_sw_pkt));
}

static void append_byte(packet_buf_t *p, uint8_t b, int64_t ts)
{
    if (p->count < MAX_PACKET_BYTES) {
        p->bytes[p->count] = b;
        p->ts[p->count] = ts;
        p->count++;
    } else {
        p->truncated = true;
    }
}

static void maybe_report(int64_t now)
{
    if (s_last_report_us == 0) { s_last_report_us = now; return; }
    uint32_t elapsed_ms = (uint32_t)((now - s_last_report_us) / 1000);
    if (elapsed_ms < s_cfg.report_interval_ms) return;

    /* Pull soft_uart_rmt stats too so we surface RMT overflows + queue drops. */
    soft_uart_rmt_stats_t sw_stats;
    soft_uart_rmt_get_stats(s_sw_handle, &sw_stats);
    s_stats.rmt_overflows = sw_stats.rmt_overflows;
    s_stats.queue_drops   = sw_stats.queue_drops;
    s_stats.frame_errs    = sw_stats.bytes_frame_err;

    uint64_t denom = s_stats.hw_bytes > s_stats.sw_bytes ? s_stats.hw_bytes : s_stats.sw_bytes;
    double byte_match_pct = denom ? (100.0 * (double)s_stats.matched_bytes / (double)denom) : 100.0;

    ESP_LOGI(TAG, "--- baud=%d window=%lums ---",
             s_stats.baud_rate, (unsigned long)elapsed_ms);
    ESP_LOGI(TAG, "  packets  : HW=%lu SW=%lu",
             (unsigned long)s_stats.hw_packets, (unsigned long)s_stats.sw_packets);
    ESP_LOGI(TAG, "  bytes    : HW=%llu SW=%llu match=%.3f%%",
             (unsigned long long)s_stats.hw_bytes,
             (unsigned long long)s_stats.sw_bytes,
             byte_match_pct);
    ESP_LOGI(TAG, "  frame_err: %llu  rmt_overflows: %llu  q_drops: %llu",
             (unsigned long long)s_stats.frame_errs,
             (unsigned long long)s_stats.rmt_overflows,
             (unsigned long long)s_stats.queue_drops);
    ESP_LOGI(TAG, "  skew_us  : avg=%lld  max=%lld",
             (long long)s_stats.avg_skew_us, (long long)s_stats.max_skew_us);

    s_last_report_us = now;
}

static void validator_task(void *arg)
{
    (void)arg;
    const int64_t quiet_us = (int64_t)s_cfg.packet_quiet_ms * 1000;
    tagged_byte_t t;
    ESP_LOGI(TAG, "validator task started (gpio=%d baud=%d)",
             s_cfg.shared_gpio, s_cfg.baud_rate);
    while (s_running) {
        if (xQueueReceive(s_byte_queue, &t, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (t.source == 0) { append_byte(&s_hw_pkt, t.byte, t.ts_us); s_last_hw_ts = t.ts_us; }
            else               { append_byte(&s_sw_pkt, t.byte, t.ts_us); s_last_sw_ts = t.ts_us; }
        }
        int64_t now = esp_timer_get_time();
        int64_t last = s_last_hw_ts > s_last_sw_ts ? s_last_hw_ts : s_last_sw_ts;
        if (last > 0 && (now - last) > quiet_us &&
            (s_hw_pkt.count > 0 || s_sw_pkt.count > 0)) {
            close_packet();
        }
        maybe_report(now);
    }
    ESP_LOGI(TAG, "validator task stopped");
    vTaskDelete(NULL);
}

esp_err_t wm_uart_validator_start(void)
{
    if (s_running) return ESP_ERR_INVALID_STATE;
    if (!s_sw_handle || !s_byte_queue) return ESP_ERR_INVALID_STATE;

    /* Register callbacks */
    wm_uart_set_raw_byte_callback(hw_byte_cb, NULL);
    esp_err_t err = soft_uart_rmt_register_byte_cb(s_sw_handle, sw_byte_cb, NULL);
    if (err != ESP_OK) { wm_uart_set_raw_byte_callback(NULL, NULL); return err; }

    err = soft_uart_rmt_start(s_sw_handle);
    if (err != ESP_OK) {
        wm_uart_set_raw_byte_callback(NULL, NULL);
        return err;
    }

    memset(&s_hw_pkt, 0, sizeof(s_hw_pkt));
    memset(&s_sw_pkt, 0, sizeof(s_sw_pkt));
    s_last_hw_ts = s_last_sw_ts = s_last_report_us = 0;
    s_running = true;

    BaseType_t ok = xTaskCreate(validator_task, "wm_validate", 6144, NULL, 4, &s_task);
    if (ok != pdPASS) {
        s_running = false;
        soft_uart_rmt_stop(s_sw_handle);
        wm_uart_set_raw_byte_callback(NULL, NULL);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t wm_uart_validator_stop(void)
{
    if (!s_running) return ESP_OK;
    s_running = false;
    vTaskDelay(pdMS_TO_TICKS(100));
    soft_uart_rmt_stop(s_sw_handle);
    wm_uart_set_raw_byte_callback(NULL, NULL);
    s_task = NULL;
    return ESP_OK;
}
```

- [ ] **Step 2: Build**

```bash
idf.py build
```
Expected: succeeds.

- [ ] **Step 3: Commit**

```bash
git add components/wm_uart_validator/wm_uart_validator.c
git commit -m "Implement wm_uart_validator byte-level comparison and stats task"
```

---

### Task 14: Wire validator into `main.c`

**Files:**
- Modify: `main/CMakeLists.txt`
- Modify: `main/main.c`

- [ ] **Step 1: Add `wm_uart_validator` to `main/CMakeLists.txt` PRIV_REQUIRES**

Change:
```cmake
    PRIV_REQUIRES
        wm_uart
        ma_uart
        cmd_parser
        printer_uart
        ble_spp
        soft_uart_rmt
```
to:
```cmake
    PRIV_REQUIRES
        wm_uart
        ma_uart
        cmd_parser
        printer_uart
        ble_spp
        soft_uart_rmt
        wm_uart_validator
```

- [ ] **Step 2: Add include and init call to `main.c`** — add near the other conditional includes at the top:

```c
#ifdef CONFIG_NCLE_WM_VALIDATOR_ENABLE
#include "wm_uart_validator.h"
#endif
```

- [ ] **Step 3: Start the validator after `wm_uart_init` + `wm_uart_start`** — find where `wm_uart_init()` / `wm_uart_start()` are called in `app_main` (search for `wm_uart_start`). Immediately after that block, add:

```c
#ifdef CONFIG_NCLE_WM_VALIDATOR_ENABLE
    {
        wm_uart_validator_config_t vcfg = {
            .shared_gpio         = CONFIG_NCLE_WM_UART_RX_PIN,
            .rmt_channel         = CONFIG_NCLE_WM_VALIDATOR_RMT_CHANNEL,
            .baud_rate           = CONFIG_NCLE_WM_UART_BAUD_RATE,
            .report_interval_ms  = CONFIG_NCLE_WM_VALIDATOR_REPORT_INTERVAL_MS,
            .packet_quiet_ms     = CONFIG_NCLE_WM_VALIDATOR_PACKET_QUIET_MS,
        };
        esp_err_t verr = wm_uart_validator_init(&vcfg);
        if (verr == ESP_OK) {
            verr = wm_uart_validator_start();
        }
        if (verr != ESP_OK) {
            ESP_LOGE(TAG, "WM validator failed to start: %d", verr);
        } else {
            ESP_LOGI(TAG, "WM validator running (GPIO %d, RMT ch %d, baud %d)",
                     CONFIG_NCLE_WM_UART_RX_PIN,
                     CONFIG_NCLE_WM_VALIDATOR_RMT_CHANNEL,
                     CONFIG_NCLE_WM_UART_BAUD_RATE);
        }
    }
#endif
```

- [ ] **Step 4: Enable validator in menuconfig**

```bash
idf.py menuconfig
# Navigate: NCLite CLEV4 → WM UART Validator (Phase 1 only) → Enable [Y]
# Save, exit
```

- [ ] **Step 5: Build + flash + monitor with WM connected**

```bash
idf.py build && idf.py -p COMx flash monitor
```

**Expected logs (after boot, with WM sending):**
```
I (xxx) NCLE_MAIN: WM validator running (GPIO 44, RMT ch 0, baud 9600)
I (xxx) wm_validate: validator task started (gpio=44 baud=9600)
I (xxx) soft_uart_rmt: started on GPIO 44 at 9600 baud
I (xxx) wm_validate: --- baud=9600 window=10000ms ---
I (xxx) wm_validate:   packets  : HW=4 SW=4
I (xxx) wm_validate:   bytes    : HW=68 SW=68 match=100.000%
I (xxx) wm_validate:   frame_err: 0  rmt_overflows: 0  q_drops: 0
I (xxx) wm_validate:   skew_us  : avg=87  max=312
```

(Exact packet counts depend on WM output rate. Key checks: match% close to 100.0, frame_err low, rmt_overflows=0, q_drops=0.)

- [ ] **Step 6: Commit**

```bash
git add main/CMakeLists.txt main/main.c sdkconfig
git commit -m "Wire wm_uart_validator into main.c under Kconfig flag"
```

---

### Task 15: Implement `wm_uart_validator_run_baud_sweep`

**Files:**
- Modify: `components/wm_uart_validator/wm_uart_validator.c`
- Modify: `main/main.c` — add a command hook (optional) or call sweep directly from a temporary test path

- [ ] **Step 1: Implement `wm_uart_validator_run_baud_sweep`** — replace the existing stub in `wm_uart_validator.c`:

```c
esp_err_t wm_uart_validator_run_baud_sweep(const int *bauds, size_t n,
                                           uint32_t seconds_per_step)
{
    if (!bauds || n == 0) return ESP_ERR_INVALID_ARG;
    if (!s_running) return ESP_ERR_INVALID_STATE;

    ESP_LOGW(TAG, "=== BAUD SWEEP START (%u steps, %lu s each) ===",
             (unsigned)n, (unsigned long)seconds_per_step);
    ESP_LOGW(TAG, "OPERATOR: set WM device baud BEFORE each step.");

    for (size_t i = 0; i < n; i++) {
        int b = bauds[i];
        ESP_LOGW(TAG, "--- Step %u/%u: baud=%d — waiting 5s for operator to set WM ---",
                 (unsigned)(i + 1), (unsigned)n, b);
        vTaskDelay(pdMS_TO_TICKS(5000));

        /* Reconfigure both sides */
        wm_uart_validator_stop();
        wm_uart_set_baud(b);
        soft_uart_rmt_set_baud(s_sw_handle, b);
        s_cfg.baud_rate = b;
        wm_uart_validator_reset_stats();
        wm_uart_validator_start();

        vTaskDelay(pdMS_TO_TICKS(seconds_per_step * 1000));

        wm_uart_validator_stats_t snap;
        wm_uart_validator_get_stats(&snap);
        uint64_t denom = snap.hw_bytes > snap.sw_bytes ? snap.hw_bytes : snap.sw_bytes;
        double match_pct = denom ? (100.0 * (double)snap.matched_bytes / (double)denom) : 0.0;

        ESP_LOGW(TAG, "SWEEP RESULT: baud=%d hw_bytes=%llu sw_bytes=%llu "
                      "match=%.3f%% frame_err=%llu rmt_ovf=%llu q_drop=%llu",
                 b,
                 (unsigned long long)snap.hw_bytes,
                 (unsigned long long)snap.sw_bytes,
                 match_pct,
                 (unsigned long long)snap.frame_errs,
                 (unsigned long long)snap.rmt_overflows,
                 (unsigned long long)snap.queue_drops);
    }
    ESP_LOGW(TAG, "=== BAUD SWEEP DONE ===");
    return ESP_OK;
}
```

- [ ] **Step 2: Build**

```bash
idf.py build
```
Expected: succeeds.

- [ ] **Step 3: Commit**

```bash
git add components/wm_uart_validator/wm_uart_validator.c
git commit -m "Implement wm_uart_validator_run_baud_sweep (operator-in-loop)"
```

---

## PHASE E — Validation Testing (Spec §7.2 T1–T5)

These tasks run the **hardware-in-loop** tests defined in the design spec. They require a physical ESP32-S3 board with a WM connected on GPIO 44. Each test produces a log capture committed to `docs/superpowers/test-results/`.

### Task 16: Test T1 — bench idle (10 min, no BLE client)

**Files:**
- Create: `docs/superpowers/test-results/T1-bench-idle.md` (log capture + summary)

- [ ] **Step 1: Ensure build has validator enabled, BLE enabled, no BLE client connected**

Verify in menuconfig:
- `NCLE_WM_VALIDATOR_ENABLE=y`
- `BLE_SPP_ENABLED=y` (or project default)
- WM device baud = 9600

- [ ] **Step 2: Flash, monitor, run 10 minutes**

```bash
idf.py -p COMx flash monitor | tee /tmp/t1-bench-idle.log
```

Power on the WM and let it stream for 10 minutes. Do NOT connect a BLE client.

- [ ] **Step 3: Stop monitor (Ctrl+])**

- [ ] **Step 4: Extract and commit results**

Create `docs/superpowers/test-results/T1-bench-idle.md`:

```markdown
# Test T1 — Bench Idle

**Date:** YYYY-MM-DD
**Duration:** 10 minutes
**Baud:** 9600
**BLE client:** none
**Pass criteria:** match ≥ 99.9%, frame_errs ≤ 1 per 10k bytes, rmt_overflows = 0

## Summary

| Metric        | Value       | Target      | Pass? |
|---------------|-------------|-------------|-------|
| HW bytes      | <fill>      | —           | —     |
| SW bytes      | <fill>      | —           | —     |
| Byte match    | <fill>%     | ≥ 99.9%     | <Y/N> |
| Frame errors  | <fill>      | ≤ 1/10k     | <Y/N> |
| RMT overflows | <fill>      | 0           | <Y/N> |
| Queue drops   | <fill>      | 0           | <Y/N> |
| Avg skew      | <fill> us   | —           | —     |
| Max skew      | <fill> us   | ≤ 208 us    | <Y/N> |

## Last 3 report windows (raw log)

```
<paste last 3 windows of wm_validate report from log here>
```

## Notes

<any observations>
```

Fill in the metrics from the log output, commit:

```bash
git add docs/superpowers/test-results/T1-bench-idle.md
git commit -m "Add T1 bench idle test results"
```

---

### Task 17: Test T2 — BLE-loaded (10 min, active BLE client)

**Files:**
- Create: `docs/superpowers/test-results/T2-ble-loaded.md`

**Setup:** Use a BLE scanner/chat app (e.g., nRF Connect) on a phone. Connect and send a short write-characteristic message every ~2 seconds for the full 10 minutes to simulate real RF load.

- [ ] **Step 1: Start fresh 10-minute capture**

```bash
idf.py -p COMx monitor | tee /tmp/t2-ble-loaded.log
```
(If re-flash is needed, `idf.py -p COMx flash monitor`.)

Connect BLE client, start sending periodic writes, keep WM streaming.

- [ ] **Step 2: After 10 min, stop monitor, create `docs/superpowers/test-results/T2-ble-loaded.md`**

Use the same template as T1, fill in metrics. **This is the primary Phase 2 go/no-go test.**

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/test-results/T2-ble-loaded.md
git commit -m "Add T2 BLE-loaded test results"
```

---

### Task 18: Test T3 — baud sweep {9600, 19200, 38400, 57600, 115200}

**Files:**
- Create: `docs/superpowers/test-results/T3-baud-sweep.md`

**Prerequisite:** The WM hardware must support each of these baud rates. If your WM only supports 9600, document this limitation and run the sweep at bauds the WM supports, plus use the GPIO-loopback test (Task 10) with a bit-banged source at higher bauds to characterize the soft UART ceiling independent of the WM.

- [ ] **Step 1: Add a one-shot sweep trigger to `main.c`** — under a new Kconfig `NCLE_WM_VALIDATOR_RUN_SWEEP`, add (after the validator start block added in Task 14):

Add to `main/Kconfig.projbuild` inside the WM Validator submenu:
```
        config NCLE_WM_VALIDATOR_RUN_SWEEP
            bool "Run baud sweep on boot (Phase 1.5)"
            default n
            depends on NCLE_WM_VALIDATOR_ENABLE

        config NCLE_WM_VALIDATOR_SWEEP_SECONDS_PER_STEP
            int "Seconds per sweep step"
            default 600
            range 30 3600
            depends on NCLE_WM_VALIDATOR_RUN_SWEEP
```

Add to `main.c` after the validator_start block:
```c
#ifdef CONFIG_NCLE_WM_VALIDATOR_RUN_SWEEP
    {
        static const int sweep_bauds[] = {9600, 19200, 38400, 57600, 115200};
        vTaskDelay(pdMS_TO_TICKS(3000)); /* settle before starting */
        wm_uart_validator_run_baud_sweep(sweep_bauds,
                                         sizeof(sweep_bauds)/sizeof(sweep_bauds[0]),
                                         CONFIG_NCLE_WM_VALIDATOR_SWEEP_SECONDS_PER_STEP);
    }
#endif
```

- [ ] **Step 2: Enable in menuconfig, reduce seconds_per_step to something practical**

```bash
idf.py menuconfig
# Enable NCLE_WM_VALIDATOR_RUN_SWEEP
# Set seconds_per_step = 600 (or shorter 120 if bench time-limited)
# Save
```

- [ ] **Step 3: Flash + monitor, following operator prompts to change WM baud between steps**

```bash
idf.py build && idf.py -p COMx flash monitor | tee /tmp/t3-baud-sweep.log
```

Between each `SWEEP RESULT` line, the log prints the next target baud and a 5-second countdown. Change the WM's baud setting physically during that window.

- [ ] **Step 4: Build `docs/superpowers/test-results/T3-baud-sweep.md`**

```markdown
# Test T3 — Baud Sweep

**Date:** YYYY-MM-DD
**Per-step duration:** <fill> s
**BLE client:** connected, periodic writes

## Results

| Baud   | HW bytes | SW bytes | Match %  | Frame err | RMT ovf | Q drops | Pass? |
|--------|----------|----------|----------|-----------|---------|---------|-------|
| 9600   |          |          |          |           |         |         |       |
| 19200  |          |          |          |           |         |         |       |
| 38400  |          |          |          |           |         |         |       |
| 57600  |          |          |          |           |         |         |       |
| 115200 |          |          |          |           |         |         |       |

## Ceiling

Highest baud passing all criteria: <fill>.

## Notes

<any observations, e.g. WM baud support limitations>
```

- [ ] **Step 5: Disable NCLE_WM_VALIDATOR_RUN_SWEEP, commit**

```bash
idf.py menuconfig
# Disable NCLE_WM_VALIDATOR_RUN_SWEEP
```

```bash
git add main/Kconfig.projbuild main/main.c docs/superpowers/test-results/T3-baud-sweep.md sdkconfig
git commit -m "Add T3 baud sweep test path and recorded results"
```

---

### Task 19: Test T4 — all-peripheral stress (30 min)

**Files:**
- Create: `docs/superpowers/test-results/T4-all-peripheral-stress.md`

**Setup:** WM + Printer + MA + BLE client all active, validator running on WM. Occasionally send a print job and MA command to exercise the other UARTs during WM streaming.

- [ ] **Step 1: Run 30-minute capture** (steps same as T2 but with printer + MA activity stimulated via BLE commands or UART stimuli)

- [ ] **Step 2: Record results in `docs/superpowers/test-results/T4-all-peripheral-stress.md`** (same template as T1)

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/test-results/T4-all-peripheral-stress.md
git commit -m "Add T4 all-peripheral stress test results"
```

---

### Task 20: Test T5 — thermal (best-effort, ~50°C ambient if available)

**Files:**
- Create: `docs/superpowers/test-results/T5-thermal.md`

- [ ] **Step 1: If thermal chamber or warm enclosure available, repeat T4 conditions at elevated ambient. Otherwise, document that T5 is skipped with reason.**

- [ ] **Step 2: Record results (or skip rationale) in `docs/superpowers/test-results/T5-thermal.md`**

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/test-results/T5-thermal.md
git commit -m "Add T5 thermal test results (or skip rationale)"
```

---

### Task 21: Phase 1 decision document

**Files:**
- Create: `docs/superpowers/test-results/PHASE-1-DECISION.md`

- [ ] **Step 1: Aggregate results and write the go/no-go decision**

```markdown
# Phase 1 Decision — Soft UART for WM

**Date:** YYYY-MM-DD
**Tests run:** T1, T2, T3, T4, T5
**Primary gate:** T2 (BLE-loaded, 10 min at 9600)
**Secondary gate:** T3 (19200 passes)

## Pass criteria (from spec §4.7)

| Criterion              | Target       | T2 Actual | T3@19200 Actual |
|------------------------|--------------|-----------|-----------------|
| Byte match             | ≥ 99.9%      |           |                 |
| Frame errors           | ≤ 1/10k      |           |                 |
| JSON mismatches        | 0            |           |                 |
| Max skew               | ≤ 208 us     |           |                 |
| RMT overflows          | 0            |           |                 |

## Decision

**<PASS — proceed to Phase 2 (GSM on UART0)>**
OR
**<FAIL at criterion X — proceed to Phase 3 (SC16IS752 I²C-UART expander)>**

## Rationale

<2-3 sentences explaining the call>

## Next steps

<Concrete next tasks: e.g., "Create Phase 2 spec at docs/superpowers/specs/...">
```

- [ ] **Step 2: Commit**

```bash
git add docs/superpowers/test-results/PHASE-1-DECISION.md
git commit -m "Record Phase 1 go/no-go decision"
```

---

## Self-Review

**Spec coverage check:** Each spec section mapped to at least one task above:

- §1–3 Problem/strategy/rationale → captured in plan header
- §4.1 Component structure → Tasks 1, 12
- §4.2 Runtime data flow → Tasks 10, 12, 14
- §4.3 `soft_uart_rmt` API → Tasks 2–9
- §4.4 `wm_uart_validator` API → Tasks 12–13
- §4.4 `wm_uart_set_raw_byte_callback` hook → Task 11
- §4.5 Comparison algorithm → Task 13
- §4.6 Baud sweep procedure → Tasks 15, 18
- §4.7 Pass/fail criteria → Task 21
- §5 Phase 2 plan → **out of scope** (separate spec, per §2 scope statement) ✓
- §6 Phase 3 fallback → **out of scope** (separate spec, per §2 scope statement) ✓
- §7.1 Decoder unit tests → Tasks 2–6
- §7.2 T1–T5 integration tests → Tasks 16–20
- §7.3 Post-cutover regression → **Phase 2** (not this plan) ✓
- §8 Error handling → built into each component implementation
- §9 Out of scope → respected

**Placeholder scan:** No "TBD" / "TODO" / "implement later" in task bodies. The test-results templates contain `<fill>` markers, which are *expected operator inputs* at execution time, not plan placeholders.

**Type consistency check:**

- `soft_uart_rmt_handle_t` — defined Task 7, used Tasks 8–14 ✓
- `soft_uart_rmt_byte_cb_t` — defined Task 7, used Tasks 10, 12 ✓
- `rmt_decoder_state_t` — defined Task 4, used Tasks 5, 8, 9 ✓
- `rmt_decoder_byte_cb_t` — defined Task 4, used Task 9 ✓
- `wm_uart_raw_byte_cb_t` — defined Task 11, used Task 12 ✓
- `wm_uart_validator_config_t` — defined Task 12, used Task 14 ✓
- `tagged_byte_t` — defined Task 12, used Task 13 ✓
- `wm_uart_set_raw_byte_callback` — defined Task 11, used Task 13 ✓
- `soft_uart_rmt_register_byte_cb` — defined Task 7, used Task 13 ✓
- `wm_uart_validator_run_baud_sweep` — declared Task 12, implemented Task 15, used Task 18 ✓

No mismatches.

**Scope check:** Plan implements Phase 1 + 1.5 only, as scoped by spec §2. Phase 2 cutover and Phase 3 fallback each get their own future plan.

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-04-21-soft-uart-wm-phase1.md`. Two execution options:

**1. Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration.

**2. Inline Execution** — Execute tasks in this session using executing-plans, batch execution with checkpoints.

Which approach?
