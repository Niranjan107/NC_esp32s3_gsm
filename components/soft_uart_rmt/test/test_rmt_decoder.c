#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include "rmt_decoder.h"

static int tests_run = 0;

#define RUN(name) do { printf("  [%2d] %s\n", ++tests_run, #name); name(); } while (0)

/* bit_time_ticks for 9600 baud at 1 MHz resolution = 104 (~104.17 us/bit). */
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
    /* 156 ticks (exactly halfway between 1 and 2 bits) - rounded div rounds up. */
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
    /* bit_time at 115200 baud, 1 MHz res = 8.68 -> round to 9. Test with 9. */
    const uint32_t bt = 9;
    assert(rmt_decoder_bits_from_duration(9, bt) == 1);
    assert(rmt_decoder_bits_from_duration(90, bt) == 10);
}

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

/* Feed a complete frame for byte 0x55 ('U') - start=0, 0x55=01010101 LSB first,
 * stop=1. Bits, LSB-first in time: 0 1 0 1 0 1 0 1 0 1 - alternating, trivial. */
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
 * Temporal: 0 (start), 1 (d0), 0 (d1..d7), 0 (stop - BAD). */
static void test_assembler_bad_stop_bit(void) {
    capture_t cap = {0};
    rmt_decoder_state_t s; rmt_decoder_init(&s, capture_cb, &cap);
    uint8_t bits[10] = {0,1,0,0,0,0,0,0,0,0}; /* stop=0, bad */
    for (int i = 0; i < 10; i++) rmt_decoder_feed(&s, bits[i], 1);
    assert(cap.count == 1);
    assert(cap.errs[0] == true);
}

/* Frame error: start bit is 1 instead of 0 - the assembler should NOT
 * enter a frame. No byte emitted. */
static void test_assembler_no_start_bit(void) {
    capture_t cap = {0};
    rmt_decoder_state_t s; rmt_decoder_init(&s, capture_cb, &cap);
    rmt_decoder_feed(&s, 1, 10); /* idle line - should emit nothing */
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

/* ---- Integration: duration -> bits -> assembler ---- */

/* Simulate RMT input for byte 'A' at 9600 baud, 1 MHz resolution (bt=104).
 * 'A' temporal bit pattern: 0 1 0 0 0 0 0 1 0 1
 * Runs: (0,1), (1,1), (0,5), (1,1), (0,1), (1,1).
 * Simulate realistic duration noise: +/-3 ticks per pulse. */
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
    RUN(test_assembler_ascii_U);
    RUN(test_assembler_ascii_0x0F_runs);
    RUN(test_assembler_bad_stop_bit);
    RUN(test_assembler_no_start_bit);
    RUN(test_assembler_back_to_back_frames);
    RUN(test_assembler_reset_midframe);
    RUN(test_integration_byte_A_with_jitter);
    RUN(test_integration_4pct_slow_5bit_run);
    printf("ALL %d TESTS PASSED\n", tests_run);
    return 0;
}
