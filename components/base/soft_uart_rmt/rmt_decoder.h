#ifndef RMT_DECODER_H_
#define RMT_DECODER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

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

/* ---- Frame assembler ---- */

typedef struct {
    uint8_t  byte;
    bool     frame_err;   /* true if start/stop bits were violated */
} rmt_decoder_frame_t;

typedef void (*rmt_decoder_byte_cb_t)(const rmt_decoder_frame_t *frame,
                                      void *user_ctx);

typedef struct {
    uint32_t bit_shift_reg;  /* bits accumulated (position 0 = oldest) */
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
 * Feed n_bits copies of level (0 or 1) into the assembler. When a full
 * 10-bit frame is assembled, cb is invoked once. Multiple frames from a
 * single call are supported (rare but possible near pulse boundaries).
 */
void rmt_decoder_feed(rmt_decoder_state_t *s, uint8_t level, uint8_t n_bits);

#ifdef __cplusplus
}
#endif
#endif
