#include "rmt_decoder.h"
#include <string.h>

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
