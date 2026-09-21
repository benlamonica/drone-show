/*------------------------------------------------------------------------------
* led_strip.c : WS2812B / SK6812 RGBW driver (see led_strip.h)
*-----------------------------------------------------------------------------*/
#include "led_strip.h"
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "led_strip.pio.h"

#define WS2812_HZ       800000u        /* one bit every 1.25 us */
#define CYCLES_PER_BIT  (ws2812_T1 + ws2812_T2 + ws2812_T3)
#define SAMPLE_MUL      2u             /* self-test samples per PIO cycle */
#define CAP_WORDS       128u           /* 4096 samples ~ 3 pixels at 24 bit */

static PIO      pio_;
static uint     sm_out_, sm_cap_ = 0xFFu;
static uint     off_out_, off_cap_;
static unsigned pin_, count_, bits_;
static led_chip_t chip_;
static int      dma_ch_ = -1;
static uint8_t  bright_ = 255;

/* Staged pixels, and the frame actually clocked out. Kept apart so brightness
 * is applied per frame without destroying the staged values. */
static uint8_t  px_[LED_STRIP_MAX][4];      /* r,g,b,w */
static uint32_t frame_[LED_STRIP_MAX];

static inline uint8_t scale(uint8_t v)
{
    return (uint8_t)(((uint32_t)v * bright_ + 127u) / 255u);
}

bool led_strip_init(unsigned pin, unsigned count, led_chip_t chip)
{
    pio_sm_config c;
    float div;

    if (count == 0 || count > LED_STRIP_MAX) return false;
    pin_ = pin; count_ = count; chip_ = chip;
    bits_ = (chip == LED_CHIP_SK6812_RGBW) ? 32 : 24;

    pio_ = pio0;
    if (!pio_can_add_program(pio_, &ws2812_program)) return false;
    off_out_ = pio_add_program(pio_, &ws2812_program);
    sm_out_  = (uint)pio_claim_unused_sm(pio_, false);
    if ((int)sm_out_ < 0) return false;

    pio_gpio_init(pio_, pin);
    pio_sm_set_consecutive_pindirs(pio_, sm_out_, pin, 1, true);

    c = ws2812_program_get_default_config(off_out_);
    sm_config_set_sideset_pins(&c, pin);
    /* Shift left so the first bit out is the MSB, and autopull exactly one
     * pixel's worth of bits. */
    sm_config_set_out_shift(&c, false, true, bits_);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    div = (float)clock_get_hz(clk_sys) / (WS2812_HZ * CYCLES_PER_BIT);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio_, sm_out_, off_out_, &c);
    pio_sm_set_enabled(pio_, sm_out_, true);

    dma_ch_ = dma_claim_unused_channel(false);
    if (dma_ch_ < 0) return false;

    memset(px_, 0, sizeof(px_));
    memset(frame_, 0, sizeof(frame_));
    return true;
}

void led_strip_set(unsigned i, uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    if (i >= count_) return;
    px_[i][0] = r; px_[i][1] = g; px_[i][2] = b; px_[i][3] = w;
}

void led_strip_fill(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
    for (uint i = 0; i < count_; i++) led_strip_set(i, r, g, b, w);
}

void led_strip_set_brightness(uint8_t b) { bright_ = b; }
unsigned led_strip_bits(void)  { return bits_; }
unsigned led_strip_count(void) { return count_; }

bool led_strip_busy(void)
{
    return dma_ch_ >= 0 && dma_channel_is_busy((uint)dma_ch_);
}

bool led_strip_show(void)
{
    if (dma_ch_ < 0 || led_strip_busy()) return false;

    for (uint i = 0; i < count_; i++) {
        uint8_t r = scale(px_[i][0]), g = scale(px_[i][1]);
        uint8_t b = scale(px_[i][2]), w = scale(px_[i][3]);
        uint32_t v;
        if (chip_ == LED_CHIP_SK6812_RGBW) {
            v = ((uint32_t)g << 24) | ((uint32_t)r << 16) | ((uint32_t)b << 8) | w;
        } else {
            /* 24 bits, left aligned: the PIO shifts out from the MSB. */
            v = (((uint32_t)g << 16) | ((uint32_t)r << 8) | b) << 8;
        }
        frame_[i] = v;
    }

    dma_channel_config dc = dma_channel_get_default_config((uint)dma_ch_);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, true);
    channel_config_set_write_increment(&dc, false);
    channel_config_set_dreq(&dc, pio_get_dreq(pio_, sm_out_, true));
    dma_channel_configure((uint)dma_ch_, &dc, &pio_->txf[sm_out_],
                          frame_, count_, true);
    return true;
}

/* --- self-test -------------------------------------------------------------
 * Watches the driven pin with a second state machine. The pad's input buffer
 * is still live while PIO drives it, so this needs no wiring. */
static uint32_t cap_buf_[CAP_WORDS];

static bool start_capture(uint32_t *sample_hz)
{
    pio_sm_config c;
    float div;
    int ch;

    if ((int)sm_cap_ == 0xFF) {
        if (!pio_can_add_program(pio_, &pin_sample_program)) return false;
        off_cap_ = pio_add_program(pio_, &pin_sample_program);
        sm_cap_  = (uint)pio_claim_unused_sm(pio_, false);
        if ((int)sm_cap_ < 0) return false;
    }
    gpio_set_input_enabled(pin_, true);

    c = pin_sample_program_get_default_config(off_cap_);
    sm_config_set_in_pins(&c, pin_);
    sm_config_set_in_shift(&c, false, true, 32);      /* MSB first, autopush */
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    div = (float)clock_get_hz(clk_sys)
          / (WS2812_HZ * CYCLES_PER_BIT * SAMPLE_MUL);
    sm_config_set_clkdiv(&c, div);
    *sample_hz = (uint32_t)(WS2812_HZ * CYCLES_PER_BIT * SAMPLE_MUL);

    pio_sm_init(pio_, sm_cap_, off_cap_, &c);
    pio_sm_clear_fifos(pio_, sm_cap_);

    ch = dma_claim_unused_channel(false);
    if (ch < 0) return false;
    dma_channel_config dc = dma_channel_get_default_config((uint)ch);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_dreq(&dc, pio_get_dreq(pio_, sm_cap_, false));
    dma_channel_configure((uint)ch, &dc, cap_buf_, &pio_->rxf[sm_cap_],
                          CAP_WORDS, true);
    pio_sm_set_enabled(pio_, sm_cap_, true);

    /* Kick the frame out while the sampler is already running. */
    led_strip_show();
    dma_channel_wait_for_finish_blocking((uint)ch);
    pio_sm_set_enabled(pio_, sm_cap_, false);
    dma_channel_unclaim((uint)ch);
    return true;
}

static inline int sample_bit(uint32_t i)
{
    return (cap_buf_[i >> 5] >> (31u - (i & 31u))) & 1u;
}

bool led_strip_selftest(led_strip_selftest_t *rep)
{
    uint32_t n = CAP_WORDS * 32, i, runs = 0;
    uint32_t h_sum0 = 0, h_n0 = 0, h_sum1 = 0, h_n1 = 0;
    uint32_t l_sum0 = 0, l_n0 = 0, l_sum1 = 0, l_n1 = 0;
    uint32_t decoded = 0, nbits = 0;
    double us_per_sample;

    memset(rep, 0, sizeof(*rep));

    /* A colour with both long and short pulses in every byte. */
    led_strip_fill(0xF0, 0x0F, 0xA5, 0x3C);
    rep->expect_pixel = (chip_ == LED_CHIP_SK6812_RGBW)
        ? 0x0FF0A53Cu                      /* G R B W */
        : 0x000FF0A5u;                     /* G R B    */

    if (!start_capture(&rep->sample_hz)) { rep->fail = "capture setup failed"; return false; }
    rep->captured = true;
    us_per_sample = 1e6 / rep->sample_hz;

    /* Walk the capture as high/low runs. Each bit is one high run followed by
     * one low run; a short high is a 0 and a long high is a 1. */
    i = 0;
    while (i < n && sample_bit(i) == 0) i++;      /* skip the idle low */
    while (i < n && nbits < 64) {
        uint32_t hi = 0, lo = 0;
        while (i < n && sample_bit(i) == 1) { hi++; i++; }
        while (i < n && sample_bit(i) == 0) { lo++; i++; }
        if (!hi || !lo) break;
        if (lo > 20 * SAMPLE_MUL) break;          /* reset gap: frame done */

        int one = hi * 2 > (ws2812_T1 + ws2812_T1 + ws2812_T2) * SAMPLE_MUL;
        if (one) { h_sum1 += hi; h_n1++; l_sum1 += lo; l_n1++; }
        else     { h_sum0 += hi; h_n0++; l_sum0 += lo; l_n0++; }
        decoded = (decoded << 1) | (uint32_t)one;
        nbits++;
        runs++;
        if (nbits == bits_) rep->first_pixel = decoded & ((bits_ == 32) ? 0xFFFFFFFFu
                                                                       : 0x00FFFFFFu);
    }

    rep->bits_decoded = nbits;
    if (h_n0) rep->t0h_us = h_sum0 * us_per_sample / h_n0;
    if (l_n0) rep->t0l_us = l_sum0 * us_per_sample / l_n0;
    if (h_n1) rep->t1h_us = h_sum1 * us_per_sample / h_n1;
    if (l_n1) rep->t1l_us = l_sum1 * us_per_sample / l_n1;

    if (nbits < bits_)                      { rep->fail = "too few bits captured";   return false; }
    if (!h_n0 || !h_n1)                     { rep->fail = "pattern lacked 0s or 1s"; return false; }
    if (rep->first_pixel != rep->expect_pixel) { rep->fail = "decoded colour mismatch"; return false; }
    /* WS2812B windows, which are the tighter of the two chips we support. */
    if (rep->t0h_us < 0.25 || rep->t0h_us > 0.55) { rep->fail = "T0H out of spec"; return false; }
    if (rep->t1h_us < 0.65 || rep->t1h_us > 0.95) { rep->fail = "T1H out of spec"; return false; }
    if (rep->t0l_us < 0.70 || rep->t0l_us > 1.00) { rep->fail = "T0L out of spec"; return false; }
    if (rep->t1l_us < 0.30 || rep->t1l_us > 0.60) { rep->fail = "T1L out of spec"; return false; }
    (void)runs;
    return true;
}
