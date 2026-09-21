/*------------------------------------------------------------------------------
* led_demo.c : WS2812B / SK6812 bring-up on the bench
*
* Two jobs:
*   1. Verify the wire format with no LEDs attached and no instruments, by
*      sampling the driven pin with a second PIO state machine and checking the
*      pulse widths against the datasheet.
*   2. Run a colour walk so a strip on a breadboard visibly proves the wiring.
*
* Wiring (WS2812B):
*   GP16 --470R--> [74AHCT125] --> DIN        level shifter: 3.3 V is marginal
*   5V  ----------------------- strip V+      1000 uF across V+/GND at the strip
*   GND ----------------------- strip GND     common with the Pico
*-----------------------------------------------------------------------------*/
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "led_strip.h"
#include "show_led.h"
#include "mavlink_min.h"
#include "hardware/uart.h"
#include <stdio.h>
#include <string.h>

#ifndef LED_PIN
#define LED_PIN   16
#endif
#ifndef LED_COUNT
#define LED_COUNT 10
#endif
#ifndef LED_CHIP
#define LED_CHIP  LED_CHIP_WS2812B
#endif
/* 10 WS2812B at full white is ~600 mA. On an external supply that is fine;
 * this is a comfortable bench brightness rather than a limit. Raise it with
 * -DLED_BRIGHTNESS=255 once you trust the wiring. */
#ifndef LED_BRIGHTNESS
#define LED_BRIGHTNESS 96
#endif
#define FRAME_MS   16          /* ~60 fps */
/* Seconds per bring-up step. Set 0 to skip straight to the animations once the
 * wiring is known good: -DRTKPICO_LED_BRINGUP_MS=0 */
#ifndef LED_BRINGUP_MS
#define LED_BRINGUP_MS 1500
#endif
#define PATTERN_MS 6000        /* how long each pattern runs */

/* Feed a real DEBUG_VECT "rgb" through a real UART and out the other side.
 *
 * The PL011 has an internal loopback bit (UARTCR.LBE), so TX is tied to RX
 * inside the peripheral: this exercises the actual serial path, the frame
 * parser and the light decoder on silicon, without a jumper or a flight
 * controller. It is the last link between "the host tests pass" and "a colour
 * from Skybrush lights the strip". */
#define UARTCR_LBE (1u << 7)

static bool loopback_light_test(uint8_t want_r, uint8_t want_g, uint8_t want_b)
{
    uint8_t payload[30], frame[64];
    mav_tx_t tx = { .sysid = 1, .compid = 1, .seq = 0 };
    const uint8_t *f;
    uint16_t len, n;
    mav_parser_t p;
    float x = want_r / 255.0f, y = want_g / 255.0f, z = want_b / 255.0f;
    absolute_time_t deadline;
    int got = 0;

    uart_init(uart1, 115200);
    hw_set_bits(&uart_get_hw(uart1)->cr, UARTCR_LBE);

    memset(payload, 0, sizeof(payload));
    memcpy(payload + 8,  &x, 4);
    memcpy(payload + 12, &y, 4);
    memcpy(payload + 16, &z, 4);
    memcpy(payload + 20, "rgb", 3);
    n = mav_pack(&tx, frame, 250 /* DEBUG_VECT */, 49 /* CRC_EXTRA */, payload, 30);

    mav_parser_reset(&p);
    show_led_init(false);

    for (uint16_t i = 0; i < n; i++) uart_putc_raw(uart1, frame[i]);

    deadline = make_timeout_time_ms(200);
    while (absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        if (!uart_is_readable(uart1)) continue;
        if (mav_parse_byte(&p, (uint8_t)uart_getc(uart1), &f, &len)) {
            if (show_led_handle_frame(f, len, 0)) { got = 1; break; }
        }
    }
    hw_clear_bits(&uart_get_hw(uart1)->cr, UARTCR_LBE);
    return got != 0;
}

/* Hue 0..1535 around the wheel, full saturation and value. Integer only:
 * this runs every frame and the RP2350 has no hardware double. */
static void hsv(uint16_t h, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t seg = (h / 256) % 6, f = h % 256;
    switch (seg) {
        case 0: *r = 255;     *g = f;       *b = 0;       break;
        case 1: *r = 255 - f; *g = 255;     *b = 0;       break;
        case 2: *r = 0;       *g = 255;     *b = f;       break;
        case 3: *r = 0;       *g = 255 - f; *b = 255;     break;
        case 4: *r = f;       *g = 0;       *b = 255;     break;
        default:*r = 255;     *g = 0;       *b = 255 - f; break;
    }
}

/* Working buffer, so patterns can fade the previous frame for tails. */
static uint8_t fb[LED_COUNT][3];

static void fb_clear(void) { memset(fb, 0, sizeof(fb)); }

static void fb_fade(uint8_t keep)     /* keep/256 of the previous frame */
{
    for (int i = 0; i < LED_COUNT; i++)
        for (int c = 0; c < 3; c++)
            fb[i][c] = (uint8_t)((fb[i][c] * keep) >> 8);
}

static void fb_push(void)
{
    uint8_t r, g, b, w;
    for (int i = 0; i < LED_COUNT; i++) {
        show_led_convert(fb[i][0] / 255.0f, fb[i][1] / 255.0f, fb[i][2] / 255.0f,
                         LED_CHIP == LED_CHIP_SK6812_RGBW, &r, &g, &b, &w);
        led_strip_set(i, r, g, b, w);
    }
    led_strip_show();
}

/* 1. whole strip one colour, hue rotating -- proves every pixel is alive */
static void pat_solid(uint32_t f)
{
    uint8_t r, g, b;
    hsv((uint16_t)((f * 6) % 1536), &r, &g, &b);
    for (int i = 0; i < LED_COUNT; i++) { fb[i][0]=r; fb[i][1]=g; fb[i][2]=b; }
}

/* 2. a single pixel running the strip, leaving a fading tail; the hue
 *    advances a little each lap so it cycles through colours */
static void pat_chase(uint32_t f)
{
    uint8_t r, g, b;
    int pos = (int)((f / 4) % LED_COUNT);
    fb_fade(150);
    hsv((uint16_t)(((f / 4 / LED_COUNT) * 96) % 1536), &r, &g, &b);
    fb[pos][0] = r; fb[pos][1] = g; fb[pos][2] = b;
}

/* 3. the whole hue wheel spread across the strip, rotating */
static void pat_rainbow(uint32_t f)
{
    for (int i = 0; i < LED_COUNT; i++) {
        uint8_t r, g, b;
        hsv((uint16_t)((i * 1536 / LED_COUNT + f * 8) % 1536), &r, &g, &b);
        fb[i][0]=r; fb[i][1]=g; fb[i][2]=b;
    }
}

/* 4. theatre chase: every third pixel lit, the pattern marching along */
static void pat_theater(uint32_t f)
{
    uint8_t r, g, b;
    int phase = (int)((f / 6) % 3);
    hsv((uint16_t)((f * 4) % 1536), &r, &g, &b);
    fb_clear();
    for (int i = phase; i < LED_COUNT; i += 3) { fb[i][0]=r; fb[i][1]=g; fb[i][2]=b; }
}

/* 5. bounce back and forth with a tail */
static void pat_bounce(uint32_t f)
{
    uint8_t r, g, b;
    uint32_t span = (LED_COUNT - 1) * 2;
    uint32_t t = (f / 3) % span;
    int pos = (int)(t < (uint32_t)LED_COUNT - 1 ? t : span - t);
    fb_fade(140);
    hsv((uint16_t)((f * 3) % 1536), &r, &g, &b);
    fb[pos][0] = r; fb[pos][1] = g; fb[pos][2] = b;
}

/* Slow, unmistakable states with console narration, run once before the
 * animations. If the strip is dark, this is what tells you why: each step is
 * held long enough to see, and the last two identify which end of the strip is
 * actually receiving data. */
static void bringup_sequence(void)
{
    struct { const char *what; int only; uint8_t r, g, b; } steps[] = {
        {"all LEDs WHITE",              -1, 255, 255, 255},
        {"all LEDs RED",                -1, 255,   0,   0},
        {"all LEDs GREEN",              -1,   0, 255,   0},
        {"all LEDs BLUE",               -1,   0,   0, 255},
        {"all OFF",                     -1,   0,   0,   0},
        {"ONLY the FIRST LED (DIN end)", 0, 255, 255, 255},
        {"ONLY the LAST LED (far end)",  LED_COUNT - 1, 255, 255, 255},
    };
    if (LED_BRINGUP_MS == 0) return;
    printf("\n--- bring-up sequence: %d ms per step ---\n", LED_BRINGUP_MS);
    for (unsigned s = 0; s < sizeof(steps)/sizeof(steps[0]); s++) {
        printf("  [%u/%u] %s\n", s + 1,
               (unsigned)(sizeof(steps)/sizeof(steps[0])), steps[s].what);
        fb_clear();
        for (int i = 0; i < LED_COUNT; i++) {
            if (steps[s].only >= 0 && i != steps[s].only) continue;
            fb[i][0] = steps[s].r; fb[i][1] = steps[s].g; fb[i][2] = steps[s].b;
        }
        /* Re-send every 50 ms: a strip that missed the first frame because it
         * was still powering up will pick up a later one. */
        for (int k = 0; k < LED_BRINGUP_MS / 50; k++) { fb_push(); sleep_ms(50); }
    }
    printf("  If every step was dark, the data line is not reaching DIN.\n");
    printf("  If only the LAST step lit, the strip is wired back to front.\n\n");
}

static const struct { const char *name; void (*fn)(uint32_t); } PATTERNS[] = {
    {"solid colour cycle", pat_solid},
    {"colour chase",       pat_chase},
    {"rainbow",            pat_rainbow},
    {"theatre chase",      pat_theater},
    {"bounce",             pat_bounce},
};
#define NPATTERNS (int)(sizeof(PATTERNS)/sizeof(PATTERNS[0]))

int main(void)
{
    led_strip_selftest_t rep;
    int i;

    stdio_init_all();
    for (i = 0; i < 50 && !stdio_usb_connected(); i++) sleep_ms(100);
    sleep_ms(300);

    printf("\n=== WS2812B / SK6812 driver bring-up ===\n");
    printf("clk_sys %u Hz, pin GP%d, %d LEDs, %s\n",
           (unsigned)clock_get_hz(clk_sys), LED_PIN, LED_COUNT,
           LED_CHIP == LED_CHIP_SK6812_RGBW ? "SK6812 RGBW (32-bit GRBW)"
                                            : "WS2812B (24-bit GRB)");

    if (!led_strip_init(LED_PIN, LED_COUNT, LED_CHIP)) {
        printf("led_strip_init FAILED\n");
        while (1) sleep_ms(1000);
    }
    printf("bits/pixel %u\n\n", led_strip_bits());

    printf("--- self-test: sampling GP%d while driving it ---\n", LED_PIN);
    bool ok = led_strip_selftest(&rep);
    printf("  sample rate     %u Hz (%.3f us/sample)\n",
           (unsigned)rep.sample_hz, 1e6 / rep.sample_hz);
    printf("  bits decoded    %u (expected >= %u)\n",
           (unsigned)rep.bits_decoded, led_strip_bits());
    printf("  T0H  %.3f us   [WS2812B 0.25-0.55]\n", rep.t0h_us);
    printf("  T0L  %.3f us   [WS2812B 0.70-1.00]\n", rep.t0l_us);
    printf("  T1H  %.3f us   [WS2812B 0.65-0.95]\n", rep.t1h_us);
    printf("  T1L  %.3f us   [WS2812B 0.30-0.60]\n", rep.t1l_us);
    printf("  pixel 0 decoded 0x%08X, expected 0x%08X\n",
           (unsigned)rep.first_pixel, (unsigned)rep.expect_pixel);
    printf("  RESULT: %s%s%s\n\n", ok ? "PASS" : "FAIL",
           rep.fail ? " - " : "", rep.fail ? rep.fail : "");

    printf("--- UART loopback: DEBUG_VECT \"rgb\" -> light command ---\n");
    {
        bool lb = loopback_light_test(0x20, 0x80, 0xC0);
        const show_led_state_t *ls = show_led_state();
        printf("  sent R32 G128 B192 through uart1 with LBE set\n");
        printf("  decoded R%3u G%3u B%3u\n", ls->r, ls->g, ls->b);
        printf("  RESULT: %s\n\n",
               (lb && ls->r == 0x20 && ls->g == 0x80 && ls->b == 0xC0)
               ? "PASS" : "FAIL");
    }

    printf("--- patterns (watch the strip) ---\n");
    printf("  %d LEDs on GP%d, brightness %d/255\n\n", LED_COUNT, LED_PIN,
           LED_BRIGHTNESS);
    led_strip_set_brightness(LED_BRIGHTNESS);
    show_led_init(LED_CHIP == LED_CHIP_SK6812_RGBW);

    bringup_sequence();

    {
        uint32_t frame = 0;
        int pat = -1;
        absolute_time_t next = get_absolute_time();

        while (1) {
            int now_pat = (int)((frame * FRAME_MS / PATTERN_MS) % NPATTERNS);
            if (now_pat != pat) {
                pat = now_pat;
                fb_clear();
                printf("  pattern %d/%d: %s\n", pat + 1, NPATTERNS,
                       PATTERNS[pat].name);
            }
            PATTERNS[pat].fn(frame);
            fb_push();

            frame++;
            next = delayed_by_ms(next, FRAME_MS);
            sleep_until(next);
        }
    }
}
