/*------------------------------------------------------------------------------
* led_strip.h : WS2812B / SK6812 RGBW strip driver (RP2350 PIO + DMA)
*
* Colour ordering and bit count follow the chip: WS2812B is 24-bit GRB, SK6812
* RGBW is 32-bit GRBW. Both use the same line timing (see led_strip.pio), so
* switching chips is a configuration change, not a rewrite.
*
* Writing is DMA-driven, so show() returns immediately; a 30-LED strip takes
* ~0.9 ms on the wire, which must not be spent blocking a loop that also runs
* rtkpos().
*-----------------------------------------------------------------------------*/
#ifndef LED_STRIP_H
#define LED_STRIP_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LED_STRIP_MAX 64

typedef enum {
    LED_CHIP_WS2812B = 0,   /* 24-bit GRB  */
    LED_CHIP_SK6812_RGBW    /* 32-bit GRBW */
} led_chip_t;

bool led_strip_init(unsigned pin, unsigned count, led_chip_t chip);

/* Stage one pixel. Values are 0-255 per channel; w is ignored on WS2812B. */
void led_strip_set(unsigned i, uint8_t r, uint8_t g, uint8_t b, uint8_t w);

/* Stage the same colour on every pixel - what a drone show actually needs,
 * since the whole airframe is one voxel in the sky. */
void led_strip_fill(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

/* Push the staged frame. Non-blocking; returns false if the previous frame is
 * still going out. */
bool led_strip_show(void);
bool led_strip_busy(void);

/* Global brightness, 0-255, applied at show() time. Full white on a long strip
 * will brown out a breadboard supply long before it troubles a drone battery. */
void led_strip_set_brightness(uint8_t b);

unsigned led_strip_bits(void);       /* 24 or 32 */
unsigned led_strip_count(void);

/* --- self-test -------------------------------------------------------------
 * Drives a known pattern and samples the output pin with a second state
 * machine, then checks the pulse widths against the datasheet and decodes the
 * colour back. Verifies the wire format with no LEDs and no instruments;
 * harmless with a strip attached. Returns true on pass and fills `report`. */
typedef struct {
    bool     captured;
    uint32_t sample_hz;
    double   t0h_us, t0l_us, t1h_us, t1l_us;   /* measured means */
    uint32_t bits_decoded;
    uint32_t first_pixel;      /* decoded colour of pixel 0, chip byte order */
    uint32_t expect_pixel;
    const char *fail;
} led_strip_selftest_t;

bool led_strip_selftest(led_strip_selftest_t *report);

#ifdef __cplusplus
}
#endif
#endif /* LED_STRIP_H */
