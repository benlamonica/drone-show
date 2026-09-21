/*------------------------------------------------------------------------------
* show_led.h : Skybrush light program -> LED strip
*
* Skybrush drives external light modules with MAVLink DEBUG_VECT messages whose
* `name` is "rgb" and whose x/y/z carry red/green/blue in the range 0..1
* (SHOW_LED0_TYPE 1 on the show firmware). The colour is authored in Blender as
* RGB and converted to RGB or RGBW by the driver, so receiving RGB and
* synthesising white here loses nothing.
*
* This layer is transport-agnostic: hand it a MAVLink frame and it decides
* whether the frame is a light command.
*-----------------------------------------------------------------------------*/
#ifndef SHOW_LED_H
#define SHOW_LED_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  r, g, b, w;
    uint32_t updates;        /* light commands accepted                     */
    uint32_t blankings;      /* times the strip was blanked on link loss     */
    uint32_t last_ms;        /* when the last one arrived                   */
    bool     have_colour;
} show_led_state_t;

/* `rgbw` selects whether white is synthesised from the RGB triple. */
void show_led_init(bool rgbw);

/* Offer a complete MAVLink frame. Returns true if it was a light command and
 * the colour was updated. Anything else is ignored and must still be
 * forwarded by the caller. */
bool show_led_handle_frame(const uint8_t *frame, uint16_t len, uint32_t now_ms);

/* Blank the colour if no light command has arrived recently, so a drone that
 * loses its link goes dark rather than freezing mid-show on a stale colour.
 * Returns true if the colour changed, which the caller MUST treat as "push
 * this to the strip" -- otherwise the blanking never leaves this module. */
bool show_led_tick(uint32_t now_ms, uint32_t timeout_ms);

const show_led_state_t *show_led_state(void);

/* Exposed for testing: 0..1 float triple -> 8-bit channels, with white
 * synthesised as the common component when rgbw is enabled. */
void show_led_convert(float rf, float gf, float bf, bool rgbw,
                      uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *w);

#ifdef __cplusplus
}
#endif
#endif /* SHOW_LED_H */
