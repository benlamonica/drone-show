/*------------------------------------------------------------------------------
* show_led.c : Skybrush light program -> LED strip (see show_led.h)
*-----------------------------------------------------------------------------*/
#include "show_led.h"
#include "mavlink_min.h"
#include <string.h>

#define MAVLINK_MSG_DEBUG_VECT 250

static show_led_state_t st_;
static bool rgbw_;

void show_led_init(bool rgbw)
{
    memset(&st_, 0, sizeof(st_));
    rgbw_ = rgbw;
}

static uint8_t to8(float v)
{
    if (!(v > 0.0f)) return 0;          /* also catches NaN */
    if (v >= 1.0f) return 255;
    return (uint8_t)(v * 255.0f + 0.5f);
}

void show_led_convert(float rf, float gf, float bf, bool rgbw,
                      uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *w)
{
    uint8_t r8 = to8(rf), g8 = to8(gf), b8 = to8(bf), w8 = 0;

    if (rgbw) {
        /* The white channel carries the neutral component and the colour
         * channels carry only what is left, which is both brighter and more
         * efficient than mixing white from RGB. Skybrush's own RGBW drivers
         * make the same substitution; the show is authored in RGB either way. */
        w8 = r8 < g8 ? (r8 < b8 ? r8 : b8) : (g8 < b8 ? g8 : b8);
        r8 = (uint8_t)(r8 - w8);
        g8 = (uint8_t)(g8 - w8);
        b8 = (uint8_t)(b8 - w8);
    }
    *r = r8; *g = g8; *b = b8; *w = w8;
}

bool show_led_handle_frame(const uint8_t *frame, uint16_t len, uint32_t now_ms)
{
    const uint8_t *p;
    uint16_t payload;
    float x, y, z;
    char name[11];

    if (len < 12 || mav_frame_msgid(frame) != MAVLINK_MSG_DEBUG_VECT) return false;

    /* DEBUG_VECT payload: time_usec(8) x(4) y(4) z(4) name[10].
     * MAVLink 2 may truncate trailing zero bytes, so a short payload is legal
     * and the missing bytes read as zero. */
    if (frame[0] == MAVLINK_STX_V2) { payload = frame[1]; p = frame + 10; }
    else                            { payload = frame[1]; p = frame + 6;  }
    if (payload < 20) return false;                 /* need at least x,y,z */

    memcpy(&x, p + 8,  4);
    memcpy(&y, p + 12, 4);
    memcpy(&z, p + 16, 4);

    memset(name, 0, sizeof(name));
    if (payload > 20) {
        uint16_t n = payload - 20;
        if (n > 10) n = 10;
        memcpy(name, p + 20, n);
    }
    if (strncmp(name, "rgb", sizeof(name)) != 0) return false;

    show_led_convert(x, y, z, rgbw_, &st_.r, &st_.g, &st_.b, &st_.w);
    st_.have_colour = true;
    st_.updates++;
    st_.last_ms = now_ms;
    return true;
}

bool show_led_tick(uint32_t now_ms, uint32_t timeout_ms)
{
    if (!st_.have_colour || !timeout_ms) return false;
    if ((uint32_t)(now_ms - st_.last_ms) < timeout_ms) return false;
    st_.r = st_.g = st_.b = st_.w = 0;
    st_.have_colour = false;
    st_.blankings++;
    return true;
}

const show_led_state_t *show_led_state(void) { return &st_; }
