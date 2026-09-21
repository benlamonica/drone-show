/*------------------------------------------------------------------------------
* main.c : RTK GPS firmware for Raspberry Pi Pico 2 W (RP2350)
*
* Sits between a raw-capable u-blox receiver and an ArduPilot flight
* controller, replacing the receiver's own position output with an RTK
* solution. Base corrections arrive over Wi-Fi from an NTRIP caster -- in this
* setup, str2str on a Raspberry Pi attached to the base receiver.
*
*   UART0  RX (GP1) <- rover receiver, UBX RXM-RAWX + RXM-SFRBX
*          TX (GP0) -> rover receiver, configuration at boot
*   UART1     (GP4/GP5) <-> flight controller, MAVLink2 both ways: GPS_INPUT
*          out at the solution rate, telemetry and mission traffic forwarded.
*          The FC needs SERIAL<n>_PROTOCOL=2, SERIAL<n>_BAUD=115, GPS_TYPE=14.
*   Wi-Fi           <- base corrections, RTCM3 over NTRIP
*
* Corrections reach the engine through a ring buffer because they are filled
* from lwIP's callback (an alarm IRQ) while the main loop is blocked inside
* rtkpos() for ~100 ms per epoch.
*-----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"

#include "rtk_engine.h"
#include "rtk_platform.h"
#include "ntrip_client.h"
#include "mav_bridge.h"
#include "show_led.h"
#include "led_strip.h"
#include "rover_cfg.h"

/* --- base correction link -------------------------------------------------- */
/* Overridable from the build system, e.g.
 *   cmake -DCMAKE_C_FLAGS='-DWIFI_SSID=\"other-net\"'
 * so a per-site or per-airframe image needs no source edit. */
#ifndef WIFI_SSID
#define WIFI_SSID       "drone-show"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD   "olympia-ratty-realises"
#endif
#ifndef NTRIP_HOST
#define NTRIP_HOST      "192.168.4.122"   /* the Pi running str2str */
#endif
#ifndef NTRIP_PORT
#define NTRIP_PORT      2101              /* registered NTRIP port */
#endif

/* Mountpoint the caster publishes. With `str2str -out ntripc://:2101/<name>`
 * this must match <name>; with `str2str -out tcpsvr://:2101` it is ignored,
 * because a plain TCP server just starts streaming. If the caster rejects it,
 * the status line prints the mountpoints it does offer. */
#ifndef NTRIP_MOUNT
#define NTRIP_MOUNT     "RTCM3"
#endif

/* --- MAVLink link ----------------------------------------------------------
 * The ground station / show controller, reached over the same Wi-Fi. UDP is
 * what mavlink-router and MAVProxy expect from a vehicle: we bind a local port
 * and send to the server, which learns our address from the first packet. */
#ifndef MAV_SERVER_HOST
#define MAV_SERVER_HOST "192.168.4.122"
#endif
#ifndef MAV_SERVER_PORT
#define MAV_SERVER_PORT 14550
#endif
#ifndef MAV_LOCAL_PORT
#define MAV_LOCAL_PORT  14555
#endif
/* Ellipsoidal-to-MSL offset at the flying site (metres). See mav_bridge.h. */
#ifndef MAV_GEOID_SEP
#define MAV_GEOID_SEP   0.0
#endif

/* --- show lighting ---------------------------------------------------------
 * Skybrush drives external light modules with DEBUG_VECT "rgb" (show firmware
 * SHOW_LED0_TYPE 1) on the same MAVLink link, so this costs no extra wires.
 * If the light stream stops, go dark rather than freeze on a stale colour. */
#ifndef SHOW_LED_PIN
#define SHOW_LED_PIN     16
#endif
#ifndef SHOW_LED_COUNT
#define SHOW_LED_COUNT   10
#endif
/* Full scale by default: in a show the airframe is one voxel in the sky and
 * you want every candela. Ten WS2812B at white is ~600 mA, so this is a bench
 * consideration, not a flight one. */
#ifndef SHOW_LED_BRIGHTNESS
#define SHOW_LED_BRIGHTNESS 255
#endif
#ifndef SHOW_LED_CHIP
#define SHOW_LED_CHIP    LED_CHIP_WS2812B
#endif
#ifndef SHOW_LED_TIMEOUT_MS
#define SHOW_LED_TIMEOUT_MS 2000
#endif
#define NTRIP_USER      ""                /* leave empty for no authentication */
#define NTRIP_PASSWORD  ""

/* --- board wiring ---------------------------------------------------------- */
#define ROVER_UART      uart0
#define ROVER_TX_PIN    0
#define ROVER_RX_PIN    1
#define ROVER_BAUD      230400

/* One UART carries MAVLink2 both ways to the flight controller: GPS_INPUT out,
 * telemetry and mission traffic in. On the FC set SERIAL<n>_PROTOCOL=2,
 * SERIAL<n>_BAUD=115, and GPS_TYPE=14 (MAV). */
#define FC_UART         uart1
#define FC_TX_PIN       4
#define FC_RX_PIN       5
#define FC_BAUD         115200

/* Surveyed base station position (ECEF metres). Leave zero to adopt the
 * position the caster broadcasts in RTCM 1005/1006, which is what str2str
 * forwards from the base receiver. */
static const double BASE_ECEF[3] = {0.0, 0.0, 0.0};

#define SOLUTION_RATE_HZ 5

/* --- status LED -----------------------------------------------------------
 * A strict priority ladder, worst problem first:
 *
 *   off     no Wi-Fi. Nothing else can work, and an unlit LED is
 *           unmistakable from across a field.
 *   2 Hz    Wi-Fi up but no corrections arriving from the caster.
 *   10 Hz   corrections flowing, RTK still converging (no fix yet, or the
 *           rover receiver has gone quiet).
 *   solid   fixed RTK solution, current. Ready to fly.
 *
 * "Flashes per second" counts full on/off cycles, so the toggle interval is
 * half the period.
 *
 * The Pico 2 W's LED is on the CYW43 module, so setting it is an SPI
 * transaction to the wireless chip. That rules out driving it from a timer
 * IRQ, so it is toggled from the main loop; a long rtkpos() epoch stretches
 * an interval, visible as slight jitter in the fast blink and harmless for a
 * status indicator.                                                         */
#define LED_FLASH_HZ_CONVERGING 10
#define LED_FLASH_HZ_NO_LINK     2

/* A solution older than this means the rover receiver has stopped talking, so
 * a stale SOLQ_FIX must not keep the LED lit. At 5 Hz this is 15 epochs. */
#define LED_SOL_TIMEOUT_US    3000000u

/* update_led() runs once per received byte -- 23 kB/s at 230400 baud -- so it
 * must be cheap. rtkpico_tick_ms() measures 134 cycles on RP2350, nearly all
 * of it the 64-bit divide by 1000; at two calls per byte that is ~4.5% of the
 * CPU spent telling the time. time_us_32() is a single timer register read, so
 * the LED deadlines are kept in microseconds and the clock is read once per
 * call. Unsigned 32-bit differences handle the ~71 minute wrap correctly. */

/* --- ring buffers ----------------------------------------------------------
 * Single producer (IRQ), single consumer (main loop), power-of-two size, so
 * head and tail need no locking while each is written by only one side. */
#define RB_ROVER_SIZE 4096
#define RB_BASE_SIZE  8192      /* a stalled epoch must not lose an RTCM frame */

typedef struct {
    volatile uint32_t head, tail;
    volatile uint32_t dropped;
    uint32_t mask;
    uint8_t *buf;
} ringbuf_t;

static uint8_t fc_store[2048];
static uint8_t rover_store[RB_ROVER_SIZE];
static uint8_t base_store[RB_BASE_SIZE];
static ringbuf_t rb_rover = {0, 0, 0, RB_ROVER_SIZE - 1, rover_store};
static ringbuf_t rb_fc    = {0, 0, 0, 2048 - 1, fc_store};
static ringbuf_t rb_base  = {0, 0, 0, RB_BASE_SIZE  - 1, base_store};

static inline void rb_put(ringbuf_t *r, uint8_t b)
{
    uint32_t h = r->head, n = (h + 1) & r->mask;
    if (n == r->tail) { r->dropped++; return; }   /* full: drop, never block */
    r->buf[h] = b;
    r->head = n;
}
static inline int rb_get(ringbuf_t *r, uint8_t *b)
{
    uint32_t t = r->tail;
    if (t == r->head) return 0;
    *b = r->buf[t];
    r->tail = (t + 1) & r->mask;
    return 1;
}

static void rover_irq(void)
{
    while (uart_is_readable(ROVER_UART)) rb_put(&rb_rover, uart_getc(ROVER_UART));
}

static void fc_irq(void)
{
    while (uart_is_readable(FC_UART)) rb_put(&rb_fc, uart_getc(FC_UART));
}

/* Light commands ride in on the FC link; the bridge hands us every frame it
 * parses, before forwarding it unchanged. */
static volatile bool led_dirty;

static void on_fc_frame(const uint8_t *frame, uint16_t len)
{
    if (show_led_handle_frame(frame, len, rtkpico_tick_ms())) led_dirty = true;
}

/* lwIP hands us corrections here, from its alarm IRQ. */
static void ntrip_rx(const uint8_t *data, uint16_t len, void *arg)
{
    (void)arg;
    for (uint16_t i = 0; i < len; i++) rb_put(&rb_base, data[i]);
}

/* u-blox rover configuration now lives in port/rover_cfg.c so app/gps_bench.c
 * drives the receiver through the identical sequence. */

/* --- engine ---------------------------------------------------------------- */
static rtkeng_t eng;            /* ~190 KB: never on a stack */

/* File scope so the blink rate can be measured over SWD without a scope:
 * sample led_toggles twice a known interval apart. */
static uint32_t last_sol_us = 0;   /* time_us_32() of the last solution */
static uint32_t led_toggles = 0;
static uint32_t led_next_us = 0;
static bool     led_on      = false;

typedef enum {
    LED_MODE_OFF = 0,      /* no Wi-Fi                                 */
    LED_MODE_NO_LINK,      /* Wi-Fi up, no corrections                 */
    LED_MODE_CONVERGING,   /* corrections flowing, no current RTK fix  */
    LED_MODE_READY         /* fixed and current                        */
} led_mode_t;

static led_mode_t led_mode(bool wifi_ok, uint32_t last_sol_us, uint32_t now_us)
{
    const ntrip_status_t *ns = ntrip_client_status();
    bool sol_current;

    if (!wifi_ok || !ns->wifi_up)        return LED_MODE_OFF;
    if (!ntrip_client_streaming())       return LED_MODE_NO_LINK;

    sol_current = last_sol_us != 0 &&
                  (uint32_t)(now_us - last_sol_us) < LED_SOL_TIMEOUT_US;
    if (!sol_current || eng.rtk.sol.stat != SOLQ_FIX)
                                         return LED_MODE_CONVERGING;
    return LED_MODE_READY;
}

static void update_led(bool wifi_ok)
{
    uint32_t now_us = time_us_32();
    led_mode_t mode = led_mode(wifi_ok, last_sol_us, now_us);
    uint32_t half_us;

    /* Steady states are driven on transition only, so we do not issue an SPI
     * transaction to the CYW43 on every pass of the main loop. */
    if (mode == LED_MODE_OFF || mode == LED_MODE_READY) {
        bool want = (mode == LED_MODE_READY);
        if (led_on != want) {
            led_on = want;
            led_toggles++;
            ntrip_client_led(led_on);
        }
        return;
    }

    half_us = (mode == LED_MODE_CONVERGING)
              ? (500000u / LED_FLASH_HZ_CONVERGING)
              : (500000u / LED_FLASH_HZ_NO_LINK);
    if ((int32_t)(now_us - led_next_us) < 0) return;

    /* Advance the deadline by exactly one interval rather than from "now", so
     * the average rate does not drift when the main loop is briefly busy. If
     * we have fallen more than a whole interval behind -- a long rtkpos()
     * epoch, or a blocking USB write -- resynchronise instead of trying to
     * catch up with a burst of toggles. */
    led_next_us += half_us;
    if ((int32_t)(now_us - led_next_us) > 0) led_next_us = now_us + half_us;
    led_on = !led_on;
    led_toggles++;
    ntrip_client_led(led_on);
}

/* Drain whatever the bridge has queued for the flight controller, without
 * blocking: rtkpos() already owns enough of the loop. */
static void service_fc_uart(void)
{
    uint8_t b;
    while (uart_is_writable(FC_UART) && mavbridge_next_fc_byte(&b)) {
        uart_putc_raw(FC_UART, b);
    }
}

int main(void)
{
    rtkeng_cfg_t cfg;
    ntrip_config_t ncfg;
    absolute_time_t next_report;
    bool wifi_ok;
    uint8_t b;

    stdio_init_all();
    rtkpico_perf_init();

    bi_decl(bi_program_description("Drone show controller: RTK GPS, MAVLink bridge, show lighting"));
    bi_decl(bi_1pin_with_name(ROVER_RX_PIN, "rover u-blox RX"));
    bi_decl(bi_1pin_with_name(FC_TX_PIN,    "MAVLink2 to flight controller"));

    uart_init(ROVER_UART, ROVER_BAUD);
    gpio_set_function(ROVER_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(ROVER_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(ROVER_UART, true);

    uart_init(FC_UART, FC_BAUD);
    gpio_set_function(FC_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(FC_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(FC_UART, true);

    irq_set_exclusive_handler(UART0_IRQ, rover_irq);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(ROVER_UART, true, false);

    irq_set_exclusive_handler(UART1_IRQ, fc_irq);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(FC_UART, true, false);

    rover_cfg_apply(ROVER_UART, ROVER_BAUD, SOLUTION_RATE_HZ);

    if (led_strip_init(SHOW_LED_PIN, SHOW_LED_COUNT, SHOW_LED_CHIP)) {
        led_strip_set_brightness(SHOW_LED_BRIGHTNESS);
        show_led_init(SHOW_LED_CHIP == LED_CHIP_SK6812_RGBW);
        mavbridge_set_frame_hook(on_fc_frame);
        led_strip_fill(0, 0, 0, 0);
        led_strip_show();
    }

    /* Bring up Wi-Fi and the correction link. A failure here is not fatal:
     * without corrections the engine still produces a single-point position,
     * which is a far better thing to hand a flight controller than nothing,
     * and the client keeps retrying in the background. */
    memset(&ncfg, 0, sizeof(ncfg));
    ncfg.wifi_ssid       = WIFI_SSID;
    ncfg.wifi_password   = WIFI_PASSWORD;
    ncfg.host            = NTRIP_HOST;
    ncfg.port            = NTRIP_PORT;
    ncfg.mountpoint      = NTRIP_MOUNT;
    ncfg.user            = NTRIP_USER;
    ncfg.password        = NTRIP_PASSWORD;
    /* Ride out a gap rather than tear down a working link. A reconnect costs
     * roughly ten seconds of corrections and forces the filter to re-acquire
     * ambiguities, while RTKLIB keeps using differential data until it is
     * rtkeng_cfg_t.max_age old (30 s). Below that age a resumed stream is
     * still usable, so giving up early is strictly worse than waiting; past
     * it, what we are holding is stale anyway and reconnecting is right.
     * Keep this tied to max_age -- they are the same threshold. */
    ncfg.stall_timeout_ms = (uint32_t)(30.0 * 1000.0)  /* = rtkeng max_age */;
    wifi_ok = ntrip_client_init(&ncfg, ntrip_rx, NULL);

    if (wifi_ok) {
        mavbridge_cfg_t mcfg;
        memset(&mcfg, 0, sizeof(mcfg));
        mcfg.server_ip   = MAV_SERVER_HOST;
        mcfg.server_port = MAV_SERVER_PORT;
        mcfg.local_port  = MAV_LOCAL_PORT;
        mcfg.gps_id      = 0;      /* first GPS instance on the FC */
        mcfg.compid      = 220;    /* MAV_COMP_ID_GPS */
        mcfg.geoid_sep   = MAV_GEOID_SEP;
        mavbridge_init(&mcfg);
    }

    rtkeng_default_cfg(&cfg);
    cfg.mode     = PMODE_KINEMA;      /* moving rover, fixed base */
    cfg.base_fmt = RTKENG_BASE_RTCM3; /* str2str forwards RTCM3 */
    cfg.dynamics = 0;                 /* the nx^3 propagation is not worth it */
    memcpy(cfg.base_ecef, BASE_ECEF, sizeof(BASE_ECEF));

    if (!rtkeng_init(&eng, &cfg)) {
        while (1) {
            if (wifi_ok) { ntrip_client_led(true);  sleep_ms(100);
                           ntrip_client_led(false); sleep_ms(100); }
            else sleep_ms(200);
        }
    }

    next_report = make_timeout_time_ms(1000);

    while (1) {
        /* Corrections first: they should be in place before the rover epoch
         * that consumes them. */
        while (rb_get(&rb_base, &b)) {
            rtkeng_base_byte(&eng, b);
            update_led(wifi_ok);
        }

        while (rb_get(&rb_rover, &b)) {
            if (rtkeng_rover_byte(&eng, b) == RTKENG_SOLUTION) {
                mavbridge_send_gps_input(&eng.rtk);
                last_sol_us = time_us_32();
            }
            update_led(wifi_ok);
        }

        /* Flight controller link, both directions. */
        while (rb_get(&rb_fc, &b)) mavbridge_fc_byte(b);
        service_fc_uart();

        /* Push the show colour. led_strip_show() is DMA-driven, so this costs
         * a few microseconds even mid-show. */
        if (show_led_tick(rtkpico_tick_ms(), SHOW_LED_TIMEOUT_MS)) led_dirty = true;
        if (led_dirty && !led_strip_busy()) {
            const show_led_state_t *ls = show_led_state();
            led_strip_fill(ls->r, ls->g, ls->b, ls->w);
            if (led_strip_show()) led_dirty = false;
        }

        update_led(wifi_ok);
        if (wifi_ok) ntrip_client_poll();

        /* Health line on USB only: the UARTs carry GNSS data. */
        if (absolute_time_diff_us(get_absolute_time(), next_report) < 0) {
            const ntrip_status_t *ns = ntrip_client_status();
            next_report = make_timeout_time_ms(1000);
            printf("stat=%d ns=%d age=%.1f ratio=%.1f | sol=%lu fix=%lu flt=%lu "
                   "| rtkpos %lu/%lu us | eph=%lu | %lums ago\n",
                   eng.rtk.sol.stat, eng.rtk.sol.ns, (double)eng.rtk.sol.age,
                   (double)eng.rtk.sol.ratio,
                   (unsigned long)eng.stats.solutions,
                   (unsigned long)eng.stats.n_fix,
                   (unsigned long)eng.stats.n_float,
                   (unsigned long)eng.stats.last_us,
                   (unsigned long)eng.stats.max_us,
                   (unsigned long)eng.stats.eph_updates,
                   (unsigned long)((time_us_32() - last_sol_us) / 1000u));
            printf("  ntrip: state=%d wifi=%d ip=%s rssi=%ld rtcm=%lu B rc=%lu "
                   "drop r=%lu b=%lu | %s\n",
                   ns->state, ns->wifi_up,
                   ns->ip[0] ? ns->ip : "-",
                   (long)(ns->rssi_valid ? ns->rssi : 0),
                   (unsigned long)ns->bytes_received,
                   (unsigned long)ns->reconnects,
                   (unsigned long)rb_rover.dropped,
                   (unsigned long)rb_base.dropped,
                   ns->status);
            {
                const show_led_state_t *ls = show_led_state();
                printf("  led: R%3u G%3u B%3u W%3u  updates=%lu\n",
                       ls->r, ls->g, ls->b, ls->w, (unsigned long)ls->updates);
            }
            {
                const mavbridge_stats_t *ms = mavbridge_stats();
                printf("  mav: net=%d fc_sys=%d fc_frames=%lu ->net=%lu "
                       "<-net=%lu gps=%lu drop=%lu resync=%lu\n",
                       ms->net_up, ms->fc_sysid,
                       (unsigned long)ms->fc_frames, (unsigned long)ms->to_net,
                       (unsigned long)ms->from_net, (unsigned long)ms->gps_sent,
                       (unsigned long)ms->dropped_to_fc,
                       (unsigned long)ms->resyncs);
            }
            {
                static const char *names[]={
                    "OFF (no Wi-Fi)", "2 Hz (no corrections)",
                    "10 Hz (RTK converging)", "SOLID (fixed, ready)"};
                printf("  led: %s\n",
                       names[led_mode(wifi_ok, last_sol_us, time_us_32())]);
            }
        }
        tight_loop_contents();
    }
}
