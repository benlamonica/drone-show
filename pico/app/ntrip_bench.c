/*------------------------------------------------------------------------------
* ntrip_bench.c : why does the correction link keep reconnecting?
*
* rtk_bench could not answer this. Its main loop sits inside rtkpos() for ~40 ms
* per epoch, so "the network misbehaved" and "we starved the network stack"
* look identical from outside. This removes the receiver and the engine
* entirely: nothing here blocks, so anything that still goes wrong is the
* network, the caster, or the client itself.
*
* It prints a timestamped event for every state change rather than a once-a-
* second sample, because the interesting part is the shape of one connection --
* how long until the first byte, how long it lived, and what closed it. A
* periodic sample cannot show that; it only catches whatever state the machine
* happened to be in.
*
* No sky view required. The receiver is not touched at all.
*-----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "ntrip_client.h"

#ifndef WIFI_SSID
#define WIFI_SSID       "drone-show"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD   "olympia-ratty-realises"
#endif
#ifndef NTRIP_HOST
#define NTRIP_HOST      "192.168.50.89"
#endif
#ifndef NTRIP_PORT
#define NTRIP_PORT      2101
#endif
#ifndef NTRIP_MOUNT
#define NTRIP_MOUNT     "BASE1"
#endif
#ifndef NTRIP_USER
#define NTRIP_USER      "rover"
#endif
#ifndef NTRIP_PASSWORD
#define NTRIP_PASSWORD  "changeme"
#endif

/* --- RTCM3 framing ---------------------------------------------------------
 * Only enough to identify message types. The CRC is not checked here: RTKLIB
 * does that downstream, and a framing-level view is what tells us whether the
 * caster is sending whole messages or cutting off mid-frame. */
static uint32_t rtcm_msgs, rtcm_bytes, msg1005, msg_msm, msg_other;

static void rtcm_tally(uint8_t b)
{
    static enum { SYNC, LEN1, LEN2, BODY, CRC } st = SYNC;
    static uint16_t len, got, type;
    rtcm_bytes++;
    switch (st) {
    case SYNC: if (b == 0xD3) { st = LEN1; } break;
    case LEN1: len = (uint16_t)((b & 0x03) << 8); st = LEN2; break;
    case LEN2: len |= b; got = 0; type = 0; st = len ? BODY : SYNC; break;
    case BODY:
        if (got == 0) type = (uint16_t)(b << 4);
        else if (got == 1) { type |= (uint16_t)(b >> 4); }
        if (++got >= len) st = CRC, got = 0;
        break;
    case CRC:
        if (++got >= 3) {
            rtcm_msgs++;
            if (type == 1005 || type == 1006) msg1005++;
            else if (type >= 1071 && type <= 1127) msg_msm++;
            else msg_other++;
            st = SYNC;
        }
        break;
    }
}

/* --- connection lifecycle --------------------------------------------------- */
static uint32_t conn_start_ms, first_byte_ms, conn_bytes;
static int      last_state = -1;
static uint32_t t0_ms;

static const char *sname(int s)
{
    switch (s) {
    case NTRIP_STATE_IDLE:            return "idle";
    case NTRIP_STATE_WIFI_CONNECTING: return "wifi-connecting";
    case NTRIP_STATE_WIFI_FAILED:     return "WIFI-FAILED";
    case NTRIP_STATE_TCP_CONNECTING:  return "tcp-connecting";
    case NTRIP_STATE_REQUEST_SENT:    return "request-sent";
    case NTRIP_STATE_STREAMING:       return "streaming";
    case NTRIP_STATE_BACKOFF:         return "backoff";
    default:                          return "?";
    }
}

static double now_s(void)
{
    return (to_ms_since_boot(get_absolute_time()) - t0_ms) / 1000.0;
}

static void on_data(const uint8_t *d, uint16_t n, void *arg)
{
    uint16_t i;
    (void)arg;
    if (!conn_bytes) first_byte_ms = to_ms_since_boot(get_absolute_time());
    conn_bytes += n;
    for (i = 0; i < n; i++) rtcm_tally(d[i]);
}

int main(void)
{
    ntrip_config_t cfg;
    absolute_time_t next;
    uint32_t conns = 0, total_bytes = 0;
    bool ok;

    stdio_init_all();
    sleep_ms(2500);
    t0_ms = to_ms_since_boot(get_absolute_time());

    printf("\n=== NTRIP connection bench (no GPS, nothing blocking) ===\n");
    printf("caster %s:%u  mount %s  user %s  wifi %s\n\n",
           NTRIP_HOST, (unsigned)NTRIP_PORT, NTRIP_MOUNT, NTRIP_USER, WIFI_SSID);

    memset(&cfg, 0, sizeof(cfg));
    cfg.wifi_ssid       = WIFI_SSID;
    cfg.wifi_password   = WIFI_PASSWORD;
    cfg.host            = NTRIP_HOST;
    cfg.port            = NTRIP_PORT;
    cfg.mountpoint      = NTRIP_MOUNT;
    cfg.user            = NTRIP_USER;
    cfg.password        = NTRIP_PASSWORD;
    cfg.stall_timeout_ms = 30000;
    ok = ntrip_client_init(&cfg, on_data, NULL);
    printf("[%8.3f] init %s\n", now_s(), ok ? "ok" : "FAILED");

    next = make_timeout_time_ms(5000);

    for (;;) {
        const ntrip_status_t *s;
        ntrip_client_poll();
        s = ntrip_client_status();

        /* Every transition, the moment it happens. */
        if ((int)s->state != last_state) {
            double t = now_s();
            if (last_state == NTRIP_STATE_STREAMING) {
                uint32_t now = to_ms_since_boot(get_absolute_time());
                printf("[%8.3f] CLOSED  lived %.1fs  %lu B  first byte +%lums  -> %s\n",
                       t, (now - conn_start_ms) / 1000.0, (unsigned long)conn_bytes,
                       conn_bytes ? (unsigned long)(first_byte_ms - conn_start_ms) : 0ul,
                       s->status);
                total_bytes += conn_bytes;
            } else {
                printf("[%8.3f] %-15s %s\n", t, sname(s->state), s->status);
            }
            if (s->state == NTRIP_STATE_STREAMING) {
                conns++;
                conn_start_ms = to_ms_since_boot(get_absolute_time());
                conn_bytes = 0;
                printf("[%8.3f] STREAMING (connection #%lu)\n", t, (unsigned long)conns);
            }
            last_state = (int)s->state;
        }

        if (absolute_time_diff_us(get_absolute_time(), next) > 0) continue;
        next = delayed_by_ms(next, 5000);

        printf("[%8.3f] %-11s rx %lu B  rtcm %lu msgs (1005:%lu msm:%lu other:%lu)"
               "  conns %lu  reconn %lu  rssi %ld  age %lums/%lums\n",
               now_s(), sname(s->state), (unsigned long)s->bytes_received,
               (unsigned long)rtcm_msgs, (unsigned long)msg1005,
               (unsigned long)msg_msm, (unsigned long)msg_other,
               (unsigned long)conns, (unsigned long)s->reconnects,
               (long)s->rssi, (unsigned long)s->data_age_ms,
               (unsigned long)s->stall_ms);
    }
}
