/*------------------------------------------------------------------------------
* rtk_bench.c : how repeatable is the rover's position *relative to the base*?
*
* For a drone show the absolute position hardly matters. What matters is how
* accurately one aircraft sits relative to another, and that is inherited
* entirely from the RTK baseline -- the vector from base to rover. A base whose
* surveyed position is off by a metre shifts every aircraft by the same metre
* and leaves the formation's shape untouched.
*
* So this logs the baseline in local ENU, not latitude and longitude, and the
* number to read afterwards is the *scatter* of a stationary rover, not its
* mean. That scatter is the floor on how tightly two aircraft can be spaced.
*
* Output is two interleaved streams on USB CDC:
*   "D," lines   one per solution (5 Hz), comma separated, for offline analysis
*   plain lines  once a second, human readable, so a field test needs no tools
* The "lock:" status line counts receiver-wide carrier-lock losses (see
* port/lockloss.h). To find what triggers them, two build switches take the
* suspects out one at a time, everything else unchanged:
*   -DRTK_BENCH_WIFI=0   never power the radio (so no corrections either)
*   -DRTK_BENCH_TEE=0    Wi-Fi and corrections as usual, but nothing is
*                        written back to the receiver's UART
* and, when built with -DRTK_BENCH_RAWDUMP=1, a third output stream:
*   "X,U,"/"X,R," every rover (UBX) and correction (RTCM3) byte, in hex, in the
*                order the engine consumed them, for replay through the host build
*
* Capture with:
*   pico/scripts/capture.py -o rtk.log --minutes 15
* or
*   picocom -b 115200 --logfile rtk.log /dev/cu.usbmodemXXXX
*-----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <malloc.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/cyw43_arch.h"

#include "rtk_engine.h"
#include "rover_cfg.h"
#include "ntrip_client.h"
#include "lockloss.h"
#include "rtklib.h"

/* --- site configuration ---------------------------------------------------
 * All overridable from the build, e.g.
 *   cmake -DCMAKE_C_FLAGS='-DNTRIP_HOST=\"192.168.50.89\"'
 * so a change of field needs no source edit. */
#ifndef WIFI_SSID
#define WIFI_SSID       "drone-show"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD   "olympia-ratty-realises"
#endif
/* ntrip_client.c uses ipaddr_aton(), so this must be a literal IP. */
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

/* Same wiring as the firmware; see app/main.c. */
#define ROVER_UART       uart0
#define ROVER_TX_PIN     0
#define ROVER_RX_PIN     1
#define ROVER_BAUD       230400
#define SOLUTION_RATE_HZ 5

static rtkeng_t eng;                 /* ~190 KB: never on a stack */
static lockloss_t lockloss;

#ifndef RTK_BENCH_WIFI
#define RTK_BENCH_WIFI 1
#endif
#ifndef RTK_BENCH_TEE
#define RTK_BENCH_TEE  1
#endif

/* --- ring buffers ----------------------------------------------------------
 * Corrections arrive from lwIP's alarm IRQ while the main loop may be inside
 * rtkpos() for tens of milliseconds, so they cannot be consumed in place. */
#define RB_SIZE 4096
typedef struct { volatile uint16_t head, tail; uint8_t buf[RB_SIZE]; } rb_t;
static rb_t rb_base;    /* corrections -> our RTKLIB engine                  */
static rb_t rb_rtcm;    /* the same bytes -> the receiver's own RTK engine    */
static rb_t rb_rover;



/* --- raw capture -----------------------------------------------------------
 * When our solution disagrees with the receiver's own, the text log cannot say
 * whether the engine or its input is at fault. Replaying the exact byte stream
 * through the host build can: same bytes, same order, so the same epochs pair
 * with the same corrections. Off by default -- it roughly triples USB traffic. */
#ifndef RTK_BENCH_RAWDUMP
#define RTK_BENCH_RAWDUMP 0
#endif
#if RTK_BENCH_RAWDUMP
typedef struct { char tag; uint16_t n; uint8_t b[48]; } dump_t;
static dump_t dump_rov = {'U', 0, {0}}, dump_cor = {'R', 0, {0}};

static void dump_flush(dump_t *d)
{
    static const char hx[] = "0123456789ABCDEF";
    char line[4 + 2 * sizeof(d->b) + 1];
    unsigned i, k = 0;
    if (!d->n) return;
    line[k++] = 'X'; line[k++] = ','; line[k++] = d->tag; line[k++] = ',';
    for (i = 0; i < d->n; i++) {
        line[k++] = hx[d->b[i] >> 4];
        line[k++] = hx[d->b[i] & 15];
    }
    line[k++] = '\n';
    fwrite(line, 1, k, stdout);
    d->n = 0;
}
static inline void dump_byte(dump_t *d, uint8_t b)
{
    d->b[d->n++] = b;
    if (d->n == sizeof(d->b)) dump_flush(d);
}
#define DUMP_BYTE(d, b) dump_byte(&(d), (b))
#define DUMP_FLUSH(d)   dump_flush(&(d))
#else
#define DUMP_BYTE(d, b) ((void)0)
#define DUMP_FLUSH(d)   ((void)0)
#endif

static inline void rb_put(rb_t *r, uint8_t b)
{
    uint16_t n = (uint16_t)((r->head + 1u) & (RB_SIZE - 1u));
    if (n != r->tail) { r->buf[r->head] = b; r->head = n; }
}
static inline int rb_get(rb_t *r, uint8_t *b)
{
    if (r->head == r->tail) return 0;
    *b = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1u) & (RB_SIZE - 1u));
    return 1;
}

/* RTCM3 framing of the bytes going out to the receiver. The drain below stops
 * whenever the TX FIFO is full, which is usually partway through a frame, and
 * the once-a-second UBX polls share that UART. Written mid-frame they land
 * inside the RTCM message, which then fails its CRC at the receiver -- the
 * 2026-10-03 field test saw ~1% of corrections rejected that way while our
 * engine, reading the same bytes before the UART, saw none bad. Polls wait
 * until this says the link is between frames. */
static uint16_t rtcm_tx_pos, rtcm_tx_len;
static uint8_t  rtcm_tx_b1;

static void rtcm_tx_track(uint8_t c)
{
    if (rtcm_tx_pos == 0 && c != 0xD3) return;      /* not inside a frame */
    rtcm_tx_pos++;
    if (rtcm_tx_pos == 2) rtcm_tx_b1 = c;
    if (rtcm_tx_pos == 3) rtcm_tx_len = (uint16_t)((((rtcm_tx_b1 & 3u) << 8) | c) + 6u);
    if (rtcm_tx_pos >= 3 && rtcm_tx_pos == rtcm_tx_len) rtcm_tx_pos = 0;
}
/* The rover UART must be drained from an interrupt, not polled. The FIFO is
 * 32 bytes -- about 1.4 ms at 230400 baud -- while this main loop can be away
 * far longer: rtkpos() runs 18-29 ms, lwIP services Wi-Fi from an alarm IRQ,
 * and printf() to USB CDC blocks. Polling drops bytes, which corrupts the long
 * RXM-SFRBX frames, so ephemerides never complete and run_epoch() stays stuck
 * below its neph<4 gate: no solution, and a display that reads "0 sats" while
 * the receiver is in fact tracking perfectly well. app/main.c has always done
 * this on an IRQ; gps_bench only got away with polling because it had no
 * Wi-Fi competing for the CPU. */
static void rover_irq(void)
{
    while (uart_is_readable(ROVER_UART)) rb_put(&rb_rover, (uint8_t)uart_getc(ROVER_UART));
}

/* Every correction byte goes two places: into our engine, and back out of the
 * UART into the receiver's own RTK engine. Running both from one stream is the
 * whole point -- the two solutions then see identical corrections, so any
 * difference between them is the engine and not the data. */
static void ntrip_rx(const uint8_t *d, uint16_t n, void *arg)
{
    (void)arg;
    for (uint16_t i = 0; i < n; i++) {
        rb_put(&rb_base, d[i]);
#if RTK_BENCH_TEE
        rb_put(&rb_rtcm, d[i]);
#endif
    }
}

/* --- running statistics ----------------------------------------------------
 * Welford, so a long run cannot lose precision the way sum-of-squares does. */
typedef struct { uint32_t n; double mean[3], m2[3]; } stats_t;
static stats_t st_fix, st_all;

static void stats_add(stats_t *s, const double *enu)
{
    int i;
    s->n++;
    for (i = 0; i < 3; i++) {
        double d = enu[i] - s->mean[i];
        s->mean[i] += d / (double)s->n;
        s->m2[i]   += d * (enu[i] - s->mean[i]);
    }
}
static double stats_sd(const stats_t *s, int i)
{
    return s->n > 1 ? sqrt(s->m2[i] / (double)(s->n - 1)) : 0.0;
}

/* Independent UBX framing, mirroring gps_bench. The engine's counters only say
 * what it accepted; these say what actually arrived on the wire, which is the
 * difference between "the receiver is quiet" and "we are losing bytes". */
static uint32_t rx_bytes, ubx_rawx, ubx_sfrbx, ubx_other, ubx_badck;

/* What is actually on the wire. Two independent framers scoring zero on bytes
 * that are demonstrably arriving means the question is no longer "why did the
 * decoder fail" but "what are these bytes" -- so capture them verbatim. */
/* UBX-MON-VER (0x0A 0x04) is the receiver's heartbeat: poll it and a healthy
 * receiver answers with its software and hardware version. It needs no
 * satellites, so it is the one check that works on a bench indoors, and a reply
 * proves the whole command path -- our TX reached it, it parsed UBX, and its
 * answer reached our RX. No reply means the receiver is not listening to UBX,
 * which is a completely different fault from "no sky view". */
/* MON-VER is swVersion[30] + hwVersion[10] + N*extension[30]. On M8 the
 * swVersion string is always "EXT CORE x.xx (hash)": it names the ROM base, not
 * the product firmware, and reads identically on an M8P and a plain M8. What
 * actually identifies the receiver -- "FWVER=HPG 1.40REF", "MOD=NEO-M8P-2" --
 * lives in the extension blocks, so reporting sw/hw alone cannot tell you
 * whether the thing in front of you has an RTK engine. */
#define MONVER_EXT_MAX 8
static char     monver_sw[31], monver_hw[11];
static char     monver_ext[MONVER_EXT_MAX][31];
static uint8_t  monver_ext_n;
static int      monver_valid;

/* UBX-CFG-GNSS readback. rover_cfg_apply() sends CFG-GNSS without checking for
 * an ACK, and a receiver asked for a constellation it does not have simply
 * declines -- which is exactly how "Galileo enabled" went unnoticed on a module
 * whose MON-VER says GPS;GLO;BDS. Polling the configuration back is the only
 * way to report what is genuinely tracking rather than what we requested. */
static uint8_t  cfggnss[60];
static uint16_t cfggnss_len;
static int      cfggnss_valid;

static const char *gnss_name(uint8_t id)
{
    switch (id) {
    case 0: return "GPS";   case 1: return "SBAS"; case 2: return "GAL";
    case 3: return "BDS";   case 4: return "IMES"; case 5: return "QZSS";
    case 6: return "GLO";   default: return "?";
    }
}

static uint8_t  raw_prev[48];
static uint16_t raw_n;
static uint32_t sync_b5;          /* 0xB5 seen at all: UBX present or absent */

/* UBX-MON-HW (0x0A 0x09) answers two questions no amount of position data can:
 * whether the receiver is actually powering the antenna's LNA, and whether the
 * RF front end is being desensitised. aStatus/aPower come from the antenna
 * supervisor; agcCnt and noisePerMS show the front end fighting interference,
 * which on a board that also runs a 2.4 GHz radio is a real possibility. */
static uint8_t  monhw[60];
static int      monhw_valid;

/* --- the receiver's own RTK solution ----------------------------------------
 * NAV-RELPOSNED (0x01 0x3C), M8P version 0, 40 bytes. relPos* are centimetres
 * and relPosHP* the 0.1 mm remainder, so the full value is cm*0.01 + hp*0.0001.
 * acc* are 0.1 mm. carrSoln lives in flags bits 3-4: 0 none, 1 float, 2 fixed
 * -- the direct analogue of RTKLIB's SOLQ_NONE / FLOAT / FIX. */
/* RXM-RTCM (0x02 0x32), 8 bytes: version, flags, subType, refStation, msgType.
 * flags bit 0 = crcFailed, bits 1-2 = msgUsed (0 unknown, 1 not used, 2 used). */
static uint32_t rtcm_in, rtcm_used, rtcm_crcfail;
static uint16_t rtcm_last_type;

static uint8_t  relpos[40];
static uint8_t  navpvt[92];
static int      relpos_valid, navpvt_valid;

static int32_t  rd_i4(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}
static uint32_t rd_u4(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
/* NED in metres, plus the high-precision remainder. */
static void relpos_ned(double *ned, double *acc, unsigned *carr)
{
    ned[0] = rd_i4(relpos +  8) * 0.01 + (int8_t)relpos[20] * 0.0001;  /* N */
    ned[1] = rd_i4(relpos + 12) * 0.01 + (int8_t)relpos[21] * 0.0001;  /* E */
    ned[2] = rd_i4(relpos + 16) * 0.01 + (int8_t)relpos[22] * 0.0001;  /* D */
    acc[0] = rd_u4(relpos + 24) * 0.0001;
    acc[1] = rd_u4(relpos + 28) * 0.0001;
    acc[2] = rd_u4(relpos + 32) * 0.0001;
    *carr  = (rd_u4(relpos + 36) >> 3) & 3u;
}

static const char *astatus_name(uint8_t v)
{
    switch (v) {
    case 0:  return "INIT";
    case 1:  return "DONTKNOW";
    case 2:  return "OK";
    case 3:  return "SHORT";
    case 4:  return "OPEN";
    default: return "?";
    }
}
static const char *apower_name(uint8_t v)
{
    return v == 0 ? "OFF" : v == 1 ? "ON" : "DONTKNOW";
}

static void ubx_tally(uint8_t b)
{
    static enum { S1, S2, CLS, ID, L1, L2, BODY, CKA, CKB } st = S1;
    static uint8_t  cls, id, cka, ckb, rxa;
    static uint16_t len, got;
    switch (st) {
    case S1:   if (b == 0xB5) st = S2; break;
    case S2:   st = (b == 0x62) ? CLS : S1; break;
    case CLS:  cls = b; cka = b; ckb = b; st = ID; break;
    case ID:   id = b; cka += b; ckb += cka; st = L1; break;
    case L1:   len = b; cka += b; ckb += cka; st = L2; break;
    case L2:   len |= (uint16_t)b << 8; cka += b; ckb += cka; got = 0;
               st = len ? BODY : CKA; break;
    case BODY:
        if      (cls == 0x0A && id == 0x09 && got < sizeof(monhw))  monhw[got]  = b;
        else if (cls == 0x01 && id == 0x3C && got < sizeof(relpos)) relpos[got] = b;
        else if (cls == 0x01 && id == 0x07 && got < sizeof(navpvt)) navpvt[got] = b;
        else if (cls == 0x02 && id == 0x32 && got < 8)              navpvt[64 + got] = b;
        else if (cls == 0x06 && id == 0x3E && got < sizeof(cfggnss)) cfggnss[got] = b;
        else if (cls == 0x0A && id == 0x04) {
            if      (got < 30) monver_sw[got]      = (char)b;
            else if (got < 40) monver_hw[got - 30] = (char)b;
            else {
                uint16_t e = (uint16_t)((got - 40) / 30);   /* extension index */
                if (e < MONVER_EXT_MAX) {
                    monver_ext[e][(got - 40) % 30] = (char)b;
                    if (e + 1 > monver_ext_n) monver_ext_n = (uint8_t)(e + 1);
                }
            }
        }
        cka += b; ckb += cka; if (++got >= len) st = CKA;
        break;
    case CKA:  rxa = b; st = CKB; break;
    case CKB:
        if (rxa == cka && b == ckb) {
            if      (cls == 0x0A && id == 0x04) monver_valid = 1;
            else if (cls == 0x06 && id == 0x3E) {
                cfggnss_len = len > sizeof(cfggnss) ? sizeof(cfggnss) : len;
                cfggnss_valid = 1;
            }
            else if (cls == 0x0A && id == 0x09) monhw_valid  = 1;
            else if (cls == 0x01 && id == 0x3C) relpos_valid = 1;
            else if (cls == 0x01 && id == 0x07) navpvt_valid = 1;
            else if (cls == 0x02 && id == 0x32) {
                const uint8_t *r = navpvt + 64;      /* scratch, see BODY above */
                rtcm_in++;
                if (r[1] & 1)               rtcm_crcfail++;
                if (((r[1] >> 1) & 3) == 2) rtcm_used++;
                rtcm_last_type = (uint16_t)(r[6] | (r[7] << 8));
            }
            else if (cls == 0x02 && id == 0x15) ubx_rawx++;
            else if (cls == 0x02 && id == 0x13) ubx_sfrbx++;
            else                                ubx_other++;
        } else ubx_badck++;
        st = S1;
        break;
    }
}

/* Raw receiver state, independent of whether the filter produced a solution.
 * Without this the summary can only echo eng.rtk.sol, which stays at its
 * initial zeros whenever run_epoch() returns RTKENG_OBS -- indistinguishable
 * from an antenna that sees nothing. */
static int    last_nobs;        /* measurements in the last decoded epoch */
static double last_cn0;         /* median C/N0 of that epoch, dBHz        */

static void snapshot_obs(const obs_t *o)
{
    double v[MAXOBS];
    int i, n = 0;
    last_nobs = o->n;
    for (i = 0; i < o->n && n < MAXOBS; i++)
        if (o->data[i].SNR[0]) v[n++] = o->data[i].SNR[0] * SNR_UNIT;
    if (!n) { last_cn0 = 0.0; return; }
    for (i = 1; i < n; i++) {            /* insertion sort: n is at most ~30 */
        double k = v[i]; int j = i - 1;
        while (j >= 0 && v[j] > k) { v[j+1] = v[j]; j--; }
        v[j+1] = k;
    }
    last_cn0 = v[n / 2];
}

static const char *ntrip_state_name(int st)
{
    switch (st) {
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

static const char *carr_name(unsigned c)
{
    return c == 2 ? "FIX" : c == 1 ? "FLOAT" : "none";
}

static const char *solq_name(int q)
{
    switch (q) {
    case SOLQ_FIX:    return "FIX";
    case SOLQ_FLOAT:  return "FLOAT";
    case SOLQ_DGPS:   return "DGPS";
    case SOLQ_SINGLE: return "SINGLE";
    default:          return "NONE";
    }
}

/* Baseline from base to rover, rotated into ENU at the base. Returns 0 if the
 * base position is not known yet -- with base_ecef left at zero the engine
 * adopts it from RTCM 1005, which only arrives once the caster is streaming. */
static int baseline_enu(double *enu, double *sd)
{
    const double *rb = eng.rtk.opt.rb;
    double basellh[3], d[3], P[9], Q[9];
    const sol_t *s = &eng.rtk.sol;
    int i;

    if (rb[0] == 0.0 && rb[1] == 0.0 && rb[2] == 0.0) return 0;
    ecef2pos(rb, basellh);
    for (i = 0; i < 3; i++) d[i] = s->rr[i] - rb[i];
    ecef2enu(basellh, d, enu);

    /* qr is the ECEF covariance as xx,yy,zz,xy,yz,zx. */
    P[0] = s->qr[0]; P[4] = s->qr[1]; P[8] = s->qr[2];
    P[1] = P[3] = s->qr[3];
    P[5] = P[7] = s->qr[4];
    P[2] = P[6] = s->qr[5];
    covenu(basellh, P, Q);
    sd[0] = sqrt(fabs(Q[0])); sd[1] = sqrt(fabs(Q[4])); sd[2] = sqrt(fabs(Q[8]));
    return 1;
}

int main(void)
{
    rtkeng_cfg_t cfg;
    ntrip_config_t ncfg;
    absolute_time_t next;
    uint32_t n_sol = 0, n_fix = 0, n_float = 0;
    uint32_t last_frames = 0, last_rx = 0, rx_rate = 0;
    bool wifi_ok, polls_due = false;

    stdio_init_all();
    sleep_ms(2500);

    printf("\n=== RTK relative-accuracy bench (RP2350) ===\n");
    printf("caster %s:%u mount %s  wifi %s\n",
           NTRIP_HOST, (unsigned)NTRIP_PORT, NTRIP_MOUNT, WIFI_SSID);

    uart_init(ROVER_UART, ROVER_BAUD);
    gpio_set_function(ROVER_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(ROVER_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(ROVER_UART, true);
    printf("configuring receiver...\n");
    rover_cfg_apply(ROVER_UART, ROVER_BAUD, SOLUTION_RATE_HZ);

    /* Only after configuration: rover_cfg_apply() does its own blocking writes
     * and baud changes, which an active RX interrupt would race. */
    irq_set_exclusive_handler(UART0_IRQ, rover_irq);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(ROVER_UART, true, false);

    rtkeng_default_cfg(&cfg);
    cfg.mode     = PMODE_KINEMA;        /* what the aircraft will actually fly */
    cfg.base_fmt = RTKENG_BASE_RTCM3;
    cfg.dynamics = 0;
    memset(cfg.base_ecef, 0, sizeof(cfg.base_ecef));   /* adopt from RTCM 1005 */
    if (!rtkeng_init(&eng, &cfg)) {
        for (;;) { printf("rtkeng_init failed\n"); sleep_ms(1000); }
    }

    memset(&ncfg, 0, sizeof(ncfg));
    ncfg.wifi_ssid      = WIFI_SSID;
    ncfg.wifi_password  = WIFI_PASSWORD;
    ncfg.host           = NTRIP_HOST;
    ncfg.port           = NTRIP_PORT;
    ncfg.mountpoint     = NTRIP_MOUNT;
    ncfg.user           = NTRIP_USER;
    ncfg.password       = NTRIP_PASSWORD;
    /* Ride out a gap rather than tear down a working link. A reconnect costs
     * roughly ten seconds of corrections and forces the filter to re-acquire
     * ambiguities, while RTKLIB keeps using differential data until it is
     * rtkeng_cfg_t.max_age old (30 s). Below that age a resumed stream is
     * still usable, so giving up early is strictly worse than waiting; past
     * it, what we are holding is stale anyway and reconnecting is right.
     * Keep this tied to max_age -- they are the same threshold. */
    ncfg.stall_timeout_ms = (uint32_t)(cfg.max_age * 1000.0);
#if RTK_BENCH_WIFI
    wifi_ok = ntrip_client_init(&ncfg, ntrip_rx, NULL);
    printf("wifi/ntrip init: %s\n\n", wifi_ok ? "ok" : "FAILED");
#else
    /* The CYW43 stays unpowered: no radio, no corrections, only the receiver. */
    wifi_ok = false;
    printf("wifi/ntrip: disabled at build (RTK_BENCH_WIFI=0)\n\n");
#endif
    lockloss_init(&lockloss);

    /* link_state, reconn and rssi ride along with every solution so a bad epoch
     * can be attributed. Without them a correction dropout and a genuinely poor
     * fix look identical in the position data afterwards. age_s is the decisive
     * one: it is how stale the differential was for THIS epoch. */
    printf("D,t_s,stat,ns,age_s,ratio,E_m,N_m,U_m,sdE,sdN,sdU,rtkpos_us,link,reconn,rssi,"
           "u_carr,u_E,u_N,u_U,u_accE,u_accN,u_accD,u_sv\n");
    next = make_timeout_time_ms(1000);

    for (;;) {
        uint8_t b;

        /* The client is a state machine driven from here; lwIP runs in the
         * background but the join, connect and reconnect steps only advance
         * when polled. Omitting this leaves it stuck at init forever. */
        if (wifi_ok) ntrip_client_poll();

        /* Corrections first: they must be in place before the rover epoch that
         * consumes them, or the filter runs a needlessly old differential. */
        while (rb_get(&rb_base, &b)) {
            DUMP_BYTE(dump_cor, b);
            rtkeng_base_byte(&eng, b);
        }
        DUMP_FLUSH(dump_cor);

        /* Same bytes out to the receiver. Strictly bounded by what the TX FIFO
         * will accept right now: this loop sits inside rtkpos() for ~40 ms and
         * must never wait on the UART. At ~250 B/s the 4 KB ring absorbs any
         * stall with room to spare. */
        while (uart_is_writable(ROVER_UART)) {
            uint8_t c;
            if (!rb_get(&rb_rtcm, &c)) break;
            uart_putc_raw(ROVER_UART, c);
            rtcm_tx_track(c);
        }

        /* Polls go out only between RTCM frames; see rtcm_tx_track(). */
        if (polls_due && rtcm_tx_pos == 0) {
            ubx_send(ROVER_UART, 0x0A, 0x09, NULL, 0);   /* poll UBX-MON-HW  */
            ubx_send(ROVER_UART, 0x0A, 0x04, NULL, 0);   /* poll UBX-MON-VER */
            ubx_send(ROVER_UART, 0x06, 0x3E, NULL, 0);   /* poll UBX-CFG-GNSS */
            polls_due = false;
        }

        while (rb_get(&rb_rover, &b)) {
            int r;
            rx_bytes++;
            if (b == 0xB5) sync_b5++;
            if (raw_n < sizeof(raw_prev)) raw_prev[raw_n++] = b;
            ubx_tally(b);
            DUMP_BYTE(dump_rov, b);
            r = rtkeng_rover_byte(&eng, b);
            /* Snapshot the raw observations on every epoch, solution or not.
             * Reading only eng.rtk.sol hides the receiver completely whenever
             * the engine declines to solve. */
            if (r == RTKENG_SOLUTION || r == RTKENG_OBS) {
                snapshot_obs(&eng.rov.obs);
                lockloss_epoch(&lockloss, eng.rov.obs.data, eng.rov.obs.n);
            }
            if (r == RTKENG_SOLUTION) {
                const sol_t *s = &eng.rtk.sol;
                double enu[3], sd[3] = {0, 0, 0};
                n_sol++;
                if (s->stat == SOLQ_FIX)   n_fix++;
                if (s->stat == SOLQ_FLOAT) n_float++;
                if (baseline_enu(enu, sd)) {
                    stats_add(&st_all, enu);
                    if (s->stat == SOLQ_FIX) stats_add(&st_fix, enu);
                    const ntrip_status_t *nl = ntrip_client_status();
                    double ned[3] = {0,0,0}, uacc[3] = {0,0,0};
                    unsigned carr = 0;
                    if (relpos_valid) relpos_ned(ned, uacc, &carr);
                    /* The receiver reports NED; ours is ENU. Up is -Down, so the
                     * two columns line up directly for differencing offline. */
                    printf("D,%.3f,%d,%d,%.1f,%.1f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%lu,%d,%lu,%ld,"
                           "%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u\n",
                           time_us_32() / 1e6, s->stat, s->ns, s->age, s->ratio,
                           enu[0], enu[1], enu[2], sd[0], sd[1], sd[2],
                           (unsigned long)eng.stats.last_us,
                           (int)nl->state, (unsigned long)nl->reconnects,
                           (long)nl->rssi,
                           carr, ned[1], ned[0], -ned[2],
                           uacc[1], uacc[0], uacc[2],
                           navpvt_valid ? (unsigned)navpvt[23] : 0u);
                }
            }
        }
        /* Before the next correction bytes, so replay sees the same order. */
        DUMP_FLUSH(dump_rov);

        if (absolute_time_diff_us(get_absolute_time(), next) > 0) continue;
        next = delayed_by_ms(next, 1000);

        polls_due = true;

        /* The failure that cost a field trip: the receiver drops out of UBX and
         * emits something else (NMEA), so every frame counter sits at zero while
         * bytes pour in. Nothing downstream can tell that apart from "no sky
         * view", because both look like no observations. Catch it explicitly on
         * the byte stream, and re-run the configuration -- the receiver keeps
         * its settings in battery-backed RAM, so a Pico reset alone does not
         * necessarily clear a bad state, but re-sending the sequence does. */
        {
            static uint32_t quiet_s;
            uint32_t frames = ubx_rawx + ubx_sfrbx + ubx_other;
            if (frames == last_frames && rx_bytes > last_rx + 200) {
                if (++quiet_s == 5) {
                    printf("!! receiver is sending %lu B/s but NO UBX frames "
                           "(0xB5 never seen).\n"
                           "!! this is NOT a sky-view problem -- it has left UBX "
                           "mode. re-running config.\n",
                           (unsigned long)(rx_bytes - last_rx));
                    rover_cfg_apply(ROVER_UART, ROVER_BAUD, SOLUTION_RATE_HZ);
                    monver_valid = 0;
                    quiet_s = 0;
                }
            } else {
                quiet_s = 0;
            }
            last_frames = frames;
            rx_rate = rx_bytes - last_rx;   /* this second's bytes, for the summary */
            last_rx = rx_bytes;
        }

        {
            const sol_t *s = &eng.rtk.sol;
            const rtkeng_stats_t *es = &eng.stats;
            {
                uint16_t k;
                printf("   raw(%u B/s, 0xB5 x%lu): ",
                       (unsigned)raw_n, (unsigned long)sync_b5);
                for (k = 0; k < raw_n; k++) {
                    char c = (char)raw_prev[k];
                    putchar((c >= 32 && c < 127) ? c : '.');
                }
                printf("\n   hex: ");
                for (k = 0; k < raw_n && k < 24; k++) printf("%02X ", raw_prev[k]);
                printf("\n");
                raw_n = 0; sync_b5 = 0;
            }
            printf("rov %5luB/s RAWX %-3lu SFRBX %-3lu bad %-3lu | "
                   "%-6s obs %2d C/N0 %2.0f  eph %2lu  sats %2d  age %4.1fs  ratio %5.1f  fix %lu%%  "
                   "sd(fix) E %.3f N %.3f U %.3f m  n=%lu  rtkpos %luus\n",
                   (unsigned long)rx_rate, (unsigned long)ubx_rawx,
                   (unsigned long)ubx_sfrbx, (unsigned long)ubx_badck,
                   solq_name(s->stat), last_nobs, last_cn0,
                   (unsigned long)es->eph_updates, s->ns, s->age, s->ratio,
                   (unsigned long)(n_sol ? 100u * n_fix / n_sol : 0u),
                   stats_sd(&st_fix, 0), stats_sd(&st_fix, 1), stats_sd(&st_fix, 2),
                   (unsigned long)st_fix.n, (unsigned long)es->last_us);

            {
                const ntrip_status_t *n = ntrip_client_status();
                printf("   link: %-15s wifi %s  ip %-15s rx %lu B  joins %lu  reconn %lu  rssi %ld"
                       "  age %lums/%lums\n",
                       ntrip_state_name(n->state), n->wifi_up ? "up" : "DOWN",
                       n->ip[0] ? n->ip : "-", (unsigned long)n->bytes_received,
                       (unsigned long)n->joins, (unsigned long)n->reconnects,
                       (long)n->rssi,
                       (unsigned long)n->data_age_ms, (unsigned long)n->stall_ms);
                if (n->status[0]) printf("   last: %s\n", n->status);
            }
            /* RTKLIB records why an epoch failed in rtk.errbuf. Without this
             * a NONE epoch is indistinguishable from a dead antenna, when in
             * fact estpos() has usually converged and valsol() then rejected
             * the fix on chi-square or GDOP -- pntpos.c writes ns/age/ratio
             * BEFORE validating, which is why the summary still shows
             * satellites with age and ratio pinned at zero. */
            if (monver_valid) {
                unsigned e;
                monver_sw[30] = monver_hw[10] = '\0';
                printf("   rcvr: UBX alive  sw '%s'  hw '%s'\n", monver_sw, monver_hw);
                for (e = 0; e < monver_ext_n; e++) {
                    monver_ext[e][30] = '\0';
                    if (monver_ext[e][0]) printf("   rcvr: ext '%s'\n", monver_ext[e]);
                }
            } else {
                printf("   rcvr: NO MON-VER reply -- receiver is not answering UBX.\n");
            }
            if (cfggnss_valid && cfggnss_len >= 4) {
                unsigned nb = cfggnss[3], k;
                printf("   rcvr: gnss enabled:");
                for (k = 0; k < nb && 4u + 8u * k + 8u <= cfggnss_len; k++) {
                    const uint8_t *blk = cfggnss + 4 + 8 * k;
                    if (blk[4] & 0x01)
                        printf(" %s(%u/%u)", gnss_name(blk[0]), blk[1], blk[2]);
                }
                printf("\n");
            }
            if (relpos_valid) {
                double ned[3], uacc[3];
                unsigned carr;
                relpos_ned(ned, uacc, &carr);
                printf("   ublox: %-5s  E %8.4f N %8.4f U %8.4f  acc %.3f/%.3f/%.3f m  sv %u\n",
                       carr_name(carr), ned[1], ned[0], -ned[2],
                       uacc[1], uacc[0], uacc[2],
                       navpvt_valid ? (unsigned)navpvt[23] : 0u);
            } else {
                printf("   ublox: no NAV-RELPOSNED yet -- receiver is not running RTK.\n");
            }
            printf("   rtcm->rx: %lu in  %lu used  %lu crc-fail   last type %u\n",
                   (unsigned long)rtcm_in, (unsigned long)rtcm_used,
                   (unsigned long)rtcm_crcfail, (unsigned)rtcm_last_type);
            printf("   lock: %lu receiver-wide carrier losses in %.1f min",
                   (unsigned long)lockloss.events, lockloss_minutes(&lockloss));
            if (lockloss.events)
                printf(" (one per %.0f s; last took %d of %d sats)",
                       lockloss_minutes(&lockloss) * 60.0 / lockloss.events,
                       lockloss.last_nslip, lockloss.last_ntrk);
            printf("   [wifi %s, tee %s]\n", RTK_BENCH_WIFI ? "on" : "off",
                   RTK_BENCH_TEE ? "on" : "off");
            {
                /* newlib grows the arena to cover the largest epoch and only
                 * trims it back past 128 KB of free top, so its maximum is the
                 * heap high-water mark: what rtkpos() needed at its worst. */
                extern char __end__[], __HeapLimit[];
                static size_t arena_max;
                struct mallinfo mi = mallinfo();
                if (mi.arena > arena_max) arena_max = mi.arena;
                printf("   heap: peak %u of %u KB  in use %u KB\n",
                       (unsigned)(arena_max / 1024),
                       (unsigned)((size_t)(__HeapLimit - __end__) / 1024),
                       (unsigned)(mi.uordblks / 1024));
            }
            if (monhw_valid) {
                unsigned noise = (unsigned)(monhw[16] | (monhw[17] << 8));
                unsigned agc   = (unsigned)(monhw[18] | (monhw[19] << 8));
                printf("   ant: %-8s power %-8s  agc %u/8191  noise %u  jam %u/255\n",
                       astatus_name(monhw[20]), apower_name(monhw[21]),
                       agc, noise, (unsigned)monhw[45]);
            }
            if (eng.rtk.neb > 0) {
                eng.rtk.errbuf[eng.rtk.neb < (int)sizeof(eng.rtk.errbuf)
                               ? eng.rtk.neb : (int)sizeof(eng.rtk.errbuf) - 1] = '\0';
                printf("   why: %s", eng.rtk.errbuf);
                if (eng.rtk.errbuf[0] && eng.rtk.errbuf[strlen(eng.rtk.errbuf)-1] != '\n')
                    printf("\n");
                eng.rtk.neb = 0;
            }
            if (!ntrip_client_streaming()) {
                const char *hb = NULL;
                uint16_t hn = ntrip_client_header(&hb);
                if (hn) {
                    uint16_t k, lim = hn > 160 ? 160 : hn;
                    printf("   caster said (%u bytes): ", (unsigned)hn);
                    for (k = 0; k < lim; k++) {
                        char c = hb[k];
                        putchar((c >= 32 && c < 127) ? c : (c == '\n' ? '|' : '.'));
                    }
                    printf("\n   hex: ");
                    for (k = 0; k < 24 && k < hn; k++) printf("%02X ", (uint8_t)hb[k]);
                    printf("\n");
                }
                printf("   -> no corrections: check wifi, caster IP, mount and credentials.\n");
            } else if (eng.rtk.opt.rb[0] == 0.0) {
                printf("   -> corrections arriving but no base position yet (waiting for RTCM 1005).\n");
            } else if (last_nobs && es->eph_updates < 4) {
                printf("   -> tracking %d sats but only %lu ephemerides: still in the 30 s\n",
                       last_nobs, (unsigned long)es->eph_updates);
                printf("      LNAV cycle. run_epoch() will not solve until 4 arrive.\n");
            } else if (!last_nobs) {
                printf("   -> receiver is decoding epochs with no measurements: check sky view.\n");
            } else if (s->stat == SOLQ_SINGLE) {
                printf("   -> corrections present but filter still single: check base/rover overlap.\n");
            }
        }
    }
}
