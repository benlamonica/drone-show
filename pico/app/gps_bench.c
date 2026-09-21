/*------------------------------------------------------------------------------
* gps_bench.c : is the rover receiver feeding us usable raw observations?
*
* The other bench targets measure RTKLIB itself -- bench_target times the
* solver, perf_probe checks the DCP double routines, test_target runs the unit
* suite. None of them talks to a receiver, so when a rover goes quiet there is
* nothing that separates "the receiver is misconfigured" from "the link is
* wrong" from "the engine rejected the data". This does that, on one screen.
*
* It drives the receiver through rover_cfg_apply() -- the identical sequence
* the firmware uses -- then reports once a second over USB CDC:
*
*   link      bytes/s on UART0, so a dead or wrong-baud link is obvious
*   ubx       per-message rates, framed and checksummed here independently of
*             RTKLIB. RAWX and SFRBX are the two that matter: without RAWX
*             there is nothing to position with, and without SFRBX no
*             ephemerides, so the solution never leaves SOLQ_NONE.
*   epochs    what the engine actually accepted, which is not the same as what
*             arrived -- a checksum-clean RAWX can still be rejected.
*   sats      count and C/N0 spread of the last epoch. A receiver with an
*             unpowered or badly sited antenna still emits RAWX, just with
*             nothing in it, and the C/N0 spread is what shows that.
*   solution  single-point fix and rtkpos() timing. With no corrections this
*             stays SINGLE, which is the correct result for a bench with no
*             base -- FLOAT or FIX here would mean corrections are reaching it.
*
* Connect with:  picocom /dev/tty.usbmodem* -b 115200
*-----------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "rtk_engine.h"
#include "rover_cfg.h"
#include "rtklib.h"

/* Same wiring as the firmware; see app/main.c. */
#define ROVER_UART       uart0
#define ROVER_TX_PIN     0
#define ROVER_RX_PIN     1
#define ROVER_BAUD       230400
#define SOLUTION_RATE_HZ 5

static rtkeng_t eng;             /* ~190 KB: never on a stack */

/* --- independent UBX framing ------------------------------------------------
 * Deliberately not reusing RTKLIB's decoder. If the engine and this agree we
 * learn nothing extra, but when they disagree -- frames arriving that RTKLIB
 * discards -- that difference is the whole diagnosis. */
typedef struct {
    uint32_t rawx, sfrbx, nav, ack, mon, other, bad_ck;
} ubx_counts_t;

static ubx_counts_t cnt;

/* A window on the raw stream. When the framer classifies nothing, the counters
 * all read zero and say only "not UBX" -- which is true of NMEA text, of
 * garbage from a baud mismatch, and of a floating line. Sixty-four bytes of the
 * actual traffic tells those three apart at a glance. */
static uint8_t  raw_sample[64];
static uint16_t raw_len;

static void ubx_tally(uint8_t b)
{
    static enum { SYNC1, SYNC2, CLS, ID, LEN1, LEN2, BODY, CKA, CKB } st = SYNC1;
    static uint8_t  cls, id, cka, ckb, rx_cka;
    static uint16_t len, got;

    switch (st) {
    case SYNC1: if (b == 0xB5) st = SYNC2; break;
    case SYNC2: st = (b == 0x62) ? CLS : SYNC1; break;
    case CLS:   cls = b; cka = b; ckb = b; st = ID; break;
    case ID:    id  = b; cka += b; ckb += cka; st = LEN1; break;
    case LEN1:  len = b; cka += b; ckb += cka; st = LEN2; break;
    case LEN2:  len |= (uint16_t)b << 8; cka += b; ckb += cka; got = 0;
                st = len ? BODY : CKA; break;
    case BODY:  cka += b; ckb += cka; if (++got >= len) st = CKA; break;
    case CKA:   rx_cka = b; st = CKB; break;
    case CKB:
        if (rx_cka == cka && b == ckb) {
            if      (cls == 0x02 && id == 0x15) cnt.rawx++;
            else if (cls == 0x02 && id == 0x13) cnt.sfrbx++;
            else if (cls == 0x01)               cnt.nav++;
            else if (cls == 0x05)               cnt.ack++;
            else if (cls == 0x0A)               cnt.mon++;
            else                                cnt.other++;
        } else {
            cnt.bad_ck++;
        }
        st = SYNC1;
        break;
    }
}

/* --- last-epoch signal quality ---------------------------------------------- */
static int    last_nsat;       /* measurements with a usable C/N0            */
static int    last_nobs;       /* measurements RTKLIB kept in the epoch       */
static double cn0_min, cn0_med, cn0_max;

static int cmp_dbl(const void *a, const void *b)
{
    double d = *(const double *)a - *(const double *)b;
    return (d > 0) - (d < 0);
}

/* True when every byte captured this interval is the same value -- the
 * signature of a stuck line rather than a stream we simply cannot parse. */
static int stuck_value(uint8_t *out)
{
    uint16_t i;
    if (!raw_len) return 0;
    for (i = 1; i < raw_len; i++) if (raw_sample[i] != raw_sample[0]) return 0;
    *out = raw_sample[0];
    return 1;
}

static void snapshot_epoch(const obs_t *o)
{
    double v[MAXOBS];
    int i, n = 0;
    last_nobs = o->n;          /* before any C/N0 filtering of ours */
    for (i = 0; i < o->n && n < MAXOBS; i++) {
        /* SNR is uint16_t in units of SNR_UNIT (0.001) dBHz. Zero means the
         * receiver reported no measurement on that signal, not 0 dBHz. */
        if (o->data[i].SNR[0]) v[n++] = o->data[i].SNR[0] * SNR_UNIT;
    }
    last_nsat = n;
    if (!n) { cn0_min = cn0_med = cn0_max = 0.0; return; }
    qsort(v, (size_t)n, sizeof v[0], cmp_dbl);
    cn0_min = v[0];
    cn0_max = v[n - 1];
    cn0_med = v[n / 2];
}

static const char *solq_name(int stat)
{
    switch (stat) {
    case SOLQ_FIX:   return "FIX";
    case SOLQ_FLOAT: return "FLOAT";
    case SOLQ_SBAS:  return "SBAS";
    case SOLQ_DGPS:  return "DGPS";
    case SOLQ_SINGLE:return "SINGLE";
    case SOLQ_PPP:   return "PPP";
    default:         return "NONE";
    }
}

int main(void)
{
    rtkeng_cfg_t cfg;
    uint32_t bytes = 0;
    ubx_counts_t prev = {0};
    absolute_time_t next;

    stdio_init_all();
    sleep_ms(2500);              /* let the USB CDC host attach before printing */

    /* Anything printed before the host opens the port is discarded by
     * stdio_usb, so treat the banner as a bonus: the once-a-second table
     * below is what a late-attaching terminal will actually see. */
    printf("\n=== u-blox rover bench (RP2350) ===\n");
    printf("UART0 GP%d/GP%d @ %u baud, target rate %u Hz\n\n",
           ROVER_TX_PIN, ROVER_RX_PIN, (unsigned)ROVER_BAUD, SOLUTION_RATE_HZ);

    uart_init(ROVER_UART, ROVER_BAUD);
    gpio_set_function(ROVER_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(ROVER_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(ROVER_UART, false, false);
    uart_set_fifo_enabled(ROVER_UART, true);

    printf("configuring receiver (baud hunt + CFG-GNSS restart, ~3 s)...\n");
    rover_cfg_apply(ROVER_UART, ROVER_BAUD, SOLUTION_RATE_HZ);
    printf("done.\n\n");

    /* Single-point only: no base, no corrections. The point of the bench is the
     * receiver and the decoder, not the RTK filter. */
    rtkeng_default_cfg(&cfg);
    cfg.mode = PMODE_SINGLE;
    /* Returns 1 on success, 0 on failure -- same convention as every other
     * caller in the tree. Keep printing rather than spinning silently: stdio
     * over USB discards anything written before a host attaches, so a one-shot
     * message on a fatal path is invisible in practice. */
    if (!rtkeng_init(&eng, &cfg)) {
        for (;;) {
            printf("rtkeng_init failed -- out of memory?\n");
            sleep_ms(1000);
        }
    }

    printf("%-8s %-42s %-22s %s\n", "link", "ubx msgs/s", "epochs", "signal / solution");
    next = make_timeout_time_ms(1000);

    for (;;) {
        while (uart_is_readable(ROVER_UART)) {
            uint8_t b = (uint8_t)uart_getc(ROVER_UART);
            bytes++;
            if (raw_len < sizeof raw_sample) raw_sample[raw_len++] = b;
            ubx_tally(b);
            /* 1 = observation epoch, 2 = ephemeris; anything else is a partial
             * or uninteresting message. */
            if (rtkeng_rover_byte(&eng, b) == 1) {
                snapshot_epoch(&eng.rov.obs);
            }
        }

        if (absolute_time_diff_us(get_absolute_time(), next) > 0) continue;
        next = delayed_by_ms(next, 1000);

        {
            const sol_t *sol = rtkeng_solution(&eng);
            const rtkeng_stats_t *st = &eng.stats;
            double llh[3] = {0, 0, 0};
            uint8_t stuck = 0;
            ubx_counts_t d = {
                cnt.rawx - prev.rawx,   cnt.sfrbx - prev.sfrbx,
                cnt.nav  - prev.nav,    cnt.ack   - prev.ack,
                cnt.mon  - prev.mon,    cnt.other - prev.other,
                cnt.bad_ck - prev.bad_ck
            };
            prev = cnt;

            rtkeng_solution_llh(&eng, llh);

            printf("%6u B/s  RAWX %-4u SFRBX %-4u NAV %-3u ACK %-2u other %-3u badck %-4u  "
                   "rov %-5u eph %-4u  ",
                   (unsigned)bytes,
                   (unsigned)d.rawx, (unsigned)d.sfrbx, (unsigned)d.nav,
                   (unsigned)d.ack, (unsigned)d.other, (unsigned)d.bad_ck,
                   (unsigned)st->rover_epochs, (unsigned)st->eph_updates);

            /* obs is what RTKLIB kept; sat is how many of those carry a C/N0.
             * obs 0 means the RAWX itself was empty or every measurement was
             * dropped as an unconfigured constellation -- a different problem
             * from obs>0 with no signal strength. */
            if (last_nsat) {
                printf("obs %2d/%2d  C/N0 %2.0f/%2.0f/%2.0f  ",
                       last_nsat, last_nobs, cn0_min, cn0_med, cn0_max);
            } else {
                printf("obs  0/%2d  C/N0  -/ -/ -   ", last_nobs);
            }

            printf("%-6s", solq_name(sol ? sol->stat : 0));
            if (sol && sol->stat && sol->stat != SOLQ_NONE) {
                /* rtkeng_solution_llh() already converts to degrees; applying
                 * R2D again here produced latitudes like 2392. */
                printf("  %.7f %.7f %7.1fm  rtkpos %luus (max %lu)",
                       llh[0], llh[1], llh[2],
                       (unsigned long)st->last_us, (unsigned long)st->max_us);
            }
            printf("\n");

            /* One line of plain-language triage, so the failure mode does not
             * have to be inferred from the counters above. */
            if (bytes == 0) {
                printf("   -> nothing on UART0. Check TX/RX are crossed, ground, and the receiver has power.\n");
            } else if (raw_len > 8 && stuck_value(&stuck)) {
                /* A stuck line is not a protocol problem and must be reported
                 * before anything that talks about configuration: all-0x00 is
                 * a line held low (unpowered receiver, missing ground, short),
                 * all-0xFF is a line held high or floating with a pull-up. */
                printf("   -> stream is solid 0x%02X: the line is stuck, not carrying data.\n", stuck);
                printf("      %s\n", stuck == 0x00
                       ? "Held low -- check the receiver has power and a common ground."
                       : "Held high/idle -- check the receiver's TX actually reaches GP1.");
            } else if (d.bad_ck && !d.rawx) {
                printf("   -> bytes arriving but checksums fail: almost certainly a baud mismatch.\n");
            } else if (!d.rawx) {
                printf("   -> link alive but no RXM-RAWX. Does this receiver's firmware support it? Check UBX-MON-VER.\n");
            } else if (d.rawx && !last_nobs) {
                printf("   -> RAWX arriving but empty: the receiver is tracking nothing.\n");
                printf("      Expected with no sky view. Outdoors, obs should climb within a minute.\n");
            } else if (!st->eph_updates) {
                printf("   -> measurements arriving but no ephemeris decoded yet.\n");
                printf("      SFRBX comes in bursts, so give it a minute of clear sky.\n");
            } else if (!st->rover_epochs) {
                printf("   -> RAWX arriving but RTKLIB accepted no epoch. Decoder/constellation mismatch.\n");
            } else if (last_nsat && cn0_med < 30.0) {
                printf("   -> tracking, but median C/N0 is low. Antenna siting or cabling.\n");
            }

            if (raw_len && !d.rawx && !d.sfrbx && !d.nav && !d.ack && !d.other) {
                uint16_t k;
                printf("   raw: ");
                for (k = 0; k < raw_len && k < 48; k++) {
                    uint8_t c = raw_sample[k];
                    putchar((c >= 32 && c < 127) ? (char)c : '.');
                }
                printf("\n        ");
                for (k = 0; k < raw_len && k < 16; k++) printf("%02X ", raw_sample[k]);
                printf("\n");
            }
            raw_len = 0;
            bytes = 0;
        }
    }
}
