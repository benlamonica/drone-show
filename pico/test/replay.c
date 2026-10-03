/*------------------------------------------------------------------------------
* replay.c : replay an rtk_bench field capture through the host engine
*
* rtk_bench built with -DRTK_BENCH_RAWDUMP=1 writes every rover (UBX) and
* correction (RTCM3) byte to USB as "X,U,<hex>" / "X,R,<hex>" lines, in the
* order its engine consumed them. Feeding those same bytes, in that order,
* through this host build reproduces the device's solutions exactly -- which
* turns "the rover misbehaved in the field" into something that can be
* bisected at a desk.
*
*   replay CAPTURE [options]
*
* It always reports receiver-wide carrier-lock losses (pico/port/lockloss.h),
* the same count rtk_bench shows live.
*
* With no options it prints how our solutions compare with the receiver's own
* RTK result (NAV-RELPOSNED, which travels in the same UBX stream) at the same
* GPS time. The receiver's fixed solution is the reference throughout: when it
* and we disagree, one of us is wrong, and the rest of the options exist to
* find out which and why.
*
*   --ref E N U     rover was static at this base->rover ENU (m): score every
*                   solution against it, not just epochs where the receiver
*                   happened to be fixed
*   --dd E N U      per-satellite double-difference residuals of the raw
*                   observations against that known baseline. Code DDs should
*                   average ~0; phase DDs should stay flat within an unbroken
*                   arc. A satellite that fails either is feeding the filter a
*                   bias no amount of tuning will remove.
*   --csv FILE      one line per solution (tow,stat,ns,ratio,E,N,U,dE,dN,dU,carr,s0nb)
*   --split PREFIX  write PREFIX.ubx and PREFIX.rtcm3, for stock RTKLIB tools
*                   (convbin -r ubx / -r rtcm3, then rnx2rtkp)
*
* Engine experiments (defaults are exactly rtk_bench's):
*   --gps --static --cont --elmin DEG --thres RATIO --minlock N --minfix N
*   --glomodear N
*   --maxs0 X       fail ambiguity validation when LAMBDA's best-candidate
*                   distance s0 exceeds X per ambiguity (0 = stock RTKLIB)
*   --fixrej SIGMA  reject a fix whose phase post-fit residual exceeds SIGMA
*                   (rtkpos.c VALPOS_FIX_REJECT; 0 = stock RTKLIB)
*
* Capture with pico/scripts/capture.py. See "Replaying a field capture" in
* pico/README.md.
*-----------------------------------------------------------------------------*/
#include "rtk_engine.h"
#include "lockloss.h"
#include "rtklib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

/* rtkpos.c: fix-rejection threshold and count (see VALPOS_FIX_REJECT) */
extern double   rtkpos_fix_reject;
extern uint32_t rtkpos_fix_rejected;
extern double   rtkpos_last_s0nb, rtkpos_max_s0nb;
extern uint32_t rtkpos_s0_rejected;

static rtkeng_t eng;                 /* far too big for a stack frame */
static lockloss_t lockloss;           /* same rule as rtk_bench's "lock:" line */

/* --- the receiver's own RTK result, keyed by GPS time of week --------------- */
#define MAXREL 40000
static struct { uint32_t itow; double n, e, d; int carr; } rel[MAXREL];
static int nrel;

static uint32_t rd_u4(const uint8_t *p)
{
    return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24;
}

/* NAV-RELPOSNED (M8P, version 0): relPos* in cm plus 0.1 mm high-precision
 * parts; carrSoln in flags bits 3-4 (0 none, 1 float, 2 fixed). Framed here
 * independently of RTKLIB, which does not decode it. */
static void relpos_scan(uint8_t b)
{
    static uint8_t buf[1024];
    static int n, len;
    if (n == 0 && b != 0xB5) return;
    if (n == 1 && b != 0x62) { n = 0; return; }
    buf[n++] = b;
    if (n == 6) {
        len = buf[4] | buf[5] << 8;
        if (len > (int)sizeof(buf) - 8) { n = 0; return; }
    }
    if (n >= 6 && n == len + 8) {
        uint8_t a = 0, c = 0;
        int i;
        for (i = 2; i < n - 2; i++) { a += buf[i]; c += a; }
        if (a == buf[n-2] && c == buf[n-1] && buf[2] == 0x01 && buf[3] == 0x3C &&
            len >= 40 && nrel < MAXREL) {
            const uint8_t *p = buf + 6;
            rel[nrel].itow = rd_u4(p + 4);
            rel[nrel].n = (int32_t)rd_u4(p +  8) * 0.01 + (int8_t)p[20] * 0.0001;
            rel[nrel].e = (int32_t)rd_u4(p + 12) * 0.01 + (int8_t)p[21] * 0.0001;
            rel[nrel].d = (int32_t)rd_u4(p + 16) * 0.01 + (int8_t)p[22] * 0.0001;
            rel[nrel].carr = (rd_u4(p + 36) >> 3) & 3;
            nrel++;
        }
        n = 0;
    }
}

static int rel_find(uint32_t itow)
{
    int lo = 0, hi = nrel - 1;
    while (lo <= hi) {
        int m = (lo + hi) / 2;
        if (rel[m].itow == itow) return m;
        if (rel[m].itow < itow) lo = m + 1; else hi = m - 1;
    }
    return -1;
}

/* --- our solutions ------------------------------------------------------------ */
#define MAXSOL 40000
static struct { double tow, e, n, u, ratio, s0nb; int stat, ns; } sol[MAXSOL];
static int nsol;

static void record_solution(void)
{
    const sol_t *s = &eng.rtk.sol;
    const double *rb = eng.rtk.opt.rb;
    double llh[3], d[3], enu[3];
    int k, week;

    if (nsol >= MAXSOL || (rb[0] == 0.0 && rb[1] == 0.0 && rb[2] == 0.0)) return;
    ecef2pos(rb, llh);                 /* same ENU frame as rtk_bench's D lines */
    for (k = 0; k < 3; k++) d[k] = s->rr[k] - rb[k];
    ecef2enu(llh, d, enu);
    sol[nsol].tow = time2gpst(s->time, &week);
    sol[nsol].e = enu[0]; sol[nsol].n = enu[1]; sol[nsol].u = enu[2];
    sol[nsol].stat = s->stat; sol[nsol].ns = s->ns; sol[nsol].ratio = s->ratio;
    sol[nsol].s0nb = rtkpos_last_s0nb;
    nsol++;
}

/* --- per-satellite double differences against a known baseline --------------- */
typedef struct {
    int n;                      /* code DDs accumulated                        */
    double c_sum, c_sq;
    int arcs;                   /* unbroken phase arcs                          */
    double p0;                  /* phase DD at the start of the current arc     */
    double p_dev;               /* largest |phase DD - arc start| over all arcs */
    double p_span;              /* longest arc (s)                              */
    double arc_t0;
    int in_arc;
} ddacc_t;
static ddacc_t dd[MAXSAT];
static int dd_ref[2];           /* reference satellite: [0] GPS, [1] GLONASS    */
static double dd_rr[3];         /* rover truth (ECEF)                           */
static int dd_nep;
static int dd_trace;            /* --ddtrace: print this satellite's DDs         */

/* Corrections reach the engine 0.5-1 s after the rover epoch they belong to,
 * so recent rover epochs are kept here until their base epoch turns up. Slip
 * flags are ORed across the 5 Hz epochs in between: a slip at an unpaired
 * epoch still breaks the phase arc, or it would read as drift. */
#define RING 16
static struct { obsd_t obs[MAXOBS]; int n; unsigned seq; } ring[RING];
static unsigned ring_seq, paired_seq;

/* Did the rover flag a slip on this satellite in any epoch after the last
 * paired one, up to and including epoch `seq`? */
static int rover_slipped(int sat, unsigned seq)
{
    int k, i;
    for (k = 0; k < RING; k++) {
        if (ring[k].seq <= paired_seq || ring[k].seq > seq) continue;
        for (i = 0; i < ring[k].n; i++) {
            if (ring[k].obs[i].sat == sat && (ring[k].obs[i].LLI[0] & (LLI_SLIP | LLI_HALFC))) return 1;
        }
    }
    return 0;
}

static void dd_rover_epoch(void)
{
    int n = 0;
    obsd_t *o;
    while (n < MAXOBS && eng.obs[n].rcv == 1 && n < eng.rov.obs.n) n++;
    if (!n) return;
    o = ring[ring_seq % RING].obs;
    memcpy(o, eng.obs, n * sizeof(obsd_t));
    ring[ring_seq % RING].n = n;
    ring_seq++;
    ring[(ring_seq - 1) % RING].seq = ring_seq;
}

static void dd_base_epoch(void)
{
    const obsd_t *ro = NULL, *bo = eng.base_obs;
    obsd_t all[MAXOBS * 2];
    double rs[6 * MAXOBS * 2], dts[2 * MAXOBS * 2], var[MAXOBS * 2];
    double yc[MAXOBS], yp[MAXOBS], el[MAXOBS], pos[3], e[3], azel[2];
    int svh[MAXOBS * 2], sat[MAXOBS], slip[MAXOBS];
    int nr = 0, nb = eng.n_base, i, j, k, ns = 0;
    unsigned seq = 0;
    const double *rb = eng.rtk.opt.rb;

    if (!nb || rb[0] == 0.0) return;
    /* The M8 does not steer its clock, so rover epochs sit a few ms off the
     * base's whole seconds. That is harmless -- satposs() works from each
     * receiver's own transmit time -- so pair the rover epoch within 50 ms. */
    for (k = 0; k < RING; k++) {
        if (!ring[k].n || fabs(timediff(ring[k].obs[0].time, bo[0].time)) > 0.05) continue;
        ro = ring[k].obs; nr = ring[k].n; seq = ring[k].seq;
    }
    if (!ro || seq <= paired_seq) return;

    memcpy(all, ro, nr * sizeof(obsd_t));
    memcpy(all + nr, bo, nb * sizeof(obsd_t));
    satposs(ro[0].time, all, nr + nb, &eng.nav, EPHOPT_BRDC, rs, dts, var, svh);
    ecef2pos(dd_rr, pos);

    for (i = 0; i < nr; i++) for (j = 0; j < nb; j++) {
        double f, rr_, rb_;
        const double *sr = rs + i * 6, *sb = rs + (nr + j) * 6;
        int s = ro[i].sat;
        if (s != bo[j].sat) continue;
        if (norm(sr, 3) <= 0.0 || norm(sb, 3) <= 0.0) break;
        if (ro[i].P[0] == 0.0 || bo[j].P[0] == 0.0) break;
        if ((f = sat2freq(s, ro[i].code[0], &eng.nav)) <= 0.0) break;
        rr_ = geodist(sr, dd_rr, e) - CLIGHT * dts[i * 2];
        satazel(pos, e, azel);
        rb_ = geodist(sb, rb, e) - CLIGHT * dts[(nr + j) * 2];
        sat[ns] = s;
        el[ns] = azel[1];
        yc[ns] = (ro[i].P[0] - rr_) - (bo[j].P[0] - rb_);
        yp[ns] = ro[i].L[0] != 0.0 && bo[j].L[0] != 0.0 ?
                 (ro[i].L[0] - bo[j].L[0]) * CLIGHT / f - (rr_ - rb_) : 0.0;
        slip[ns] = (bo[j].LLI[0] & (LLI_SLIP | LLI_HALFC)) || rover_slipped(s, seq);
        ns++;
        break;
    }
    paired_seq = seq;
    for (k = 0; k < 2; k++) {
        int r = -1, sysk = k ? SYS_GLO : SYS_GPS;
        for (i = 0; i < ns; i++) if (sat[i] == dd_ref[k]) r = i;
        if (r < 0) {                  /* (re)choose the highest satellite */
            for (i = 0; i < ns; i++) {
                if (satsys(sat[i], NULL) != sysk) continue;
                if (r < 0 || el[i] > el[r]) r = i;
            }
            if (r < 0) continue;
            dd_ref[k] = sat[r];
            for (i = 0; i < MAXSAT; i++) {
                if (satsys(i + 1, NULL) == sysk) dd[i].in_arc = 0;
            }
        }
        for (i = 0; i < ns; i++) {
            ddacc_t *a = dd + sat[i] - 1;
            double c, p, t;
            if (i == r || satsys(sat[i], NULL) != sysk) continue;
            c = yc[i] - yc[r];
            a->n++; a->c_sum += c; a->c_sq += c * c;
            if (yp[i] == 0.0 || yp[r] == 0.0 || slip[i] || slip[r]) { a->in_arc = 0; continue; }
            p = yp[i] - yp[r];
            t = timediff(ro[0].time, gpst2time(0, 0.0));
            if (sat[i] == dd_trace) {
                char id[8];
                int wk;
                satno2id(sat[r], id);
                printf("  trace %.1f ref %s code %+8.3f phase %+12.3f m (%+.3f cyc) %s\n",
                       time2gpst(ro[0].time, &wk), id, c, p, p / (CLIGHT / sat2freq(sat[i], CODE_L1C, &eng.nav)),
                       a->in_arc ? "" : "arc start");
            }
            if (!a->in_arc) { a->in_arc = 1; a->p0 = p; a->arc_t0 = t; a->arcs++; continue; }
            if (fabs(p - a->p0) > a->p_dev) a->p_dev = fabs(p - a->p0);
            if (t - a->arc_t0 > a->p_span) a->p_span = t - a->arc_t0;
        }
    }
    dd_nep++;
}

static void dd_report(void)
{
    int i;
    char id[8];
    printf("double differences against the known baseline: %d epochs\n", dd_nep);
    printf("  sat   n    code DD mean     sd  | phase arcs  max dev (m)  longest arc (s)\n");
    for (i = 0; i < MAXSAT; i++) {
        const ddacc_t *a = dd + i;
        double m, sd;
        if (a->n < 10) continue;
        satno2id(i + 1, id);
        m = a->c_sum / a->n;
        sd = sqrt(fabs(a->c_sq / a->n - m * m));
        printf("  %-4s %4d   %+8.3f   %6.3f  |   %4d      %7.3f      %6.0f\n",
               id, a->n, m, sd, a->arcs, a->p_dev, a->p_span);
    }
    for (i = 0; i < 2; i++) {
        if (!dd_ref[i]) continue;
        satno2id(dd_ref[i], id);
        printf("  reference %s: %s (last chosen)\n", i ? "GLONASS" : "GPS", id);
    }
}

/* --- main ------------------------------------------------------------------- */
static int hexval(int c) { return c <= '9' ? c - '0' : (c & ~32) - 'A' + 10; }

static void usage(void)
{
    fprintf(stderr, "usage: replay CAPTURE [--ref E N U] [--dd E N U] [--csv FILE] "
            "[--split PREFIX]\n              [--gps] [--static] [--cont] [--elmin DEG] "
            "[--thres R] [--minlock N] [--minfix N] [--glomodear N] [--fixrej SIGMA] [--maxs0 X]\n");
}

int main(int argc, char **argv)
{
    rtkeng_cfg_t cfg;
    FILE *fp, *csv = NULL, *fu = NULL, *fr = NULL;
    static char line[8192];
    double elmin = -1, thres = -1, ref[3] = {0}, ddb[3] = {0};
    int i, gps_only = 0, cont = 0, stat_mode = 0, glomodear = -1, minlock = -1, minfix = -1;
    int have_ref = 0, have_dd = 0;
    const char *csvpath = NULL, *split = NULL;

    if (argc < 2) { usage(); return 1; }
    for (i = 2; i < argc; i++) {
        if      (!strcmp(argv[i], "--gps"))    gps_only = 1;
        else if (!strcmp(argv[i], "--cont"))   cont = 1;
        else if (!strcmp(argv[i], "--static")) stat_mode = 1;
        else if (!strcmp(argv[i], "--elmin")     && i + 1 < argc) elmin = atof(argv[++i]);
        else if (!strcmp(argv[i], "--thres")     && i + 1 < argc) thres = atof(argv[++i]);
        else if (!strcmp(argv[i], "--minlock")   && i + 1 < argc) minlock = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--minfix")    && i + 1 < argc) minfix = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--glomodear") && i + 1 < argc) glomodear = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--fixrej")    && i + 1 < argc) rtkpos_fix_reject = atof(argv[++i]);
        else if (!strcmp(argv[i], "--maxs0")     && i + 1 < argc) rtkpos_max_s0nb = atof(argv[++i]);
        else if (!strcmp(argv[i], "--csv")       && i + 1 < argc) csvpath = argv[++i];
        else if (!strcmp(argv[i], "--split")     && i + 1 < argc) split = argv[++i];
        else if (!strcmp(argv[i], "--ref") && i + 3 < argc) {
            ref[0] = atof(argv[++i]); ref[1] = atof(argv[++i]); ref[2] = atof(argv[++i]);
            have_ref = 1;
        }
        else if (!strcmp(argv[i], "--ddtrace") && i + 1 < argc) dd_trace = satid2no(argv[++i]);
        else if (!strcmp(argv[i], "--dd") && i + 3 < argc) {
            ddb[0] = atof(argv[++i]); ddb[1] = atof(argv[++i]); ddb[2] = atof(argv[++i]);
            have_dd = 1;
        }
        else { usage(); return 1; }
    }

    /* Exactly rtk_bench.c main(); anything else and the replay would not
     * reproduce the device. */
    rtkeng_default_cfg(&cfg);
    cfg.mode     = PMODE_KINEMA;
    cfg.base_fmt = RTKENG_BASE_RTCM3;
    cfg.dynamics = 0;
    memset(cfg.base_ecef, 0, sizeof(cfg.base_ecef));   /* adopt from RTCM 1005 */
    if (gps_only)   cfg.navsys = SYS_GPS;
    if (cont)       cfg.ar_mode = ARMODE_CONT;
    if (stat_mode)  cfg.mode = PMODE_STATIC;
    if (elmin >= 0) cfg.elmin_deg = elmin;
    if (!rtkeng_init(&eng, &cfg)) { fprintf(stderr, "rtkeng_init failed\n"); return 1; }
    lockloss_init(&lockloss);
    if (glomodear >= 0) eng.rtk.opt.glomodear = glomodear;
    if (thres >= 0)     eng.rtk.opt.thresar[0] = thres;
    if (minlock >= 0)   eng.rtk.opt.minlock = minlock;
    if (minfix >= 0)    eng.rtk.opt.minfix = minfix;

    if (!(fp = fopen(argv[1], "r"))) { perror(argv[1]); return 1; }
    if (csvpath && !(csv = fopen(csvpath, "w"))) { perror(csvpath); return 1; }
    if (split) {
        char p[1024];
        snprintf(p, sizeof p, "%s.ubx", split);
        if (!(fu = fopen(p, "wb"))) { perror(p); return 1; }
        snprintf(p, sizeof p, "%s.rtcm3", split);
        if (!(fr = fopen(p, "wb"))) { perror(p); return 1; }
    }

    while (fgets(line, sizeof line, fp)) {
        const char *h;
        int rover;
        if (strncmp(line, "X,U,", 4) && strncmp(line, "X,R,", 4)) continue;
        rover = line[2] == 'U';
        for (h = line + 4; isxdigit((unsigned char)h[0]) && isxdigit((unsigned char)h[1]); h += 2) {
            uint8_t b = (uint8_t)(hexval(h[0]) << 4 | hexval(h[1]));
            int r;
            if (!rover) {
                if (fr) fputc(b, fr);
                r = rtkeng_base_byte(&eng, b);
                if (r == RTKENG_OBS && have_dd && eng.rtk.opt.rb[0] != 0.0) {
                    double pos[3], d[3];
                    int k;
                    ecef2pos(eng.rtk.opt.rb, pos);
                    enu2ecef(pos, ddb, d);
                    for (k = 0; k < 3; k++) dd_rr[k] = eng.rtk.opt.rb[k] + d[k];
                    dd_base_epoch();
                }
                continue;
            }
            if (fu) fputc(b, fu);
            relpos_scan(b);
            r = rtkeng_rover_byte(&eng, b);
            if (r == RTKENG_SOLUTION) record_solution();
            if (r == RTKENG_SOLUTION || r == RTKENG_OBS) {
                lockloss_epoch(&lockloss, eng.rov.obs.data, eng.rov.obs.n);
            }
            if ((r == RTKENG_SOLUTION || r == RTKENG_OBS) && have_dd) dd_rover_epoch();
        }
    }
    fclose(fp);
    if (fu) fclose(fu);
    if (fr) fclose(fr);

    /* --- against the receiver's own fix, epoch by epoch ----------------------- */
    {
        int nfix = 0, nflt = 0, nnone = 0, npair = 0, nufix = 0, cf = 0, cl = 0, wrong = 0;
        double sf[3] = {0}, sl[3] = {0}, hf2 = 0, hl2 = 0, hmax = 0;
        for (i = 0; i < nrel; i++) if (rel[i].carr == 2) nufix++;
        for (i = 0; i < nsol; i++) {
            int j = rel_find((uint32_t)floor(sol[i].tow * 1000.0 + 0.5));
            double de, dn, du, h;
            if (sol[i].stat == SOLQ_FIX) nfix++;
            else if (sol[i].stat == SOLQ_FLOAT) nflt++;
            else nnone++;
            if (j < 0 || rel[j].carr != 2) {
                if (csv) fprintf(csv, "%.1f,%d,%d,%.1f,%.4f,%.4f,%.4f,,,,%d,%.3f\n", sol[i].tow,
                                 sol[i].stat, sol[i].ns, sol[i].ratio, sol[i].e, sol[i].n,
                                 sol[i].u, j < 0 ? -1 : rel[j].carr, sol[i].s0nb);
                continue;
            }
            npair++;
            de = sol[i].e - rel[j].e; dn = sol[i].n - rel[j].n; du = sol[i].u + rel[j].d;
            h = sqrt(de * de + dn * dn);
            if (csv) fprintf(csv, "%.1f,%d,%d,%.1f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,2,%.3f\n", sol[i].tow,
                             sol[i].stat, sol[i].ns, sol[i].ratio, sol[i].e, sol[i].n, sol[i].u,
                             de, dn, du, sol[i].s0nb);
            if (sol[i].stat == SOLQ_FIX) {
                sf[0] += de; sf[1] += dn; sf[2] += du; cf++;
                hf2 += h * h; if (h > hmax) hmax = h;
                if (h > 0.10 || fabs(du) > 0.20) wrong++;
            } else if (sol[i].stat == SOLQ_FLOAT) {
                sl[0] += de; sl[1] += dn; sl[2] += du; cl++;
                hl2 += h * h;
            }
        }
        printf("solutions %d: FIX %d (%.1f%%) FLOAT %d NONE %d\n", nsol, nfix,
               nsol ? 100.0 * nfix / nsol : 0.0, nflt, nnone);
        if (rtkpos_max_s0nb > 0.0)
            printf("fixes rejected on s0/nb > %.2f: %lu\n",
                   rtkpos_max_s0nb, (unsigned long)rtkpos_s0_rejected);
        if (rtkpos_fix_reject > 0.0)
            printf("fixes rejected on phase residual > %.1f sigma: %lu\n",
                   rtkpos_fix_reject, (unsigned long)rtkpos_fix_rejected);
        printf("receiver-wide carrier-lock losses: %u in %.1f min of rover epochs%s\n",
               lockloss.events, lockloss_minutes(&lockloss),
               lockloss.events ? "" : " (none)");
        printf("receiver NAV-RELPOSNED %d, fixed %d; our solutions at receiver-fixed epochs %d\n",
               nrel, nufix, npair);
        if (cf) printf("  our FIX   vs receiver FIX: mean dE %+.3f dN %+.3f dU %+.3f  "
                       "horiz RMS %.3f max %.3f  wrong (>10 cm h or >20 cm v) %d of %d\n",
                       sf[0] / cf, sf[1] / cf, sf[2] / cf, sqrt(hf2 / cf), hmax, wrong, cf);
        if (cl) printf("  our FLOAT vs receiver FIX: mean dE %+.3f dN %+.3f dU %+.3f  horiz RMS %.3f\n",
                       sl[0] / cl, sl[1] / cl, sl[2] / cl, sqrt(hl2 / cl));
    }
    /* --- against a known static position, every epoch ------------------------- */
    if (have_ref) {
        int cf = 0, wrong = 0, cl = 0, first = -1, run = 0, longest = 0;
        double hf2 = 0, hmax = 0, hl2 = 0;
        for (i = 0; i < nsol; i++) {
            double de = sol[i].e - ref[0], dn = sol[i].n - ref[1], du = sol[i].u - ref[2];
            double h = sqrt(de * de + dn * dn);
            if (sol[i].stat == SOLQ_FIX) {
                cf++; hf2 += h * h; if (h > hmax) hmax = h;
                if (h > 0.10 || fabs(du) > 0.20) wrong++;
                if (first < 0) first = i;
                if (++run > longest) longest = run;
            } else {
                run = 0;
                if (sol[i].stat == SOLQ_FLOAT) { cl++; hl2 += h * h; }
            }
        }
        printf("against --ref: FIX %d, wrong %d, horiz RMS %.3f max %.3f; first fix after %.0f s, "
               "longest run %d epochs; FLOAT horiz RMS %.3f\n",
               cf, wrong, cf ? sqrt(hf2 / cf) : 0.0, hmax,
               first < 0 ? -1.0 : sol[first].tow - sol[0].tow, longest, cl ? sqrt(hl2 / cl) : 0.0);
    }
    if (have_dd) dd_report();
    printf("base (RTCM 1005) ECEF %.4f %.4f %.4f\n",
           eng.rtk.opt.rb[0], eng.rtk.opt.rb[1], eng.rtk.opt.rb[2]);
    if (csv) fclose(csv);
    rtkeng_free(&eng);
    return 0;
}
