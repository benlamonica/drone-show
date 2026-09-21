/*------------------------------------------------------------------------------
* bench_common.h : benchmark body shared by the host and on-target runners
*
* Stage timings are what matter for sizing the flight-controller loop:
*   decode  - UBX framing + RXM-RAWX parse (per byte, both streams)
*   rtkpos  - satellite positions, SPP, EKF update, ambiguity resolution
* rtkpos dominates and is the number to compare against the epoch period.
*-----------------------------------------------------------------------------*/
#ifndef BENCH_COMMON_H
#define BENCH_COMMON_H
#include "sim_scenario.h"
#include "rtk_platform.h"
#include <stdio.h>
#include <string.h>

typedef struct {
    const char *name;
    int  ngps, ngal;
    int  mode;          /* PMODE_KINEMA / PMODE_STATIC */
    int  ar_mode;
    int  dynamics;      /* adds 6 states AND a full nx*nx*nx propagation */
    int  epochs;
} bench_case_t;

/* Size of the EKF state vector. This is what actually drives cost:
 * resamb_LAMBDA() and udpos() form products over the FULL state dimension,
 * not over the satellites currently in view. nx = NP + MAXSAT*nf. */
static int bench_nx(int dynamics) { return (dynamics?9:3)+MAXSAT*1; }

typedef struct {
    uint32_t decode_us, rtkpos_max_us, rtkpos_mean_us;
    uint32_t epoch_max_us, epoch_mean_us;
    int      nfix, nsol, nsat_avg;
    uint32_t bytes;
} bench_out_t;

/* One scenario, measured stage by stage. Buffers are static: an rtkeng_t plus
 * a scenario is ~150 KB, far past any sane stack. */
static void bench_run(const bench_case_t *bc, bench_out_t *out)
{
    static sim_scenario_t s;
    static rtkeng_t eng;
    sim_scenario_cfg_t sc;
    rtkeng_cfg_t ec;
    uint32_t hz=rtkpico_cpu_hz(),acc_ep=0,max_ep=0;
    int i,nsat_tot=0;

    memset(out,0,sizeof(*out));
    sim_scenario_default(&sc);
    sc.ngps=bc->ngps; sc.ngal=bc->ngal;
    sc.motion=(bc->mode==PMODE_STATIC)?SIM_MOTION_STATIC:SIM_MOTION_CIRCLE;

    rtkeng_default_cfg(&ec);
    ec.mode=bc->mode;
    ec.ar_mode=bc->ar_mode;
    ec.dynamics=bc->dynamics;

    sim_scenario_init(&s,&sc);
    memcpy(ec.base_ecef,s.base.pos,sizeof(double)*3);
    if (!rtkeng_init(&eng,&ec)) { printf("rtkeng_init failed\n"); return; }

    for (i=0;i<bc->epochs;i++) {
        uint32_t ep0=rtkpico_cycles();
        int got=sim_scenario_step(&s,&eng);
        uint32_t ep=rtkpico_cycles()-ep0;
        uint32_t ep_us=(uint32_t)((uint64_t)ep*1000000ull/hz);

        acc_ep+=ep_us;
        if (ep_us>max_ep) max_ep=ep_us;

        if (got) {
            out->nsol++;
            if (eng.rtk.sol.stat==SOLQ_FIX) out->nfix++;
            nsat_tot+=eng.rtk.sol.ns;
        }
    }
    out->bytes         =s.rover_bytes+s.base_bytes;
    out->rtkpos_max_us =eng.stats.max_us;
    out->rtkpos_mean_us=eng.stats.solutions?
                        (uint32_t)(eng.stats.total_us/eng.stats.solutions):0;
    out->epoch_mean_us =bc->epochs?acc_ep/bc->epochs:0;
    out->epoch_max_us  =max_ep;
    out->nsat_avg      =out->nsol?nsat_tot/out->nsol:0;
    /* everything in the epoch that is not the filter: framing + decode + sim */
    out->decode_us     =out->epoch_mean_us>out->rtkpos_mean_us?
                        out->epoch_mean_us-out->rtkpos_mean_us:0;
    rtkeng_free(&eng);
}

static void bench_header(void)
{
    printf("\n%-26s %3s %5s %5s %9s %9s %9s\n",
           "case","nx","sats","fix%","rtkpos","rtkpos","epoch");
    printf("%-26s %3s %5s %5s %9s %9s %9s\n",
           "","","","","mean us","max us","max us");
    printf("---------------------------------------------------------------"
           "---------------\n");
}

static void bench_report(const bench_case_t *bc, const bench_out_t *o)
{
    printf("%-26s %3d %5d %5d %9u %9u %9u\n",
           bc->name,bench_nx(bc->dynamics),o->nsat_avg,
           o->nsol?100*o->nfix/o->nsol:0,
           o->rtkpos_mean_us,o->rtkpos_max_us,o->epoch_max_us);
}

/* Headroom against the target solution rate. */
static void bench_rates(uint32_t worst_us)
{
    const int hz[]={1,5,10,20};
    int i;
    printf("\nWorst-case rtkpos() = %u us. Headroom at:\n",worst_us);
    for (i=0;i<4;i++) {
        uint32_t budget=1000000u/hz[i];
        printf("  %2d Hz (%6u us budget): %5.1f%% used%s\n",hz[i],budget,
               100.0*worst_us/budget,
               worst_us<budget?"":"   <-- OVER BUDGET");
    }
}

#ifdef PICO_BUILD
#define BENCH_EPOCHS 12       /* the simulator itself is expensive on target */
#else
#define BENCH_EPOCHS 150
#endif

static const bench_case_t bench_cases[]={
  /* name                        gps gal mode          AR              dyn  */
    {"static, AR fix+hold",       24,18,PMODE_STATIC,ARMODE_FIXHOLD,0,BENCH_EPOCHS},
    {"static, AR off",            24,18,PMODE_STATIC,ARMODE_OFF,    0,BENCH_EPOCHS},
    {"kinematic, no dynamics",    24,18,PMODE_KINEMA,ARMODE_FIXHOLD,0,BENCH_EPOCHS},
    {"kinematic, dynamics=1",     24,18,PMODE_KINEMA,ARMODE_FIXHOLD,1,BENCH_EPOCHS},
    {"kinematic, AR off",         24,18,PMODE_KINEMA,ARMODE_OFF,    0,BENCH_EPOCHS},
    {"GPS only, kinematic",       24, 0,PMODE_KINEMA,ARMODE_FIXHOLD,0,BENCH_EPOCHS},
};
#define BENCH_NCASES ((int)(sizeof(bench_cases)/sizeof(bench_cases[0])))

#endif
