/*------------------------------------------------------------------------------
* rtk_engine.c : byte-in / fix-out RTK engine (see rtk_engine.h)
*-----------------------------------------------------------------------------*/
#include "rtk_engine.h"
#include "rtk_platform.h"

/* --- configuration -------------------------------------------------------- */

void rtkeng_default_cfg(rtkeng_cfg_t *cfg)
{
    memset(cfg,0,sizeof(*cfg));
    cfg->mode        =PMODE_KINEMA;   /* moving rover, fixed base            */
    cfg->base_fmt    =RTKENG_BASE_UBX;
    /* GPS+GLONASS, matching what this hardware can actually deliver: MON-VER
     * reports "GPS;GLO;BDS" on both ends, and the base already broadcasts 1087
     * and 1230. Galileo was never available on an M8P, so naming SYS_GAL here
     * only reserved 36 satellite slots that could never be filled. Requires a
     * build with RTKPICO_ENABLE_GLO and RTKPICO_ENABLE_GAL=OFF; see the note
     * in rover_cfg.c. This pairing is a hardware question, not a preference. */
    cfg->navsys      =SYS_GPS|SYS_GLO;
    /* Measured, not assumed: dropping this to 10 deg changed the common
     * satellite count by 0.2 on average (6.2 vs 6.0), so the mask was never
     * what limited it -- the rover simply sees 8-10 GPS satellites and shares
     * 6-7 with the base. Keep 15 deg, because the low-elevation signals it
     * rejects are the weakest ones, and weak signals are exactly what loses
     * half-cycle lock and resets the ambiguities. */
    cfg->elmin_deg   =15.0;
    cfg->ar_mode     =ARMODE_FIXHOLD;
    cfg->ar_thres    =3.0;
    /* Off by default. It costs a full nx^3 covariance propagation per epoch
     * (roughly 2.5x the total), and the position states are already tightly
     * constrained by the carrier phase. Turn it on only if you need the
     * velocity states, and set accel_h/accel_v to match the airframe. */
    cfg->dynamics    =0;
    cfg->accel_h     =3.0;            /* m/s^2, sized for a multirotor       */
    cfg->accel_v     =1.0;
    cfg->max_age     =30.0;
    cfg->min_lock2fix=0;
    return;
}

/* Translate the engine config into an RTKLIB prcopt_t. Everything not set
 * here keeps prcopt_default, which is tuned for exactly this L1 RTK case. */
static void build_prcopt(const rtkeng_cfg_t *cfg, prcopt_t *opt)
{
    int i;

    *opt=prcopt_default;
    opt->mode     =cfg->mode;
    opt->nf       =1;                 /* L1/E1 only                          */
    opt->navsys   =cfg->navsys;
    opt->elmin    =cfg->elmin_deg*D2R;
    opt->modear   =cfg->ar_mode;
    opt->thresar[0]=cfg->ar_thres;
    opt->dynamics =cfg->dynamics;
    opt->maxtdiff =cfg->max_age;
    opt->minlock  =cfg->min_lock2fix;
    if (cfg->dynamics&&cfg->accel_h>0.0) opt->prn[3]=cfg->accel_h;
    if (cfg->dynamics&&cfg->accel_v>0.0) opt->prn[4]=cfg->accel_v;
    opt->sateph   =EPHOPT_BRDC;       /* broadcast ephemeris only            */
    opt->ionoopt  =IONOOPT_BRDC;      /* Klobuchar; differenced away on short
                                         baselines but needed by the SPP seed */
    opt->tropopt  =TROPOPT_SAAS;
    opt->tidecorr =0;                 /* tidedisp() is stubbed out           */
    opt->refpos   =0;                 /* 0 = base position taken from opt->rb */
    /* GLONASS observations are used for the float solution but its ambiguities
     * are not resolved. GLONASS is FDMA, so each satellite sits on its own
     * frequency and the receiver's inter-channel bias does not difference away
     * the way it does for the CDMA systems; resolving it needs that bias
     * calibrated. Both ends here are the same model and firmware (NEO-M8P-2,
     * HPG 1.43REF), so the bias should be near zero and glomodear=1 is the
     * defensible next step -- but a wrong bias yields confidently wrong fixes,
     * which is a far worse failure for a drone show than simply not fixing.
     * Turn it on as its own change, with its own measurement. Most of the
     * benefit is here already: more satellites means a better float and more
     * redundancy for the GPS ambiguity resolution that does run. */
    opt->glomodear=0;
    opt->bdsmodear=0;
    opt->niter    =1;
    for (i=0;i<3;i++) opt->rb[i]=cfg->base_ecef[i];
}

/* --- navigation data ------------------------------------------------------ */

static int alloc_nav(nav_t *nav)
{
    eph_t  eph0={0,-1,-1};
    alm_t  alm0={0,-1};
    geph_t geph0={0,-1};
    int i;

    memset(nav,0,sizeof(*nav));
    if (!(nav->eph =(eph_t  *)malloc(sizeof(eph_t )*MAXSAT))||
        !(nav->alm =(alm_t  *)malloc(sizeof(alm_t )*MAXSAT))||
        !(nav->geph=(geph_t *)malloc(sizeof(geph_t)*(NSATGLO?NSATGLO:1)))) {
        free(nav->eph); free(nav->alm); free(nav->geph);
        nav->eph=NULL; nav->alm=NULL; nav->geph=NULL;
        return 0;
    }
    nav->n =nav->nmax =MAXSAT;
    nav->na=nav->namax=MAXSAT;
    nav->ng=nav->ngmax=NSATGLO;
    for (i=0;i<MAXSAT ;i++) nav->eph [i]=eph0;
    for (i=0;i<MAXSAT ;i++) nav->alm [i]=alm0;
    for (i=0;i<NSATGLO;i++) nav->geph[i]=geph0;

    /* 2.4.3 derives carrier frequency from sat2freq(sat,code,nav) rather than
     * a precomputed nav->lam table. The only per-satellite frequency data it
     * still needs from us is the GLONASS FCN table (FDMA); a zeroed entry
     * means "unknown", so seed the neutral channel. Irrelevant unless
     * RTKPICO_ENABLE_GLO is set, but wrong-by-default is worse than unused. */
    for (i=0;i<32;i++) nav->glo_fcn[i]=0;
    return 1;
}

static void free_nav(nav_t *nav)
{
    free(nav->eph ); nav->eph =NULL; nav->n =nav->nmax =0;
    free(nav->alm ); nav->alm =NULL; nav->na=nav->namax=0;
    free(nav->geph); nav->geph=NULL; nav->ng=nav->ngmax=0;
}

/* Copy one updated ephemeris plus the broadcast iono/UTC block out of a
 * decoder's private nav_t into the engine's. Mirrors rtksvr's updatenav(). */
static void merge_eph(nav_t *dst, const nav_t *src, int sat)
{
    int i;
    if (sat>=1&&sat<=MAXSAT&&src->eph&&dst->eph) {
        if (satsys(sat,NULL)==SYS_GLO) {
            /* GLONASS ephemerides live in geph[] indexed by slot */
            int prn; satsys(sat,&prn);
            if (src->geph&&dst->geph&&prn>=1&&prn<=NSATGLO) {
                dst->geph[prn-1]=src->geph[prn-1];
            }
        }
        else dst->eph[sat-1]=src->eph[sat-1];
    }
    for (i=0;i<8;i++) dst->ion_gps[i]=src->ion_gps[i];
    for (i=0;i<4;i++) dst->ion_gal[i]=src->ion_gal[i];
    /* 2.4.3 widened the UTC parameter blocks to 8 and folded the leap-second
     * count into them (utc_gps[4]=dt_LS), so nav_t.leaps no longer exists. */
    for (i=0;i<8;i++) dst->utc_gps[i]=src->utc_gps[i];
    for (i=0;i<8;i++) dst->utc_gal[i]=src->utc_gal[i];
}

/* init_raw() and init_rtcm() size their ephemeris pools at MAXSAT*2. The upper
 * half holds Galileo's second (F/NAV) set and nothing else ever indexes it
 * (rtcm3.c type 1045, ublox.c decode_enav). With Galileo compiled out satno()
 * never yields a Galileo satellite, so each decoder carries 18 KB of heap that
 * can never be written -- the difference between fitting GPS+GLONASS and the
 * "Out of memory" panic seen in the field once GLONASS joined the solution.
 * Shrinking in place leaves the tail free for rtkpos()'s per-epoch matrices. */
static void trim_eph_pool(nav_t *nav)
{
#ifndef ENAGAL
    eph_t *p=(eph_t *)realloc(nav->eph,sizeof(eph_t)*MAXSAT);
    if (p) { nav->eph=p; nav->n=nav->nmax=MAXSAT; }
#else
    (void)nav;
#endif
}

/* --- lifecycle ------------------------------------------------------------ */

int rtkeng_init(rtkeng_t *e, const rtkeng_cfg_t *cfg)
{
    prcopt_t opt;

    memset(e,0,sizeof(*e));
    e->cfg=*cfg;

    if (cfg->mode>=PMODE_PPP_KINEMA) return 0;   /* PPP is not linked in */

    build_prcopt(&e->cfg,&opt);

    if (!alloc_nav(&e->nav)) return 0;
    /* 2.4.3's init_raw() takes the stream format; ublox.c allocates its own
     * per-receiver state (raw->rcv_data) based on it. */
    if (!init_raw(&e->rov,STRFMT_UBX)) { free_nav(&e->nav); return 0; }
    trim_eph_pool(&e->rov.nav);

    if (cfg->base_fmt==RTKENG_BASE_RTCM3) {
        if (!init_rtcm(&e->rtcm)) { free_raw(&e->rov); free_nav(&e->nav); return 0; }
        trim_eph_pool(&e->rtcm.nav);
    }
    else {
        if (!init_raw(&e->bas,STRFMT_UBX)) { free_raw(&e->rov); free_nav(&e->nav); return 0; }
        trim_eph_pool(&e->bas.nav);
    }
    rtkinit(&e->rtk,&opt);
    rtkpico_perf_init();
    e->inited=1;
    return 1;
}

void rtkeng_free(rtkeng_t *e)
{
    if (!e->inited) return;
    rtkfree(&e->rtk);
    free_raw(&e->rov);
    if (e->cfg.base_fmt==RTKENG_BASE_RTCM3) free_rtcm(&e->rtcm);
    else                                    free_raw(&e->bas);
    free_nav(&e->nav);
    e->inited=0;
}

/* --- solution ------------------------------------------------------------- */

const sol_t *rtkeng_solution(const rtkeng_t *e) { return &e->rtk.sol; }

void rtkeng_solution_llh(const rtkeng_t *e, double *llh)
{
    double pos[3];
    ecef2pos(e->rtk.sol.rr,pos);
    llh[0]=pos[0]*R2D; llh[1]=pos[1]*R2D; llh[2]=pos[2];
}

/* Run the filter for one rover epoch. obs[] is rover-then-base, each side
 * already sorted by satellite (selsat() in rtkpos.c is a merge-join and
 * silently drops satellites if either side is unsorted). */
static int run_epoch(rtkeng_t *e)
{
    obs_t o;
    int i,n=0,nu,stat;
    uint32_t t0,dt,hz;

    /* rover */
    for (i=0;i<e->rov.obs.n&&n<MAXOBS;i++) {
        e->obs[n]=e->rov.obs.data[i];
        e->obs[n].rcv=1;
        n++;
    }
    o.data=e->obs; o.n=n; o.nmax=n;
    sortobs(&o);
    nu=o.n; n=nu;

    /* base: latest complete epoch, already tagged and sorted */
    for (i=0;i<e->n_base&&n<(int)(sizeof(e->obs)/sizeof(e->obs[0]));i++) {
        e->obs[n++]=e->base_obs[i];
    }
    if (nu<=0) return RTKENG_NONE;

    /* Do not run the filter before we can actually position. At cold start the
     * receiver reports observations well before the 30 s LNAV cycle delivers
     * ephemerides; with opt.dynamics set, rtkpos() would coast the state
     * forward from an uninitialised solution on every one of those epochs and
     * the filter never recovers. Requiring ephemerides first also stops us
     * emitting a garbage position to the flight controller. */
    {
        int neph=0;
        for (i=0;i<nu;i++) {
            int sat=e->obs[i].sat;
            if (sat>=1&&sat<=MAXSAT&&e->nav.eph[sat-1].sat==sat) neph++;
        }
        if (neph<4) return RTKENG_OBS;
    }

    t0=rtkpico_cycles();
    stat=rtkpos(&e->rtk,e->obs,n,&e->nav);
    dt=rtkpico_cycles()-t0;

    hz=rtkpico_cpu_hz();
    e->stats.last_us=(uint32_t)((uint64_t)dt*1000000ull/(hz?hz:1));
    if (e->stats.last_us>e->stats.max_us) e->stats.max_us=e->stats.last_us;
    e->stats.total_us+=e->stats.last_us;
    e->stats.solutions++;

    if (!stat) { e->stats.n_none++; return RTKENG_OBS; }
    switch (e->rtk.sol.stat) {
        case SOLQ_FIX:    e->stats.n_fix++;    break;
        case SOLQ_FLOAT:  e->stats.n_float++;  break;
        case SOLQ_SINGLE: e->stats.n_single++; break;
        default:          e->stats.n_none++;   break;
    }
    return RTKENG_SOLUTION;
}

/* --- rover input ---------------------------------------------------------- */

int rtkeng_rover_byte(rtkeng_t *e, unsigned char data)
{
    int ret=input_ubx(&e->rov,data);

    if (ret==1) {                       /* observation epoch complete */
        e->stats.rover_epochs++;
        /* RXM-RAWX carries the full week number, so the first epoch is enough
         * to pin the wall clock; adjgpsweek() needs it before any LNAV
         * subframe can resolve its 10-bit week correctly. */
        if (!e->time_seeded&&e->rov.time.time!=0) {
            rtkpico_set_gps_time(e->rov.time);
            e->time_seeded=1;
            /* RTCM3 MSM carries only time of week (GLONASS: time of day), and
             * adjweek() takes the week from rtcm.time, seeding it from the
             * clock on the first message and only ever stepping half a week
             * from there. Corrections that arrive before the rover has
             * tracked anything latch the seed week, and every base epoch then
             * reads as years old (2026-10-10: age 84672000 s, 140 weeks).
             * Re-anchor on real time and drop what was stamped with the
             * wrong week. */
            e->rtcm.time=e->rov.time;
            e->n_base=0;
        }
        return run_epoch(e);
    }
    else if (ret==2) {                  /* ephemeris updated */
        merge_eph(&e->nav,&e->rov.nav,e->rov.ephsat);
        e->stats.eph_updates++;
        return RTKENG_EPHEMERIS;
    }
    else if (ret==9) {                  /* almanac / iono / utc */
        merge_eph(&e->nav,&e->rov.nav,0);
    }
    return RTKENG_NONE;
}

/* --- base input ----------------------------------------------------------- */

/* Snapshot a completed base epoch. Kept separate from the rover path so a
 * dropped base packet just ages the correction instead of stalling the
 * filter; rtkpos() enforces cfg.max_age. */
static void store_base_obs(rtkeng_t *e, const obsd_t *src, int n, gtime_t time)
{
    obs_t o;
    int i,m=0;
    for (i=0;i<n&&m<MAXOBS;i++) {
        e->base_obs[m]=src[i];
        e->base_obs[m].rcv=2;
        m++;
    }
    o.data=e->base_obs; o.n=m; o.nmax=m;
    sortobs(&o);
    e->n_base=o.n;
    e->base_time=time;
    e->stats.base_epochs++;
}

int rtkeng_base_byte(rtkeng_t *e, unsigned char data)
{
    int ret;

    if (e->cfg.base_fmt==RTKENG_BASE_RTCM3) {
        ret=input_rtcm3(&e->rtcm,data);
        if (ret==1) {
            store_base_obs(e,e->rtcm.obs.data,e->rtcm.obs.n,e->rtcm.time);
            return RTKENG_OBS;
        }
        else if (ret==2) {
            merge_eph(&e->nav,&e->rtcm.nav,e->rtcm.ephsat);
            e->stats.eph_updates++;
            return RTKENG_EPHEMERIS;
        }
        else if (ret==5) {
            /* RTCM 1005/1006 antenna reference point: adopt it only when the
             * caller did not supply a surveyed position. */
            if (e->cfg.base_ecef[0]==0.0&&e->cfg.base_ecef[1]==0.0&&
                e->cfg.base_ecef[2]==0.0) {
                int i;
                for (i=0;i<3;i++) e->rtk.opt.rb[i]=e->rtcm.sta.pos[i];
            }
        }
        return RTKENG_NONE;
    }

    ret=input_ubx(&e->bas,data);
    if (ret==1) {
        store_base_obs(e,e->bas.obs.data,e->bas.obs.n,e->bas.time);
        return RTKENG_OBS;
    }
    else if (ret==2) {
        merge_eph(&e->nav,&e->bas.nav,e->bas.ephsat);
        e->stats.eph_updates++;
        return RTKENG_EPHEMERIS;
    }
    return RTKENG_NONE;
}

/* --- buffered helpers ----------------------------------------------------- */

int rtkeng_rover_buf(rtkeng_t *e, const unsigned char *buf, int n)
{
    int i,nsol=0;
    for (i=0;i<n;i++) if (rtkeng_rover_byte(e,buf[i])==RTKENG_SOLUTION) nsol++;
    return nsol;
}

int rtkeng_base_buf(rtkeng_t *e, const unsigned char *buf, int n)
{
    int i,nep=0;
    for (i=0;i<n;i++) if (rtkeng_base_byte(e,buf[i])==RTKENG_OBS) nep++;
    return nep;
}
