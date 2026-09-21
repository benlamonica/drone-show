/*------------------------------------------------------------------------------
* gnss_sim.c : GNSS constellation and observable simulator (see gnss_sim.h)
*-----------------------------------------------------------------------------*/
#include "gnss_sim.h"
#include <string.h>

#define MU_GPS 3.9860050E14      /* earth gravitational constant (IS-GPS-200) */
#define SAT_VEL_TT 1E-3          /* satellite velocity differencing step,
                                    matching ephpos() in ephemeris.c        */

/* --- deterministic PRNG ---------------------------------------------------
 * xorshift32 + Box-Muller. Tests must be bit-reproducible across host and
 * target, so this cannot use rand(). */
uint32_t sim_rand_u32(uint32_t *s)
{
    uint32_t x=*s?*s:0x9E3779B9u;
    x^=x<<13; x^=x>>17; x^=x<<5;
    return *s=x;
}
static double sim_rand_unit(uint32_t *s)      /* (0,1) */
{
    return ((double)(sim_rand_u32(s)>>8)+0.5)/16777216.0;
}
double sim_rand_gauss(uint32_t *s)
{
    double u1=sim_rand_unit(s),u2=sim_rand_unit(s);
    return sqrt(-2.0*log(u1))*cos(2.0*PI*u2);
}

void sim_default_errmodel(sim_errmodel_t *e)
{
    memset(e,0,sizeof(*e));
    /* Matched to the error model RTKLIB actually assumes, so the simulator
     * does not manufacture outliers: prcopt_default has err[1]=err[2]=0.003 m
     * for phase and eratio[0]=100, i.e. code sigma ~ 0.3 m with the same
     * a + b/sin(el) shape. Simulating noise far outside that just exercises
     * the outlier rejector. */
    e->code_sigma   =0.30;
    e->phase_sigma  =0.003;  /* 3 mm                                          */
    e->iono_zenith  =4.0;
    /* Differential ionosphere is what actually limits single-frequency RTK.
     * 0.8 mm/km is a quiet mid-latitude day; 4-10 mm/km is a disturbed one.
     * At 4 mm/km a 1.2 km baseline already carries ~0.15 cycles of residual,
     * which is enough to stop the ratio test from ever passing -- so this
     * value is the difference between "fixes in 30 s" and "never fixes".    */
    e->iono_gradient=0.0008;
    e->tropo_scale  =1.0;
    e->tropo_wet_zenith  =0.10;    /* ~10 cm ZWD, a temperate day           */
    e->tropo_wet_gradient=0.0003;  /* 0.3 mm/km differential, typical short
                                      baseline                              */
    e->multipath    =0.10;   /* extra code-only error, 1/sin(el) weighted     */
    e->elev_mask_deg=10.0;   /* what the receiver reports, not the RTK mask   */
    e->sat_clk_err  =1.5;
}

/* --- constellation -------------------------------------------------------- */

/* One satellite in a Walker-delta pattern. Harmonic correction terms are given
 * small but non-zero values so the LNAV encode/decode path is exercised. */
static void make_eph(eph_t *eph, int sat, int idx, int nplane, int nper,
                     double sma, double inc_deg, int week, double toe,
                     uint32_t *rng)
{
    int plane=idx/nper, slot=idx%nper;
    double raan,m0;

    memset(eph,0,sizeof(*eph));
    /* LNAV transmits OMG0/M0 as signed 32-bit semicircles, so both must be
     * wrapped to [-pi,pi) or the encoded field overflows. */
    raan=2.0*PI*plane/nplane;
    if (raan>=PI) raan-=2.0*PI;
    /* stagger successive planes so satellites do not line up in phase */
    m0=2.0*PI*slot/nper+2.0*PI*plane/(nplane*nper);
    while (m0>=PI) m0-=2.0*PI;
    while (m0<-PI) m0+=2.0*PI;

    eph->sat  =sat;
    eph->iode =(idx*7+11)&0xFF;
    eph->iodc =eph->iode;             /* decode_subfrm3() enforces this */
    eph->sva  =0;
    eph->svh  =0;
    eph->week =week;
    eph->code =1;
    eph->flag =0;
    eph->A    =sma;
    eph->e    =0.004+0.001*(idx%3);
    eph->i0   =inc_deg*D2R;
    eph->OMG0 =raan;
    eph->omg  =0.3+0.1*(idx%5);
    eph->M0   =m0;
    eph->deln =1.2E-9*(1.0+0.1*(idx%4));
    eph->OMGd =-8.0E-9;
    eph->idot =1.0E-10;
    eph->crs  =  10.0+2.0*(idx%5);
    eph->crc  = 200.0+5.0*(idx%7);
    eph->cuc  =-1.0E-6;
    eph->cus  = 5.0E-6;
    eph->cic  = 8.0E-8;
    eph->cis  =-2.0E-7;
    eph->toes =toe;
    eph->fit  =4.0;
    eph->f0   =1.0E-4*sim_rand_gauss(rng);
    eph->f1   =1.0E-12;
    eph->f2   =0.0;
    eph->tgd[0]=-5.0E-9;
    eph->toe  =gpst2time(week,toe);
    eph->toc  =eph->toe;
    eph->ttr  =gpst2time(week,toe-3600.0);
}

void sim_make_constellation(sim_constellation_t *c, int week, double toe,
                            int ngps, int ngal)
{
    uint32_t rng=0xC0FFEEu;
    int i,n=0;

    memset(c,0,sizeof(*c));
    c->week=week; c->toe=toe;

    for (i=0;i<ngps&&n<SIM_MAXSAT;i++,n++) {
        c->sat[n]=satno(SYS_GPS,i+1);
        make_eph(&c->eph[n],c->sat[n],i,6,(ngps+5)/6,26559710.0,55.0,week,toe,&rng);
    }
#ifdef ENAGAL
    for (i=0;i<ngal&&n<SIM_MAXSAT;i++,n++) {
        c->sat[n]=satno(SYS_GAL,i+1);
        if (!c->sat[n]) break;
        make_eph(&c->eph[n],c->sat[n],i,3,(ngal+2)/3,29599800.0,56.0,week,toe,&rng);
    }
#else
    (void)ngal;
#endif
    c->n=n;
}

/* --- stations ------------------------------------------------------------- */

void sim_station_init(sim_station_t *st, const double *llh, uint32_t seed)
{
    double pos[3];
    memset(st,0,sizeof(*st));
    pos[0]=llh[0]*D2R; pos[1]=llh[1]*D2R; pos[2]=llh[2];
    pos2ecef(pos,st->pos);
    st->rng=seed?seed:1;
    st->clk_bias =1.0E-4*sim_rand_gauss(&st->rng);
    st->clk_drift=1.0E-9;
}

void sim_force_slip(sim_station_t *st, int sat)
{
    if (sat>=1&&sat<=MAXSAT) {
        st->tracked[sat-1]=0;
        st->locktime[sat-1]=0;
    }
}

/* --- observation generation ----------------------------------------------- */

/* Satellite position/clock at transmit time, solved for signal travel time.
 *
 * `dtr` is the receiver clock bias and it matters. The epoch is timestamped
 * with the receiver's own clock, so the instant of reception in true time is
 * (trx - dtr) and the signal left the satellite at (trx - dtr - tau).
 *
 * This mirrors what satposs() in ephemeris.c derives: it takes
 * t_tx = t_obs - P/CLIGHT - dts, and since P carries a +CLIGHT*dtr term that
 * reduces to t_obs - r/CLIGHT - dtr. Dropping dtr here would evaluate the
 * orbit up to 0.4 m away from where RTKLIB places it (dtr ~ 1e-4 s against a
 * 3.9 km/s satellite), and because the rover and base clocks differ that error
 * does NOT cancel in the double difference -- it shows up as a per-satellite
 * bias of up to a cycle, which is enough to stop ambiguity resolution dead. */
static int sat_state(const eph_t *eph, gtime_t trx, double dtr, const double *rr,
                     double *rs, double *dts, double *range, double *e)
{
    double tau=0.075,var,r,rst[3],dtst[2];
    gtime_t ttx={0};
    int i;

    for (i=0;i<4;i++) {
        ttx=timeadd(trx,-dtr-tau);
        eph2pos(ttx,eph,rs,dts,&var);
        r=geodist(rs,rr,e);        /* includes the Sagnac correction */
        if (r<=0.0) return 0;
        tau=r/CLIGHT;
    }

    /* eph2pos() fills only rs[0..2] and dts[0]; RTKLIB derives satellite
     * velocity and clock drift by finite differencing inside ephpos(). A
     * caller that reads rs[3..5] straight after eph2pos() is reading
     * uninitialised memory, which here produced a smooth, plausible and
     * completely wrong Doppler. Difference it the way ephpos() does. */
    eph2pos(timeadd(ttx,SAT_VEL_TT),eph,rst,dtst,&var);
    for (i=0;i<3;i++) rs[i+3]=(rst[i]-rs[i])/SAT_VEL_TT;
    dts[1]=(dtst[0]-dts[0])/SAT_VEL_TT;

    *range=r;
    return 1;
}

/* Troposphere, decomposed the way the estimator assumes it.
 *
 * zdres() in rtkpos.c removes only the hydrostatic delay, as
 * tropmapf(Niell) * tropmodel(zenith, humidity=0), unconditionally and for
 * both stations. So that component cancels in the double difference by
 * construction, and simulating it any other way (e.g. tropmodel() with a
 * secant mapping and non-zero humidity) injects a bias the filter cannot
 * remove and no ambiguity set can absorb.
 *
 * What genuinely does not cancel is the *wet* delay difference between the two
 * stations, so that is modelled separately with its own baseline gradient.
 * This is the physical limit on single-frequency RTK range, alongside iono. */
static double tropo_delay(const sim_errmodel_t *err, gtime_t time,
                          const double *pos, const double *azel,
                          const double *ref_llh)
{
    double zazel[2]={0.0,PI/2.0},zhd,mapf,wet,dist_km=0.0,dlat,dlon;

    zhd =tropmodel(time,pos,zazel,0.0);        /* zenith hydrostatic  */
    mapf=tropmapf(time,pos,azel,NULL);         /* Niell, as in zdres  */

    if (ref_llh) {
        dlat=(pos[0]-ref_llh[0]*D2R)*RE_WGS84;
        dlon=(pos[1]-ref_llh[1]*D2R)*RE_WGS84*cos(pos[0]);
        dist_km=sqrt(dlat*dlat+dlon*dlon)/1000.0;
    }
    wet=err->tropo_wet_zenith+err->tropo_wet_gradient*dist_km;

    return mapf*(zhd+wet);
}

/* Thin-shell ionosphere with an obliquity factor and a horizontal gradient
 * referenced to `ref_llh`, so the delay does not cancel exactly over a
 * baseline. This is what limits single-frequency RTK range. */
static double iono_delay(const sim_errmodel_t *err, const double *pos,
                         const double *ref_llh, const double *azel)
{
    double z,mapf,dist_km=0.0,dlat,dlon;

    z=PI/2.0-azel[1];
    mapf=1.0/sqrt(1.0-pow(RE_WGS84/(RE_WGS84+HION)*sin(z),2.0));

    if (ref_llh) {
        dlat=(pos[0]-ref_llh[0]*D2R)*RE_WGS84;
        dlon=(pos[1]-ref_llh[1]*D2R)*RE_WGS84*cos(pos[0]);
        dist_km=sqrt(dlat*dlat+dlon*dlon)/1000.0;
    }
    return mapf*(err->iono_zenith+err->iono_gradient*dist_km);
}

int sim_observe(const sim_constellation_t *c, sim_station_t *st,
                const sim_errmodel_t *err, gtime_t time, const double *ref_llh,
                obsd_t *obs, ubx_meas_aux_t *aux, int maxn)
{
    double pos[3],rs[6],dts[2],e[3],azel[2],r;
    double dtr,trop,iono,lam,sigc,sigp,mp,el_sin;
    int i,n=0,sat;

    ecef2pos(st->pos,pos);
    dtr=st->clk_bias+st->clk_drift*0.0;

    for (i=0;i<c->n&&n<maxn;i++) {
        sat=c->sat[i];
        if (!sat_state(&c->eph[i],time,dtr,st->pos,rs,dts,&r,e)) continue;
        satazel(pos,e,azel);
        if (azel[1]<err->elev_mask_deg*D2R) continue;

        el_sin=sin(azel[1]); if (el_sin<0.05) el_sin=0.05;

        trop=err->tropo_scale*tropo_delay(err,time,pos,azel,ref_llh);
        iono=iono_delay(err,pos,ref_llh,azel);

        /* Elevation-dependent noise, the standard a+b/sin(el) shape. */
        sigc=err->code_sigma /el_sin;
        sigp=err->phase_sigma/el_sin;
        mp  =err->multipath*sim_rand_gauss(&st->rng)/el_sin;

        /* 2.4.3 removed the global lam_carr[]; L1/E1 share this frequency. */
        lam=CLIGHT/FREQ1;

        /* Draw a fresh integer ambiguity on first lock or after a slip. */
        if (!st->tracked[sat-1]) {
            st->amb[sat-1]=(double)(int)(sim_rand_gauss(&st->rng)*1.0E5);
            st->tracked[sat-1]=1;
            st->locktime[sat-1]=0;
        }

        memset(&obs[n],0,sizeof(obsd_t));
        obs[n].time=time;
        obs[n].sat =(unsigned char)sat;
        obs[n].rcv =0;
        obs[n].code[0]=satsys(sat,NULL)==SYS_GAL?CODE_L1X:CODE_L1C;

        /* rho + c*(dtr - dts) + trop +/- iono  (+iono on code, -iono on phase) */
        obs[n].P[0]=r+CLIGHT*(dtr-dts[0])+trop+iono
                    +err->sat_clk_err*0.0            /* common-mode: cancels  */
                    +mp+sigc*sim_rand_gauss(&st->rng);
        obs[n].L[0]=(r+CLIGHT*(dtr-dts[0])+trop-iono
                    +sigp*sim_rand_gauss(&st->rng))/lam+st->amb[sat-1];
        /* Doppler must use satellite velocity RELATIVE TO THE RECEIVER,
         * plus the earth-rotation rate term -- exactly the model estvel() in
         * pntpos.c inverts (vs[j] = rs[j+3] - x[j]). Omitting the receiver's
         * own velocity is invisible while dynamics==0, because nothing
         * consumes the velocity states, but with dynamics==1 it seeds the
         * filter with a velocity that is wrong by the full airspeed and the
         * solution collapses. */
        {
            double vs[3],rate;
            vs[0]=rs[3]-st->vel[0];
            vs[1]=rs[4]-st->vel[1];
            vs[2]=rs[5]-st->vel[2];
            rate=vs[0]*e[0]+vs[1]*e[1]+vs[2]*e[2]
                 +OMGE/CLIGHT*(rs[4]*st->pos[0]+rs[1]*st->vel[0]
                              -rs[3]*st->pos[1]-rs[0]*st->vel[1]);
            /* estvel() models -D*lambda = rate + rx_drift - c*sat_drift, so
             * both clock-rate terms belong here or the velocity solution is
             * biased per satellite. */
            rate+=CLIGHT*st->clk_drift-CLIGHT*dts[1];
            obs[n].D[0]=(float)(-rate/lam);
        }
        /* 2.4.3 stores SNR as uint16_t in SNR_UNIT (0.001) dBHz. */
        obs[n].SNR[0]=(uint16_t)((30.0+20.0*el_sin)/SNR_UNIT+0.5);
        obs[n].LLI[0]=0;

        if (aux) {
            uint32_t lt=st->locktime[sat-1];
            aux[n].locktime_ms=(uint16_t)(lt>64000?64000:lt);
            aux[n].cno       =(uint8_t)(30.0+20.0*el_sin);
            aux[n].pr_stdev  =5;
            aux[n].cp_stdev  =1;
            aux[n].do_stdev  =1;
            aux[n].trk_stat  =0x07;      /* pr + cp valid, half-cycle known */
            aux[n].half_cyc_sub=st->half_sub[sat-1];
        }
        n++;
    }
    /* advance lock counters only for satellites actually reported */
    for (i=0;i<n;i++) {
        int s=obs[i].sat;
        st->locktime[s-1]+=1000;         /* 1 Hz epochs */
    }
    return n;
}
