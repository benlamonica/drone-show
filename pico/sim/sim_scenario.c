/*------------------------------------------------------------------------------
* sim_scenario.c : base+rover RTK scenario driver (see sim_scenario.h)
*-----------------------------------------------------------------------------*/
#include "sim_scenario.h"
#include "rtk_platform.h"
#include <string.h>
#include <stdlib.h>

/* Message and epoch buffers, shared by everything below. They are ~7 KB, and
 * this runs on the RP2350 too, where core 0 has an 8 KB stack with RTKLIB's
 * own frames still to come on top; the calls never overlap, so one set does. */
static unsigned char sim_buf[UBX_MAX_MSG];
static obsd_t sim_obs[SIM_MAXSAT];
static ubx_meas_aux_t sim_aux[SIM_MAXSAT];

void sim_scenario_default(sim_scenario_cfg_t *cfg)
{
    memset(cfg,0,sizeof(*cfg));
    /* A 1.2 km baseline: comfortably inside single-frequency RTK range. */
    cfg->base_llh[0] =35.680000; cfg->base_llh[1] =139.760000; cfg->base_llh[2] =40.0;
    cfg->rover_llh[0]=35.690000; cfg->rover_llh[1]=139.764000; cfg->rover_llh[2]=80.0;
    /* Realistic constellation sizes: 24 GPS + 18 Galileo puts 8-12 GPS and
     * 5-8 Galileo above a 15 deg mask, which is what a receiver actually
     * sees. Fewer than this and SPP cannot even solve for two system clocks.
     * A GLONASS build gets the full 24-slot constellation instead, since that
     * is the pairing the M8P flies (only one of the two is ever compiled in). */
    cfg->ngps=24; cfg->ngal=18; cfg->nglo=NSATGLO>0?24:0;
    cfg->base_fmt=SIM_BASE_UBX;
    cfg->week=2300;
    cfg->toe=302400.0;
    cfg->start_tow=302400.0;
    cfg->rate_hz=1.0;
    cfg->motion=SIM_MOTION_STATIC;
    cfg->radius=30.0;
    cfg->speed=8.0;
    cfg->seed=12345;
    sim_default_errmodel(&cfg->err);
}

void sim_scenario_init(sim_scenario_t *s, const sim_scenario_cfg_t *cfg)
{
    memset(s,0,sizeof(*s));
    s->cfg=*cfg;
    sim_make_constellation(&s->con,cfg->week,cfg->toe,cfg->ngps,cfg->ngal,cfg->nglo);
    sim_station_init(&s->base ,cfg->base_llh ,cfg->seed);
    sim_station_init(&s->rover,cfg->rover_llh,cfg->seed^0xA5A5A5A5u);
    s->time=gpst2time(cfg->week,cfg->start_tow);
    s->epoch=0;
    /* Seed the wall clock so adjgpsweek() resolves the 10-bit LNAV week even
     * for subframes that arrive before the first RXM-RAWX. */
    rtkpico_set_gps_time(s->time);
}

/* Move the rover for this epoch. Position is expressed as an ENU offset from
 * the configured start point and converted back to ECEF. */
static void move_rover(sim_scenario_t *s)
{
    double t=s->epoch/(s->cfg.rate_hz>0.0?s->cfg.rate_hz:1.0);
    double llh0[3],p0[3],enu[3]={0,0,0},venu[3],dr[3],pos0[3];

    llh0[0]=s->cfg.rover_llh[0]*D2R;
    llh0[1]=s->cfg.rover_llh[1]*D2R;
    llh0[2]=s->cfg.rover_llh[2];
    pos2ecef(llh0,p0);

    venu[0]=venu[1]=venu[2]=0.0;
    if (s->cfg.motion==SIM_MOTION_CIRCLE) {
        double w=s->cfg.radius>0.0?s->cfg.speed/s->cfg.radius:0.0;
        enu[0]=s->cfg.radius*cos(w*t);
        enu[1]=s->cfg.radius*sin(w*t);
        enu[2]=5.0*sin(0.25*w*t);
        venu[0]=-s->cfg.radius*w*sin(w*t);
        venu[1]= s->cfg.radius*w*cos(w*t);
        venu[2]= 1.25*w*cos(0.25*w*t);
    }
    else if (s->cfg.motion==SIM_MOTION_LINE) {
        enu[0]=s->cfg.speed*t;
        venu[0]=s->cfg.speed;
    }
    pos0[0]=llh0[0]; pos0[1]=llh0[1]; pos0[2]=llh0[2];
    enu2ecef(pos0,enu,dr);
    /* sim_observe() needs the velocity in ECEF, the same frame as st->pos;
     * the trajectory above is parameterised in ENU. */
    enu2ecef(pos0,venu,s->rover.vel);
    s->rover.pos[0]=p0[0]+dr[0];
    s->rover.pos[1]=p0[1]+dr[1];
    s->rover.pos[2]=p0[2]+dr[2];
    memcpy(s->true_rover_ecef,s->rover.pos,sizeof(double)*3);
}

/* Push all three ephemeris subframes for every satellite through SFRBX. This
 * is faster than the real 30 s LNAV cycle but uses the identical decode path. */
static void send_ephemerides(sim_scenario_t *s, rtkeng_t *eng)
{
    double tow=time2gpst(s->time,NULL);
    int i,id,len,prn;

    for (i=0;i<s->con.n;i++) {
        if (satsys(s->con.sat[i],&prn)!=SYS_GPS) continue;   /* LNAV only */
        for (id=1;id<=3;id++) {
            len=ubx_encode_sfrbx_gps(sim_buf,sizeof(sim_buf),prn,id,&s->con.eph[i],tow);
            if (len>0) {
                rtkeng_rover_buf(eng,sim_buf,len);
                s->rover_bytes+=len;
            }
        }
    }
#ifdef ENAGAL
    /* Galileo I/NAV page encoding is not implemented; inject E1 ephemerides
     * directly so the filter still sees a dual-constellation geometry. The
     * GPS satellites keep the byte-level path under test. */
    for (i=0;i<s->con.n;i++) {
        int sat=s->con.sat[i];
        if (satsys(sat,NULL)!=SYS_GAL) continue;
        eng->nav.eph[sat-1]=s->con.eph[i];
    }
#endif
}

/* GLONASS strings 1-4 for every GLONASS satellite, all in the current frame.
 * These must follow at least one RXM-RAWX: decode_gnav() dates the ephemeris
 * from the receiver's latest epoch, and a real receiver has always reported
 * observations by the time it has a navigation frame to pass on. */
static void send_glonass_strings(sim_scenario_t *s, rtkeng_t *eng)
{
    int i,m,len,prn;

    for (i=0;i<s->con.n;i++) {
        if (satsys(s->con.sat[i],&prn)!=SYS_GLO) continue;
        for (m=1;m<=4;m++) {
            len=ubx_encode_sfrbx_glo(sim_buf,sizeof(sim_buf),&s->con.geph[prn-1],m,s->time);
            if (len>0) {
                rtkeng_rover_buf(eng,sim_buf,len);
                s->rover_bytes+=len;
            }
        }
    }
}

/* The base as the field sees it: RTCM3 1005 (position) plus MSM7, 1077 for
 * GPS and 1087 for GLONASS, generated by RTKLIB's own encoder so the decoder
 * meets exactly the structure a u-blox base produces -- including the GLONASS
 * channel in MSM7's extended satellite info, which is where the rover learns
 * the base's wavelengths from. Allocated on first use (rtcm_t is ~95 KB), and
 * only by tests that ask for it. */
#ifdef RTKPICO_HAVE_RTCM3E
static rtcm_t *rtcm_enc;

static void send_base_rtcm3(sim_scenario_t *s, rtkeng_t *eng, const obsd_t *obs, int n)
{
    int i,prn,has_glo=0;

    if (!rtcm_enc) {
        if (!(rtcm_enc=(rtcm_t *)calloc(1,sizeof(rtcm_t)))) return;
        if (!init_rtcm(rtcm_enc)) { free(rtcm_enc); rtcm_enc=NULL; return; }
    }
    for (i=0;i<s->con.n;i++) {
        if (satsys(s->con.sat[i],&prn)!=SYS_GLO) continue;
        rtcm_enc->nav.geph[prn-1]=s->con.geph[prn-1];
    }
    if (n>MAXOBS) n=MAXOBS;
    for (i=0;i<n;i++) {
        rtcm_enc->obs.data[i]=obs[i];
        if (satsys(obs[i].sat,NULL)==SYS_GLO) has_glo=1;
    }
    rtcm_enc->obs.n=n;
    rtcm_enc->time=s->time;
    rtcm_enc->staid=1;
    memcpy(rtcm_enc->sta.pos,s->base.pos,sizeof(double)*3);

    if (gen_rtcm3(rtcm_enc,1005,0,0)) {
        rtkeng_base_buf(eng,rtcm_enc->buff,rtcm_enc->nbyte);
        s->base_bytes+=rtcm_enc->nbyte;
    }
    /* multiple-message bit: set on all but the last MSM of the epoch */
    if (gen_rtcm3(rtcm_enc,1077,0,has_glo)) {
        rtkeng_base_buf(eng,rtcm_enc->buff,rtcm_enc->nbyte);
        s->base_bytes+=rtcm_enc->nbyte;
    }
    if (has_glo&&gen_rtcm3(rtcm_enc,1087,0,0)) {
        rtkeng_base_buf(eng,rtcm_enc->buff,rtcm_enc->nbyte);
        s->base_bytes+=rtcm_enc->nbyte;
    }
}
#endif

int sim_scenario_step(sim_scenario_t *s, rtkeng_t *eng)
{
    int n,len,nsol=0;

    move_rover(s);

    /* A real receiver has ephemerides before it starts reporting raw
     * measurements, so deliver them ahead of the first observation epoch. */
    if (s->epoch==0) send_ephemerides(s,eng);

    /* --- base station: generate, encode, feed ---------------------------- */
    n=sim_observe(&s->con,&s->base,&s->cfg.err,s->time,s->cfg.base_llh,
                  sim_obs,sim_aux,SIM_MAXSAT);
#ifdef RTKPICO_HAVE_RTCM3E
    if (s->cfg.base_fmt==SIM_BASE_RTCM3) {
        send_base_rtcm3(s,eng,sim_obs,n);
    }
    else
#endif
    {
        len=ubx_encode_rawx(sim_buf,sizeof(sim_buf),s->time,sim_obs,n,sim_aux);
        if (len>0) { rtkeng_base_buf(eng,sim_buf,len); s->base_bytes+=len; }
    }

    /* --- rover ------------------------------------------------------------ */
    n=sim_observe(&s->con,&s->rover,&s->cfg.err,s->time,s->cfg.base_llh,
                  sim_obs,sim_aux,SIM_MAXSAT);
    len=ubx_encode_rawx(sim_buf,sizeof(sim_buf),s->time,sim_obs,n,sim_aux);
    if (len>0) {
        nsol=rtkeng_rover_buf(eng,sim_buf,len);
        s->rover_bytes+=len;
    }
    if (s->epoch==0) send_glonass_strings(s,eng);

    s->epoch++;
    s->time=timeadd(s->time,1.0/(s->cfg.rate_hz>0.0?s->cfg.rate_hz:1.0));
    return nsol>0;
}

void sim_scenario_error(const sim_scenario_t *s, const rtkeng_t *eng,
                        double *herr, double *verr3d)
{
    const sol_t *sol=rtkeng_solution(eng);
    double pos[3],dr[3],enu[3];
    int i;

    for (i=0;i<3;i++) dr[i]=sol->rr[i]-s->true_rover_ecef[i];
    ecef2pos(s->true_rover_ecef,pos);
    ecef2enu(pos,dr,enu);
    if (herr)   *herr=sqrt(enu[0]*enu[0]+enu[1]*enu[1]);
    if (verr3d) *verr3d=sqrt(enu[0]*enu[0]+enu[1]*enu[1]+enu[2]*enu[2]);
}
