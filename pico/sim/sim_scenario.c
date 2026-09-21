/*------------------------------------------------------------------------------
* sim_scenario.c : base+rover RTK scenario driver (see sim_scenario.h)
*-----------------------------------------------------------------------------*/
#include "sim_scenario.h"
#include "rtk_platform.h"
#include <string.h>

void sim_scenario_default(sim_scenario_cfg_t *cfg)
{
    memset(cfg,0,sizeof(*cfg));
    /* A 1.2 km baseline: comfortably inside single-frequency RTK range. */
    cfg->base_llh[0] =35.680000; cfg->base_llh[1] =139.760000; cfg->base_llh[2] =40.0;
    cfg->rover_llh[0]=35.690000; cfg->rover_llh[1]=139.764000; cfg->rover_llh[2]=80.0;
    /* Realistic constellation sizes: 24 GPS + 18 Galileo puts 8-12 GPS and
     * 5-8 Galileo above a 15 deg mask, which is what a receiver actually
     * sees. Fewer than this and SPP cannot even solve for two system clocks. */
    cfg->ngps=24; cfg->ngal=18;
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
    sim_make_constellation(&s->con,cfg->week,cfg->toe,cfg->ngps,cfg->ngal);
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
    unsigned char buf[UBX_MAX_MSG];
    double tow=time2gpst(s->time,NULL);
    int i,id,len,prn;

    for (i=0;i<s->con.n;i++) {
        if (satsys(s->con.sat[i],&prn)!=SYS_GPS) continue;   /* LNAV only */
        for (id=1;id<=3;id++) {
            len=ubx_encode_sfrbx_gps(buf,sizeof(buf),prn,id,&s->con.eph[i],tow);
            if (len>0) {
                rtkeng_rover_buf(eng,buf,len);
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

int sim_scenario_step(sim_scenario_t *s, rtkeng_t *eng)
{
    unsigned char buf[UBX_MAX_MSG];
    obsd_t obs[SIM_MAXSAT];
    ubx_meas_aux_t aux[SIM_MAXSAT];
    int n,len,nsol=0;

    move_rover(s);

    /* A real receiver has ephemerides before it starts reporting raw
     * measurements, so deliver them ahead of the first observation epoch. */
    if (s->epoch==0) send_ephemerides(s,eng);

    /* --- base station: generate, encode, feed ---------------------------- */
    n=sim_observe(&s->con,&s->base,&s->cfg.err,s->time,s->cfg.base_llh,
                  obs,aux,SIM_MAXSAT);
    len=ubx_encode_rawx(buf,sizeof(buf),s->time,obs,n,aux);
    if (len>0) { rtkeng_base_buf(eng,buf,len); s->base_bytes+=len; }

    /* --- rover ------------------------------------------------------------ */
    n=sim_observe(&s->con,&s->rover,&s->cfg.err,s->time,s->cfg.base_llh,
                  obs,aux,SIM_MAXSAT);
    len=ubx_encode_rawx(buf,sizeof(buf),s->time,obs,n,aux);
    if (len>0) {
        nsol=rtkeng_rover_buf(eng,buf,len);
        s->rover_bytes+=len;
    }

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
