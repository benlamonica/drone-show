/*------------------------------------------------------------------------------
* rtk_engine.h : byte-in / fix-out RTK engine for the RP2350 port
*
* Wraps the RTKLIB real-time pieces (raw decode -> nav merge -> rtkpos) in a
* single object with no threads, no files and no dynamic reconfiguration. It is
* the embedded equivalent of rtksvr.c.
*
* Usage:
*     static rtkeng_t eng;                 // ~50 KB: never put this on a stack
*     rtkeng_cfg_t cfg; rtkeng_default_cfg(&cfg);
*     cfg.base_ecef[0..2] = surveyed base position;
*     rtkeng_init(&eng,&cfg);
*     for each byte from the rover receiver:  rtkeng_rover_byte(&eng,b)
*     for each byte from the base link:       rtkeng_base_byte (&eng,b)
* rtkeng_rover_byte() returns RTKENG_SOLUTION when eng.rtk.sol has been updated.
*-----------------------------------------------------------------------------*/
#ifndef RTK_ENGINE_H
#define RTK_ENGINE_H
#include "rtklib.h"

#ifdef __cplusplus
extern "C" {
#endif

/* return codes from rtkeng_rover_byte() / rtkeng_base_byte() */
#define RTKENG_NONE      0   /* byte consumed, nothing completed              */
#define RTKENG_SOLUTION  1   /* a new solution is available (rover only)      */
#define RTKENG_EPHEMERIS 2   /* an ephemeris was updated                      */
#define RTKENG_OBS       3   /* an observation epoch completed, no solution
                                (e.g. base epoch, or rover with no base data) */

/* base station link format */
#define RTKENG_BASE_UBX   0  /* raw UBX passthrough from a second receiver    */
#define RTKENG_BASE_RTCM3 1  /* RTCM3 (NTRIP caster, u-blox base, str2str)    */

typedef struct {
    int    mode;            /* PMODE_KINEMA (drone), PMODE_STATIC, PMODE_SINGLE */
    int    base_fmt;        /* RTKENG_BASE_UBX | RTKENG_BASE_RTCM3            */
    int    navsys;          /* SYS_GPS|SYS_GLO                                */
    double base_ecef[3];    /* surveyed base position; {0,0,0} = take from
                               RTCM 1005/1006 (RTCM3 only)                    */
    double elmin_deg;       /* elevation mask                                 */
    int    ar_mode;         /* ARMODE_OFF/CONT/INST/FIXHOLD                   */
    double ar_thres;        /* ratio test threshold (3.0 typical)             */
    int    dynamics;        /* 0=none, 1=velocity+acceleration states         */
    /* Acceleration process noise (m/s^2), used only when dynamics != 0.
     * RTKLIB's defaults are 0.1 horizontal / 0.01 vertical, which model a
     * vehicle that barely manoeuvres. A multirotor pulling even 2 m/s^2 will
     * outrun that model, the innovations blow up, observations get rejected
     * and the solution collapses. 0 keeps the RTKLIB default. */
    double accel_h, accel_v;
    double max_age;         /* max age of differential (s)                    */
    int    min_lock2fix;    /* min lock count before a sat joins AR           */
} rtkeng_cfg_t;

typedef struct {            /* engine counters, for health reporting          */
    uint32_t rover_epochs;  /* rover observation epochs decoded               */
    uint32_t base_epochs;   /* base observation epochs decoded                */
    uint32_t solutions;     /* rtkpos() calls that produced a solution        */
    uint32_t n_fix;         /* solutions with stat==SOLQ_FIX                  */
    uint32_t n_float;       /* solutions with stat==SOLQ_FLOAT                */
    uint32_t n_single;      /* solutions with stat==SOLQ_SINGLE               */
    uint32_t n_none;        /* rtkpos() failures                              */
    uint32_t eph_updates;   /* ephemerides accepted                           */
    uint32_t last_us;       /* duration of the last rtkpos() call (us)        */
    uint32_t max_us;        /* worst-case rtkpos() duration (us)              */
    uint64_t total_us;      /* cumulative rtkpos() time, for the mean         */
} rtkeng_stats_t;

typedef struct {
    rtk_t   rtk;            /* filter state + solution                        */
    raw_t   rov;            /* rover UBX decoder                              */
    raw_t   bas;            /* base UBX decoder   (base_fmt == UBX)           */
    rtcm_t  rtcm;           /* base RTCM3 decoder (base_fmt == RTCM3)         */
    nav_t   nav;            /* merged navigation data used for positioning    */

    obsd_t  obs[MAXOBS*2];  /* rover epoch followed by base epoch             */
    obsd_t  base_obs[MAXOBS];
    int     n_base;
    gtime_t base_time;

    rtkeng_cfg_t   cfg;
    rtkeng_stats_t stats;
    int     time_seeded;    /* wall clock has been set from GNSS              */
    int     inited;
} rtkeng_t;

void rtkeng_default_cfg(rtkeng_cfg_t *cfg);
int  rtkeng_init(rtkeng_t *e, const rtkeng_cfg_t *cfg);
void rtkeng_free(rtkeng_t *e);

int  rtkeng_rover_byte(rtkeng_t *e, unsigned char data);
int  rtkeng_base_byte (rtkeng_t *e, unsigned char data);

/* Convenience: feed a buffer. Returns the number of solutions produced. */
int  rtkeng_rover_buf(rtkeng_t *e, const unsigned char *buf, int n);
int  rtkeng_base_buf (rtkeng_t *e, const unsigned char *buf, int n);

const sol_t *rtkeng_solution(const rtkeng_t *e);
/* Solution position as lat/lon/height (deg, deg, m ellipsoidal). */
void rtkeng_solution_llh(const rtkeng_t *e, double *llh);

#ifdef __cplusplus
}
#endif
#endif /* RTK_ENGINE_H */
