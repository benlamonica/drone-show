/*------------------------------------------------------------------------------
* sim_scenario.h : drives a base+rover RTK scenario into an rtkeng_t
*
* Everything reaches the engine as UBX bytes, so a scenario run exercises
* framing, checksums, RXM-RAWX decode, LNAV subframe assembly, ephemeris
* decode, and the filter -- the same path as real hardware.
*-----------------------------------------------------------------------------*/
#ifndef SIM_SCENARIO_H
#define SIM_SCENARIO_H
#include "gnss_sim.h"
#include "rtk_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_MOTION_STATIC 0
#define SIM_MOTION_CIRCLE 1     /* drone orbiting the take-off point */
#define SIM_MOTION_LINE   2

#define SIM_BASE_UBX      0     /* base as RXM-RAWX, like a second receiver  */
#define SIM_BASE_RTCM3    1     /* base as RTCM3 1005 + MSM7, like the field  */

typedef struct {
    double base_llh[3];         /* base station (deg,deg,m)                  */
    double rover_llh[3];        /* rover start point                         */
    int    ngps, ngal, nglo;    /* constellation sizes; a system the build
                                   leaves out is simply not generated        */
    int    base_fmt;            /* SIM_BASE_*; configure the engine to match */
    int    week;
    double toe;                 /* time of ephemeris (s in week)             */
    double start_tow;           /* first epoch (s in week)                   */
    double rate_hz;             /* observation rate                          */
    int    motion;
    double radius;              /* SIM_MOTION_CIRCLE: radius (m)             */
    double speed;               /* m/s                                       */
    uint32_t seed;
    sim_errmodel_t err;
} sim_scenario_cfg_t;

typedef struct {
    sim_scenario_cfg_t cfg;
    sim_constellation_t con;
    sim_station_t base, rover;
    gtime_t time;
    int epoch;
    double true_rover_ecef[3];  /* truth for the epoch just generated        */
    /* byte counters, for throughput accounting */
    uint32_t rover_bytes, base_bytes;
} sim_scenario_t;

void sim_scenario_default(sim_scenario_cfg_t *cfg);
void sim_scenario_init(sim_scenario_t *s, const sim_scenario_cfg_t *cfg);

/* Advance one epoch and feed both streams to `eng`.
 * Returns 1 if the engine produced a solution for this epoch. */
int  sim_scenario_step(sim_scenario_t *s, rtkeng_t *eng);

/* Horizontal / 3D error of the engine's current solution against truth. */
void sim_scenario_error(const sim_scenario_t *s, const rtkeng_t *eng,
                        double *herr, double *verr3d);

#ifdef __cplusplus
}
#endif
#endif
