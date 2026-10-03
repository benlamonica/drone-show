/*------------------------------------------------------------------------------
* gnss_sim.h : GNSS constellation and observable simulator
*
* Generates a plausible GPS(+Galileo or +GLONASS) constellation and, for any receiver
* position and epoch, the pseudorange / carrier-phase / Doppler a receiver
* would report. Paired with ubx_encode.c this feeds the RTK engine through the
* real UBX decode path.
*
* Error model is deliberately explicit so tests can dial individual effects in
* and out: satellite clock, troposphere, ionosphere (with a spatial gradient so
* it does not cancel perfectly over a baseline), receiver clock, per-satellite
* integer ambiguities, elevation-dependent noise, multipath and cycle slips.
*-----------------------------------------------------------------------------*/
#ifndef GNSS_SIM_H
#define GNSS_SIM_H
#include "rtklib.h"
#include "ubx_encode.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_MAXSAT   64
#define SIM_MAXGLO   24          /* GLONASS slots 1..24                      */

typedef struct {
    int    n;                    /* satellites in the constellation          */
    int    sat[SIM_MAXSAT];      /* RTKLIB satellite numbers                 */
    eph_t  eph[SIM_MAXSAT];      /* truth ephemerides (GPS/Galileo)          */
    geph_t geph[SIM_MAXGLO];     /* truth ephemerides (GLONASS), by slot-1   */
    int    week;
    double toe;                  /* time of ephemeris (s in week)            */
} sim_constellation_t;

typedef struct {
    double pos[3];               /* true ECEF position (m)                   */
    double vel[3];               /* true ECEF velocity (m/s)                 */
    double clk_bias;             /* receiver clock bias (s)                  */
    double clk_drift;            /* receiver clock drift (s/s)               */
    double amb[MAXSAT];          /* integer ambiguity per satellite (cycles) */
    uint8_t tracked[MAXSAT];     /* 1 once the ambiguity has been drawn      */
    uint32_t locktime[MAXSAT];   /* ms of continuous carrier lock            */
    uint8_t half_sub[MAXSAT];    /* report trkStat halfSub (bit 3), as a real
                                    receiver does once it has resolved the
                                    half-cycle and subtracted it              */
    uint32_t rng;                /* PRNG state (per station)                 */
    /* GLONASS inter-frequency code bias (m), indexed by channel+7. Each FDMA
     * channel passes through the antenna and front end with its own group
     * delay; when base and rover hardware differ these do not cancel. Zero
     * unless a test sets them. */
    double glo_code_bias[14];
} sim_station_t;

typedef struct {
    double code_sigma;      /* code noise at zenith (m), 1-sigma             */
    double phase_sigma;     /* carrier noise at zenith (m), 1-sigma          */
    double iono_zenith;     /* zenith ionospheric delay (m)                  */
    double iono_gradient;   /* iono change per km of baseline (m/km)         */
    double tropo_scale;     /* multiplier on the whole tropospheric delay    */
    double tropo_wet_zenith;/* zenith wet delay (m): the part that does not
                               cancel between stations                       */
    double tropo_wet_gradient; /* wet delay change per km of baseline (m/km)  */
    double multipath;       /* extra low-elevation code error (m)            */
    double elev_mask_deg;   /* satellites below this are not reported        */
    double sat_clk_err;     /* broadcast clock error (m), common-mode        */
} sim_errmodel_t;

void sim_default_errmodel(sim_errmodel_t *e);

/* Build a Walker-style constellation: `ngps` GPS, `ngal` Galileo and `nglo`
 * GLONASS SVs (each only where the build enables that system). GLONASS takes
 * the real 3-plane, 8-slot layout and slot->channel table, with its reference
 * time on the nearest 15-minute GLONASS boundary to `toe`. */
void sim_make_constellation(sim_constellation_t *c, int week, double toe,
                            int ngps, int ngal, int nglo);

/* GLONASS frequency channel (-7..+6) of a slot (1..24). */
int  sim_glo_fcn(int slot);

/* Initialise a station at a geodetic position (deg, deg, m). */
void sim_station_init(sim_station_t *st, const double *llh, uint32_t seed);

/* Generate one epoch of observations for `st`. `ref_llh` is the reference used
 * for the ionospheric gradient (pass the base station's position). Returns the
 * number of satellites written to obs[]/aux[]. */
int  sim_observe(const sim_constellation_t *c, sim_station_t *st,
                 const sim_errmodel_t *err, gtime_t time, const double *ref_llh,
                 obsd_t *obs, ubx_meas_aux_t *aux, int maxn);

/* Force a cycle slip on `sat` at the next epoch (redraws the ambiguity and
 * resets lock time, which makes decode_rxmrawx() raise LLI_SLIP). */
void sim_force_slip(sim_station_t *st, int sat);

/* Deterministic PRNG helpers, exposed so tests can build repeatable scenarios */
uint32_t sim_rand_u32(uint32_t *state);
double   sim_rand_gauss(uint32_t *state);

#ifdef __cplusplus
}
#endif
#endif /* GNSS_SIM_H */
