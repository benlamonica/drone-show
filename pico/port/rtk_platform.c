/*------------------------------------------------------------------------------
* rtk_platform.c : clock + cycle-counter shims (builds for RP2350 and host)
*-----------------------------------------------------------------------------*/
#include "rtk_platform.h"

#ifdef PICO_BUILD
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/clocks.h"
#include "hardware/structs/scb.h"
#else
#include <time.h>
#endif

/* UTC seconds (gtime_t.time) corresponding to elapsed_us()==base_us_ */
static gtime_t base_utc_={0};
static uint64_t base_us_=0;
static int have_gnss_time_=0;

static uint64_t elapsed_us(void)
{
#ifdef PICO_BUILD
    return time_us_64();
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC,&ts);
    return (uint64_t)ts.tv_sec*1000000ull+(uint64_t)(ts.tv_nsec/1000);
#endif
}

/* Establish the seeded era on first use: only the 1024-week GPS epoch matters
 * before the receiver hands us real time (see adjgpsweek()). */
static void ensure_base(void)
{
    if (base_utc_.time) return;
    base_utc_=gpst2utc(gpst2time(RTKPICO_SEED_GPS_WEEK,0.0));
    base_us_=elapsed_us();
}

void rtkpico_utc_now(double *ep)
{
    gtime_t t;
    ensure_base();
    t=timeadd(base_utc_,(double)(elapsed_us()-base_us_)*1E-6);
    time2epoch(t,ep);
}

void rtkpico_set_gps_time(gtime_t gpst)
{
    if (gpst.time==0) return;
    base_utc_=gpst2utc(gpst);
    base_us_=elapsed_us();
    have_gnss_time_=1;
    /* timeget() adds timeoffset_ on top of us; clear it so the two agree. */
    timeset(base_utc_);
}

int rtkpico_time_is_gnss(void) { return have_gnss_time_; }

unsigned int rtkpico_tick_ms(void)
{
    return (unsigned int)(elapsed_us()/1000ull);
}

void rtkpico_sleep_ms(int ms)
{
    if (ms<=0) return;
#ifdef PICO_BUILD
    sleep_ms((uint32_t)ms);
#else
    struct timespec ts;
    ts.tv_sec=ms/1000; ts.tv_nsec=(long)(ms%1000)*1000000L;
    nanosleep(&ts,NULL);
#endif
}

/* --- cycle counter -------------------------------------------------------- */

#ifdef PICO_BUILD
/* Cortex-M33 DWT. Not enabled out of reset: set TRCENA then CYCCNTENA. */
#define DWT_CTRL   (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004u)
#define DEMCR      (*(volatile uint32_t *)0xE000EDFCu)
#define DEMCR_TRCENA      (1u<<24)
#define DWT_CTRL_CYCCNTENA (1u<<0)

void rtkpico_perf_init(void)
{
    DEMCR|=DEMCR_TRCENA;
    DWT_CYCCNT=0;
    DWT_CTRL|=DWT_CTRL_CYCCNTENA;
}
uint32_t rtkpico_cycles(void) { return DWT_CYCCNT; }
uint32_t rtkpico_cpu_hz(void) { return clock_get_hz(clk_sys); }
#else
/* Host: synthesise a 150 MHz tick so numbers line up with the target. */
#define HOST_NOMINAL_HZ 150000000u
void rtkpico_perf_init(void) {}
uint32_t rtkpico_cycles(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC,&ts);
    return (uint32_t)(((uint64_t)ts.tv_sec*1000000000ull+ts.tv_nsec)
                      /(1000000000ull/HOST_NOMINAL_HZ));
}
uint32_t rtkpico_cpu_hz(void) { return HOST_NOMINAL_HZ; }
#endif
