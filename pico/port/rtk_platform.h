/*------------------------------------------------------------------------------
* rtk_platform.h : clock + cycle-counter shims for the RTKLIB embedded port
*-----------------------------------------------------------------------------*/
#ifndef RTK_PLATFORM_H
#define RTK_PLATFORM_H
#include "rtklib.h"

#ifdef __cplusplus
extern "C" {
#endif

/* hooks called by rtkcmn.c under RTKLIB_EMBEDDED */
void         rtkpico_utc_now(double *ep);   /* current UTC as epoch[6]        */
unsigned int rtkpico_tick_ms(void);         /* monotonic ms since boot        */
void         rtkpico_sleep_ms(int ms);

/* Seed the wall clock from GNSS. RTKLIB resolves the 10-bit week in GPS LNAV
 * against timeget(), so this must happen before ephemerides are trusted; the
 * engine calls it from the first RXM-RAWX. */
void    rtkpico_set_gps_time(gtime_t gpst);
int     rtkpico_time_is_gnss(void);         /* 0 = still on the seeded era    */

/* --- cycle-accurate timing ------------------------------------------------
 * On RP2350 this is the Cortex-M33 DWT cycle counter (1 tick = 1 clk_sys
 * cycle, 150 MHz by default). On the host it is a monotonic ns clock scaled
 * to a nominal 150 MHz so the same benchmark numbers are comparable. */
void     rtkpico_perf_init(void);
uint32_t rtkpico_cycles(void);
uint32_t rtkpico_cpu_hz(void);

#ifdef __cplusplus
}
#endif
#endif /* RTK_PLATFORM_H */
