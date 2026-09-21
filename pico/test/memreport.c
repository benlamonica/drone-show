/*------------------------------------------------------------------------------
* memreport.c : where the RAM goes, stock RTKLIB vs this port
*
* Build twice (with and without RTKLIB_EMBEDDED) to see the effect of the
* trimming; see pico/test/CMakeLists.txt.
*-----------------------------------------------------------------------------*/
#include "rtklib.h"
#include "rtk_engine.h"
#include <stdio.h>

#define ROW(name,bytes) printf("  %-34s %9zu\n",name,(size_t)(bytes))

int main(void)
{
    size_t ekf,ephpool,total;
    int nx=3+MAXSAT*1;

#ifdef RTKLIB_EMBEDDED
    printf("=== EMBEDDED configuration ===\n");
#else
    printf("=== STOCK RTKLIB configuration ===\n");
#endif
    printf("MAXSAT=%d  NFREQ=%d  NEXOBS=%d  MAXOBS=%d  NSYS=%d\n\n",
           MAXSAT,NFREQ,NEXOBS,MAXOBS,NSYS);

    printf("  %-34s %9s\n","structure","bytes");
    printf("  ---------------------------------- ---------\n");
    ROW("nav_t (navigation data)",sizeof(nav_t));
    ROW("raw_t (one UBX decoder)",sizeof(raw_t));
    ROW("rtcm_t (RTCM3 decoder)",sizeof(rtcm_t));
    ROW("rtk_t (filter state)",sizeof(rtk_t));
    ROW("  of which prcopt_t",sizeof(prcopt_t));
    ROW("ssat_t x MAXSAT",sizeof(ssat_t)*MAXSAT);
    ROW("obsd_t",sizeof(obsd_t));

    ekf=(size_t)nx*8+(size_t)nx*nx*8+3*8+9*8;
    ephpool=sizeof(eph_t)*MAXSAT;
    printf("\n  heap, allocated once at init:\n");
    printf("  %-34s %9zu   (nx=%d)\n","EKF x/P/xa/Pa",ekf,nx);
    printf("  %-34s %9zu\n","eph pool per nav_t",ephpool);
    printf("  %-34s %9zu\n","obs pools per raw_t",2*(size_t)MAXOBS*sizeof(obsd_t));

#ifdef RTKLIB_EMBEDDED
    total=sizeof(rtkeng_t)+ekf+3*ephpool;
    printf("\n  %-34s %9zu  (%.1f KB)\n","rtkeng_t + heap, live total",
           total,total/1024.0);
    printf("  %-34s %9d  (%.1f%% of RP2350 SRAM)\n","vs 520 KB available",
           520*1024,100.0*total/(520.0*1024));
#else
    total=sizeof(raw_t)*2+sizeof(rtk_t)+sizeof(nav_t)+ekf+3*ephpool;
    printf("\n  %-34s %9zu  (%.1f KB)\n",
           "equivalent live total",total,total/1024.0);
    printf("  %-34s %9d  -> %s\n","vs 520 KB available",520*1024,
           total>520u*1024?"DOES NOT FIT":"fits");
#endif
    return 0;
}
