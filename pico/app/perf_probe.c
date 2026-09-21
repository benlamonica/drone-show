/*------------------------------------------------------------------------------
* perf_probe.c : where does an RTK epoch actually spend its cycles on RP2350?
*
* The RP2350 has no double-precision FPU. Everything RTKLIB does is double, so
* the first question is always whether the SDK's fast double routines (the
* RP2350 DCP coprocessor) are linked in, or whether we silently fell back to
* the compiler's generic soft-float. The primitive timings below answer that:
* a DCP double multiply is on the order of 20 cycles, the generic aeabi
* routine several times that.
*-----------------------------------------------------------------------------*/
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/clocks.h"
#include "rtk_platform.h"
#include "gnss_sim.h"
#include <stdio.h>
#include <math.h>

static volatile double sink;
static uint32_t hz;

#define TIME(label,iters,body) do {                                     \
    uint32_t _t0=rtkpico_cycles();                                      \
    for (int _i=0;_i<(iters);_i++) { body; }                            \
    uint32_t _c=rtkpico_cycles()-_t0;                                   \
    printf("  %-26s %8.1f cycles/op  (%.3f us/op)\n",label,             \
           (double)_c/(iters),(double)_c/(iters)*1e6/hz);               \
} while (0)

int main(void)
{
    static sim_constellation_t con;
    double a=1.0000001,b=3.14159265,c=0.0;
    double rs[6],dts[2],var,pos[3],e[3],azel[2];
    int i;

    stdio_init_all();
    rtkpico_perf_init();
    for (i=0;i<50&&!stdio_usb_connected();i++) sleep_ms(100);
    sleep_ms(300);
    hz=clock_get_hz(clk_sys);

    printf("\n=== RP2350 double-precision / RTKLIB primitive costs ===\n");
    printf("clk_sys = %u Hz\n\n",(unsigned)hz);

    printf("scalar double primitives:\n");
    TIME("add",            20000, c+=a);
    TIME("multiply",       20000, c*=a);
    TIME("divide",         20000, c=b/a);
    TIME("sqrt",           20000, c=sqrt(a+_i));
    TIME("sin",            10000, c=sin(a+_i));
    TIME("cos",            10000, c=cos(a+_i));
    TIME("atan2",          10000, c=atan2(a+_i,b));
    sink=c;

    printf("\nsingle-precision, for comparison:\n");
    { volatile float f=1.0001f,g=3.14159f;
      TIME("float multiply", 20000, f*=g);
      TIME("float sqrtf",    20000, f=sqrtf(g+_i));
    }

    /* update_led() runs once per received byte, so the cost of its cheapest
     * component -- reading the millisecond clock -- is what decides whether
     * that is affordable. */
    printf("\nstatus-LED support:\n");
    {
        volatile uint32_t u=0;
        TIME("rtkpico_tick_ms",   20000, u+=rtkpico_tick_ms());
        TIME("time_us_64",        20000, u+=(uint32_t)time_us_64());
        TIME("time_us_32",        20000, u+=time_us_32());
        (void)u;
    }

    printf("\nRTKLIB kernels:\n");
    sim_make_constellation(&con,2300,302400.0,24,0);
    {
        gtime_t t=gpst2time(2300,302400.0);
        TIME("eph2pos (1 satellite)", 200, eph2pos(t,&con.eph[_i%24],rs,dts,&var));
    }
    pos[0]=35.68*D2R; pos[1]=139.76*D2R; pos[2]=40.0;
    {
        double rr[3]; pos2ecef(pos,rr);
        TIME("geodist",  2000, sink=geodist(rs,rr,e));
        TIME("satazel",  2000, sink=satazel(pos,e,azel));
    }
    {
        static double A[400],B[400],C[400];
        for (i=0;i<400;i++){A[i]=i*0.001;B[i]=i*0.002;}
        TIME("matmul 20x20x20", 20, matmul("NN",20,20,20,1.0,A,B,0.0,C));
    }
    {
        static double A[400],Y[400];
        TIME("matinv 20x20", 20, ({ for(int k=0;k<400;k++)A[k]=(k%21==0)?2.0:0.01;
                                    matcpy(Y,A,20,20); matinv(Y,20); }));
    }
    printf("\n(a DCP double multiply is ~20 cycles; the generic soft-float\n"
           " aeabi routine is several times that -- use this to confirm which\n"
           " implementation actually got linked)\n");
    while (1) sleep_ms(1000);
}
