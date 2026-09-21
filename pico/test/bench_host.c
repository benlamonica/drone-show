/*------------------------------------------------------------------------------
* bench_host.c : host runner for the RTK performance benchmark
*
* Same body as the on-target bench (bench_common.h), so numbers are directly
* comparable -- the host clock is scaled to a nominal 150 MHz to match the
* RP2350's default clk_sys. Host figures still flatter the target: an arm64 or
* x86 core has hardware double precision, wide out-of-order execution and
* megabytes of cache, where the RP2350 runs doubles on the DCP coprocessor
* behind a 16 KB XIP cache. Treat the host run as a regression check and the
* on-target run as the real budget.
*-----------------------------------------------------------------------------*/
#include "bench_common.h"

int main(void)
{
    bench_out_t o;
    uint32_t worst=0;
    int i;

    rtkpico_perf_init();
    printf("RTKLIB Pico port - performance benchmark (HOST)\n");
    printf("MAXSAT=%d NFREQ=%d MAXOBS=%d nominal clock %u Hz\n",
           MAXSAT,NFREQ,MAXOBS,rtkpico_cpu_hz());
    printf("sizeof(rtkeng_t)=%zu bytes\n",sizeof(rtkeng_t));

    bench_header();
    for (i=0;i<BENCH_NCASES;i++) {
        bench_run(&bench_cases[i],&o);
        bench_report(&bench_cases[i],&o);
        if (o.rtkpos_max_us>worst) worst=o.rtkpos_max_us;
    }
    bench_rates(worst);
    return 0;
}
