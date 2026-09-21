/*------------------------------------------------------------------------------
* bench_target.c : on-target performance benchmark (RP2350)
*
* Same benchmark body as the host runner, measured with the Cortex-M33 DWT
* cycle counter. Prints over USB CDC; connect with the /tmp screen or
* `picocom /dev/tty.usbmodem* 115200`.
*-----------------------------------------------------------------------------*/
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "bench_common.h"

int main(void)
{
    bench_out_t o;
    uint32_t worst=0;
    int i;

    stdio_init_all();
    rtkpico_perf_init();

    /* wait for the USB console so no output is lost */
    for (i=0;i<50&&!stdio_usb_connected();i++) sleep_ms(100);
    sleep_ms(300);

    printf("\n=============================================================\n");
    printf("RTKLIB Pico port - performance benchmark (RP2350 on target)\n");
    printf("=============================================================\n");
    printf("clk_sys      : %u Hz\n",(unsigned)clock_get_hz(clk_sys));
    printf("config       : MAXSAT=%d NFREQ=%d MAXOBS=%d\n",MAXSAT,NFREQ,MAXOBS);
    printf("rtkeng_t     : %u bytes\n",(unsigned)sizeof(rtkeng_t));
    printf("double impl  : pico (RP2350 DCP coprocessor)\n");
#if PICO_COPY_TO_RAM
    printf("code         : copied to SRAM\n");
#else
    printf("code         : XIP from flash\n");
#endif

    bench_header();
    for (i=0;i<BENCH_NCASES;i++) {
        bench_run(&bench_cases[i],&o);
        bench_report(&bench_cases[i],&o);
        if (o.rtkpos_max_us>worst) worst=o.rtkpos_max_us;
    }
    bench_rates(worst);

    printf("\nbenchmark complete\n");
    while (1) { sleep_ms(1000); }
}
