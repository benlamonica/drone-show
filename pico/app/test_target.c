/*------------------------------------------------------------------------------
* test_target.c : run the full test suite on the RP2350
*
* The host tests and this share their sources exactly, so a green run here
* proves the trimmed configuration, the DCP double-precision routines and the
* Thumb-2 code generation all agree with the host.
*-----------------------------------------------------------------------------*/
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include <stdio.h>

int  test_ubx_main(void);
int  test_rtk_main(void);

int main(void)
{
    int i,fail=0;

    stdio_init_all();
    for (i=0;i<50&&!stdio_usb_connected();i++) sleep_ms(100);
    sleep_ms(300);

    printf("\n===== RTKLIB Pico port: on-target test suite =====\n");
    printf("clk_sys=%u Hz\n",(unsigned)clock_get_hz(clk_sys));

    fail+=test_ubx_main();
    fail+=test_rtk_main();

    printf("\n===== on-target suite %s =====\n",fail?"FAILED":"PASSED");
    while (1) sleep_ms(1000);
}
