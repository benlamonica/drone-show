/*------------------------------------------------------------------------------
* test_target.c : run the full test suite on the RP2350
*
* The host tests and this share their sources exactly, so a green run here
* proves the trimmed configuration, the DCP double-precision routines and the
* Thumb-2 code generation all agree with the host.
*
* Memory is the part that differs. The RTK tests borrow one engine and one
* scenario, allocated here once: statically they were two of each, 340 KB of
* bss, which left too little heap for the engine to start.
*
* The stack is the other constraint. The SDK gives core 0 the two 4 KB scratch
* banks, and directly below them the heap may grow with nothing between.
* RTKLIB needs ~13 KB: the single-point path (rtkpos -> pntpos -> estpos, with
* a 3.7 KB prcopt_t copy on the way) writes ~10 KB deep, and rtkpos() calls
* outsolstat() every epoch, whose 8 KB message buffer drops the stack pointer
* ~12 KB even though it returns at once. rtkpico_stack_guard() in the CMake
* file reserves RAM for it at link time and arms the MSPLIM stack guard at the
* new bottom, so an overflow faults. On top of that this runner paints the
* stack at start and reports the deepest use at the end.
*-----------------------------------------------------------------------------*/
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include "rtk_engine.h"
#include "sim_scenario.h"

int  test_ubx_main(void);
int  test_rtk_suite(rtkeng_t *eng, sim_scenario_t *sim);

/* From the linker script: with rtkpico_stack_guard(), __StackLimit is the
 * bottom of core 0's stack (and the heap's ceiling), __StackTop its top. */
extern uint32_t __StackLimit[], __StackTop[];
extern char __end__[], __HeapLimit[];

#define PAINT       0xA5A5A5A5u
#define STACK_FLOOR __StackLimit

/* Fill the unused stack below the current frame with a pattern. */
static void stack_paint(void)
{
    volatile uint32_t *p=STACK_FLOOR;
    uint32_t sp;
    __asm volatile ("mov %0, sp" : "=r"(sp));
    while ((uint32_t)p<sp-256u) *p++=PAINT;
}

/* Deepest stack use since stack_paint(), in bytes. Returns -1 if the pattern
 * at the very bottom is gone -- the MSPLIM guard should have faulted first. */
static int stack_used(void)
{
    const uint32_t *p=STACK_FLOOR;
    if (*p!=PAINT) return -1;
    while (p<__StackTop&&*p==PAINT) p++;
    return (int)((uint32_t)__StackTop-(uint32_t)p);
}

int main(void)
{
    rtkeng_t *eng;
    sim_scenario_t *sim;
    int i,fail=0,used;
    size_t heap=(size_t)(__HeapLimit-__end__);

    stack_paint();
    stdio_init_all();
    for (i=0;i<50&&!stdio_usb_connected();i++) sleep_ms(100);
    sleep_ms(300);

    printf("\n===== RTKLIB Pico port: on-target test suite =====\n");
    printf("clk_sys=%u Hz  heap %u KB  stack %u KB (scratch X+Y + reserve)\n",
           (unsigned)clock_get_hz(clk_sys),(unsigned)(heap/1024),
           (unsigned)(((uint32_t)__StackTop-(uint32_t)STACK_FLOOR)/1024));

    /* Up front and for the whole run, before the UBX tests' allocations can
     * fragment the heap. */
    eng=(rtkeng_t *)malloc(sizeof(rtkeng_t));
    sim=(sim_scenario_t *)malloc(sizeof(sim_scenario_t));
    if (!eng||!sim) {
        printf("cannot allocate the engine (%u + %u bytes)\n",
               (unsigned)sizeof(rtkeng_t),(unsigned)sizeof(sim_scenario_t));
        fail=1;
    }
    else {
        fail+=test_ubx_main();
        fail+=test_rtk_suite(eng,sim);
    }

    used=stack_used();
    {
        struct mallinfo mi=mallinfo();
        printf("\nheap high-water %u of %u KB\n",(unsigned)(mi.arena/1024),
               (unsigned)(heap/1024));
    }
    if (used<0) {
        printf("stack: reached the bottom of the reserve -- FAIL\n");
        fail++;
    }
    else {
        printf("stack: deepest %d of %u bytes\n",used,
               (unsigned)((uint32_t)__StackTop-(uint32_t)STACK_FLOOR));
    }
    printf("\n===== on-target suite %s =====\n",fail?"FAILED":"PASSED");
    while (1) sleep_ms(1000);
}
