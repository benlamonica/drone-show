#-------------------------------------------------------------------------------
# RTKLIB on Raspberry Pi Pico 2 W (RP2350)
#
#   cmake -S pico -B build/pico -DPICO_BOARD=pico2_w
#   cmake --build build/pico -j8
#
# Produces:
#   drone-show-controller.uf2      - the flight-controller GPS firmware
#   bench_target.uf2 - on-target performance benchmark
#   test_target.uf2  - the host test-suite, run on the RP2350
#-------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2_w CACHE STRING "Board type")
set(PICO_PLATFORM rp2350 CACHE STRING "Platform")

include(${CMAKE_CURRENT_LIST_DIR}/cmake/pico_sdk_import.cmake)
project(rtkpico C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(RTK_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

# RTKLIB now lives in its own repository, vendored here as a submodule, so the
# port can be contributed upstream independently. RTK_ROOT stays the repo root
# (the pico/ tree lives there); RTKLIB_SRC points into the submodule.
set(RTKLIB_SRC ${RTK_ROOT}/rtklib/src)

if(NOT EXISTS ${RTKLIB_SRC}/rtklib.h)
    message(FATAL_ERROR
        "RTKLIB sources not found at ${RTKLIB_SRC}.\n"
        "The rtklib submodule is not checked out. Run:\n"
        "    git submodule update --init --recursive")
endif()

# Defaults match the NEO-M8P-2 pairing this port targets: MON-VER reports its
# capability as "GPS;GLO;BDS", so Galileo is not available and GLONASS is. The
# two must move together -- rtk_engine.c asks for SYS_GPS|SYS_GLO, and building
# with the opposite pair silently drops every GLONASS satellite (satno() cannot
# number them) while reserving 36 slots for Galileo satellites that can never
# arrive. Measured on this hardware: GPS+GLO is 249 KB live and 454 us worst
# -case rtkpos(), against 285 KB and 591 us for GPS+GAL -- cheaper both ways.
# Enabling GLONASS *in addition to* Galileo is what exhausted the heap once, at
# 402 KB; that is a three-constellation build, which this receiver cannot feed.
option(RTKPICO_ENABLE_GAL  "Include Galileo E1 (not present on M8P)"  OFF)
option(RTKPICO_ENABLE_GLO  "Include GLONASS (FDMA; +24 states)"       ON)
option(RTKPICO_RTCM3       "Accept RTCM3 base corrections" ON)
option(RTKPICO_COPY_TO_RAM "Run all code from SRAM instead of XIP flash" OFF)

# --- the trimmed RTKLIB core ---------------------------------------------------
set(RTK_SRCS
    ${RTKLIB_SRC}/rtkcmn.c
    ${RTKLIB_SRC}/rtkpos.c
    ${RTKLIB_SRC}/pntpos.c
    ${RTKLIB_SRC}/ephemeris.c
    ${RTKLIB_SRC}/lambda.c
    ${RTKLIB_SRC}/rcvraw.c
    ${RTKLIB_SRC}/rcv/ublox.c
    ${RTK_ROOT}/pico/port/rtk_stubs.c
    ${RTK_ROOT}/pico/port/rtk_platform.c
    ${RTK_ROOT}/pico/port/rtk_engine.c
    ${RTK_ROOT}/pico/port/rover_cfg.c
    ${RTK_ROOT}/pico/port/ubx_cfg.c
    ${RTK_ROOT}/pico/port/lockloss.c
)
if(RTKPICO_RTCM3)
  list(APPEND RTK_SRCS ${RTKLIB_SRC}/rtcm.c ${RTKLIB_SRC}/rtcm3.c)
endif()

add_library(rtkpico STATIC ${RTK_SRCS})
target_include_directories(rtkpico PUBLIC
    ${RTKLIB_SRC} ${RTK_ROOT}/pico/config ${RTK_ROOT}/pico/port)
target_compile_definitions(rtkpico PUBLIC RTKLIB_EMBEDDED PICO_BUILD)
if(NOT RTKPICO_ENABLE_GAL)
  target_compile_definitions(rtkpico PUBLIC RTKPICO_NO_GAL)
endif()
if(RTKPICO_ENABLE_GLO)
  target_compile_definitions(rtkpico PUBLIC RTKPICO_ENABLE_GLO)
endif()
if(NOT RTKPICO_RTCM3)
  target_compile_definitions(rtkpico PUBLIC RTKPICO_NO_RTCM)
endif()
# RTKLIB is 2013-era C; these warnings are pervasive and not actionable.
target_compile_options(rtkpico PRIVATE
    -Wall -Wno-unused-but-set-variable -Wno-unused-variable
    -Wno-unused-parameter -Wno-misleading-indentation -Wno-sign-compare
    -Wno-maybe-uninitialized -Wno-format-truncation)
target_link_libraries(rtkpico PUBLIC pico_stdlib hardware_uart hardware_irq)

# --- simulator (used by the on-target bench and tests) -------------------------
add_library(rtksim STATIC
    ${RTK_ROOT}/pico/sim/gnss_sim.c
    ${RTK_ROOT}/pico/sim/ubx_encode.c
    ${RTK_ROOT}/pico/sim/sim_scenario.c
)
target_include_directories(rtksim PUBLIC ${RTK_ROOT}/pico/sim)
target_link_libraries(rtksim PUBLIC rtkpico)
target_compile_options(rtksim PRIVATE -Wall -Wno-unused-parameter)

# Every target shares this: RP2350's double-precision coprocessor, plus a
# generous stack because rtkpos()'s call chain is deep.
function(rtkpico_configure target)
  # The RP2350 has no hardware double FPU; the DCP coprocessor implementation
  # is far faster than the compiler's soft-float routines. This is the single
  # most important performance switch in the build -- RTKLIB is double
  # precision from end to end.
  pico_set_double_implementation(${target} pico)
  pico_set_float_implementation(${target} pico)
  if(RTKPICO_COPY_TO_RAM)
    # Trades ~200 KB of SRAM for freedom from XIP cache misses in the filter.
    pico_set_binary_type(${target} copy_to_ram)
  endif()
  target_compile_options(${target} PRIVATE -O2 -ffunction-sections -fdata-sections)
  pico_add_extra_outputs(${target})
endfunction()

# Stack for anything that runs rtkpos(). The SDK gives core 0 the two 4 KB
# scratch banks and lets the heap grow right up to them; RTKLIB needs ~13 KB
# (measured on target: single-point positioning copies the 3.7 KB prcopt_t,
# and outsolstat()'s 8 KB buffer moves the stack pointer every epoch). This
# lowers __StackLimit -- the SDK's own _sbrk stops the heap there -- via a
# one-file linker override (pico/linker/section_end.incl), and enables
# PICO_USE_STACK_GUARDS, which on the RP2350 arms the core's stack limit
# register (MSPLIM) at the new bottom: an overflow faults instead of silently
# writing into the heap. Core 1 must stay unused (the stack spans SCRATCH_X);
# the linker asserts that.
set(RTKPICO_STACK_RESERVE 0x4000 CACHE STRING
    "RAM below the scratch banks reserved for core 0's stack (bytes)")
function(rtkpico_stack_guard target)
  pico_add_linker_script_override_path(${target} ${RTK_ROOT}/pico/linker FILES section_end.incl)
  pico_set_linker_script_var(${target} RTKPICO_STACK_RESERVE ${RTKPICO_STACK_RESERVE})
  # keep __HeapLimit (picolibc/LLVM, and our own reporting) in step
  pico_set_linker_script_var(${target} HEAP_LIMIT __StackLimit)
  target_compile_definitions(${target} PRIVATE PICO_USE_STACK_GUARDS=1)
endfunction()

# --- firmware ------------------------------------------------------------------
add_executable(drone-show-controller
    ${RTK_ROOT}/pico/app/main.c
    ${RTK_ROOT}/pico/port/ntrip_client.c
    ${RTK_ROOT}/pico/port/mavlink_min.c
    ${RTK_ROOT}/pico/port/mav_bridge.c
    ${RTK_ROOT}/pico/port/show_led.c
    ${RTK_ROOT}/pico/port/led_strip.c
)
pico_generate_pio_header(drone-show-controller ${RTK_ROOT}/pico/port/led_strip.pio)
# lwIP is driven from an alarm IRQ (threadsafe_background), not polled: the
# main loop is inside rtkpos() for ~100 ms per epoch and the polling variant
# would stall the TCP stack for exactly that long, every epoch.
target_link_libraries(drone-show-controller
    rtkpico
    pico_cyw43_arch_lwip_threadsafe_background
    hardware_pio hardware_dma
)
target_include_directories(drone-show-controller PRIVATE ${RTK_ROOT}/pico/config)
pico_enable_stdio_usb(drone-show-controller 1)
pico_enable_stdio_uart(drone-show-controller 0)   # UARTs carry GNSS data, not console text
# The health line must never stall the control loop. With the default 500 ms
# blocking timeout, a connected-but-slow USB host cost ~160 ms/s of main-loop
# time -- enough to visibly slow the status LED and to delay correction
# processing and NMEA output. Status output is best-effort by definition.
target_compile_definitions(drone-show-controller PRIVATE PICO_STDIO_USB_STDOUT_TIMEOUT_US=1000)

# Per-site configuration without editing main.c, e.g.
#   cmake -S pico -B build/pico -DRTKPICO_WIFI_SSID=other-net -DRTKPICO_NTRIP_HOST=10.0.0.5
foreach(v WIFI_SSID WIFI_PASSWORD NTRIP_HOST NTRIP_MOUNT MAV_SERVER_HOST)
  if(RTKPICO_${v})
    target_compile_definitions(drone-show-controller PRIVATE ${v}="${RTKPICO_${v}}")
  endif()
endforeach()
foreach(v SHOW_LED_PIN SHOW_LED_COUNT SHOW_LED_BRIGHTNESS)
  if(DEFINED RTKPICO_${v})
    target_compile_definitions(drone-show-controller PRIVATE ${v}=${RTKPICO_${v}})
  endif()
endforeach()
if(RTKPICO_NTRIP_PORT)
  target_compile_definitions(drone-show-controller PRIVATE NTRIP_PORT=${RTKPICO_NTRIP_PORT})
endif()
rtkpico_configure(drone-show-controller)

# --- on-target benchmark -------------------------------------------------------
add_executable(bench_target ${RTK_ROOT}/pico/app/bench_target.c)
target_include_directories(bench_target PRIVATE ${RTK_ROOT}/pico/test)
target_link_libraries(bench_target rtksim)
pico_enable_stdio_usb(bench_target 1)
pico_enable_stdio_uart(bench_target 0)
rtkpico_configure(bench_target)

# --- test suite, on target -----------------------------------------------------
add_executable(test_target
    ${RTK_ROOT}/pico/app/test_target.c
    ${RTK_ROOT}/pico/test/test_util.c
    ${RTK_ROOT}/pico/test/test_ubx.c
    ${RTK_ROOT}/pico/test/test_rtk.c
)
target_compile_definitions(test_target PRIVATE RTKPICO_TARGET_SUITE)
target_include_directories(test_target PRIVATE ${RTK_ROOT}/pico/test)
target_link_libraries(test_target rtksim)
pico_enable_stdio_usb(test_target 1)
pico_enable_stdio_uart(test_target 0)
rtkpico_configure(test_target)
rtkpico_stack_guard(test_target)

# --- primitive cost probe ------------------------------------------------------
# --- NTRIP connection bench ------------------------------------------------
# Network only: no receiver, no engine, nothing that blocks. Isolates whether a
# reconnect is the network or our own main loop starving the stack.
add_executable(ntrip_bench
    ${RTK_ROOT}/pico/app/ntrip_bench.c
    ${RTK_ROOT}/pico/port/ntrip_client.c
)
target_link_libraries(ntrip_bench pico_stdlib pico_cyw43_arch_lwip_threadsafe_background)
target_include_directories(ntrip_bench PRIVATE ${RTK_ROOT}/pico/config ${RTK_ROOT}/pico/port)
pico_enable_stdio_usb(ntrip_bench 1)
pico_enable_stdio_uart(ntrip_bench 0)
pico_add_extra_outputs(ntrip_bench)

# --- RTK relative-accuracy bench -------------------------------------------
# Rover + NTRIP corrections, logging the base->rover baseline in ENU. Needs the
# same lwIP background context as the firmware: the main loop sits inside
# rtkpos() for tens of ms and would stall a polled stack.
add_executable(rtk_bench
    ${RTK_ROOT}/pico/app/rtk_bench.c
    ${RTK_ROOT}/pico/port/ntrip_client.c
)
target_link_libraries(rtk_bench rtkpico pico_cyw43_arch_lwip_threadsafe_background)
target_include_directories(rtk_bench PRIVATE ${RTK_ROOT}/pico/config)
pico_enable_stdio_usb(rtk_bench 1)
pico_enable_stdio_uart(rtk_bench 0)
rtkpico_configure(rtk_bench)

# --- receiver bench --------------------------------------------------------
# Talks to the rover over UART0 and reports what is actually arriving. Shares
# rover_cfg.c with the firmware so it configures the receiver identically.
add_executable(gps_bench ${RTK_ROOT}/pico/app/gps_bench.c)
target_link_libraries(gps_bench rtkpico)
target_include_directories(gps_bench PRIVATE ${RTK_ROOT}/pico/config)
# USB only: uart0 is the rover link, so console text there would be transmitted
# straight at the receiver.
pico_enable_stdio_usb(gps_bench 1)
pico_enable_stdio_uart(gps_bench 0)
rtkpico_configure(gps_bench)

add_executable(perf_probe ${RTK_ROOT}/pico/app/perf_probe.c)
target_link_libraries(perf_probe rtksim)
pico_enable_stdio_usb(perf_probe 1)
pico_enable_stdio_uart(perf_probe 0)
rtkpico_configure(perf_probe)

# --- LED strip bring-up --------------------------------------------------------
add_executable(led_demo
    ${RTK_ROOT}/pico/app/led_demo.c
    ${RTK_ROOT}/pico/port/led_strip.c
    ${RTK_ROOT}/pico/port/show_led.c
    ${RTK_ROOT}/pico/port/mavlink_min.c
)
pico_generate_pio_header(led_demo ${RTK_ROOT}/pico/port/led_strip.pio)
target_include_directories(led_demo PRIVATE ${RTK_ROOT}/pico/port)
target_link_libraries(led_demo pico_stdlib hardware_pio hardware_dma hardware_clocks hardware_uart)
pico_enable_stdio_usb(led_demo 1)
pico_enable_stdio_uart(led_demo 0)
foreach(v LED_PIN LED_COUNT LED_BRIGHTNESS LED_BRINGUP_MS)
  if(DEFINED RTKPICO_${v})
    target_compile_definitions(led_demo PRIVATE ${v}=${RTKPICO_${v}})
  endif()
endforeach()
rtkpico_configure(led_demo)
