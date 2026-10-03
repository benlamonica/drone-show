#-------------------------------------------------------------------------------
# Host build of the RTKLIB Pico port: same sources, same RTKLIB_EMBEDDED
# configuration, native toolchain. Lets the port be developed and regression
# tested without hardware; bench_host mirrors bench_target.
#-------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.13)
project(rtkpico_host C)

set(CMAKE_C_STANDARD 99)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_C_FLAGS_RELEASE "-O2 -g")

set(RTK_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)

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

find_library(M_LIB m)

# The port is built twice, because it ships in two constellation pairings and
# each has code the other never compiles:
#   rtkpico      GPS + Galileo  (the library default)
#   rtkpico_glo  GPS + GLONASS  (what the M8P firmware flies: FDMA channels,
#                               PZ-90 ephemerides, MSM7 1087 from the base)
# Every test runs against both, so `ctest` covers the shipping configuration
# and not only the one that happens to be the default.
function(rtkpico_host_config SUFFIX)
  set(LIB rtkpico${SUFFIX})
  set(SIM rtksim${SUFFIX})
  add_library(${LIB} STATIC
      ${RTKLIB_SRC}/rtkcmn.c
      ${RTKLIB_SRC}/rtkpos.c
      ${RTKLIB_SRC}/pntpos.c
      ${RTKLIB_SRC}/ephemeris.c
      ${RTKLIB_SRC}/lambda.c
      ${RTKLIB_SRC}/rcvraw.c
      ${RTKLIB_SRC}/rcv/ublox.c
      ${RTKLIB_SRC}/rtcm.c
      ${RTKLIB_SRC}/rtcm3.c
      ${RTK_ROOT}/pico/port/rtk_stubs.c
      ${RTK_ROOT}/pico/port/rtk_platform.c
      ${RTK_ROOT}/pico/port/rtk_engine.c
      ${RTK_ROOT}/pico/port/mavlink_min.c
      ${RTK_ROOT}/pico/port/mav_bridge.c
      ${RTK_ROOT}/pico/port/show_led.c
      ${RTK_ROOT}/pico/port/ubx_cfg.c
      ${RTK_ROOT}/pico/port/lockloss.c
      # RTKLIB's RTCM3 encoder: host only, so the simulated base can send MSM7
      # exactly as a u-blox base does. The firmware never encodes RTCM.
      ${RTKLIB_SRC}/rtcm3e.c
  )
  target_include_directories(${LIB} PUBLIC
      ${RTKLIB_SRC} ${RTK_ROOT}/pico/config ${RTK_ROOT}/pico/port)
  target_compile_definitions(${LIB} PUBLIC RTKLIB_EMBEDDED RTKPICO_HAVE_RTCM3E ${ARGN})
  # RTKLIB is 2013-era C: these warnings are pervasive and not actionable here.
  target_compile_options(${LIB} PRIVATE -Wall -Wno-unused-but-set-variable
      -Wno-unused-variable -Wno-unused-parameter -Wno-misleading-indentation
      -Wno-excess-initializers -Wno-initializer-overrides)
  if(APPLE)
    target_compile_definitions(${LIB} PRIVATE _DARWIN_C_SOURCE)
  endif()
  if(M_LIB)
    target_link_libraries(${LIB} PUBLIC ${M_LIB})
  endif()

  add_library(${SIM} STATIC
      ${RTK_ROOT}/pico/sim/gnss_sim.c
      ${RTK_ROOT}/pico/sim/ubx_encode.c
      ${RTK_ROOT}/pico/sim/sim_scenario.c
  )
  target_include_directories(${SIM} PUBLIC ${RTK_ROOT}/pico/sim)
  target_link_libraries(${SIM} PUBLIC ${LIB})

  foreach(t test_ubx test_rtk test_mavlink test_led test_ubxcfg test_lockloss)
    add_executable(${t}${SUFFIX} ${CMAKE_CURRENT_LIST_DIR}/${t}.c
                                 ${CMAKE_CURRENT_LIST_DIR}/test_util.c)
    target_include_directories(${t}${SUFFIX} PRIVATE ${CMAKE_CURRENT_LIST_DIR})
    target_link_libraries(${t}${SUFFIX} PRIVATE ${SIM})
    add_test(NAME ${t}${SUFFIX} COMMAND ${t}${SUFFIX})
  endforeach()
endfunction()

enable_testing()
rtkpico_host_config("")
rtkpico_host_config(_glo RTKPICO_NO_GAL RTKPICO_ENABLE_GLO)

# Field-capture replay (not a CTest test: it needs a capture file). Built
# against the GPS+GLONASS library, the pairing the firmware flies, so it
# reproduces the board. See "Replaying a field capture" in pico/README.md.
add_executable(replay ${CMAKE_CURRENT_LIST_DIR}/replay.c)
target_link_libraries(replay PRIVATE rtkpico_glo)

add_executable(bench_host ${CMAKE_CURRENT_LIST_DIR}/bench_host.c)
target_include_directories(bench_host PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(bench_host PRIVATE rtksim)

# Memory accounting, built both ways so the trimming can be quantified.
add_executable(memreport ${CMAKE_CURRENT_LIST_DIR}/memreport.c)
target_link_libraries(memreport PRIVATE rtkpico)

# Same file against stock RTKLIB defaults (no RTKLIB_EMBEDDED, no trimming).
add_executable(memreport_stock ${CMAKE_CURRENT_LIST_DIR}/memreport.c
                               ${RTKLIB_SRC}/rtkcmn.c)
target_include_directories(memreport_stock PRIVATE
    ${RTKLIB_SRC} ${RTK_ROOT}/pico/port ${RTK_ROOT}/pico/config)
target_compile_definitions(memreport_stock PRIVATE ENAGAL RTKPICO_SEED_GPS_WEEK=2300)
target_compile_options(memreport_stock PRIVATE -w)
if(APPLE)
  target_compile_definitions(memreport_stock PRIVATE _DARWIN_C_SOURCE)
endif()
