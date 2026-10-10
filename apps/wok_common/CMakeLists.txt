# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
project(wok_common C)

set(WOK_MSGS_INC ${CMAKE_CURRENT_SOURCE_DIR}/msgs/inc CACHE PATH "" FORCE)
set(WOK_HAL_INC  ${CMAKE_CURRENT_SOURCE_DIR}/hal/inc  CACHE PATH "" FORCE)

# HAL backend selection. linux_sim (default) is the physics simulation used
# for development and regression; flight_stub is the no-op starting point
# for a hardware port (every call returns WOK_HAL_ERROR until implemented).
# The cFS top-level configure does not forward extra -D options to the
# per-target sub-build, so set it there after `make prep`, e.g.
#   (cd build/native/default_cpu1 && cmake -DWOK_HAL_BACKEND=flight_stub .)
# then rebuild. The value is cached, so it persists across rebuilds.
set(WOK_HAL_BACKEND "linux_sim" CACHE STRING "HAL backend: linux_sim or flight_stub")

if(WOK_HAL_BACKEND STREQUAL "linux_sim")
    set(WOK_HAL_SRC
        hal/src/linux_sim/wok_hal_sim_common.c
        hal/src/linux_sim/wok_hal_sim_geology.c
        hal/src/linux_sim/wok_hal_sim_coupling.c
        hal/src/linux_sim/wok_hal_sim_mobility.c
        hal/src/linux_sim/wok_hal_sim_geo.c
        hal/src/linux_sim/wok_hal_sim_seis.c
        hal/src/linux_sim/wok_hal_sim_power.c
        hal/src/linux_sim/wok_hal_sim_storage.c
        hal/src/linux_sim/wok_hal_sim_watchdog.c
        hal/src/linux_sim/wok_hal_sim_drill.c
    )
elseif(WOK_HAL_BACKEND STREQUAL "flight_stub")
    set(WOK_HAL_SRC
        hal/src/flight_stub/wok_hal_flight_stub.c
        hal/src/flight_stub/wok_hal_drill_flight_stub.c
    )
else()
    message(FATAL_ERROR "WOK_HAL_BACKEND must be 'linux_sim' or 'flight_stub' (got '${WOK_HAL_BACKEND}')")
endif()

add_library(wok_hal STATIC ${WOK_HAL_SRC})
target_include_directories(wok_hal PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/hal/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/msgs/inc
)
target_link_libraries(wok_hal m)
