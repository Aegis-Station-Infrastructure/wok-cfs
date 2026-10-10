# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
project(wok_common C)

set(WOK_MSGS_INC ${CMAKE_CURRENT_SOURCE_DIR}/msgs/inc CACHE PATH "" FORCE)
set(WOK_HAL_INC  ${CMAKE_CURRENT_SOURCE_DIR}/hal/inc  CACHE PATH "" FORCE)

add_library(wok_hal STATIC
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
target_include_directories(wok_hal PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/hal/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/msgs/inc
)
target_link_libraries(wok_hal m)
