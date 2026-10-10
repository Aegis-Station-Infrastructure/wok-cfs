# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
project(wok_hk_health C)

set(APP_SRC_FILES
    fsw/src/wok_hk_app.c
)

add_cfe_app(wok_hk_health ${APP_SRC_FILES})

target_include_directories(wok_hk_health PUBLIC
    fsw/inc
    fsw/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_common/msgs/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_common/hal/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_sequencer/fsw/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_nav/fsw/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_geo/fsw/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_seis/fsw/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_hk_health/fsw/inc
)
target_link_libraries(wok_hk_health wok_hal)
