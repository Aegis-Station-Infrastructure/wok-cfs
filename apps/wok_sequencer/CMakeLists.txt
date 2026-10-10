# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
project(wok_sequencer C)

set(APP_SRC_FILES
    fsw/src/wok_seq_app.c
)

add_cfe_app(wok_sequencer ${APP_SRC_FILES})

target_include_directories(wok_sequencer PUBLIC
    fsw/inc
    fsw/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_common/msgs/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_common/hal/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_sequencer/fsw/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_nav/fsw/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_geo/fsw/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_seis/fsw/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_hk_health/fsw/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_drill/fsw/inc
)

# Pod-target trigonometry (CP-3 placement choreography)
target_link_libraries(wok_sequencer m)
