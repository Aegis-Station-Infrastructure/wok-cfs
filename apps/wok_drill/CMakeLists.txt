# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
project(wok_drill C)

set(APP_SRC_FILES
    fsw/src/wok_drill_app.c
)

add_cfe_app(wok_drill ${APP_SRC_FILES})

target_include_directories(wok_drill PUBLIC
    fsw/inc
    fsw/src
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_common/msgs/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../wok_common/hal/inc
)
target_link_libraries(wok_drill wok_hal)
