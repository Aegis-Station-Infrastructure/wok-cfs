# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
# Add this to your sample_defs/targets.cmake
# (inside the cpu1 target block or as a global append)

list(APPEND MISSION_GLOBAL_APPLIST
    wok_common
    wok_sequencer
    wok_nav
    wok_geo
    wok_seis
    wok_drill
    wok_hk_health
)
