/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK Mission — Performance Monitor IDs
 *
 * Used with CFE_ES_PerfLogEntry / CFE_ES_PerfLogExit for timing analysis.
 * Keep these unique across all apps in the mission.
 ************************************************************************/
#ifndef WOK_PERFIDS_H
#define WOK_PERFIDS_H

/* Base offset — adjust if colliding with other mission apps' perf IDs */
#define WOK_PERF_BASE            40

#define WOK_SEQ_PERF_ID          (WOK_PERF_BASE + 0)
#define WOK_NAV_PERF_ID          (WOK_PERF_BASE + 1)
#define WOK_GEO_PERF_ID          (WOK_PERF_BASE + 2)
#define WOK_SEIS_PERF_ID         (WOK_PERF_BASE + 3)
#define WOK_HK_PERF_ID           (WOK_PERF_BASE + 4)
#define WOK_DRILL_PERF_ID        (WOK_PERF_BASE + 5)

#endif /* WOK_PERFIDS_H */
