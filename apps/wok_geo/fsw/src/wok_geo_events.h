/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK_GEO — Event IDs
 ************************************************************************/
#ifndef WOK_GEO_EVENTS_H
#define WOK_GEO_EVENTS_H

#define WOK_GEO_INIT_INF_EID         1
#define WOK_GEO_NOOP_INF_EID         2
#define WOK_GEO_RESET_INF_EID        3
#define WOK_GEO_CMD_ERR_EID          4
#define WOK_GEO_PIPE_ERR_EID         5
#define WOK_GEO_LEN_ERR_EID          6
#define WOK_GEO_SUB_ERR_EID          7
#define WOK_GEO_DEPLOY_EID           10
#define WOK_GEO_DEPLOYED_EID         11
#define WOK_GEO_STOW_EID             12
#define WOK_GEO_STOWED_EID           13
#define WOK_GEO_ABORT_EID            14
#define WOK_GEO_FAULT_JAM_EID        20
#define WOK_GEO_FAULT_TIMEOUT_EID    21
#define WOK_GEO_FAULT_OVERCUR_EID    22
#define WOK_GEO_POD_EID              30   /* per-pod placement lifecycle */

#endif /* WOK_GEO_EVENTS_H */
