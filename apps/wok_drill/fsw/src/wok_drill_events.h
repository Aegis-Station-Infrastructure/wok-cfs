/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK_DRILL — Event IDs
 ************************************************************************/
#ifndef WOK_DRILL_EVENTS_H
#define WOK_DRILL_EVENTS_H

#define WOK_DRILL_INIT_INF_EID          1
#define WOK_DRILL_NOOP_INF_EID          2
#define WOK_DRILL_RESET_INF_EID         3
#define WOK_DRILL_CMD_ERR_EID           4   /* unknown CC / rejected in this state */
#define WOK_DRILL_PIPE_ERR_EID          5
#define WOK_DRILL_LEN_ERR_EID           6
#define WOK_DRILL_INIT_ERR_EID          7
#define WOK_DRILL_MID_ERR_EID           8   /* unexpected MsgId on the pipe */
#define WOK_DRILL_HAL_ERR_EID           9   /* HAL call rejected a command  */
#define WOK_DRILL_DEPLOY_EID            10
#define WOK_DRILL_STOW_EID              11
#define WOK_DRILL_START_DRILL_EID       12
#define WOK_DRILL_STOP_DRILL_EID        13
#define WOK_DRILL_START_SAMPLE_EID      14
#define WOK_DRILL_ESTOP_EID             15
#define WOK_DRILL_ARM_READY_EID         16
#define WOK_DRILL_DRILL_DONE_EID        17
#define WOK_DRILL_SAMPLE_DONE_EID       18
#define WOK_DRILL_STOWED_EID            19
#define WOK_DRILL_LOG_WRITTEN_EID       20
#define WOK_DRILL_LOG_WRITE_ERR_EID     21
#define WOK_DRILL_FAULT_ARM_TIMEOUT_EID 30
#define WOK_DRILL_FAULT_OVERCUR_EID     31
#define WOK_DRILL_FAULT_STALL_EID       32
#define WOK_DRILL_FAULT_OVER_DEPTH_EID  33
#define WOK_DRILL_FAULT_THERMAL_EID     34
#define WOK_DRILL_FAULT_ENTRY_EID       35

#endif /* WOK_DRILL_EVENTS_H */
