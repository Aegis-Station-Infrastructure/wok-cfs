/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK_SEIS — Event IDs
 ************************************************************************/
#ifndef WOK_SEIS_EVENTS_H
#define WOK_SEIS_EVENTS_H

#define WOK_SEIS_INIT_INF_EID         1
#define WOK_SEIS_NOOP_INF_EID         2
#define WOK_SEIS_RESET_INF_EID        3
#define WOK_SEIS_CMD_ERR_EID          4
#define WOK_SEIS_PIPE_ERR_EID         5
#define WOK_SEIS_LEN_ERR_EID          6
#define WOK_SEIS_SUB_ERR_EID          7
#define WOK_SEIS_ACQ_START_EID        10
#define WOK_SEIS_ACQ_STOP_EID         11
#define WOK_SEIS_TRACE_FULL_EID       12   /* trace buffer filled, truncating */
#define WOK_SEIS_POD_GEOM_EID         13   /* SET_POD_GEOMETRY accepted       */
#define WOK_SEIS_PICKS_EID            14   /* first-break picks on the stack  */
#define WOK_SEIS_EVAL_EID             15   /* EVALUATE verdict                */
#define WOK_SEIS_QC_CLIP_EID          20
#define WOK_SEIS_QC_DEAD_CHAN_EID     21
#define WOK_SEIS_QC_DROPOUT_EID       22
#define WOK_SEIS_TRACE_WRITTEN_EID    30   /* seis_NNNN.wok archived          */
#define WOK_SEIS_SRC_CFG_EID          31   /* CONFIG_SOURCE accepted          */
#define WOK_SEIS_CORR_EID             32   /* correlation complete            */
#define WOK_SEIS_PLATE_RESEAT_EID     33   /* PLATE_RESEAT accepted           */
#define WOK_SEIS_DATA_WRITE_ERR_EID   34   /* any product write failed        */
#define WOK_SEIS_CORR_ERR_EID         35   /* correlation skipped / clamped   */
#define WOK_SEIS_DOWNLINK_EID         36   /* downlink started                */
#define WOK_SEIS_DOWNLINK_DONE_EID    37
#define WOK_SEIS_COUP_WRITTEN_EID     38   /* coup_NNNN.wok archived          */
#define WOK_SEIS_CORR_WRITTEN_EID     39   /* corr_NNNN.wok archived          */
#define WOK_SEIS_STACK_WRITTEN_EID    40   /* stack_NNNN.wok archived         */
#define WOK_SEIS_SRC_ARM_EID          41   /* arming (bank charging)          */
#define WOK_SEIS_SRC_ARM_ERR_EID      42
#define WOK_SEIS_SRC_FIRED_EID        43
#define WOK_SEIS_SRC_FIRE_ERR_EID     44
#define WOK_SEIS_SRC_RESET_EID        45
#define WOK_SEIS_SRC_CFG_ERR_EID      46   /* bad mode / HAL rejected config  */
#define WOK_SEIS_PLATE_LOW_EID        50   /* plate coupling below baseline   */
#define WOK_SEIS_PLATE_RESEAT_ERR_EID 51
#define WOK_SEIS_STACK_SKIP_EID       52   /* record kept out of the stack    */
#define WOK_SEIS_POD_SCORES_EID       53   /* test-shot per-pod scores        */
#define WOK_SEIS_SYNC_EID             54   /* test-shot sync residuals        */

#endif /* WOK_SEIS_EVENTS_H */
