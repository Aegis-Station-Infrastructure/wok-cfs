/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/************************************************************************
 * WOK_SEQ — Event IDs
 ************************************************************************/
#ifndef WOK_SEQ_EVENTS_H
#define WOK_SEQ_EVENTS_H

#define WOK_SEQ_INIT_INF_EID         1
#define WOK_SEQ_NOOP_INF_EID         2
#define WOK_SEQ_RESET_INF_EID        3
#define WOK_SEQ_CMD_ERR_EID          4
#define WOK_SEQ_PIPE_ERR_EID         5
#define WOK_SEQ_LEN_ERR_EID          6
#define WOK_SEQ_SUB_ERR_EID          7
#define WOK_SEQ_TLM_LEN_ERR_EID      8   /* child TLM length mismatch    */
#define WOK_SEQ_STATE_CHANGE_EID     10
#define WOK_SEQ_START_EID            11
#define WOK_SEQ_ABORT_EID            12
#define WOK_SEQ_RESUME_EID           13
#define WOK_SEQ_CYCLE_COMPLETE_EID   14  /* all waypoints complete       */
#define WOK_SEQ_MODE_SET_EID         15  /* SET_MODE accepted            */
#define WOK_SEQ_PLACEMENT_SET_EID    16  /* SET_PLACEMENT accepted       */
#define WOK_SEQ_WP_LOADED_EID        17  /* LOAD_WAYPOINT accepted       */
#define WOK_SEQ_WP_COUNT_EID         18  /* SET_WP_COUNT accepted        */
#define WOK_SEQ_RELEASE_EID          19  /* RELEASE accepted             */
#define WOK_SEQ_FAULT_EID            20
#define WOK_SEQ_SAFE_ENTRY_EID       21  /* fault -> SAFE                */
#define WOK_SEQ_INHIBIT_EID          22
#define WOK_SEQ_HOLD_ENTRY_EID       30  /* recalibration hold entered   */
#define WOK_SEQ_HOLD_RELEASED_EID    31
#define WOK_SEQ_HOLD_TIMEOUT_EID     32
#define WOK_SEQ_SITE_GEOM_EID        33  /* site pose snapshot at DEPLOY */
#define WOK_SEQ_ARRAY_PLACED_EID     34
#define WOK_SEQ_ARRAY_VERIFIED_EID   35
#define WOK_SEQ_ARRAY_RECAPTURED_EID 36
#define WOK_SEQ_POD_RESEAT_EID       37  /* targeted pod reseat (VERIFY) */
#define WOK_SEQ_PARTIAL_ARRAY_EID    38  /* below partial-array minimum  */
#define WOK_SEQ_SOURCE_FIRED_EID     40
#define WOK_SEQ_SOURCE_TIMEOUT_EID   41
#define WOK_SEQ_TEST_SHOT_TIMEOUT_EID 42
#define WOK_SEQ_RECORD_EID           43  /* strike record complete       */
#define WOK_SEQ_PLATE_RESEAT_EID     44  /* plate coupling low, re-shot  */
#define WOK_SEQ_PLATE_PODSIDE_EID    45  /* low ratio attributed to pod  */
#define WOK_SEQ_PLATE_EXHAUSTED_EID  46  /* reseat budget spent          */
#define WOK_SEQ_ICE_DRILL_EID        50  /* ice flagged -> DRILL         */
#define WOK_SEQ_ICE_NO_DRILL_EID     51  /* ice flagged, not drill-flagged */
#define WOK_SEQ_DRILL_BATT_SKIP_EID  52  /* battery too low to drill     */
#define WOK_SEQ_DRILL_FAULT_EID      53  /* drill fault -> e-stop + SAFE */
#define WOK_SEQ_DRILL_DONE_EID       54
#define WOK_SEQ_RETURN_TARGET_EID    55  /* return-to-best-site decision */
#define WOK_SEQ_TIEPOINT_UNMET_EID   56
#define WOK_SEQ_MISSION_COMPLETE_EID 57

#endif /* WOK_SEQ_EVENTS_H */
