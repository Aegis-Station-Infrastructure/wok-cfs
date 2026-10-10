/*
** WOK TO_LAB subscription table.
** Derived from the NASA cFS TO_LAB subscription table (Apache-2.0,
** https://github.com/nasa/to_lab); NASA-originated portions remain
** under that license. WOK additions (c) 2026 Aegis Station.
*/
#include "cfe_tbl_filedef.h"
#include "cfe_sb_api_typedefs.h"
#include "to_lab_tbl.h"
#include "cfe_msgids.h"
#include "to_lab_msgids.h"

/* Some cFS trees generate an empty cfe_sb_msgids.h wrapper — fall back
 * to the cFE default SB HK telemetry MID (0x0803). */
#ifndef CFE_SB_HK_TLM_MID
#define CFE_SB_HK_TLM_MID 0x0803
#endif

TO_LAB_Subs_t Subscriptions = {
    .Subs = {
        /* cFE core telemetry */
        {CFE_SB_MSGID_WRAP_VALUE(CFE_ES_HK_TLM_MID), {0, 0}, 4},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_EVS_HK_TLM_MID), {0, 0}, 4},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_SB_HK_TLM_MID), {0, 0}, 4},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_TBL_HK_TLM_MID), {0, 0}, 4},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_TIME_HK_TLM_MID), {0, 0}, 4},
        {CFE_SB_MSGID_WRAP_VALUE(CFE_EVS_LONG_EVENT_MSG_MID), {0, 0}, 32},
        {CFE_SB_MSGID_WRAP_VALUE(TO_LAB_HK_TLM_MID), {0, 0}, 4},

        /* WOK mission telemetry (0x08D0-0x08D9) */
        {CFE_SB_MSGID_WRAP_VALUE(0x08D0), {0, 0}, 4},  /* SEQ HK      */
        {CFE_SB_MSGID_WRAP_VALUE(0x08D1), {0, 0}, 4},  /* SEQ MODE    */
        {CFE_SB_MSGID_WRAP_VALUE(0x08D2), {0, 0}, 4},  /* NAV HK      */
        {CFE_SB_MSGID_WRAP_VALUE(0x08D3), {0, 0}, 4},  /* NAV STATUS  */
        {CFE_SB_MSGID_WRAP_VALUE(0x08D4), {0, 0}, 4},  /* GEO HK      */
        {CFE_SB_MSGID_WRAP_VALUE(0x08D5), {0, 0}, 4},  /* GEO STATUS  */
        {CFE_SB_MSGID_WRAP_VALUE(0x08D6), {0, 0}, 4},  /* SEIS HK     */
        {CFE_SB_MSGID_WRAP_VALUE(0x08D7), {0, 0}, 4},  /* SEIS STATUS */
        {CFE_SB_MSGID_WRAP_VALUE(0x08D8), {0, 0}, 4},  /* SEIS QLOOK  */
        {CFE_SB_MSGID_WRAP_VALUE(0x08D9), {0, 0}, 4},  /* HK HEALTH   */
        {CFE_SB_MSGID_WRAP_VALUE(0x08DA), {0, 0}, 4},  /* DRILL HK    */
        {CFE_SB_MSGID_WRAP_VALUE(0x08DB), {0, 0}, 4},  /* DRILL STATUS*/
        {CFE_SB_MSGID_WRAP_VALUE(0x08DC), {0, 0}, 32}, /* SEIS FILE CHUNKS */

        /* End marker */
        {CFE_SB_MSGID_RESERVED, {0, 0}, 0}
    }
};

CFE_TBL_FILEDEF(Subscriptions, TO_LAB.Subscriptions, TO Lab Sub Tbl, to_lab_sub.tbl)
