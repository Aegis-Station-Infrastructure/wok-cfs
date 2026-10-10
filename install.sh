#!/usr/bin/env bash
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
#
# install.sh — Install WOK apps into an existing cFS Draco tree
#
# Usage:
#   cd wok-cfs
#   ./install.sh ~/wok-cfs-build
#
# This copies WOK apps flat into the cFS apps/ directory, updates
# targets.cmake, the startup script, and the TO_LAB/SCH_LAB tables,
# and patches ci_lab/to_lab for multi-instance port offsets.
#

set -eo pipefail

if [ -z "$1" ]; then
    echo "Usage: ./install.sh <cfs_root>"
    echo "  Example: ./install.sh ~/wok-cfs-build"
    exit 1
fi

CFS="$1"
HERE="$(cd "$(dirname "$0")" && pwd)"

if [ ! -d "$CFS/cfe" ]; then
    echo "ERROR: $CFS doesn't look like a cFS tree (no cfe/ directory)"
    exit 1
fi

# WOK was developed and verified against the cFS v7.0.0 (Draco) bundle
# tag. Other revisions may well work, but the ci_lab/to_lab/sch_lab
# patches below are only tested there.
CFS_TAG="$(git -C "$CFS" describe --tags --always 2>/dev/null || echo unknown)"
if [ "$CFS_TAG" != "v7.0.0" ]; then
    echo ""
    echo "  WARNING: $CFS is at '$CFS_TAG', not the cFS v7.0.0 tag."
    echo "           WOK is verified on v7.0.0 only; continuing anyway."
    echo ""
fi

# cFS SB pipes are POSIX mqueues; the kernel default (10) clamps every
# pipe to depth 10 and starves the file-downlink path. See the Build &
# Install Guide, "Kernel message-queue depth".
MSG_MAX=$(cat /proc/sys/fs/mqueue/msg_max 2>/dev/null || echo 0)
if [ "$MSG_MAX" -lt 64 ]; then
    echo ""
    echo "  WARNING: fs.mqueue.msg_max is $MSG_MAX (cFS pipes clamp to this depth)."
    echo "           Bulk file downlink will drop chunks. Fix with:"
    echo "             sudo sysctl fs.mqueue.msg_max=128"
    echo ""
fi

echo ""
echo "  Installing WOK into: $CFS"
echo ""

# Copy apps flat into cFS apps/
for app in wok_sequencer wok_nav wok_geo wok_seis wok_drill wok_hk_health wok_common; do
    if [ -d "$HERE/apps/$app" ]; then
        cp -r "$HERE/apps/$app" "$CFS/apps/"
        echo "  OK apps/$app/"
    fi
done

# Update targets.cmake
TARGETS=""
for f in "$CFS/sample_defs/targets.cmake" "$CFS/targets.cmake"; do
    [ -f "$f" ] && TARGETS="$f" && break
done

if [ -n "$TARGETS" ]; then
    if grep -q "wok_sequencer" "$TARGETS" 2>/dev/null; then
        echo "  targets.cmake already has WOK"
    else
        cp "$TARGETS" "$TARGETS.bak"
        cat "$HERE/cfs_integration/targets_snippet.cmake" >> "$TARGETS"
        echo "  OK targets.cmake updated"
    fi
else
    echo "  WARNING: targets.cmake not found"
    echo "  Copy sample_defs first: cp -r $CFS/cfe/cmake/sample_defs $CFS/sample_defs"
fi

# Update startup script
STARTUP=""
for f in "$CFS/sample_defs/cpu1_cfe_es_startup.scr" "$CFS/sample_defs/cfe_es_startup.scr"; do
    [ -f "$f" ] && STARTUP="$f" && break
done

if [ -n "$STARTUP" ]; then
    if grep -q "wok_sequencer" "$STARTUP" 2>/dev/null; then
        echo "  Startup script already has WOK"
    else
        cp "$STARTUP" "$STARTUP.bak"
        # Insert WOK entries before the first ! line (the stock script
        # has a whole comment block of ! lines after the terminator)
        awk 'BEGIN { done = 0 }
             !done && /^!/ {
                 print "CFE_APP, wok_sequencer, WOK_Seq_AppMain,   WOK_SEQ,   80, 16384, 0x0, 0;"
                 print "CFE_APP, wok_seis,      WOK_Seis_AppMain,  WOK_SEIS,  85, 16384, 0x0, 0;"
                 print "CFE_APP, wok_nav,       WOK_Nav_AppMain,   WOK_NAV,   90, 16384, 0x0, 0;"
                 print "CFE_APP, wok_geo,       WOK_Geo_AppMain,   WOK_GEO,   90, 16384, 0x0, 0;"
                 print "CFE_APP, wok_drill,     WOK_Drill_AppMain, WOK_DRILL, 90, 16384, 0x0, 0;"
                 print "CFE_APP, wok_hk_health, WOK_Hk_AppMain,    WOK_HK,    95, 16384, 0x0, 0;"
                 done = 1
             }
             { print }' "$STARTUP.bak" > "$STARTUP"
        echo "  OK startup script updated"
    fi
else
    echo "  WARNING: startup script not found"
fi

# Update TO_LAB subscription table
if [ -f "$HERE/cfs_integration/to_lab_sub.c" ] && [ -d "$CFS/apps/to_lab/fsw/tables" ]; then
    # Keep the stock table as .bak (only on first install — a re-run
    # must not clobber it with the WOK version)
    if [ ! -f "$CFS/apps/to_lab/fsw/tables/to_lab_sub.c.bak" ]; then
        cp "$CFS/apps/to_lab/fsw/tables/to_lab_sub.c" "$CFS/apps/to_lab/fsw/tables/to_lab_sub.c.bak" 2>/dev/null || true
    fi
    cp "$HERE/cfs_integration/to_lab_sub.c" "$CFS/apps/to_lab/fsw/tables/"
    echo "  OK to_lab_sub.c updated"
fi

# Add the WOK SEND_HK wakeups to the SCH_LAB table (before the
# MSGID_RESERVED terminator of the .Config array). Idempotent.
SCHTBL="$CFS/apps/sch_lab/fsw/tables/sch_lab_table.c"
if [ -f "$SCHTBL" ]; then
    if grep -q "0x18D1" "$SCHTBL"; then
        echo "  sch_lab_table.c already has WOK SEND_HK entries"
    elif grep -q "{CFE_SB_MSGID_RESERVED, 0, 0}," "$SCHTBL"; then
        cp "$SCHTBL" "$SCHTBL.bak"
        sed -i '0,/{CFE_SB_MSGID_RESERVED, 0, 0},/s//\/* WOK Mission SEND_HK wakeups (every 4th tick) *\/\n        {CFE_SB_MSGID_WRAP_VALUE(0x18D1), 4, 0},  \/* SEQ SEND_HK   *\/\n        {CFE_SB_MSGID_WRAP_VALUE(0x18D3), 4, 0},  \/* NAV SEND_HK   *\/\n        {CFE_SB_MSGID_WRAP_VALUE(0x18D5), 4, 0},  \/* GEO SEND_HK   *\/\n        {CFE_SB_MSGID_WRAP_VALUE(0x18D7), 4, 0},  \/* SEIS SEND_HK  *\/\n        {CFE_SB_MSGID_WRAP_VALUE(0x18D9), 4, 0},  \/* HK SEND_HK    *\/\n        {CFE_SB_MSGID_WRAP_VALUE(0x18DB), 4, 0},  \/* DRILL SEND_HK *\/\n        {CFE_SB_MSGID_RESERVED, 0, 0},/' "$SCHTBL"
        echo "  OK sch_lab_table.c WOK SEND_HK entries added"
    else
        echo "  ERROR: sch_lab_table.c terminator not found — add the entries from"
        echo "         $HERE/cfs_integration/sch_lab_patch.txt by hand. Without them"
        echo "         no WOK app ever receives SEND_HK and the mission never leaves IDLE."
        exit 1
    fi
else
    echo "  ERROR: sch_lab table not found at $SCHTBL"
    exit 1
fi

# Patch ci_lab/to_lab to honour AEGIS_PORT_OFFSET so several WOK
# instances — a survey group — coexist on one host, each on its own
# port slot. Idempotent: skips if already applied.
if [ -f "$HERE/cfs_integration/port-offset.patch" ]; then
    if grep -q "AEGIS_PORT_OFFSET" "$CFS/apps/ci_lab/fsw/src/ci_lab_app.c" 2>/dev/null && \
       grep -q "AEGIS_PORT_OFFSET" "$CFS/apps/to_lab/fsw/src/to_lab_app.c" 2>/dev/null; then
        echo "  ci_lab/to_lab already honour AEGIS_PORT_OFFSET"
    elif patch -p1 -d "$CFS" -N --dry-run --silent < "$HERE/cfs_integration/port-offset.patch" >/dev/null 2>&1; then
        patch -p1 -d "$CFS" -N --no-backup-if-mismatch < "$HERE/cfs_integration/port-offset.patch" >/dev/null
        echo "  OK ci_lab/to_lab AEGIS_PORT_OFFSET patch applied"
    else
        echo "  ERROR: port-offset.patch did not apply cleanly — multi-instance"
        echo "         WOK groups need ci_lab/to_lab to honour AEGIS_PORT_OFFSET."
        echo "         Apply $HERE/cfs_integration/port-offset.patch by hand and re-run."
        exit 1
    fi
fi

echo ""
echo "  Done. Now build:"
echo "    cd $CFS"
echo "    make SIMULATION=native prep   # if first time"
echo "    make -j\$(nproc)"
echo "    make install"
echo ""
echo "  Then check the installed startup script:"
echo "    head -15 $CFS/build/exe/cpu1/cf/cfe_es_startup.scr"
echo ""
