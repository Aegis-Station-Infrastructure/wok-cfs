#!/usr/bin/env bash
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
#
# smoke_test.sh — boot core-cpu1 from a built cFS tree and confirm that all
# six WOK apps report "App initialised" on the EVS console port.
#
#   tools/smoke_test.sh <cfs>/build/exe/cpu1 [timeout_s]

set -uo pipefail
EXE="${1:?usage: smoke_test.sh <path/to/build/exe/cpu1> [timeout_s]}"
TIMEOUT="${2:-30}"
LOG="${WOK_SMOKE_LOG:-$(mktemp)}"
APPS=(WOK_SEQ WOK_SEIS WOK_NAV WOK_GEO WOK_DRILL WOK_HK)

[ -x "$EXE/core-cpu1" ] || { echo "core-cpu1 not found in $EXE"; exit 2; }
cd "$EXE"
./core-cpu1 > "$LOG" 2>&1 &
PID=$!
trap 'kill $PID 2>/dev/null; wait $PID 2>/dev/null' EXIT

ok=0
for ((i=0; i<TIMEOUT; i++)); do
    sleep 1
    ok=0
    for a in "${APPS[@]}"; do
        grep -q "$a: App initialised" "$LOG" && ok=$((ok+1))
    done
    [ $ok -eq ${#APPS[@]} ] && break
    kill -0 $PID 2>/dev/null || break
done

echo "--- WOK init events ($ok/${#APPS[@]}) after ${i}s, log: $LOG"
grep -E 'WOK_[A-Z]+: App initialised' "$LOG" || true
if [ $ok -ne ${#APPS[@]} ]; then
    echo "--- FAILED; last 40 log lines:"; tail -40 "$LOG"; exit 1
fi
echo "--- smoke test OK"
