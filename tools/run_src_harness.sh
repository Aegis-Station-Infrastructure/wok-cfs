#!/usr/bin/env bash
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
#
# run_src_harness.sh — build tools/wok_src_harness.c against the linux_sim
# HAL (no cFS needed) and run its fast, non-wall-clock verification modes.
# Exit status is non-zero if the compile or any mode fails.
#
#   tools/run_src_harness.sh            # tune bank pods sync
#   tools/run_src_harness.sh tune hal   # any modes listed in the harness header

set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
SIM="$REPO/apps/wok_common/hal/src/linux_sim"
OUT="${WOK_HARNESS_OUT:-$(mktemp -d)}"
MODES=("$@")
[ ${#MODES[@]} -eq 0 ] && MODES=(tune bank pods sync)

gcc -O2 -std=c99 "$REPO/tools/wok_src_harness.c" \
    "$SIM/wok_hal_sim_seis.c" "$SIM/wok_hal_sim_geology.c" \
    "$SIM/wok_hal_sim_coupling.c" "$SIM/wok_hal_sim_geo.c" \
    "$SIM/wok_hal_sim_power.c" \
    -I"$REPO/apps/wok_common/hal/inc" -I"$SIM" \
    -I"$REPO/apps/wok_common/msgs/inc" -lm -o "$OUT/harness"
echo "harness built: $OUT/harness"

rc=0
for m in "${MODES[@]}"; do
    echo "=== harness $m"
    if (cd "$OUT" && ./harness "$m"); then
        echo "=== $m: OK"
    else
        echo "=== $m: FAILED (exit $?)"; rc=1
    fi
done
exit $rc
