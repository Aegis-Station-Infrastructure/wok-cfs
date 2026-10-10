#!/usr/bin/env bash
# Copyright 2026 Aegis Station Infrastructure LLC
# SPDX-License-Identifier: Apache-2.0
#
# wok_sim.sh — one-command sim stack control
#
#   tools/wok_sim.sh up      start core-cpu1 + WebSocket bridge + dashboard
#                            HTTP server, enable TO_LAB, print the URL
#   tools/wok_sim.sh down    stop everything started by `up`
#   tools/wok_sim.sh status  show what's running
#
# Assumes the standard dev layout: cFS tree at ~/wok-cfs-build built+installed
# (see WOK_Build_Install_Guide.md). Logs + PID files live in /tmp/wok_sim/.
# The dashboard needs no LAN config — the URL is printed for localhost and
# the machine's LAN address.

set -uo pipefail

CFS_EXE="${WOK_CFS_EXE:-$HOME/wok-cfs-build/build/exe/cpu1}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
GROUND="$HERE/ground"
RUN=/tmp/wok_sim
WS_PORT="${WOK_WS_PORT:-8765}"
HTTP_PORT="${WOK_HTTP_PORT:-8090}"

start_one() {  # name, workdir, command...
    local name=$1 dir=$2; shift 2
    if [ -f "$RUN/$name.pid" ] && kill -0 "$(cat "$RUN/$name.pid")" 2>/dev/null; then
        echo "  [=] $name already running (PID $(cat "$RUN/$name.pid"))"
        return
    fi
    # exec so no intermediate shell lingers holding our stdout (otherwise
    # `wok_sim.sh up | tee` blocks until the core exits) and so the PID
    # file names the real process.
    (cd "$dir" && exec setsid nohup "$@" > "$RUN/$name.log" 2>&1 < /dev/null) &
    echo $! > "$RUN/$name.pid"
    sleep 1
    if kill -0 "$(cat "$RUN/$name.pid")" 2>/dev/null; then
        echo "  [+] $name started (PID $(cat "$RUN/$name.pid"), log $RUN/$name.log)"
    else
        echo "  [!] $name FAILED — see $RUN/$name.log"
    fi
}

stop_one() {
    local name=$1
    if [ -f "$RUN/$name.pid" ]; then
        local pid; pid=$(cat "$RUN/$name.pid")
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null
            sleep 0.5
            kill -9 "$pid" 2>/dev/null || true
            echo "  [-] $name stopped (PID $pid)"
        fi
        rm -f "$RUN/$name.pid"
    fi
}

case "${1:-}" in
  up)
    mkdir -p "$RUN"
    MSG_MAX=$(cat /proc/sys/fs/mqueue/msg_max 2>/dev/null || echo 0)
    if [ "$MSG_MAX" -lt 64 ]; then
        echo "  [!] fs.mqueue.msg_max is $MSG_MAX — SB pipes clamp and the"
        echo "      file downlink will starve. In a separate terminal run:"
        echo "        sudo sysctl fs.mqueue.msg_max=128"
    fi
    [ -x "$CFS_EXE/core-cpu1" ] || { echo "  [!] $CFS_EXE/core-cpu1 not found (build first, or set WOK_CFS_EXE)"; exit 1; }

    start_one core   "$CFS_EXE" ./core-cpu1
    sleep 4   # let apps init before commanding
    start_one bridge "$GROUND"  python3 wok_bridge.py --ws-port "$WS_PORT"
    start_one http   "$GROUND"  python3 -m http.server "$HTTP_PORT" --directory "$GROUND"

    # TO_LAB enable so the console/scripts see TLM even before a dashboard
    # client connects (the bridge re-sends it on every client connect).
    python3 - <<'EOF'
import socket, struct
pkt = struct.pack(">HHH", 0x1880, 0xC001, len(b"127.0.0.1".ljust(16, b"\0")) + 1) \
    + struct.pack("BB", 6, 0) + b"127.0.0.1".ljust(16, b"\0")
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(pkt, ("127.0.0.1", 1234))
EOF
    echo "  [+] TO_LAB enabled"

    LAN_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
    echo ""
    echo "  Dashboard : http://127.0.0.1:${HTTP_PORT}/wok_index_html.html"
    [ -n "$LAN_IP" ] && \
    echo "        LAN : http://${LAN_IP}:${HTTP_PORT}/wok_index_html.html"
    echo "  Bridge    : ws://127.0.0.1:${WS_PORT}"
    echo "  Console   : python3 $GROUND/wok_console.py"
    echo ""
    echo "  NOTE: the sim battery drains in real time (~28 min to the"
    echo "  drill inhibit, ~53 min to SAFE) — cycle 'down'/'up' between"
    echo "  mission runs."
    ;;

  down)
    stop_one http
    stop_one bridge
    stop_one core
    pkill -x core-cpu1 2>/dev/null && echo "  [-] stray core-cpu1 killed" || true
    # Stray bridges/servers not started by us (match by /proc cmdline —
    # pkill -f would match this script's own shell)
    for p in $(pgrep -x python3); do
        case "$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)" in
            *wok_bridge.py*|*"http.server"*"$GROUND"*)
                kill "$p" 2>/dev/null && echo "  [-] stray python ($p) killed";;
        esac
    done
    echo "  sim stack down"
    ;;

  status)
    for name in core bridge http; do
        if [ -f "$RUN/$name.pid" ] && kill -0 "$(cat "$RUN/$name.pid")" 2>/dev/null; then
            echo "  $name: RUNNING (PID $(cat "$RUN/$name.pid"))"
        else
            echo "  $name: stopped"
        fi
    done
    ;;

  *)
    echo "usage: $0 up|down|status"
    exit 2
    ;;
esac
