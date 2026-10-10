# WOK cFS — Build & Install Guide

**Document:** WOK-BI-001  
**Version:** 2.1  
**Date:** July 2026  
**Platform:** Ubuntu / Pop!_OS Linux, cFS Draco v7.0.0

---

## Prerequisites

- git
- cmake >= 3.14
- gcc, g++, make (apt install build-essential)
- python3 (for ground console)
- ~500MB disk space

Verify:

    git --version
    cmake --version
    gcc --version

### Kernel message-queue depth (required for file downlink)

Linux caps POSIX message queues at 10 messages by default, which silently
clamps every cFS software-bus pipe to depth 10. Routine telemetry alone
runs TO_LAB's pipe at that capacity, so bulk file-downlink chunks are
almost always the packet that gets dropped. Raise the limit:

    sudo sysctl fs.mqueue.msg_max=128

Make it persistent across reboots:

    echo "fs.mqueue.msg_max=128" | sudo tee /etc/sysctl.d/99-cfs.conf

Verify (no privileges needed):

    cat /proc/sys/fs/mqueue/msg_max   # should print 128

## 1. Clone cFS Draco

**Clone the `v7.0.0` tag** — that is the Draco release WOK is verified
against. A bare `git clone` gives you nasa/cFS's development branch,
which contains a different app set and is not what this guide assumes.

    git clone --branch v7.0.0 https://github.com/nasa/cFS.git ~/wok-cfs-build
    cd ~/wok-cfs-build
    git submodule init
    git submodule update --recursive

This pulls cFE, OSAL, PSP, CI_LAB, TO_LAB, SCH_LAB, and sample_app.

Verify:

    ls cfe/CMakeLists.txt osal/CMakeLists.txt psp/CMakeLists.txt
    ls apps/          # should show: ci_lab  sample_app  sch_lab  to_lab

## 2. Copy the Makefile

The top-level Makefile is a thin cmake wrapper shipped inside the cFE
submodule:

    cp ~/wok-cfs-build/cfe/cmake/Makefile.sample ~/wok-cfs-build/Makefile

The key targets are:

    make SIMULATION=native prep    # configure
    make                           # build
    make install                   # install to build/exe/cpu1/

## 3. Copy sample_defs

Draco keeps sample definitions inside cfe/cmake/:

    cp -r ~/wok-cfs-build/cfe/cmake/sample_defs ~/wok-cfs-build/sample_defs

## 4. Clone This Repo and Install WOK Apps

    git clone https://github.com/Aegis-Station-Infrastructure/wok-cfs.git ~/wok-cfs
    cd ~/wok-cfs
    ./install.sh ~/wok-cfs-build

The script copies all six WOK apps + wok_common into the cFS tree,
updates targets.cmake and the startup script, installs the TO_LAB
subscription table, adds the WOK SEND_HK entries to the SCH_LAB table,
and patches ci_lab/to_lab for multi-instance port offsets. **It covers
steps 6–9 below — if it ran clean, skip to step 10.** Steps 6–9 remain
as the manual/reference path.

If you prefer to copy the apps by hand instead of running install.sh:

    cp -r apps/wok_common     ~/wok-cfs-build/apps/
    cp -r apps/wok_sequencer  ~/wok-cfs-build/apps/
    cp -r apps/wok_nav        ~/wok-cfs-build/apps/
    cp -r apps/wok_geo        ~/wok-cfs-build/apps/
    cp -r apps/wok_seis       ~/wok-cfs-build/apps/
    cp -r apps/wok_hk_health  ~/wok-cfs-build/apps/
    cp -r apps/wok_drill      ~/wok-cfs-build/apps/

## 5. Directory Layout After Install

    ~/wok-cfs-build/apps/
      ci_lab/           (stock Draco)
      to_lab/           (stock Draco)
      sch_lab/          (stock Draco)
      sample_app/       (stock Draco)
      wok_common/       (HAL + shared headers + ground tools)
      wok_sequencer/    (WOK app — flat, same level as ci_lab)
      wok_nav/
      wok_geo/
      wok_seis/
      wok_drill/
      wok_hk_health/

This is the standard Draco layout — all apps are siblings in apps/.

## 6. Update targets.cmake *(skip if install.sh ran clean)*

Add WOK apps to the mission app list:

    cat >> ~/wok-cfs-build/sample_defs/targets.cmake << 'EOF'

    # >>> WOK MISSION APPS
    list(APPEND MISSION_GLOBAL_APPLIST
        wok_common
        wok_sequencer
        wok_nav
        wok_geo
        wok_seis
        wok_hk_health
        wok_drill
    )
    # <<< WOK MISSION APPS
    EOF

## 7. Update Startup Script *(skip if install.sh ran clean)*

Add WOK app entries **before** the first `!` line (the `!` is the end-of-file marker — anything after it is ignored):

Edit `~/wok-cfs-build/sample_defs/cpu1_cfe_es_startup.scr` and add before the `!`:

    CFE_APP, wok_sequencer, WOK_Seq_AppMain,   WOK_SEQ,   80, 16384, 0x0, 0;
    CFE_APP, wok_seis,      WOK_Seis_AppMain,  WOK_SEIS,  85, 16384, 0x0, 0;
    CFE_APP, wok_nav,       WOK_Nav_AppMain,    WOK_NAV,   90, 16384, 0x0, 0;
    CFE_APP, wok_geo,       WOK_Geo_AppMain,    WOK_GEO,   90, 16384, 0x0, 0;
    CFE_APP, wok_drill,     WOK_Drill_AppMain,  WOK_DRILL, 90, 16384, 0x0, 0;
    CFE_APP, wok_hk_health, WOK_Hk_AppMain,     WOK_HK,    95, 16384, 0x0, 0;

**Critical:** The `!` character marks end-of-file for the cFS startup parser. WOK entries must appear before it.

## 8. Update TO_LAB Subscription Table *(skip if install.sh ran clean)*

Edit `~/wok-cfs-build/apps/to_lab/fsw/tables/to_lab_sub.c` to subscribe TO_LAB to WOK telemetry MIDs. Add these entries inside the `.Subs` array before the `CFE_SB_MSGID_RESERVED` terminator:

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

The 0x08DC entry (depth 32) carries bulk file downlink — without it,
`seis.fetch` returns nothing. Or replace the file entirely with
`cfs_integration/to_lab_sub.c` from the WOK repo, which contains all of
the above (this is what install.sh does).

## 9. Update SCH_LAB Table *(skip if install.sh ran clean)*

Without these entries no SEND_HK requests reach the WOK apps, and the
sequencer's state machine — which runs only on the HK tick — never
advances: the rover will sit in IDLE forever after `seq.start`.

`install.sh` adds these for you. By hand: add WOK SEND_HK entries to `~/wok-cfs-build/apps/sch_lab/fsw/tables/sch_lab_table.c` inside the `.Config` array, before the `{CFE_SB_MSGID_RESERVED, 0, 0}` terminator:

    {CFE_SB_MSGID_WRAP_VALUE(0x18D1), 4, 0},  /* SEQ SEND_HK   */
    {CFE_SB_MSGID_WRAP_VALUE(0x18D3), 4, 0},  /* NAV SEND_HK   */
    {CFE_SB_MSGID_WRAP_VALUE(0x18D5), 4, 0},  /* GEO SEND_HK   */
    {CFE_SB_MSGID_WRAP_VALUE(0x18D7), 4, 0},  /* SEIS SEND_HK  */
    {CFE_SB_MSGID_WRAP_VALUE(0x18D9), 4, 0},  /* HK SEND_HK    */
    {CFE_SB_MSGID_WRAP_VALUE(0x18DB), 4, 0},  /* DRILL SEND_HK */

The `4` means every 4th scheduler tick. The stock Draco table runs at
`TickRate = 100` (10 ms ticks), so SEND_HK arrives at roughly 25 Hz —
that is expected; WOK sequencing timeouts are wall-clock, not
tick-counted, so the exact rate is not critical.

## 10. Build

    cd ~/wok-cfs-build
    make SIMULATION=native prep
    make -j$(nproc)
    make install

Verify all WOK apps built:

    ls ~/wok-cfs-build/build/exe/cpu1/cf/wok_*.so

Expected output:

    wok_sequencer.so  wok_nav.so  wok_geo.so  wok_seis.so  wok_hk_health.so  wok_drill.so

## 11. Verify Startup Script

After `make install`, check that WOK entries are in the installed script and before the `!`:

    head -20 ~/wok-cfs-build/build/exe/cpu1/cf/cfe_es_startup.scr

## 12. Run

    cd ~/wok-cfs-build/build/exe/cpu1
    ./core-cpu1

Look for init events from all 6 WOK apps:

    WOK_SEQ: App initialised (v2.3.0-seqid) — 7 waypoints loaded
    WOK_SEIS: App initialised (v5.0.0) — dual-mode EM source
    WOK_NAV: App initialised (relative-step model)
    WOK_GEO: App initialised (v2.0.0)
    WOK_DRILL: App initialised (v1.1.0) — stiffness log + evolved-gas
    WOK_HK: App initialised (v2.0.0)

(The "Startup Sync failed" line that follows is normal — see
Troubleshooting.)

## 13. Enable Telemetry Output

In a separate terminal, enable TO_LAB to send telemetry to localhost:

    ~/wok-cfs-build/build/exe/host/cmdUtil --pktid=0x1880 --cmdcode=6 --endian=LE --string="16:127.0.0.1"

Note: 0x1880 is TO_LAB's command MID (not a WOK MID — the WOK MIDs start at 0x18D0).

## 14. Launch Ground Console

    python3 ~/wok-cfs/ground/wok_console.py

Quick connectivity test:

    wok> allnoop
    wok> allhk
    wok> tlm

Full test suite:

    wok> test.all

You are now set up. To fly a mission, see the Quick Start in the
README and the [Operations Manual](WOK_Operations_Manual.md). For a
one-command sim stack (core + web dashboard), see `tools/wok_sim.sh`.

## 15. Clear Stale Shared Memory

If HAL init fails on startup (common after abnormal shutdown):

    ipcrm --all=shm

Then restart cFS. **Caution:** this removes *every* SysV shared-memory
segment you own, not just cFS's — don't run it while other
shared-memory applications you care about are running.

## Troubleshooting

**Rover never leaves IDLE after `seq.start`** — The SCH_LAB table
(step 9) is missing the WOK SEND_HK entries. The sequencer's state
machine runs only on the HK tick; with no SEND_HK requests it never
advances. install.sh does *not* patch SCH_LAB — step 9 is always
manual. Rebuild and reinstall after editing the table.

**Stale data products** — The sim writes archives to `/tmp/wok_data`,
and file numbering restarts with each boot, so products from a
previous run get mixed with (or overwritten by) the new mission's.
Clear it before a fresh mission: `rm -rf /tmp/wok_data`.

**"No subscribers for MsgId"** — Normal during startup for some cFE internal messages.

**"Startup Sync failed"** — Normal. The cfe_testcase app doesn't complete sync. WOK apps are fine.

**WOK apps not loading** — Check that WOK entries are before the `!` in `build/exe/cpu1/cf/cfe_es_startup.scr`.

**No telemetry** — Ensure TO_LAB enable command was sent. Start the console before sending `allhk`. TO_LAB sends to UDP port 2234.

**No drill telemetry** — Verify DRILL MIDs (0x08DA, 0x08DB) are in `to_lab_sub.c` and DRILL SEND_HK (0x18DB) is in `sch_lab_table.c`. Rebuild and reinstall after changes.

**MID collisions** — WOK MIDs must be at 0x18D0+ (cmd) and 0x08D0+ (tlm). If using old 0x1880 range, collisions with TO_LAB, CI_LAB, SAMPLE_APP will occur.

**Build errors about CFE_SB_MsgId_t** — App source must use `CFE_SB_ValueToMsgId(WOK_XXX_MID)` when calling `CFE_SB_Subscribe()`. Raw integer MIDs won't compile on Draco.
