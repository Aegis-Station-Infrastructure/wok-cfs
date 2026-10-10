/*
 * Copyright 2026 Aegis Station Infrastructure LLC
 * SPDX-License-Identifier: Apache-2.0
 */
/*
** WOK HAL — Linux Sim Ground-Coupling Truth Model
** File: wok_common/hal/src/linux_sim/wok_hal_sim_coupling.h
**
** Single source of coupling truth for the Linux sim (WOK-CP-001):
** how well a geophone pod or the source coupling plate transfers ground
** motion, per surface position and per seating attempt. Exists so the
** CP reseat/retry logic and the degraded-array policy are exercisable
** in sim rather than validated by inspection — the same principle as
** the shared geology model: the sim must be able to reward correct
** behavior (a reseat really can improve a bad seat; a bad seat really
** does depress amplitudes).
**
** All functions are PURE (no state): each app links its own copy of the
** HAL lib, so cross-app consistency comes from determinism, not shared
** memory. "Randomness" is an integer hash of (position quantized to cm,
** attempt, salt) — the same seat attempt at the same spot always lands
** the same way, and attempt k+1 is an independent draw from the same
** position's seat-quality distribution. Coordinates are the mission
** start frame, as in wok_hal_sim_geology.h.
**
** Model:
**   Seat quality Q(x,y) in [0.05, 0.95] — the probability that a single
**   seating attempt lands a GOOD seat. Base 0.85 (regolith mostly seats
**   fine), minus Gaussian "unconsolidated fluff" patches (one placed on
**   the default survey line so array tests see a marginal pod), plus
**   clast-scale jitter on 0.25 m cells.
**
**   Pod coupling C(x,y,attempt) in (0,1] — amplitude transfer factor:
**   good seat -> 0.90..1.00; bad seat (perched on a clast / in fluff)
**   -> 0.30..0.75. Multiplies the pod's received signal amplitude.
**
**   Seating force F(x,y,attempt) [N] — the arm force-feedback PROXY
**   (WOK-CP-001): correlated with C but deliberately imperfect
**   (±12% instrument/emplacement jitter), so a force threshold catches
**   most bad seats while leaving the active test shot (§4.1) a real
**   role as arbiter.
**
**   Plate coupling P(x,y,attempt) in (0,1] — same construction for the
**   rover-pressed source plate, easier and tighter than a pod seat:
**   good -> 0.92..1.00 (probability Q+0.10), bad -> 0.55..0.80.
**   Multiplies transfer into the ground; the actuator-side measured
**   peak force is NOT affected, which is what makes the §4.4 ratio
**   (near-pod response / plate force) a real coupling metric.
**
** Sim-internal header: not part of the flight HAL contract.
*/

#ifndef WOK_HAL_SIM_COUPLING_H
#define WOK_HAL_SIM_COUPLING_H

#include <stdint.h>

/* Probability that one seating attempt at (x,y) lands a good seat. */
float WOK_SimCoupling_SeatQuality(float x_m, float y_m);

/* Pod amplitude-transfer factor for seat attempt `attempt` at (x,y). */
float WOK_SimCoupling_PodCoupling(float x_m, float y_m, uint8_t attempt);

/* Seating-force proxy (peak arm force, N) for the same attempt. */
float WOK_SimCoupling_SeatForceN(float x_m, float y_m, uint8_t attempt);

/* Source-plate transfer factor for plate seating `attempt` at (x,y). */
float WOK_SimCoupling_PlateCoupling(float x_m, float y_m, uint8_t attempt);

/* Per-pod sync-bias UNIT draw in [-1, +1] (CP-5, WOK-CP-001):
 * wireless pods carry their own clocks; each pod's t0 reference is
 * offset by a per-pod bias. Deterministic per pod position (a pod's
 * clock error doesn't change between the shots of a site); the sim
 * scales it by the WOK_SIM_SYNC_ERR_MS magnitude. */
float WOK_SimCoupling_PodSyncBias(float x_m, float y_m);

#endif /* WOK_HAL_SIM_COUPLING_H */
