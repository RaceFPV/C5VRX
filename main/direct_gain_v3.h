#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "arc_phy.h"

#define DG3_STATES (ARC_VENDOR_GAIN_MAX + 1u)

typedef enum {
    DG3_ACQUIRE = 0,
    DG3_HOLD,
    DG3_SETTLE,
    DG3_VERIFY,
} dg3_state_t;

typedef enum {
    DG3_FINE = 0,
    DG3_BB,
    DG3_RF,
} dg3_transition_t;

typedef struct {
    uint8_t p50, p90, p95;
    uint16_t origin_pm, clip_pm;
    uint8_t coherence;
    uint64_t observed_us;
} dg3_observation_t;

/* Operating level: where the Q4 envelope is held. Units are the core's
 * Q4 power (bucket-centre radius squared, 0..113; axis rail ~56). A higher
 * level uses more of the 4-bit IQ range (finer Phase8 angle steps) and
 * tolerates more rail samples, which Phase8 handles well because per-axis
 * clipping only bends the angle slightly. */
typedef struct {
    uint8_t lo, hi;                 /* healthy P50 band */
    uint8_t target_up, target_down; /* P50 aimed for after a correction */
    uint8_t healthy_p95;            /* also the prediction P95 ceiling */
    uint16_t healthy_clip_pm;
    uint8_t high_p90, high_p95;
    uint16_t high_clip_pm;
    uint16_t sat_clip_pm;           /* emergency drop */
    uint8_t sat_p95;
} dg3_level_t;

#define DG3_LEVELS 3u
extern const dg3_level_t dg3_levels[DG3_LEVELS];

typedef struct {
    arc_gain_table_t table;
    arc_gain_tuple_t tuple[DG3_STATES];
    uint16_t relative_power_q10[DG3_STATES];
    uint16_t tuple_settle_us[DG3_STATES];
    uint16_t uncertainty_pm[DG3_STATES];
    uint16_t artifact_score[DG3_STATES];
    uint8_t confidence[DG3_STATES];
    uint8_t bad_state[DG3_STATES];
    uint16_t settle_us[3];
    /* Continuous requested gain relative to the current physical tuple. */
    int32_t virtual_gain_q8;
    uint8_t current_gain, target_gain, survival_gain;
    uint8_t prior_gain, corrections;
    uint8_t high_windows, weak_windows;
    uint8_t last_direction;
    dg3_transition_t transition;
    dg3_state_t state;
    dg3_observation_t before, before_previous, previous, last_tracking;
    uint8_t stable_windows;
    uint64_t write_us;
    uint32_t writes, holds, verified, learned, overloads;
    /* V5 anti-hunt: direction reversals of consecutive writes. */
    int8_t last_write_dir;
    uint8_t reversals;
    uint64_t dir_write_us, damp_until_us;
    uint32_t damp_events;
    uint8_t level;   /* index into dg3_levels; reset selects L0 */
} direct_gain_v3_t;

/* Select the operating level (clamped); takes effect on the next window. */
void direct_gain_v3_set_level(direct_gain_v3_t *v3, uint8_t level);
/* Saturation test shared with the fast overload sentinel. */
bool direct_gain_v3_saturated(uint8_t level, const dg3_observation_t *o);

void direct_gain_v3_reset(direct_gain_v3_t *v3, const arc_gain_table_t *table,
                          uint8_t current_gain, uint8_t survival_gain);
dg3_observation_t direct_gain_v3_measure(const uint8_t *samples, size_t bytes,
                                          const uint8_t phase8_lut[256],
                                          uint64_t observed_us);
uint8_t direct_gain_v3_tick(direct_gain_v3_t *v3,
                            const dg3_observation_t *observation);
void direct_gain_v3_sync_applied(direct_gain_v3_t *v3, uint8_t gain,
                                 uint64_t write_us);
