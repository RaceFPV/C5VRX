#include "direct_gain_v3.h"
#include "direct_gain_v3_lut.h"

#include <string.h>

/* A Q4 cell represents the half-integer position n+0.5. The lookup is built
 * once; the 1 ms observer performs only table reads and histogram counts. */
static uint8_t s_power[256];
static uint8_t s_flags[256];
static bool s_lut_ready;

/* Direct Gain V5 = this Direct Gain V3 core plus the full-range no-carrier
 * target, the direct mode below and anti-hunt damping (menu name
 * "DIRECT GAIN V5"). The observer feeds one fresh RX descriptor per window
 * on a 200 us timer cadence, so one window is ~0.2 ms.
 *
 * Direct mode: act on the first window outside the healthy band and jump
 * straight to the predicted destination. The healthy band stays a strict
 * zero-write zone and the per-transition hysteresis is unchanged, so a
 * steady carrier still gets no gain writes (no per-line pumping). */
#define DG3_STABLE_WINDOWS 1u   /* was 3 */
#define DG3_HIGH_WINDOWS   1u   /* was 2 */
#define DG3_WEAK_WINDOWS   1u   /* was 4 */
/* V5 anti-hunt. Direct mode must not chase a level that dithers across a
 * band edge (fast fades, antenna nulls). Two direction reversals of
 * consecutive writes within DG3_REVERSAL_US each arm a DG3_DAMP_US period in
 * which an out-of-band decision needs DG3_DAMPED_WINDOWS windows. Saturation
 * is never damped: it keeps the immediate emergency path. */
#define DG3_REVERSAL_US     20000u
#define DG3_REVERSALS_ARM   2u
#define DG3_DAMP_US        200000u
#define DG3_DAMPED_WINDOWS  8u   /* ~1.6 ms at the 200 us cadence */

static int clamp_i(int value, int low, int high)
{
    return value < low ? low : value > high ? high : value;
}

static int abs_i(int value) { return value < 0 ? -value : value; }

/* L0 is the V5 band (radius ~3.6-5.7 codes, no rail). L1 and L2 raise the
 * held envelope toward the 4-bit rail (L2 radius ~5.3-7.2, up to ~30 % rail
 * samples at the top of the band) for finer Phase8 angle quantization. */
const dg3_level_t dg3_levels[DG3_LEVELS] = {
    {13, 32, 17, 27, 65,  20, 53,  72,  20, 100,  95},
    {20, 42, 26, 36, 80, 150, 66,  88, 150, 350, 100},
    {28, 52, 34, 46, 95, 350, 80, 100, 350, 600, 108},
};

#define DG3_DEFAULT_LEVEL 0u

static const dg3_level_t *lv(const direct_gain_v3_t *v3)
{
    return &dg3_levels[v3->level < DG3_LEVELS ? v3->level : 0u];
}

void direct_gain_v3_set_level(direct_gain_v3_t *v3, uint8_t level)
{
    if (!v3) return;
    v3->level = level < DG3_LEVELS ? level : (uint8_t)(DG3_LEVELS - 1u);
    v3->high_windows = v3->weak_windows = 0;
    v3->last_direction = 0;
}

bool direct_gain_v3_saturated(uint8_t level, const dg3_observation_t *o)
{
    const dg3_level_t *l = &dg3_levels[level < DG3_LEVELS ? level : 0u];
    return o->clip_pm >= l->sat_clip_pm || o->p95 >= l->sat_p95;
}

static void prepare_lut(void)
{
    if (s_lut_ready) return;
    for (unsigned raw = 0; raw < 256; ++raw) {
        int q = (int8_t)((raw & 15u) << 4) >> 4;
        int i = (int8_t)(raw & 240u) >> 4;
        int ci = 2 * i + 1, cq = 2 * q + 1;
        int power = (ci * ci + cq * cq + 2) / 4;
        s_power[raw] = (uint8_t)power;
        s_flags[raw] = (power <= 4 ? 1u : 0u) |
                       ((i == -8 || i == 7 || q == -8 || q == 7) ? 2u : 0u);
    }
    s_lut_ready = true;
}

dg3_observation_t direct_gain_v3_measure(const uint8_t *samples, size_t bytes,
                                          const uint8_t phase8_lut[256],
                                          uint64_t observed_us)
{
    dg3_observation_t result = {.observed_us = observed_us};
    if (!samples || !bytes || !phase8_lut) return result;
    prepare_lut();
    uint16_t hist[114] = {0};
    unsigned origin = 0, clip = 0, coherent = 0;
    uint8_t prior_phase = 0;
    for (size_t n = 0; n < bytes; ++n) {
        uint8_t raw = samples[n];
        uint8_t power = s_power[raw];
        ++hist[power];
        origin += s_flags[raw] & 1u;
        clip += (s_flags[raw] >> 1) & 1u;
        uint8_t phase = phase8_lut[raw];
        if (n && power >= 8) {
            int delta = ((int)phase - (int)prior_phase + 128) & 255;
            delta -= 128;
            if (abs_i(delta) <= 32) ++coherent;
        }
        prior_phase = phase;
    }
    unsigned cumulative = 0;
    bool found_p50 = false, found_p90 = false;
    unsigned p50_rank = (unsigned)((bytes + 1u) / 2u);
    unsigned p90_rank = (unsigned)((bytes * 90u + 99u) / 100u);
    unsigned p95_rank = (unsigned)((bytes * 95u + 99u) / 100u);
    for (unsigned p = 0; p < 114u; ++p) {
        cumulative += hist[p];
        if (!found_p50 && cumulative >= p50_rank) {
            result.p50 = (uint8_t)p;
            found_p50 = true;
        }
        if (!found_p90 && cumulative >= p90_rank) {
            result.p90 = (uint8_t)p;
            found_p90 = true;
        }
        if (cumulative >= p95_rank) { result.p95 = (uint8_t)p; break; }
    }
    result.origin_pm = (uint16_t)(origin * 1000u / bytes);
    result.clip_pm = (uint16_t)(clip * 1000u / bytes);
    result.coherence = bytes > 1u ? (uint8_t)(coherent * 100u / (bytes - 1u)) : 0;
    return result;
}

static dg3_transition_t transition_kind(const direct_gain_v3_t *v3,
                                         uint8_t from, uint8_t to)
{
    const arc_gain_tuple_t *a = &v3->tuple[from], *b = &v3->tuple[to];
    if (a->rf_stage != b->rf_stage) return DG3_RF;
    if (a->bb_code != b->bb_code) return DG3_BB;
    return DG3_FINE;
}

static bool carrier(const dg3_observation_t *o)
{
    return o->coherence >= 55 && o->p50 >= 5 && o->origin_pm < 650;
}

static bool healthy(const direct_gain_v3_t *v3, const dg3_observation_t *o)
{
    const dg3_level_t *l = lv(v3);
    return carrier(o) && o->p50 >= l->lo && o->p50 <= l->hi &&
           o->p95 <= l->healthy_p95 && o->clip_pm < l->healthy_clip_pm &&
           o->origin_pm <= 250;
}

static bool valid_learning(const direct_gain_v3_t *v3,
                           const dg3_observation_t *o)
{
    const dg3_level_t *l = lv(v3);
    return carrier(o) && o->p50 >= 7 && o->p50 <= l->hi + 13 &&
           o->p95 < l->high_p95 + 8 && o->clip_pm < l->healthy_clip_pm &&
           o->origin_pm < 300;
}

static int power_db_q8(unsigned power)
{
    if (power < 1u) power = 1u;
    if (power > 113u) power = 113u;
    return s_dg3_power_db_q8[power];
}

/* Convert a Q10 power ratio to Q8 dB using an integer log2 iteration. */
static int ratio_db_q8(int ratio_q10)
{
    if (ratio_q10 < 1) ratio_q10 = 1;
    int64_t x = ((int64_t)ratio_q10 << 16) / 1024;
    int whole = 0;
    while (x < 65536) { x <<= 1; --whole; }
    while (x >= 131072) { x >>= 1; ++whole; }
    int fraction = 0;
    for (int bit = 7; bit >= 0; --bit) {
        x = (x * x) >> 16;
        if (x >= 131072) {
            x >>= 1;
            fraction |= 1 << bit;
        }
    }
    return ((whole * 256 + fraction) * 771 + 128) / 256;
}

void direct_gain_v3_reset(direct_gain_v3_t *v3, const arc_gain_table_t *table,
                          uint8_t current_gain, uint8_t survival_gain)
{
    if (!v3) return;
    memset(v3, 0, sizeof(*v3));
    v3->level = DG3_DEFAULT_LEVEL;   /* callers re-apply their level */
    prepare_lut();
    if (table && table->max_index >= 20u &&
        table->max_index <= ARC_VENDOR_GAIN_MAX) v3->table = *table;
    else arc_gain_table_from_bytes(&v3->table, NULL, ARC_VENDOR_GAIN_MAX);
    for (unsigned g = 0; g <= v3->table.max_index; ++g)
        (void)arc_gain_tuple_decode(&v3->table, (uint8_t)g, &v3->tuple[g]);
    v3->current_gain = (uint8_t)clamp_i(current_gain, 20, v3->table.max_index);
    v3->target_gain = v3->current_gain;
    v3->survival_gain = (uint8_t)clamp_i(survival_gain, 20, v3->table.max_index);
    v3->relative_power_q10[v3->current_gain] = 1024u;
    v3->confidence[v3->current_gain] = 1u;
    v3->uncertainty_pm[v3->current_gain] = 50u;
    v3->state = DG3_ACQUIRE;
}

/* An unknown bank or RF stage is never assigned a fabricated dB value.
 * Within one BB bank the physical Fine order supplies only a low-confidence
 * local prior. Measured tuple responses replace that prior. */
static bool predict(const direct_gain_v3_t *v3, uint8_t candidate,
                    int *ratio_q10, int *uncertainty_pm)
{
    uint8_t current = v3->current_gain;
    if (v3->confidence[current] && v3->confidence[candidate]) {
        int base = v3->relative_power_q10[current];
        if (!base) return false;
        *ratio_q10 = clamp_i((int)v3->relative_power_q10[candidate] * 1024 / base,
                             1, 8192);
        *uncertainty_pm = clamp_i(v3->uncertainty_pm[candidate] +
                                  v3->uncertainty_pm[current], 80, 900);
        return true;
    }
    if (transition_kind(v3, current, candidate) != DG3_FINE) return false;
    int steps = (int)v3->tuple[current].fine_code -
                (int)v3->tuple[candidate].fine_code;
    if (abs_i(steps) > 2) return false;
    int ratio = 1024;
    while (steps > 0) { ratio = ratio * 121 / 100; --steps; }
    while (steps < 0) { ratio = ratio * 100 / 121; ++steps; }
    *ratio_q10 = ratio;
    *uncertainty_pm = 300;
    return true;
}

static uint8_t adjacent_physical(const direct_gain_v3_t *v3, bool up)
{
    const arc_gain_tuple_t *current = &v3->tuple[v3->current_gain];
    uint8_t best = v3->current_gain;
    for (int distance = 1; distance <= 2; ++distance) {
        int wanted_fine = (int)current->fine_code +
                          (up ? -distance : distance);
        for (unsigned g = 20u; g <= v3->table.max_index; ++g) {
            const arc_gain_tuple_t *t = &v3->tuple[g];
            if (v3->bad_state[g] >= 3u) continue;
            if (v3->confidence[g] && v3->confidence[v3->current_gain]) {
                bool measured_up = v3->relative_power_q10[g] >
                                   v3->relative_power_q10[v3->current_gain];
                if (measured_up != up) continue;
            }
            if (t->rf_stage == current->rf_stage &&
                t->bb_code == current->bb_code &&
                t->fine_code == wanted_fine) return (uint8_t)g;
        }
    }
    /* Fine exhausted: enter the adjacent BB bank at its least aggressive
     * boundary state. This is a structural range move, not a dB prediction. */
    int wanted_bb = up ? (int)current->bb_code * 2 + 1 :
                         ((int)current->bb_code - 1) / 2;
    for (int distance = 0; distance < 6; ++distance) {
        int boundary_fine = up ? 5 - distance : distance;
        for (unsigned g = 20u; g <= v3->table.max_index; ++g) {
            const arc_gain_tuple_t *t = &v3->tuple[g];
            if (v3->bad_state[g] >= 3u) continue;
            if (t->rf_stage == current->rf_stage &&
                t->bb_code == wanted_bb &&
                t->fine_code == boundary_fine) return (uint8_t)g;
        }
    }
    /* A stage boundary is the last resort. Pick its weakest/strongest
     * available tuple and verify before any further physical write. */
    int wanted_rf = (int)current->rf_stage + (up ? 1 : -1);
    for (int distance = 0; distance < 6; ++distance) {
        int boundary_fine = up ? 5 - distance : distance;
        for (unsigned g = 20u; g <= v3->table.max_index; ++g) {
            const arc_gain_tuple_t *t = &v3->tuple[g];
            if (v3->bad_state[g] >= 3u) continue;
            if ((int)t->rf_stage == wanted_rf && t->bb_code == 1u &&
                t->fine_code == boundary_fine) return (uint8_t)g;
        }
    }
    return best;
}

static uint8_t emergency_drop(const direct_gain_v3_t *v3)
{
    const arc_gain_tuple_t *current = &v3->tuple[v3->current_gain];
    if (current->rf_stage > 0u) {
        for (unsigned g = 20u; g <= v3->table.max_index; ++g) {
            const arc_gain_tuple_t *t = &v3->tuple[g];
            if (t->rf_stage + 1u == current->rf_stage &&
                t->bb_code == 1u && t->fine_code == 5u) return (uint8_t)g;
        }
    }
    if (current->bb_code > 1u) {
        unsigned lower_bb = (current->bb_code - 1u) / 2u;
        for (unsigned g = 20u; g <= v3->table.max_index; ++g) {
            const arc_gain_tuple_t *t = &v3->tuple[g];
            if (t->rf_stage == current->rf_stage &&
                t->bb_code == lower_bb && t->fine_code == 5u)
                return (uint8_t)g;
        }
    }
    return adjacent_physical(v3, false);
}

static uint8_t select_destination(const direct_gain_v3_t *v3,
                                  const dg3_observation_t *o, bool up,
                                  int32_t desired_q8)
{
    uint8_t best = v3->current_gain;
    int best_class = 4, best_error = 99999, best_uncertainty = 999;
    int best_artifact = 999;
    for (unsigned g = 20u; g <= v3->table.max_index; ++g) {
        if (g == v3->current_gain) continue;
        if (v3->bad_state[g] >= 3u) continue;
        int ratio, uncertainty;
        if (!predict(v3, (uint8_t)g, &ratio, &uncertainty)) continue;
        int p50 = (int)o->p50 * ratio / 1024;
        int p95_worst = (int)o->p95 * ratio * (1000 + uncertainty) / 1024000;
        if (p50 < lv(v3)->lo || p50 > lv(v3)->hi ||
            p95_worst > lv(v3)->healthy_p95) continue;
        if (up && ratio <= 1024) continue;
        if (!up && ratio >= 1024) continue;
        int kind = transition_kind(v3, v3->current_gain, (uint8_t)g);
        int delta_q8 = ratio_db_q8(ratio);
        int hysteresis_q8 = kind == DG3_FINE ? 90 :
                            kind == DG3_BB ? 192 : 384;
        if ((up && desired_q8 < delta_q8 + hysteresis_q8) ||
            (!up && desired_q8 > delta_q8 - hysteresis_q8)) continue;
        int error = abs_i((int)desired_q8 - delta_q8);
        int artifact = v3->artifact_score[g];
        if (kind < best_class ||
            (kind == best_class && error < best_error) ||
            (kind == best_class && error == best_error &&
             uncertainty < best_uncertainty) ||
            (kind == best_class && error == best_error &&
             uncertainty == best_uncertainty && artifact < best_artifact)) {
            best = (uint8_t)g;
            best_class = kind;
            best_error = error;
            best_uncertainty = uncertainty;
            best_artifact = artifact;
        }
    }
    return best == v3->current_gain ? adjacent_physical(v3, up) : best;
}

static void learn_transition(direct_gain_v3_t *v3,
                             const dg3_observation_t *after)
{
    if (!valid_learning(v3, &v3->before) || !valid_learning(v3, after) ||
        abs_i((int)v3->before.coherence - (int)after->coherence) > 15) return;
    const dg3_observation_t *prior = &v3->before_previous;
    if (!prior->observed_us || prior->observed_us >= v3->before.observed_us ||
        v3->before.observed_us - prior->observed_us > 10000u ||
        abs_i((int)prior->p50 - (int)v3->before.p50) >
            clamp_i(v3->before.p50 / 4, 2, 10) ||
        abs_i((int)prior->coherence - (int)v3->before.coherence) > 12)
        return;
    uint8_t a = v3->prior_gain, b = v3->current_gain;
    uint32_t base = v3->relative_power_q10[a];
    if (!base) return;
    if (v3->transition == DG3_FINE) {
        int steps = abs_i((int)v3->tuple[a].fine_code -
                          (int)v3->tuple[b].fine_code);
        int high = 1024, low = 1024;
        while (steps-- > 0) { high = high * 18 / 10; low = low * 10 / 18; }
        int observed_ratio = (int)after->p50 * 1024 / v3->before.p50;
        if (observed_ratio < low || observed_ratio > high) return;
    }
    uint32_t estimate = base * after->p50 / v3->before.p50;
    estimate = (uint32_t)clamp_i((int)estimate, 1, 65535);
    if (!v3->confidence[b]) {
        v3->relative_power_q10[b] = (uint16_t)estimate;
        v3->uncertainty_pm[b] = 350u;
    } else {
        uint32_t old = v3->relative_power_q10[b];
        uint32_t residual_pm = old ?
            (uint32_t)(abs_i((int)old - (int)estimate) * 1000) / old :
            1000u;
        v3->uncertainty_pm[b] = (uint16_t)clamp_i(
            (3 * (int)v3->uncertainty_pm[b] + (int)residual_pm) / 4,
            60, 900);
        if (residual_pm > 450u) {
            if (v3->confidence[b] > 0u) --v3->confidence[b];
            return;
        }
        v3->relative_power_q10[b] = (uint16_t)((3u * old + estimate) / 4u);
    }
    if (v3->confidence[b] < 15u) ++v3->confidence[b];
    ++v3->learned;
}

static uint8_t start_write(direct_gain_v3_t *v3,
                           const dg3_observation_t *o,
                           const dg3_observation_t *prior, uint8_t target)
{
    if (target == v3->current_gain) return target;
    int8_t dir = target > v3->current_gain ? 1 : -1;
    uint64_t now = o->observed_us;
    if (v3->last_write_dir && dir != v3->last_write_dir &&
        now >= v3->dir_write_us && now - v3->dir_write_us < DG3_REVERSAL_US) {
        if (++v3->reversals >= DG3_REVERSALS_ARM) {
            v3->damp_until_us = now + DG3_DAMP_US;
            v3->reversals = 0;
            ++v3->damp_events;
        }
    } else if (now < v3->dir_write_us || now - v3->dir_write_us >= DG3_REVERSAL_US) {
        v3->reversals = 0;
    }
    v3->last_write_dir = dir;
    v3->dir_write_us = now;
    int ratio_q10 = 0, uncertainty_pm = 0;
    if (predict(v3, target, &ratio_q10, &uncertainty_pm))
        v3->virtual_gain_q8 -= ratio_db_q8(ratio_q10);
    else
        v3->virtual_gain_q8 = 0;
    v3->virtual_gain_q8 = clamp_i(v3->virtual_gain_q8,
                                  -12 * 256, 12 * 256);
    v3->before = *o;
    v3->before_previous = *prior;
    v3->prior_gain = v3->current_gain;
    v3->target_gain = target;
    v3->transition = transition_kind(v3, v3->current_gain, target);
    v3->current_gain = target;
    v3->state = DG3_SETTLE;
    v3->stable_windows = 0;
    v3->write_us = o->observed_us;
    ++v3->writes;
    return target;
}

uint8_t direct_gain_v3_tick(direct_gain_v3_t *v3,
                            const dg3_observation_t *o)
{
    if (!v3 || !o) return 0u;
    dg3_observation_t prior = v3->last_tracking;
    v3->last_tracking = *o;
    bool no_carrier = o->p50 <= 4 && o->origin_pm >= 650 && o->coherence < 20;
    bool saturated = direct_gain_v3_saturated(v3->level, o);
    if (no_carrier) {
        /* No usable carrier: listen at the table's maximum gain, not at the
         * survival gain (first index of the highest RF stage, G62). A weak
         * carrier is quantizer-starved at G62 and reads as no carrier, so the
         * old target was a trap that never explored G63..max (pre-q4-lab.md
         * far sweep; walk test 2026-09-29, where native AGC reached further).
         * Without a carrier the maximum still reads P50 1-3 / origin ~90 %,
         * so this state is stable; a strong carrier appearing here takes the
         * saturation path below on the next window. */
        v3->state = DG3_ACQUIRE;
        v3->stable_windows = 0;
        v3->high_windows = v3->weak_windows = 0;
        v3->virtual_gain_q8 = 0;
        v3->last_direction = 0;
        return start_write(v3, o, &prior, v3->table.max_index);
    }
    if (v3->state == DG3_SETTLE) {
        /* The freshness guard grows from prior settle measurements. The
         * signal still has to pass the multi-window stability check below. */
        uint64_t minimum_guard = v3->settle_us[v3->transition] > 400u ?
            (uint64_t)v3->settle_us[v3->transition] * 3u / 4u : 300u;
        if (o->observed_us <= v3->write_us ||
            o->observed_us - v3->write_us < minimum_guard)
            return v3->current_gain;
        if (v3->stable_windows &&
            abs_i((int)o->p50 - (int)v3->previous.p50) <= 2 &&
            abs_i((int)o->p95 - (int)v3->previous.p95) <= 4 &&
            abs_i((int)o->origin_pm - (int)v3->previous.origin_pm) <= 60 &&
            abs_i((int)o->coherence - (int)v3->previous.coherence) <= 12) {
            if (v3->stable_windows < DG3_STABLE_WINDOWS) ++v3->stable_windows;
        } else v3->stable_windows = 1u;
        v3->previous = *o;
        if (v3->stable_windows < DG3_STABLE_WINDOWS) return v3->current_gain;
        uint64_t elapsed = o->observed_us - v3->write_us;
        if (elapsed < 65535u) {
            uint16_t *settle = &v3->settle_us[v3->transition];
            *settle = *settle ? (uint16_t)((3u * *settle + elapsed) / 4u) :
                                (uint16_t)elapsed;
            uint16_t *tuple_settle = &v3->tuple_settle_us[v3->current_gain];
            *tuple_settle = *tuple_settle ?
                (uint16_t)((3u * *tuple_settle + elapsed) / 4u) :
                (uint16_t)elapsed;
        }
        int quality_drop = (int)v3->before.coherence - (int)o->coherence;
        int clipping_rise = (int)o->clip_pm - (int)v3->before.clip_pm;
        int artifact = clamp_i((quality_drop > 0 ? quality_drop * 4 : 0) +
                               (clipping_rise > 0 ? clipping_rise / 4 : 0),
                               0, 1000);
        uint16_t *score = &v3->artifact_score[v3->current_gain];
        *score = *score ? (uint16_t)((3u * *score + artifact) / 4u) :
                          (uint16_t)artifact;
        if (saturated || o->origin_pm >= 500 || o->coherence < 30) {
            if (v3->bad_state[v3->current_gain] < 15u)
                ++v3->bad_state[v3->current_gain];
        } else if (healthy(v3, o)) v3->bad_state[v3->current_gain] = 0u;
        learn_transition(v3, o);
        v3->state = DG3_VERIFY;
        ++v3->verified;
    }
    if (saturated) {
        ++v3->overloads;
        v3->high_windows = v3->weak_windows = 0;
        v3->virtual_gain_q8 = 0;
        return start_write(v3, o, &prior, emergency_drop(v3));
    }
    if (healthy(v3, o)) {
        v3->state = DG3_HOLD;
        v3->corrections = 0;
        v3->high_windows = v3->weak_windows = 0;
        v3->virtual_gain_q8 = 0;
        v3->last_direction = 0;
        ++v3->holds;
        return v3->current_gain;
    }
    /* Act as soon as the envelope leaves the healthy band, before it
     * reaches the grainy/collapsing region, not after noise appeared. */
    const dg3_level_t *l = lv(v3);
    bool high = o->p50 > l->hi || o->p90 >= l->high_p90 ||
                o->p95 > l->high_p95 || o->clip_pm >= l->high_clip_pm;
    bool weak = o->p50 < l->lo && carrier(o);
    /* Schmitt bands retain the previous direction through small envelope
     * fluctuations; they release only after crossing the inner boundary. */
    if (v3->last_direction == 2 &&
        (o->p50 > l->hi - 2 || o->p90 > l->high_p90 - 6 ||
         o->p95 > l->high_p95 - 7)) high = true;
    if (v3->last_direction == 1 && o->p50 < l->lo + 1 && carrier(o))
        weak = true;
    if (high) {
        v3->weak_windows = 0;
        v3->last_direction = 2;
        if (v3->high_windows < 255u) ++v3->high_windows;
    } else if (weak) {
        v3->high_windows = 0;
        v3->last_direction = 1;
        if (v3->weak_windows < 255u) ++v3->weak_windows;
    } else {
        v3->high_windows = v3->weak_windows = 0;
        v3->last_direction = 0;
        return v3->current_gain;
    }
    bool damped = o->observed_us < v3->damp_until_us;
    unsigned need_high = damped ? DG3_DAMPED_WINDOWS : DG3_HIGH_WINDOWS;
    unsigned need_weak = damped ? DG3_DAMPED_WINDOWS : DG3_WEAK_WINDOWS;
    if ((high && v3->high_windows < need_high) ||
        (weak && v3->weak_windows < need_weak)) return v3->current_gain;
    int target_power = weak ? l->target_up : l->target_down;
    int error_q8 = power_db_q8((unsigned)target_power) -
                   power_db_q8(o->p50);
    /* Direct: request the full relative correction in one step, both ways. */
    v3->virtual_gain_q8 = error_q8;
    v3->virtual_gain_q8 = clamp_i(v3->virtual_gain_q8,
                                  -12 * 256, 12 * 256);
    uint8_t target = select_destination(v3, o, weak,
                                        v3->virtual_gain_q8);
    if (target == v3->current_gain) return target;
    if (v3->state == DG3_VERIFY) ++v3->corrections;
    v3->high_windows = v3->weak_windows = 0;
    return start_write(v3, o, &prior, target);
}

void direct_gain_v3_sync_applied(direct_gain_v3_t *v3, uint8_t gain,
                                 uint64_t write_us)
{
    if (!v3) return;
    if (gain != v3->current_gain) {
        v3->current_gain = gain;
        v3->target_gain = gain;
        v3->stable_windows = 0;
    }
    v3->write_us = write_us;
    v3->state = DG3_SETTLE;
}
