#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "direct_gain_v3.h"

unsigned char phy_param[0x800];

static dg3_observation_t obs(int p50, int p95, int origin, int clip,
                              int coherence, uint64_t us)
{
    return (dg3_observation_t){
        .p50 = (uint8_t)p50, .p95 = (uint8_t)p95,
        .origin_pm = (uint16_t)origin, .clip_pm = (uint16_t)clip,
        .coherence = (uint8_t)coherence, .observed_us = us,
    };
}

int main(void)
{
    arc_gain_table_t table;
    arc_gain_table_from_bytes(&table, NULL, 81u);
    direct_gain_v3_t v3;
    direct_gain_v3_reset(&v3, &table, 35u, 62u);

    uint8_t phase[256] = {0};
    uint8_t raw[256];
    memset(raw, 0x00, sizeof(raw));
    dg3_observation_t m = direct_gain_v3_measure(raw, sizeof(raw), phase, 1u);
    assert(m.p50 == 1u && m.p95 == 1u && m.origin_pm == 1000u);
    memset(raw, 0x77, sizeof(raw));
    m = direct_gain_v3_measure(raw, sizeof(raw), phase, 2u);
    assert(m.p50 == 113u && m.p95 == 113u && m.clip_pm == 1000u);

    /* A broad healthy envelope is a strict zero-write zone. */
    for (uint64_t us = 1000u; us < 100000u; us += 1000u) {
        dg3_observation_t good = obs(22, 40, 0, 0, 99, us);
        assert(direct_gain_v3_tick(&v3, &good) == 35u);
    }
    assert(v3.writes == 0u && v3.state == DG3_HOLD);

    /* Direct mode: the first coherent, origin-heavy weak window already
     * selects a physical Fine move (no multi-window wait). */
    dg3_observation_t weak = obs(10, 17, 200, 0, 90, 199000u);
    uint8_t next = direct_gain_v3_tick(&v3, &weak);
    assert(next != 35u && v3.writes == 1u);
    /* Learning needs a stable pre-write pair; run the move once more from a
     * stable weak history to exercise it. */
    direct_gain_v3_reset(&v3, &table, 35u, 62u);
    dg3_observation_t pre = obs(22, 40, 0, 0, 99, 198000u);
    assert(direct_gain_v3_tick(&v3, &pre) == 35u);          /* in band */
    weak.observed_us = 199000u;
    weak.p50 = 22; weak.p95 = 40; weak.origin_pm = 0;       /* same as pre */
    assert(direct_gain_v3_tick(&v3, &weak) == 35u);          /* still in band */
    weak = obs(10, 17, 200, 0, 90, 200000u);
    v3.last_tracking = obs(10, 17, 200, 0, 90, 199500u);     /* stable weak prior */
    next = direct_gain_v3_tick(&v3, &weak);
    assert(next != 35u && v3.writes == 1u &&
           v3.tuple[next].rf_stage == v3.tuple[35].rf_stage &&
           v3.tuple[next].bb_code == v3.tuple[35].bb_code);
    direct_gain_v3_sync_applied(&v3, next, 200000u);
    /* One stable window after the physical settle guard verifies. */
    dg3_observation_t settled = obs(17, 28, 30, 0, 91, 200600u);
    assert(direct_gain_v3_tick(&v3, &settled) == next);
    assert(v3.state == DG3_HOLD && v3.verified == 1u && v3.learned == 1u);
    /* ...and a steady carrier in band stays write-free. */
    uint32_t steady_writes = v3.writes;
    for (unsigned k = 0; k < 200u; ++k) {
        settled.observed_us += 1000u;
        assert(direct_gain_v3_tick(&v3, &settled) == next);
    }
    assert(v3.writes == steady_writes);
    assert(v3.confidence[next] > 0u && v3.settle_us[DG3_FINE] > 0u);

    /* Poor phase with a healthy envelope is not a reason to pump gain. */
    dg3_observation_t multipath = obs(20, 35, 50, 0, 20, 300000u);
    assert(direct_gain_v3_tick(&v3, &multipath) == next);
    assert(v3.writes == 1u);

    /* Saturation drops sensitivity; carrier loss listens at maximum gain
     * (the table maximum, not the G62 survival trap). */
    dg3_observation_t clipped = obs(50, 105, 0, 200, 90, 400000u);
    uint8_t down = direct_gain_v3_tick(&v3, &clipped);
    assert(down != next && v3.overloads == 1u);
    dg3_observation_t lost = obs(1, 2, 950, 0, 0, 500000u);
    assert(direct_gain_v3_tick(&v3, &lost) == table.max_index);
    /* Still no carrier at maximum gain: stays there, no further writes. */
    uint32_t lost_writes = v3.writes;
    for (unsigned k = 0; k < 20u; ++k) {
        lost.observed_us += 5000u;
        assert(direct_gain_v3_tick(&v3, &lost) == table.max_index);
    }
    assert(v3.writes == lost_writes);
    /* A strong carrier appearing at maximum gain is dropped at once. */
    dg3_observation_t strong = obs(60, 110, 0, 300, 90, lost.observed_us + 5000u);
    assert(direct_gain_v3_tick(&v3, &strong) < table.max_index);

    /* Measured tuple response wins over numeric index order. G34 is marked
     * stronger than G35; G33 is the useful measured gain-down destination. */
    direct_gain_v3_reset(&v3, &table, 35u, 62u);
    v3.relative_power_q10[34] = 1200u;
    v3.relative_power_q10[33] = 620u;
    v3.confidence[34] = v3.confidence[33] = 3u;
    dg3_observation_t high = obs(40, 55, 0, 0, 95, 600000u);
    assert(direct_gain_v3_tick(&v3, &high) == 33u);          /* direct */

    /* At a Fine boundary, a known BB+Fine tuple is selected directly. */
    direct_gain_v3_reset(&v3, &table, 21u, 62u);
    assert(v3.tuple[21].rf_stage == v3.tuple[20].rf_stage);
    assert(v3.tuple[21].bb_code != v3.tuple[20].bb_code);
    v3.relative_power_q10[20] = 700u;
    v3.confidence[20] = 3u;
    high.observed_us += 100000u;
    assert(direct_gain_v3_tick(&v3, &high) == 20u);          /* direct */
    assert(v3.transition == DG3_BB);

    direct_gain_v3_reset(&v3, &table, 35u, 62u);
    v3.bad_state[34] = 3u;
    high.observed_us += 100000u;
    assert(direct_gain_v3_tick(&v3, &high) != 34u);          /* bad state skipped */

    /* A rapidly changing input around the write must not teach a false
     * receiver gain ratio, even if the post-write envelope stabilizes. */
    direct_gain_v3_reset(&v3, &table, 35u, 62u);
    v3.last_tracking = obs(10, 17, 200, 0, 90, 802000u);
    weak = obs(7, 13, 200, 0, 90, 803000u);                  /* input moved */
    next = direct_gain_v3_tick(&v3, &weak);
    assert(next != 35u);
    direct_gain_v3_sync_applied(&v3, next, weak.observed_us);
    settled = obs(17, 28, 30, 0, 91, 804000u);
    assert(direct_gain_v3_tick(&v3, &settled) == next);
    assert(v3.verified == 1u && v3.learned == 0u);

    /* V5 anti-hunt: a level dithering across both band edges makes the
     * writes reverse direction; after two quick reversals every out-of-band
     * decision needs two windows for 200 ms, then direct mode returns. */
    direct_gain_v3_reset(&v3, &table, 40u, 62u);
    uint64_t t = 2000000u;
    uint32_t w0 = v3.writes;
    for (unsigned k = 0; k < 40u; ++k) {
        dg3_observation_t o = (k & 1u) ? obs(8, 14, 100, 0, 90, t)
                                       : obs(40, 60, 0, 0, 95, t);
        uint8_t g = direct_gain_v3_tick(&v3, &o);
        direct_gain_v3_sync_applied(&v3, g, t);
        t += 1000u;
    }
    assert(v3.damp_events >= 1u);
    /* Undamped this would have written on (nearly) every window. */
    assert(v3.writes - w0 < 30u);
    /* After the damp period a single out-of-band window acts again. */
    t += 300000u;
    dg3_observation_t late = obs(8, 14, 100, 0, 90, t);
    uint32_t writes_before_late = v3.writes;
    (void)direct_gain_v3_tick(&v3, &late);   /* first window: direct again */
    assert(v3.writes == writes_before_late + 1u);
    /* Saturation is never damped. */
    v3.damp_until_us = late.observed_us + 1000000u;
    dg3_observation_t sat = obs(60, 110, 0, 300, 90, late.observed_us + 5000u);
    uint8_t pre_sat = v3.current_gain;
    assert(direct_gain_v3_tick(&v3, &sat) < pre_sat);

    /* Levels: L2 holds a strong envelope with ~25 % rail samples that L0
     * would correct, and still drops immediately on real overload. */
    {
        dg3_observation_t strong = obs(45, 80, 0, 250, 95, 9000000u);
        direct_gain_v3_reset(&v3, &table, 40u, 62u);
        direct_gain_v3_set_level(&v3, 2u);
        uint32_t w = v3.writes;
        assert(direct_gain_v3_tick(&v3, &strong) == 40u);
        assert(v3.writes == w && v3.state == DG3_HOLD);
        direct_gain_v3_reset(&v3, &table, 40u, 62u);
        assert(v3.level == 0u);
        strong.observed_us += 1000u;
        assert(direct_gain_v3_tick(&v3, &strong) < 40u);
        dg3_observation_t rail = obs(60, 90, 0, 150, 95, 0u);
        assert(direct_gain_v3_saturated(0u, &rail));
        assert(!direct_gain_v3_saturated(2u, &rail));
        rail.clip_pm = 700u;
        assert(direct_gain_v3_saturated(2u, &rail));
        for (unsigned i = 0; i < DG3_LEVELS; ++i) {
            const dg3_level_t *l = &dg3_levels[i];
            assert(l->lo < l->target_up && l->target_up < l->target_down &&
                   l->target_down < l->hi && l->healthy_p95 < l->high_p95 &&
                   l->healthy_clip_pm <= l->high_clip_pm &&
                   l->high_clip_pm < l->sat_clip_pm && l->high_p95 < l->sat_p95);
        }
    }

    puts("direct gain v3 core: OK");
    return 0;
}
