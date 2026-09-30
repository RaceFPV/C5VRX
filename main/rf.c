/**
 * rf.c - ESP32-C5 Wi-Fi/PHY receive-only frontend initialization.
 *
 * Configures the RF frontend to receive at 5865 MHz (channel 173) / BW40
 * and routes MODEM_DIAG Q4/I4 to the PARLIO RX GPIO pins.
 *
 * Reference: Seamless Golden 16K (proven best live build).
 * Derived from C5VRX-2 wifi5.c -- stripped of all research/debug baggage.
 *
 * IMPORTANT: BW40 failure returns an error. NO BW20 fallback.
 */

#include "rf.h"
#include "native_analog_agc.h"
#include "native_agc_pace.h"
#include "freertos/FreeRTOS.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "esp_timer.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_phy_init.h"
#include "esp_rom_gpio.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "soc/gpio_sig_map.h"
#include "modem/modem_syscon_reg.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "heap_memory_layout.h"
#include "esp_rom_sys.h"
#include "esp_private/wifi_os_adapter.h"

/* Fixed receiver configuration -- not configurable at runtime. */
#define RF_CHANNEL_NUMBER   173u
#define RF_BANDWIDTH        WIFI_BW40

/* Runtime analog filter state; startup remains BW40. */
static bool s_analog_bw40 = true;

/* Issue #117/#119 native hardware AGC, the default receive gain owner.
 * Decided once per boot from NVS before PHY init: the vendor AGC cannot be
 * restored after phy_disable_agc()/phy_rfagc_disable(), so it is never
 * disabled instead. An explicit NVS value of 0 selects the firmware gain
 * controllers (Direct Gain V3 et al.) as a fallback. */
#define NATIVE_AGC_NVS_NAMESPACE "c5vrx"
#define NATIVE_AGC_NVS_KEY       "native_agc"
#define RX_AGC_CTRL_REG          0x600A7030u
static bool s_native_agc;
static bool analog_agc_cancel(void);
static void analog_agc_transition_begin(void);
static void analog_agc_transition_end(void);
static unsigned s_analog_rf_transitions;
/* #121 native AGC start gain. 0x600A7094[8:2] is the gain each native
 * acquisition starts from (vendor: max-1 = 82, ESPARGOS esp-sdr semantics).
 * The loop re-acquires several times per video line, so every restart
 * swings from 82 down to the operating point (~40-65). A lower start keeps
 * those swings short. This is a one-time native-AGC policy setting, not a
 * CPU gain decision; 0 keeps the vendor value. Persisted in NVS, re-applied
 * after boot and after every channel retune (vendor paths rewrite it). */
#define NATIVE_INITGAIN_NVS_KEY "agc_init"
#define NATIVE_INITGAIN_REG     0x600A7094u
#define NATIVE_INITGAIN_MASK    0x000001FCu
#define NATIVE_INITGAIN_MIN     40u
static uint8_t s_native_initgain;
static uint8_t s_native_initgain_vendor;

/* #121 section 9 lab A/B: PHY PLL/RXCAL tracking stays stopped unless this
 * boot was explicitly requested with NVS c5vrx/pll_track = 1. */
#define PLL_TRACK_NVS_KEY "pll_track"
static bool s_pll_track;
static volatile uint32_t s_native_agc_blocked_writes;

/* MAC TX queue hardware registers (IDF-pinned: ESP32-C5, IDF 6.0.x).
 * Identical to C5VRX-2 wifi5.c proven addresses. */
#define REG32(a)         (*(volatile uint32_t *)(uintptr_t)(a))
#define MAC_TXQ0_CONF    0x600a4d6cu
#define MAC_TXQ_STRIDE   0x10u
#define MAC_TXQ_ENABLE   0x80000000u
#define MAC_TXQ_COUNT    5u

/* RX digital filter register (0x600A0430[21:18]) */
#define RX_FILTER_REG       0x600A0430u
#define RX_FILTER_SHIFT     18u
#define RX_FILTER_MASK      (0xFu << RX_FILTER_SHIFT)
#define ADC_RATE_REG        0x600A0448u
#define RX_GAIN_STATUS_REG  0x600A702Cu
#define RX_IQ_CORR_REG      0x600A0438u
#define ADC_RATE_SEL_MASK   0x3u

/* Continuous modem front-end un-gating registers.
 * Required to keep the C5 ADC / modem continuously clocking 80 MS/s IQ
 * into MODEM_DIAG when no 802.11 Wi-Fi packets are present. */
#define DUMP_CTRL       0x600a9004u
#define DUMP_PTR_MODE   0x600a9008u
#define DUMP_FORMAT     0x600a9018u
#define FE_PATH         0x600a20b4u
#define FE_ENABLE       0x600a0800u
#define SOURCE_CTRL     0x600a08ccu
#define SOURCE_MUX      0x600a70b8u
#define MODEM_CLOCK     0x600a9c04u
#define CTRL_ENABLE     0x80000000u
#define CTRL_DUMP_FIRST 0x00020000u
#define TX_START_SELECT 0x00060000u
#define SELECTOR_MASK   0x01fe0000u
#define HP_SRAM_USAGE   0x60095004u

/* MODEM_DIAG lane mapping: Q[9:6] on DIAG[6:9], I[9:6] on DIAG[16:19].
 * GPIO mapping correlated against physical ESP32-C5 hardware captures.
 * These GPIOs connect to the PARLIO RX data_gpio_nums[] array (same order). */
static const gpio_num_t s_iq_pins[8] = {
    GPIO_NUM_1, GPIO_NUM_0, GPIO_NUM_25, GPIO_NUM_7,   /* Q[9:6] */
    GPIO_NUM_10, GPIO_NUM_5, GPIO_NUM_3, GPIO_NUM_4,   /* I[9:6] */
};
static const uint8_t s_iq_diag[8] = {
    6u, 7u, 8u, 9u,     /* DIAG[6:9]  = Q[9:6] */
    16u, 17u, 18u, 19u, /* DIAG[16:19] = I[9:6] */
};
/* Issue #123 fine lanes: nibble = {sign bit 9, bits 7, 6, 5}. Inside
 * |x| < 256 per axis this equals the contiguous [8:5] window: twice the
 * resolution, same signed-nibble meaning, so the Phase8 LUT is unchanged.
 * Outside it folds (+288 reads as +32). DIAG[5]/[15] were proven bit-exact
 * against the RF dump with 6-bit reference alignment (tools/analyze_all_diag.py
 * passes CAND_123 and I_BUS_4_9). Only the P8 FINE demodulator selects them.
 * Native AGC overshoots the window only while it re-acquires (the restart
 * transients, ~6% of samples), which are saturated garbage on the coarse
 * lanes too; trapped samples (radius ~2.5 coarse cells) stay inside. */
static const uint8_t s_iq_diag_fine[8] = {
    5u, 6u, 7u, 9u,
    15u, 16u, 17u, 19u,
};
static bool s_iq_fine;

/* Internal vendor symbol -- globally exported by the pinned IDF 6.0.x
 * pp (protocol processing) library for ESP32-C5. */
extern int lmac_stop_hw_txq(void);

static const char *TAG = "c5vrx3_rf";
static arc_gain_table_t s_arc_gain_table;
static rf_phy_snapshot_t s_arc_receive_tuple;
static uint32_t s_arc_generation;
static uint8_t s_current_gain_val = 52u;

static void arc_capture_vendor_state(void);

/**
 * Disable all 5 LMAC MAC TX hardware queues.
 * Called once after Wi-Fi start to ensure the frontend is receive-only.
 */
static esp_err_t lock_rx_only(void)
{
    (void)lmac_stop_hw_txq();
    for (unsigned q = 0u; q < MAC_TXQ_COUNT; ++q)
        REG32(MAC_TXQ0_CONF - q * MAC_TXQ_STRIDE) &= ~MAC_TXQ_ENABLE;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    /* Verify all queues are disabled. */
    for (unsigned q = 0u; q < MAC_TXQ_COUNT; ++q) {
        if ((REG32(MAC_TXQ0_CONF - q * MAC_TXQ_STRIDE) & MAC_TXQ_ENABLE) != 0u)
            return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

/**
 * Route MODEM_DIAG DIAG[6:9] and DIAG[16:19] to the GPIO pins used by
 * PARLIO RX. Called after Wi-Fi initializes the PHY clock domain.
 */
static esp_err_t route_modem_iq(void)
{
    uint64_t mask = 0u;
    for (unsigned lane = 0u; lane < 8u; ++lane)
        mask |= 1ULL << s_iq_pins[lane];
    const gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;
    for (unsigned lane = 0u; lane < 8u; ++lane) {
        esp_rom_gpio_connect_out_signal(s_iq_pins[lane],
                                        MODEM_DIAG0_IDX + s_iq_diag[lane],
                                        false, false);
    }
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    return ESP_OK;
}

/* LAB: baseband packet AGC off while the RF-side AGC (705C, never touched
 * in native mode) stays on. The packet AGC is what re-acquires every
 * 25-50 us; whether the RF AGC alone keeps gain in range is exactly what
 * this measures. Reversible: phy_enable_agc() clears 7030[29] and strobes
 * 702C[23]. RAM only; a retune or reboot restores the packet AGC. */
static bool s_bb_agc_off;

/* Native AGC policy registers written by phy_agc_reg_init_new() and
 * bb_agc_reg_update() (disassembly of the pinned libphy, 2026-09-30).
 * Semantics are unknown, so each entry is one change against the vendor
 * value, persisted as an index (NVS c5vrx/agc_tune) and applied at boot and
 * after every retune. Index 0 is the vendor configuration (no write).
 * tools/agc_tune_sweep.py steps through them with serial '8' and measures
 * each boot's native acquisitions per sample (probe AGC_WORDS). */
typedef struct {
    const char *name;
    uint32_t reg;
    uint8_t shift, width;   /* width 0 = whole word */
    uint32_t raw;
} agc_tune_t;

static const agc_tune_t s_agc_tunes[] = {
    {"vendor",            0u,          0u,  0u, 0u},
    /* 7128[31:24] = 0xD2 (-46) at init: target level candidate. */
    {"7128_target_-40",   0x600A7128u, 24u, 8u, 0xD8u},
    {"7128_target_-34",   0x600A7128u, 24u, 8u, 0xDEu},
    {"7128_target_-52",   0x600A7128u, 24u, 8u, 0xCCu},
    /* phy_set_rx_comp_new(): -30/-32 dB into 702C[7:0] and 70A0[31:24]. */
    {"702C_comp_-24",     0x600A702Cu, 0u,  8u, 0xE8u},
    {"702C_comp_-36",     0x600A702Cu, 0u,  8u, 0xDCu},
    {"70A0_comp_-24",     0x600A70A0u, 24u, 8u, 0xE8u},
    {"70A0_comp_-36",     0x600A70A0u, 24u, 8u, 0xDCu},
    /* 7034[30:24] = 10, 71B0[27:21] = 30, 7158[6:0] = 13 at init. */
    {"7034_5",            0x600A7034u, 24u, 7u, 5u},
    {"7034_20",           0x600A7034u, 24u, 7u, 20u},
    {"71B0_15",           0x600A71B0u, 21u, 7u, 15u},
    {"71B0_60",           0x600A71B0u, 21u, 7u, 60u},
    {"7158_6",            0x600A7158u, 0u,  7u, 6u},
    {"7158_26",           0x600A7158u, 0u,  7u, 26u},
    /* 8028 = 0xC0403020: four stacked level thresholds (bb_agc_reg_update). */
    {"8028_half",         0x600A8028u, 0u,  0u, 0x60201810u},
    {"8028_double",       0x600A8028u, 0u,  0u, 0xFF806040u},
};
#define AGC_TUNE_COUNT (sizeof(s_agc_tunes) / sizeof(s_agc_tunes[0]))
#define AGC_TUNE_NVS_KEY "agc_tune"
static uint8_t s_agc_tune;
static uint32_t s_agc_tune_before, s_agc_tune_after;

static uint8_t agc_tune_load(void)
{
    nvs_handle_t handle;
    uint8_t value = 0;
    if (nvs_open(NATIVE_AGC_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return 0;
    if (nvs_get_u8(handle, AGC_TUNE_NVS_KEY, &value) != ESP_OK) value = 0;
    nvs_close(handle);
    return value < AGC_TUNE_COUNT ? value : 0u;
}

static void agc_tune_apply(void)
{
    const agc_tune_t *t = &s_agc_tunes[s_agc_tune];
#if !CONFIG_C5VRX_PHY_PHASE_TAP_PROBE
    return;   /* sweep candidates are lab-only: production runs vendor AGC */
#endif
    if (!s_native_agc || !t->reg) return;
    uint32_t v = REG32(t->reg);
    s_agc_tune_before = v;
    if (!t->width) {
        v = t->raw;
    } else {
        uint32_t mask = ((1u << t->width) - 1u) << t->shift;
        v = (v & ~mask) | ((t->raw << t->shift) & mask);
    }
    REG32(t->reg) = v;
    s_agc_tune_after = REG32(t->reg);
}

void rf_agc_tune_report(void)
{
    const agc_tune_t *t = &s_agc_tunes[s_agc_tune];
    printf("AGC_TUNE idx=%u count=%u name=%s reg=0x%08lx before=0x%08lx after=0x%08lx\n",
           s_agc_tune, (unsigned)AGC_TUNE_COUNT, t->name, (unsigned long)t->reg,
           (unsigned long)s_agc_tune_before, (unsigned long)s_agc_tune_after);
}

uint8_t rf_agc_tune_index(void)
{
    return s_agc_tune;
}

esp_err_t rf_agc_tune_select_next(void)
{
    uint8_t next = (uint8_t)((s_agc_tune + 1u) % AGC_TUNE_COUNT);
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NATIVE_AGC_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_u8(handle, AGC_TUNE_NVS_KEY, next);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

/* Native AGC level offset: a signed dB field of the vendor AGC, shifted
 * relative to its vendor value. The hardware AGC keeps every gain decision;
 * this only moves the level it settles on (agc_offset.h calibrates it).
 * Candidates from the libphy disassembly; which one is the settling target
 * is measured by tools/agc_tune_sweep.py. Re-applied at boot and retune. */
typedef struct { const char *name; uint32_t reg; uint8_t shift; } agc_offset_field_t;
static const agc_offset_field_t s_agc_offset_fields[] = {
    {"7128_target", 0x600A7128u, 24u},   /* -46 dB, phy_agc_reg_init_new */
    {"702C_comp",   0x600A702Cu, 0u},    /* -30/-32 dB, phy_set_rx_comp_new */
    {"70A0_comp",   0x600A70A0u, 24u},   /* -30/-32 dB, phy_set_rx_comp_new */
};
#define AGC_OFFSET_FIELDS (sizeof(s_agc_offset_fields) / sizeof(s_agc_offset_fields[0]))
#define AGC_OFFSET_NVS_FIELD "agc_off_f"
#define AGC_OFFSET_NVS_DB    "agc_off_db"
/* Explicit native-acquisition experiment. Field semantics are inferred from
 * vendor writes and measured acquisition duration, not named timing units.
 * Never force gain or disable AGC. One field changes per boot profile. */
static const agc_tune_t s_native_acq_profiles[] = {
    {"vendor",     0u,          0u,  0u,  0u},
    {"7034_5",     0x600A7034u, 24u, 7u,  5u},
    {"7034_2",     0x600A7034u, 24u, 7u,  2u},
    {"7034_1",     0x600A7034u, 24u, 7u,  1u},
    {"7158_6",     0x600A7158u,  0u, 7u,  6u},
    {"7158_2",     0x600A7158u,  0u, 7u,  2u},
    {"7158_1",     0x600A7158u,  0u, 7u,  1u},
    {"71B0_15",    0x600A71B0u, 21u, 7u, 15u},
    {"71B0_5",     0x600A71B0u, 21u, 7u,  5u},
    {"71B0_1",     0x600A71B0u, 21u, 7u,  1u},
    {"7034_127",   0x600A7034u, 24u, 7u, 127u},
};
#define NATIVE_ACQ_COUNT (sizeof(s_native_acq_profiles) / sizeof(s_native_acq_profiles[0]))
/* User-selected live default after the 7034=127 picture comparison.
 * The standalone meter starts vendor unless NVS explicitly selects a profile. */
#if CONFIG_C5VRX_NATIVE_AGC_CAPTURE_ONLY
#define NATIVE_ACQ_DEFAULT_INDEX 0u
#else
#define NATIVE_ACQ_DEFAULT_INDEX 10u
#endif
static uint8_t s_native_acq_profile;
static uint32_t s_native_acq_before, s_native_acq_after;

static uint8_t native_acq_load(void)
{
    nvs_handle_t handle;
    uint8_t value = NATIVE_ACQ_DEFAULT_INDEX;
    if (nvs_open(NATIVE_AGC_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
        return NATIVE_ACQ_DEFAULT_INDEX;
    (void)nvs_get_u8(handle, "agc_acq", &value);
    nvs_close(handle);
    return value < NATIVE_ACQ_COUNT ? value : NATIVE_ACQ_DEFAULT_INDEX;
}

static void native_acq_apply(void)
{
    const agc_tune_t *p = &s_native_acq_profiles[s_native_acq_profile];
    if (!s_native_agc || !p->reg) return;
    s_native_acq_before = REG32(p->reg);
    uint32_t mask = ((1u << p->width) - 1u) << p->shift;
    REG32(p->reg) = (s_native_acq_before & ~mask) | ((p->raw << p->shift) & mask);
    s_native_acq_after = REG32(p->reg);
}

void rf_native_acq_report(void)
{
    const agc_tune_t *p = &s_native_acq_profiles[s_native_acq_profile];
    printf("AGC_ACQ idx=%u count=%u name=%s native=%u reg=0x%08lx "
           "before=0x%08lx after=0x%08lx ch=%s mhz=%u\n",
           s_native_acq_profile, (unsigned)NATIVE_ACQ_COUNT, p->name,
           s_native_agc ? 1u : 0u, (unsigned long)p->reg,
           (unsigned long)s_native_acq_before, (unsigned long)s_native_acq_after,
           rf_get_current_channel()->name, rf_get_frequency_mhz());
}

/* Store a complete, one-shot boot measurement request in one transaction.
 * Reset older sweep/start-gain overrides to make each field comparison useful.
 * The selected native profile remains active for a live picture comparison. */
static esp_err_t native_acq_arm_value(uint8_t value)
{
    if (!s_native_agc) return ESP_ERR_INVALID_STATE;
    if (value >= NATIVE_ACQ_COUNT) return ESP_ERR_INVALID_ARG;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NATIVE_AGC_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_u8(handle, "agc_acq", value);
#if CONFIG_C5VRX_NATIVE_AGC_CAPTURE_ONLY
    if (err == ESP_OK) err = nvs_set_u8(handle, "agc_capture", 1u);
#else
    if (err == ESP_OK) err = nvs_set_u8(handle, "agc_capture", 0u);
#endif
    if (err == ESP_OK) err = nvs_set_u8(handle, AGC_TUNE_NVS_KEY, 0u);
    if (err == ESP_OK) err = nvs_set_u8(handle, NATIVE_INITGAIN_NVS_KEY, 0u);
    if (err == ESP_OK) err = nvs_set_u8(handle, AGC_OFFSET_NVS_DB, 0u);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}
esp_err_t rf_native_acq_arm(bool next)
{
    return native_acq_arm_value(next ?
        (uint8_t)((s_native_acq_profile + 1u) % NATIVE_ACQ_COUNT) : 0u);
}
esp_err_t rf_native_acq_arm_max(void)
{
    /* Appended profile preserves every existing persisted index. */
    return native_acq_arm_value((uint8_t)(NATIVE_ACQ_COUNT - 1u));
}
static uint8_t s_agc_offset_field;
/* Pinned C5 libphy audit: phy_bb_wdt_rst_enable() owns only 7C40[31].
 * No watchdog interrupt/status-clear/config write is part of this test. */
#define BB_WDG_CFG_REG  0x600A7C3Cu
#define BB_WDG_CTRL_REG 0x600A7C40u
#define BB_WDG_STATUS_REG 0x600A7C08u
#define BB_WDG_RESET_BIT (1u << 31)
static bool s_native_wdg_blocked;
static uint32_t s_native_wdg_vendor_bit;

void rf_native_wdg_report(const char *reason)
{
    uint32_t ctrl = REG32(BB_WDG_CTRL_REG);
    printf("AGC_WDG reason=%s native=%u requested_block=%u reset_en=%u "
           "cfg=0x%08lx ctrl=0x%08lx status=0x%08lx "
           "agc_ctrl=0x%08lx comp_ctrl=0x%08lx gain_state=0x%08lx\n",
           reason, s_native_agc ? 1u : 0u, s_native_wdg_blocked ? 1u : 0u,
           (ctrl & BB_WDG_RESET_BIT) ? 1u : 0u,
           (unsigned long)REG32(BB_WDG_CFG_REG), (unsigned long)ctrl,
           (unsigned long)REG32(BB_WDG_STATUS_REG),
           (unsigned long)REG32(RX_AGC_CTRL_REG),
           (unsigned long)REG32(0x600A702Cu),
           (unsigned long)REG32(0x600A7078u));
}

static void native_wdg_restore(void)
{
    if (!s_native_wdg_blocked) return;
    REG32(BB_WDG_CTRL_REG) = (REG32(BB_WDG_CTRL_REG) & ~BB_WDG_RESET_BIT) |
                            s_native_wdg_vendor_bit;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    s_native_wdg_blocked = false;
    rf_native_wdg_report("restore");
}

esp_err_t rf_native_wdg_toggle(void)
{
    if (s_native_wdg_blocked) { native_wdg_restore(); return ESP_OK; }
    /* Keep the first causality comparison free of gain/timing overrides. */
    if (!s_native_agc || s_native_acq_profile || s_agc_tune ||
        s_native_initgain || rf_agc_offset_db() || rf_fine_iq_active() ||
        (REG32(RX_AGC_CTRL_REG) & (1u << 29)) ||
        (REG32(0x600A702Cu) & (1u << 23))) return ESP_ERR_INVALID_STATE;
    uint32_t ctrl = REG32(BB_WDG_CTRL_REG);
    s_native_wdg_vendor_bit = ctrl & BB_WDG_RESET_BIT;
    rf_native_wdg_report("before");
    if (!s_native_wdg_vendor_bit) return ESP_ERR_NOT_SUPPORTED;
    REG32(BB_WDG_CTRL_REG) = ctrl & ~BB_WDG_RESET_BIT;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    s_native_wdg_blocked = true;
    rf_native_wdg_report("block_reset");
    if (REG32(BB_WDG_CTRL_REG) & BB_WDG_RESET_BIT) {
        native_wdg_restore();
        return ESP_FAIL;
    }
    return ESP_OK;
}

static int8_t s_agc_offset_db;
static int8_t s_agc_offset_vendor[AGC_OFFSET_FIELDS];
static bool s_agc_offset_vendor_ok[AGC_OFFSET_FIELDS];

static void agc_offset_capture_vendor(void)
{
    for (unsigned i = 0; i < AGC_OFFSET_FIELDS; ++i) {
        if (s_agc_offset_vendor_ok[i]) continue;
        const agc_offset_field_t *f = &s_agc_offset_fields[i];
        s_agc_offset_vendor[i] = (int8_t)((REG32(f->reg) >> f->shift) & 0xFFu);
        s_agc_offset_vendor_ok[i] = true;
    }
}

static void agc_offset_write(unsigned field, int value)
{
    const agc_offset_field_t *f = &s_agc_offset_fields[field];
    uint32_t mask = 0xFFu << f->shift;
    REG32(f->reg) = (REG32(f->reg) & ~mask) | (((uint32_t)(uint8_t)(int8_t)value) << f->shift);
}

static void agc_offset_apply(void)
{
    if (!s_native_agc) return;
    agc_offset_capture_vendor();
    for (unsigned i = 0; i < AGC_OFFSET_FIELDS; ++i)
        agc_offset_write(i, s_agc_offset_vendor[i] +
                            (i == s_agc_offset_field ? s_agc_offset_db : 0));
}

static void agc_offset_load(void)
{
    nvs_handle_t handle;
    /* Default: 70A0[31:24], the field with the largest measured level effect
     * in the register sweep (docs/native-agc-v2.md). */
    uint8_t field = 2, db = 0;
    s_agc_offset_field = field;
    if (nvs_open(NATIVE_AGC_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return;
    (void)nvs_get_u8(handle, AGC_OFFSET_NVS_FIELD, &field);
    (void)nvs_get_u8(handle, AGC_OFFSET_NVS_DB, &db);
    nvs_close(handle);
    s_agc_offset_field = field < AGC_OFFSET_FIELDS ? field : 0u;
    int v = (int8_t)db;
    s_agc_offset_db = (int8_t)(v < -12 ? -12 : v > 12 ? 12 : v);
}

esp_err_t rf_agc_offset_save(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NATIVE_AGC_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_u8(handle, AGC_OFFSET_NVS_FIELD, s_agc_offset_field);
    if (err == ESP_OK) err = nvs_set_u8(handle, AGC_OFFSET_NVS_DB, (uint8_t)s_agc_offset_db);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

void rf_agc_offset_set(int db)
{
    if (db) (void)native_agc_pace_set(0, 20);
    analog_agc_cancel();
    if (db < -12) db = -12;
    if (db > 12) db = 12;
    s_agc_offset_db = (int8_t)db;
    agc_offset_apply();
}

int rf_agc_offset_db(void)
{
    return s_agc_offset_db;
}

void rf_agc_offset_next_field(void)
{
    analog_agc_cancel();
    s_agc_offset_field = (uint8_t)((s_agc_offset_field + 1u) % AGC_OFFSET_FIELDS);
    s_agc_offset_db = 0;
    agc_offset_apply();
}

const char *rf_agc_offset_field_name(void)
{
    return s_agc_offset_fields[s_agc_offset_field].name;
}

void rf_set_bb_agc(bool enable)
{
    (void)native_agc_pace_set(0, 20);
    analog_agc_cancel();
    if (!s_native_agc) return;
    extern void phy_disable_agc(void);
    extern void phy_enable_agc(void);
    if (enable) phy_enable_agc();
    else phy_disable_agc();
    s_bb_agc_off = !enable;
}

bool rf_bb_agc_enabled(void)
{
    return !s_bb_agc_off;
}

void rf_set_fine_iq(bool fine)
{
    if (fine) (void)native_agc_pace_set(0, 20);
    analog_agc_cancel();
    if (fine == s_iq_fine) return;
    const uint8_t *diag = fine ? s_iq_diag_fine : s_iq_diag;
    for (unsigned lane = 0u; lane < 8u; ++lane) {
        esp_rom_gpio_connect_out_signal(s_iq_pins[lane],
                                        MODEM_DIAG0_IDX + diag[lane],
                                        false, false);
    }
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    s_iq_fine = fine;
}

bool rf_fine_iq_active(void)
{
    return s_iq_fine;
}

static void rf_enable_continuous_modem(void)
{
    /* Keep CPU ownership of HP SRAM */
    REG32(HP_SRAM_USAGE) = (REG32(HP_SRAM_USAGE) & 0xfffef0ffu) | 0x00010000u;

    /* Un-gate modem clocks and force front-end active. */
    REG32(SOURCE_CTRL) &= 0xff87ffffu;
    REG32(SOURCE_MUX) = (REG32(SOURCE_MUX) & 0xfffffff8u) | 1u;
    REG32(MODEM_CLOCK) = UINT32_MAX;
    REG32(FE_ENABLE) |= 4u;
    REG32(FE_PATH) &= ~1u;

    /* Configure DUMP_FORMAT mode 0 (proven golden RF dump configuration) */
    uint32_t v = REG32(DUMP_FORMAT);
    v = (v & 0xff03ffffu) | 0x006c0000u;
    REG32(DUMP_FORMAT) = v;
    v = (REG32(DUMP_FORMAT) & 0xfffc0fffu) | 0x0001a000u;
    REG32(DUMP_FORMAT) = v;
    v = (REG32(DUMP_FORMAT) & 0xfffff03fu) | 0x00000640u;
    REG32(DUMP_FORMAT) = v;
    v = (REG32(DUMP_FORMAT) & 0xffffffc0u) | 0x18u;
    REG32(DUMP_FORMAT) = v | 0x01000000u;

    /* Set TX_START selector in pre-trigger circular mode (TX_START_SELECT = 0x00060000).
     * Because MAC TX queues are quiescent, TX_START never fires. With CTRL_DUMP_FIRST,
     * the hardware continuously streams pre-trigger samples onto the MODEM_DIAG bus.
     * Crucial: 0x01e00000 software trigger bits are masked out. */
    REG32(DUMP_PTR_MODE) = (REG32(DUMP_PTR_MODE) & ~SELECTOR_MASK) | TX_START_SELECT;

    /* Control: CTRL_DUMP_FIRST, length 16384, ENABLE */
    uint32_t ctrl = REG32(DUMP_CTRL);
    ctrl &= ~(CTRL_ENABLE | 0x00080000u | 0x00040000u); /* Clear ENABLE, START, DONE */
    ctrl |= CTRL_DUMP_FIRST;
    ctrl = (ctrl & ~0x0001ffffu) | 16384u;
    REG32(DUMP_CTRL) = ctrl;

    __asm__ __volatile__("fence iorw, iorw" ::: "memory");

    /* Arm dump engine with ENABLE only */
    REG32(DUMP_CTRL) = ctrl | CTRL_ENABLE;

    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

/* Track all timers created/armed by the closed-source Wi-Fi stack */
typedef struct {
    void *timer;
    void *fn;
    void *arg;
    uint32_t period_ms;
    bool repeat;
    bool armed;
    uint32_t arm_count;
} tracked_timer_t;

#define MAX_TRACKED_TIMERS 32
static tracked_timer_t s_tracked_timers[MAX_TRACKED_TIMERS];
static size_t s_num_tracked_timers = 0;
static wifi_osi_funcs_t s_custom_osi_funcs;

static tracked_timer_t *find_or_create_timer_slot(void *timer)
{
    for (size_t i = 0; i < s_num_tracked_timers; ++i) {
        if (s_tracked_timers[i].timer == timer) return &s_tracked_timers[i];
    }
    if (s_num_tracked_timers < MAX_TRACKED_TIMERS) {
        tracked_timer_t *slot = &s_tracked_timers[s_num_tracked_timers++];
        memset(slot, 0, sizeof(*slot));
        slot->timer = timer;
        return slot;
    }
    return NULL;
}

static void tracked_timer_setfn(void *ptimer, void *pfunction, void *parg)
{
    tracked_timer_t *slot = find_or_create_timer_slot(ptimer);
    if (slot) {
        slot->fn = pfunction;
        slot->arg = parg;
    }
    g_wifi_osi_funcs._timer_setfn(ptimer, pfunction, parg);
}

static void tracked_timer_arm(void *timer, uint32_t tmout, bool repeat)
{
    tracked_timer_t *slot = find_or_create_timer_slot(timer);
    if (slot) {
        slot->period_ms = tmout;
        slot->repeat = repeat;
        slot->armed = true;
        slot->arm_count++;
    }
    g_wifi_osi_funcs._timer_arm(timer, tmout, repeat);
}

static void tracked_timer_arm_us(void *ptimer, uint32_t us, bool repeat)
{
    tracked_timer_t *slot = find_or_create_timer_slot(ptimer);
    if (slot) {
        slot->period_ms = (us + 500u) / 1000u;
        slot->repeat = repeat;
        slot->armed = true;
        slot->arm_count++;
    }
    g_wifi_osi_funcs._timer_arm_us(ptimer, us, repeat);
}

static void tracked_timer_disarm(void *timer)
{
    tracked_timer_t *slot = find_or_create_timer_slot(timer);
    if (slot) {
        slot->armed = false;
    }
    g_wifi_osi_funcs._timer_disarm(timer);
}

static void tracked_timer_done(void *ptimer)
{
    tracked_timer_t *slot = find_or_create_timer_slot(ptimer);
    if (slot) {
        slot->armed = false;
    }
    g_wifi_osi_funcs._timer_done(ptimer);
}

void rf_dump_tracked_timers(void)
{
    printf("\n=== WI-FI VENDOR TIMERS INVENTORY (%u tracked) ===\n", (unsigned)s_num_tracked_timers);
    for (size_t i = 0; i < s_num_tracked_timers; ++i) {
        printf(" [%u] fn=0x%08lx period=%4lu ms repeat=%d armed=%d arms=%lu\n",
               (unsigned)i,
               (unsigned long)(uintptr_t)s_tracked_timers[i].fn,
               (unsigned long)s_tracked_timers[i].period_ms,
               s_tracked_timers[i].repeat ? 1 : 0,
               s_tracked_timers[i].armed ? 1 : 0,
               (unsigned long)s_tracked_timers[i].arm_count);
    }
    printf("==================================================\n\n");
    fflush(stdout);
}

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) err = nvs_flash_init();
    }
    return err;
}

esp_err_t rf_prepare_fresh_phy_calibration(void)
{
    /* Espressif documents this API for diagnostic flows before Wi-Fi init.
     * C5VRX invokes it only as a request for the NEXT boot: the live receiver
     * is not recalibrated in-place. The caller reboots immediately after a
     * successful erase, so esp_wifi_init() on the next boot sees no stored PHY
     * calibration data and rebuilds the vendor calibration state normally. */
    esp_err_t err = esp_phy_erase_cal_data_in_nvs();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not erase PHY calibration namespace: %s",
                 esp_err_to_name(err));
    }
    return err;
}

static bool native_agc_boot_requested(void)
{
    nvs_handle_t handle;
    uint8_t value = 1u;
    if (nvs_open(NATIVE_AGC_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
        return true;
    if (nvs_get_u8(handle, NATIVE_AGC_NVS_KEY, &value) != ESP_OK) value = 1u;
    nvs_close(handle);
    return value != 0u;
}

static bool pll_track_boot_requested(void)
{
    nvs_handle_t handle;
    uint8_t value = 0;
    if (nvs_open(NATIVE_AGC_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
        return false;
    if (nvs_get_u8(handle, PLL_TRACK_NVS_KEY, &value) != ESP_OK) value = 0;
    nvs_close(handle);
    return value == 1u;
}

esp_err_t rf_request_pll_track_boot(bool enable)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NATIVE_AGC_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_u8(handle, PLL_TRACK_NVS_KEY, enable ? 1u : 0u);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

bool rf_pll_track_active(void)
{
    return s_pll_track;
}

static uint8_t native_initgain_load(void)
{
    nvs_handle_t handle;
    uint8_t value = 0;
    if (nvs_open(NATIVE_AGC_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
        return 0;
    if (nvs_get_u8(handle, NATIVE_INITGAIN_NVS_KEY, &value) != ESP_OK) value = 0;
    nvs_close(handle);
    return value;
}

static void native_initgain_apply(void)
{
    uint32_t reg = REG32(NATIVE_INITGAIN_REG);
    uint8_t observed = (uint8_t)((reg & NATIVE_INITGAIN_MASK) >> 2);
    /* A retune need not rewrite this field. Never mistake our override for
     * the vendor default or progressively lower the restore ceiling. */
    if (!s_native_initgain_vendor) s_native_initgain_vendor = observed;
    if (!s_native_agc || !s_native_initgain) return;
    uint8_t g = s_native_initgain;
    /* Never above the vendor start (the calibrated maximum - 1). */
    if (g > s_native_initgain_vendor) g = s_native_initgain_vendor;
    REG32(NATIVE_INITGAIN_REG) = (reg & ~NATIVE_INITGAIN_MASK) | ((uint32_t)g << 2);
}

uint8_t rf_native_initgain(void)
{
    return (uint8_t)((REG32(NATIVE_INITGAIN_REG) & NATIVE_INITGAIN_MASK) >> 2);
}

uint8_t rf_native_initgain_vendor(void)
{
    return s_native_initgain_vendor;
}

esp_err_t rf_set_native_initgain(uint8_t gain)
{
    analog_agc_cancel();
    if (gain && gain < NATIVE_INITGAIN_MIN) return ESP_ERR_INVALID_ARG;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NATIVE_AGC_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_u8(handle, NATIVE_INITGAIN_NVS_KEY, gain);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) return err;
    if (!gain && s_native_initgain && s_native_agc) {
        uint32_t reg = REG32(NATIVE_INITGAIN_REG);
        REG32(NATIVE_INITGAIN_REG) = (reg & ~NATIVE_INITGAIN_MASK) |
                                     ((uint32_t)s_native_initgain_vendor << 2);
    }
    s_native_initgain = gain;
    if (gain) {
        /* Keep the captured vendor value; apply only the override. */
        uint32_t reg = REG32(NATIVE_INITGAIN_REG);
        uint8_t g = gain > s_native_initgain_vendor ? s_native_initgain_vendor : gain;
        if (s_native_agc)
            REG32(NATIVE_INITGAIN_REG) = (reg & ~NATIVE_INITGAIN_MASK) | ((uint32_t)g << 2);
    }
    return ESP_OK;
}

esp_err_t rf_request_native_agc_boot(bool enable)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NATIVE_AGC_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_u8(handle, NATIVE_AGC_NVS_KEY, enable ? 1u : 0u);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

bool rf_native_agc_active(void)
{
    return s_native_agc;
}

void rf_get_native_agc_state(rf_native_agc_state_t *state)
{
    if (!state) return;
    state->active = s_native_agc;
    state->gain_status_reg = REG32(RX_GAIN_STATUS_REG);
    state->agc_ctrl_reg = REG32(RX_AGC_CTRL_REG);
    state->blocked_writes = s_native_agc_blocked_writes;
}

/* Read-only dump of the AGC register block programmed by the vendor AGC init,
 * update and saturation-gain routines. Evidence for #117 Phase 4 tuning. */
void rf_dump_agc_regs(void)
{
    printf("AGCREGS native=%u", s_native_agc ? 1u : 0u);
    for (uint32_t addr = 0x600A7000u; addr < 0x600A7200u; addr += 4u) {
        if ((addr & 0x1Fu) == 0u) printf("\nAGCREGS 0x%08lx:", (unsigned long)addr);
        printf(" %08lx", (unsigned long)REG32(addr));
    }
    /* bb_agc_reg_update() programs 0x600A8004..0x600A807C. */
    for (uint32_t addr = 0x600A8000u; addr < 0x600A8080u; addr += 4u) {
        if ((addr & 0x1Fu) == 0u) printf("\nAGCREGS 0x%08lx:", (unsigned long)addr);
        printf(" %08lx", (unsigned long)REG32(addr));
    }
    printf("\n");
}

/* Native AGC state telemetry (#121 section 7). 0x600A706C[15:8] is the
 * signed signal RSSI read by the vendor phy_get_sigrssi(). Its low byte and
 * 0x600A7078[7:0] are undecoded AGC state: on hardware they hold 82-83 with
 * no carrier, sit in the 30s-60s with a carrier, and 7078 repeatedly jumps
 * back to >=80 (native re-acquisition from the top) while a carrier is
 * present. Telemetry only, never a control input. */
#define AGC_LIVE_REG_A 0x600A706Cu
#define AGC_LIVE_REG_B 0x600A7078u
#define AGC_POLL_SAMPLES 4000u

void rf_native_gain_stats(unsigned samples, rf_native_gain_stats_t *out)
{
    static uint16_t hist[256];
    if (!out || !samples) return;
    memset(hist, 0, sizeof(hist));
    uint32_t switches = 0, restarts = 0;
    uint8_t last = (uint8_t)REG32(AGC_LIVE_REG_A);
    uint8_t last_b = (uint8_t)REG32(AGC_LIVE_REG_B);
    uint8_t start_index = rf_native_initgain();
    int64_t start = esp_timer_get_time();
    for (unsigned n = 0; n < samples; ++n) {
        uint8_t v = (uint8_t)REG32(AGC_LIVE_REG_A);
        uint8_t b = (uint8_t)REG32(AGC_LIVE_REG_B);
        ++hist[v];
        switches += v != last;
        restarts += start_index && b == start_index && last_b != start_index;
        last = v;
        last_b = b;
        esp_rom_delay_us(2);
    }
    int64_t elapsed = esp_timer_get_time() - start;
    *out = (rf_native_gain_stats_t){0};
    out->elapsed_us = (uint32_t)elapsed;
    out->switches = switches;
    out->switches_per_ms_x10 = elapsed > 0 ?
        (uint32_t)((uint64_t)switches * 10000u / (uint64_t)elapsed) : 0u;
    out->restarts = restarts;
    out->restarts_per_ms_x10 = elapsed > 0 ?
        (uint32_t)((uint64_t)restarts * 10000u / (uint64_t)elapsed) : 0u;
    unsigned cumulative = 0;
    bool have_min = false, have_median = false;
    for (unsigned v = 0; v < 256u; ++v) {
        if (!hist[v]) continue;
        if (!have_min) { out->min = (uint8_t)v; have_min = true; }
        out->max = (uint8_t)v;
        cumulative += hist[v];
        if (!have_median && cumulative * 2u >= samples) {
            out->median = (uint8_t)v;
            have_median = true;
        }
    }
}

/* Median of the native AGC state byte over ~0.4 ms. With a carrier it sits
 * lower the stronger the input, so the signal meter uses it as the gain the
 * hardware actually applied. Re-entries at the start gain are outliers the
 * median rejects. Read-only; own histogram so the console task's
 * rf_native_gain_stats() can run concurrently. */
uint8_t rf_native_gain_index(void)
{
    enum { SAMPLES = 128u };
    uint8_t hist[128] = {0};
    for (unsigned n = 0; n < SAMPLES; ++n) {
        uint8_t v = (uint8_t)REG32(AGC_LIVE_REG_A);
        if (hist[v & 0x7Fu] < 255u) ++hist[v & 0x7Fu];
        esp_rom_delay_us(2);
    }
    unsigned cumulative = 0;
    for (unsigned v = 0; v < 128u; ++v) {
        cumulative += hist[v];
        if (cumulative * 2u >= SAMPLES) return (uint8_t)v;
    }
    return 127u;
}

/* LAB (#121 section 6A): fields the vendor AGC init (phy_agc_reg_init_new)
 * programs that may hold the native target level, thresholds or hysteresis.
 * Each field is stepped reversibly from its captured boot value; 'restore'
 * writes every captured value back, and a reboot re-runs vendor init.
 * Fields are raw bit ranges: their semantics are exactly what this tests. */
typedef struct {
    const char *name;
    uint32_t reg;
    uint8_t shift, width, step;
    bool is_signed;
} agc_lab_field_t;

/* Round 1 (7128[31:24], 7034[30:24], 7158[6:0], 71B0[27:21], plus the
 * saturation bytes of 7064/7114) moved neither the Q4 target nor the switch
 * rate on hardware. Round 2: packet-detect count (phy_rx_pkdet_num_set writes
 * 0x808 to 7068) and the rx-sense detection threshold (phy_rx_sense_set writes
 * 7010/7014[31:23] and 7044[7:0]); packet detections re-arm a packet AGC. */
/* Rounds 1-3 on hardware were invalidated: the receiver had been moved to an
 * adjacent channel (BOOT short-click) and saw no carrier. All candidates are
 * retested together on the correct channel.
 *
 * The first three entries come from documented semantics rather than guesses.
 * ESPARGOS esp-sdr (C5/C6/C61 manual gain) names 0x600A7094[8:2] the AGC
 * initial gain and 0x600A713C[24:18] the AGC gain threshold. The pinned C5
 * AGC max-gain setup routine writes max-1 (82) into both, which matches the
 * native restarts jumping back to 82. esp-sdr also disables RF saturation
 * intervention for stable gain; on C5 that is the rfagc block, whose only
 * register write in phy_rfagc_disable() is 0x600A705C = 0 (a width-0 entry
 * below toggles the whole word between 0 and its boot value). */
/* 0x600A80xx entries: written only by the vendor bb_agc_reg_update()
 * (8028 = 0xC0403020 looks like four stacked level thresholds, 8018/801C/8020
 * like single levels); the prime candidates for the native target level. */
static const agc_lab_field_t s_agc_lab_fields[] = {
    {"7094_8_2_initgain", 0x600A7094u, 2u, 7u, 8u, false},
    {"713C_24_18_thresh", 0x600A713Cu, 18u, 7u, 8u, false},
    {"705C_rfsat_word",   0x600A705Cu, 0u, 0u, 0u, false},
    {"8028_7_0",          0x600A8028u, 0u, 8u, 8u, false},
    {"8028_15_8",         0x600A8028u, 8u, 8u, 8u, false},
    {"8028_23_16",        0x600A8028u, 16u, 8u, 8u, false},
    {"8028_31_24",        0x600A8028u, 24u, 8u, 8u, false},
    {"8020_9_0",          0x600A8020u, 0u, 10u, 32u, false},
    {"801C_7_0",          0x600A801Cu, 0u, 8u, 16u, false},
    {"7128_31_24", 0x600A7128u, 24u, 8u, 2u, true},
    {"7034_30_24", 0x600A7034u, 24u, 7u, 1u, false},
    {"7158_6_0",   0x600A7158u, 0u,  7u, 1u, false},
    {"71B0_27_21", 0x600A71B0u, 21u, 7u, 1u, false},
    {"7068_7_0",   0x600A7068u, 0u,  8u, 2u, false},
    {"7068_15_8",  0x600A7068u, 8u,  8u, 2u, false},
    {"7010_31_23", 0x600A7010u, 23u, 9u, 4u, true},
    {"7014_31_23", 0x600A7014u, 23u, 9u, 4u, true},
    {"7044_7_0",   0x600A7044u, 0u,  8u, 4u, true},
};
#define AGC_LAB_FIELD_COUNT (sizeof(s_agc_lab_fields) / sizeof(s_agc_lab_fields[0]))
static uint32_t s_agc_lab_boot[AGC_LAB_FIELD_COUNT];
static bool s_agc_lab_captured[AGC_LAB_FIELD_COUNT];
static unsigned s_agc_lab_selected;

static uint32_t agc_lab_get(const agc_lab_field_t *f)
{
    if (!f->width) return REG32(f->reg);
    return (REG32(f->reg) >> f->shift) & ((1u << f->width) - 1u);
}

static int agc_lab_value(const agc_lab_field_t *f, uint32_t raw)
{
    if (!f->width) return raw ? 1 : 0; /* whole-word toggle: 1 = vendor value */
    if (f->is_signed && (raw & (1u << (f->width - 1u))))
        return (int)raw - (int)(1u << f->width);
    return (int)raw;
}

static void agc_lab_capture(unsigned i)
{
    if (s_agc_lab_captured[i]) return;
    s_agc_lab_boot[i] = agc_lab_get(&s_agc_lab_fields[i]);
    s_agc_lab_captured[i] = true;
}

void rf_lab_agc_field(char action)
{
    analog_agc_cancel();
    if (action == 'P') {
        s_agc_lab_selected = (s_agc_lab_selected + 1u) % AGC_LAB_FIELD_COUNT;
    } else if (action == 'B') {
        for (unsigned i = 0; i < AGC_LAB_FIELD_COUNT; ++i) {
            if (!s_agc_lab_captured[i]) continue;
            const agc_lab_field_t *f = &s_agc_lab_fields[i];
            if (!f->width) {
                REG32(f->reg) = s_agc_lab_boot[i];
                continue;
            }
            uint32_t mask = ((1u << f->width) - 1u) << f->shift;
            REG32(f->reg) = (REG32(f->reg) & ~mask) | (s_agc_lab_boot[i] << f->shift);
        }
    } else if (action == 'M' || action == 'J') {
        unsigned i = s_agc_lab_selected;
        const agc_lab_field_t *f = &s_agc_lab_fields[i];
        agc_lab_capture(i);
        if (!f->width) {
            /* 'J' = intervention off (word 0), 'M' = restore vendor word. */
            REG32(f->reg) = action == 'J' ? 0u : s_agc_lab_boot[i];
            action = 0;
        }
    }
    if (action == 'M' || action == 'J') {
        unsigned i = s_agc_lab_selected;
        const agc_lab_field_t *f = &s_agc_lab_fields[i];
        int lo = f->is_signed ? -(1 << (f->width - 1u)) : 0;
        int hi = f->is_signed ? (1 << (f->width - 1u)) - 1 : (int)((1u << f->width) - 1u);
        int v = agc_lab_value(f, agc_lab_get(f)) +
                (action == 'M' ? f->step : -(int)f->step);
        if (v < lo) v = lo;
        if (v > hi) v = hi;
        uint32_t mask = ((1u << f->width) - 1u) << f->shift;
        uint32_t raw = (uint32_t)v & ((1u << f->width) - 1u);
        REG32(f->reg) = (REG32(f->reg) & ~mask) | (raw << f->shift);
    }
    const agc_lab_field_t *f = &s_agc_lab_fields[s_agc_lab_selected];
    agc_lab_capture(s_agc_lab_selected);
    printf("AGCFIELD sel=%s value=%d boot=%d reg=0x%08lx\n", f->name,
           agc_lab_value(f, agc_lab_get(f)),
           agc_lab_value(f, s_agc_lab_boot[s_agc_lab_selected]),
           (unsigned long)REG32(f->reg));
}

void rf_poll_agc_live(void)
{
    static uint16_t hist_a[256], hist_b[256];
    memset(hist_a, 0, sizeof(hist_a));
    memset(hist_b, 0, sizeof(hist_b));
    uint32_t switches_a = 0, switches_b = 0;
    uint8_t last_a = (uint8_t)REG32(AGC_LIVE_REG_A);
    uint8_t last_b = (uint8_t)REG32(AGC_LIVE_REG_B);
    int64_t start = esp_timer_get_time();
    for (unsigned n = 0; n < AGC_POLL_SAMPLES; ++n) {
        uint8_t a = (uint8_t)REG32(AGC_LIVE_REG_A);
        uint8_t b = (uint8_t)REG32(AGC_LIVE_REG_B);
        ++hist_a[a];
        ++hist_b[b];
        switches_a += a != last_a;
        switches_b += b != last_b;
        last_a = a;
        last_b = b;
        esp_rom_delay_us(2);
    }
    int64_t elapsed_us = esp_timer_get_time() - start;
    printf("AGCPOLL n=%u elapsed_us=%lld switches_a=%lu switches_b=%lu\n",
           AGC_POLL_SAMPLES, (long long)elapsed_us,
           (unsigned long)switches_a, (unsigned long)switches_b);
    printf("AGCPOLL hist_a(706C[7:0]):");
    for (unsigned v = 0; v < 256u; ++v)
        if (hist_a[v]) printf(" %u:%u", v, hist_a[v]);
    printf("\nAGCPOLL hist_b(7078[7:0]):");
    for (unsigned v = 0; v < 256u; ++v)
        if (hist_b[v]) printf(" %u:%u", v, hist_b[v]);
    printf("\n");
}

/* Explicit analog policy trial, owned by the slow control task. The source
 * shim applies after PHY calibration, irrespective of archive/ROM binding.
 * These are raw BB-policy candidates; no continuous-FM mode is asserted. */
#define ANALOG_AGC_REG 0x600A8020u
static native_analog_agc_t s_analog_agc;
static portMUX_TYPE s_analog_agc_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_analog_agc_request;

static bool analog_agc_eligible(void)
{
    return !native_agc_pace_enabled() &&
           !__atomic_load_n(&s_analog_rf_transitions, __ATOMIC_ACQUIRE) &&
           s_native_agc && !s_bb_agc_off && !s_iq_fine &&
           !s_agc_tune && !s_native_initgain &&
           (s_native_acq_profile == 0u || s_native_acq_profile == NATIVE_ACQ_COUNT - 1u) &&
           !rf_agc_offset_db() && !s_native_wdg_blocked &&
           rf_native_initgain() == rf_native_initgain_vendor() &&
           !(REG32(RX_AGC_CTRL_REG) & (1u << 29)) &&
           !(REG32(0x600A702Cu) & (1u << 23)) &&
           ((REG32(0x600A7034u) >> 24) & 0x7fu) == (s_native_acq_profile ? 127u : 10u) &&
           (REG32(0x600A7158u) & 0x7fu) == 13u &&
           ((REG32(0x600A71B0u) >> 21) & 0x7fu) == 30u;
}

void rf_analog_agc_request(unsigned profile)
{
    if (profile <= 2u)
        __atomic_store_n(&s_analog_agc_request, (int)profile + 1, __ATOMIC_RELEASE);
}

void rf_analog_agc_report(const char *reason)
{
    portENTER_CRITICAL(&s_analog_agc_lock);
    native_analog_agc_t state = s_analog_agc;
    uint32_t word = REG32(ANALOG_AGC_REG);
    portEXIT_CRITICAL(&s_analog_agc_lock);
    printf("AGC_ANALOG reason=%s active=%u profile=%u reg=0x%08lx "
           "baseline=%lu applied=%lu deadline_us=%lld native=%u ch=%s mhz=%u\n",
           reason, state.active ? 1u : 0u, state.profile, (unsigned long)word,
           (unsigned long)state.baseline, (unsigned long)state.applied,
           (long long)state.deadline_us, s_native_agc ? 1u : 0u,
           rf_get_current_channel()->name, rf_get_frequency_mhz());
}

static bool analog_agc_cancel(void)
{
    __atomic_store_n(&s_analog_agc_request, 0, __ATOMIC_RELEASE);
    portENTER_CRITICAL(&s_analog_agc_lock);
    bool active = s_analog_agc.active;
    const char *reason = "cancel";
    uint32_t word;
    if (active && native_analog_agc_end(&s_analog_agc, REG32(ANALOG_AGC_REG), &word)) {
        REG32(ANALOG_AGC_REG) = word;
        __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    }
    if (active && (REG32(ANALOG_AGC_REG) & ANALOG_AGC_MASK) != s_analog_agc.baseline)
        reason = "cancel_field_changed_or_restore_failed";
    portEXIT_CRITICAL(&s_analog_agc_lock);
    if (active) rf_analog_agc_report(reason);
    return active;
}

static void analog_agc_transition_begin(void)
{
    __atomic_add_fetch(&s_analog_rf_transitions, 1u, __ATOMIC_ACQ_REL);
    native_agc_pace_suspend();
    (void)analog_agc_cancel();
}

static void analog_agc_transition_end(void)
{
    if (__atomic_sub_fetch(&s_analog_rf_transitions, 1u, __ATOMIC_ACQ_REL) == 0u &&
        native_agc_pace_enabled()) {
        esp_err_t err = native_agc_pace_resume();
        if (err != ESP_OK) ESP_LOGW(TAG, "AGC pace resume: %s", esp_err_to_name(err));
    }
}

bool rf_analog_agc_service(void)
{
    const int request = __atomic_exchange_n(&s_analog_agc_request, 0, __ATOMIC_ACQ_REL);
    if (request == 1) return analog_agc_cancel();
    const int64_t now = esp_timer_get_time();
    const char *reason = NULL;
    portENTER_CRITICAL(&s_analog_agc_lock);
    const bool active = s_analog_agc.active;
    if (!active && !request) {
        portEXIT_CRITICAL(&s_analog_agc_lock);
        return false; /* Default path performs no PHY reads or writes. */
    }
    bool changed = false;
    uint32_t word = REG32(ANALOG_AGC_REG), write;
    if (native_analog_agc_poll(&s_analog_agc, word, now, analog_agc_eligible(), &write)) {
        REG32(ANALOG_AGC_REG) = write;
        __asm__ __volatile__("fence iorw, iorw" ::: "memory");
        changed = true;
        reason = (REG32(ANALOG_AGC_REG) & ANALOG_AGC_MASK) == s_analog_agc.baseline ?
                 "restored" : "restore_readback_failed";
    } else if (active && !s_analog_agc.active) {
        changed = true;
        reason = "overwritten";
    }
    if (request > 1) {
        word = REG32(ANALOG_AGC_REG);
        if (native_analog_agc_begin(&s_analog_agc, (unsigned)(request - 1), word,
                                    now, analog_agc_eligible(), &write)) {
            changed = true;
            REG32(ANALOG_AGC_REG) = write;
            __asm__ __volatile__("fence iorw, iorw" ::: "memory");
            if ((REG32(ANALOG_AGC_REG) & ANALOG_AGC_MASK) == s_analog_agc.applied)
                reason = "started";
            else {
                /* Failed/overwritten readback: retire without repeated writes. */
                if (native_analog_agc_end(&s_analog_agc, REG32(ANALOG_AGC_REG), &write)) {
                    REG32(ANALOG_AGC_REG) = write;
                    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
                }
                reason = "readback_failed";
            }
        } else reason = "rejected_need_coarse_native_supported_baseline";
    }
    portEXIT_CRITICAL(&s_analog_agc_lock);
    if (reason) rf_analog_agc_report(reason);
    return changed;
}

esp_err_t rf_start(void)
{
    /* NVS is required by ESP-IDF Wi-Fi/PHY initialization. */
    esp_err_t err = init_nvs();
    if (err != ESP_OK) return err;
    s_native_agc = native_agc_boot_requested();
    s_agc_tune = agc_tune_load();
#if CONFIG_C5VRX_PHY_PHASE_TAP_PROBE
    s_native_acq_profile = s_agc_tune == 0 ? native_acq_load() : 0;
#else
    s_native_acq_profile = native_acq_load();
#endif
    agc_offset_load();
    s_native_initgain = native_initgain_load();
    s_pll_track = pll_track_boot_requested();

    /* esp_netif_init + default event loop are required by esp_wifi_init().
     * Tolerant of ESP_ERR_INVALID_STATE (already initialized by IDF). */
    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    /* Install tracked OSI functions to inventory all Wi-Fi vendor timers */
    s_custom_osi_funcs = g_wifi_osi_funcs;
    s_custom_osi_funcs._timer_setfn = tracked_timer_setfn;
    s_custom_osi_funcs._timer_arm = tracked_timer_arm;
    s_custom_osi_funcs._timer_arm_us = tracked_timer_arm_us;
    s_custom_osi_funcs._timer_disarm = tracked_timer_disarm;
    s_custom_osi_funcs._timer_done = tracked_timer_done;

    /* Initialize Wi-Fi driver with RAM-only storage -- no NVS needed.
     * Crucial: sta_disconnected_pm MUST be false. By default, ESP-IDF enables
     * power management for disconnected stations, periodically shutting down
     * RF, PHY, and BB when idle, which causes periodic loss of MODEM_DIAG clocking. */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.osi_funcs = &s_custom_osi_funcs;
    cfg.sta_disconnected_pm = false;
    if ((err = esp_wifi_init(&cfg)) != ESP_OK) return err;
    if ((err = esp_wifi_set_storage(WIFI_STORAGE_RAM)) != ESP_OK) return err;
    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK) return err;
    if ((err = esp_wifi_start()) != ESP_OK) return err;

    /* Dump initial Wi-Fi timers armed during startup */
    rf_dump_tracked_timers();

    /* Force 5 GHz band only. */
#if CONFIG_SOC_WIFI_SUPPORT_5G
    if ((err = esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY)) != ESP_OK)
        return err;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif

    /* No power saving -- PHY clock must remain alive at all times. */
    if ((err = esp_wifi_set_ps(WIFI_PS_NONE)) != ESP_OK) return err;

    /* Restrict 5 GHz protocols. */
    wifi_protocols_t protocols = {
        .ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                  WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX,
        .ghz_5g = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N,
    };
    if ((err = esp_wifi_set_protocols(WIFI_IF_STA, &protocols)) != ESP_OK)
        return err;

    /* Set BW40 on 5 GHz. Hard failure if not available -- NO BW20 fallback.
     * BW40 is a fixed hardware requirement for MODEM_DIAG IQ precision. */
    wifi_bandwidths_t bandwidths = {
        .ghz_2g = WIFI_BW20,
        .ghz_5g = RF_BANDWIDTH,
    };
    err = esp_wifi_set_bandwidths(WIFI_IF_STA, &bandwidths);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BW40 not available (err=%s). No BW20 fallback.", esp_err_to_name(err));
        return err;  /* Hard failure. BW20 produces degraded Q4/I4. */
    }

    /* Channel 173 = 5865 MHz. */
    if ((err = esp_wifi_set_channel(RF_CHANNEL_NUMBER, WIFI_SECOND_CHAN_NONE)) != ESP_OK)
        return err;

    /* Promiscuous mode keeps the RX path and MODEM_DIAG bus active.
     * Zero filter mask prevents LMAC from buffering packets or firing software interrupts. */
    if ((err = esp_wifi_set_promiscuous(true)) != ESP_OK) return err;
    wifi_promiscuous_filter_t filter = { .filter_mask = 0 };
    (void)esp_wifi_set_promiscuous_filter(&filter);

    /* Hardware-disable all 5 LMAC TX queues. Receive-only from here on. */
    if ((err = lock_rx_only()) != ESP_OK) return err;

    /* Verify channel lock. */
    uint8_t primary = 0u;
    wifi_second_chan_t secondary = WIFI_SECOND_CHAN_NONE;
    if ((err = esp_wifi_get_channel(&primary, &secondary)) != ESP_OK) return err;
    if (primary != RF_CHANNEL_NUMBER) {
        ESP_LOGE(TAG, "Channel mismatch: got %u, expected %u", primary, RF_CHANNEL_NUMBER);
        return ESP_ERR_INVALID_STATE;
    }

    /* Route MODEM_DIAG to PARLIO RX GPIO pins. */
    if ((err = route_modem_iq()) != ESP_OK) return err;

    /* Un-gate modem ADC clock and force continuous sampling. */
    rf_enable_continuous_modem();

    /* Keep the vendor Wi-Fi packet AGC out of the analog-FM receive path.
     * C5VRX has its own slow analog-video gain controller below; leaving the
     * packet AGC enabled lets the closed PHY hunt/recalibrate independently,
     * which invalidates our gain model and can desensitize weak-signal receive.
     * The native-AGC experiment is the single exception: it never disables the
     * vendor loop and C5VRX makes zero gain decisions or writes. */
    extern void phy_disable_agc(void);
    extern void phy_rfagc_disable(void);
    if (!s_native_agc) {
        phy_disable_agc();
        phy_rfagc_disable();
    }

    /* Boot wide for full analog-FM video bandwidth. Runtime BW20/AUTO is
     * explicitly opt-in from the native menu; BW40 remains the safe default. */
    extern void phy_wifi_fbw_sel(uint32_t val);
    phy_wifi_fbw_sel(s_analog_bw40 ? 1u : 0u);

    /* Force high-sensitivity sweet-spot gain (index 52).
     * Provides sensitive reception of weak carriers out of the box while
     * active AGC dynamically manages gain tracking and overload protection. */
    extern void phy_force_rx_gain(bool enable, uint8_t gain_idx);
    extern void phy_fft_scale_force(bool force_en, int8_t force_value);
    if (s_native_agc) {
        /* Release, never choose: the vendor loop owns RF/BB/fine gain. */
        phy_force_rx_gain(false, 0);
        phy_fft_scale_force(false, 0);
        native_initgain_apply();
        agc_tune_apply();
        agc_offset_apply();
        native_acq_apply();
    } else {
        phy_force_rx_gain(true, 52);
    }

    /* Vendor PHY initialization has now generated both valid RX gain tables
     * and completed its own calibration. Capture that state read-only before
     * C5VRX freezes receiver ownership. */
    arc_capture_vendor_state();

    /* Stop the PHY PLL / RXCAL tracking timer so it never recalibrates RF /
     * RX hardware during continuous analog video reception. A lab boot may
     * keep it running to A/B the native-AGC stuck-low state (#121 sec. 9). */
#if !CONFIG_ESP_PHY_DISABLE_PLL_TRACK
    if (!s_pll_track) {
        extern void phy_track_pll_deinit(void);
        phy_track_pll_deinit();
    }
#else
    s_pll_track = false;
#endif

    ESP_EARLY_LOGW(TAG, "RF ready: 5865 MHz / ch%u / BW40 / gain=%s / sta_disconnected_pm=0 / pll_track=%s",
                   RF_CHANNEL_NUMBER,
                   s_native_agc ? "NATIVE_HW_AGC(zero firmware writes)" : "forced(52)",
                   s_pll_track ? "ENABLED(lab)" : "disabled");
    return ESP_OK;
}

extern void phy_wifi_fbw_sel(uint32_t val);
extern void phy_force_rx_gain(bool enable, uint8_t gain_idx);
extern void phy_disable_agc(void);
extern void phy_rfagc_disable(void);
extern void phy_set_freq(uint16_t freq_mhz, int offset);
extern void phy_chip_set_chan_offset(int offset_khz);
extern void phy_fft_scale_force(bool force_en, int8_t force_value);

/* Read-only C5 PHY observations. Estimator/calibration routines are not called
 * while live because they reconfigure clocks and receive state. */
extern int phy_get_noise_floor(void) __attribute__((weak));
extern int phy_get_rssi(void) __attribute__((weak));

/* Standard FPV Channel Table: 6 Bands x 8 Channels = 48 Channels
 * RaceBand (R), Boscam A (A), Boscam B (B), Boscam E (E), FatShark (F), LowBand (L) */
static const fpv_channel_t s_fpv_channels[FPV_BAND_COUNT][8] = {
    [FPV_BAND_R] = { /* RaceBand (R1..R8) */
        { "R1", 5658 }, { "R2", 5695 }, { "R3", 5732 }, { "R4", 5769 },
        { "R5", 5806 }, { "R6", 5843 }, { "R7", 5880 }, { "R8", 5917 },
    },
    [FPV_BAND_A] = { /* Boscam A (A1..A8) - Default A1 is 5865 MHz */
        { "A1", 5865 }, { "A2", 5845 }, { "A3", 5825 }, { "A4", 5805 },
        { "A5", 5785 }, { "A6", 5765 }, { "A7", 5745 }, { "A8", 5725 },
    },
    [FPV_BAND_B] = { /* Boscam B (B1..B8) */
        { "B1", 5733 }, { "B2", 5752 }, { "B3", 5771 }, { "B4", 5790 },
        { "B5", 5809 }, { "B6", 5828 }, { "B7", 5847 }, { "B8", 5866 },
    },
    [FPV_BAND_E] = { /* Boscam E (E1..E8) */
        { "E1", 5705 }, { "E2", 5685 }, { "E3", 5665 }, { "E4", 5645 },
        { "E5", 5885 }, { "E6", 5905 }, { "E7", 5925 }, { "E8", 5945 },
    },
    [FPV_BAND_F] = { /* FatShark / Airwave (F1..F8) */
        { "F1", 5740 }, { "F2", 5760 }, { "F3", 5780 }, { "F4", 5800 },
        { "F5", 5820 }, { "F6", 5840 }, { "F7", 5860 }, { "F8", 5880 },
    },
    [FPV_BAND_L] = { /* LowBand / Band D (L1..L8) */
        { "L1", 5362 }, { "L2", 5399 }, { "L3", 5436 }, { "L4", 5473 },
        { "L5", 5510 }, { "L6", 5547 }, { "L7", 5584 }, { "L8", 5621 },
    },
};

static const char *s_band_names[FPV_BAND_COUNT] = {
    [FPV_BAND_R] = "RaceBand (R)",
    [FPV_BAND_A] = "Boscam A (A)",
    [FPV_BAND_B] = "Boscam B (B)",
    [FPV_BAND_E] = "Boscam E (E)",
    [FPV_BAND_F] = "FatShark (F)",
    [FPV_BAND_L] = "LowBand (L)",
};

static fpv_band_t s_current_band = FPV_BAND_A;
static uint8_t s_current_channel_idx = 0; /* 0..7 (Default A1: 5865 MHz) */
static uint16_t s_current_freq_mhz = 5865u;
static int s_current_offset_khz = 0;

#define C5_WIFI5_MIN_MHZ 5180u
#define C5_WIFI5_MAX_MHZ 5885u

typedef struct {
    uint8_t channel;
    uint16_t mhz;
} wifi5_center_t;

/* Public ESP-IDF 5 GHz centers used as the supported RF bootstrap.
 * Non-exact FPV centers are experimental and are retuned only after first
 * placing the closed PHY on the nearest known-good public center. */
static const wifi5_center_t s_wifi5_centers[] = {
    {132, 5660}, {136, 5680}, {140, 5700}, {144, 5720},
    {149, 5745}, {153, 5765}, {157, 5785}, {161, 5805},
    {165, 5825}, {169, 5845}, {173, 5865}, {177, 5885},
};

static bool plan_wifi5_center(uint16_t freq_mhz, uint8_t *channel, uint16_t *center_mhz)
{
    if (freq_mhz < C5_WIFI5_MIN_MHZ || freq_mhz > C5_WIFI5_MAX_MHZ) {
        return false;
    }

    unsigned best = 0;
    int best_delta = 0x7fffffff;
    for (unsigned i = 0; i < sizeof(s_wifi5_centers) / sizeof(s_wifi5_centers[0]); ++i) {
        int d = (int)freq_mhz - (int)s_wifi5_centers[i].mhz;
        if (d < 0) d = -d;
        if (d < best_delta) {
            best_delta = d;
            best = i;
        }
    }

    if (channel) *channel = s_wifi5_centers[best].channel;
    if (center_mhz) *center_mhz = s_wifi5_centers[best].mhz;
    return true;
}

void rf_set_analog_bandwidth(bool bw40)
{
    analog_agc_transition_begin();
    native_wdg_restore();
    s_analog_bw40 = bw40;
    phy_wifi_fbw_sel(bw40 ? 1u : 0u);
    analog_agc_transition_end();
}

bool rf_get_analog_bandwidth(void)
{
    return s_analog_bw40;
}

void rf_set_rx_gain(bool force, uint8_t gain_idx)
{
    /* Single choke point: in the native experiment every firmware gain write
     * is refused and counted, so a non-zero count taints that capture. */
    if (s_native_agc) {
        ++s_native_agc_blocked_writes;
        return;
    }
    if (force) {
        s_current_gain_val = gain_idx;
    }
    phy_force_rx_gain(force, gain_idx);
}

uint32_t rf_get_rx_gain_reg(void)
{
    return REG32(RX_GAIN_STATUS_REG);
}

void rf_get_phy_snapshot(rf_phy_snapshot_t *snapshot)
{
    if (!snapshot) return;

    snapshot->gain_reg = REG32(RX_GAIN_STATUS_REG);
    snapshot->rx_filter_reg = REG32(RX_FILTER_REG);
    snapshot->adc_rate_reg = REG32(ADC_RATE_REG);
    snapshot->source_mux_reg = REG32(SOURCE_MUX);
    snapshot->iq_correction_reg = REG32(RX_IQ_CORR_REG);
    snapshot->rx_filter_mode =
        (uint8_t)((snapshot->rx_filter_reg & RX_FILTER_MASK) >> RX_FILTER_SHIFT);
    snapshot->adc_rate_sel =
        (uint8_t)(snapshot->adc_rate_reg & ADC_RATE_SEL_MASK);
    snapshot->iq_correction =
        arc_iq_correction_decode(snapshot->iq_correction_reg);
    if (!arc_gain_tuple_decode(&s_arc_gain_table, s_current_gain_val,
                               &snapshot->gain_tuple)) {
        snapshot->gain_tuple = (arc_gain_tuple_t){0};
        snapshot->gain_tuple.gain_index = s_current_gain_val;
    }
}

static void arc_capture_vendor_state(void)
{
    arc_phy_capture_gain_table(&s_arc_gain_table);
    rf_get_phy_snapshot(&s_arc_receive_tuple);
    /* Publish last so readers never associate a new generation with a tuple
     * that is still being filled. The controller is the sole retune owner. */
    ++s_arc_generation;
}

const rf_phy_snapshot_t *rf_get_arc_receive_tuple(void)
{
    return &s_arc_receive_tuple;
}

void rf_set_fft_scale_force(bool force, int8_t value)
{
    if (s_native_agc && force) {
        ++s_native_agc_blocked_writes;
        return;
    }
    /* The symbol is exported by the ESP32-C5 ROM PHY and is also used by
     * Espressif's CSI gain-control design. Keep it lab-only: whether it is
     * upstream of raw MODEM_DIAG is exactly what the FFT probe measures. */
    phy_fft_scale_force(force, value);
}


bool rf_try_get_noise_floor_dbm(int *dbm)
{
    if (!dbm || !phy_get_noise_floor) return false;
    int value = phy_get_noise_floor();
    /* Reject impossible values instead of feeding an ABI mismatch into AUTO. */
    if (value < -140 || value > -20) return false;
    *dbm = value;
    return true;
}

bool rf_try_get_wideband_rssi_dbm(int *dbm)
{
    if (!dbm || !phy_get_rssi) return false;
    int value = phy_get_rssi();
    if (value < -140 || value > 10) return false;
    *dbm = value;
    return true;
}

const arc_gain_table_t *rf_get_arc_gain_table(void)
{
    return &s_arc_gain_table;
}

uint8_t rf_get_arc_survival_gain(void)
{
    return arc_gain_highest_rf_stage_start(&s_arc_gain_table);
}

uint32_t rf_get_arc_generation(void)
{
    return s_arc_generation;
}

const fpv_channel_t *rf_get_current_channel(void)
{
    return &s_fpv_channels[s_current_band][s_current_channel_idx];
}

size_t rf_get_channel_index(void)
{
    return (size_t)s_current_band * 8u + s_current_channel_idx;
}

size_t rf_get_channel_count(void)
{
    return FPV_BAND_COUNT * 8u;
}

fpv_band_t rf_get_current_band(void)
{
    return s_current_band;
}

const char *rf_get_band_name(fpv_band_t band)
{
    if (band >= FPV_BAND_COUNT) return "Unknown";
    return s_band_names[band];
}

uint8_t rf_get_current_channel_number(void)
{
    return s_current_channel_idx + 1u;
}

uint16_t rf_get_frequency_mhz(void)
{
    return s_current_freq_mhz;
}

int rf_get_frequency_offset_khz(void)
{
    return s_current_offset_khz;
}

void rf_set_frequency_offset_khz(int offset_khz)
{
    analog_agc_transition_begin();
    native_wdg_restore();
    /* Strict clamping: +/- 1500 kHz (+/- 1.5 MHz) maximum.
     * Adjacent FPV channels are at least 19-20 MHz apart. Clamping strictly
     * to +/- 1.5 MHz guarantees 100% that tuning is locked to the selected
     * channel and can NEVER hop or switch to another channel. */
    if (offset_khz < -1500) offset_khz = -1500;
    if (offset_khz > 1500)  offset_khz = 1500;

    s_current_offset_khz = offset_khz;
    phy_chip_set_chan_offset(offset_khz);
    if (!s_native_agc) phy_force_rx_gain(true, s_current_gain_val);
    analog_agc_transition_end();
}

void rf_step_frequency_offset_khz(int delta_khz)
{
    rf_set_frequency_offset_khz(s_current_offset_khz + delta_khz);
}

esp_err_t rf_set_channel(size_t index)
{
    if (index >= FPV_BAND_COUNT * 8u) {
        return ESP_ERR_INVALID_ARG;
    }

    fpv_band_t new_band = (fpv_band_t)(index / 8u);
    uint8_t new_idx = (uint8_t)(index % 8u);
    uint16_t requested_mhz = s_fpv_channels[new_band][new_idx].freq_mhz;
    analog_agc_cancel();
    native_wdg_restore();

    uint8_t wifi_channel = 0;
    uint16_t wifi_center_mhz = 0;
    if (!plan_wifi5_center(requested_mhz, &wifi_channel, &wifi_center_mhz)) {
        printf("[RF:TUNE] Refusing %u MHz: outside ESP32-C5 5 GHz operating window %u-%u MHz\n",
               requested_mhz, C5_WIFI5_MIN_MHZ, C5_WIFI5_MAX_MHZ);
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* Always establish a supported/public RF center first. Exact-overlap FPV
     * channels (e.g. A1/A2/...) need no undocumented frequency call at all. */
    analog_agc_transition_begin();
    esp_err_t err = esp_wifi_set_channel(wifi_channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        analog_agc_transition_end();
        return err;
    }

    uint8_t verify_primary = 0;
    wifi_second_chan_t verify_secondary = WIFI_SECOND_CHAN_NONE;
    err = esp_wifi_get_channel(&verify_primary, &verify_secondary);
    if (err != ESP_OK || verify_primary != wifi_channel) {
        analog_agc_transition_end();
        return (err != ESP_OK) ? err : ESP_ERR_INVALID_STATE;
    }

    if (requested_mhz != wifi_center_mhz) {
        /* EXPERIMENTAL: two-argument ABI is known, but arbitrary-frequency
         * semantics still require RF hardware validation. Starting from the
         * nearest public center minimizes the size of this undocumented step. */
        phy_set_freq(requested_mhz, 0);
    }

    rf_enable_continuous_modem();

    /* Public/undocumented retune paths can touch PHY receive state. Re-assert
     * the currently selected analog bandwidth after every channel change. */
    if (s_native_agc) {
        /* Restore native-owned state symmetrically with rf_start(): vendor
         * retune paths may touch digital/baseband scaling (#121 section 8). */
        s_bb_agc_off = false;
        phy_force_rx_gain(false, 0);
        phy_fft_scale_force(false, 0);
        native_initgain_apply();
        agc_tune_apply();
        agc_offset_apply();
        native_acq_apply();
    } else {
        phy_disable_agc();
        phy_rfagc_disable();
    }
    phy_wifi_fbw_sel(s_analog_bw40 ? 1u : 0u);
    if (!s_native_agc) phy_force_rx_gain(true, s_current_gain_val);

    /* A channel change may make the vendor PHY regenerate its active RX gain
     * table and calibrated receive state. Recapture only after the retune and
     * all receive-state reassertions succeeded, then publish one generation
     * change so ARC cannot keep stale spans/maxima or temporal state. */
    arc_capture_vendor_state();

    /* Commit logical state only after the supported bootstrap succeeded. */
    s_current_band = new_band;
    s_current_channel_idx = new_idx;
    s_current_freq_mhz = requested_mhz;
    s_current_offset_khz = 0;

    analog_agc_transition_end();
    return ESP_OK;
}

esp_err_t rf_cycle_channel(void)
{
    size_t start = rf_get_channel_index();
    for (size_t step = 1; step <= FPV_BAND_COUNT * 8u; ++step) {
        size_t next = (start + step) % (FPV_BAND_COUNT * 8u);
        esp_err_t err = rf_set_channel(next);
        if (err == ESP_OK) return ESP_OK;
        if (err != ESP_ERR_NOT_SUPPORTED) return err;
    }
    return ESP_ERR_NOT_FOUND;
}

void rf_cycle_band(void)
{
    fpv_band_t start_band = s_current_band;
    uint8_t channel_idx = s_current_channel_idx;
    for (unsigned step = 1; step <= FPV_BAND_COUNT; ++step) {
        fpv_band_t band = (fpv_band_t)((start_band + step) % FPV_BAND_COUNT);
        esp_err_t err = rf_set_channel((size_t)band * 8u + channel_idx);
        if (err == ESP_OK) return;
        if (err != ESP_ERR_NOT_SUPPORTED) return;
    }
}

void rf_cycle_channel_in_band(void)
{
    fpv_band_t band = s_current_band;
    uint8_t start_idx = s_current_channel_idx;
    for (unsigned step = 1; step <= 8u; ++step) {
        uint8_t idx = (uint8_t)((start_idx + step) % 8u);
        esp_err_t err = rf_set_channel((size_t)band * 8u + idx);
        if (err == ESP_OK) return;
        if (err != ESP_ERR_NOT_SUPPORTED) return;
    }
}
