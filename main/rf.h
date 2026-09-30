#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "arc_phy.h"

/**
 * rf_start() - Initialize the ESP32-C5 Wi-Fi/PHY receive-only frontend.
 *
 * Configures:
 *   - 5 GHz band only (WIFI_BAND_MODE_5G_ONLY)
 *   - Channel 173 = 5865 MHz / BW40
 *   - No power saving (WIFI_PS_NONE)
 *   - Promiscuous RX to keep MODEM_DIAG active
 *   - All 5 LMAC TX queues hardware-disabled (receive-only)
 *   - MODEM_DIAG DIAG[6:9] (Q[9:6]) and DIAG[16:19] (I[9:6]) routed to GPIO
 *
 * Returns ESP_OK on success.
 * Returns an error if BW40 is not available -- NO BW20 fallback.
 * Returns an error if channel verification fails.
 */
esp_err_t rf_start(void);

/**
 * Erase only Espressif's stored PHY calibration namespace. This does not
 * recalibrate the already-running receiver; callers must reboot afterwards so
 * the next Wi-Fi/PHY initialization performs a fresh calibration.
 *
 * PRE-Q4 lab only: production must never trigger calibration while live video
 * owns the RF chain.
 */
esp_err_t rf_prepare_fresh_phy_calibration(void);

/**
 * Dump all vendor timers intercepted during Wi-Fi operation.
 */
void rf_dump_tracked_timers(void);

/**
 * Control PHY receiver frontend gain.
 * force = true sets fixed gain index (0 = min gain / max attenuation, ~30-60 = high gain).
 * force = false restores automatic / default PHY gain.
 */
/** Runtime analog receive filter: true=BW40, false=BW20. */
void rf_set_analog_bandwidth(bool bw40);
bool rf_get_analog_bandwidth(void);

void rf_set_rx_gain(bool force, uint8_t gain_idx);
uint32_t rf_get_rx_gain_reg(void);

/**
 * Read-only snapshot of known ESP32-C5 RX PHY registers used for lab
 * characterization. No undocumented PHY function is called by this helper.
 */
typedef struct {
    uint32_t gain_reg;
    uint32_t rx_filter_reg;
    uint32_t adc_rate_reg;
    uint32_t source_mux_reg;
    uint32_t iq_correction_reg;
    uint8_t rx_filter_mode;
    uint8_t adc_rate_sel;
    arc_iq_correction_t iq_correction;
    arc_gain_tuple_t gain_tuple;
} rf_phy_snapshot_t;

void rf_get_phy_snapshot(rf_phy_snapshot_t *snapshot);
/* Vendor-calibrated, read-only receive tuple captured atomically with the ARC
 * gain table after PHY init or a successful channel retune. It is diagnostic
 * state only; undocumented fields are never swept or reapplied in LOCK. */
const rf_phy_snapshot_t *rf_get_arc_receive_tuple(void);

/**
 * Force/release the PHY FFT scaling stage. This is exposed only so the lab can
 * prove whether FFT gain sits upstream or downstream of the MODEM_DIAG Q4/I4
 * tap. Production receive control must not depend on this until hardware data
 * demonstrates a Q4/I4 effect.
 */
void rf_set_fft_scale_force(bool force, int8_t value);

/**
 * Issue #117/#119 native hardware AGC: the default gain owner.
 *
 * Firmware gain control is the opt-in fallback. The choice applies to the NEXT
 * boot only (persisted in NVS, caller reboots); no stored value means native.
 * When
 * active, rf_start() never calls phy_disable_agc()/phy_rfagc_disable(), releases
 * forced gain and FFT scale once, and every later rf_set_rx_gain() or FFT force
 * is refused and counted in blocked_writes. The registers are raw read-only
 * telemetry; their field semantics are not yet decoded.
 */
typedef struct {
    bool active;
    uint32_t gain_status_reg;  /* 0x600A702C: forced-gain / index state */
    uint32_t agc_ctrl_reg;     /* 0x600A7030: phy_disable_agc() state */
    uint32_t blocked_writes;
} rf_native_agc_state_t;

esp_err_t rf_request_native_agc_boot(bool enable);
/* #121 section 9 lab A/B: keep PHY PLL/RXCAL tracking running next boot. */
esp_err_t rf_request_pll_track_boot(bool enable);
bool rf_pll_track_active(void);
/* #121 native AGC start gain (0x600A7094[8:2]): 0 = vendor value, else
 * 40..vendor. Persisted; applied at boot, on retune and immediately. */
esp_err_t rf_set_native_initgain(uint8_t gain);
uint8_t rf_native_initgain(void);
uint8_t rf_native_initgain_vendor(void);
void rf_dump_agc_regs(void);
void rf_poll_agc_live(void);
/* LAB (#121): 'P' select next candidate AGC field, 'M'/'J' step it up/down,
 * 'B' restore all captured vendor values. Prints the selected field. */
void rf_lab_agc_field(char action);

/* Read-only state-byte statistics. Neither gain-index nor restart semantics
 * have been validated against per-sample C5 gain metadata. */
typedef struct {
    uint8_t median, min, max;
    uint32_t switches;
    uint32_t switches_per_ms_x10;
    uint32_t restarts;            /* 7078 re-entries into configured start byte; proxy only */
    uint32_t restarts_per_ms_x10;
    uint32_t elapsed_us;
} rf_native_gain_stats_t;
void rf_native_gain_stats(unsigned samples, rf_native_gain_stats_t *out);
/* Median native AGC state byte (706C[7:0]) over ~0.4 ms: the gain index the
 * hardware applied. Telemetry only (signal meter), never a control input. */
uint8_t rf_native_gain_index(void);
/* Issue #123 sign-preserving fine IQ lanes (I/Q {9,7,6,5}); false = the
 * production top four bits. Folds above |x| >= 256 per axis. Selected by
 * the P8 FINE demodulator only. */
void rf_set_fine_iq(bool fine);
/* 0 = coarse {9,8,7,6}, 1 = fine {9,7,6,5}, 2 = ultrafine {9,6,5,4}. */
void rf_set_iq_lanes(uint8_t level);
uint8_t rf_iq_lane_level(void);
/* Native AGC level offset (dB against vendor) on one candidate field;
 * the hardware AGC still makes every gain decision. See agc_offset.h. */
void rf_agc_offset_set(int db);
int rf_agc_offset_db(void);
esp_err_t rf_agc_offset_save(void);
void rf_agc_offset_next_field(void);
const char *rf_agc_offset_field_name(void);
/* LAB: native AGC policy register candidate (NVS index, 0 = vendor). */
void rf_agc_tune_report(void);
uint8_t rf_agc_tune_index(void);
esp_err_t rf_agc_tune_select_next(void);
/* Native AGC acquisition lab: one policy field per boot, no forced gain. */
esp_err_t rf_native_acq_arm(bool next);
esp_err_t rf_native_acq_arm_max(void);
void rf_native_acq_report(void);
/* RAM-only lab switch: suppress BB watchdog resets, never disable AGC.
 * Reboot or retune restores vendor. Status is raw, not an event count. */
esp_err_t rf_native_wdg_toggle(void);
void rf_native_wdg_report(const char *reason);
/* Explicit RAM-only 20 s BB-policy trial: 0 cancel, 1 raw +32, 2 raw -32.
 * Native gain owns RX throughout. Raw field semantics remain unvalidated. */
void rf_analog_agc_request(unsigned profile);
bool rf_analog_agc_service(void);
void rf_analog_agc_report(const char *reason);
/* LAB: baseband packet AGC on/off with the RF AGC left on (native only). */
void rf_set_bb_agc(bool enable);
bool rf_bb_agc_enabled(void);
bool rf_fine_iq_active(void);
bool rf_native_agc_active(void);
void rf_get_native_agc_state(rf_native_agc_state_t *state);

/**
 * Experimental PHY observability/control used only by explicit RX profiles.
 * These wrappers keep undocumented symbols isolated in rf.c.
 *
 * Hardware AGC mode is opt-in and reversible. Normal C5VRX operation keeps
 * Espressif packet AGC disabled and uses the fixed-gain C5VRX controller.
 */
bool rf_try_get_noise_floor_dbm(int *dbm);
bool rf_try_get_wideband_rssi_dbm(int *dbm);
const arc_gain_table_t *rf_get_arc_gain_table(void);
uint8_t rf_get_arc_survival_gain(void);
/* Changes after every successful PHY init/channel retune and lets the ARC
 * supervisor discard controller state derived from an older vendor table. */
uint32_t rf_get_arc_generation(void);

/**
 * FPV Channel and Carrier Frequency Fine-Tuning:
 */
typedef struct {
    const char *name;     /* e.g. "A1", "R5", etc. */
    uint16_t freq_mhz;   /* Base channel center frequency in MHz */
} fpv_channel_t;

typedef enum {
    FPV_BAND_R = 0,  /* RaceBand (R1..R8) */
    FPV_BAND_A = 1,  /* Boscam A (A1..A8) */
    FPV_BAND_B = 2,  /* Boscam B (B1..B8) */
    FPV_BAND_E = 3,  /* Boscam E (E1..E8) */
    FPV_BAND_F = 4,  /* FatShark / Airwave (F1..F8) */
    FPV_BAND_L = 5,  /* LowBand (L1..L8) */
    FPV_BAND_COUNT = 6
} fpv_band_t;

const fpv_channel_t *rf_get_current_channel(void);
size_t rf_get_channel_index(void);
size_t rf_get_channel_count(void);
esp_err_t rf_set_channel(size_t index);
esp_err_t rf_cycle_channel(void);

fpv_band_t rf_get_current_band(void);
const char *rf_get_band_name(fpv_band_t band);
void rf_cycle_band(void);
uint8_t rf_get_current_channel_number(void);
void rf_cycle_channel_in_band(void);

uint16_t rf_get_frequency_mhz(void);
int rf_get_frequency_offset_khz(void);
void rf_set_frequency_offset_khz(int offset_khz);
void rf_step_frequency_offset_khz(int delta_khz);
