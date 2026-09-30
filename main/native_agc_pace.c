#include "native_agc_pace.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_private/esp_clk.h"
#include "freertos/FreeRTOS.h"
#include <stdio.h>

#define AGC_CTRL (*(volatile uint32_t *)0x600A7030u)
#define FORCE_CTRL (*(volatile uint32_t *)0x600A702Cu)
#define HOLD_BIT (1u << 29)
/* Native AGC state bytes (docs/native-agc-v2.md): 7078[7:0] re-enters the
 * start gain when an acquisition begins; 706C[7:0] follows the gain walk. */
#define AGC_STATE_B (*(volatile uint32_t *)0x600A7078u)
#define AGC_STATE_A (*(volatile uint32_t *)0x600A706Cu)
#define AGC_INITGAIN_REG (*(volatile uint32_t *)0x600A7094u)
/* window 0 = exactly one native acquisition: open, wait for it to start,
 * close once the gain has held still for ACQ1_STILL_US. */
#define ACQ1_STILL_US   1u
#define ACQ1_TIMEOUT_US 60u
static gptimer_handle_t s_timer;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_period = 1000, s_window = 20;
static bool s_enabled, s_running, s_acquiring;
static uint64_t s_opened, s_opens, s_skipped;
static uint32_t s_late_max, s_open_max, s_faults;
/* Windows below this are timed inside the ISR on the CPU cycle counter:
 * interrupt latency (~3-5 us) would otherwise dominate a timer-alarm close. */
#define SHORT_WINDOW_US 10u
static uint32_t s_acq1_no_start;
static uint32_t s_cycles_per_us = 240;

static void IRAM_ATTR gate(bool hold)
{
    uint32_t word = AGC_CTRL;
    AGC_CTRL = hold ? word | HOLD_BIT : word & ~HOLD_BIT;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

static bool IRAM_ATTR alarm(gptimer_handle_t timer,
                           const gptimer_alarm_event_data_t *event, void *ctx)
{
    (void)ctx;
    uint64_t now, next;
    portENTER_CRITICAL_ISR(&s_lock);
    if (!s_running) {
        portEXIT_CRITICAL_ISR(&s_lock);
        return false;
    }
    uint64_t late = event->count_value > event->alarm_value ?
                    event->count_value - event->alarm_value : 0;
    if (late > s_late_max) s_late_max = late > UINT32_MAX ? UINT32_MAX : late;
    if (s_acquiring) {
        gate(true);
        if (gptimer_get_raw_count(timer, &now) != ESP_OK) {
            gate(false);
            s_running = s_acquiring = false;
            ++s_faults;
            portEXIT_CRITICAL_ISR(&s_lock);
            return false;
        }
        uint64_t duration = now - s_opened;
        if (duration > s_open_max)
            s_open_max = duration > UINT32_MAX ? UINT32_MAX : duration;
        next = s_opened + s_period;
        if (next <= now) {
            uint64_t missed = (now - next) / s_period + 1;
            s_skipped += missed;
            next += missed * s_period;
        }
        s_acquiring = false;
    } else {
        /* Resume the gate only. Do not pulse force-gain or reset the FSM.
         * Whether this resumes useful native tracking needs board validation. */
        if (s_window < SHORT_WINDOW_US) {
            uint32_t start = esp_cpu_get_cycle_count();
            if (s_window == 0) {
                /* One acquisition: wait for its start (7078 at the start gain),
                 * then for the gain to hold still, then close. */
                uint8_t init = (uint8_t)((AGC_INITGAIN_REG >> 2) & 0x7Fu);
                uint32_t timeout = ACQ1_TIMEOUT_US * s_cycles_per_us;
                uint32_t still = ACQ1_STILL_US * s_cycles_per_us;
                gate(false);
                bool started = false;
                uint8_t last = (uint8_t)AGC_STATE_A;
                uint32_t since = esp_cpu_get_cycle_count();
                while (esp_cpu_get_cycle_count() - start < timeout) {
                    uint32_t t = esp_cpu_get_cycle_count();
                    if (!started) {
                        if ((uint8_t)AGC_STATE_B == init) { started = true; since = t; }
                        continue;
                    }
                    uint8_t g = (uint8_t)AGC_STATE_A;
                    if (g != last || (uint8_t)AGC_STATE_B == init) { last = g; since = t; }
                    else if (t - since >= still) break;
                }
                gate(true);
                if (!started) ++s_acq1_no_start;
            } else {
                /* Exact short window: open, spin on the cycle counter, close. */
                uint32_t span = s_window * s_cycles_per_us;
                gate(false);
                while (esp_cpu_get_cycle_count() - start < span) { }
                gate(true);
            }
            uint32_t spent = (esp_cpu_get_cycle_count() - start) / s_cycles_per_us;
            if (spent > s_open_max) s_open_max = spent;
            ++s_opens;
            if (gptimer_get_raw_count(timer, &now) != ESP_OK) {
                gate(false);
                s_running = s_acquiring = false;
                ++s_faults;
                portEXIT_CRITICAL_ISR(&s_lock);
                return false;
            }
            next = (event->alarm_value) + s_period;
            if (next <= now) {
                uint64_t missed = (now - next) / s_period + 1;
                s_skipped += missed;
                next += missed * s_period;
            }
            portEXIT_CRITICAL_ISR(&s_lock);
            gptimer_alarm_config_t short_cfg = {.alarm_count = next};
            if (gptimer_set_alarm_action(timer, &short_cfg) != ESP_OK) {
                portENTER_CRITICAL_ISR(&s_lock);
                gate(false);
                s_running = s_acquiring = false;
                ++s_faults;
                portEXIT_CRITICAL_ISR(&s_lock);
            }
            return false;
        }
        gate(false);
        if (gptimer_get_raw_count(timer, &now) != ESP_OK) {
            s_running = s_acquiring = false;
            ++s_faults;
            portEXIT_CRITICAL_ISR(&s_lock);
            return false;
        }
        s_opened = now;
        ++s_opens;
        s_acquiring = true;
        next = now + s_window;
    }
    portEXIT_CRITICAL_ISR(&s_lock);
    gptimer_alarm_config_t cfg = {.alarm_count = next};
    if (gptimer_set_alarm_action(timer, &cfg) != ESP_OK) {
        portENTER_CRITICAL_ISR(&s_lock);
        gate(false); /* Fail back to continuous native tracking. */
        s_running = s_acquiring = false;
        ++s_faults;
        portEXIT_CRITICAL_ISR(&s_lock);
    }
    return false;
}

void native_agc_pace_suspend(void)
{
    portENTER_CRITICAL(&s_lock);
    bool running = s_running;
    s_running = false;
    if (running) gate(false);
    s_acquiring = false;
    portEXIT_CRITICAL(&s_lock);
    if (s_timer) {
        (void)gptimer_stop(s_timer);
        (void)gptimer_set_alarm_action(s_timer, NULL);
    }
}

esp_err_t native_agc_pace_resume(void)
{
    if (!s_enabled) return ESP_OK;
    if (s_running) return ESP_OK;
    /* Never take ownership of a pre-existing freeze or forced gain. */
    if ((AGC_CTRL & HOLD_BIT) || (FORCE_CTRL & (1u << 23)))
        return ESP_ERR_INVALID_STATE;
    esp_err_t err;
    if (!s_timer) {
        gptimer_config_t config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000,
            .intr_priority = 2,
        };
        if ((err = gptimer_new_timer(&config, &s_timer)) != ESP_OK) return err;
        gptimer_event_callbacks_t cb = {.on_alarm = alarm};
        err = gptimer_register_event_callbacks(s_timer, &cb, NULL);
        if (err == ESP_OK) err = gptimer_enable(s_timer);
        if (err != ESP_OK) {
            gptimer_del_timer(s_timer);
            s_timer = NULL;
            return err;
        }
    }
    if ((err = gptimer_set_raw_count(s_timer, 0)) != ESP_OK) return err;
    s_cycles_per_us = (uint32_t)(esp_clk_cpu_freq() / 1000000);
    bool short_window = s_window < SHORT_WINDOW_US;
    /* Short windows start held and open exactly at the first alarm; longer
     * ones start open and close at the first alarm, as before. */
    gptimer_alarm_config_t cfg = {.alarm_count = short_window ? s_period : s_window};
    if ((err = gptimer_set_alarm_action(s_timer, &cfg)) != ESP_OK) return err;
    portENTER_CRITICAL(&s_lock);
    s_opened = 0;
    s_running = true;
    s_acquiring = !short_window;
    if (short_window) gate(true);
    else ++s_opens;
    portEXIT_CRITICAL(&s_lock);
    err = gptimer_start(s_timer);
    if (err != ESP_OK) native_agc_pace_suspend();
    return err;
}

esp_err_t native_agc_pace_set(uint32_t period, uint32_t window)
{
    if (period && (period < 250 || period > 20000 ||
                   window > 200 || window >= period)) return ESP_ERR_INVALID_ARG;
    native_agc_pace_suspend();
    portENTER_CRITICAL(&s_lock);
    if (period) {
        s_period = period;
        s_window = window;
    }
    s_enabled = period != 0;
    portEXIT_CRITICAL(&s_lock);
    return native_agc_pace_resume();
}

bool native_agc_pace_enabled(void) { return s_enabled; }

void native_agc_pace_report(void)
{
    portENTER_CRITICAL(&s_lock);
    bool enabled = s_enabled, running = s_running, open = s_acquiring;
    uint64_t opens = s_opens, skipped = s_skipped;
    uint32_t late = s_late_max, duration = s_open_max, faults = s_faults;
    uint32_t ctrl = AGC_CTRL;
    portEXIT_CRITICAL(&s_lock);
    printf("AGC_PACE enabled=%u running=%u acquiring=%u period_us=%lu "
           "window_us=%lu opens=%llu skipped=%llu late_max_us=%lu "
           "open_max_us=%lu faults=%lu acq1_no_start=%lu ctrl=0x%08lx ram_only=1\n",
           enabled, running, open, (unsigned long)s_period,
           (unsigned long)s_window, (unsigned long long)opens,
           (unsigned long long)skipped, (unsigned long)late,
           (unsigned long)duration, (unsigned long)faults,
           (unsigned long)s_acq1_no_start, (unsigned long)ctrl);
}

void native_agc_pace_cycle(bool window)
{
    /* 16683 us = one NTSC field and 20000 us = one PAL field: the open window
     * then lands on nearly the same line every field instead of wandering. */
    static const uint32_t periods[] = {250, 500, 1000, 2000, 5000, 10000, 16683, 20000};
    static const uint32_t windows[] = {0, 1, 2, 3, 5, 10, 20, 50, 100, 200};
    const uint32_t *values = window ? windows : periods;
    unsigned count = window ? sizeof(windows)/sizeof(windows[0]) :
                              sizeof(periods)/sizeof(periods[0]);
    uint32_t current = window ? s_window : s_period;
    unsigned index = 0;
    while (index + 1 < count && values[index] != current) ++index;
    uint32_t next = values[(index + 1) % count];
    esp_err_t err = native_agc_pace_set(window ? s_period : next,
                                      window ? next : s_window);
    printf("AGC_PACE configure=%s err=%s\n", window ? "window" : "period",
           esp_err_to_name(err));
    native_agc_pace_report();
}

void native_agc_pace_toggle(void)
{
    esp_err_t err = native_agc_pace_set(s_enabled ? 0 : s_period, s_window);
    printf("AGC_PACE toggle err=%s\n", esp_err_to_name(err));
    native_agc_pace_report();
}
