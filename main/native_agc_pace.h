#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* User-requested experiment: hardware chooses gain, firmware gates tracking.
 * Period/window are scheduled times, not measured PHY acquisition times. */
esp_err_t native_agc_pace_set(uint32_t period_us, uint32_t acquire_us);
void native_agc_pace_suspend(void);
esp_err_t native_agc_pace_resume(void);
bool native_agc_pace_enabled(void);
void native_agc_pace_report(void);
void native_agc_pace_cycle(bool window);
void native_agc_pace_toggle(void);
