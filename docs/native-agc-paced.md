# Scheduled native AGC windows (2026-09-30, PR #122)

User requested: let native AGC choose gain briefly, then pause tracking between
updates. This explicitly changes the earlier continuous-native-only requirement.
Hardware still selects gain; firmware does not calculate or force an index.

## Implementation

The experimental normal build boots FULL Phase8, coarse IQ and a 1000 us
scheduled release period with a 20 us open window. Nominally it holds tracking
for the remaining 980 us. Both values are starting test settings, not an
established optimum or a measured acquisition duration. The operator-selected
7034=127 acquisition profile remains separate.

`native_agc_pace.c` uses a 1 MHz GPTimer and two short interrupt callbacks per
cycle. It modifies only `0x600A7030[29]`: set pauses BB AGC; clear releases it.
The owned bit is confirmed by the pinned PHY disable/enable disassembly and
the earlier live BB-AGC-off experiment. Periodic release deliberately does not
pulse `702C[23]`, force a gain, or reset the BB FSM. Whether clearing the gate
alone reliably resumes tracking must be verified on this board.

IQ/PARLIO/BitScrambler/DAC continue running. There is no sample HOLD, replacement
or DSP filter. Interrupt arrival delays can extend the open window and release
period; `T` reports observed timer lateness and maximum open duration. The first
window begins at timer start after the continuous vendor boot setup.

Retunes suspend pacing and release AGC for vendor calibration, then restart
the selected cadence. Fine IQ, another demodulator, nonzero level offset or the
manual BB-AGC lab switch disable pacing. The 8020 trial refuses pacing, and
starting pacing cancels an active 8020 trial. Timer errors release the AGC gate.
Configured-but-inactive is distinguishable by `running=0` and error reporting.

## Controls

| Key | Action |
| --- | --- |
| `~` | Toggle paced / continuous native AGC; disabling always allowed |
| `:` | Cycle period: 250, 500, 1000, 2000, 5000, 10000, 20000 us |
| `;` | Cycle window: 10, 20, 50, 100, 200 us |
| `T` | Print `AGC_PACE` configuration and timer diagnostics |

Period/window changes start pacing. Choices are RAM-only; reboot uses build
defaults. Kconfig controls boot enable, period and window. Disable
`C5VRX_NATIVE_AGC_PACED` for the normal selectable-demod/continuous boot path.
The standalone meter does not start pacing by default.

## First board comparison

This build has compiled; picture improvement and native resume behavior have
not yet been verified. Require FULL Phase8, the same channel/carrier and 127
profile for continuous -> paced -> continuous. Confirm `running=1` and rising
`opens` in `T`; these count scheduled releases, **not actual gain acquisitions**.
Check both weaker and stronger RF: a clean stationary image with stuck gain is
not success. Stop via `~` if response stalls, overload appears, or periodic
lines/glitches increase. Longer periods increase the worst-case delay before
the next scheduled opportunity to adjust; overload bypass is not implemented.
The earlier native-off test found no useful autonomous overload correction
while BB gain was frozen. No automatic quality/range benefit is claimed.

The operator's approximately 5 us current update estimate is not a validated
gain-acquisition interval. Status transitions must not be equated to gain
writes. This experiment schedules tracking opportunities and cannot promise
one completed native acquisition per window or exactly one gain change.
