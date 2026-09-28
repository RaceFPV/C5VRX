# Issue 103: live Phase8-HR visual test

This branch loads `fm_phase8_hr_live.bsasm` directly for the live 6BIT@40
video path. It forces 6BIT@40 after loading saved settings and does not let
the on-screen menu switch to 4BIT@80. The normal mainline Phase5 program is
not a runtime fallback in this build. The finite Phase8 arithmetic oracle
still runs at boot and its result can be read with USB `p`.

The live program reuses the two-bundle, eight-slot Counter-A schedule proved
by PR #102. It maps every Q4/I4 endpoint to Phase8, loads
`(76 - 2*previous) mod 256`, adds `2*current`, and emits counter bits 10..15
twice. This gives `floor((76 + 2*delta_phase8)/4)` in the linear region:
signed Phase8 deltas -38 through +89. In particular, -32 produces DAC 3
instead of the DAC 60 produced by the original bias-80/multiplier-3 oracle.

This is an **experimental lower-swing mapping**, not a safe full-range
saturator. Deltas outside the linear region wrap. The host test covers all
65,536 ordered raw endpoint pairs: 32,768 are in the linear region, 23,247
wrap below it, and 9,521 wrap above it. This uniform pair count is not a
prediction of their frequency in a real camera signal. Full-gain calibration,
range guarding, and live picture quality remain open.

Validation before flash:

```text
python tools/gen_phase8_hr_live.py
python tools/test_phase8_hr_live.py
python tools/validate_build.py
idf.py build
```

For the live A/B, check picture lock, sync tears, color, sharpness, static,
and close/weak RF behavior. USB `p` reports transport counters; verify
`tx_empty`, `rx_ovf`, `gdma_in`, `gdma_out`, `bs_empty`, and `bs_eof` stay zero.
Stop the experiment and restore the previous firmware if the picture becomes
unusable or sync is unstable.
