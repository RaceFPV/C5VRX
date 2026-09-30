# C5VRX-5: a range-first receive pipeline inside the ESP32-C5 (research)

Status (2026-09-30): **research and design only.** No firmware, LUT, PHY
setting or board change is made by this document. It proposes a new pipeline
for more range and less grain, ranks the levers by expected dB, and gives the
measurements that must decide each one. The numbers marked *model* come from
a small offline model built on the repo's own signal generator
(`tools/hc_estimator_bench.py`) and the live Phase8 LUT; they are not
hardware results.

Scope set by the operator: everything may change, provided it fits the
ESP32-C5 (XIAO board, one C5, existing resistor DAC). Native hardware AGC is
the preferred gain owner. The C5VRX-4 experiment (`experiments/c5vrx-4`) is
evaluated here too.

## Summary

1. **Nobody has measured the sensitivity yet.** No document in the repo gives
   the RF input (dBm or attenuator dB) at which the picture fails, for C5VRX or
   for a reference receiver. "Range is bad" can be 4 dB or 20 dB. The first
   deliverable must be a calibrated attenuator comparison against the goggle's
   own receiver (section 7, step 0). Every lever below is only as real as that
   comparison.
2. **What the chip can realistically add:** about **4-5 dB** of threshold in
   total from firmware and PHY settings (roughly 1.6-1.8x distance). The
   biggest share is a **~±9 MHz pre-detection filter**, followed by a
   history-conditioned decoder and finer IQ lanes. A PLL tracking demodulator
   gives no benefit at this deviation (*model*).
3. **"VEEL beter" range needs the antenna as well.** A 5.8 GHz RHCP antenna on
   the existing U.FL is +3-6 dB for a few euros and needs no firmware. With the
   chip-internal levers that is roughly 7-10 dB (2.2-3x distance). An external
   LNA or a second C5 for diversity go further but leave the "one C5" scope.
4. **The grain has three concrete sources in the current path, and each has a
   fix:**
   - Under native AGC the trapped radius is ~135 raw codes. On the coarse tap
     `I[9:6]/Q[9:6]` bit 8 then just repeats the sign bit 9, so **the live
     native path (and C5VRX-4) effectively gets I3/Q3.** The sign-preserving
     fine tap `{9,7,6,5}` gets that bit back for free: luma error 4.0 -> 2.8
     IRE, chroma-band error 4.7 -> 3.6 IRE at a strong signal (*model*).
   - **Phase8 FULL uses about 17 of the 64 DAC codes for the whole video
     swing** (one code = 312.5 kHz ≈ 7.8 IRE at the bench's 40 kHz/IRE). That
     step size alone is a ~2.3 IRE rms quantization floor, and the goggle
     amplifies it together with the small video level. A saturating
     transfer with twice the slope halves the step (*model*: luma error 2.1 ->
     1.7 IRE on fine lanes).
   - The native packet AGC re-acquires every ~25-50 us with 2.4-3.4 us
     saturated walks (docs/native-agc-v2.md). No demodulator removes that; it
     has to be stopped in the PHY.
5. **C5VRX-4 assessment:** keep its paced-native result (operator: "een stuk
   beter") as evidence that stopping re-acquisition is the right target. Do
   not continue span-75/Phase6: it gives up CFO/deviation headroom and
   precision, it still uses the coarse tap (I3/Q3 in practice under native
   AGC), and the operator saw a less clean picture than C5VRX-3.

## 1. Hard limits that shape the design

| Limit | Value | Source |
|---|---|---|
| Live IQ capture | 8 PARLIO RX lines, 40 MS/s (bus is 80 MS/s, every second sample) | `SOC_PARLIO_RX_UNIT_MAX_DATA_WIDTH 8` (IDF v6.0.2 soc_caps); continuous-iq-findings.md |
| IQ bits on the bus | `DIAG[0..9]` = Q[9:0], `DIAG[10..19]` = I[9:0]; `DIAG[20..31]` toggle only with a carrier, identity unproven | continuous-iq-findings.md, phase5-360-comprehensive-findings.md |
| DSP engine | one BitScrambler, 8 instruction slots, one bundle per 40 MHz cycle, one 1024x16 LUT, lookup result in the next bundle | c5vrx-4/RESEARCH.md, TRM ch. 44 |
| Throughput | 2 bundles per 20 MS/s output pair; more bundles only by lowering the unique output rate | c5vrx-4/RESEARCH.md |
| CPU | cannot touch samples at rate (sync flywheel measured ~0.5 us per code, range-max.md) | range-max.md |
| Output | 6-bit resistor DAC, PARLIO TX 40 MHz, `[D,D]` hold | README |
| Native AGC | 802.11 packet AGC: detect -> 2.4-3.4 us walk -> trapped gain -> abort -> re-detect, every ~25-50 us; trapped radius ~135 raw, spread 80-440 | native-agc-v2.md |
| BW20 filter | about ±8-9 MHz; as a fixed mode it caused Hanover bars and detail loss | fix-cvbs-jitter-and-static.md, pr-derived-findings.md |

External DSP/FPGA (c5vrx-4/RESEARCH.md, "wider architecture") is out of
scope for this proposal.

## 2. Where the range goes (loss budget)

Reference: a conventional analog FPV receiver with ~18 MHz IF, analog limiter
and discriminator, video LPF, and an RHCP antenna.

| Loss vs reference | Estimate | Evidence level | Chip-fixable? |
|---|---|---|---|
| Linear dual-band antenna vs RHCP VTX | 3-6 dB | physics (3 dB polarization) + unknown antenna gain | antenna swap, no firmware |
| Pre-detection noise bandwidth 40 MHz vs ~18 MHz | 2.5-3 dB | *model* (below); BW20 hardware A/B failed as a fixed mode | only through a PHY filter state |
| Q4 quantization at the edge | ~1 dB | *model*; Widrow argument in range-max.md | fine lanes recover ~0.8 dB |
| Endpoint discriminator clicks vs MAP estimator | 1.5-2 dB | offline HC bench (range-max.md) | yes, LUT |
| Noise figure (Wi-Fi LNA vs FPV RX) | 0-3 dB | unknown | no (external LNA) |
| Self-desense from DAC/PARLIO | not dominant | pre-q4-lab.md `S` runs (weak evidence) | - |
| Unexplained remainder | **unknown** | needs step 0 | - |

### Model: threshold of the candidate detectors

Fixed receiver noise σ = 0.56 coarse step per axis (measured at maximum gain),
carrier radius varied, synthetic PAL CVBS, 4 MHz per 100 IRE, CFO 150 kHz,
two seeds x 40 lines. SNR = 100 IRE over rms error in 0-5 MHz, active video.
"clk" = output samples per mille with |error| > 40 IRE before filtering.

| C/N (40 MHz) | Live P8, coarse | P8, fine lanes | Endpoint, 10-bit IQ | ±9 MHz filter + adjacent, 10-bit | 2nd-order PLL, 10-bit | ±12 MHz filter + adjacent |
|---|---|---|---|---|---|---|
| 2.9 dB | 6.0 dB / 556 | 7.3 / 529 | 8.0 / 517 | **11.5 / 363** | <0 | 8.6 / 525 |
| 4.9 dB | 9.9 / 454 | 11.3 / 417 | 12.0 / 403 | **15.0 / 255** | 11.8 / 380 | 12.6 / 421 |
| 6.9 dB | 14.0 / 346 | 15.1 / 305 | 15.4 / 287 | **17.2 / 160** | 14.8 / 263 | 15.7 / 319 |
| 8.9 dB | 16.7 / 234 | 17.5 / 192 | 17.9 / 177 | 18.8 / 83 | 16.8 / 156 | 17.8 / 215 |
| 12.9 dB | 20.6 / 57 | 16.1 / 183 (folds) | 21.6 / 32 | 21.1 / 11 | 19.8 / 28 | 20.6 / 57 |
| 17.6 dB | 24.0 / 1.8 | 11.7 / 534 (folds) | 25.1 / 0.6 | 22.5 / 1.1 | 21.9 / 1.3 | 22.3 / 4.2 |

Reading, at a 15 dB output SNR "usable" line:

- live coarse ≈ 7.6 dB C/N, fine lanes ≈ 6.8 dB (**+0.8 dB**), full 10-bit
  IQ ≈ 6.7 dB: quantization costs only ~1 dB at the edge.
- **±9 MHz channel filter ≈ 4.9 dB (+2.7 dB vs live)**, with about half the
  clicks. It costs ~1.5 dB of SNR at strong signal (sideband truncation), which
  matches the hardware BW20 complaint. The filter is only worth it near the
  edge, so it has to be switched rather than fixed.
- ±12 MHz buys almost nothing: the gain needs a filter near Carson bandwidth.
- The PLL does not beat the plain discriminator at this deviation.
- Fine lanes fold above ~radius 3.5 coarse cells. They need an amplitude owner
  that keeps the carrier inside ±256. Native AGC does that (trapped ~135 raw).

Model limits: flat AWGN, no multipath, no AGC transitions, ideal FIR (the
real PHY filter shape is unknown), one picture set. The half-sample reference
offset in the scoring biases all candidates alike; only relative numbers are
used.

## 3. Proposed pipeline

```text
 RHCP 5.8 GHz antenna (U.FL)                                  [hardware, +3-6 dB]
   |
 C5 RF + native AGC, re-acquisition stopped in the PHY        [S2: grain, dashes]
   |  (detect -> acquire -> hold; re-acquire only on a real level change)
   |
 PHY channel filter: BW40 when strong, ~±9 MHz near the edge  [S4: +2-3 dB]
   |
 MODEM_DIAG fine tap I{9,7,6,5} / Q{9,7,6,5}, fixed           [S3: +0.8 dB, -3 dB grain]
   |
 PARLIO RX 40 MS/s ring (unchanged)
   |
 BitScrambler, 2 bundles per pair:
   quadrant-conditioned 8-bit phase decoder  (HC8)            [S5: +1.5-2 dB, fewer clicks]
   -> delta -> saturating DAC transfer at ~2x slope          [S6: -3 dB DAC grain]
   |
 PARLIO TX 20 MS/s [D,D] -> 6-bit DAC
   |
 optional: passive 5.5 MHz LPF + 6.0/6.5 MHz trap            [S7: audio/alias grain]
```

What is deliberately *not* in it: a CPU gain controller, firmware sample
HOLD/replacement/masking, lane switching, span-75 or lower output rates, a
PLL, and any external processor.

### S2. Native AGC: stop the re-acquisition at the source

Problem (measured, native-agc-v2.md): the packet AGC treats the continuous FM
carrier as a stream of aborted packets. Every ~25-50 us it walks the gain for
2.4-3.4 us with saturated samples (the thin black/rainbow dashes), then traps
a *different* gain (the line-to-line texture). About 6-13 % of samples are
acquisition garbage. The operator's paced-native test (1 ms period, 20 us
window) was "een stuk beter", which confirms that fewer re-acquisitions is the
right direction.

Goal: hardware picks the gain; it re-acquires only when the carrier level
really changes. Research ladder, cheapest first:

1. **Identify `DIAG[20..31]`.** `DIAG[0..19]` equal dump-word bits 0..19. The
   dump word carries the gain index in bits 20..27 and the AGC FSM state in
   28..31 (ESPARGOS esp-sdr format, confirmed on the RF dump in
   native-agc-v2.md). `DIAG[20..31]` toggle only with a carrier, which fits.
   Test with the existing `phy_phase_tap_probe` method: route `DIAG[20..27]`
   to the 8 PARLIO pins in a bounded capture and correlate with dump bits
   20..27. If it holds, the native AGC becomes observable **live, per sample,
   without the MAC-dump SRAM crash** from native-agc-v2.md. That is the
   instrument for everything below.
2. **Find what ends each "packet".** With the FSM state visible, record the
   state sequence around an abort. Candidates: SIG/header failure, a packet
   length or timeout, a power-change restart (most Wi-Fi AGCs re-detect on a
   level jump or drop), and the packet-detect / rx-sense / CCA thresholds
   (`7068`, `7010/7014/7044`, `701C`). Earlier sweeps changed one field per
   boot and only looked at the trapped radius. They never looked at *why* the
   FSM left the trapped state.
3. **Detect once, then hold.** native-agc-v2.md found that suppressing
   detection freezes the gain. What was never tried: let one acquisition
   happen, *then* suppress detection, with the power-change restart (if the
   PHY has one) left on. That gives native gain choice, zero switching while
   the level is steady, and hardware re-acquisition on a real fade.
4. **Fallback, operator decision required:** the paced gate from C5VRX-4, but
   with its open window placed in the vertical blanking interval so the walk
   lands on lines the goggle does not show. It keeps hardware gain choice,
   but it is a CPU-timed gate. That conflicts with the standing rule against
   firmware gain workarounds, so it is listed only as a fallback.

Also worth measuring once step 1 works: the rx-comp level fields
(`702C[7:0]`, `70A0[31:24]`) moved the trapped radius from 92 to 156 in the
sweep, but the live check was inconclusive. A trapped radius of ~180-200 would
use the fine window better (S3).

### S3. Fixed fine tap under native AGC

Under native AGC almost every trapped sample has |I|,|Q| < 256, so on the
coarse tap bit 8 equals bit 9 and the live path gets 3 useful bits per axis
(radius ~2.1 cells). The fine tap `{9,7,6,5}` is an exact 2x rescale inside
±256: same angle, same Phase8 LUT, no calibration (range-max.md). It gives
radius ~4.2 cells.

*Model* at the native trapped level (radius 135 raw, strong signal):

| C/N | Tap | Luma rms (IRE) | Chroma-band rms (IRE) |
|---|---|---|---|
| 25 dB | coarse `9..6` | 4.15 | 5.18 |
| 25 dB | fine `9,7,6,5` | 2.79 | 3.82 |
| 35 dB | coarse `9..6` | 4.00 | 4.69 |
| 35 dB | fine `9,7,6,5` | 2.75 | 3.57 |

This agrees with the earlier false-colour table (radius 2.5 -> 3.9 IRE,
radius 4.0 -> 2.4 IRE). Only acquisition samples fold, and those are
saturated garbage on the coarse tap as well. P8 FINE already exists on
`feat/native-agc-v2`. The missing piece is a clean A/B: same channel, native
AGC, P8 FULL vs P8 FINE, at two RF levels. This is the cheapest improvement in
the whole plan.

Do not add automatic lane switching under native AGC. The AGC already
normalizes the level, and switching would add a second actuator.

### S4. Pre-detection filter through the PHY

The BitScrambler cannot filter complex IQ: a two-sample prefilter needs a
16-bit LUT address. The only place a channel filter can go is the PHY
before the tap. The model says the gain is real (+2.7 dB near the edge), but
only with a filter close to Carson bandwidth (±9 MHz), and it costs detail at
a strong signal. BW20 as a *fixed* mode failed on hardware (Hanover bars).
Three points were never controlled in that test:

- **Carrier centring.** A ±9 MHz filter with a 1-2 MHz CFO clips one set of
  FM sidebands, which could be the whole Hanover-bar result. AFC V2
  (burst-confirmed porch reference, branch `feat/native-agc-v2`) can centre
  first.
- **Tap position.** Whether BW20 changes the samples at MODEM_DIAG at all is
  unproven (range-max.md). Measure VTX-off `noise_r2` at BW40 vs BW20 first.
  If the noise does not drop, stop here.
- **Only at the edge.** range-max.md already built an edge-only BW gear.
  Under native AGC the trigger would be "native gain at max and coherence
  falling", read from S2's `DIAG` gain lanes, not from a firmware gain.

Also worth a look: `phy_rx_filter_mode` has more states (0/4/8, coupled to
the ADC rate). Check them only as whole vendor tuples, never as a free
coefficient.

### S5. Demodulator: HC8

The history-conditioned decoder (HC, `main/fm_hc.bsasm`) reached ~40 % fewer
clicks than Phase8 at the edge (~1.5-2 dB) in the offline bench. It is built
on Golden's 5-bit phase, so it is coarser than Phase8 at a strong signal
(luma 3.4 vs 3.0 IRE).

**HC8** combines both: the decoder is addressed by
`(previous-phase quadrant << 8) | raw` (1024 entries) and returns an **8-bit**
MAP phase. The difference runs on Counter A, as in live Phase8. Observations
from `fm_phase8_hr_live.bsasm`: the live program already performs one raw
lookup per pair, and LUT address bits 24..25 are currently fed by low bits of
the retained previous term. So the LUT is effectively replicated 4x, and those
two address bits could carry the previous quadrant instead. Whether that
routing fits alongside the counter load (`set 24..31 O0..O7` / `ldctia`)
needs a dataflow proof. It is plausible, not shown.

Training requirements (lessons from trajectory-v2 and the MMSE rejection):
train on fine-lane geometry at the native radius distribution, include CFO
and acquisition-garbage samples, keep a static fallback for unseen addresses,
and score on an independent picture/seed/CFO set. An MMSE pair table is out:
it learned picture content.

### S6. DAC transfer: use the DAC

Phase8 FULL maps the full ±128-bin range (±20 MHz per 50 ns) onto 64 codes to
avoid a modulo cliff (issues #111/#113). The video needs only about ±3-5 MHz,
so sync-to-white occupies ~17 codes. *Model* (strong signal, saturating
transfer, same phase input):

| Input | Slope (code/bin) | Codes used by video | Luma rms (IRE) |
|---|---|---|---|
| 10-bit IQ | 0.25 (live) | 15 | 1.52 |
| 10-bit IQ | 0.50 | 29 | 1.08 |
| fine lanes, r = 135 | 0.25 (live) | 19 | 2.08 |
| fine lanes, r = 135 | 0.50 | 33 | 1.71 |

Beyond the error numbers, a 2x swing gives the goggle twice the video level,
so its input gain amplifies less of our quantization and noise. This needs a
second lookup per pair, `delta -> DAC`, with saturation instead of wrap. LUT
capacity fits: the HC8 decoder uses the high byte of all 1024 words, and the
DAC map uses the low byte, replicated per bank (the Golden field-sharing
trick). Two lookups in two bundles is exactly Golden's shape. Whether the
counter arithmetic also fits there is the same dataflow question as S5.

Prerequisites: the real VTX deviation and CFO spread, needed to place pedestal
and slope. Measure them from `z` windows or the flywheel's learned levels; do
not use the bench's assumed 40 kHz/IRE. Also check that the resulting level
into 75 ohm is still within spec.

### S7. Optional passive output filter

The output carries FM noise up to 10 MHz (noise power rises with f²), the
RTC6705 audio subcarriers at 6.0/6.5 MHz and Q4 harmonics (native-agc-v2.md).
The 470 pF node is about a 9 MHz pole, not a video filter
(c5vrx-4/RESEARCH.md). A small passive LC low-pass at ~5.5 MHz with a
6.0-6.5 MHz trap is what every analog FPV receiver has. It is a change to the
tested DAC network, so under AGENTS.md it needs an explicit decision and its
own A/B, and it must keep PAL chroma at 4.43 MHz.

## 4. Why not the other candidates

| Candidate | Verdict | Reason |
|---|---|---|
| C5VRX-4 span-75 / Phase6 at 13.33 MS/s | stop | ±6.67 MHz wrap limit, less precision, -1.5 dB at PAL chroma, coarse tap (I3/Q3 under native); operator: less clean than C5VRX-3 |
| PLL / tracking demodulator | drop | *model*: no gain at video deviation; would not fit the BitScrambler anyway |
| Channel filter in the BitScrambler | impossible | needs pair addressing of raw IQ (16-bit address) |
| More IQ lanes than 8 | impossible on C5 | PARLIO RX is 8 lines; a second capture peripheral cannot be combined by the one BitScrambler |
| Ultrafine / folded region-aware decoding | later | native radius spread (80-440) makes folds ambiguous; revisit only after S2 narrows the spread |
| CPU sync flywheel | parked | ~0.5 us per code, covers 1 line in 6 (range-max.md) |
| Direct Gain V5 as gain owner | keep as baseline | works, but it is a firmware gain controller; the operator prefers native |
| Paced native via CPU timer | fallback only | see S2 step 4 |

## 5. Expected result

| Stage | Range (threshold) | Grain at strong signal | Confidence |
|---|---|---|---|
| RHCP antenna | +3-6 dB | slightly (higher C/N) | high |
| S2 stop re-acquisition | ~0 (unknown near edge) | removes the dashes and line texture | medium (paced test) |
| S3 fine tap | +0.8 dB | luma error -3 dB | high (model + exact rescale) |
| S4 ±9 MHz edge filter | +2-3 dB | none (off when strong) | low-medium (earlier hardware failure) |
| S5 HC8 | +1.5-2 dB | neutral vs Phase8 (must be shown) | medium (offline only) |
| S6 2x DAC slope | ~0 | luma error -1.7 dB (fine lanes), 2x level | medium (depends on real deviation) |
| S7 LC filter | ~0 | less 6-10 MHz noise, no audio pattern | medium |

Chip-only: about +4-5 dB, roughly 1.6-1.8x distance. With the antenna,
roughly 2.2-3x. If step 0 shows a much larger gap to the reference receiver,
the remainder is a defect to find (antenna/cable, AGC holding the RF stage
low, CFO) before any of the above.

## 6. Relation to earlier work

- Builds on: native-agc-v2.md (mechanism, register candidates, P8 FINE), range-max.md
  (fine-lane exactness, BW gear, HC), c5vrx-4 (paced-native result),
  continuous-iq-findings.md (DIAG mapping).
- Corrects an implicit assumption in C5VRX-4 and live native mode: the
  coarse tap is not "4-bit" under native AGC.
- Does not revive BW20 as a free +3 dB: it asks for the three controls the
  earlier test lacked.

## 7. Experiment plan (in order, each with a stop rule)

0. **Sensitivity baseline.** VTX -> SMA attenuators (10/20/30 dB + 1 dB steps)
   -> splitter or swap -> C5VRX and the goggle's own receiver. Record the
   attenuation at "usable picture" and at "sync lost" for both, 3 repeats,
   same channel. Output: the dB gap. Everything else is compared against it.
1. **Antenna A/B** at step 0's threshold: stock antenna vs 5.8 GHz RHCP.
2. **`DIAG[20..31]` identity** (bounded PARLIO capture + dump correlation).
   Stop rule: no match after the full bit-alignment search -> S2 stays blind
   and falls back to register A/B by picture.
3. **P8 FULL vs P8 FINE under native AGC**, two RF levels, operator rating plus
   `E` rows. Stop rule: FINE visibly worse -> check the fold rate.
4. **Re-acquisition root cause** with the step-2 instrument: abort reason,
   detect-once-then-hold trial. Stop rule: no register holds the trapped state
   -> operator decides on the VBI-aligned paced fallback.
5. **BW20 at the tap:** VTX-off `noise_r2` BW40 vs BW20. Stop if unchanged.
   Otherwise centre with AFC V2 and repeat step 0 at BW20 vs BW40.
6. **HC8 and 2x DAC transfer:** offline bench first (independent sets,
   fine-lane native statistics, real measured deviation), then dataflow proof
   against `fm_phase8_hr_live.bsasm`, then step 0 again.
7. **LC output filter**, only if the operator accepts a DAC-network change.

Success criterion throughout (from c5vrx-4/README.md): more controlled RF
attenuation at equal picture quality and fade recovery. A cleaner picture at
the same attenuation is a quality result, not a range result.
