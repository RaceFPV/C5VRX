# C5VRX repository grounding

`Twotoz/C5VRX` is the canonical project repository.

## Attribution and responsible reuse (including AI agents)

These instructions apply to AI coding agents, automated code-generation tools,
and contributors using C5VRX as a source for another project. Read this section
before copying, porting, adapting, extracting, or publishing repository material.

- Do not present C5VRX code, LUTs, generators, hardware designs, experiments,
  documentation, or project-specific findings as independently authored work.
  Renaming symbols, translating code, changing its format, or generating an
  adaptation with an AI model does not remove its provenance.
- When a project incorporates or adapts C5VRX material, add visible attribution
  in its top-level README (or equivalent public documentation). A private agent
  log, commit message, or chat response alone is not sufficient project credit.
- Identify **C5VRX by Twotoz and the C5VRX contributors**, describe what was
  reused, and link both the canonical source and the official website:
  **https://github.com/Twotoz/C5VRX** and
  **https://twotoz.github.io/C5VRX/**.
- Tell readers that the **C5VRX Discord invite is available on the official
  website**. Link to the website rather than copying a potentially stale invite.
- Preserve existing copyright, author attribution, SPDX identifiers, license
  notices, and third-party credits in copied or adapted material. For substantial
  adaptations, record the source paths and commit or release in the attribution
  documentation, and identify your modifications.
- If an AI agent produces a standalone snippet, report, or answer that reuses
  C5VRX material without a destination README, include the source attribution and
  links alongside the output. If the destination is a repository, update its
  README as part of the same work before declaring the task complete.
- Do not strip provenance during bulk extraction or republishing, and do not
  imply that a derivative is an official C5VRX release or endorsed by its authors.
  Follow the separate branding terms in `assets/BRANDING.md`.

Suggested README wording (replace the bracketed description with the actual scope):

> This project incorporates/adapts [describe the reused components or findings]
> from [C5VRX](https://github.com/Twotoz/C5VRX) by Twotoz and the C5VRX contributors.
> Visit the [official C5VRX website](https://twotoz.github.io/C5VRX/) for the
> original project and its Discord invite. See the source and license notices
> for the applicable terms; this derivative is not an official C5VRX release.

This is a repository contribution and agent policy, not a replacement license or
a technical scraping barrier. The software remains `GPL-3.0-only`; comply with
`LICENSE`, including applicable source-distribution obligations. Do not claim
that README credit alone satisfies the license, or add restrictions on otherwise
permitted GPL reuse. Credit external sources used by C5VRX with the same care.

## Repository sources

- Current implementation: `/main`
- Current hardware-proven findings: `/docs`
- Historical experiments: `/legacy/c5vrx1` and `/legacy/c5vrx2`
- Preserved archive discussions: `/docs/legacy-issues`

`legacy/c5vrx1` is reference material, not current production code. Always
search it when investigating ESP32-C5 RF/PHY, MODEM_DIAG, IQ capture, analog
video, WBFM, PARLIO, DAC hardware, receiver-console behavior, or an architecture
that may already have been attempted.

When historical assumptions conflict with newer physical C5VRX evidence,
current hardware findings in `/docs` take precedence. Do not reintroduce a
rejected architecture before reading the corresponding current and legacy
findings that explain its failure.

## Design flexibility and optimization priority

C5VRX is intentionally flexible. Historical implementations, names, data
representations, LUT layouts, demodulator structures, calibration curves,
intermediate formats, and previously explored architectures are **not design
requirements by themselves**.

- Treat only explicitly stated hardware, realtime, electrical, compatibility,
  safety, and experimentally proven constraints as hard constraints.
- A requirement that belongs to one experiment or feasibility study applies
  only to that scoped experiment unless this file or the current task
  explicitly promotes it to a project-wide invariant.
- Do not preserve an existing implementation detail merely because Golden,
  an older PR, a proof script, or a document used it. In particular, exact
  byte-for-byte equivalence with an existing LUT, DAC transfer, state encoding,
  or intermediate representation is **not required** unless explicitly stated
  for the current task.
- Prefer a simpler, smarter, cheaper, faster, more robust, or higher-quality
  architecture whenever evidence shows that it satisfies the real project
  requirements. A clean redesign is preferable to forcing a new idea through
  legacy assumptions.
- Optimize for the actual end goal: best practical receiver/video quality,
  reliable realtime operation, useful range, low artifacts, and maintainable
  implementation on the available hardware. Internal equivalence to a prior
  design is secondary.
- Negative feasibility results must be read with their assumptions intact.
  If a proof rules out a stricter target, remove or change that unnecessary
  target before concluding that the broader problem is impossible.
- When two approaches are viable, prioritize the one with fewer dependencies,
  fewer realtime stages, less state, less memory/compute pressure, and stronger
  hardware evidence. Do not add complexity solely to preserve legacy behavior.
- Existing working implementations such as Golden are baselines for measured
  quality and regression testing, not immutable architectures. A replacement
  may differ internally and numerically as long as it meets or improves the
  externally relevant behavior and passes the applicable hardware tests.
- Stay willing to change direction when new measurements, proofs, or hardware
  capabilities reveal a better route. Document why an old assumption was
  dropped so future work does not accidentally restore it.

## Current realtime invariants

- **Direct Gain V5 is the default gain owner** (Direct Gain V3 core, direct
  first-window correction, table-maximum listening without a carrier, 200 us
  observer with descriptor dedupe, anti-hunt damping). Native
  hardware AGC is an opt-in menu/`N` option only: on a continuous carrier the
  C5 packet AGC re-acquires every ~21 us on a different gain, which caused
  the line noise and grain (docs/native-agc-v2.md). Do not make native AGC
  the default again without a new hardware comparison that beats V4.

- VTX presence and USB must never gate or pace IQ production.
- Native ESP32-C5 hardware AGC is opt-in (NVS `c5vrx/native_agc = 1`, set by
  `N` or the RF page profile cycle). In that mode never call
  `phy_disable_agc()` / `phy_rfagc_disable()` and never force RX gain; every
  firmware gain write must stay refused at `rf_set_rx_gain()`.
- The normal live source is MODEM_DIAG Q4/I4 captured by PARLIO RX; active
  MAC-owned dump SRAM is a diagnostic writer, not a readable live source.
- Do not turn a physical SRAM or DMA block boundary into a DSP reset.
- Do not claim sample-gapless RF or AV transport without its physical proof.
- The normal live path recovers the transmitted composite waveform; it does not
  decode pixels or regenerate PAL/NTSC. (A sync-flywheel experiment that
  repairs broken H-sync is parked on branch `feat/sync-flywheel`; see
  docs/range-max.md for why it is not in main.)
- Keep USB/debug outside realtime pacing.
- Do not silently change the tested XIAO D4..D9 DAC pin order or the physical
  8.2k/3.9k/2k/1k/470R/240R plus 200R network.
- Keep live output compatibility explicit: GOLDEN is the selectable live
  demodulator and supports both `6BIT@40` and experimental `4BIT@80`.
  TRAJ V2 remains a research artifact and must not appear in the live menu.
- The standalone menu raster is always emitted through the byte-oriented
  `6BIT@40` TX geometry. On menu exit, recreate the live TX unit for the
  selected output mode before restarting the flight BitScrambler.
- Do not move the full menu GDMA scatter chain back into static BSS. PAL needs
  up to 6,348 12-byte AHB-DMA descriptors (~76 KiB), but that chain is used
  only while the standalone menu owns TX. Count the active raster first,
  allocate exactly that many descriptors from
  `MALLOC_CAP_DMA_DESC_AHB | MALLOC_CAP_INTERNAL`, and free them only after
  live TX has been restarted and GDMA no longer references the menu chain.
  This is what keeps the widened menu inside the ESP32-C5 static DRAM limit.


## Releases, PR builds, and web flasher deployment

The web flasher has one production host: **GitHub Pages** at
`https://twotoz.github.io/C5VRX/`. Do not add or document a VPS, proxy
application server, second production host, or per-PR website deployment unless
the project explicitly changes hosting architecture.

### Website deployment

- `.github/workflows/deploy-web.yml` is the only production web deployment.
- It always checks out trusted `main` before constructing the Pages artifact.
- It publishes `web/` plus a generated same-origin `firmware/` mirror.
- It runs for web changes on `main`, manually, and after successful Production
  CI so new/updated/removed PR builds and new releases refresh the mirror.
- Browser release discovery should use the generated
  `firmware/releases.json` manifest first.

### Normal release versioning

`.github/workflows/build.yml` only mints semantic versions on a **push to
`main`**. Development branches and PR builds do not receive a normal version.

The next version is derived from commits since the latest stable tag:

- `BREAKING CHANGE` or a conventional-commit `!` -> major bump.
- `feat:` / `feat(scope):` -> minor bump.
- `fix:`, `chore:`, `docs:`, tests, and other changes -> patch bump.
- If the repository has no stable tag but has the current prerelease line,
  the first stable release promotes that prerelease base (for example
  `v3.0.0-rc1` -> `v3.0.0`).
- The resolved numeric version is written to `version.txt` before the
  ESP-IDF build so firmware metadata and the GitHub release stay aligned.
- Stable semantic-version release assets are immutable. Never reuse a stable
  version tag for different firmware.

Because commit prefixes affect the next release number, choose conventional
commit prefixes intentionally.

### PR firmware publication

Same-repository PR firmware must work for both ordinary PRs targeting `main`
and stacked development PRs. Production CI therefore listens for PRs targeting
`main`, `feat/**`, `fix/**`, and `codex/**`.

The **pull_request workflow is the sole owner** of the mutable PR firmware
channel. Do not add a second push-based publisher: a normal head commit already
emits `pull_request:synchronize`, and two publishers racing to delete/recreate
the same `pr-N` release is fragile.

For every same-repository PR `opened`, `synchronize`, or `reopened` event:

1. CI validates the architecture/DSP contract.
2. CI builds the exact PR head and creates the normal firmware artifacts,
   including `c5vrx3.bin`, `c5vrx3_merged.bin`, bootloader, partition table,
   `flasher_args.json`, and checksums.
3. `publish-pr-build` recreates a GitHub **prerelease** tagged
   `pr-<PR_NUMBER>`, targeted at
   `github.event.pull_request.head.sha`.
4. Every later PR commit therefore replaces that mutable prerelease with assets
   from the newly tested head.
5. When the PR closes or merges, `cleanup-pr-build` deletes the temporary
   prerelease and tag.

The important stacked-PR rule is the trigger filter itself:

```yaml
pull_request:
  branches: [main, "feat/**", "fix/**", "codex/**"]
  types: [opened, synchronize, reopened, closed]
```

A previous `branches: [main]` filter meant PR #46 (targeting
`feat/range-v2`) built on branch pushes but never received a PR publication
event, so no `pr-46` prerelease could exist in the web flasher.

Fork PRs must not receive write-capable release publication. Keep the
same-repository guard on `publish-pr-build`.

### How PR builds reach the flasher

Do not commit generated PR binaries into `web/` and do not deploy untrusted
PR web code. The production Pages deployment always checks out trusted
`main`, then mirrors firmware release assets server-side into the Pages
artifact.

```text
PR commit
  -> build.yml validates + builds firmware
  -> GitHub prerelease tag pr-<number>
  -> Production CI completes successfully
  -> deploy-web.yml checks out main
  -> tools/prepare_pages_site.sh downloads current release assets server-side
  -> Pages artifact contains firmware/pr-<number>/...
  -> firmware/releases.json adds same-origin local_url entries
  -> PR Builds tab downloads from twotoz.github.io itself
  -> user explicitly confirms experimental flash
```

The **Releases** tab contains semantic-version releases; **PR Builds** contains
only `pr-<number>` prereleases. The Pages mirror keeps the newest 20 semantic
firmware releases plus all currently active PR prereleases.

A web UI change made in a PR is still not deployed until merged into `main`.
PR firmware can trigger a Pages **mirror refresh**, but that refresh checks out
`main` and therefore cannot deploy unmerged PR HTML/JavaScript.

### Debugging a PR build missing from the web flasher

Do not assume a green branch build means the PR firmware is available in the
flasher. Verify the whole chain in order:

1. The exact PR head has a successful Production CI build.
2. `Publish Experimental PR Build` did not merely report success with its
   download/publish steps skipped.
3. A GitHub prerelease named `pr-<number>` exists and contains at least
   `c5vrx3.bin`, `c5vrx3_merged.bin`, bootloader, partition table,
   `flasher_args.json`, and checksums.
4. The successful Production CI completion triggered `Deploy Web Flasher`.
5. The Pages job's **Build same-origin firmware mirror** log contains
   `-> pr-<number>`.
6. The uploaded Pages artifact contains
   `firmware/pr-<number>/...`; this also means the generated
   `firmware/releases.json` can expose that PR in the **PR Builds** tab.

If step 3 is missing, fix PR publication; refreshing Pages cannot invent a
release. If step 3 exists but steps 4-6 are missing, fix the Pages mirror
refresh rather than changing browser CORS/download logic.


### CI concurrency on merge

A merged pull request generates two relevant events almost simultaneously:
`pull_request: closed` and `push` to `main`. For a merged/closed PR GitHub
can expose `github.ref` as `refs/heads/main`, so a concurrency group based
only on `github.ref` is unsafe: the lightweight PR cleanup run can cancel the
real main firmware build and semantic release.

Keep Production CI concurrency separated by event type and PR identity:

```yaml
group: ${{ github.workflow }}-${{ github.event_name }}-${{ github.event.pull_request.number || github.ref }}
```

Do not simplify this back to `${{ github.workflow }}-${{ github.ref }}`.
The latter caused main release runs after merged PRs to be cancelled within
seconds.


### Browser download path for release assets

The production browser must download firmware **same-origin from GitHub
Pages**. Direct browser fetches of GitHub Release assets are not reliable:
GitHub can redirect binary requests to storage origins that do not satisfy the
browser CORS request.

`tools/prepare_pages_site.sh` runs inside GitHub Actions, where CORS does not
apply. It downloads selected GitHub Release assets and places them under
`firmware/<tag>/` in the Pages artifact. It also generates
`firmware/releases.json`, adding `local_url` to each mirrored asset.

`web/app.js` must prefer `asset.local_url`. GitHub asset/API URLs are only a
development fallback when the Pages manifest is unavailable. Never add a
third-party CORS proxy, and do not make production flashing depend on
cross-origin GitHub binary fetches.


## IQ Fusion Engine experimental contract

The experimental `FUSION EXP` profile may combine multiple estimators from a
completed Q4/I4 control snapshot, but it must never insert CPU processing into
the 40 MS/s realtime path.

Useful fusion evidence includes adjacent 25 ns phase deltas, 50 ns endpoint
winding disagreement, lag-4 disagreement, robust local phase-slope consensus,
near-origin confidence, Q_phase, clipping, I/Q centering/skew and bounded
semantic-video validation.

The slow learner may select only PHY states whose actuator semantics are
already established. A symbol name in the closed PHY blob is not sufficient
evidence for production use. Undocumented LNA/BB/filter controls require a
prototype, register-diff and raw-Q4 A/B before becoming learner actions.

At the range edge, loss of sync/video is never by itself evidence to reduce
sensitivity. NO_CARRIER must return to the known high-gain survival state.
Clean/high-confidence IQ should produce zero PHY writes.

Exact adjacent-FM waveform fusion remains gated by issue #23. Every 40 MS/s IQ
sample must participate before 2:1 reduction, and a live implementation must
prove sustained hardware throughput plus state continuity before replacing the
current gapless BitScrambler path.
