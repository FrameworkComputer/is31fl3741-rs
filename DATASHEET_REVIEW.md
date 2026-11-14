# IS31FL3741A datasheet review

Compared against IS31FL3741A datasheet Rev. D, November 6, 2024, on November 13, 2026.
Local source: `~/Documents/datasheets/IS31FL3741A_DS.pdf`.

## Current state — checked November 14, 2026

Reconciled against implementation commit `8028a9b`. Correctness fixes, register
APIs, compatibility-preserving documentation, static mapping storage, and
regression tooling are implemented. The de-ghost API is complete; its optical
efficacy is a separate unchecked investigation below.

Latest recorded checks: 26 all-feature tests in debug/release, 22 core-only,
23 Framework-only, 24 Adafruit-only, and 25 Adafruit+graphics tests passed.
Strict host/diagnostic Clippy, docs, and formatting passed. These are the prior
recorded results, not newly executed tests on November 14. Counts elsewhere
record the suite size at that particular change.

Last recorded hardware state: normal updated `ledtest` restored and application
serial enumeration confirmed; camera controls restored. Device presence/state
has not been rechecked today. Remaining contract choices are listed at the end.

This started as a static code and datasheet review; selected findings now have host and Framework hardware validation. Original findings below describe the reviewed code, not necessarily the current implementation.

Check an item when it has been fixed or explicitly resolved and its relevant checks are complete. Unchecked items may be partially implemented: consult their **Status** notes. **Automated**, **Hardware**, and **Review/build** paragraphs describe planned checks, not claims that every listed check ran. Actual hardware evidence is in [HARDWARE_TEST_RESULTS.md](examples/ledmatrix/HARDWARE_TEST_RESULTS.md), initially committed in `9dfce47` and extended by subsequent hardware-validation commits.

Update this checklist as each item progresses, in the same commit as the change or its validation evidence. Record actual baseline/fixed outcomes and remaining limitations; never infer a hardware pass from compilation or mock tests.

## Implementation plan

The user authorized reset/bootloader entry and flashing for the supplied Framework
LED Matrix; re-identify it before every hardware session. Hardware/tool setup is
also documented in `~/clone/inputmodule-rs` and the checked-in Framework README.

For each issue: establish the unmodified baseline, implement narrowly, test the
fix with the same host/hardware assertions, restore state, update this checklist,
and commit separately with only actual validation evidence. Follow the detailed
workflow below; do not treat compilation as a hardware behavioral baseline.

## Repeatable issue workflow

This is the handoff procedure for continuing with the same rigor. Runbook
commands below are instructions, not additional claims of tests already run.
The results beside each checklist item and in the hardware records are the
actual evidence. Do not depend on session-local `/tmp` scripts; those are not
part of the retained harness.

### 1. Define the issue and preserve the baseline

- Work on one issue at a time. Identify the datasheet page/table and concrete
  expected behavior, current implementation, affected public contract, and
  applicable host/hardware checks before editing driver code.
- Inspect `git status` and preserve unrelated changes. Record the baseline
  commit, any uncommitted differences, compiler version, profile, and relevant
  dependency lockfile. Keep baseline and fixed builds in separate directories.
- Add the focused regression test/diagnostic first. Run it against the unmodified
  implementation on the host **and on the Framework hardware where applicable**.
  Save the exact command, output, and exit/result. A failure must be the expected
  behavioral failure, not an unrelated compiler/linker/USB error.
- Use the same test, diagnostic source, build profile, inputs, bus/clock settings,
  and board for before/after comparisons. When historical examples lack USB test
  dependencies, copy only the harness and necessary example manifest/lockfile;
  record that adaptation. Do not backport the production fix into the baseline.
- For already-committed fixes, replay the parent and fixed revisions in isolated
  checkouts and label the evidence as a later replay. Never imply that later
  hardware validation happened before the original commit.

Example: reproduce the invalid-page host regression from before `8028a9b`.
Run from the repository root; choose a fresh baseline directory:

```sh
baseline_dir=$(mktemp -d /tmp/is31-before-invalid-page.XXXXXX)
git worktree add --detach "$baseline_dir" f5f6f49
cp tests/core.rs "$baseline_dir/tests/core.rs"
cp tests/support/mod.rs "$baseline_dir/tests/support/mod.rs"
(cd "$baseline_dir" && cargo test --all-features --test core \
  invalid_mapping_pages_are_rejected_without_bus_access -- --exact)
```

Expected baseline: the test fails because page2 is accepted. On the fixed driver
it passes. For a future issue, use its own parent/test and copy any required
support files; check that a failure really corresponds to that issue. Retain
logs before removing a baseline checkout. Never reset the working checkout to
obtain the old implementation.

Different issue types require different baseline evidence:

| Type | Valid baseline and follow-up |
| --- | --- |
| Behavior bug | Expected host/hardware failure before, same assertion passes after. |
| Missing API | Record absence separately. A missing-method compile failure is not an on-device runtime baseline. Exercise actual register/event behavior after implementation; explicitly identify any hardware baseline that could not be established. Do not invent a no-op “old implementation.” |
| Optimization | Correctness may pass both versions. Measure the original cost, repeat an identical workload afterward, and preserve mappings/output/error behavior. Do not manufacture a correctness failure. |
| Documentation | Inspect the inaccurate/missing wording first, cross-check source/datasheet, then review/build the correction. Do not invent a runtime or hardware failure for wording changes. |
| Test/tool improvement | Run the previous suite/tool first. Demonstrate a real coverage/acceptance/recovery gap if one exists; distinguish added coverage from a fixed driver defect. |

### 2. Implement narrowly and retain regression coverage

- Make the smallest change that satisfies the expected behavior. Preserve existing
  public interfaces/defaults unless a decision has been made explicitly. Leave
  open contract/ownership questions unchecked rather than silently choosing a
  new design while the user is away.
- Put public-behavior host tests in `tests/core.rs`, Adafruit/graphics checks in
  `tests/adafruit.rs`, and mapping compatibility in `tests/mapping.rs`. Reuse
  `tests/support/mod.rs` for recorded writes, read values, and transaction failures.
- Check boundaries, both register pages and their final channels, reserved-bit
  masks, preservation of unrelated fields, and error propagation. For multi-step
  I²C operations, inject a failure at **each** transaction and check that no
  unintended subsequent writes occur. Check invalid inputs cause no bus access.
- Preserve independent expected results. Frozen mapping CSVs establish compatibility
  with the reviewed mapping, not correctness against an unseen PCB. Do not update
  a fixture solely to make a changed implementation pass.
- Extend `examples/ledmatrix/examples/hardware_check.rs` with a focused command
  when a hardware regression needs it. Report each API separately; never hide a
  failing assertion behind a generic combined PASS. Label exploratory measurements
  `OBS`, not PASS, until there is a defined acceptance condition.

### 3. Validate the fix on the host and target

Run the focused test first, then the relevant shared checks. Current full host
matrix (repository root):

```sh
cargo test --all-features
cargo test --all-features --release
cargo test --no-default-features
cargo test --no-default-features --features framework_ledmatrix
cargo test --no-default-features --features adafruit_rgb_13x9
cargo test --no-default-features --features adafruit_rgb_13x9,embedded_graphics
cargo clippy --all-features --all-targets -- -D warnings
cargo clippy --no-default-features --all-targets -- -D warnings
cargo doc --all-features --no-deps
cargo fmt --all --check
git diff --check
cargo build --target thumbv6m-none-eabi --all-features
```

For diagnostic/firmware changes, from `examples/ledmatrix`:

```sh
cargo clippy --release --example hardware_check -- -D warnings
cargo build --release --example hardware_check && \
  elf2uf2-rs target/thumbv6m-none-eabi/release/examples/hardware_check hardware_check.uf2
```

Keep build and conversion dependent (`&&`): a failed build must not lead to
flashing an older binary left in `target/`. Use the same profile before/after;
release is required for performance comparisons. Do not call a successful
cross-build proof that firmware ran. The historical Adafruit example build had
linker warnings about missing `_start`; no Adafruit runtime validation is claimed.
See [host test instructions](tests/README.md) and
[mapping measurement methodology](measurements/README.md).

### 4. Run controlled hardware checks and restore state

Re-identify the intended board and serial device before every flash. The recorded
application is USB `32ac:0020`; bootloader is `2e8a:0003` with volume `RPI-RP2`.
Do not assume `/dev/ttyACM0` still belongs to the matrix. Inspect `/dev/serial/by-id`
and USB enumeration; record board revision/IC marking if actually inspected.
Otherwise retain “unknown/uninspected.” Follow the
[Framework build/bootloader/flash instructions](examples/ledmatrix/README.md).
If desktop automount is inactive, identify and mount the **RPI-RP2** volume
explicitly (for example `udisksctl mount -b /dev/disk/by-label/RPI-RP2`); never
substitute an arbitrary block device. Wait for application re-enumeration and
serial permissions to settle after flashing. Record mount/serial failures as
infrastructure failures, not test outcomes.

The diagnostic accepts single ASCII commands at 115200 baud:

| Command | Report/effect |
| --- | --- |
| `F` | One PASS/FAIL: complete 351-channel PWM write/readback. |
| `C` | One PASS/FAIL: coordinates/corners, skipping the frame check so an old frame bug cannot mask a bounds bug. |
| `T`, `S` | One PASS/FAIL: frame, bounds, shutdown/resume sequence. |
| `A` | Five independent lines: GCC, resistors, logic, IRQ mask ACK, IRQ status access. The last two alone do not validate events. |
| `G`, `R`, `L`, `M`, `I` | One API check from `A`. |
| `X` | Expected deliberate failure with verified restoration; then repeat `A`/`T` without resetting. |
| `U` | Setup zero-PWM/scaling contract, explicit scaling, and verified restoration. |
| `V` | Invalid mapping pages2/255 rejected, PWM unchanged, callback/state restored. |
| `P` | Release mapping-only timer and checksum; excludes I²C/USB. |
| `E` | Four observation lines: masked open, latched open, auto-clear open, latched short. |
| `Q` | Three drive-level controls for open detection. |
| `0`–`6` | Hold optical patterns; **not** automatically restored after a successful command. |
| `Z` | Restore the state saved before the optical sequence. |

For a manual diagnostic query on Linux, this recipe needs only Python's standard
library. Set `MATRIX_PORT` to the verified device; `A` expects five complete lines.
Change the command/count for other rows of the table and save the full output:

```sh
MATRIX_PORT=/dev/ttyACM0  # example only: verify the actual device first
python3 - "$MATRIX_PORT" A 5 <<'PY'
import os, select, sys, termios, time
fd = os.open(sys.argv[1], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
original = termios.tcgetattr(fd)
try:
    settings = termios.tcgetattr(fd)
    settings[0] = settings[1] = settings[3] = 0
    settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    settings[4] = settings[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, settings)
    termios.tcflush(fd, termios.TCIFLUSH)
    os.write(fd, sys.argv[2].encode('ascii'))
    data = b''
    deadline = time.monotonic() + 10
    while data.count(b'\n') < int(sys.argv[3]) and time.monotonic() < deadline:
        if select.select([fd], [], [], 0.1)[0]:
            data += os.read(fd, 4096)
    print(data.decode(), end='')
    if data.count(b'\n') != int(sys.argv[3]):
        raise SystemExit('Missing/extra diagnostic report lines')
finally:
    termios.tcsetattr(fd, termios.TCSANOW, original)
    os.close(fd)
PY
```

This reader checks completeness, **not correctness**. Compare each report with
the expected result; `X` intentionally says FAIL, other unexpected FAIL lines
must not be accepted. Use the retained interrupt checker for event assertions.
Repeat focused fixed checks at least three consecutive times, and run relevant
normal checks afterward (for example `V`, `T`, `A`, `U`). An initial failure can
contaminate subsequent checks: preserve the first failure and state effects,
restore/reinitialize, and do not describe later initial-state failures as three
independent reproductions of the same assertion.

Diagnostics must snapshot the state they change and restore on success **and**
failure, then verify readback. Current snapshots cover PWM, scaling, configuration,
GCC, resistor selection, and PWM frequency. Restore a changed callback too.
Attempt remaining cleanup operations if one fails; report uncertain state if
writes or verification fail. The write-only interrupt mask is owned by the
diagnostic and returned to disabled, not read back or restored from an arbitrary
prior value. Fault/status history is read-only and cannot be restored: I (including A)
can acknowledge status, E/Q generate/acknowledge events and replace results,
and U resets history. Keep these exceptions in reports/docs.
Physical bus-disconnection recovery has not been demonstrated.

At the end, send `Z` if optical modes were used, build/convert/flash normal
`ledtest` with the intended final driver, and verify application re-enumeration.
Restore camera settings and read them back. Do not record “restored” while the
flash/mount/control operation is merely pending or has timed out.

### 5. Use evidence appropriate to the claim

- **Register behavior:** read back the complete affected range, including page
  boundaries/unmapped channels, and compare independent expected data. A visible
  pattern alone cannot catch PG1 `0xAA`, which is unused on the Framework board.
- **Interrupts/detection:** use `scripts/check_interrupts.py` from the Framework
  example directory with a fresh output JSON. E/Q observations are specific to
  this board/settings. Full PWM/scaling at GCC1 produced six active opens;
  SW9 is disabled. A status read acknowledged flags/released INTB. Auto-clear was
  sampled low at 1ms/high by 12ms, not measured as an exact 8ms interval. The
  256-channel short bitmap is unresolved, not evidence of 256 physical shorts.
  Do not use an empty bitmap as a passing event test or infer guaranteed completion.
- **Optical behavior:** use `scripts/capture_optical.py`; verify framing, start
  with dark mode0, lock exposure/white balance, keep pattern/settings constant,
  repeat resistor conditions, retain cropped frames and clipping metrics, and
  restore camera/matrix in cleanup. The visibility guard must reject a lit pattern
  missing from the crop. RGB means are not calibrated luminance; clipped cores,
  bloom, and reflections limit conclusions. No conclusive difference is not
  evidence that ghosting was eliminated. See both retained optical comparisons.
- **Performance:** use `measurements/measure_mapping.py` and release `P` with the
  same toolchain/workload. Check checksums/frozen mappings before accepting gains.
  Compare linked firmware size too: probe objects omit external static tables.
  Distinguish local stack reservation from whole-program stack and mapping-only
  timing from end-to-end I²C display throughput.
- **Coverage limits:** Framework hardware does not validate Adafruit wiring/colors.
  Digital configuration readback does not measure analog logic thresholds. Do
  not introduce physical shorts or remove LEDs on the working board for a test.

### 6. Record, update, then commit one issue

Keep source tests/diagnostics and concise results in version control. Store host
regressions in `tests/`, measurements under `measurements/`, and dated hardware
records/cropped evidence under `examples/ledmatrix/test-results/`. Append actual
runs to `examples/ledmatrix/HARDWARE_TEST_RESULTS.md`; keep prior records intact.
Generated binaries, raw full-scene camera frames, build caches, and temporary
logs stay untracked. Never overwrite a baseline report with the fixed output.

Use this evidence template for each item/run:

```text
Issue / datasheet reference:
Baseline driver commit + uncommitted changes:
Fixed driver commit or precise working-tree diff:
Harness commit + any copied manifest/lockfile adaptations and lockfile hash:
Built/flashed firmware path and SHA-256 (for example sha256sum hardware_check.uf2):
Toolchain/profile / board identity (unknown if uninspected):
I2C speed / scan rows / PWM / GCC / scaling / resistor settings:
Exact baseline command, observed result, exit code or serial report:
Exact fixed command, observed result, repeat count:
Broader checks actually executed:
Raw/cropped evidence paths:
Cleanup attempted and restoration actually verified:
Remaining limits / decisions:
```

Update the issue's checkbox and Status in this file **in the same commit** as the
fix/evidence. Cite commits and results; distinguish implementation completion
from separate efficacy/physical-validation questions. Keep incomplete work
unchecked with an explicit reason. Update the current-state summary when the
implementation/test totals change, keeping historical counts attached to their
original steps. Do not claim an actual CI run when only workflow commands ran locally.

Commit the fix together with its persistent regression and checklist/evidence
(one issue per commit; harness-only changes can be their own passing commit).
Do not commit a knowingly failing regression alone to the main branch. Suggested
commit body structure, filled with actual outcomes only:

```text
Baseline: <revision, command, actual expected failure or measured cost>.
After: <command, actual result and hardware repeat count>.
Checks: <only checks actually executed>.
Hardware/evidence: <board/settings/report path; actual limits>.
Cleanup: <what was restored and how verified>.
```

If baseline evidence or hardware verification is unavailable, say so plainly in
the checklist and commit; never fill this template with anticipated results.
Retest after further code changes invalidate earlier results. Once relevant
checks pass, do not repeat unrelated tests without a new change or unresolved
concern. Finish with `git status --short` and report any remaining changes.

## Test approach

Use a recording/scripted `embedded_hal::i2c::I2c` mock for register transactions, decoding, boundary cases, and error propagation. These tests do not require attached hardware. Add regression tests with each substantive fix/API rather than tests that only repeat constants.

Use the existing `examples/ledmatrix` RP2040 firmware as the starting point for a hardware diagnostic. It already selects `Sw1Sw8`, matching the Framework mapping. The persistent `hardware_check` example reports test results over USB serial and uses diagnostic-local I²C reads; driver register helpers remain private. Any diagnostic page selection must use the unlock sequence and restore state or reinitialize between cases.

Before flashing, identify the attached board and firmware recovery path, record its chip marking/revision against this **A** datasheet if accessible (otherwise explicitly uninspected), and choose modest scaling/GCC for visible tests. Firmware flashing and register tests have been performed; physical fault injection has not. Physical chip marking and board revision remain uninspected. Register readback and optical observation complement mock tests; they do not replace them. Framework hardware does not validate Adafruit-specific layout behavior.

Record firmware revision, board revision, settings, expected/observed values, and pass/fail for each hardware run. Leave items unchecked until their implementation/documentation resolution and relevant checks are complete; explicitly record hardware checks that remain unavailable.

## Straightforward work

These have a clear expected behavior and can be implemented independently without choosing a new architecture. “Straightforward” describes the decision involved; substantive code changes still need the tests listed below. Items remain unchecked until resolved and verified.

### Correctness fixes

Small, focused changes with existing APIs and clear regression cases.

- [x] **Fix the PG1 transfer length in `fill_matrix()`.**
  **Status:** Fixed in `3e73edd`. Host patterned-frame regression passed. Hardware baseline `21c5c1a` failed full PWM readback; fixed `9e7cae9` passed three times. See `9dfce47` evidence.

  **Original finding:** In `src/lib.rs`, the second transfer used `&buf[..=0xAA]`: 171 bytes including the register address, so only 170 PWM values are sent. PG1 has 171 registers (`0x00–0xAA`), leaving the final channel unchanged. Use `&buf[..=0xAB]`, as in `fill()` and `set_scaling()`. See datasheet Table 7, p. 11.

  **Automated:** Send a 351-byte pattern with distinct values at indices 179, 180, 349, and 350 through a recording I²C mock. Assert the two payloads cover exactly PG0 `0x00–0xB3` and PG1 `0x00–0xAA`, including the last value. **Hardware:** Read back all PWM registers and compare against the pattern. The Framework mapping does not use SW9, so PG1 `0xAA` (SW9/CS39) has no visible LED on this board; a visual test alone cannot catch this bug.

- [x] **Preserve configuration bits in `shutdown()`.**
  **Status:** Fixed in `0d38b86`. Host test covers all 256 configuration bytes and both shutdown states. Hardware baseline `3e73edd` failed preservation; fixed `9e7cae9` passed three times. Subsequent baseline requests failed initial-state checks because the first failure left shutdown active; this limitation is recorded in the hardware report. Optical blanking was not separately measured.

  **Original finding:** Writing the whole configuration register as `0` or `1` clears the scan-row selection, logic threshold, and detection bits. Modify only SSD (bit 0), preserving the other bits. In particular, shutdown must retain a previous `sw_enablement()` setting. See Table 11, p. 14.

  **Automated:** Start with each valid SWS value and both LGC values; exercise shutdown on/off and assert only bit 0 changes. Include OSDE bits in the preservation check without repeatedly triggering detection on hardware. **Hardware:** Select `Sw1Sw8`, capture PG4 `0x00`, cycle shutdown, and read it again. Expect `0x10` while shut down and `0x11` when running if other bits are zero. Confirm blanking and restoration of the same LED pattern. Register readback is the decisive scan-setting check.

- [x] **Fix core pixel coordinate bounds.**
  **Status:** Fixed in `21c5c1a`. Host boundary checks pass. Hardware baseline `7d71c1a` failed coordinate rejection; fixed `9e7cae9` passed three times, including corner register-image checks. Camera still showed four bright corners.

  **Original finding:** `pixel()` used `x > width` and `y > height`; these must be `>=` because dimensions are counts. In the reviewed implementation an invalid coordinate could reach the mapping callback, write the wrong LED, or panic.

  **Automated:** For a 9×34 device, accept `(0,0)` and `(8,33)`; reject `(9,0)`, `(0,34)`, `(9,34)`, and coordinates of 255. Invalid calls must invoke neither the mapping callback nor I²C. **Hardware:** Display the four valid corner pixels, then issue invalid calls and confirm errors with the displayed pattern unchanged.

- [x] **Fix Adafruit graphics clipping.**
  **Status:** Fixed in `21c5c1a`. Host clipping regression failed on the parent and passes with the fix; retained suite passes in debug and release. No Adafruit hardware validation: Framework wiring cannot establish RGB layout or output.

  **Original finding:** `DrawTarget::draw_iter()` accepted `0..=13` and `0..=9`. Valid coordinates are `0..13` and `0..9`. Out-of-bounds graphics pixels must be discarded.

  **Automated:** With both Adafruit and graphics features enabled, draw valid corners plus negative coordinates, `(13,0)`, `(0,9)`, and large signed coordinates. Expect success overall and writes only for valid pixels. **Hardware:** Framework hardware cannot validate the Adafruit wiring or RGB output; use mock coverage for this item.

- [x] **Validate RGB coordinates before flattening.**
  **Status:** Fixed in `21c5c1a`. Host invalid-coordinate regression failed on the parent and passes with the fix; retained suite passes in debug and release and exercises the last valid pixel. No Adafruit hardware validation.

  **Original finding:** `AdafruitRGB13x9::pixel_rgb()` computed `x + y * 13` before checking the original coordinates. For example, `(13, 0)` aliases `(0, 1)`. Larger inputs can overflow `u8` arithmetic or access an invalid lookup index. Reject `x >= 13` or `y >= 9` before computing the index.

  **Automated:** Accept `(0,0)` and `(12,8)` and check their three mapped channel writes. Reject `(13,0)`, `(0,9)`, and inputs containing 255 without any I²C writes. Run in debug and release to catch arithmetic differences. Check that invalid `(13,0)` never updates valid `(0,1)`. **Hardware:** Adafruit-specific; no additional Framework test is needed.

- [x] **Reject unsupported PWM pages from custom mapping callbacks.**
  **Status:** Fixed in `8028a9b`, found during follow-up. Baseline host test fails on page2 and hardware V fails three requests because every nonzero page is silently mapped to page1. Fixed `pixel()` returns the existing `InvalidFrame` error for pages2–255 before I²C access. Exhaustive host invalid-page test passes with zero transactions; hardware V tests pages2/255 and passes three requests plus a repeat, with unchanged PWM and restored callback/state. T, five A reports, and U pass afterward. All six workflow test commands pass (26 all-feature debug/release, 22 core-only, 23 Framework, 24 Adafruit, 25 Adafruit+graphics); strict host/diagnostic Clippy, docs, and formatting pass. Normal updated ledtest restored and serial enumeration confirmed. Valid page0/page1 mappings are unchanged.

### Documentation corrections

No behavioral decision is needed for these wording fixes.

- [x] **Correct the documented I²C address.**
  **Status:** Corrected in `0343f73`. Before/after wording checked; `cargo doc --all-features --no-deps` passed. Matrix tests communicate at `0x30`; ID readback and alternate physical straps were not tested.

  **Original finding:** The `address` field documentation suggested `0x74`; supported seven-bit addresses are `0x30–0x33`, selected by ADDR wiring. The ID example `0x60` is the corresponding eight-bit address byte for seven-bit address `0x30`. See Table 1, p. 7, and ID register description, p. 10.

  **Review/build:** Cross-check the four seven-bit addresses and ADDR pin connections against Table 1; ensure examples use seven-bit addresses. **Hardware:** At the board’s existing strap setting, verify ACK at the configured address and read the ID (`0x60` for address `0x30`). Other strap settings need suitable hardware access and are not required to validate this board.

- [x] **Clarify `set_address()` documentation.**
  **Status:** Corrected in `2ca9a30`. Before/after wording checked and documentation build passed. Retained host address-targeting test passes; no hardware address-programming claim is made.

  This changes the address targeted by the driver, not the chip's hardware address, which is selected by the ADDR pin.

  **Automated:** Change the driver target address and verify subsequent mock transactions use it, with no address-programming register write. **Hardware:** No extra test is needed; switching to an unpopulated address would only demonstrate a NACK, not hardware address programmability.

- [x] **Correct setup's “maximum voltage” wording.**
  **Status:** Corrected in `b53dbe3`. Before/after wording checked and documentation build passed. No runtime test was needed for this wording correction.

  Setup sets maximum global current control, not voltage.

  **Review:** Confirm the text describes GCC/current control and does not claim to configure voltage. No dedicated runtime test is needed; the global-current and setup checks cover the behavior.

### Small additive APIs

The register behavior is defined. Choose names consistent with the driver, preserve unrelated fields, and avoid changing setup defaults. These are optional features rather than prerequisites for the bug fixes.

- [x] **Expose global current control.**
  **Status:** Implemented in `534400e`; register behavior and visible brightness response verified, with photometric limits below. Missing-method compile baseline only, not an on-device failure baseline. Host case passes. Fixed-only hardware readback passed for GCC 0, 1, 32, 128, and 255 on three runs (`9dfce47`). Exhaustive GCC and bus-failure host coverage passes. Fixed-exposure camera sweep at GCC 0/16/64/255 shows dark/increasing brightness; some LED cores clip, so linearity was not measured. See optical evidence directory.

  **Original finding:** PG4 `0x01` was hardcoded to `0xFF` during setup with no public setter. Setup still selects that value; `set_global_current()` now exposes control. See Table 12, p. 14.

  **Automated:** Check exact PG4 `0x01` writes for 0, 1, 128, and 255, with no changes to PWM/scaling. **Hardware:** Hold PWM and scaling fixed, sweep GCC from 0 through several modest levels, and confirm blank at zero and increasing brightness. Read GCC back. A photodiode or fixed-exposure camera can quantify brightness; USB power includes controller overhead and is not a direct LED-current measurement.

- [x] **Expose de-ghost resistor selection.**
  **Status:** Implemented in `78fb89a`; API encodings and register behavior verified. Optical efficacy remains a separate investigation below. Missing-method compile baseline only. One host encoding case passes; all 64 combinations passed hardware register readback on three runs (`9dfce47`). All 64 encodings and bus-failure host coverage pass. Camera comparisons at 5ms and, with corrected framing, 50ms show no conclusive resistor-dependent ghosting change; efficacy remains unverified. The capture tool now rejects invisible lit patterns against a dark reference. Register configuration is verified.

  **Original finding:** CS pull-up and SW pull-down selection at PG4 `0x02` had no API. `set_deghost_resistors()` now exposes both fields. Follow Rev. D's corrected PUR/PDR definitions. See Table 13, p. 15.

  **Automated:** Exercise all eight PUR and PDR encodings independently. Assert PDR occupies bits 6:4, PUR bits 2:0, and reserved bits stay zero. **Hardware:** Read PG4 `0x02` back after selecting asymmetric settings to catch swapped fields. Compare sparse bright patterns on a dark background across resistor settings using fixed exposure. No visible ghosting is not proof that selection is broken; register readback verifies configuration, while the optical test evaluates this board’s behavior.

- [x] **Expose interrupt configuration.**
  **Status:** Implemented in `bed1985`; host encodings and observed event signaling now verified. Missing-method compile baseline only. All eight host mask encodings and bus-failure checks pass; all eight combinations received hardware ACKs (`9dfce47`). Further E hardware observations verify masked open events leave INTB high, enabled events latch low until status read, and auto-clear releases between 1ms and 12ms samples. Short-event signaling was also observed; physical short diagnosis remains unresolved. Permanent interrupt regression passes three runs; see interrupt evidence directory. Diagnostic now tests GPIO28 INTB with a pull-up.

  Implement `0xF0` open/short interrupt enables and the auto-clear option. See Table 5, p. 10.

  **Automated:** Verify all combinations of IO, IS, and IAC encode into bits 0, 1, and 4 of global register `0xF0`; no reserved bits are set. It is write-only, so do not depend on readback. **Hardware:** First check the schematic for INTB routing and probe access. If accessible, use a logic analyzer/scope and a reproducible detection event to compare enabled/disabled masks and the documented auto-clear behavior after more than 8 ms. Without INTB access, mark electrical interrupt behavior unverified.

- [x] **Expose interrupt status.**
  **Status:** Implemented in `ba5b4b7`; host decoding and observed event acknowledgement now verified. Missing-method compile baseline only. All 256 host status bytes (including reserved bits) and bus-failure checks pass; hardware status access succeeded (`9dfce47`). Further E hardware checks correlate open status with exactly six unmapped active channels; first status read returns 1, second returns 0 and INTB releases. Short status2 was observed, but its 256-channel bitmap is not accepted as physical short diagnosis. Permanent interrupt regression passes three runs.

  Read open/short status from `0xF1`. See Table 6, p. 10.

  **Automated:** Supply `0x00`, `0x01`, `0x02`, and `0x03` from global register `0xF1` and check decoding of no fault, open, short, and both. Mask reserved bits and propagate bus errors. **Hardware:** Compare status with separate open/short detection reports and, if accessible, INTB. Record status before and after repeated reads to characterize any clearing behavior rather than assuming it.

- [x] **Expose logic-threshold selection.**
  **Status:** Implemented in `9e7cae9`; selection and bit preservation verified, with electrical threshold measurement outside the completed API checks. Missing-method compile baseline only. One host preservation case passes; both selections passed hardware configuration readback with continued communication on three runs (`9dfce47`). All 256 configuration bytes under both selections and every transaction failure now have passing host coverage. Actual electrical thresholds were not measured.

  Configuration bit 3 selects 1.4V/0.4V or 2.4V/0.6V thresholds. Preserve the other configuration bits. See Table 11, p. 14.

  **Automated:** Toggle bit 3 while preserving SWS, OSDE, and SSD. **Hardware:** Read the configuration back for both selections and confirm communication remains functional at the board’s normal I/O voltage. Actual threshold validation needs controlled logic amplitudes and suitable equipment; successful communication alone does not measure the thresholds.

## Think through before implementing

These involve an API contract, compatibility choice, hardware uncertainty, or a measurement that could change the implementation. Each item states the decision to settle before coding; the existing test plan follows it.

### Public API and startup contracts

Settle these before expanding the API surface or changing existing caller behavior.

- [ ] **Validate `fill_matrix()` input length before any I²C writes.**
  **Status:** Partial fix in `3e73edd`: short inputs now assert before I²C access; extra values are documented as ignored. Host tests cover panic-before-write for lengths 0, 179, 180, and 350 and ignored extra data at length 352. The planned recoverable-error versus fixed-array contract remains undecided; do not describe partial hardware writes as current behavior.

  **Decision:** Keep a slice and return a validation error, or require a fixed-size array? Decide whether extra bytes are accepted and account for return-type/caller compatibility. The current short-input assertion prevents partial writes but still panics.

  **Original finding:** The function accepted arbitrary slices and could panic after updating PG0 for lengths 180–350. Current code asserts before any I²C access; the remaining question is the error/type contract. Validate up front and return an error, or accept `&[u8; 351]`.

  **Automated:** If keeping a slice API, try lengths 0, 179, 180, 350, 351, and 352. Invalid lengths must return an error before any I²C operation; document and test whether extra bytes are rejected or ignored. If using an array API, compile a valid `[u8; 351]` example and verify wrong-sized arrays fail to compile. **Hardware:** For a runtime-validated API, preserve a known pattern across invalid calls and check returned errors; mock coverage is sufficient for the no-write guarantee.

- [x] **Resolve scaling initialization in `setup()`.**
  **Status:** Documented in `f5f6f49`: existing behavior without changing startup defaults: setup leaves PWM/scaling zero and requires an explicit `set_scaling()` call. A future default change remains a separate design decision. New U diagnostic verifies all 351 PWM/scaling bytes are zero after setup, GCC255/config1, then all scaling bytes50 after an explicit setter. U passes three hardware requests before the wording change and repeated requests afterward, with the full original state restored. Documentation build passes.
  **Resolution:** Preserve the existing zero-scaling startup and document the explicit scaling call. Any change to the default is a future behavior decision, not unfinished work on this documentation item.

  Reset clears all scaling registers to zero, and setup leaves them there. PWM writes therefore produce no light until `set_scaling()` is called. Either initialize scaling during setup or clearly document the required call. Existing examples already call `set_scaling()`. See Tables 7 and 9, pp. 11 and 13.

  **Automated:** Assert the documented setup postcondition: either all 351 scaling values are initialized, or setup leaves them zero and the documented example explicitly sets them. **Hardware:** From reset, run setup, apply a modest PWM pattern, then set scaling. Confirm behavior matches the selected contract. Repeat setup after changing settings to verify reset behavior, not just cold boot.

- [x] **Remove or explain `addresses::SHUTDOWN = 0x0A`.**
  **Status:** Explained in `1bc2913` as a misleading legacy constant, retaining the existing value and source compatibility without adding a deprecation warning. Documentation directs callers to `shutdown()`, which uses configuration bit 0. Any future removal remains a separate compatibility decision. Baseline source search found no Rust uses; before/after documentation builds pass and rustdoc link resolves. No runtime change or new hardware test.
  **Resolution:** Retain and explain the legacy constant without breaking source compatibility or adding warnings. Future removal/deprecation is not part of this completed item.

  It is unused and does not correspond to a shutdown register in this datasheet. Software shutdown is configuration bit 0 in PG4 `0x00`.

  **Review/build:** Search for uses of `SHUTDOWN`, remove or document the misleading constant, and build all feature combinations used by the project. If removing a public constant, record the API compatibility impact. The shutdown register test above verifies the actual mechanism; no separate hardware test is needed.

- [ ] **Expose per-channel current scaling.**
  **Decision:** Expose raw hardware channel indices, logical coordinates, or both? Share mapping and bounds validation with PWM without making RGB channel semantics ambiguous.

  PG2/PG3 support independent 8-bit scaling for every LED; `set_scaling()` currently applies one value to all channels. See Table 9, p. 13.

  **Automated:** Check channel-to-register mapping at both page boundaries and the final channel, plus rejection of invalid channels. Verify only the selected scaling byte changes. **Hardware:** Keep PWM and GCC fixed; change scaling for one mapped LED through 0 and several levels. Confirm its brightness changes while a neighboring reference LED remains steady; repeat for LEDs on both scaling pages and read registers back.

### Diagnostics and hardware behavior

These need a defined lifecycle and hardware evidence, beyond correct register encodings.

- [ ] **Implement open/short detection triggers.**
  **Status:** Diagnostic-only E/Q workflow measured drive preconditions, interrupt transitions, and result behavior; no public detection API added. Full PWM/scaling produced six expected open flags; lower-drive controls produced none. Physical meaning of short bitmap and a guaranteed completion condition remain unresolved. Lifecycle/state-restoration design is still pending.
  **Decision:** Choose synchronous versus staged detection, establish completion timing, and define how GCC and configuration are restored after success or bus failure.

  Configure OSDE (configuration bits 2:1). Set GCC to `0x01` before detection, and transition OSDE through `00` when retriggering. Define how the API restores the previous current setting. See pp. 14–15.

  **Automated:** Assert GCC becomes `0x01` before OSDE is enabled, retriggers pass through `00`, and SWS/LGC/SSD are preserved. Test repeated open and short triggers and injected I²C failures, including the chosen current-restoration/error policy. **Hardware:** Run open and short detection separately, twice each, and record configuration, GCC, and result bytes. Do not treat the first available result read as proof of completed detection: establish completion timing from further documentation or measurements before restoring GCC. Use naturally unpopulated channels for an open-detection check; a healthy board cannot prove short detection against a known fault.

- [ ] **Read and decode per-LED open/short results.**
  **Decision:** Choose a raw bitmap versus decoded channel/coordinate interface and how it relates to detection completion. Account for unpopulated channels in Framework results.

  PG4 `0x03–0x2F` stores the detection results. See Tables 14-1/14-2 and Figure 10, p. 15.

  **Automated:** Decode synthetic one-hot bitmaps for all 351 channels, including every five-byte SW boundary and CS33/CS39; ignore the unused bit 7 of each fifth byte. Check that results map to the correct LED coordinates, independently of the PWM page layout. **Hardware:** Save raw 45-byte open and short reports and decode them against the Framework population map. Across all nine SW rows the mapping uses 306 of 351 channels. With SW1–SW8 enabled, only six active channels are unmapped (SW3–SW8/CS39); E observations reported exactly those six. SW9 is disabled, so its 39 unused channels are not expected active opens. A known populated-channel fault requires a suitable fixture or spare board; do not short or remove LEDs on the working matrix for this review.

- [ ] **Establish de-ghost efficacy on a reproducible ghosting case.**
  **Status:** The API and all 64 register combinations are verified. Camera comparisons at 5ms and corrected-framing 50ms did not establish a conclusive resistor-dependent effect. This is an evidence limit, not an unimplemented resistor API. See optical records from `39a7c04` and `472eb4e`; no efficacy claim is made.

  **Next evidence needed:** A repeatable unintended-LED baseline distinguishable from reflected light, camera bloom, and intended LED output, or suitable optical/electrical measurements. Do not mark this complete merely because register readback passes.

### Performance and memory tradeoffs

Transaction counts are source-level estimates, not measured speedups. Resolve correctness issues before optimizing. Commit `3e73edd` includes the PG1 length correction and an up-front short-input assertion. Frame readback has before/after hardware evidence; the assertion has host coverage, while its longer-term error contract remains undecided.

- [ ] **Use bulk writes for Framework uniform fills.**
  **Decision:** May a Framework fill overwrite unmapped channels, and what guarantee applies to custom mappings? Choose semantics before replacing the current loop.

  `LedMatrix::fill_brightness()` currently performs 306 pixel updates, each with unlock, page selection, and a register write: 918 I²C write calls. The existing core `fill()` uses six calls for both complete pages. Decide whether the wrapper may update all 351 hardware channels or must preserve unmapped channels. `LedMatrix::new()` accepts a custom mapping, so a blanket replacement can change behavior for custom layouts or mappings that intentionally repeat channels.

  **Automated:** For the supported mapping, compare the resulting PWM register image against the existing per-pixel path and assert the intended transaction reduction. Cover brightness 0 and 255, both pages, unmapped-channel policy, and custom-mapping behavior. **Hardware:** Compare visible output and elapsed fill time at the same bus speed, brightness, GCC, and scaling; record transaction count and bytes on a logic analyzer if available.

- [ ] **Add optional buffered drawing and bulk RGB fills.**
  **Decision:** Choose explicit buffering ownership, RAM cost, flush/retry semantics, and how it interacts with immediate writes and graphics traits.

  `AdafruitRGB13x9::fill_rgb()` currently uses 117 × 3 × 3 = 1,053 I²C write calls. A hardware-order framebuffer needs 351 bytes of persistent RAM and can flush through the two-page bulk path in six calls. Offer buffering as an explicit wrapper/API so callers can choose its RAM cost and deferred-update behavior. Route buffered graphics drawing and RGB fills through the same mapping, preserving channel order and page boundaries. Consider optimized `DrawTarget` clear/fill methods where they help avoid per-pixel bus writes.

  **Automated:** Compare buffered output with the existing RGB mapping for all 117 pixels, using distinct R/G/B values. Assert drawing alone performs no bus access and a full flush sends every channel in six writes. Check clipping and failed-flush retry behavior, including failure after the first page was sent. **Hardware:** Use Adafruit hardware for color/layout validation; measure frame time and bus traffic on the available board separately. A two-page flush is not an atomic visual update.

- [ ] **Skip unchanged framebuffer data and coalesce dirty ranges.**
  **Decision:** Depends on buffering. Decide dirty-state storage, coalescing policy, failure recovery, and resynchronization after external writes/reset.

  After a buffered API exists, track changed register ranges per page and send contiguous runs. Avoid a full-frame transfer for a sparse animation or an unchanged frame. Compare the cost of merging nearby runs against the extra register/address and page-selection overhead; avoid assuming one write per changed pixel is cheapest. Document dirty-tracking RAM separately from the 351-byte buffer, and define resynchronization after reset or external register writes.

  **Automated:** Check unchanged frames cause no traffic, edits spanning a page boundary stay in separate payloads, and sparse/dense edits produce the correct final register image. Inject failures at each flush stage and retain enough dirty state to retry all data not known to be committed. **Hardware:** Compare unchanged, single-pixel, scattered-pixel, and full-frame workloads at a fixed bus speed.

- [ ] **Avoid redundant page selections where ownership permits.**
  **Decision:** Choose bus ownership and cache invalidation rules before introducing a persistent page cache; operation-local batching may avoid a public API change.

  `write_register()` always unlocks and selects a page, even for successive writes to the same page. Caching can reduce subsequent same-page pixel updates from three I²C write calls to one. However, `i2c` and `address` are public, so callers can change the chip's selected page or target without notifying the driver. Choose an encapsulated API with explicit invalidation/access guards, an opt-in exclusive-ownership wrapper, or operation-local batching before introducing a persistent cache. Record compatibility implications for public struct literals and fields.

  **Automated:** Check repeated same-page writes, page transitions, reset, address changes, external-access invalidation, and I²C failures. Never record a page as selected before selection succeeds; invalidate conservatively after uncertain bus outcomes. **Hardware:** Compare a same-page pixel sequence and an alternating-page sequence; verify PWM writes land on the intended pages after reset and recovery.

- [x] **Measure lookup-table storage and mapping cost before changing it.**
  **Status:** Optimized in `4d82cbb`. Baseline `0d66ff2` emitted per-call stack initialization (Framework 892 bytes plus saves; Adafruit 836 plus saves). Static storage preserves every frozen mapping and reduces linked release ledtest text by 1,460 bytes. Framework hardware mapping-only workload dropped from ~472ms to ~16.4ms per 30,600 calls, checksum unchanged; T/A checks pass before/after. Reproducible probe, exact results, and limitations are in `measurements/README.md`. Normal firmware restored. No public mapping architecture change was needed.
  **Resolution:** Measurements justified static storage, with no mapping API change. The retained probe and frozen mappings document the comparison.

  **Original finding:** Adafruit and Framework declared local tables containing 702 and 612 bytes respectively. Actual release probes demonstrated stack initialization; both tables are now static. These table-content sizes differ from the measured stack reservations. Inspect release output for `thumbv6m-none-eabi`; consider explicit `static` tables or arithmetic for regular sections only if code size, stack use, or CPU time improves. Function-pointer mapping also limits inlining, but a generic mapping API should be justified by measurements because I²C traffic is the larger visible opportunity.

  **Automated/review:** Compare release code size and generated assembly/stack use before and after any change. Exhaustively verify all valid coordinates against the existing mappings, including irregular Framework edge wiring and RGB channel permutations. **Hardware:** Measure mapping CPU cost separately from I²C transfer time if pursuing this optimization; no dedicated optical test is needed when the mapping remains identical.

## Reviewed and consistent with the datasheet

- Page selection and the `0xC5` unlock sequence.
- Reset register `0x3F` and reset value `0xAE`.
- PWM frequency register and enum values.
- Scan-row setting enum values.
- Transfer lengths in uniform `fill()` and `set_scaling()`.
- Separate register-address write and data read: Figure 7, p. 8, shows STOP followed by START.

## Next work, once decisions or evidence are available

1. Choose the `fill_matrix()` validation contract (recoverable slice error versus fixed-size array).
2. Choose per-channel scaling addressing and the public detection/result lifecycle. Use the retained E/Q observations to address drive preconditions, status acknowledgement, and unresolved short results before selecting a completion policy.
3. Choose uniform-fill semantics for custom/unmapped channels before bulk wrapper writes; choose buffering ownership and flush/retry behavior before dirty tracking.
4. Choose bus ownership/invalidation before persistent page caching. Static-table storage is already optimized and measured.
5. Pursue de-ghost efficacy and physical short diagnosis only with a reproducible fault/optical baseline. They are not demonstrated fixes awaiting an unchecked implementation step.

## Regression coverage and diagnostic follow-up

- [x] Document the repeatable issue workflow for handoff. Includes baseline/replay rules, host and hardware checks, checked-in tool/command inventory, restoration and evidence limits, per-issue evidence template, and factual commit requirements. Validated four shell snippets with `bash -n`, embedded Python with `compile`, local links/harness paths, referenced commits, and `git diff --check`. Documentation only; no new runtime or hardware validation claimed.

- [x] Strengthen register API host coverage. Baseline: existing 18 tests passed. After adding exhaustive parameter, preservation, and per-transaction failure checks: 23 all-feature tests pass in debug/release; 21 pass without default features; strict host Clippy passes. This is coverage expansion, not a demonstrated driver bug fix. No additional hardware test was run for this host-only change.
- [x] Report each diagnostic API independently and restore state after failures. Baseline: old `A` gave one combined line; historical shutdown failure contaminated subsequent requests. Revised hardware: three `X` deliberate failures restored state, then `A X A T A` returned distinct API lines and successful reruns with verified restoration. Strict diagnostic Clippy and build pass. Physical bus disconnection was not tested; cleanup errors explicitly report uncertain state. See hardware run record.
- [x] Capture controlled camera brightness and ghosting comparisons. Zero-current baseline and GCC16/64/255 sweep show increasing brightness; some LED cores clip. Repeated sparse disabled/8kΩ/32kΩ captures show no conclusive ghosting difference. Cropped evidence, three-frame RGB/clipping metrics, settings, and limitations are retained under `examples/ledmatrix/test-results/2026-11-13-optical`. Permanent capture script ran end to end and restored saved matrix state and original camera controls. This completes the comparison task, not proof of de-ghost efficacy.
- [x] Replace placeholder register API documentation with parameter meanings, reset defaults, register scope, and detection/INTB limitations. Baseline: five methods shared the generic “Configure or inspect” sentence; documentation built. After: reviewed against datasheet tables, documentation builds, all 23 host tests and strict Clippy pass. Documentation only; no new hardware test for this change.

- [x] Preserve exhaustive mapping fixtures and a repeatable Cortex-M0+ measurement probe. Both fixture tests pass before/after the static-table change; 25 all-feature debug/release tests, 21 no-default tests, and strict host/diagnostic Clippy pass.

- [x] Investigate INTB/status/event behavior using existing board wiring. Three permanent hardware regression runs pass open-map, masking, read-ack, auto-clear interval, and short-event signaling checks. Diagnostic snapshots now include scaling. Full reports and limits are retained under `examples/ledmatrix/test-results/2026-11-13-interrupts`.

- [x] Reject off-target camera evidence automatically and repeat de-ghost comparison at longer exposure. Baseline script completed an outdated-crop run despite nearly black lit frames; new visibility guard rejected it (exit1) and restored state. Corrected-crop 50ms run completed seven settings. No conclusive electronic ghosting difference was established. Evidence/limits are in the longer-exposure results directory; matrix T/A and camera-restoration checks pass, and normal firmware is restored.
- [x] Add independent device-feature coverage and fixture-change CI triggers. Baseline workflow covered all-features/no-default only and did not list CSV fixtures in its path filters; three standalone device test commands already passed locally. Revised workflow includes Framework-only, Adafruit-only, and Adafruit+graphics, and both Rust job filters include `tests/**`. All six workflow test commands ran locally and passed (25 all-feature debug/release, 21 core-only, 22 Framework, 23 Adafruit, 24 Adafruit+graphics). Formatting/whitespace checks pass. No GitHub Actions or new hardware run is claimed for this CI-only change.

## Remaining decisions after autonomous follow-up

The checked entries above now include measured static-table optimization, actual
interrupt signaling, camera-evidence rejection, compatibility-preserving legacy
constant explanation, existing setup-contract documentation, and invalid-page
validation. The last recorded hardware state is normal updated ledtest with camera controls
restored; neither was rechecked during this documentation reconciliation. Remaining unchecked items are intentional:

- Recoverable slice-error versus fixed-array contract for `fill_matrix()`.
- Per-channel scaling interface and production detection/result lifecycle.
- Uniform-fill semantics for custom/unmapped channels, buffering/dirty tracking,
  and page-cache ownership/invalidation.
- De-ghost efficacy remains unproven despite two usable exposure comparisons.
- The short bitmap's physical meaning and a guaranteed detection-completion
  condition remain unresolved. No physical short was introduced.
