# IS31FL3741A datasheet review

Compared against IS31FL3741A datasheet Rev. D, November 6, 2024, on November 13, 2026.
Local source: `~/Documents/datasheets/IS31FL3741A_DS.pdf`.

This is a static code and datasheet review. Findings have not been verified on hardware.
Check an item when it has been fixed or explicitly resolved.

## Implementation plan

I have one framework led matrix connected, with the example running. You can use the bootloader interface to reset and flash it.
See ~/clone/inputmodule-rs for tools and instructions.

For each item:

- Test without the fix first, to get a baseline
- Test the fix
- Commit. The message must include that how it was tested. But only what you actually tested. No hypotheticals

## Test approach

Use a recording/scripted `embedded_hal::i2c::I2c` mock for register transactions, decoding, boundary cases, and error propagation. These tests do not require attached hardware. Add regression tests with each substantive fix/API rather than tests that only repeat constants.

Use the existing `examples/ledmatrix` RP2040 firmware as the starting point for a hardware diagnostic. It already selects `Sw1Sw8`, matching the Framework mapping. A diagnostic needs a way to report errors and register dumps over USB/serial, plus register-read support (currently private in the driver). Any diagnostic page selection must use the unlock sequence and restore state or reinitialize between cases.

Before flashing, identify the attached board and firmware recovery path, confirm its chip marking/revision against this **A** datasheet, and choose modest scaling/GCC for visible tests. Firmware flashing and physical fault injection have not been performed as part of this review. Register readback and optical observation complement mock tests; they do not replace them. Framework hardware does not validate Adafruit-specific layout behavior.

Record firmware revision, board revision, settings, expected/observed values, and pass/fail for each hardware run. Leave items unchecked until their implementation/documentation resolution and relevant checks are complete; explicitly record hardware checks that remain unavailable.

## Straightforward work

These have a clear expected behavior and can be implemented independently without choosing a new architecture. “Straightforward” describes the decision involved; substantive code changes still need the tests listed below. Items remain unchecked until resolved and verified.

### Correctness fixes

Small, focused changes with existing APIs and clear regression cases.

- [ ] **Fix the PG1 transfer length in `fill_matrix()`.**
  In `src/lib.rs`, the second transfer uses `&buf[..=0xAA]`: 171 bytes including the register address, so only 170 PWM values are sent. PG1 has 171 registers (`0x00–0xAA`), leaving the final channel unchanged. Use `&buf[..=0xAB]`, as in `fill()` and `set_scaling()`. See datasheet Table 7, p. 11.

  **Automated:** Send a 351-byte pattern with distinct values at indices 179, 180, 349, and 350 through a recording I²C mock. Assert the two payloads cover exactly PG0 `0x00–0xB3` and PG1 `0x00–0xAA`, including the last value. **Hardware:** Read back all PWM registers and compare against the pattern. The Framework mapping does not use SW9, so PG1 `0xAA` (SW9/CS39) has no visible LED on this board; a visual test alone cannot catch this bug.

- [ ] **Preserve configuration bits in `shutdown()`.**
  Writing the whole configuration register as `0` or `1` clears the scan-row selection, logic threshold, and detection bits. Modify only SSD (bit 0), preserving the other bits. In particular, shutdown must retain a previous `sw_enablement()` setting. See Table 11, p. 14.

  **Automated:** Start with each valid SWS value and both LGC values; exercise shutdown on/off and assert only bit 0 changes. Include OSDE bits in the preservation check without repeatedly triggering detection on hardware. **Hardware:** Select `Sw1Sw8`, capture PG4 `0x00`, cycle shutdown, and read it again. Expect `0x10` while shut down and `0x11` when running if other bits are zero. Confirm blanking and restoration of the same LED pattern. Register readback is the decisive scan-setting check.

- [ ] **Fix core pixel coordinate bounds.**
  `pixel()` uses `x > width` and `y > height`; these must be `>=` because dimensions are counts. Currently an invalid coordinate can reach the mapping callback, write the wrong LED, or panic.

  **Automated:** For a 9×34 device, accept `(0,0)` and `(8,33)`; reject `(9,0)`, `(0,34)`, `(9,34)`, and coordinates of 255. Invalid calls must invoke neither the mapping callback nor I²C. **Hardware:** Display the four valid corner pixels, then issue invalid calls and confirm errors with the displayed pattern unchanged.

- [ ] **Fix Adafruit graphics clipping.**
  `DrawTarget::draw_iter()` accepts `0..=13` and `0..=9`. Valid coordinates are `0..13` and `0..9`. Out-of-bounds graphics pixels must be discarded.

  **Automated:** With both Adafruit and graphics features enabled, draw valid corners plus negative coordinates, `(13,0)`, `(0,9)`, and large signed coordinates. Expect success overall and writes only for valid pixels. **Hardware:** Framework hardware cannot validate the Adafruit wiring or RGB output; use mock coverage for this item.

- [ ] **Validate RGB coordinates before flattening.**
  `AdafruitRGB13x9::pixel_rgb()` computes `x + y * 13` before checking the original coordinates. For example, `(13, 0)` aliases `(0, 1)`. Larger inputs can overflow `u8` arithmetic or access an invalid lookup index. Reject `x >= 13` or `y >= 9` before computing the index.

  **Automated:** Accept `(0,0)` and `(12,8)` and check their three mapped channel writes. Reject `(13,0)`, `(0,9)`, and inputs containing 255 without any I²C writes. Run in debug and release to catch arithmetic differences. Check that invalid `(13,0)` never updates valid `(0,1)`. **Hardware:** Adafruit-specific; no additional Framework test is needed.

### Documentation corrections

No behavioral decision is needed for these wording fixes.

- [ ] **Correct the documented I²C address.**
  The `address` field documentation suggests `0x74`; supported seven-bit addresses are `0x30–0x33`, selected by ADDR wiring. The ID example `0x60` is the corresponding eight-bit address byte for seven-bit address `0x30`. See Table 1, p. 7, and ID register description, p. 10.

  **Review/build:** Cross-check the four seven-bit addresses and ADDR pin connections against Table 1; ensure examples use seven-bit addresses. **Hardware:** At the board’s existing strap setting, verify ACK at the configured address and read the ID (`0x60` for address `0x30`). Other strap settings need suitable hardware access and are not required to validate this board.

- [ ] **Clarify `set_address()` documentation.**
  This changes the address targeted by the driver, not the chip's hardware address, which is selected by the ADDR pin.

  **Automated:** Change the driver target address and verify subsequent mock transactions use it, with no address-programming register write. **Hardware:** No extra test is needed; switching to an unpopulated address would only demonstrate a NACK, not hardware address programmability.

- [ ] **Correct setup's “maximum voltage” wording.**
  Setup sets maximum global current control, not voltage.

  **Review:** Confirm the text describes GCC/current control and does not claim to configure voltage. No dedicated runtime test is needed; the global-current and setup checks cover the behavior.

### Small additive APIs

The register behavior is defined. Choose names consistent with the driver, preserve unrelated fields, and avoid changing setup defaults. These are optional features rather than prerequisites for the bug fixes.

- [ ] **Expose global current control.**
  PG4 `0x01` is currently hardcoded to `0xFF` during setup. Add a public setter. See Table 12, p. 14.

  **Automated:** Check exact PG4 `0x01` writes for 0, 1, 128, and 255, with no changes to PWM/scaling. **Hardware:** Hold PWM and scaling fixed, sweep GCC from 0 through several modest levels, and confirm blank at zero and increasing brightness. Read GCC back. A photodiode or fixed-exposure camera can quantify brightness; USB power includes controller overhead and is not a direct LED-current measurement.

- [ ] **Expose de-ghost resistor selection.**
  Add configuration for CS pull-up and SW pull-down resistors through PG4 `0x02`. `PULL_UP_REGISTER` exists but has no associated API. Follow Rev. D's corrected PUR/PDR definitions. See Table 13, p. 15.

  **Automated:** Exercise all eight PUR and PDR encodings independently. Assert PDR occupies bits 6:4, PUR bits 2:0, and reserved bits stay zero. **Hardware:** Read PG4 `0x02` back after selecting asymmetric settings to catch swapped fields. Compare sparse bright patterns on a dark background across resistor settings using fixed exposure. No visible ghosting is not proof that selection is broken; register readback verifies configuration, while the optical test evaluates this board’s behavior.

- [ ] **Expose interrupt configuration.**
  Implement `0xF0` open/short interrupt enables and the auto-clear option. See Table 5, p. 10.

  **Automated:** Verify all combinations of IO, IS, and IAC encode into bits 0, 1, and 4 of global register `0xF0`; no reserved bits are set. It is write-only, so do not depend on readback. **Hardware:** First check the schematic for INTB routing and probe access. If accessible, use a logic analyzer/scope and a reproducible detection event to compare enabled/disabled masks and the documented auto-clear behavior after more than 8 ms. Without INTB access, mark electrical interrupt behavior unverified.

- [ ] **Expose interrupt status.**
  Read open/short status from `0xF1`. See Table 6, p. 10.

  **Automated:** Supply `0x00`, `0x01`, `0x02`, and `0x03` from global register `0xF1` and check decoding of no fault, open, short, and both. Mask reserved bits and propagate bus errors. **Hardware:** Compare status with separate open/short detection reports and, if accessible, INTB. Record status before and after repeated reads to characterize any clearing behavior rather than assuming it.

- [ ] **Expose logic-threshold selection.**
  Configuration bit 3 selects 1.4V/0.4V or 2.4V/0.6V thresholds. Preserve the other configuration bits. See Table 11, p. 14.

  **Automated:** Toggle bit 3 while preserving SWS, OSDE, and SSD. **Hardware:** Read the configuration back for both selections and confirm communication remains functional at the board’s normal I/O voltage. Actual threshold validation needs controlled logic amplitudes and suitable equipment; successful communication alone does not measure the thresholds.

## Think through before implementing

These involve an API contract, compatibility choice, hardware uncertainty, or a measurement that could change the implementation. Each item states the decision to settle before coding; the existing test plan follows it.

### Public API and startup contracts

Settle these before expanding the API surface or changing existing caller behavior.

- [ ] **Validate `fill_matrix()` input length before any I²C writes.**
  **Decision:** Keep a slice and return a validation error, or require a fixed-size array? Decide whether extra bytes are accepted and account for return-type/caller compatibility. The current short-input assertion prevents partial writes but still panics.

  The function needs 351 bytes but accepts an arbitrary slice. Short input panics; with 180–350 bytes it can panic after updating PG0. Validate up front and return an error, or accept `&[u8; 351]`.

  **Automated:** If keeping a slice API, try lengths 0, 179, 180, 350, 351, and 352. Invalid lengths must return an error before any I²C operation; document and test whether extra bytes are rejected or ignored. If using an array API, compile a valid `[u8; 351]` example and verify wrong-sized arrays fail to compile. **Hardware:** For a runtime-validated API, preserve a known pattern across invalid calls and check returned errors; mock coverage is sufficient for the no-write guarantee.

- [ ] **Resolve scaling initialization in `setup()`.**
  **Decision:** Should setup leave scaling at reset defaults or initialize it? Choose the documented postcondition and whether changing existing startup behavior is acceptable.

  Reset clears all scaling registers to zero, and setup leaves them there. PWM writes therefore produce no light until `set_scaling()` is called. Either initialize scaling during setup or clearly document the required call. Existing examples already call `set_scaling()`. See Tables 7 and 9, pp. 11 and 13.

  **Automated:** Assert the documented setup postcondition: either all 351 scaling values are initialized, or setup leaves them zero and the documented example explicitly sets them. **Hardware:** From reset, run setup, apply a modest PWM pattern, then set scaling. Confirm behavior matches the selected contract. Repeat setup after changing settings to verify reset behavior, not just cold boot.

- [ ] **Remove or explain `addresses::SHUTDOWN = 0x0A`.**
  **Decision:** Remove a misleading public constant in a breaking release, deprecate it, or retain it with an explanation?

  It is unused and does not correspond to a shutdown register in this datasheet. Software shutdown is configuration bit 0 in PG4 `0x00`.

  **Review/build:** Search for uses of `SHUTDOWN`, remove or document the misleading constant, and build all feature combinations used by the project. If removing a public constant, record the API compatibility impact. The shutdown register test above verifies the actual mechanism; no separate hardware test is needed.

- [ ] **Expose per-channel current scaling.**
  **Decision:** Expose raw hardware channel indices, logical coordinates, or both? Share mapping and bounds validation with PWM without making RGB channel semantics ambiguous.

  PG2/PG3 support independent 8-bit scaling for every LED; `set_scaling()` currently applies one value to all channels. See Table 9, p. 13.

  **Automated:** Check channel-to-register mapping at both page boundaries and the final channel, plus rejection of invalid channels. Verify only the selected scaling byte changes. **Hardware:** Keep PWM and GCC fixed; change scaling for one mapped LED through 0 and several levels. Confirm its brightness changes while a neighboring reference LED remains steady; repeat for LEDs on both scaling pages and read registers back.

### Diagnostics and hardware behavior

These need a defined lifecycle and hardware evidence, beyond correct register encodings.

- [ ] **Implement open/short detection triggers.**
  **Decision:** Choose synchronous versus staged detection, establish completion timing, and define how GCC and configuration are restored after success or bus failure.

  Configure OSDE (configuration bits 2:1). Set GCC to `0x01` before detection, and transition OSDE through `00` when retriggering. Define how the API restores the previous current setting. See pp. 14–15.

  **Automated:** Assert GCC becomes `0x01` before OSDE is enabled, retriggers pass through `00`, and SWS/LGC/SSD are preserved. Test repeated open and short triggers and injected I²C failures, including the chosen current-restoration/error policy. **Hardware:** Run open and short detection separately, twice each, and record configuration, GCC, and result bytes. Do not treat the first available result read as proof of completed detection: establish completion timing from further documentation or measurements before restoring GCC. Use naturally unpopulated channels for an open-detection check; a healthy board cannot prove short detection against a known fault.

- [ ] **Read and decode per-LED open/short results.**
  **Decision:** Choose a raw bitmap versus decoded channel/coordinate interface and how it relates to detection completion. Account for unpopulated channels in Framework results.

  PG4 `0x03–0x2F` stores the detection results. See Tables 14-1/14-2 and Figure 10, p. 15.

  **Automated:** Decode synthetic one-hot bitmaps for all 351 channels, including every five-byte SW boundary and CS33/CS39; ignore the unused bit 7 of each fifth byte. Check that results map to the correct LED coordinates, independently of the PWM page layout. **Hardware:** Save raw 45-byte open and short reports and decode them against the Framework population map. The current mapping contains 306 populated channels, so the other 45 are candidates for open indications, subject to confirming board wiring and detection behavior. A known populated-channel fault requires a suitable fixture or spare board; do not short or remove LEDs on the working matrix for this review.

### Performance and memory tradeoffs

Transaction counts are source-level estimates, not measured speedups. Resolve correctness issues before optimizing. The current working tree includes a PG1 length correction and an up-front short-input assertion; those still need verification under their checklist items.

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

- [ ] **Measure lookup-table storage and mapping cost before changing it.**
  **Decision:** Measure release storage, stack use, and CPU cost first. Only change representation or the mapping API if the measured benefit warrants it.

  The Adafruit mapping declares a local 702-byte table and the Framework mapping a local 612-byte table. These sizes describe table contents, not proven stack allocations: optimized builds may already place them in read-only storage. Inspect release output for `thumbv6m-none-eabi`; consider explicit `static` tables or arithmetic for regular sections only if code size, stack use, or CPU time improves. Function-pointer mapping also limits inlining, but a generic mapping API should be justified by measurements because I²C traffic is the larger visible opportunity.

  **Automated/review:** Compare release code size and generated assembly/stack use before and after any change. Exhaustively verify all valid coordinates against the existing mappings, including irregular Framework edge wiring and RGB channel permutations. **Hardware:** Measure mapping CPU cost separately from I²C transfer time if pursuing this optimization; no dedicated optical test is needed when the mapping remains identical.

## Reviewed and consistent with the datasheet

- Page selection and the `0xC5` unlock sequence.
- Reset register `0x3F` and reset value `0xAE`.
- PWM frequency register and enum values.
- Scan-row setting enum values.
- Transfer lengths in uniform `fill()` and `set_scaling()`.
- Separate register-address write and data read: Figure 7, p. 8, shows STOP followed by START.

## Suggested execution order

1. Complete the straightforward correctness and documentation fixes with focused regression checks.
2. Settle the `fill_matrix()` validation and setup/scaling contracts; verify existing working-tree fixes.
3. Add a Framework diagnostic for register dumps, corners, page-boundary patterns, and shutdown/resume. Add small register APIs as needed for the hardware checks.
4. Decide uniform-fill semantics, then measure the bulk-write improvement. Consider buffering next; dirty-range flushing depends on it.
5. Define detection completion, state restoration, and result representation before implementing the diagnostic workflow. Validate INTB only if the board exposes it.
6. Defer persistent page caching until ownership/invalidation is explicit, and mapping changes until release measurements justify them.
