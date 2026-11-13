# Hardware validation — 2026-11-13

Driver: `8176735`. USB bootloader support: `c96cdbc`.
Attached Framework LED Matrix: application USB `32ac:0020`, RP2040 bootloader `2e8a:0003`.
Physical board revision and IC marking were not inspected.
Settings: I2C 1 MHz, scaling 50/255, PWM 29 kHz, SW1–SW8 enabled.

Three consecutive diagnostic requests returned:

```
PASS: PWM351 bounds shutdown resume
```

Each run verified:

- Patterned frame readback across all 180 PG0 and 171 PG1 registers, including PG1 `0xAA`.
- Four valid corner writes and rejection of `(9,0)`, `(0,34)`, `(9,34)`, `(255,255)`, followed by full PWM readback confirming the corner pattern.
- Configuration readback before shutdown, during shutdown, and after resume: only SSD changes, preserving SW1–SW8 selection.
- Full PWM readback after resume confirming the pattern is retained.

Both diagnostic and original `ledtest` entered USB bootloader mode using `32 AC 02 00`. Finally restored `ledtest` and confirmed application USB enumeration.

These are register checks; optical output and Adafruit wiring were not verified. Adafruit bounds/clipping have host mock coverage.

## Repeat

From this directory:

```
cargo build --example hardware_check
elf2uf2-rs target/thumbv6m-none-eabi/debug/examples/hardware_check hardware_check.uf2
```

Send the bootloader command to the module serial port. Wait for RPI-RP2 to mount and copy the UF2 onto it. Open the resulting serial port at 115200 baud, send ASCII `T`, and read a report line. Repeat as desired. USB polling and bootloader entry remain active after a reported failure. Tests leave a four-corner pattern on success; on failure they may leave an intermediate test state.

Restore normal operation by sending the bootloader command, building `ledtest`, converting it with `elf2uf2-rs`, and copying its UF2 to RPI-RP2.

Host checks passed: 10 all-feature tests, 8 without defaults, strict all-feature Clippy, formatting, LED matrix example Clippy, embedded library build. Adafruit examples cross-built but emitted existing linker warnings about missing `_start`; that build alone does not establish runnable Adafruit firmware.

## Corrective before/after hardware validation — 2026-11-13

Earlier API commits had host-only validation. The results below were obtained
subsequently; they do not retroactively establish hardware testing before those
commits. Driver revisions were exported to isolated directories under
`/tmp/is31-hil-before-after`. All used the same expanded diagnostic source and
current example manifest/lockfile (for USB reporting), with each revision's
unaltered driver source. Settings: 1 MHz I²C, scaling 50, 29 kHz PWM, SW1–SW8.
Physical IC marking and board revision remain uninspected.

| Check | Actual pre-fix revision | Observed baseline | Fixed revision `9e7cae9` |
| --- | --- | --- | --- |
| Full 351-channel PWM readback | `21c5c1a` | `FAIL: PWM351 readback` (one request) | `PASS: PWM351` (three requests) |
| Coordinate rejection and corner register image | `7d71c1a` | `FAIL: invalid coordinate` (three requests) | `PASS: bounds` (three requests) |
| Shutdown configuration preservation and resume | `3e73edd` | First request: `FAIL: config preservation`; subsequent two: `FAIL: initial config` because the failing test left the chip shut down | `PASS: PWM351 bounds shutdown resume` (three requests) |

Commands `F`, `C`, and `S` select these checks. `C` skips the frame check so the
older frame bug cannot hide the coordinate result. `S` currently runs the full
frame/bounds/shutdown sequence. A camera still captured with the fixed bounds
firmware displayed four bright corner LEDs. This is a qualitative corner-pattern
observation, not a brightness, ghosting, or threshold measurement.

### New API hardware checks

Command `A` on driver `9e7cae9` returned, on three consecutive requests:

```
PASS: GCC resistor logic readback; IRQ ACK/status only
```

- GCC values 0, 1, 32, 128, and 255 matched PG4 `0x01` readback; previous GCC restored.
- All 64 pull-down/pull-up combinations matched PG4 `0x02`; default 8kΩ/8kΩ restored.
- Both logic selections matched configuration readback with unrelated bits preserved; communication continued; low-threshold selection restored.
- All eight interrupt-mask combinations were accepted over I²C; mask restored to zero. Status read succeeded. No known fault event or INTB assertion was tested, so interrupt functionality remains unverified.

These are fixed-revision checks only. Their parent revisions lack the APIs;
the earlier missing-method compile failures demonstrate absence, not an on-device
runtime failure. No synthetic no-op implementation was used as a baseline.
Adafruit-specific behavior and short-input panics were not tested on this board.

After these runs, rebuilt and restored `ledtest`; application serial enumerated
at `/dev/ttyACM0`. Host all-feature tests passed (18 total), strict all-feature
Clippy passed, and strict hardware_check Clippy passed. Expanded hardware
diagnostic and restored ledtest both built.

## Independent reporting and recovery

Baseline diagnostic (`9dfce47` firmware artifact) returned one combined API PASS
line on each of three `A` requests. Its earlier shutdown baseline left the chip
shut down after failure, contaminating subsequent requests (see above).

The revised diagnostic snapshots all PWM bytes, configuration, GCC, resistor
selection, and PWM frequency before each check. It attempts all restore writes
on failure and success, then compares a fresh snapshot. Scaling is untouched.
The diagnostic owns the write-only interrupt mask and returns it to disabled;
it does not claim to read or preserve an arbitrary prior mask. Cleanup bus errors
are reported as state uncertain. Physical bus-disconnection recovery was not tested.

Hardware results: three `X` requests each returned
`FAIL: deliberate injection; state restored`. The injection changes GCC, resistor
selection, software shutdown, and PWM data before failing. Then `A X A T A`
returned five distinct API PASS lines per `A`, the expected deliberate failure
for `X`, and a full frame/bounds/shutdown PASS for `T`. Every successful line
reported state restored after matching readback. No firmware reset was required
between commands. IRQ PASS lines explicitly cover mask ACK/status access only.

Expanded firmware built and passed strict example Clippy. Host tests: 23
all-feature debug/release, 21 no-default-feature, strict host Clippy passed in
the preceding coverage step. No driver behavior was changed in this diagnostic step.
