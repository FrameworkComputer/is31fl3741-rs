# Framework LED Matrix

## Hardware regression checks

Keep repeatable on-device checks in `examples/hardware_check.rs`. The firmware
accepts ASCII `T` over USB serial and reports PASS/FAIL. It checks all 351 PWM
registers, coordinate bounds, shutdown configuration preservation, and PWM
retention after resume. Tests restore and verify the saved PWM/configuration state on success and failure.
A cleanup bus error explicitly reports uncertain state. These are register checks; inspect optical
output separately.

From this directory, build and convert the diagnostic:

```sh
cargo build --example hardware_check
elf2uf2-rs target/thumbv6m-none-eabi/debug/examples/hardware_check hardware_check.uf2
```

Use the bootloader entry procedure below, then copy the UF2 to the matrix's
RPI-RP2 drive. Send `T` to the application serial port at 115200 baud and save
the report. Repeat requests to check that the workflow remains repeatable.
Restore `ledtest` using the same build, conversion, and flashing procedure when done.

Retain firmware source and concise run records in version control. Append dated
results to `HARDWARE_TEST_RESULTS.md`, including the exact driver/firmware commit
(and any uncommitted changes), board revision if known, bus speed, scaling, PWM
frequency, scan setting, report, and observations. Keep earlier run records intact.
Generated UF2 files, binaries, and temporary captures should remain untracked.
The existing historical record describes prior runs, not verification of later changes.

Host regression test commands and coverage are documented in
[`tests/README.md`](../../tests/README.md).

## Automated bootloader entry

The example exposes a USB serial port (VID:PID `32ac:0020`) and accepts the
same bootloader command as the production inputmodule firmware. From
`inputmodule-rs`, run:

```
cargo run -p inputmodule-control -- led-matrix --bootloader
```

Alternatively, send bytes `32 AC 02 00` to its serial port with Python and
pyserial (replace the port with the matrix's port):

```python
import serial

with serial.Serial("/dev/ttyACM0", 115200, timeout=1) as port:
    port.write(bytes.fromhex("32 AC 02 00"))
    port.flush()
```

The matrix re-enumerates as `RPI-RP2`, ready for UF2 flashing. Repeat this
command after each flash to return to the bootloader without touching the
hardware. Only bootloader entry is implemented from the inputmodule protocol.
Host sleep also enters the bootloader when SLEEP# is connected.

## Build and run with flip-link

Make sure you're in bootloader mode and then run the following command, which
automatically flashes the matrix.

```
cargo install flip-link
cargo install elf2uf2-rs
cargo run --example ledtest
```

## Building a UF2 file

To create a UF2 file for manual flashing:

```
cargo install elf2uf2-rs
cargo build --example ledtest
elf2uf2-rs target/thumbv6m-none-eabi/debug/examples/ledtest ledtest.uf2
```

Then copy `ledtest.uf2` to the RP2040 mass storage device while in bootloader mode.

Diagnostic commands: `T` runs the full original check, `F` checks complete PWM
frame readback, `C` checks coordinate rejection and corners without the frame
check, and `S` runs the full sequence including shutdown/resume. `A` checks GCC,
all resistor combinations, and logic-selection register readback, plus interrupt
mask ACKs and a status read. `A` does not establish interrupt event behavior.

The diagnostic now restores and verifies the saved PWM/configuration state after
successful and failed tests. `A` emits five separate lines; use `G` (GCC), `R`
(resistors), `L` (logic), `M` (interrupt mask ACK), or `I` (status access) alone.
`X` deliberately changes state and fails; expect
`FAIL: deliberate injection; state restored`, then rerun `A` or `T` to verify
continued operation. A cleanup error explicitly reports uncertain state.
The write-only interrupt mask is always returned to disabled. Other touched
registers are restored from readback; scaling is unchanged. Tests no longer
leave the corner pattern displayed. Older run descriptions above refer to
older diagnostic firmware.


## Camera checks

Build and flash `hardware_check` as above. Optical commands hold their pattern:
`0`, `1`, `2`, `3` display uniform PWM 128 at GCC 0, 16, 64, 255; `4`, `5`, `6`
display the same sparse pattern (x=4, y=1,5,...,33, PWM 255, GCC 255) with both
resistors disabled, 8kΩ, and 32kΩ respectively. Scaling remains at diagnostic
startup value 50. `Z` restores the state saved before the first optical command.
Restore with `Z` before running normal tests; normal tests preserve the state
present at their own start. Do not leave optical commands unattended on failure:
send `Z` again or restore normal firmware if cleanup fails.

On Linux, with `ffmpeg`, `v4l2-ctl`, and Python Pillow available:

```sh
python3 scripts/capture_optical.py --port /dev/ttyACM0 \
  --camera /dev/video0 --output /tmp/matrix-optical-new-run \
  --crop 175 185 490 280
```

Verify the selected serial device and camera first. The crop is specific to the
recorded camera position; adjust it to include the matrix after moving the camera.
The output directory must be new. The script records three frames per setting,
keeps cropped first-frame evidence and a comparison sheet, and records RGB means
and clipping counts. It uses manual 5ms exposure and 4600K white balance, then
restores the matrix with `Z` and the camera controls in `finally`. Raw full-scene
frames are temporary. It does not flash or restore application firmware.

Inspect evidence before accepting a result. Camera RGB values are not calibrated
light measurements; clipping prevents claims about linearity. Similar ghosting
images cannot establish absence of faint ghosting below this exposure's visibility.
See `test-results/2026-11-13-optical` for this run's cropped evidence and limitations.

`P` measures 30,600 Framework mapping calls without I²C, printing elapsed
microseconds and a checksum. Use a release build for comparisons; see
`../../measurements/README.md` for the before/after storage and timing study.

## Detection and interrupt observations

`E` emits four `OBS:` lines: open detection with interrupts masked, open latched,
open auto-clear, and short latched. GPIO28 reads INTB with an internal pull-up.
It uses GCC1, PWM255, scaling255 and observes the pin at 1/12/100ms; it saves and
restores all touched configuration, PWM, and scaling. The interrupt mask returns
to disabled. Read-only status/history is acknowledged or replaced, not restored.
`Q` varies scaling/PWM through 50/255, 255/128, and 50/128 using the same procedure.
These are observations, not a production detection-completion API.

Run the retained interrupt regression on the loaded diagnostic:

```sh
python3 scripts/check_interrupts.py --port /dev/ttyACM0 \
  --output /tmp/matrix-interrupt-new-run.json
```

It expects the Framework six-channel open map and observed INTB/status behavior.
An empty bitmap will fail the check rather than silently passing. Short-event
signaling is tested, but the short bitmap's physical meaning is unresolved;
no artificial short is introduced. Exact evidence and limitations are retained
in `test-results/2026-11-13-interrupts`.
