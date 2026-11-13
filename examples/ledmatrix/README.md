# Framework LED Matrix

## Hardware regression checks

Keep repeatable on-device checks in `examples/hardware_check.rs`. The firmware
accepts ASCII `T` over USB serial and reports PASS/FAIL. It checks all 351 PWM
registers, coordinate bounds, shutdown configuration preservation, and PWM
retention after resume. Successful tests leave a four-corner pattern. A failed
test may leave an intermediate state. These are register checks; inspect optical
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
