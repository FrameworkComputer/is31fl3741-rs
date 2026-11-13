# Framework LED Matrix

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
