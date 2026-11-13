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
