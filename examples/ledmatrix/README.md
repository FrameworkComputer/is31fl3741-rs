# Framework LED Matrix

## Build and run with flip-link

Make sure you're in bootloader mode and then run the following command, which
automatically flashes the matrix.

```
cargo install flip-link
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
