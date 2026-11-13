# Mapping storage and CPU measurements

Measured 2026-11-13 with rustc 1.99.0 (b940084d7 2026-09-28), target
`thumbv6m-none-eabi`, release optimization. Baseline driver `0d66ff2` uses local
lookup arrays; the containing commit changes only their storage to local
`static LOOKUP` declarations. Public APIs, table values, and mappings are unchanged.

| Measurement | Local arrays | Static arrays |
| --- | ---: | ---: |
| Adafruit probe local stack reservation | 836 bytes | 0 bytes |
| Framework probe local stack reservation | 892 bytes | 0 bytes |
| Saved registers in each probe | 20 bytes | 8 bytes |
| Adafruit probe function code | 1,880 bytes | 80 bytes |
| Framework probe function code | 2,114 bytes | 44 bytes |
| Linked release ledtest `text` (GNU size) | 34,092 bytes | 32,632 bytes |
| Linked release ledtest `data` / `bss` | 0 / 4 bytes | 0 / 4 bytes |

Stack values describe these emitted functions, not whole-program stack maxima
or panic paths. Before, assembly reserves stack and stores each table element
on every lookup. After, functions reference read-only table symbols. The probe
object alone does not contain the external static tables, so its whole-object
size is not a fair total-flash comparison; the linked ledtest measurement above
includes retained data/code. This is one toolchain/workload, not a general
promise about all compilers or optimization levels.

## Matrix hardware timing

Release `hardware_check`, command `P`, makes 100 complete scans of all 306
Framework coordinates (30,600 mappings). Black-boxed inputs and function pointer
prevent constant folding. Timer interval includes loop and checksum overhead,
but excludes all I²C and USB transfers. USB is serviced before/after the timed
interval. Both variants use the same benchmark source and clock configuration.

- Baseline microseconds: 471987, 471793, 471762; additional request 471842.
- Static microseconds: 16509, 16435, 16429; additional request 16446.
- Every run checksum: 4876500.

This is about 29× faster for this mapping-only workload; it is not an end-to-end
display throughput claim. `T` frame/bounds/shutdown and all five `A` API reports
passed before and after. No Adafruit hardware was available; exhaustive host
fixtures preserve all 351 Adafruit channel mappings and all 306 Framework
mappings. Both fixture tests passed before and after the change. The baseline
correctness tests passed: this is a measured performance issue, not a newly
observed incorrect LED mapping.

## Repeat

From the repository root, with the target and LLVM tools installed:

```sh
python3 measurements/measure_mapping.py --output /tmp/mapping-measurement
```

This builds the library and emits assembly, an object, and size/symbol reports.
To compare the baseline, use a separate checkout of `0d66ff2` with the same probe
and script. Compare the `probe_adafruit` and `probe_framework` prologues and code
sizes, then build the linked example separately:

```sh
cd examples/ledmatrix
cargo build --release --example ledtest
arm-none-eabi-size target/thumbv6m-none-eabi/release/examples/ledtest
cargo build --release --example hardware_check
```

Flash the release diagnostic using the documented USB bootloader procedure and
request `P`, `T`, and `A`. Restore normal firmware afterward. Do not flash the
standalone probe: it is an analysis library, not an application.

Validation for the containing change: 25 all-feature host tests pass in debug
and release; 21 pass without default features; strict Clippy passes for both
host feature configurations and release hardware_check. Measurement script ran
successfully. Normal ledtest was rebuilt/restored and serial enumeration verified.
