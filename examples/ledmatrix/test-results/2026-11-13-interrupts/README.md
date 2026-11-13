# Detection and interrupt observations

Driver baseline `4d82cbb` plus the diagnostic source in this commit. Framework
matrix at 1 MHz I²C, 29 kHz PWM, SW1–SW8 enabled; GPIO28 input uses the RP2040
pull-up for open-drain INTB. No physical faults were introduced. Board revision
and chip marking remain uninspected. Production detection API remains deferred.

## Baseline and controls

At scaling50/PWM128 the original observation read status before the bitmap at
12ms and recorded no flags or faults. A full-drive follow-up reads the bitmap
before status and observes INTB at 1, 12, and 100ms. To avoid attributing the
result to multiple changed variables, Q independently varied drive levels with
the same longer observation procedure:

| Scaling | PWM | Enabled open status / bitmap | INTB at 1/12/100ms |
| --- | --- | --- | --- |
| 50 | 255 | 0 / no faults | high / high / high |
| 255 | 128 | 0 / no faults | high / high / high |
| 50 | 128 | 0 / no faults | high / high / high |
| 255 | 255 | open / six faults | low / low / low |

The three lower-drive controls repeated twice. Do not generalize these observed
preconditions into guaranteed requirements for all chips or boards. Datasheet
explicitly requires GCC1; this diagnostic keeps GCC1 throughout detection.

## E results (scaling255/PWM255/GCC1)

| Case | INTB 1ms | 12ms | 100ms | First/second status | INTB after status read |
| --- | --- | --- | --- | --- | --- |
| Open, interrupt masked | high | high | high | 1 / 0 | high |
| Open, interrupt enabled, no auto-clear | low | low | low | 1 / 0 | high |
| Open, enabled, auto-clear | low | high | high | 0 / 0 | high |
| Short, enabled, no auto-clear | low | low | low | 2 / 0 | high |

Open bitmap bits identify exactly SW3–SW8/CS39, the six unmapped channels among
SW1–SW8 in the frozen Framework mapping fixture. SW9 is disabled, so the other
39 unpopulated channels were not used as expected active opens.

The short bitmap reports CS1–CS32 on each of eight active SW rows (256 flags).
That is not interpreted as 256 physical shorts. Short-event signaling/status is
observed; the physical meaning/reliability of this diagnostic result is unresolved.
No claim is made about detecting an actual known short.

Status reads acknowledged events in these runs: first read returns 1/2, second
returns zero, and latched INTB releases after the first read. Auto-clear released
the pin between the 1ms and 12ms samples and cleared status before the 100ms read;
this does not measure an exact 8ms delay. Bitmap remained available at 12ms.
These observations do not establish a guaranteed detection completion criterion.

## Regression and cleanup

`reports.json` contains three runs from the permanent `scripts/check_interrupts.py`.
It checks the open population map, mask behavior, latched/read-ack behavior,
auto-clear interval, and short-event signaling while explicitly leaving short
physical diagnosis unvalidated. Further manual E runs produced the same results.
`X`, `T`, and all five `A` reports passed their expected recovery/status checks
after observations (X intentionally reports failure with verified restoration).

Every E/Q case snapshots/restores PWM, scaling, configuration, GCC, resistor and
frequency registers and verifies readback. The diagnostic owns the interrupt
mask and returns it to disabled. Read-only fault/status history cannot be
restored; reads/auto-clear acknowledge status. Detection results replace the
previous bitmap. These commands intentionally generate/acknowledge events.

An initial 128-byte report buffer was too small and returned `FAIL: report size`;
that run is not accepted as observation evidence. The retained diagnostic uses
384 bytes and complete reports. A control-firmware compile error was corrected
before the successfully rebuilt image was used for Q evidence; no results are
attributed to the failed build/stale artifact.
