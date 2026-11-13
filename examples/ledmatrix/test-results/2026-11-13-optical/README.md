# Camera brightness and ghosting comparison

Driver `9e7cae9`; isolated diagnostic base `2c958af` plus optical commands in the
containing commit. Framework board, 1 MHz I²C, 29 kHz PWM, SW1–SW8, scaling 50.
Camera 640×480; manual exposure 50 (5 ms), white balance 4600 K, dynamic frame
rate off. Compare `comparison.png`; individual PNGs crop only the matrix region.
Three consecutive frames were measured per setting after discarding 15 frames
and waiting 0.5 seconds after each pattern command. Measurements are mean RGB
camera sample values in the same crop, not calibrated luminance.

| Mode | Pattern | Mean RGB range over three frames |
| --- | --- | --- |
| 0 | Uniform PWM128, GCC0 | 0.00067–0.00068 |
| 1 | Uniform PWM128, GCC16 | 4.895–4.910 |
| 2 | Uniform PWM128, GCC64 | 16.317–16.324 |
| 3 | Uniform PWM128, GCC255 | 36.752–36.756 |
| 4 | Sparse PWM255/GCC255, resistors disabled | 1.291–1.294 |
| 5 | Same sparse pattern, both resistors 8kΩ | 1.282–1.288 |
| 6 | Same sparse pattern, both resistors 32kΩ | 1.284–1.289 |
| 5 repeat | 8kΩ | 1.287–1.290 |
| 4 repeat | Disabled | 1.297–1.302 |
| 0 repeat | Uniform PWM128, GCC0 | 0.00049–0.00056 |

Visual inspection confirms uniform brightness increases at GCC16, 64, and 255;
GCC0 is dark. GCC64, GCC255, and sparse frames have clipped channel samples
(counts in measurements.json), so these images do not establish linearity.

Sparse images show the same nine intended lit LEDs. No conclusive change in
unintended illumination is visible across resistor settings. Total crop means
are dominated by the intended lit LEDs and are not a ghosting metric. This is an
inconclusive de-ghost efficacy test, not proof that ghosting is absent or that the
resistor feature has no effect. Register readback independently verified the settings.

After capture, `Z` restored the saved register image; `T` and all five `A` reports
passed. Original camera auto-exposure, dynamic frame rate, exposure value, and
auto white balance were restored (a grouped restoration partially failed, then
individual controls were restored and read back successfully). Normal `ledtest`
firmware was restored and serial enumeration confirmed.

The permanent `scripts/capture_optical.py` was also run end to end on hardware
in a second capture series. Its `finally` cleanup restores camera controls one
at a time to avoid the grouped-control failure above.
