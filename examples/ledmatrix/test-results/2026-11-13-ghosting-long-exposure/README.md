# Longer-exposure sparse-pattern comparison and framing guard

Diagnostic base `93c8146`, optical patterns unchanged. Camera manual 500 exposure
units (50 ms), 4600 K white balance, dynamic frame rate disabled. This is 10× the
previous exposure. GCC255, scaling50, PWM255, same nine sparse LEDs; resistor
settings disabled/8kΩ/32kΩ/8kΩ/disabled, bounded by dark GCC0 frames. Three frames
per setting; cropped evidence and comparison sheet are adjacent.

The old crop no longer contained the matrix. A nearly black initial run completed
with the original script, but human inspection rejected it. A full-frame framing
check showed the lit matrix lower in the camera view. Full-scene images are not
retained here. `rejected-framing.json` retains the failed measurement context.

The capture tool now starts with a dark reference and requires each lit crop's
mean to exceed it by 0.05 camera RGB units (configurable). Repeating the wrong
crop returned exit1 with a visibility error, while its finally cleanup restored
matrix and camera state. With corrected crop (200,330)-(465,420), all seven
settings passed visibility and capture completed normally. This guard detects
missing lit evidence, not correct framing of every LED or calibrated brightness.

Mean RGB values across three frames:

| Condition | Range |
| --- | --- |
| Dark before | 0 |
| Disabled | 6.486–6.502 |
| 8kΩ | 6.470–6.480 |
| 32kΩ | 6.456–6.466 |
| 8kΩ repeat | 6.461–6.466 |
| Disabled repeat | 6.489–6.498 |
| Dark after | 0 |

Faint surrounding matrix detail is visible at this exposure, alongside clipped
bright cores and light spread. The resistor settings do not produce a conclusive
visual difference attributable to electronic ghosting. Total-crop means include
intended light, reflections, and camera effects; they are not a ghost-current
measurement. Do not claim de-ghost efficacy or absence of ghosting from this run.
Register selection and field encodings are already independently verified.

After both the rejected and accepted runs the script sent Z and restored original
camera controls. Post-run T and A checks and camera control readback are recorded
in HARDWARE_TEST_RESULTS.md. Normal ledtest firmware was restored afterward.
