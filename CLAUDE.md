We are working on a complete rewrite of the motion planner and more. Here's the high level scope:


Feature scope:
- Rust end-to-end for new code; single source compiled f64 host / f32 MCU. Rust links as staticlib into Klipper's existing C MCU build, which stays C for now.
- NURBS-native, internal primitive through the planner.
- Support for G2, G3, G5, G5.1. spline-fitting for older slicers that emit g1-dense gcode.
- Phase stepping with open loop steppers with BTT Octopus pro and similar (H723 chip)
- EtherCAT support as a future backend, with the planner architecturally designed to accommodate it
- Regular stepping for non-phase-capable drivers (e.g. 2209 on Z)
- Only smooth shaper support, pre-baked into NURBS. Possibly impulse shapers in the future as composition.
- Extruder is synchronized to the motion after IS is applied.
- Non-linear PA from bleeding-edge kalico, applied IS-then-PA
- Axis limits are calculated against shaped dynamics (shaper aware TOPP-RA, not fixed de-rating)
- Third order motion as primary profile
- User configurable corner rounding, shape selected to minimize the time through the corner at given quality (ringing), and respecting current limits.sed offload
- Real time communication with MCUs, no queue-based offload.
- Trajectory evaluation on MCU at modulation rate (20-40kHz) for true phase stepping. MCU receives the shape with PA and IS already baked in, to reduce load.
- Telemetry as a first-class subsystem
- Explicit position/step decoupling. For future closed loop support.
- Real-time per-axis offset applied outside the planner, for bed mesh, thermal expansion compensation, and probing.
- Asymmetric PA (separate K for accel vs decel)


Target hardware:
- A rigid machine with single spike on each axis resonance graph. 120hz on Y and 180hz on X
- With regular klipper it could achieve motion up to 1000mm/s and 65k acceleration with 65scv before skipping steps.
- Extruder could achieve roughly 50k with acceptable pressure advance before acceleration becomes too high.
- Max flow of about 80mm cubic.
- Host: Pi 5
- MCU1: Octopus Pro with H723, 4 5160 drivers for AB steppers, 1 more 5160 for extruder
- MCU2: Octopus with F4x chip, 2209 for Z

Nice to have:
- A mechanical-frequency tracking system separate from the shaper, alerting on drift without auto-applying changes



There was a previous attempt on this, but proper research haven't been done and some things were made with wrong assumptions. but some solutions could be reused. so use it for
reference, but carefully. The branch is magnum-opus.
