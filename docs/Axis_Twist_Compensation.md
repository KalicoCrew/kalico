# Axis Twist Compensation

Position-dependent probe errors can result from twisted rails or other probe
location bias. The `[axis_twist_compensation <probe_name>]` module corrects
these errors with a two-dimensional map, including variations that cannot be
represented as independent X and Y curves.

Calibration measures the same physical bed locations twice: first with the
target probe, then with a reference probe or manual nozzle measurements. It
subtracts the target measurements from the reference measurements and removes
the mean difference. Constant probe Z offsets remain separate and should still
be calibrated normally. The reference probe's existing compensation, if any,
is used. The target's old map is temporarily removed for calibration and restored
if calibration fails or is cancelled.

## Configuration

Configure shared mesh settings in the base section. Use probe names from
`LIST_PROBES` when running calibration:

```ini
[axis_twist_compensation]
reference_probe: contact_probe
mesh_min: 20, 20
mesh_max: 180, 180
probe_count: 5, 5
algorithm: bicubic
mesh_pps: 2, 2
horizontal_move_z: 5
speed: 50
```

Omit `reference_probe`, or set it to `manual`, to measure the reference mesh
manually with the nozzle.
Named `[axis_twist_compensation <target>]` sections can override the shared
settings. Each probe has its own map. A reference probe cannot be the same
probe as its target. Calibration settings are independent of `[bed_mesh]`,
which is not required for compensation. Circular meshes use `mesh_radius`,
`mesh_origin`, and `round_probe_count`, as in bed mesh.

Set probe XY offsets correctly before calibrating. Mesh bounds describe physical
bed locations; they must be reachable by both probes, or by the target probe
and nozzle when using manual calibration. Choose a clearance height suitable
for both probes. Level the gantry and stabilize the printer's temperature
before calibration, keeping the bed and printer mechanically unchanged between
passes. The calibration does not home or level the printer automatically.

## Calibration

1. Home and prepare the printer, then run:

   ```gcode
   AXIS_TWIST_COMPENSATION_CALIBRATE PROBE=inductive_probe REFERENCE_PROBE=contact_probe
   ```

2. The target probe measures the entire configured area. The reference probe
   then measures the same locations. For a manual reference mesh, send `TESTZ`
   to find nozzle contact and `ACCEPT` to record each point through the API
   (for example, a printer web interface). Use `ABORT` to cancel.
3. The completed compensation map becomes active immediately. Run `SAVE_CONFIG`
   to create or update the target's named section, including the selected
   reference, layout, and mesh data.
4. Calibrate the target probe's constant Z offset as needed, and regenerate any
   bed-leveling mesh captured before the new compensation was active.

The calibration command waits for both passes to finish. When called from a
G-code macro, the macro pauses while the user performs manual moves and accepts
points, then continues after successful calibration. `ABORT` or a probing error
stops the macro and restores the previous map.

Calibration replaces the previous map only after both passes succeed. Errors and
manual cancellation restore that map and leave its saved data intact. The active
bed-leveling mesh and bed-mesh profiles are not replaced by calibration.

Subsequent probing, including rapid scanning, selects the map for the probe
being used. Probes without maps receive no correction. Mesh sampling outside
the calibrated area uses the nearest mesh boundary, following bed-mesh behavior.

## Legacy data

Existing `z_compensations` / `zy_compensations` arrays in the base section
are imported for the default probe if it has no existing calibration. The
conversion creates a regular mesh covering the machine's XY work area,
preserving the original additive corrections and extending the old curves
using their original linear formula. Probe offsets are accounted for when
placing the generated mesh. Conversion requires no probing or configuration
changes; the old settings remain the source until a new mesh is calibrated.

Only full-mesh calibration is supported. `AXIS` and `SAMPLE_COUNT` are rejected,
and old line-calibration coordinates are ignored. Configure mesh bounds before
calibrating again. The command's `REFERENCE_PROBE` parameter overrides the
configured reference for that run and the generated config block; without an
override, the configured reference is used.

See the [Configuration Reference](Config_Reference.md#axis_twist_compensation)
and [G-Code Reference](G-Codes.md#axis_twist_compensation) for options and commands.
