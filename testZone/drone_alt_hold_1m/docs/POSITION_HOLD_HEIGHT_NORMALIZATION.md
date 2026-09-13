# Height-normalized flow and calibration-origin position hold

## 1. Correct control contract

- Altitude target: `ALT_TARGET_M = 0.50 m`, measured as clearance from the
  lowest point of the drone to the floor.
- Horizontal target: the point where initial calibration completed,
  `(positionForwardM, positionRightM) = (0,0)`.
- Horizontal tolerance: no intentional 0.50 m acceptance radius. The
  controller continuously drives estimated position error toward zero, within
  its safe velocity/tilt authority.

MTF01P raw range at the 0.50 m clearance target is approximately 0.62 m for
the current 0.12 m sensor-to-drone-bottom mounting offset.

## 2. Flow normalization across altitude

MTF01P flow X/Y is reported as `cm/s @ 1 m`. Every accepted frame is converted
using the vertical sensor-to-ground range belonging to that frame:

```text
velocity_scale = (vertical_range_m / 1.0 m) * 0.01
sample_velocity_mps = (flow_raw - calibrated_raw_bias) * velocity_scale
```

The range used here is the tilt-corrected optical geometry from the sensor to
the floor. It is not the altitude-control clearance after subtracting the
0.12 m mounting offset and is not the commanded altitude target.

Median and EMA filtering now operate after this conversion, in `m/s`. At a
changing altitude this avoids mixing raw flow state from the old height with
the scale of the new height. Downstream velocity, position and controller gains
therefore remain in SI units and do not depend on the current altitude target.

The adaptive zero-motion threshold uses the same scale:

```text
deadband_mps = min(0.12, max(0.015, 3 * raw_sigma * velocity_scale))
```

## 3. Horizontal position behavior

Once completed calibration is visible to the control loop, yaw and horizontal
position are captured once. Fresh flow is separately required before ARM.
Body-frame velocity is rotated into this fixed yaw-reference frame and
integrated. The controller uses:

```text
desired_velocity_forward = -0.40 1/s * positionForwardM
desired_velocity_right   = -0.40 1/s * positionRightM
```

The vector magnitude is capped at 0.35 m/s. Velocity PI, integral limit, tilt
limit and tilt slew remain `3.0 deg/(m/s)`, `0.3 deg/m`, `1.5 deg`, `3.5 deg`
and `8 deg/s` respectively. These bounds protect stability; they are not a
position deadband.

A flow outage freezes the last estimate and removes horizontal controller
authority after the existing 0.35 s grace. Recovery must pass the existing
0.20 s confirmation before control resumes toward the same calibration point.
Firmware does not silently redefine the current location as a new origin.

## 4. Parameter-change ledger

| Item | Old | New | Reason | Expected effect | Possible negative effect | Required test |
|---|---:|---:|---|---|---|---|
| Altitude target | `1.00 m` | `0.50 m` | Correct requested height | Lower-energy test and hold at 50 cm clearance | More ground effect and lower flow scale/texture footprint | Restrained then netted 0.50 m altitude test |
| Flow filter domain | raw first, height scale after EMA | per-sample scale to m/s, then median/EMA | Keep filter state valid across changing height | Same physical motion has comparable m/s at different heights | Range noise enters before flow filtering | Props-off measured translation at several heights |
| Deadband units | `1.5..12 cm/s` after scale | `0.015..0.12 m/s` | One SI contract downstream | No steady-height numerical change | Bad range changes threshold; accepted-range gate remains mandatory | Stationary noise at every intended height/surface |
| Horizontal reference | first airborne flow lock; cleared/relocked after loss | one post-calibration origin for powered session | Preserve the requested initial position | Error remains traceable to the calibration point | Optical/yaw drift is no longer hidden by relock/leak | Closed-path translation and flow-loss/recovery replay |
| Position leak | `0.020 1/s` | removed | A leaky estimate moves the target and reduces accuracy | Constant displacement is not forgotten | Sensor bias accumulates for longer | Long stationary drift measurement |
| Position P | `0.40 1/s` | `0.40 1/s` | No 50 cm horizontal normalization is required | Existing small-signal response retained | Gain still requires flight validation | Tune only after velocity damping is stable |
| Velocity clamp | `±0.35 m/s` independently per axis | vector magnitude `<=0.35 m/s` | Isotropic bound | Diagonal commands obey the declared maximum | Less diagonal authority than old implementation | Static vector sweep and restrained diagonal displacement |
| Build identity | `cfg=0xA41F0801`, telemetry v5 | `cfg=0xA41F0901`, telemetry v5 | Reject the misinterpreted 50 cm-radius binary | Traceable evidence | Both sketches must be flashed together | Verify startup and sender STATUS identity |

## 5. Acceptance sequence

1. Props removed, wait for `[POS] Moc calib da khoa`. Translate the frame by
   independently measured distances along each axis and diagonally. Confirm
   correct signs and that telemetry position returns near zero when the frame
   returns to the calibration point.
2. Repeat equal measured translation velocities at several sensor heights in
   the intended future commanded-height envelope. Compare normalized m/s and
   integrated metres with an independent reference.
3. Hold still for at least 60 s at each height/surface. Record mean/RMS/drift,
   quality, range and timing. Position accuracy must be reported as measured;
   there is no artificial 50 cm pass band.
4. Force short and longer flow outages. Confirm correction decays, position
   does not reset to zero, recovery waits 0.20 s, and no command step exceeds
   the existing 8 deg/s slew.
5. Validate attitude and vertical control first. Test the new 0.50 m altitude
   target on a restrained rig, then in a netted low-energy flight. Only after
   altitude is repeatable should horizontal damping and position accuracy be
   evaluated.

Optical flow is relative and yaw has no compass reference. Physical position
accuracy must therefore be measured independently; telemetry alone cannot
prove true position.
