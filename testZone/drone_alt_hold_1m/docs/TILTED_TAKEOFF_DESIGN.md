# Tilted-surface takeoff design

Analysis date: 2026-09-09

Pipeline task: TASK 4 — TAKEOFF ON SLIGHTLY TILTED SURFACE

Status: implemented; static/model and compile validation passed, hardware validation pending

## 1. Scope and evidence boundary

This task addresses attitude demand and motor distribution only during the
Task 3 takeoff ground phases and a short post-liftoff handover. It does not
retune airborne attitude/rate gains, altitude/position control, sensor fusion,
the normal airborne mixer, ESC bounds, hover throttle, landing, or Task 5
robustness behavior.

The supplied flight logs show large motor spreads and saturation in historical
airborne firmware, but contain no identified tilted-ground run and no sample at
the TAKEOFF command edge. They therefore do not validate any numeric ground
limit below. Installed tilted-surface behavior is **UNKNOWN**.

## 2. Current-path audit

### 2.1 Confirmed static root causes

1. TAKEOFF calls `resetControllers()`, which sets pitch/roll targets directly
   to the fixed trim. A craft accepted with reported tilt up to
   `ARM_MAX_ANGLE=25 deg` can therefore present the full attitude error on the
   first ground-control tick.
2. Task 3 correctly sets rate-PID `iGate=0` before confirmed liftoff, so no new
   integral accumulates. It does not limit proportional/derivative output.
   Each pitch/roll rate controller can still request up to 120 us.
3. The normal mixer preserves pitch/roll by shifting collective upward when a
   low-side motor would fall below 1050 us. At low takeoff collective this can
   raise one motor far above the 40 us/s base ramp. With simultaneous saturated
   pitch and roll requests, the static path permits hundreds of microseconds of
   motor spread even though altitude throttle itself is ramping smoothly.
4. Normal `angleBoost()` is active from the TAKEOFF mode edge. Ground contact
   does not benefit from cosine-based vertical-thrust compensation, so this is
   another path by which a reported tilt changes collective before control
   authority exists.
5. Mixer anti-windup reports clipping for the following control tick and has
   no sign. That is useful airborne, but cannot prevent the first ground motor
   transient; `iGate=0` is the effective ground anti-windup mechanism.
6. Resetting each cascaded setpoint LPF initializes it to zero. Merely assigning
   a captured nonzero surface target after reset would still introduce a
   filtered target transient unless the LPF is preset to the capture value.

### 2.2 Attitude-reference limitation

IMU calibration computes `pitchOffset`/`rollOffset` from gravity at boot and
subtracts them from every reported angle. It cannot distinguish fixed sensor
mounting bias from a surface tilt present during calibration. Consequently:

- boot/calibration on a tilted surface can make a physically tilted craft read
  approximately zero;
- no controller-only change can reconstruct true earth-level airframe attitude
  from those conflated values;
- with Task 8 calibration-origin position hold, the supported procedure must
  calibrate stationary on a known level fixture and then tilt that fixture at
  the same horizontal location without rebooting; translating the craft is
  recorded as position error against the calibration origin and therefore
  produces a return demand after TAKEOFF;
- the exact residual mount/alignment error remains **UNKNOWN** until measured.

Changing this estimator reference without a surveyed mounting offset would be
an unsafe Task 4 expansion. This task instead captures the reported attitude at
TAKEOFF and limits how it is driven toward the existing calibrated trim target.

### 2.3 Evidence classification

- **Confirmed:** all six static paths above and the estimator reference
  ambiguity are directly present in current source.
- **Strongly supported:** the normal low-throttle mixer can create a hazardous
  corner-motor lead on a constrained tilted frame; this follows numerically
  from its min-motor collective shift and output limits.
- **UNKNOWN:** the installed surface angle, thrust needed to unload each skid,
  acceptable motor delta, true contact-to-liftoff timing, vibration-triggered
  rate peaks, and behavior of the newly built Task 3 firmware on hardware.

## 3. Selected sequence

The public modes and Task 3 transition-phase byte remain unchanged.

```text
TAKEOFF command
  -> validate reported pitch/roll/rates
  -> capture pitch/roll/yaw and preset setpoint filters
  -> SPOOL_UP: hold captured attitude, I=0, ground-limited mixer, no angle boost
  -> GROUND_TRANSITION: slew target toward calibrated trim, I=0,
     ground-limited mixer, angle/rate abort monitoring
  -> confirmed liftoff: retain current target/controller state
  -> ATTITUDE_TRANSFER: crossfade ground-limited and normal mixer outputs,
     keep I=0, continue target slew
  -> normal airborne mixer/controller; enable I only after crossfade
  -> position hold may arm only after target profile reaches trim
```

### Ground-output invariant

For each ground tick, calculate the existing X-frame pitch/roll/yaw correction,
scale the combined correction vector so its largest absolute component is no
more than `TAKEOFF_GROUND_MOTOR_DELTA_US`, and clamp every motor to:

```text
[MOTOR_MIN_US, min(normal mixer ceiling,
                   requested takeoff collective + ground motor delta)]
```

Unlike the normal mixer, this path never raises its base to preserve a
low-side correction. It deliberately sacrifices attitude authority while the
airframe is physically constrained. Global motor limits remain unchanged.

### Abort and failsafe behavior

- Reject TAKEOFF if reported pitch or roll exceeds the mild-surface admission
  bound. Existing rate/link/sensor checks still apply.
- During pre-liftoff phases, require angle/rate violation to persist for a short
  confirmation time, then use the Task 3 pre-lift failsafe motor ramp. Do not
  attempt aggressive leveling on a surface outside the designed envelope.
- Existing 4 s GROUND_TRANSITION no-lift and 15 s total takeoff timeouts remain
  the time bounds; no duplicate state timer is introduced.
- Link/sensor/LAND pre-lift aborts retain the Task 3 monotonic motor ramp.
- KILL, invalid attitude, and MPU failure retain the existing TRIPPED path.

## 4. Parameter ledger

Every numeric value is an initial conservative engineering bound. Hardware
flight evidence is unavailable, so observations are mandatory before accepting
or widening any limit.

| Parameter/behavior | Old | New | Unit | Subsystem/reason/evidence | Expected positive effect | Possible negative effect | Required real-flight observation |
|---|---:|---:|---|---|---|---|---|
| TAKEOFF admission tilt | `ARM_MAX_ANGLE=25` effective | `TAKEOFF_GROUND_MAX_TILT_DEG=8` | deg/axis | 25 deg is not “slight”; exact safe surface angle UNKNOWN | Rejects large ground attitude demand | May reject a usable surface or mounting/calibration error | Level-calibrated restrained tests at measured 0, 3, 5, then <=8 deg |
| Ground abort tilt | none beyond 45 deg error trip | `TAKEOFF_GROUND_ABORT_ANGLE_DEG=12` | deg/axis | Bounds divergence/contact rotation before lift; optimum UNKNOWN | Stops escalating corner load | False abort under vibration/estimator transient | Log continuous angle and abort duration |
| Ground abort rate | takeoff entry <=15; no ongoing ground bound | `TAKEOFF_GROUND_ABORT_RATE_DPS=45` | deg/s/axis | Detect skid release/tip motion without using a single sample; optimum UNKNOWN | Aborts rapid tip/rotation | Could be too high for a light frame or too low under vibration | Record filtered rates on restrained spool |
| Abort confirmation | absent | `TAKEOFF_GROUND_ABORT_CONFIRM_S=0.10` | s | Reject isolated filtered spikes; 25 control ticks nominal, hardware evidence UNKNOWN | Avoids one-frame abort while remaining bounded | Permits 0.10 s of unsafe motion | Verify elapsed violation and Task 3 ramp entry |
| Ground target | trim immediately | captured attitude during SPOOL; slew to trim in GROUND_TRANSITION | deg | Removes mode-edge error while retaining existing level target | Bumpless command edge | Holds initial tilt longer and relies on level calibration | Log capture, target, actual attitude |
| Setpoint filter initial state | reset to zero | preset to captured pitch/roll | deg | Confirmed LPF transient if capture is nonzero | Capture is truly bumpless | Preserves a bad capture until validation/abort | Compare first filtered target/error sample |
| `TAKEOFF_LEVEL_SLEW_DEG_PER_S` | only 20 Hz LPF after immediate target | 3.0 | deg/s | Explicit bounded leveling; optimum UNKNOWN | Gradual corner unloading instead of a step | May not reach trim before liftoff | Measure target slope and lift attitude |
| `TAKEOFF_GROUND_MOTOR_DELTA_US` | no ground-specific bound; normal mixer can use full band | 50 | us above requested collective | Static mixer analysis; installed thrust effect UNKNOWN | Prevents one motor from outrunning base ramp aggressively | May provide insufficient leveling torque | Plot base and each motor during restrained tilted spool |
| Ground angle boost | globally enabled | unchanged globally; bypassed before liftoff | boolean/phase | Cosine boost assumes free flight, not ground constraint | Removes tilt-dependent collective increase on ground | Slightly less vertical component near release | Compare requested base and achieved lift point |
| `TAKEOFF_ATTITUDE_TRANSFER_S` | immediate normal mixer/I authority at lift confirmation | 0.50 | s | Crossfade bounds handover; optimum UNKNOWN | Avoids motor jump at liftoff boundary | Temporarily reduces disturbance rejection | Measure per-tick motor deltas and attitude peak after lift |
| Ground/transfer rate I | ground 0, airborne 1 immediately | 0 through transfer, then existing 1 | ratio | Existing integrals start reset; prevent windup during limited authority | No hidden I demand during saturation/crossfade | Delays integral disturbance rejection by 0.5 s | Log `iPitch/iRoll/iYaw` through handover |

No existing gain, filter cutoff, estimator weight, hover/takeoff throttle, motor
minimum/maximum, normal mixer coefficient, position parameter, or landing
parameter is changed.

## 5. Verification and rollback

### Static/model checks

1. TAKEOFF capture occurs after controller reset and presets both setpoint LPFs.
2. SPOOL_UP holds the capture; GROUND_TRANSITION alone begins the 3 deg/s target
   slew; liftoff does not reset target/controller state.
3. Ground output never exceeds requested collective +50 us, never leaves global
   bounds, and does not call `angleBoost()`.
4. Pitch/roll/yaw integrals remain zero/gated through ground and crossfade.
5. The first crossfade tick is ground output plus a bounded fraction of the
   normal-output difference; alpha is monotonic from zero to one.
6. Every pre-lift abort routes to Task 3 MOTOR_RAMP_DOWN.
7. No schema, task, lock, sensor, altitude, landing, or normal airborne mixer
   behavior changes.

### Props-off and restrained validation

1. Calibrate level. With props removed, tilt the fixture in place to measured
   angles (do not silently relocate the calibration-origin position) and verify
   admission/rejection, captured targets, 3 deg/s target motion, zero I, and
   exact per-motor bound.
2. Inject short and sustained angle/rate violations; verify only sustained
   violations abort and all motor commands then decrease through the Task 3
   ramp.
3. Restrained with propellers, repeat 0 deg before 3/5 deg. Confirm the high
   corner motor never exceeds base+50 us before lift and no skid unloads
   violently.
4. Only then perform a netted low-altitude takeoff. Verify liftoff confirmation,
   0.5 s crossfade, target continuity, motor continuity, and attitude recovery.

Expected successful log signature: capture equals measured reported attitude;
setpoint is continuous; ground I terms remain zero; no ground motor exceeds
base+50 us; crossfade alpha progresses monotonically; no motor step occurs at
liftoff; pitch/roll converge toward trim after becoming controllable.

Rollback Task 4 if calibration on a known level surface does not reproduce the
same reference, ground target jumps, any motor exceeds the bound, a motor jumps
at crossfade, integral grows while authority is limited, abort fails to enter
the monotonic Task 3 ramp, attitude diverges after lift, or control timing
regresses. Compilation/model checks are not evidence of safe flight.

## 6. Implementation and verification status

The bounded design is implemented in `drone_alt_hold_1m.ino` with config
signature `0xA41F0401`. No telemetry layout, RTOS task, lock, sensor path,
airborne controller gain, normal mixer formula, altitude/position behavior,
motor bound, or landing parameter changed.

Static/model checks on 2026-09-09 passed:

- capture follows controller reset and presets both cascaded setpoint LPFs;
- SPOOL_UP holds capture; GROUND_TRANSITION/airborne profile moves toward trim
  at no more than 3 deg/s; position-hold suppression cannot overwrite it;
- all 500 combinations of throttle 1050/1100/1350/1430 us and pitch/roll/yaw
  controller requests -120/-60/0/+60/+120 us produced ground motor commands
  inside `[1050, base+50] us`;
- crossfade models spanning ground/normal endpoints 1050--1780 us reached exact
  endpoints over 125 nominal 250 Hz ticks, with a maximum 6 us step for fixed
  endpoints;
- both sketches retain the same 81-field, 248-byte telemetry-v4 layout;
- the control path remains lock-free, the altitude snapshot retains one writer,
  and `git diff --check` passes.

Warning-enabled fresh Arduino CLI builds with FQBN `esp32:esp32:esp32` passed:

- flight controller: 979267 bytes flash (74%), 49588 bytes global RAM (15%);
- sender: 889324 bytes flash (67%), 45784 bytes global RAM (13%).

No compiler warning was emitted. No props-off, restrained, or flight test was
performed. The level-reference assumption, installed tilt/rate vibration,
50 us motor authority, 3 deg/s slew, abort thresholds, 0.50 s transfer, and
post-liftoff attitude response all remain **UNKNOWN** on hardware.
