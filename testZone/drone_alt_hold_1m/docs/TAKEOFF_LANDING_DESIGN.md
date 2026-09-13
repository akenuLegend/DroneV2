# Takeoff and Landing Redesign

Analysis date: 2026-09-09

Pipeline task: TASK 3 — TAKEOFF AND LANDING REDESIGN

Status: implemented; static and compile validation passed, hardware validation pending

## 1. Scope and evidence boundary

This task changes only the vertical takeoff/landing transition logic in
`drone_alt_hold_1m.ino` and the matching operator documentation/telemetry
interpretation. It does not tune attitude gains, position gains, the mixer,
motor limits, sensor filters, fusion weights, or the tilted-surface behavior
reserved for TASK 4.

The supplied flight log spans several historical firmware revisions. It is
evidence of hazardous outcomes, but it is not proof that every current clamp
or slew value has been flight-tested. All new numeric values below are initial
safety bounds requiring props-off, restrained, and low-altitude validation.

## 2. Current-path audit

### 2.1 Confirmed bugs

1. `takeoffElapsedS` is reset and compared with the 4 s no-rise and 15 s
   timeout limits, but is never incremented. Both aborts therefore have a dead
   timebase.
2. ARMED_IDLE finishes at `MOTOR_MIN_US=1050 us`, but TAKEOFF immediately sets
   `takeoffThrottleF` and `altitudeThrottleSlewF` to
   `TAKEOFF_START_US=1350 us`. The next motor command can step by 300 us rather
   than following the configured ramp.
3. When the ramp reaches hover without detecting lift, the current condition
   exits the open-loop ground phase. It then advances the mission altitude
   setpoint and enables controller integration while the aircraft may still be
   constrained by the floor. This recreates a stored-demand/slingshot path.
4. `startLanding()` initializes the throttle slew state with
   `min(throttleCmd, HOVER_THROTTLE)`. Entering landing above hover can therefore
   create an immediate downward throttle discontinuity despite the later slew
   limiter.

### 2.2 Strongly supported safety problems

- Historical data records `z=0.094 m`, `vz=-0.790 m/s`, and throttle `1580 us`
  at failsafe entry, followed by a rebound. The exact binary differs from the
  current source, but the current landing loop can still command upward
  correction while crossing `ALT_LANDED_M` too quickly to satisfy its
  low-vertical-speed confirmation.
- Historical LAND data reaches `z=0.088 m`, `vz=-0.790 m/s`, and large attitude
  correction before impact. Continuing horizontal position correction and
  unconstrained attitude mixing during floor contact is a landing hazard.
- A single sample currently satisfies liftoff. A filtered range/velocity spike
  can therefore enable ascent control prematurely. The occurrence rate on the
  installed airframe is UNKNOWN.

### 2.3 Unknowns

- The exact collective PWM at liftoff for current battery, propellers, mass,
  ESC calibration, and floor effect is UNKNOWN. Observations only indicate
  approximately 1400 us begins lift and 1500 us climbs strongly.
- The best liftoff-confirmation duration, touchdown collective decay, and
  final motor-ramp duration are UNKNOWN until hardware tests.
- Current estimator delay and the true vertical speed at floor contact remain
  UNKNOWN for the newly built telemetry-v4 firmware.

## 3. Selected state machine

The public `FlightMode` remains compatible. A bounded internal transition
phase makes the required states explicit and observable.

```text
DISARMED
  -> ARMED_IDLE
  -> TAKEOFF / SPOOL_UP
  -> TAKEOFF / GROUND_TRANSITION
  -> TAKEOFF / CONTROLLED_ASCENT
  -> TAKEOFF / ALTITUDE_CAPTURE
  -> ALT_HOLD

ALT_HOLD or airborne TAKEOFF
  -> LANDING or FAILSAFE / CONTROLLED_DESCENT
  -> LANDING or FAILSAFE / GROUND_APPROACH
  -> LANDING or FAILSAFE / TOUCHDOWN_DETECTION
  -> LANDING or FAILSAFE / MOTOR_RAMP_DOWN
  -> DISARMED
```

### 3.1 Takeoff invariants

- TAKEOFF starts from the actual current collective, nominally 1050 us, not
  from 1350 us.
- One existing 40 us/s slew applies continuously through SPOOL_UP and
  GROUND_TRANSITION. `TAKEOFF_START_US` becomes a phase boundary, not a command
  step.
- Until liftoff is confirmed, altitude setpoint follows measured height,
  altitude PI and attitude integrators remain reset/gated, and reaching hover
  does not enable closed-loop climb.
- Liftoff requires the existing height/velocity predicate continuously for a
  short confirmation interval. A loss of the predicate resets confirmation.
- The 4 s no-rise timer starts at GROUND_TRANSITION; the 15 s timeout covers
  the complete takeoff. Both advance from control `dt`.
- A pre-liftoff LAND/failsafe/no-rise abort ramps each current motor output down
  instead of jumping to idle or invoking airborne descent control.
- After confirmed liftoff, the existing 0.25 m/s altitude trajectory and
  acceleration-limited velocity target perform CONTROLLED_ASCENT. Entering the
  target tolerance changes to ALTITUDE_CAPTURE; 0.6 s of current hold criteria
  then enters ALT_HOLD.

### 3.2 Landing invariants

- Landing initializes the throttle slew from the actual current throttle; it
  does not clamp the stored slew state immediately to hover.
- The altitude integrator is reset at landing entry. The output slew provides
  bumpless transfer while preventing a stored positive integral from raising
  touchdown throttle.
- CONTROLLED_DESCENT and GROUND_APPROACH retain the existing 0.12/0.05 m/s
  altitude trajectories and current landing throttle caps.
- Landing suppresses new horizontal position-hold authority and slews the
  existing correction back toward trim using the existing tilt slew. This is
  limited to landing behavior; tilted takeoff remains TASK 4.
- Crossing the filtered fresh MTF landed threshold latches
  TOUCHDOWN_DETECTION. It cannot return to a higher-throttle phase on a bounce.
- TOUCHDOWN_DETECTION removes controller/mixer authority and ramps each actual
  motor output toward `MOTOR_MIN_US`; it never increases a motor. Stable
  height/vertical speed for the existing confirmation time, or a bounded
  touchdown timeout, advances to MOTOR_RAMP_DOWN.
- MOTOR_RAMP_DOWN interpolates each actual motor output to `IDLE_THROTTLE`, then
  disarms. KILL and severe attitude/IMU faults retain the existing TRIPPED path.
- Loss of both altitude sources before touchdown retains the bounded blind
  landing behavior and timeout.

## 4. Parameter ledger

Unchanged constants are listed because TASK 3 changes their role or relies on
them. New values are conservative initial engineering bounds, not flight-proven
tuning.

| Parameter | Old | New | Unit | Reason/evidence | Expected positive effect | Possible negative effect | Required observation |
|---|---:|---:|---|---|---|---|---|
| TAKEOFF start output | 1350 | current output (nominal 1050) | us | Confirmed 300 us state-transition step | Removes command discontinuity | Adds several seconds before ground transition | Props-off PWM trace must be monotonic at 40 us/s |
| `TAKEOFF_START_US` | 1350 | 1350 (unchanged; phase boundary) | us | 1300 does not prove airborne | Separates low spool from lift search | Boundary is not a measured thrust threshold | Record phase/PWM/liftoff point |
| `TAKEOFF_RAMP_US_PER_S` | 40 | 40 (unchanged; applies from current output) | us/s | Existing low slew; no evidence supports a faster spool | Avoids aggressive spool and jump | Slow takeoff; total timeout margin is limited | Measure command slope and time to lift |
| `TAKEOFF_LIFTOFF_CONFIRM_S` | absent/single sample | 0.15 | s | Reject a lone filtered height/Vz excursion; installed optimum UNKNOWN | Prevents premature closed-loop transfer | Excessive value could delay takeover after real lift | Log continuous predicate and transfer height |
| `TAKEOFF_NO_RISE_S` | 4.0, dead timer | 4.0, active in GROUND_TRANSITION | s | Existing intended abort restored without retuning | Stops indefinite high ground throttle | May abort a severely underpowered but otherwise healthy craft | Verify phase-local timeout and motor ramp-down |
| `TAKEOFF_TIMEOUT_S` | 15.0, dead timer | 15.0, active total timer | s | Existing intended overall bound restored | Bounds entire transition | Could be tight if actual lift is near configured hover | Log total transition time; do not increase without evidence |
| Landing slew initialization | `min(current, hover)` | current throttle | us | Confirmed entry discontinuity in code | Bumpless landing entry | Takes finite time to reach landing cap | PWM trace must have no entry step |
| `LAND_TOUCHDOWN_DOWN_US_PER_S` | absent | 250 | us/s | Historical high-throttle floor impact/bounce; exact optimum UNKNOWN | Rapidly removes post-contact energy without a command step | False touchdown at 0.10 m can reduce authority early | Restrained floor-approach test and rollback on premature drop |
| `LAND_TOUCHDOWN_TIMEOUT_S` | absent | 2.0 | s | Prevent indefinite MOTOR_MIN spin if Vz never settles | Guarantees progress after latched near-floor event | False touchdown becomes an eventual landing abort | Verify latch only occurs on fresh filtered MTF |
| `LAND_MOTOR_RAMP_S` | immediate disarm after confirmation | 1.0 | s | Requirement for MOTOR_RAMP_DOWN rather than a step | Predictable final shutdown, lower bounce energy | Motors remain powered briefly after contact | Props-off and restrained per-motor ramp trace |

No gain, estimator weight, source threshold, mixer coefficient, motor limit, or
hover value changes in this design.

## 5. Abort and failsafe matrix

| Condition | Phase/action |
|---|---|
| LAND during SPOOL_UP/GROUND_TRANSITION | LANDING/MOTOR_RAMP_DOWN |
| Link/MTF/no-rise failure before confirmed lift | FAILSAFE/MOTOR_RAMP_DOWN |
| Link/MTF/ceiling/flight timeout after lift | FAILSAFE/CONTROLLED_DESCENT or GROUND_APPROACH |
| TAKEOFF total timeout | FAILSAFE; motor ramp if not lifted, controlled landing otherwise |
| Fresh range crosses landed threshold | Latch TOUCHDOWN_DETECTION |
| Stable touchdown confirmation | MOTOR_RAMP_DOWN |
| Touchdown confirmation never settles for 2 s | MOTOR_RAMP_DOWN |
| Both altitude sources lost during descent | Existing blind throttle descent, then TRIPPED timeout |
| KILL, invalid attitude, sustained excessive angle, MPU failure | Existing TRIPPED path |

## 6. Verification and rollback

### Static/code

- Every takeoff phase is reachable in order and no ground phase advances the
  altitude target or altitude integral.
- `takeoffElapsedS` increments only in TAKEOFF and both abort comparisons are
  reachable.
- TAKEOFF initializes from `throttleCmd`; no assignment commands 1350 us at the
  mode edge.
- TOUCHDOWN_DETECTION and MOTOR_RAMP_DOWN never increase any motor.
- LAND entry preserves the current slew value; position hold is suppressed.
- No new lock, queue wait, allocation, I2C, UART, or serial loop is added to the
  control path.

### Bench, props removed

1. Capture telemetry phase and four motor commands from ARM through an aborted
   takeoff. Confirm 1050-to-1350 slope is 40 us/s with no step.
2. With range held at ground, confirm no-rise occurs 4 s after
   GROUND_TRANSITION and all four outputs ramp to 1000 us.
3. Inject a one-sample liftoff predicate and confirm it does not transfer;
   maintain it for 0.15 s and confirm CONTROLLED_ASCENT.
4. Trigger LAND at collective above hover and confirm no entry discontinuity.
5. Drive range below the landed threshold and confirm phase latching, monotonic
   per-motor decay, bounded touchdown timeout, final ramp, and disarm.

### Restrained/low-risk and low-altitude

- Restrained: verify motors remain monotonic in pre-lift abort and touchdown
  phases, with KILL still immediate.
- Low altitude: first test only vertical behavior in a netted area. Record
  phase, altitude/Vz/setpoint, integral/correction, collective, and all motors.
- Expected successful signature: no PWM step, lift transfer after confirmed
  motion, setpoint begins at measured height, capture Vz decays, landing never
  increases motors after touchdown latch, and final disarm follows the ramp.

Rollback the Task 3 implementation if any phase skips/reverses unexpectedly,
takeoff motor slope exceeds its bound, liftoff transfer occurs on a transient,
landing entry steps collective, touchdown latches above the threshold, a motor
increases after touchdown latch, KILL/failsafe timing regresses, or control-loop
timing worsens. Compilation alone is not flight validation.

## 7. Implementation and verification status

The state machine above is implemented in the flight sketch. Telemetry v4
repurposes its previous reserved byte as `transitionPhase`; packet version,
layout size, and transport remain compatible, and the sender prints status as
`[MODE/PHASE]`. The ground transition gates attitude/altitude integrators and
position hold, while landing suppresses position hold and uses the explicit
touchdown/final-ramp paths described above.

Static checks on 2026-09-09 passed:

- both sketches declare the same 81 telemetry fields and compute to 248 bytes;
- `takeoffElapsedS += dt`, the four takeoff phases, and four landing phases are
  present, and TAKEOFF initializes from the actual current output;
- no blocking semaphore/queue wait was added to the control path;
- a numeric simulation from motor starts 1000, 1050, 1350, 1430, 1580, and
  1780 us found no increase in TOUCHDOWN_DETECTION or MOTOR_RAMP_DOWN and a
  final endpoint of 1000 us;
- `git diff --check` passed.

Warning-enabled fresh Arduino CLI builds with FQBN `esp32:esp32:esp32` also
passed for both final sketches in `.tmp_task3_compile_20260909_02`: flight
controller 976771 bytes flash/49556 bytes global RAM and sender 889324 bytes
flash/45784 bytes global RAM. No compiler warning was emitted.

No props-off bench run, restrained run, low-altitude flight, or free-flight
test was performed. Therefore the four new numeric safety bounds in section 4,
installed liftoff timing, touchdown behavior, and control-loop timing effects
remain **UNKNOWN**. Build/static success is not evidence of flight safety.
