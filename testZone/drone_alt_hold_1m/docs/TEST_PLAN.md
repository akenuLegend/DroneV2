# Repeatable verification and flight-tuning test plan

## 1. Purpose, boundary, and stop rule

This plan converts the remaining UNKNOWNs for configuration `0xA41F1001` into
repeatable evidence. It does not assert that any hardware test has been run.
Software completion (source audit, models, schema and build) is separate from
airframe validation. Free flight remains prohibited until every preceding gate
has passed on the same physical build and configuration.

The order is mandatory:

```text
static/replay -> props-off bench -> guarded motor/restraint
-> netted low-energy vertical -> horizontal damping/position
-> normal takeoff/landing -> tilted launch -> bounded fault validation
-> only then consider free flight
```

Stop immediately on the first failed acceptance item, unexpected motor motion,
sign/mapping error, growing oscillation, sustained saturation, sensor/timing
gap outside the predeclared rig limit, hard contact, smoke/heat, loss of command
or loss of the independent power-cut method. Do not “try once more” with a
higher throttle or wider safety limit. Save the evidence and use the rollback
procedure in section 13.

## 2. Roles, test environment, and safety prerequisites

Minimum roles for any powered-propeller test:

- test director/operator at the sender;
- safety observer with authority to abort;
- data recorder, if one person cannot operate and capture logs reliably.

Required controls:

- legal/private test site, safety enclosure or net appropriate to the stage,
  eye protection, fire-safe battery handling, and no uninvolved people;
- physical restraint rated for maximum possible thrust and configured so it
  cannot foul propellers or bias the sensor under test;
- an independent, directly accessible battery/power disconnect. The software
  KILL is not an independent hardware kill and must not be the only protection;
- known propeller type/orientation, motor order, ESC calibration, battery ID/
  state, all-up mass, MTF mounting offset, surface/lighting and measured launch
  angle recorded before the run;
- a level calibration fixture. Tilted-launch tests must calibrate on this level
  fixture, then tilt it at the same horizontal location without rebooting;
- terminal capture for both drone and sender. The firmware has no persistent
  onboard log, so a run without captured output is not tuning evidence.

Do not test deliberate KILL, MPU loss, uncontrolled source loss, or an unknown
controller response in unrestrained flight. Battery, RPM/current, stack margin,
raw innovations and independent position/attitude require external instruments
because telemetry v5 does not contain them.

## 3. Run identity and required record

Create one run directory or worksheet per attempt. Record before power-up:

| Field | Required content |
|---|---|
| Run ID | Unique date/time and sequential run number |
| Firmware | Source revision/diff identity and startup build date/time |
| Configuration | Drone startup must print `cfg=0xA41F1001`; sender `STATUS` prints the same value as `cfg=A41F1001`. Any parameter edit requires a new signature |
| Telemetry | Version 5, 248 bytes, both sketches built/loaded as a pair |
| Hardware | Airframe, motor/ESC/prop IDs, wiring, MTF/BMP/MPU/OLED IDs, sensor offset |
| Energy | Battery ID, chemistry/cell count, measured start/end voltage; current if instrumented |
| Geometry | Mass, CG check, launch angle, restraint/enclosure, floor material/texture |
| Environment | Temperature, lighting, airflow, barometric conditions |
| Test recipe | Stage/test ID, exact input sequence, duration, acceptance and abort bounds selected before starting |
| Result | PASS/FAIL/INVALID; operator and observer names; anomaly and rollback action |

Capture the full startup stream, every event line, and continuous sender output.
The baseline sender text is necessary but not sufficient for all later tuning
criteria: `STATUS` prints only a subset of the 81 fields carried in telemetry
v4. It prints config/mode/phase/reason, pitch/roll, motors/throttle/hover,
altitude-control and flow/position/timing diagnostics, but omits packet time/
sequence, yaw/rates, `outP/outR/outY`, attitude I terms, biases, vibration/clip,
`hoverTarget`, separate range/barometer heights, last command and radio
rx/bad/lost counters. Before a stage relies on an omitted field, use a
versioned packet decoder or a diagnostic build with a new config signature and
repeat Stage A. Do not infer an omitted value from another printed value.

Across the default sender capture plus that traceable supplemental capture,
preserve at minimum:

- `cfg`, timestamp/sequence, mode, transition phase and reason;
- `loopHz`, current/max control `dt`, cumulative overruns;
- pitch, roll, yaw and rates; `outP/outR/outY`; attitude I terms;
- M1--M4, throttle, hover and hover target;
- fused/range/barometer altitude, setpoint, Vz, altitude velocity setpoint,
  correction and altitude integral;
- range/barometer/flow ages, source, strength, flow quality/fresh flag;
- flow forward/right velocity, integrated position and requested position tilt;
- MTF/BMP rates and maxima, UART backlog, altitude-task gap, CRC/stale/read/
  mutex errors and radio receive/bad/lost counts.

Also capture an independent synchronized reference where the acceptance depends
on true angle, height, velocity, PWM, thrust, RPM/current or battery voltage.
Mark a run INVALID if provenance, time alignment or required fields are absent.

## 4. Stage A — static, model, schema and build gate

Run after any source or parameter change, before connecting a flight battery.

1. Diff only the intended files; record every parameter old/new/unit/reason,
   positive/negative effect, observation and rollback criterion.
2. Verify a unique `FLIGHT_CONFIG_SIGNATURE`; update expected run identity.
3. Run static ownership checks: one altitude-snapshot writer, bounded seqlock
   readers, atomic command mailbox, no semaphore/queue/I2C/UART/display wait in
   `controlTick()` or `loop()`.
4. Compare the ordered drone/sender telemetry structs: 81 typed fields, static
   size 248 bytes and <=250-byte assertion.
5. Run deterministic models for command/KILL interleavings, sender callback
   ordering, coherent snapshot fallback, non-finite/staleness behavior,
   takeoff timers, ground mixer bounds, target slew/authority transfer,
   landing entry/re-entry, touchdown and final motor monotonicity.
6. Run documentation constant/link checks, verify the capture plan distinguishes
   packet fields from the sender's printed subset, and run `git diff --check`.
7. Perform fresh `--clean --warnings all` Arduino CLI builds of both sketches
   using FQBN `esp32:esp32:esp32`; archive console results and artifact time.

Acceptance: every check passes, no warning, packet schemas match, generated
artifacts are not mistaken for source, and no unrelated worktree change is
overwritten. The Task 6 reference proof is drone 980567 bytes flash/49588 RAM
and sender 889368/45792, but a different result is not automatically failure;
explain every change. A documentation-only Task 7 does not require rebuilding
unchanged source if source identity and the exact Task 6 build proof remain
verified.

## 5. Stage B — props-off startup, command and sensor bench

### B1. Electrical/startup

1. Remove all propellers and mechanically identify M1 FL/CW, M2 FR/CCW,
   M3 BR/CW and M4 BL/CCW.
2. Power sender, start terminal capture, then power drone on a level fixture.
3. Confirm ESC pins produce 1000 us before task allocation/startup can stop.
4. Confirm MPU identity and exact DLPF/gyro/accel/divider readback, successful
   stationary calibration, BMP address/config, MTF Micolink traffic and task
   core/priority startup reports.
5. Confirm configuration/telemetry identity before issuing any action.

Acceptance: no motor command above idle before ARM, exact MPU readback, coherent
near-zero level/ground estimates, valid link and no unexplained startup error.
Deliberate MPU readback corruption/disconnect is accepted only on a disposable
or instrumented bench setup; it must stop initialization with ESC at 1000 us.

### B2. Attitude, range and flow signs

1. At rest record at least 60 s of timing, sensor rate/age, vibration and drift.
2. Tilt exactly one physical axis through surveyed small angles. Confirm pitch/
   roll sign, magnitude and corresponding gyro-rate sign. Rotate yaw slowly and
   confirm yaw-rate sign and expected gyro-only drift after stopping.
3. Move the unpowered frame through independently measured vertical positions
   and smooth profiles. Record true clearance, raw sensor distance externally
   if available, range/barometer/fused altitude and Vz.
4. On every intended floor/lighting condition, translate the frame forward then
   right without rotation. Confirm `flowForwardMps` and `flowRightMps` are
   positive on their matching movements. Reverse each movement and verify sign.
5. Repeat stationary/translation trials over height, texture, light and mild
   tilt; record strength, quality, rejection/recovery and apparent drift.

Acceptance: no cross-axis/sign ambiguity, clearance matches measured 0.12 m
geometry within a predeclared measurement tolerance, stationary outputs have no
unexplained trend, and timing/rate/error distributions are recorded—not merely
described as “looks stable.” A wrong sign blocks all motor/flight stages.

### B3. Props-off state/command matrix

Execute each from a fresh known state and record exact transition/time:

| Input/fault | Expected result |
|---|---|
| Boot without valid sender | Remain DISARMED; no auto-arm |
| ARM with missing/stale MTF or BMP, motion, tilt, vibration/clip | Reject with matching reason |
| Valid ARM held through checks | DISARMED -> ARMED_IDLE after 2 s |
| No TAKEOFF | Auto-DISARM after 15 s ARMED_IDLE timeout |
| DISARM in ARMED_IDLE | DISARMED; all outputs 1000 us |
| TAKEOFF command edge | Starts from actual armed-idle output; no 1050->1350 step |
| No-rise injection | Abort 4 s after GROUND_TRANSITION; monotonic motor ramp |
| One brief liftoff predicate | Must not transfer; continuous 0.15 s may transfer |
| Repeated LAND while LANDING | Phase/timer do not restart |
| LAND while FAILSAFE | Does not downgrade FAILSAFE or change its phase/timer |
| Safety promotion during LANDING | Becomes FAILSAFE while preserving landing progress |
| KILL | TRIPPED/reset required; commanded outputs become 1000 us |

## 6. Stage C — props-off fault injection and communications

Test one fault at a time; restore and reboot where the state is latched.

| Fault | Injection | Expected software behavior / evidence |
|---|---|---|
| Invalid command | Wrong peer, size, magic, version, CRC, command | No action; bad/drop counter where applicable; invalid data never refreshes link |
| Packet burst/queue full | Controlled packet generator | Callback never blocks; loss counted; repeated valid action is handled once |
| Sender callback delay | Instrumented/replay build | No overlapping accepted sends; warning after 100 ms; drone watchdog remains fallback |
| Sender/link loss | Turn sender off | Link becomes lost after >3 s; ARMED_IDLE disarms; TAKEOFF/HOLD model routes to FAILSAFE landing |
| MTF checksum/stale/time reset | UART replay | Bad/stale counts change; no invalid update; valid reboot/recovery is bounded |
| MTF disconnect | Remove sensor data | Range age/source progress through fallback; TAKEOFF/HOLD failsafe after >800 ms |
| BMP disconnect/read failure | Remove/interrupt AUX safely | BMP failure/age changes; MTF-only remains coherent; pre-arm rejects BMP stale |
| Both altitude sources | Replay/disconnect in non-prop or guarded simulation | Coast lasts <=600 ms, then NONE; landing uses documented blind path, never stale closed-loop data |
| Flow loss/recovery | Remove texture/quality or replay | Hold decays/slews to trim, calibration-origin position freezes rather than clearing, recovery confirms for 0.20 s and resumes toward the same origin without a tilt step |
| Altitude publisher stall | Instrumented model | Last coherent snapshot retained, invalid after >100 ms publication age |
| MPU repeated read failure | Safe bus fault, props off | After 50 scheduled failures enters TRIPPED; do not claim exactly 50 ms without timing evidence |
| NaN/Inf injection | Host/model or instrumented bench | Invalid attitude trips; invalid altitude becomes NONE/fallback; no non-finite motor command |

Acceptance: observed state/counters match the table, recovery never silently
returns from FAILSAFE/TRIPPED to flight, and no control-path blocking appears.
These tests validate software decisions, not physical safety of a sensor-loss
landing.

## 7. Stage D — guarded motors and restrained authority

Proceed only after Stages A--C pass. Start without propellers for PWM/order,
then use propellers only inside a rated enclosure/restraint.

### D1. Motor order, direction and mapping

- Exercise the ESC calibration utility separately if needed; never confuse it
  with active flight firmware.
- Verify each output pin drives the labeled motor and each prop/motor direction
  matches the X-frame definition.
- With small restrained attitude disturbances, verify the corrective motor
  pairs oppose the disturbance. Abort immediately on any positive feedback.
- Record PWM, thrust/current/RPM if available and compare four motor curves,
  startup consistency, dead zone and saturation.

Acceptance: correct order/direction/sign, repeatable synchronized minimum, no
unexpected output, and motor matching within a tolerance defined before test.
Do not hide mechanical/ESC mismatch with trim or PID changes.

### D2. Timing and interference matrix

For at least 60 s each, run motors off/on and OLED enabled/disabled, telemetry
active, MTF present/absent and normal BMP reads. Record distributions and
worst cases for control period/overruns, MTF/BMP rates/gaps/read time, AUX wait,
altitude-task gap and UART backlog. Separately measure CPU frequency, loop-task
affinity/priority and all task stack high-water marks with an instrumented
diagnostic build.

Acceptance limits must be declared from bench distributions and control/airframe
response evidence. The current project intentionally has no invented control-
deadline trip threshold. A rising overrun count, stale altitude publication or
unbounded backlog blocks powered flight even if average loopHz looks correct.

### D3. Ground takeoff profile and authority

- From ARMED_IDLE verify the collective slope is 40 us/s through the 1350 us
  phase boundary with no output step.
- With P/R/yaw feedback demand held near zero, verify the CG compensation is
  exactly zero at 1050 us, rises continuously with collective, and reaches a
  `+15/-15 us` front/rear pitch-axis split at 1430 us. M1/M2 must be the raised
  pair for the reported forward battery offset, mean requested PWM must remain
  unchanged, and every output must remain inside the ground mixer bound.
- At measured 0° first, then only in later tilted stages, verify no pre-lift
  motor exceeds `[1050, requested base+50] us`, angle boost is absent, P/R/yaw
  integrals stay zero and a no-rise abort decreases every motor.
- Inject/reproduce the liftoff predicate on the rig if it can be done without
  actual release; verify the 0.50 s transfer is continuous and integrators stay
  gated until completion.

Acceptance: all Task 3/4 invariants and the Task 10 CG feed-forward ramp match
the telemetry/PWM trace and there is no skid/corner unload, motor step or
growing attitude response. A reversed corrective pair, worsening nose-down
motion, or rear-motor unload is an immediate rollback—not a reason to increase
the value. Default sender
text cannot establish the I-term criterion; the supplemental capture defined
in section 3 is mandatory for this test.

## 8. Stage E — netted low-energy vertical characterization

Use a restraint that allows limited vertical motion without permitting escape
or hard impact. The current firmware automatically enables horizontal hold
when eligible; the first vertical identification should use a traceable test
configuration that suppresses horizontal authority without corrupting MTF
range. Such a source change requires a distinct config signature and Stage A
rebuild. Do not pretend position hold is disabled merely because drift appears
small.

### E1. Hover feed-forward

1. Begin at the validated current baseline; do not jump directly to 1500 us.
2. Run brief stable segments outside strong ground effect. If all upstream
   gates pass, adjust hover only while DISARMED in <=5 us steps.
3. Reject segments with attitude/mixer saturation, sensor dropout, battery sag
   or restraint contact; they cannot identify hover feed-forward.

Acceptance: a repeatable hover region has near-zero mean vertical speed and a
small, non-trending altitude integral/correction without persistent motor
saturation. Repeat across intended battery state and payload before changing
the compiled default.

### E2. Estimator delay and vertical controller

Use independently measured small vertical profiles/disturbances. First identify
raw/separate sensor delay/noise and source transitions; telemetry v5 is not
sufficient for every internal stage, so add a separate diagnostic stream if
needed. Tune in dependency order:

1. sensor validity/confidence and end-to-end estimator delay;
2. hover feed-forward;
3. vertical-velocity P/damping;
4. altitude position P;
5. velocity I for residual steady bias;
6. correction and slew limits only if achieved authority proves them necessary.

Change one parameter/term per signed build using handbook steps. Acceptance is
predeclared bounded tracking without a second growing sign-changing Vz cycle,
without unexplained source step, persistent PI/mixer saturation or contact.
Abort at the first growing oscillation or facility height/velocity bound.

## 9. Stage F — takeoff, altitude capture and normal landing

Run in a netted/restrained area at measured 0° before any tilted launch.

### F1. Takeoff/capture

- Confirm SPOOL_UP -> GROUND_TRANSITION -> CONTROLLED_ASCENT ->
  ALTITUDE_CAPTURE -> ALT_HOLD in order.
- Liftoff must require the continuous predicate; altitude setpoint and PI must
  not accumulate on the floor.
- After lift, setpoint rises no faster than 0.12 m/s, vertical velocity demand
  remains within +0.20/-0.12 m/s, and transfer lasts 0.50 s;
  capture requires current tolerance/Vz for 0.6 s.
- Record actual lift PWM, angle, motor spread, max altitude/Vz, source quality,
  correction/I and timing. One successful run is insufficient; repeat across
  the declared battery range.

Acceptance: no command/motor discontinuity, no growing vertical/lateral cycle,
no sustained saturation or source loss, and all facility abort bounds remain
unreached. Firmware ceiling 1.60 m is a failsafe, not a performance acceptance
target.

### F2. Landing/touchdown

Start with stable hover and LAND. Later repeat only bounded disturbances that
the rig can safely contain.

- Verify controlled descent 0.08 m/s setpoint slew above 0.40 m and 0.04 m/s
  near ground, with throttle constrained by `min(hover+50,1470)`.
- Verify position hold is suppressed during landing.
- Fresh MTF clearance <=0.10 m latches TOUCHDOWN_DETECTION; no bounce returns
  to a higher-authority phase.
- After latch, every motor is monotonic toward 1050; stable confirmation or
  2 s timeout enters the 1 s ramp to 1000 and DISARMED.
- Repeat LAND commands and an instrumented safety promotion; phase/timer must
  not reset and FAILSAFE must never downgrade.

Acceptance: no upward throttle step at LAND entry, no post-contact motor
increase/rebound, no lateral correction into contact, correct final disarm and
no landing timeout. Abort on hard contact, excessive tilt or any motor rise
after touchdown latch.

## 10. Stage G — horizontal velocity damping and position hold

Only begin after the vertical estimator/controller and 0° takeoff/landing are
repeatable. Complete props-off sign/quality tests on every intended surface.

The current code couples velocity PI and position integration. For disciplined
tuning, create traceable diagnostic candidates that enable velocity damping
before outer position integration; each candidate needs a new signature and
Stage A. Do not tune several gains, confidence and filter constants at once.

1. Verify flow quality/age remains eligible throughout a stationary hover.
2. Apply small known forward/right disturbances separately. Tune velocity P
   for damping, then minimal I for persistent bias.
3. Enable/tune `POS_HOLD_KP_PER_S` from low authority, keeping the tilt and
   velocity limits unchanged. The target is zero position error at the
   calibration origin; do not add a horizontal deadband merely to pass a test.
4. Test good -> degraded -> lost -> recovered flow. Correction must decay
   bumplessly, the calibration-origin estimate must freeze (not clear), and
   recovery must confirm for 0.20 s before resuming toward the same reference.
5. At measured sensor heights spanning the future commanded-height envelope,
   repeat the same physical translation speed/distance. Confirm per-sample
   height normalization produces consistent m/s and radial position; record
   range, raw flow in a diagnostic build, filtered flow, quality and reference
   motion. Repeat across texture, lighting and yaw motion; quantify gyro-yaw
   drift coupling rather than extending duration blindly.

Acceptance: correct corrective sign, decaying rather than growing motion,
bounded 3.5°/8°/s commands, no altitude/mixer saturation coupling, no jump on
flow loss/recovery, and the smallest repeatable independently measured
horizontal error achievable without oscillation or saturation after a
predeclared settling interval. Record the measured error and uncertainty;
telemetry `r` is diagnostic and cannot by itself prove physical position. Abort
on pendulum growth, cross-axis response, quality collapse or sustained limits.

## 11. Stage H — mildly tilted-surface launch

Prerequisites: repeatable Stage F at 0°, proven motor/attitude signs and a level
calibration fixture. Calibrate level, tilt the fixture in place without reboot,
measure surface angle, and test sequentially 0°, 3°, then 5°. Do not jump to
the 8° admission limit;
8° is a software bound, not demonstrated safe capability.

For every angle verify:

- captured reported P/R equals the pre-command attitude and the setpoint is
  bumpless; SPOOL holds capture and target moves toward trim at <=3°/s;
- pre-lift I terms remain zero, no motor exceeds base+50 us and angle boost is
  absent;
- sustained >12° or >45°/s for 0.10 s aborts through monotonic motor ramp;
- confirmed lift begins the 0.50 s per-motor transfer without a step, then
  attitude converges without divergence or saturation.

The captured ground target/setpoint is not a telemetry-v4 field, and attitude
I terms are not printed by the current sender. Therefore the first two criteria
require the traceable diagnostic capture defined in section 3; default sender
text alone makes this stage INVALID, not PASS.

Acceptance: all invariants pass, no skid unload/tip, no false abort, no growing
attitude error and no control-timing regression. A failed angle blocks that and
all steeper angles; do not widen admission/abort/motor bounds to force success.

## 12. Stage I — bounded in-air fault validation

Physical fault behavior is tested only after normal low-energy flight is
repeatable and only inside a rig capable of containing the worst response.
Begin with replay and props-off results from Stage C. Advance one fault at a
time, at the minimum safe height/energy selected by the test director.

- link loss: after >3 s, controlled FAILSAFE landing without auto-resume;
- MTF loss: after >800 ms, controlled landing using BMP if credible;
- BMP loss: MTF-only continuity without an altitude/throttle step;
- flow loss: altitude continues while horizontal correction decays;
- both altitude sources: physical outcome of coast/NONE/blind descent remains
  high risk and must use restraint; do not test in free flight;
- KILL/MPU/invalid attitude: test only props-off or on a rig designed for the
  immediate/latching response, never as a normal airborne acceptance test.

Acceptance: transition, reason, source and motor behavior match the documented
state machine, no silent recovery to flight, and physical response remains
within the rig's predeclared bounds. Software-correct state transitions do not
prove that the resulting descent is safe on an unrestricted aircraft.

## 13. Abort, rollback and retest procedure

On any abort/failure:

1. Use the safest available action for the current state; use KILL only for an
   immediate hazard because it removes thrust rather than landing normally.
2. Isolate power, make the area safe, and preserve both complete logs plus the
   exact binary/config identity. Mark incomplete evidence INVALID, not PASS.
3. Identify the first violated acceptance item and classify it as confirmed,
   suspected or evidence-gap. Do not change a downstream gain to mask an
   upstream sensor, timing, mapping, motor or battery fault.
4. Revert only the latest candidate parameter/behavior to its recorded prior
   value, or produce one smaller evidence-backed candidate. Preserve all Tasks
   3--6 safety fixes and increment the configuration signature for a new binary.
5. Repeat Stage A and every affected earlier stage. A later-stage pass never
   waives an earlier failed gate.

Immediate rollback triggers include first growing altitude/lateral oscillation,
motor/order/sign mismatch, unexpected throttle rise during LAND, any motor rise
after touchdown latch, takeoff/transfer step, persistent correction/mixer
saturation, source-recovery throttle step, timing/backlog deterioration, false
ground/liftoff detection, hard contact, excessive tilt or unavailable KILL/
independent power cut.

## 14. Completion evidence matrix

| Capability | Software status | Hardware/field evidence required before claiming validated |
|---|---|---|
| Architecture, ownership, schema | Tasks 0--6 static/model/build PASS | Runtime affinity/priority, stack, WCET/jitter and interference matrix |
| IMU/attitude | Finite/readback defenses present | Axis/sign, bias/thermal/vibration, reference-angle and restrained response |
| MTF/BMP/fusion | Gates/fallback/instrumentation present | Rate/latency/noise/strength/innovation/source-recovery datasets |
| Mixer/motors | Equations/bounds/models PASS | Order/direction, four thrust curves, CG, RPM/current and achieved authority |
| Takeoff/landing | State/timer/ramp models PASS | Repeated 0° netted tests, contact behavior and then staged tilt tests |
| Position hold | Bounded code path present | Axis/confidence/surface tests, velocity damping then position integration |
| Failsafes | Static/fault models PASS | Only safe restrained physical cases; no claim for untested free-flight faults |
| Overall project | Documentation/software pipeline may be COMPLETE | Aircraft flight tuning/validation remains pending until all applicable rows pass |

The test record, not task status or compilation, determines whether an
individual airframe is ready for the next physical stage.
