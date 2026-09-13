# Flight Issues and Root-Cause Analysis

## 1. Scope and evidence rules

This document is the TASK 1 evidence baseline for
`testZone/drone_alt_hold_1m`. It correlates the complete 286-line
`FLIGHT_LOGS_ANALYSIS.md` report (63 telemetry rows across five flights) with
the current flight-controller and sender sources. TASK 1 changes no flight
behavior and applies no tuning value.

The log is not a post-change acceptance test of the current source. It spans
several intermediate configurations:

- Logged altitude correction reaches `-100/+131 us`; current code limits it to
  `-45/+70 us`.
- Logged position-hold tilt reaches `7 deg`; current code limits it to
  `3.5 deg`.
- The report states a 200 Hz control loop; current code schedules 250 Hz.
- Flight 1 uses `H=1400` and a motor ceiling of 1610 us; later flights use
  `H=1430` and reach 1780 us.

There is no firmware hash, build identifier, or parameter snapshot for each
flight. Consequently this report distinguishes:

- **Confirmed:** directly demonstrated by a log row, current source, or both.
- **Strongly suspected:** evidence supports the mechanism, but the available
  telemetry cannot isolate it from other causes.
- **Hypothesis / unresolved:** plausible, but not supported sufficiently to
  justify a flight-control change.

“Historical” means confirmed for the logged firmware revision. “Current
status unverified” means a mitigation exists in today's source but has not
passed a new flight test.

## 2. Observed envelope

| Flight | Main observation | Quantitative evidence |
|---|---|---|
| 1 | Weak lift and failsafe close to the floor | `z_max=0.301 m`, `corr=+80 us`, M1 repeatedly `1610 us` |
| 2 | Rise, right drift, flow loss, then fall | `z_max=0.609 m`, `v_z=-0.483..+0.469 m/s`, right position `+0.81 m`, flow quality falls to 52 |
| 3 | Overshoot, impact with high throttle, rebound | `z=1.031 m` at `+0.595 m/s`; later `z=0.094 m`, `v_z=-0.790 m/s`, `T=1580 us`; rebound to `1.142 m` |
| 4 | Slingshot and hard fall | setpoint `0.769 m` at height `0.174 m`; then `z=1.216 m` at `+0.893 m/s`; peak `1.388 m`; impact at `-0.967 m/s` |
| 5 | Repeating vertical and lateral oscillation; landing flip | approximately `0.078..0.990 m`, `v_z=-0.790..+0.633 m/s`, flow speed up to `1.04 m/s`, roll reaches `-156.7 deg` after impact |

These flights confirm an unsafe historical envelope. They do not prove that
the current lower gains, slew limits, and landing clamps have solved it.

## 3. Root-cause summary

| ID | Concern | Cause class | Finding |
|---|---|---|---|
| F-01 | Evidence provenance and observability | Instrumentation/process | Confirmed limitation |
| F-02 | Altitude yo-yo and overshoot | Algorithm + parameter | Historical mechanism confirmed; current outcome unverified |
| F-03 | Takeoff ground transition | Algorithm/timing | Current timer regression confirmed; unsafe effect strongly suspected |
| F-04 | Delayed vertical response | Estimator/filter | Strongly suspected |
| F-05 | Weak MTF range confidence | Sensor + estimator | Weak returns and binary trust confirmed; physical cause unresolved |
| F-06 | Position drift and pendulum motion | Algorithm + parameter + sensor | Historical behavior confirmed; current mitigation unverified |
| F-07 | Landing surge/bounce/flip | State machine + controller | Historical failure confirmed; current residual risks strongly suspected |
| F-08 | Throttle excursions and motor spread | Mixer/motor + attitude | Command saturation confirmed; physical imbalance unresolved |
| F-09 | Estimator drift/source transition | Estimator | Not demonstrated by available telemetry |
| F-10 | PID saturation, windup, derivative noise | Controller | Saturation confirmed; windup/noise unresolved |
| F-11 | Incorrect `dt`, loop timing, dual-core interference | Timing/architecture | Not demonstrated; current measurement gap confirmed |
| F-12 | Hover/feed-forward choice | Parameter/plant | 1430 us is only a rough estimate, not calibrated |

## 4. Detailed issues

### F-01 — Logs cannot identify the exact firmware or controller components

**Evidence**

- The mutually incompatible limits listed in section 1 prove that the five
  flights do not all use the current parameter set.
- `TelemetryPacket` contains `loopHz`, `outP/outR/outY`, attitude integrals,
  separate range/barometer altitude, and flags
  (`drone_alt_hold_1m.ino:412-443`), but `printStatus()` omits most of them
  (`sender_1m/sender_1m.ino:175-200`).
- Altitude integral, vertical-velocity setpoint, raw/filtered MTF data,
  barometer innovation, effective fusion weights, achieved collective, and
  mixer saturation axes are not preserved in the supplied log.

**Likely root cause**

The logging contract was designed for operator status, not system
identification. Missing build provenance and internal controller/estimator
terms make several proposed causes observationally equivalent.

**Confidence**

**Confirmed.** This is a direct source and artifact limitation.

**Relevant code**

- Flight telemetry declaration and population:
  `drone_alt_hold_1m.ino:412-443`, `:2127-2182`.
- Sender status formatter: `sender_1m/sender_1m.ino:175-200`.

**Relevant parameter**

`TELEM_HZ=10`; packet version 3; no log schema/build-ID field.

**Recommended change**

Before controller tuning, add a machine-readable diagnostic mode with firmware
hash/config CRC and timestamped controller, estimator, freshness, saturation,
loop-period, and state-transition fields. Keep normal radio telemetry compact;
store or stream diagnostics at a rate sufficient for the altitude bandwidth.

**How to verify**

Run a static schema test and a propeller-off replay. Every row must identify
the binary/configuration and allow reconstruction of altitude error -> velocity
setpoint -> PI terms -> requested throttle -> achieved collective. Reject a
flight dataset as tuning evidence if rows are dropped or provenance is absent.

### F-02 — Historical altitude yo-yo and overshoot

**Evidence**

- Flight 4 advances the setpoint to `0.769 m` while the measured height has
  fallen to `0.174 m`; 0.995 s later the vehicle is at `1.216 m` and still
  climbing at `+0.893 m/s`, then peaks at `1.388 m`.
- Flight 5 cycles roughly `0.08 <-> 0.99 m` with vertical speed up to
  `+0.633/-0.721 m/s` before LAND.
- The historical report records the prior pull-down expression
  `fminf(ALT_TARGET_M, fminf(z + 0.15, nextSp))`. It makes the command follow a
  fall and then rebuild, supplying a repeatable forcing term.
- Current code uses a monotonic 0.25 m/s setpoint ramp
  (`drone_alt_hold_1m.ino:2812`) and no longer contains that expression.

**Likely root cause**

For the logged revision, the altitude setpoint pull-down behavior, excessive
command authority, and phase-delayed vertical feedback combined into a
limit-cycle/slingshot. The log supports a multi-cause failure; it does not
support attributing the full oscillation to one PID gain.

**Confidence**

**Confirmed historical algorithm fault and observed oscillation. Strongly
suspected multi-loop interaction. Current outcome unverified.**

**Relevant code**

- Current takeoff trajectory: `drone_alt_hold_1m.ino:2777-2828`.
- Current cascaded altitude loop: `:2855-2881`.
- Historical expression: `FLIGHT_LOGS_ANALYSIS.md:198-207`.

**Relevant parameter**

Historical log: corrections as large as `-100/+131 us`. Current:
`ALT_SETPOINT_SLEW_MPS=0.25`, `ALT_POS_KP=0.70`, velocity limits
`+0.35/-0.20 m/s`, `ALT_VEL_KP=80`, `ALT_VEL_KI=30`, correction limits
`+70/-45 us` (`drone_alt_hold_1m.ino:135-163`).

**Recommended change**

Retain the monotonic trajectory concept, but do not declare the issue fixed
until estimator latency and achieved actuator authority are measured. In the
implementation task, gate takeoff trajectory progression on a valid ground to
air transition and tune the vertical loop from logged step/impulse response,
one parameter family at a time. Do not widen correction limits to compensate
for an uncalibrated hover value.

**How to verify**

First replay synthetic height/fall sequences through the trajectory and assert
that the setpoint never decreases during TAKEOFF. Then perform low-height,
netted tests with position hold disabled for diagnosis. Acceptance requires no
repeated sign-changing vertical-speed cycle, no correction saturation for more
than a short transient, and bounded peak height/impact speed defined before the
test. Abort on the first cycle or mixer saturation.

### F-03 — Current takeoff timeout is inert and lift does not gate trajectory

**Evidence**

- `takeoffElapsedS` is declared at `drone_alt_hold_1m.ino:589`, reset at
  `:2042`, and checked at `:2816-2819`, but is never incremented in the current
  file.
- The tracked diff shows that `takeoffElapsedS += dt` existed previously and
  was removed while the takeoff trajectory was edited.
- Before lift, the open-loop ramp freezes the setpoint only while
  `takeoffThrottleF < HOVER_THROTTLE` (`:2783-2797`). At hover throttle it exits
  that branch and advances toward the altitude target even if `liftDetected`
  is false.
- `takeoffLiftConfirmed` changes a reason string but does not gate that
  trajectory.

**Likely root cause**

A state-machine timebase regression disabled both the 4 s no-rise and 15 s
takeoff timeout. Separately, the ramp-complete condition is being used as a
proxy for airborne state. On a tilted/stuck/under-thrust airframe this can
accumulate a large altitude error before lift, creating a delayed surge.

**Confidence**

**Confirmed source defect. Strongly suspected safety effect.** The exact
in-flight outcome still requires safe validation.

**Relevant code**

`drone_alt_hold_1m.ino:589`, `:2038-2044`, `:2777-2820`.

**Relevant parameter**

`TAKEOFF_START_US=1350`, `TAKEOFF_RAMP_US_PER_S=40`,
`TAKEOFF_LIFTOFF_M=0.06`, `TAKEOFF_LIFTOFF_VZ_MPS=0.10`,
`TAKEOFF_NO_RISE_S=4`, `TAKEOFF_TIMEOUT_S=15`.

**Recommended change**

Restore a monotonic elapsed-time update from the measured control `dt`, reset
it only on the documented state transition, and separate “ramp reached hover”
from “airborne confirmed.” Add explicit abort behavior for no lift and tilted
surface conditions before permitting the altitude trajectory.

**How to verify**

Use propeller-off state-machine tests with injected altitude streams: no rise,
brief false rise, valid lift, and sensor loss. Assert exact timeout transition,
no altitude setpoint growth before confirmed lift, no controller integral
growth while grounded, and deterministic reset after disarm. Only then use a
restrained low-thrust test; do not validate this first in free flight.

### F-04 — Vertical estimator/filter delay can amplify overshoot

**Evidence**

- Current range path stacks median-5, a 0.10 s EMA, 15-sample slope regression,
  a 0.20 s velocity filter, and a 0.18 s fused-velocity filter
  (`drone_alt_hold_1m.ino:179-190`, `:1245-1261`, `:1401-1493`).
- The altitude controller reacts to that fused vertical speed, not raw range.
- Flights 3–5 reverse between approximately `+0.6` and `-0.8 m/s` while the
  controller correction reverses near its limits, consistent with delayed
  braking, but the log does not contain raw-versus-filtered timing.
- Range/barometer ages are commonly 0–22 ms during the displayed oscillations,
  so gross packet staleness is not the same as total estimator phase delay.

**Likely root cause**

The sum of smoothing and derivative-estimation delays may leave the velocity
loop reacting after vertical momentum has already changed. Actual phase delay
depends on the unknown MTF publish rate and task jitter.

**Confidence**

**Strongly suspected, not confirmed.** The existing log cannot measure group
delay.

**Relevant code**

Range filtering/velocity regression: `drone_alt_hold_1m.ino:1217-1280`.
Fusion prediction/update: `:1401-1493`.

**Relevant parameter**

`RANGE_FILTER_TAU_S=0.10`, `RANGE_VELOCITY_SAMPLES=15`,
`RANGE_VELOCITY_TAU_S=0.20`, `FUSION_HEIGHT_TAU_MTF_S=0.12`,
`FUSION_HEIGHT_TAU_BARO_S=0.45`, `FUSION_VELOCITY_TAU_S=0.18`.

**Recommended change**

Measure end-to-end step response before changing a time constant. The future
estimator should assign each measurement its real sample timestamp and avoid
serially filtering the same information more than necessary. Any faster filter
candidate must be evaluated for both lag reduction and noise amplification.

**How to verify**

Move the unpowered vehicle through a measured vertical profile and log raw
range, filtered range, range velocity, barometer, fused state, sample time, and
receive time. Cross-correlate with an independent reference. Record delay,
noise RMS, and outlier recovery; repeat over height and surface types. A change
passes only if delay improves without violating noise/outlier limits.

### F-05 — Weak MTF returns receive nearly full altitude trust

**Evidence**

- At high points of Flight 4, strength is only 11 and 15 while source remains
  `FUSED`; at Flight 3's 1.031 m point strength is 27.
- Current `acceptRange()` rejects only `strength < 10`
  (`drone_alt_hold_1m.ino:1265`). Every accepted range then receives the same
  90% fused-height and 70% fused-velocity share when sensors agree
  (`:1440-1446`).
- Current code does not turn strength, range, incidence angle, innovation
  history, or recent reject count into a continuous range variance/weight.

**Likely root cause**

This is a confidence-model problem: a binary validity threshold is being used
where measurement quality changes continuously. It can allow a marginal return
to dominate BMP388. The physical reason for low strength is still unknown.

**Confidence**

**Confirmed algorithmic weakness and confirmed weak logged returns.** Whether
those accepted weak samples contain enough error to cause the vertical cycle is
**unresolved** because raw sensor disagreement is not logged.

**Relevant code**

MTF acceptance: `drone_alt_hold_1m.ino:1264-1280`. Fixed-weight fusion:
`:1430-1460`.

**Relevant parameter**

Strength threshold `10`; `FUSION_BARO_HEIGHT_WEIGHT=0.10`,
`FUSION_BARO_VEL_WEIGHT=0.30`, innovation gate `0.45 m`; distance validity
`30..12000 mm`; the current controlled mission target is 0.50 m clearance.

**Recommended change**

In the fusion-design task, derive confidence from strength, status, operating
range, tilt, innovation consistency, and reject history; degrade weight before
hard rejection. Do not simply raise the strength threshold: that could cause
frequent MTF dropouts and shift control to a drifting barometer. Inspect the
sensor window, mounting, floor reflectivity, and configuration independently.

**How to verify**

Collect stationary and motion datasets at 0.1–1.6 m over all intended floor
surfaces and tilts. Compare range to a reference while sweeping strength.
Establish error/variance versus strength and a hysteretic reject/recovery rule.
Inject low-strength sequences in replay and confirm a bounded, continuous
fused output with explicit source/confidence telemetry.

### F-06 — Horizontal position hold loses or overdrives authority

**Evidence**

- Flight 2 accumulates `+0.81 m` right displacement before flow changes from
  OK to unavailable as quality falls through 66, 60, 56, and 52.
- Flight 5 reaches approximately `+/-1.04 m/s` flow speed and commands up to
  7 degrees in the logged revision, forming a lateral pendulum.
- Current gains/limits are lower (`KP_vel=3`, max tilt `3.5 deg`, slew
  `8 deg/s`) and the minimum quality is lower (`40`), but no current flight
  validates that combination.
- The comment says full confidence at quality 110; the expression
  `0.40 + (quality - 40) / 50` reaches 1.0 at quality 70
  (`drone_alt_hold_1m.ino:2755-2759`).

**Likely root cause**

The historical failure combines aggressive position authority with abrupt
loss of flow. The current code reduces authority but admits lower-quality flow
and grants full authority earlier than documented. This is both a parameter
selection and confidence-mapping problem. Axis signs are not yet bench-proven.

**Confidence**

**Confirmed historical drift/oscillation and confirmed current comment/code
mismatch. Current closed-loop stability is unverified.**

**Relevant code**

Flow gates/filters: `drone_alt_hold_1m.ino:198-224`, `:1282-1335`.
Position controller: `:2669-2768`.

**Relevant parameter**

`FLOW_MIN_QUALITY=40`, `FLOW_FILTER_TAU_S=0.12`,
`POS_HOLD_FLOW_CONFIRM_S=0.20`, position gain `POS_HOLD_KP_PER_S=0.40`, vector
`POS_HOLD_MAX_VEL_MPS=0.35`, velocity PI
`3.0/0.3`, integral limit `1.5 deg`, max tilt `3.5 deg`, tilt slew `8 deg/s`.
Task 9 retains the post-calibration origin without a horizontal deadband.

**Recommended change**

Correct or redesign the quality-to-confidence contract from measured flow
error rather than its comment. Verify axis mapping first. Task 8 adds
per-sample SI height normalization and retains the calibration origin through
lost-flow states with bumpless correction decay; these changes remain
unvalidated. Tune velocity damping before enabling position integration; keep
position hold disabled during the first vertical-loop validation.

**How to verify**

With props removed, push the frame forward/right and rotate yaw; verify
reported body axes and corrective pitch/roll signs. Replay good/degraded/lost
flow without discontinuities. Then use a tethered or netted low-height test,
enabling velocity damping before position integration. Abort on growing
oscillation, saturation, sign mismatch, or flow confidence discontinuity.

### F-07 — Landing can retain lift/lateral correction at ground contact

**Evidence**

- Flight 3 enters FAILSAFE at `z=0.094 m`, falling at `-0.790 m/s`, but retains
  `T=1580 us`; it subsequently rebounds to `1.142 m`.
- In Flight 5, LAND is requested while the vehicle is already moving rapidly.
  The setpoint descends at about the intended 0.12 m/s, but at `z=0.088 m` the
  vehicle is still falling at `-0.790 m/s`, roll is `-54.4 deg`, and one motor
  is `1780 us`; the next rows show a flip/trip.
- Current `startLanding()` clamps its starting throttle and immediately
  disarms if any altitude source reports `<=0.15 m`
  (`drone_alt_hold_1m.ino:1891-1908`). This mitigation is unvalidated and does
  not require a fresh range measurement.
- Current position hold runs for every airborne state, including LANDING and
  FAILSAFE (`:2924-2939`), and retains the last correction for 0.35 s after
  flow becomes unusable (`:2669-2698`).

**Likely root cause**

Historical landing/failsafe transitions preserved excessive collective and
lateral attitude authority during/after impact. Current throttle slew/cap logic
addresses the surge statically, but ground detection can falsely trust
BARO/COAST/fused altitude and horizontal correction can continue into the
touchdown zone.

**Confidence**

**Confirmed historical bounce and flip. Strongly suspected current residual
state-machine risk. Current landing performance unverified.**

**Relevant code**

Landing entry: `drone_alt_hold_1m.ino:1891-1908`; landing setpoint/controller:
`:2829-2921`; position hold: `:2669-2768`, `:2924-2939`.

**Relevant parameter**

Descent rates `0.12/0.05 m/s`; near-ground threshold `0.40 m`; landing
throttle rise/fall `55/70 us/s`, near-ground fall `50 us/s`, headroom `50 us`,
absolute cap `1470 us`; landed gate `0.10 m`, `|v_z|<=0.25 m/s` for `0.40 s`.

**Recommended change**

Use a landing sub-state machine: stabilize/decelerate, descend, near-ground,
touchdown-confirm, and disarm. Require fresh credible range plus persistence
for touchdown; do not use BARO/COAST alone for immediate ground-impact disarm.
Fade/disable position integration and limit attitude demand near the floor.
Define a hard response for excessive tilt or impact distinct from normal
closed-loop landing.

**How to verify**

Use deterministic replay for LAND during climb, fall, lateral motion, weak
range, and barometer-only conditions. On a thrust stand or restrained rig,
verify that LAND never steps throttle upward, caps apply, and stale lateral
commands decay. Progress through low-height tests only after ground detection
has zero false positives/negatives in surface trials. Acceptance includes no
post-contact throttle recovery and no motor at a high limit near touchdown.

### F-08 — Mixer saturation and motor spread consume altitude/attitude authority

**Evidence**

- Flight 1 repeatedly pins M1 at its then ceiling of 1610 us while another
  motor falls to 1098 us.
- Flights 3 and 5 repeatedly pin one motor at 1780 us while others are near
  1100 us; Flight 5 shows `1780/1115/1390/1376` near impact.
- Current mixer explicitly shifts collective to fit pitch/roll/yaw demand and
  records throttle/axis saturation (`drone_alt_hold_1m.ino:1714-1767`). Thus
  logged `T` is requested pre-mix throttle, not necessarily achieved mean
  collective.
- The log omits axis controller outputs and mixer saturation flags.

**Likely root cause**

Attitude correction and collective demand compete for a finite PWM range. At
the limits, altitude authority changes even if `T` appears moderate. Possible
motor/ESC/prop/thrust asymmetry may increase the required attitude correction,
but PWM commands alone cannot distinguish physical imbalance from attitude
error, frame geometry, calibration, or mixer sign.

**Confidence**

**Confirmed commanded saturation/large spread. Physical motor imbalance is
unresolved.**

**Relevant code**

Mixer/saturation: `drone_alt_hold_1m.ino:1703-1775`; altitude anti-windup uses
previous mixer flags at `:2867-2879`.

**Relevant parameter**

`MOTOR_MIN_US=1050`; airborne maximum is hover plus a 25% band, capped to
350 us (`H=1430` gives 1780 us); yaw headroom and pitch/roll scaling are
implemented in the mixer.

**Recommended change**

Instrument requested versus achieved collective and per-axis desaturation.
Calibrate each ESC/motor/prop on a guarded thrust stand and inspect frame CG,
prop direction, arm geometry, vibration, and idle consistency. Do not tune
individual motor offsets from these flight rows. Controller design must account
for lost collective authority during desaturation.

**How to verify**

Props-off: verify motor order, mixer sign, PWM min/max, and saturation flags.
Guarded thrust stand: compare thrust/current/RPM versus PWM for all four units.
Restrained attitude test: command small axis steps and verify expected motors.
Acceptance requires matched response within a predefined tolerance and correct
telemetry of every limit event before free flight.

### F-09 — Barometer drift or estimator source transition is not established

**Evidence**

- Most oscillation rows report `src=FUSED` with fresh age fields; the final
  post-impact rows switch to BMP after MTF ages to 306/807 ms.
- `src=FUSED` does not expose MTF altitude, aligned BMP altitude, innovation,
  offset, or dynamic confidence. Therefore it cannot show agreement accuracy.
- Current code slowly aligns BMP to range with an 8 s time constant and a
  0.45 m innovation gate (`drone_alt_hold_1m.ino:1408-1434`).

**Likely root cause**

No estimator-drift root cause can be selected from the evidence. Barometer
drift, range bias, slow alignment, innovation rejection, and source switching
remain distinguishable only with separate sensor traces.

**Confidence**

**Hypothesis / unresolved.** Gross stale data is not supported in the shown
pre-impact rows, but measurement bias and transition discontinuity are not
ruled out.

**Relevant code**

Barometer update and velocity: `drone_alt_hold_1m.ino:1340-1399`; alignment and
fusion: `:1401-1493`; source publication: `:1560-1583`.

**Relevant parameter**

BMP388 ODR 50 Hz, pressure oversampling 8x, IIR coefficient 7, attempted read
spacing 22 ms; `BARO_ALIGN_TAU_S=8`, `BARO_INNOVATION_GATE_M=0.45`, barometer
height fusion time constant `0.45 s`, coast window `600 ms`.

**Recommended change**

Do not retune alignment or weights from the fused trace. Log both calibrated
measurements, innovation, offset, acceptance/reject reason, confidence, and
source transition. The fusion redesign should make covariance/confidence and
bumpless fallback explicit.

**How to verify**

Run stationary thermal/drift tests, known vertical profiles, deliberate MTF
occlusion, and recovery. Quantify bias, drift rate, innovation distribution,
switching step, and coast error. Source transitions must be continuous within
a defined bound and must not create a throttle transient in controller replay.

### F-10 — Saturation is visible; integral windup and derivative noise are not

**Evidence**

- Altitude correction and motors repeatedly reach their logged limits, proving
  controller/actuator saturation in the historical flights.
- Current altitude PI has conditional integration, a `+/-40 us` I limit, and
  considers mixer throttle saturation plus previous throttle slew saturation
  (`drone_alt_hold_1m.ino:2867-2881`). Attitude loops also receive per-axis
  mixer limits.
- Neither altitude integral nor altitude P term is logged. Attitude P/I/D
  terms, raw gyro noise, and output spectra are also absent.
- Vertical control is PI; “derivative noise” can refer only to the estimated
  vertical velocity or attitude rate/D paths, which are not separated in the
  report.

**Likely root cause**

Saturation certainly contributes to nonlinear behavior. Classic integral
windup may have occurred in a historical revision, but it cannot be confirmed
from `corr` alone. Derivative/rate noise is likewise possible but unsupported.

**Confidence**

**Confirmed saturation. Integral windup and derivative noise are hypotheses,
not findings.**

**Relevant code**

Altitude PI/anti-windup: `drone_alt_hold_1m.ino:2855-2881`; attitude PID and
filtered gyro path are mapped in `docs/FLIGHT_CONTROL_ARCHITECTURE.md` sections
5 and 6; mixer feedback: `:1703-1767`, `:2935-2956`.

**Relevant parameter**

Altitude `KP=80 us/(m/s)`, `KI=30 us/m`, I limit `40 us`; correction limits
`+70/-45 us`. Attitude PID coefficients and derivative filters remain separate
from these altitude parameters.

**Recommended change**

Log each loop's error, P/I/D terms, unclamped output, saturation source, and
applied output. Evaluate anti-windup with actuator/slew saturation injection.
Only alter I or D after the trace identifies accumulation or high-frequency
noise; reducing I blindly can worsen hover bias, while extra filtering can add
phase lag.

**How to verify**

Use host/replay tests that hold each actuator limit in both error directions.
The I term must stop growing into the limit and unwind promptly. Measure gyro,
rate term, vertical velocity, and motor-output spectra during stationary and
vibration tests. Set quantitative bounds before any gain change.

### F-11 — Incorrect `dt` and core interference are unmeasured, not proven

**Evidence**

- Current MPU schedule is nominally 1 kHz with control every four samples
  (250 Hz). Control `dt` is measured from `micros()` and constrained to
  0.5–50 ms; missed samples are skipped rather than replayed.
- The report's “200 Hz” label conflicts with current code, confirming version
  drift rather than incorrect current `dt`.
- Sender output omits `loopHz`; no per-tick `dt`, deadline miss, task runtime,
  UART backlog, or I2C mutex wait appears in the logs.
- Current OLED is enabled and uses the same core and AUX I2C mutex as BMP388.
  Its own comment notes a display transaction around 29 ms. These recorded
  flights likely predate that enablement, so OLED cannot be assigned as their
  cause.

**Likely root cause**

There is no evidence that incorrect `dt` caused these flights. Current
dual-core contention and blocking display/I2C activity are architectural jitter
risks that need direct measurement, especially because altitude acquisition,
UART draining, BMP reads, telemetry, and OLED share core 0.

**Confidence**

**Incorrect `dt`: unresolved/not demonstrated. Missing timing observability:
confirmed. OLED/core interference: hypothesis for current firmware.**

**Relevant code**

Task creation and core assignments: `drone_alt_hold_1m.ino:2420-2516`;
altitude/OLED mutex calls `:985-990`, `:1373-1382`, `:2333-2380`; main timing
loop `:2970-3010`.

**Relevant parameter**

MPU `1000 Hz`, control `250 Hz`, ESC PWM `200 Hz`, telemetry `10 Hz`, OLED
`4 Hz`, BMP attempted about `45.5 Hz`. MTF actual publish rate, successful BMP
rate, task jitter/runtime, CPU clock, and Arduino loop-task affinity remain
runtime UNKNOWNs.

**Recommended change**

Instrument actual period/runtime/max latency for each task and sensor, UART
queue/backlog, I2C mutex wait, missed MPU deadlines, and `dt` clamp events.
Move or suspend nonessential display work during flight if measurement shows
deadline interference. Do not change loop frequencies without CPU and sensor
rate data.

**How to verify**

Run timing tests with OLED off/on and telemetry off/on, first props removed and
then on a restrained vibration rig. Report distributions and worst cases, not
averages alone. Acceptance requires no control `dt` clamp events, bounded sensor
age/backlog, and timing margins defined against each deadline.

### F-12 — Hover throttle is an estimate, not a calibrated plant parameter

**Evidence**

- User observation says the frame can begin flying around 1400 us and flies
  high at 1500 us.
- Flight 5 includes near-stationary samples around `T=1434..1438 us`, but these
  coexist with large motor spread and attitude demand.
- Flight 1 at `H=1400` fails to climb normally, yet its old mixer ceiling and
  attitude redistribution confound the result.
- Current source uses `HOVER_DEFAULT=1430`; its nearby comment still says
  “take 1400,” and README values also drift from code.

**Likely root cause**

1430 us is a plausible starting feed-forward, but not an identified hover
value across battery voltage, payload, motor variation, and ground effect.
Incorrect feed-forward consumes PI and saturation margin; it does not alone
explain the historical cycles.

**Confidence**

**Strongly supported as an uncalibrated parameter; exact correct value
unresolved.**

**Relevant code**

`drone_alt_hold_1m.ino:109-120`; runtime `HOVER` command handling and persistence
paths documented in `docs/FLIGHT_CONTROL_ARCHITECTURE.md` section 10.

**Relevant parameter**

`HOVER_DEFAULT=1430`, allowed range `1050..1750`, runtime slew `100 us/s`,
motor band 25% capped at 350 us.

**Recommended change**

Calibrate thrust/feed-forward after motor matching and estimator validation.
Use battery-voltage-aware or slowly learned hover compensation only with
strict bounds and correct state gating. Do not use hover adjustment to mask
mixer saturation or sensor delay. Synchronize comments/README only after the
validated value is selected.

**How to verify**

Measure total thrust versus PWM and battery voltage on a guarded stand, then
confirm with brief stable hover segments outside ground effect. A valid value
keeps steady-state altitude PI near zero without persistent mixer saturation.
Repeat across intended battery state and payload.

## 5. Parameter decisions from this analysis

No flight parameter is changed in TASK 1. The current lower correction limits,
position gains, tilt limit, setpoint slew, and landing throttle slew are
reasonable risk-reduction candidates, but they remain **statically present,
not flight-validated**.

The evidence specifically rejects these blind changes:

- Raising throttle/correction limits to cure weak lift before motor/hover and
  mixer authority are measured.
- Raising the MTF strength threshold without designing continuous confidence
  and BMP fallback.
- Lowering flow quality threshold without an error-versus-quality dataset.
- Reducing every filter time constant at once; that trades lag for noise and
  makes attribution impossible.
- Removing I or D terms solely because saturation or oscillation is visible.

## 6. Root-cause priority and dependency order

1. Restore trustworthy provenance and diagnostic telemetry (F-01).
2. Correct and host-test takeoff state/time invariants (F-03).
3. Characterize MTF/BMP measurements, confidence, rate, and latency (F-04,
   F-05, F-09, F-11).
4. Validate motors, mixer mapping, and achieved authority (F-08, F-12).
5. Validate vertical estimator and loop with position hold disabled (F-02,
   F-10).
6. Validate horizontal axes/confidence and tune damping before position
   integration (F-06).
7. Validate landing sub-states, ground detection, and near-ground authority
   (F-07).

This order prevents a downstream controller from being tuned around an
upstream sensor, timing, or actuator defect.

## 7. Safe verification progression and rollback conditions

The future implementation tasks should advance only through these gates:

1. Static/host replay: state transitions, timeouts, estimator source changes,
   saturation, anti-windup, and output bounds.
2. Props removed: sensor axes, flow signs, motor order, mixer direction,
   command loss, KILL, and all telemetry.
3. Guarded thrust stand/restraint: motor matching, PWM/thrust, vibration, CPU
   timing, and no unexpected throttle step.
4. Netted low-altitude vertical test with horizontal hold disabled.
5. Netted low-altitude velocity damping, then position integration.
6. LAND from stable hover, followed only later by LAND during bounded vertical
   and lateral disturbances.

Stop and roll back the latest behavioral change if any of these occurs:

- first growing altitude or lateral oscillation;
- correction/mixer saturation lasting beyond a declared transient;
- uncommanded upward throttle during LAND;
- range confidence/source discontinuity large enough to move throttle;
- sensor age, queue backlog, or control `dt` outside its declared bound;
- motor-order/sign mismatch, unexpected tilt, hard contact, or loss of KILL.

## 8. Remaining uncertainties

- Exact binary/configuration for each recorded flight.
- Actual MTF publish rate, latency, configuration, and error versus strength,
  range, surface, and tilt.
- Successful BMP388 rate, read latency, thermal drift, and vibration response.
- Raw-to-fused estimator group delay and source-transition step.
- Current loop/task jitter, runtime, core affinity, and OLED/I2C interference.
- Altitude/attitude PI/D internal histories in the recorded failures.
- Achieved collective and exact saturation axes at each logged row.
- Per-motor thrust/current/RPM matching, frame CG, and vibration.
- Verified flow axes and error versus quality/height/surface.
- Current takeoff, altitude hold, position hold, and landing behavior after the
  unvalidated source changes.

Until these uncertainties are measured, a successful compile or bench sensor
read is not evidence of safe free flight.
