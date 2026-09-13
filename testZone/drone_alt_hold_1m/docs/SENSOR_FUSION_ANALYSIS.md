# MTF01P + BMP388 Sensor-Fusion and Dual-Core Analysis

Analysis date: 2026-09-09

Pipeline task: TASK 2 — MTF01P + BMP388 + DUAL-CORE ARCHITECTURE

Status: analysis and bounded implementation complete; runtime bench data pending

## 1. Scope and decision boundary

This document defines how MTF01P, BMP388, and the ESP32 execution contexts
should cooperate without disturbing the 1 kHz IMU schedule or 250 Hz flight
control path. It uses:

- the current `drone_alt_hold_1m.ino` and `sender_1m.ino` sources;
- `FLIGHT_CONTROL_ARCHITECTURE.md` and `FLIGHT_ISSUES.md`;
- the repository's standalone MTF01P and BMP388 diagnostic sketches;
- the [MicoAir MTF-01P specification](https://micoair.com/optical_range_sensor_mtf-01p/);
- the [Bosch BMP388 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf);
- the [Adafruit BMP3XX driver implementation](https://github.com/adafruit/Adafruit_BMP3XX/blob/master/Adafruit_BMP3XX.cpp).

Manufacturer capability is not the same as measured installed performance.
All rates, latencies, noise levels, and signal-quality relationships that have
not been measured on this exact airframe remain explicitly **UNKNOWN**.

No controller gain, sensor weight, confidence threshold, or filter time
constant is justified by this analysis alone. The implementation authorized by
this task is limited to deterministic scheduling boundaries, correct timestamp
semantics, and observability needed to obtain that evidence.

## 2. Measurement characteristics

### 2.1 MTF01P range and optical flow

| Property | Manufacturer/configuration | Installed-system knowledge |
|---|---|---|
| Interface | UART, 115200 baud, 3.3 V LVTTL | Configured on UART1 GPIO19/23 at 115200 |
| Protocol | Micolink, MAVLink, or MSP | Flight parser accepts Micolink message `0x51`, 20-byte payload only |
| Nominal output rate | 100 Hz | **UNKNOWN until measured**; current firmware neither configures nor reports it |
| ToF range | Up to 12 m under specified reflectance/light | Mission-effective range is not established; current control ceiling is 1.6 m |
| Dead zone | Manufacturer states 2 cm | Airframe clearance and 12 cm mounting offset dominate near-ground use |
| Ranging accuracy | 2 cm from 0.1–2 m at stated 90% reflectance; 2% above 2 m | **UNKNOWN** on installed floors, lighting, tilt, vibration, and mounting |
| Beam | 808 nm, 1.5 degree emitting angle | Floor material/geometry response **UNKNOWN** |
| Flow working distance | Above 8 cm | Current gate is 5 cm, which is below manufacturer guidance |
| Flow illumination | More than 60 lux | Not measured or gated in firmware |
| Flow FOV/max speed | 42 degrees; specified up to 7 m/s at 1 m | Current software rejects above 3 m/s; installed accuracy **UNKNOWN** |

The Micolink frame contains two distinct clocks:

- `sensorMs`: the sensor's timestamp, used to detect duplicate/stale/rebooted
  sensor data;
- `receivedUs`: the ESP32 time at which the parser drains the final byte, used
  for local freshness age.

`receivedUs` is a drain timestamp, not proof of the physical sampling instant.
If the UART buffer backs up, it overstates sample recency. Task 2 therefore
uses `sensorMs` deltas for range/flow filter `dt`, regression timestamps, and
MTF sample-gap diagnostics, while retaining `receivedUs` for conservative
host-side freshness. UART backlog and altitude-task gap are reported
separately.

#### MTF failure conditions

- wrong UART protocol or baud;
- invalid additive checksum or payload length/message ID;
- duplicated, regressing, or frozen sensor timestamp;
- status other than the verified normal code;
- range outside the operational gate;
- weak/unstable return due to target, light, tilt, contamination, or geometry;
- single-frame step/spike;
- UART backlog/overflow or task starvation;
- sensor reboot, disconnect, or power disturbance.

Range and flow validity must remain separate. Poor optical texture/quality must
not discard an otherwise credible range measurement.

### 2.2 BMP388 pressure altitude

| Property | Manufacturer/driver | Installed-system knowledge |
|---|---|---|
| Pressure range | 300–1250 hPa full-accuracy operating range | Current gate is 30000–110000 Pa; upper part of sensor range is not accepted |
| Relative accuracy | Typical +/-8 Pa (about +/-0.66 m under datasheet conditions) | Not sufficient by itself to resolve a 1 m mission without local zero/alignment |
| Current configuration | Pressure oversampling 8x, temperature no oversampling, IIR coefficient 7 | Confirmed in source |
| Conversion time | Datasheet high-resolution 8x/1x: typical 18.69 ms, max 21.53 ms | Actual installed `performReading()` duration **UNKNOWN** |
| Nominal ODR setting | 50 Hz written through the Adafruit API | Does not establish actual rate in this driver path |
| API power mode | `performReading()` selects forced mode on each call | Actual cadence is host-triggered and must respect conversion/read behavior |
| Read attempt cadence | At most once per 22 ms (about 45.5 attempts/s) | Successful/new-sample rate and transaction latency **UNKNOWN** |
| Long-term behavior | Sensitive to weather, temperature, prop wash and enclosure pressure | Installed drift/noise/thermal response **UNKNOWN** |

The Adafruit driver labels `performReading()` a blocking read, configures
forced mode, then reads data. Bosch specifies that forced mode performs one
measurement and returns to sleep, and that readout timing must respect maximum
conversion time. Consequently `BMP3_ODR_50_HZ` is not evidence of 50 new
samples/s in this program. Measure successful cadence and I2C call duration;
do not infer them from the register setting.

BMP altitude is relative to pressure zero captured on the ground. It is useful
for slow trend, continuity when range temporarily degrades, and detecting a
grossly inconsistent range. It is not a precise absolute 1 m reference.

#### BMP failure conditions

- missing ACK, wrong address, wiring/power failure;
- I2C timeout, mutex starvation, or library read failure;
- invalid/non-finite/out-of-gate pressure;
- repeated old sample or premature forced-mode read;
- temperature transient, enclosure heating, prop wash, room pressure change;
- slow bias drift after ground calibration;
- altitude discontinuity if offset/alignment is reset incorrectly.

### 2.3 MPU6050 interaction

The MPU6050 is not an independent altitude sensor in the current design.
Attitude supplies `cos(pitch)*cos(roll)` for ToF slant-range correction. Using
accelerometer Z for vertical propagation is deferred because attitude error,
gravity subtraction, vibration, and accelerometer bias would double-integrate
rapidly. It requires a characterized noise/bias model and replay data first.

The control core writes a single volatile tilt cosine; the sensor task reads it
without a lock. One aligned 32-bit float read is bounded, but it has no sample
timestamp. At the mission's current small tilt limit, this is acceptable as a
known approximation; a future full inertial estimator should transfer a
timestamped attitude snapshot instead.

## 3. Current estimator audit

### 3.1 Useful properties to retain

- Nonblocking byte-wise Micolink parser with checksum and sensor-time ordering.
- Range/flow gates are independent.
- MTF and BMP processing run off the control core.
- Altitude is transferred to the control loop through an odd/even sequence
  snapshot; the 250 Hz reader takes no lock and retries a bounded four times.
- All filtering uses fixed memory and bounded O(1) work, except tiny fixed-size
  median/regression loops.
- Source modes distinguish MTF, BMP, FUSED, COAST, and NONE.
- Fusion predicts continuously and avoids a direct source-output switch.

### 3.2 Baseline weaknesses and Task 2 disposition

1. Accepted MTF samples use binary trust. Strength 11 and strength 255 receive
   identical fusion weight when other gates pass.
2. The fixed blend has no reported confidence, variance, innovation, or reject
   history.
3. Range filtering stacks median-5, EMA, 15-sample derivative regression,
   derivative LPF, and fused-velocity LPF. End-to-end phase delay is UNKNOWN.
4. Baseline BMP and OLED shared a mutex with `portMAX_DELAY`; Task 2 replaces
   runtime BMP acquisition with a bounded 2 ms wait and OLED with zero-wait.
5. UART draining, BMP I2C, preprocessing, fusion, and snapshot publication
   remain one task. Task 2 drains MTF both before and after BMP work and
   suppresses OLED bus transfers in airborne states; splitting tasks remains
   evidence-gated.
6. Baseline BMP time came from the poll's pre-read `nowMs`; Task 2 records
   host-read completion and explicitly does not claim that as physical sample
   time.
7. Task 2 adds frame/sample rate, call duration, mutex wait/miss, UART backlog,
   altitude-task gap, parser error, controller `dt`, scheduler overrun,
   altitude velocity-setpoint and altitude-integral telemetry.
8. The same fresh sample is presented to the continuous fusion correction on
   multiple 1 ms service passes. Its time-constant form is deterministic, but
   the estimator does not make measurement-update versus prediction-update
   events explicit.

### 3.3 Unknowns that block weight/filter tuning

- MTF sample rate, transport delay, jitter, error versus strength, and reject
  rate on intended surfaces.
- BMP new-sample rate, forced-read behavior, transaction latency, noise and
  drift after installation.
- MTF/BMP innovation distribution in stationary and vertical motion tests.
- True raw-to-fused phase delay.
- Core/task worst-case execution time and I2C mutex wait.

## 4. Fusion-strategy evaluation

| Strategy | Benefits | Risks/requirements | Decision |
|---|---|---|---|
| Direct source switch | Simple | Height/throttle steps, no continuity, fragile around gates | Reject |
| Fixed weighted blend | Cheap and deterministic | Ignores changing quality; current weakness | Retain only as temporary behavior baseline |
| Complementary alpha-beta observer with confidence-scaled corrections | Bounded O(1), transparent, timestamped, bumpless, easy replay | Needs measured confidence mapping and delay characterization | **Selected target architecture** |
| Linear Kalman filter | Formal covariance and multi-rate updates | False precision without calibrated noise/process models; more states/tuning | Defer until datasets justify it |
| EKF including IMU acceleration/bias | Can bridge rapid dynamics | Gravity/tilt/bias/vibration errors; largest verification burden | Defer |

Sophistication is not the selection criterion. The complementary alpha-beta
observer is selected because all operations and limits are explicit and it can
be upgraded to a Kalman form later without changing task/data ownership.

## 5. Selected target estimator

### 5.1 State and clocks

State at estimator time `t`:

```text
x = [z, vz, b_baro]
```

- `z`: clearance of the lowest airframe point above the local surface.
- `vz`: upward vertical velocity.
- `b_baro`: slowly changing offset mapping pressure altitude into the local
  range reference.

Each measurement carries its source sample/order time and local receive/read
completion time. All unsigned time subtraction uses wrap-safe arithmetic.
Prediction runs at a bounded estimator cadence; corrections run only when a
new accepted measurement is observed.

```text
dt = clamp((t_now - t_prev), dt_min, dt_max)
z_pred  = z + vz * dt
vz_pred = vz * exp(-dt / tau_coast_drag) only when fully coasting;
          otherwise vz
```

The first implementation may retain constant-velocity prediction. No vertical
accelerometer term is added until its error budget is measured.

### 5.2 Measurement normalization

```text
z_range = distance_m * cos(pitch) * cos(roll) - ground_reference
z_baro  = pressure_altitude(P, P0) + b_baro
innovation_range = z_range - z_pred
innovation_baro  = z_baro  - z_pred
```

`ground_reference` must be defined unambiguously. The current code measures
`groundRangeM` but subtracts a fixed 0.12 m sensor-to-bottom distance. Those
quantities are not interchangeable: one is measured sensor-to-floor range on
the ground; the other is physical sensor height above the lowest airframe
point. Task 2 does not change this until a geometry/ground test resolves the
intended clearance zero.

### 5.3 Confidence contract

Confidence is a bounded diagnostic value `c in [0,1]`; hard-invalid data has
`c=0`. It should be composed from separately logged terms:

```text
c_range = c_status * c_age * c_strength * c_distance * c_tilt
          * c_innovation * c_recovery
c_baro  = c_valid * c_age * c_read * c_innovation * c_drift
```

Rules:

- status/checksum/timestamp/range hard failures force zero;
- age decays monotonically and reaches zero at stale timeout;
- strength-to-error and quality-to-error curves come from bench datasets, not
  guessed thresholds;
- innovation uses hysteresis and consecutive-sample recovery;
- source recovery ramps correction authority to avoid a step;
- confidence itself is logged with its component/reject reason.

No numeric strength-confidence curve is chosen in this task because the
installed error-versus-strength data does not exist.

### 5.4 Confidence-scaled correction

For each new accepted measurement, use bounded alpha-beta corrections:

```text
r = z_meas - z_pred
z  = z_pred  + clamp(alpha(c) * r, -dz_max, +dz_max)
vz = vz_pred + clamp(beta(c) * r / dt_meas, -dv_max, +dv_max)
```

BMP normally receives a smaller/faster-bounded height correction and is used
primarily for slow trend/continuity. Its alignment state changes only while
both sensors are credible and the vehicle state permits learning. Freeze
alignment during TAKEOFF transients, LANDING near ground, source disagreement,
and either sensor's recovery period.

Exact `alpha`, `beta`, innovation bounds, and learning rates require replay
identification. Until then the existing fixed fusion remains the flight
behavior baseline while instrumentation is added.

### 5.5 Explicit source-state machine

```text
UNINITIALIZED
  -> RANGE_PRIMARY       credible MTF
  -> BARO_ONLY           credible BMP and no MTF

RANGE_PRIMARY
  -> FUSED               both credible after confirmation
  -> RANGE_DEGRADED      MTF confidence declining
  -> BARO_ONLY           MTF stale/rejected, BMP credible
  -> COAST               neither credible, within bounded coast

FUSED
  -> RANGE_PRIMARY       BMP rejected/stale
  -> RANGE_DEGRADED      MTF degraded, BMP credible
  -> BARO_ONLY           MTF unavailable after hysteresis
  -> COAST               both unavailable

BARO_ONLY / RANGE_DEGRADED
  -> FUSED               recovered sensor credible for N samples and correction ramped
  -> COAST / NONE        source loss / coast expiry

COAST -> credible source with bounded correction, or NONE at timeout
NONE  -> controller-specific safe response; never silently reuse stale data
```

Entry/recovery confirmation prevents one marginal frame from changing source.
The control loop receives state, confidence, ages, and source mode from one
coherent snapshot.

## 6. Stale-data and failsafe contract

There are three different ages and they must not be conflated:

1. sensor progression age: time since a new sensor timestamp/sample;
2. local receive/read-completion age;
3. estimator publication age: task liveness only.

Proposed policy hierarchy:

- `fresh`: eligible for correction;
- `degraded`: eligible with reduced/ramped authority;
- `stale`: no measurement correction, prediction only;
- `coast expired`: source NONE and mode-specific safety action.

Current 250 ms freshness, 600 ms coast, and 800 ms MTF failsafe are retained in
Task 2 because no dropout distribution supports alternatives. Later tests must
show that each timeout is longer than normal jitter/recovery but shorter than
unsafe open-loop divergence.

## 7. Filtering requirements

Filtering must be designed as an end-to-end delay/noise budget:

- retain a short robust outlier gate (median or confirmed-step logic);
- avoid multiple low-pass stages that filter the same quantity unknowingly;
- derive velocity over real timestamps, not assumed rate;
- correct the observer only on new samples;
- keep height and velocity filter bandwidths traceable;
- reset/preset filters bumplessly after stale/reboot/source recovery;
- expose raw, preprocessed, and fused signals during diagnostics.

The current filter constants are not changed. A bench motion profile must
measure delay and RMS/P95 noise before replacement.

## 8. Dual-core execution architecture

### 8.1 Core allocation

| Context | Core | Priority | Expected cadence | Classification |
|---|---:|---:|---:|---|
| Arduino control `loopTask` | Must be verified as core 1 | Framework setting, runtime report required | IMU 1 kHz; control/mixer 250 Hz | Hard real-time flight path |
| ESP-NOW callback/validator | Core 0/framework + pinned task | Callback/framework; validator max-1 | Event driven, bounded packet work | Safety command path |
| Altitude acquisition/preprocess/fusion | Core 0 | 2, above telemetry/OLED | 1 tick service; MTF nominal capability 100 Hz; BMP host attempt >=22 ms | Soft real-time sensor path |
| Telemetry | Core 0 | 1 | 10 Hz | Noncritical |
| OLED | Core 0 | 1 | 4 Hz while safe to service | Noncritical/blocking I/O |

Core 1 must perform only IMU acquisition, attitude estimation, state/control,
mixer, ESC output, and bounded snapshot reads/publishes. It must never call
BMP/OLED I2C, MTF UART blocking reads, ESP-NOW send waits, or serial logging in
the high-rate path.

### 8.2 Communication and ownership

- Altitude task owns parser, range/flow filters, BMP state, alignment, fusion,
  and all sensor diagnostic counters.
- Control task owns flight state, all controllers, mixer, motors, and control
  timing diagnostics.
- Producer publishes one plain-old-data altitude snapshot under an odd/even
  sequence counter; consumer performs at most four lock-free retries.
- Control publishes telemetry by the same mechanism to the lower-priority
  sender task.
- No mutex, queue wait, heap allocation, or unbounded retry is introduced in
  the 1 kHz/250 Hz control path.

The existing seqlocks assume one writer per snapshot. Adding a second sensor
writer directly to `altitudeShared` would violate that invariant. If MTF and
BMP acquisition are later split into separate tasks, each must publish its own
single-writer raw snapshot to one estimator task; they must not share mutable
filter state.

### 8.3 Blocking paths and bounded mitigation (implemented)

| Path | Current risk | Task 2 implementation decision |
|---|---|---|
| BMP `performReading()` | Blocking/latency UNKNOWN | Measure call duration; retain off control core |
| BMP AUX mutex | `portMAX_DELAY` can wait behind OLED | Use a short bounded acquisition; skip/retry and count miss |
| OLED `display()` | Approximately 29 ms bus ownership by source comment | Do not transfer display while airborne; use zero-wait mutex otherwise |
| MTF UART | Draining delayed by BMP/OLED | Drain before and after BMP; report max UART backlog/gap |
| ESP-NOW send | Asynchronous but framework timing UNKNOWN | Remain in low-priority telemetry task |
| Serial events | Driver behavior/runtime UNKNOWN | No periodic serial in control path; keep event messages sparse |

Skipping one noncritical BMP/display operation is preferable to an unbounded
wait. This does not make sensor rate “known”; the miss and measured cadence
must be visible.

### 8.4 Race-condition audit

- `altitudeShared`, `telemShared`, and `oledShared`: safe under the current
  single-writer seqlock pattern; retry is bounded.
- `altitudeTiltCos`: lock-free scalar with no timestamp; acceptable temporary
  approximation, documented above.
- `mode` read by OLED task only through its snapshot; do not read flight-state
  globals directly from non-control tasks.
- Diagnostic counters produced on core 0 must enter the altitude snapshot
  before telemetry; control-core counters enter telemetry directly.
- Resetting diagnostic maxima across cores would add races; use monotonic
  all-time maxima/counters or reset only within their owner.

## 9. Implemented instrumentation contract

Telemetry is version 4 and its packed layout is statically asserted to 248
bytes on both drone and sender, below ESP-NOW's 250-byte legacy limit. It adds:

- explicit manual configuration signature and startup build timestamp;
- current/max control period and cumulative scheduler overrun count;
- MTF decoded-frame rate and maximum decoded-frame gap;
- BMP successful-read rate and maximum call duration;
- maximum AUX mutex wait and count of bounded-lock misses;
- maximum altitude-task service gap and UART backlog;
- MTF checksum/stale counters and BMP failure counter;
- altitude velocity setpoint and altitude integral.

Rates are reported in tenths of hertz. Durations are microseconds saturated to
16 bits; counters saturate in telemetry but remain wider internally where
practical. Startup logs report task core and priority.

These metrics identify whether the next change belongs to sensor configuration,
fusion, scheduling, or control. They do not change sensor weights or gains.
Durations/counters saturate at 65535 in the radio packet; source counters are
kept wider where practical. Rates are one-second-window values in 0.1 Hz.

## 10. Alternatives explicitly deferred

- Splitting MTF/BMP into two writers: deferred until measured BMP blocking or
  UART backlog proves the single task misses its deadline.
- Fusing vertical acceleration: deferred until vibration, gravity subtraction,
  and bias are characterized.
- Numeric confidence mapping: deferred until range error versus strength/
  distance/surface data exists.
- Kalman/EKF: deferred until measurement/process covariance and timing are
  identified.
- Filter/gain/timeout changes: deferred; current flight evidence is from mixed
  historical revisions.

## 11. Verification plan for Task 2 changes

### Static/code verification

- Flight control path contains no new mutex, queue wait, I2C, UART wait, or
  heap allocation.
- Every snapshot remains single-writer and readers have bounded retries.
- Drone/sender telemetry layouts and version match; packet size is <=250 bytes.
- Wrap-safe time arithmetic and saturation are used.
- MTF is drained on both sides of BMP work; OLED cannot own AUX bus in flight.

### Bench test (props removed)

1. Capture startup build/config signature, core IDs, task priorities, and
   telemetry packet size.
2. Run at least 60 s with OLED enabled/disabled and MTF connected/disconnected.
3. Verify MTF rate/gap/backlog, BMP rate/read time, lock misses, altitude-task
   gap, control period, and overrun counters change plausibly.
4. Cover MTF duplicate/stale/bad checksum by replay or controlled disconnect.
5. Confirm sender rejects old telemetry version/size rather than misdecoding.

### Restrained/low-risk test

- Run motors on a guarded restraint and compare timing maxima with props-off
  results.
- Move the frame through known vertical steps; record raw separate sensor
  altitudes and fused response.
- Cover weak range, MTF occlusion, and BMP-only fallback without a throttle
  discontinuity.

### Low-altitude flight observation

Only after earlier gates pass, fly in a netted area with position hold disabled
for the first vertical test. Expected log signature:

- control near 250 Hz without increasing overrun count;
- MTF near its measured bench rate with bounded gap/backlog;
- BMP successful rate consistent with measured call duration and few/no mutex
  misses;
- source changes are explainable by ages/status and do not step altitude;
- altitude integral and correction do not accumulate against saturation.

### Rollback conditions

Roll back the Task 2 implementation if:

- control period/overrun worsens;
- telemetry packet is rejected or exceeds 250 bytes;
- altitude snapshot publication becomes stale;
- MTF backlog/gap grows when BMP/OLED runs;
- BMP cadence collapses because of bounded-lock policy;
- any source switch, altitude step, or throttle step appears;
- KILL, failsafe, or task startup behavior regresses.

## 12. Analysis gate

- Measurement characteristics, rates, latency, range, failure and stale
  behavior: **documented**, with installed unknowns kept UNKNOWN.
- Filtering and IMU interaction: **documented**.
- Fusion alternatives and deterministic target equations/state transitions:
  **documented**.
- Critical/noncritical tasks, affinity, priorities, frequencies, communication,
  races, and blocking paths: **documented**.
- Implementation scope justified by evidence and bounded before mutation:
  **YES** — scheduling/timestamp/observability only; no tuning.
- Bench, low-risk, low-altitude, expected-log, and rollback plan: **documented**.

Analysis was completed and checkpointed before applying sections 8.3 and 9.

## 13. Implementation and verification result

### Implemented source changes

- `drone_alt_hold_1m.ino`
  - telemetry v4/config signature and a statically asserted 248-byte payload;
  - owner-local controller and altitude-task diagnostics transferred through
    the existing lock-free snapshots;
  - MTF range/flow `dt` and regression use sensor timestamp deltas, preventing
    BMP-induced UART bursts from collapsing multiple frame intervals;
  - MTF UART drains immediately before and after BMP service;
  - BMP AUX mutex wait is bounded to 2 ms and misses are counted;
  - BMP freshness begins at host-read completion;
  - OLED bus transfer is suppressed in airborne states and otherwise uses a
    zero-wait mutex;
  - startup reports build/config identity plus actual task core/priority.
- `sender_1m.ino`
  - matching telemetry v4/248-byte schema and 250-byte receive buffer;
  - prints controller `dt`/overruns, altitude velocity setpoint/integral,
    MTF/BMP rates, timing maxima, backlog, and failure counters.

### Static and build verification

- Drone/sender telemetry versions, order, and field sizes: **PASS**; both
  compile-time assertions evaluate to 248 bytes and <=250 bytes.
- Control path lock audit: **PASS**; no mutex, queue wait, BMP/OLED I2C, or MTF
  drain was added to `controlTick()`/`loop()`.
- Altitude snapshot ownership: **PASS**; `altitudeTask` remains the sole writer
  and the control reader remains bounded to four seqlock attempts.
- Timestamp audit: **PASS**; sensor-time deltas drive MTF filter/regression,
  receive time drives host freshness, BMP uses completion time.
- Timeout audit: **PASS**; BMP waits at most 2 ms for AUX, OLED uses a zero-wait
  acquisition, and I2C driver timeouts remain 10 ms (AUX) and 5 ms (MPU).
- Whitespace/static manifest checks: **PASS** for the Task 2 implementation and
  documentation files. A whole-tree scan also found only preserved pre-existing
  whitespace in `AGENTS.md` and `FLIGHT_LOGS_ANALYSIS.md`; Task 2 did not rewrite
  those source-of-truth inputs.
- Fresh flight-controller compile with
  `arduino-cli compile --fqbn esp32:esp32:esp32`: **PASS** in
  `.tmp_task2_compile_20260909_01/drone` — 975095 bytes flash (74%), 49532
  bytes global RAM (15%).
- Fresh sender compile with the same FQBN: **PASS** in
  `.tmp_task2_compile_20260909_01/sender` — 889068 bytes flash (67%), 45784
  bytes global RAM (13%).

The temporary build tree was removed after recording the results; generated
artifacts are not retained as project source.

### What remains UNKNOWN after compilation

Compilation cannot measure installed MTF/BMP rates, read latency, task gap,
UART backlog, mutex misses, control jitter, sensor accuracy, filter delay, or
flight response. The new telemetry makes these observable; no fabricated bench
or flight result is recorded.

### Bench/flight status

- Static/code and compile gates: complete.
- Hardware bench, restrained motor, and low-altitude flight gates: **NOT RUN**;
  require the physical aircraft and controlled test area.
- Current estimator weights/filters remain unchanged and unvalidated in flight.
