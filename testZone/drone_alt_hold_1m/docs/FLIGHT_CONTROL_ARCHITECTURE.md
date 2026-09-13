# Flight-control architecture baseline

Baseline date: 2026-09-09

Pipeline task: TASK 0 — SYSTEM BASELINE

Behavioral changes in this task: none

## 1. Scope and source of truth

The active test aircraft is the `drone_alt_hold_1m` pair:

- `../drone_alt_hold_1m.ino`: complete onboard flight controller.
- `../sender_1m/sender_1m.ino`: complete ESP-NOW ground station.
- `../README.md`: operating notes; useful context, but code is authoritative
  when its values disagree with the sketch.
- `../../../main/cali/cali.ino`: separate ESC calibration/test utility. It is
  not linked into the flight firmware.
- `../../../main/PIN_MAPPING.md` and `../../../main/drone.fzz`: wiring
  references. The Fritzing file is binary and is not executable flight logic.

`main/drone_espnow_final/drone_espnow_final.ino` and
`testZone/drone_pilot/` are historical/parallel firmware, not code called by
this target. There is no shared library layer: both active firmwares are
single-file Arduino sketches.

Line references below refer to the current uncommitted
`drone_alt_hold_1m.ino` baseline inspected for TASK 0.

## 2. Platform and hardware

| Item | Baseline | Evidence / status |
|---|---|---|
| MCU | ESP32 Classic, dual core | Compile guard rejects S2/S3/C3/C6/H2 at lines 1635–1639; README names `ESP32 Dev Module` |
| Framework | Arduino-ESP32 with FreeRTOS/ESP-IDF APIs | Arduino `setup/loop`, `xTaskCreatePinnedToCore`, ESP-NOW and low-level LEDC |
| CPU frequency | UNKNOWN at source level | The sketch does not set or report CPU clock; board profile/build setting must be recorded at build/runtime |
| IMU | MPU6050 at `0x68`, I2C0 GPIO32/33, 400 kHz | Lines 40–49 and 935–955 |
| Range/flow | MicoAir MTF01P, UART1 GPIO19/23, 115200 8N1, Micolink | Lines 51–60, 1113–1177 and 2420–2421 |
| Barometer | BMP388 at `0x77` then `0x76`, I2C1 GPIO21/22, 400 kHz | Lines 51–60 and 1613–1626 |
| Display | SSD1306 128x64 at `0x3C`, shared I2C1 | Lines 62–70 and 2414–2435 |
| Motors | Four PWM ESCs, X frame | M1 FL/CW GPIO26; M2 FR/CCW GPIO13; M3 BR/CW GPIO14; M4 BL/CCW GPIO27 |
| Radio | ESP-NOW on Wi-Fi channel 1 | Lines 72–89 and 1911–2015 |
| Compass/GNSS | None in this target | Yaw is gyro-integrated and may drift; GPS pins in the global pin map are unused here |

The MPU has its own I2C controller (`Wire`). BMP388 and OLED share
`I2C_AUX` and a FreeRTOS mutex. MTF01P has its own hardware UART. Therefore
slow AUX-bus activity cannot directly lock the MPU bus, but it can delay the
core-0 altitude task.

## 3. Execution and core allocation

### 3.1 Onboard ESP32

| Execution context | Affinity | Priority | Nominal cadence | Work | Blocking behavior |
|---|---:|---:|---:|---|---|
| Arduino `loopTask` running `loop()` | Not explicitly pinned by this sketch; normally core 1 on ESP32 Arduino, runtime verification required | UNKNOWN in sketch | MPU schedule 1000 Hz; control every 4 samples = 250 Hz | MPU read, IMU accumulation, attitude estimator, state machine, all flight controllers, mixer, motor write, snapshot publication | MPU I2C read has 5 ms bus timeout; loop otherwise busy-polls until next sample |
| Wi-Fi receive callback | Framework-owned; comment says core 0 | Framework-owned; comment says priority 23 | Event driven | Peer filter, bounded copy, zero-time queue send | Never waits; drops and counts when queue full |
| `espnowTask` | Core 0 | `configMAX_PRIORITIES - 1` | Event driven | Packet length/magic/version/CRC validation, de-duplication, command mailbox | Blocks on queue when idle; bounded processing per packet |
| `altitudeTask` | Core 0 | 2 | Service loop nominally every 1 RTOS tick | Drain MTF UART, attempt BMP read, drain MTF again, calibrate/fuse, publish altitude/flow/diagnostic snapshot | BMP AUX-mutex wait is bounded to 2 ms; read duration remains runtime UNKNOWN |
| `telemTask` | Core 0 | 1 | 10 Hz | Read telemetry seqlock and send 248-byte ESP-NOW packet | Periodic delay; `esp_now_send` is asynchronous |
| `oledTask` | Core 0 | 1 | 4 Hz (`250 ms`) while disarmed/armed-idle | Render snapshot and conditionally transmit display buffer | No airborne transfer; otherwise AUX mutex is zero-wait, so a busy bus drops the display refresh. Task 5 disables OLED if the mutex or task allocation fails |

The safety-critical path is therefore intended to remain on the Arduino loop
core while sensor/radio/display work is on core 0. Because `loopTask` affinity
is not set in this sketch, the exact core and priority must be printed or
verified from the selected Arduino-ESP32 build before treating this separation
as proven on hardware.

### 3.2 Ground station

The sender uses its Arduino `loop()` plus framework ESP-NOW callbacks. It has
no application-created pinned tasks. Its loop drains serial input and a
nonblocking telemetry queue, repeats an action for 1.2 s, sends a heartbeat
every 200 ms (5 Hz), and prints fresh telemetry every 500 ms (2 Hz). Task 5
marks a send pending before calling `esp_now_send()` so an early callback cannot
leave a false pending state stuck true. It never timeout-clears an accepted send;
a delayed callback cannot therefore clear the state of a newer overlapping send.

### 3.3 Inter-context data transfer

| Producer → consumer | Mechanism | Bounded / synchronization behavior |
|---|---|---|
| Wi-Fi callback → `espnowTask` | FreeRTOS queue, length 4, max payload copy 23 bytes | Callback uses zero timeout; overflow increments `espnowLost` |
| `espnowTask` → control loop | Atomic single-slot command mailbox | Single producer writes payload before release-CAS publication; an occupied normal action is preserved for the repeated sender packet to retry, KILL may preempt it, and control claims once with an acquire exchange |
| `altitudeTask` → control loop | Odd/even sequence-counter snapshot (`altitudeSeq`) | Reader copies into a temporary candidate, retries at most four times, commits only a coherent read, otherwise retains the last coherent snapshot; no lock in flight path |
| Control loop → `telemTask` | Odd/even sequence-counter snapshot (`telemSeq`) | Reader retries at most four times |
| Control loop → `oledTask` | Odd/even sequence-counter snapshot (`oledSeq`) | Reader retries at most four times |
| BMP388 ↔ OLED | FreeRTOS mutex around I2C transfers | BMP waits at most 2 ms; OLED never waits and does not transfer while airborne. If mutex allocation fails, BMP remains the only AUX user and OLED is disabled |

There is no persistent log queue or storage writer.

## 4. Timing and frequencies

| Signal/loop | Configured or derived rate | What is actually known |
|---|---:|---|
| MPU6050 internal output | 1000 Hz | `SMPLRT_DIV=0`, DLPF enabled; configured in lines 940–945 |
| MPU I2C acquisition schedule | 1000 Hz | `SAMPLE_HZ=250*4`; missed periods are skipped rather than replayed |
| Attitude/state/control/motor command calculation | 250 Hz | One tick after four acquisition opportunities; runtime `loopHz` counts ticks with a valid accumulated IMU sample |
| ESC PWM carrier | 200 Hz | Hardware PWM frame rate; duty is requested at up to 250 Hz but takes effect at a PWM boundary, so physical command-frame rate is at most 200 Hz |
| MTF UART baud | 115200 baud | Configured |
| MTF measurement publish rate | **UNKNOWN until hardware run** | Telemetry v4 now reports decoded fresh frame rate and sensor-time max gap; manufacturer capability is 100 Hz but the installed value is not yet measured |
| MTF parser service rate | Nominally 1 RTOS tick | Task drains every available byte each service pass |
| BMP388 sensor ODR | 50 Hz | Configured |
| BMP read attempts | At most one per 22 ms, about 45.5 Hz | Telemetry v4 reports successful-read rate and maximum call time; values remain **UNKNOWN** until a hardware run |
| Altitude fusion update | Nominally once per altitude service pass | Runs even between new sensor samples while the last measurement is still fresh |
| Altitude snapshot freshness at control input | 100 ms publisher watchdog | Control clears all sources if the task snapshot itself is older than 100 ms |
| Range/baro/flow sample freshness | 250 ms each | Per-source age gates |
| Both-altitude-source coast | Up to 600 ms | Fusion prediction with vertical-speed decay |
| MTF flight failsafe | 800 ms | TAKEOFF/ALT_HOLD enters failsafe landing when range age exceeds this |
| Telemetry snapshot/send | 10 Hz / 10 Hz | Publish on control path and send from core 0 |
| OLED | 4 Hz only while DISARMED/ARMED_IDLE | In-flight I2C display transfers are intentionally suppressed |
| Local periodic Serial telemetry | Disabled | `TELEM_SERIAL=0`; startup/config/event messages remain active |
| ESP-NOW heartbeat | 5 Hz | Sender interval 200 ms |
| Link failsafe | 3 s | Only a fully valid packet refreshes the watchdog |

Control `dt` is measured from `micros()` between control ticks and constrained
to 0.5–50 ms. The 1 kHz scheduler sets `loopOverrun` and skips ahead when more
than one sample period late. It does not execute catch-up iterations.

## 5. End-to-end dependency map

### 5.1 Attitude and motors

```text
MPU6050 raw accel/gyro (1 kHz I2C)
  -> axis scaling + clip detection
  -> 4-sample accumulation/mean (nominal control rate 250 Hz)
  -> accel PT1 20 Hz + gyro PT2 30 Hz
  -> accel-confidence weighting
  -> Madgwick quaternion + online gyro-bias learning
  -> pitch/roll angles + filtered body rates + integrated yaw heading
  -> pitch/roll angle sqrt-controller
  -> desired pitch/roll rates
  -> rate PID (D on measurement, D-term PT1 25 Hz, integral gating)
  -> yaw heading sqrt-controller / yaw-rate PID
  -> X-frame mixer + desaturation/saturation flags + angle boost
  -> optional thrust linearization (currently disabled, expo 0)
  -> LEDC duty conversion
  -> four ESCs at 200 Hz PWM
```

Mixer equations before desaturation are:

```text
M1 front-left  = T + P + R + Y
M2 front-right = T + P - R - Y
M3 back-right  = T - P - R + Y
M4 back-left   = T - P + R - Y
```

Pitch/roll are allocated first, yaw keeps a configured minimum headroom, and
collective throttle is shifted last to preserve attitude authority. Mixer
saturation flags feed the corresponding PID integrator gate on the next
control tick.

### 5.2 Altitude

```text
MTF01P UART Micolink message 0x51
  -> additive frame-checksum + message length/type + monotonic sensor-time gate
  -> status=0x01, strength>=10, range 30..12000 mm gate
  -> multi-frame step/spike confirmation
  -> tilt correction cos(pitch)*cos(roll)
  -> median-5 -> EMA tau 0.10 s
  -> 15-sample linear-regression vertical speed -> LPF tau 0.20 s
  -> range clearance = filtered sensor-to-ground range - fixed 0.12 m mount offset
                                                                  |
BMP388 pressure (configured 50 Hz ODR, attempted about 45.5 Hz)              |
  -> pressure validity gate                                                  |
  -> pressure-zero altitude conversion                                       |
  -> EMA coefficient 0.08                                                    |
  -> 12-sample finite-difference vertical speed -> LPF coefficient 0.10      |
  -> slow offset alignment to MTF (tau 8 s, innovation gate 0.45 m)          |
                                                                  v           v
                       source selection and observer-style fusion
                         both agree: height 90% MTF / 10% BMP
                                     Vz 70% MTF / 30% BMP
                         else: MTF only, BMP only, COAST, or NONE
                                      |
                                      v
altitude setpoint trajectory -> altitude P + deadband
  -> bounded vertical-velocity target
  -> acceleration limiter
  -> vertical-velocity PI + deadband + anti-windup
  -> asymmetric correction clamp (-45/+70 us)
  -> mode-dependent throttle clamp and two-way slew limiter
  -> angle boost -> mixer -> ESCs
```

The calibration window requires at least 100 MTF range samples, 50 BMP
samples, and MTF range sigma no greater than 0.03 m. It records
`groundRangeM`, but the current control altitude does not subtract that value;
it subtracts the fixed physical mount offset `0.12 m`. BMP altitude uses the
calibrated ground pressure. This is a confirmed description of the current
data path, not a judgment on correctness.

### 5.3 Horizontal position hold

```text
MTF01P message 0x51 flow X/Y
  -> requires accepted range + flow status 0x01 + quality >= 40
  -> stationary zero-bias/noise calibration
  -> scale each sample by synchronized vertical range into m/s
  -> median-5 + EMA tau 0.12 s in SI velocity
  -> configured axis swap/sign mapping
  -> adaptive deadband and finite/speed gate
  -> body forward/right velocity
  -> yaw-reference rotation + integration from post-calibration origin
  -> position error in metres, target (0,0) at calibration origin
  -> position P 0.40 1/s; velocity-vector limit 0.35 m/s
  -> velocity PI + quality-scaled authority
  -> tilt clamp 3.5 deg + slew 8 deg/s
  -> pitch/roll setpoints
  -> existing attitude/rate loops -> mixer -> motors
```

Position hold requires fresh range and flow, altitude at least 0.10 m, and
0.20 s of confirmation after the takeoff throttle-ramp phase. The horizontal
origin is captured once after calibration and odometry remains relative to it
for the powered session. A short 0.35 s flow loss holds the last correction;
longer loss freezes the estimate and slews back toward trim. Recovery confirms
flow for 0.20 s before returning toward the same origin; it does not silently
relock. There is no absolute horizontal position sensor, so unobserved motion,
integrated flow error and gyro-only yaw drift limit attainable accuracy.

## 6. Controller inventory

| Controller | Structure | Output / limit | Source region |
|---|---|---|---|
| Pitch attitude | Angle sqrt-controller → rate PID | Desired rate ±200 deg/s; motor correction limited by current throttle band, maximum configured 350 us | Config 265–300; implementation 727–770 and 2934–2939 |
| Roll attitude | Angle sqrt-controller → rate PID | Same structure/limits as pitch | Same regions |
| Yaw | Integrated gyro heading → sqrt-controller → rate PID | Desired yaw rate ±160 deg/s; mixer preserves 20% yaw headroom before serving additional yaw | 278–300, 2941–2953 |
| Altitude | Position P → acceleration-limited velocity target → velocity PI | Vz target -0.20/+0.35 m/s; correction -45/+70 us; integral ±40 us | 131–196, 2778–2921 |
| Horizontal forward | Calibration-origin position P → velocity PI | Position gain 0.40 1/s; vector velocity <=0.35 m/s; integral ±1.5 deg; total tilt ±3.5 deg | Position constants and `updateHorizontalOdometry()`/`updatePositionHold()` |
| Horizontal right | Same as forward | Same | Same regions |

Rate PID uses derivative on measurement, a 25 Hz D-term filter, its own output
saturation guard, and previous-cycle mixer saturation. Pitch/roll I-term is
relaxed as demanded angle rate grows. All attitude integrators are disabled
during the open-loop takeoff throttle ramp and reset continuously in
ARMED_IDLE.

## 7. Flight state machine, takeoff, and landing

```text
DISARMED
  -- valid ARM request + 2 s continuous pre-arm checks --> ARMED_IDLE
  -- severe sensor/attitude/KILL fault -----------------> TRIPPED

ARMED_IDLE
  -- TAKEOFF + fresh MTF/BMP/link/attitude checks ------> TAKEOFF
  -- DISARM, link loss, or 15 s idle timeout -----------> DISARMED

TAKEOFF / SPOOL_UP
  -- actual collective reaches 1350 us -----------------> GROUND_TRANSITION

TAKEOFF / GROUND_TRANSITION
  -- lift predicate continuous for 0.15 s --------------> CONTROLLED_ASCENT
  -- no lift for 4 s / pre-lift abort ------------------> MOTOR_RAMP_DOWN

TAKEOFF / CONTROLLED_ASCENT
  -- near 0.50 m target ---------------------------------> ALTITUDE_CAPTURE

TAKEOFF / ALTITUDE_CAPTURE
  -- altitude within 0.08 m and |Vz|<0.15 m/s for 0.6 s -> ALT_HOLD
  -- LAND -----------------------------------------------> LANDING
  -- controlled failsafe -------------------------------> FAILSAFE
  -- severe fault ---------------------------------------> TRIPPED

ALT_HOLD
  -- LAND -----------------------------------------------> LANDING
  -- controlled failsafe -------------------------------> FAILSAFE
  -- severe fault ---------------------------------------> TRIPPED

LANDING / FAILSAFE / CONTROLLED_DESCENT
  -- altitude enters near-ground region ----------------> GROUND_APPROACH

LANDING / FAILSAFE / GROUND_APPROACH
  -- fresh MTF clearance <=0.10 m ----------------------> TOUCHDOWN_DETECTION

LANDING / FAILSAFE / TOUCHDOWN_DETECTION
  -- fresh MTF clearance<=0.10 m and |Vz|<=0.25 m/s
     for 0.40 s, or 2 s timeout ------------------------> MOTOR_RAMP_DOWN

LANDING / FAILSAFE / MOTOR_RAMP_DOWN
  -- 1.0 s per-motor interpolation to 1000 us ----------> DISARMED
  -- landing timeout / blind timeout / severe fault ----> TRIPPED

TRIPPED -- no software transition; reset required ------> boot/DISARMED
```

Pre-arm checks require a valid sender link, calibrated IMU and altitude
estimator, fresh MTF range/flow and BMP, an initialized calibration-origin
position, near-zero altitude,
level/still attitude, and no vibration or clipping.

Current takeoff behavior (TASK 3, refined by TASK 10):

1. ARMED_IDLE slews all motors from 1000 to 1050 us over 0.4 s.
2. TAKEOFF initializes its base from the actual current collective, nominally
   1050 us. The base rises continuously at 40 us/s; 1350 us is now only the
   SPOOL_UP/GROUND_TRANSITION boundary.
3. Liftoff requires fresh altitude and either clearance >=0.06 m or both
   clearance >=0.025 m and Vz >=0.10 m/s continuously for 0.15 s.
4. Before confirmation, attitude integrators and altitude PI are reset/gated,
   the altitude setpoint follows the measured height, and the base stops at
   hover rather than accumulating closed-loop demand on the ground.
5. `takeoffElapsedS` advances from control `dt`: the phase-local no-lift abort
   is active after 4 s in GROUND_TRANSITION and the overall takeoff timeout is
   active after 20 s. The longer total bound contains the slower airborne
   trajectory; it does not extend powered no-rise time on the ground.
6. Confirmed liftoff transfers at the current command without a collective
   step; the altitude setpoint then rises toward 0.50 m at 0.12 m/s, with
   velocity demand limited to +0.20/-0.12 m/s.

Current landing behavior (TASK 3, refined by TASK 10):

1. Landing starts from current altitude setpoint and actual current throttle;
   it no longer clamps the slew state to hover at entry.
2. Setpoint descends at 0.08 m/s above 0.40 m and 0.04 m/s near ground.
3. Landing throttle may increase at 55 us/s and decrease at 50 us/s, or
   35 us/s near ground. It is capped at `min(hover+50, 1470)`.
4. If all altitude sources are lost, throttle falls open-loop at 25 us/s and a
   blind-landing timeout trips after 8 s.
5. Landing suppresses horizontal position hold. A fresh MTF clearance <=0.10 m
   latches TOUCHDOWN_DETECTION, bypasses controller/mixer authority, and decays
   each actual motor toward 1050 us at 250 us/s without increasing any motor.
6. The existing 0.40 s stable touchdown condition or a bounded 2 s touchdown
   timeout enters MOTOR_RAMP_DOWN. Each motor then interpolates from its actual
   entry value to 1000 us over 1.0 s before DISARMED.

The TASK 0 baseline found the dead takeoff timer and 1050-to-1350 us edge; both
are fixed by TASK 3. Numeric liftoff confirmation, touchdown decay/timeout, and
final ramp remain engineering starting bounds with hardware evidence UNKNOWN.

## 8. Failsafes

| Trigger | State/action | Evidence |
|---|---|---|
| Invalid/no sender packet for 3 s in TAKEOFF/ALT_HOLD | `FAILSAFE`, controlled landing | 2073–2092 |
| MTF range age >800 ms in TAKEOFF/ALT_HOLD | `FAILSAFE`, BMP may continue altitude estimate | 2646–2653 |
| Fused/selected altitude >1.60 m | `FAILSAFE` landing | 2654–2656 |
| Flight duration >60 s | `FAILSAFE` landing | 2658–2660 |
| Both altitude sources unavailable during landing | Open-loop throttle descent, then TRIPPED after 8 s | 2837–2853 |
| Landing state lasts >24 s | `TRIPPED` | `updateAltitudeSafety()` |
| Pitch/roll setpoint error >45 deg for 0.25 s | `TRIPPED` | 2566–2578 |
| Pitch/roll/rate/yaw state becomes NaN or Inf | `TRIPPED` | Task 5 finite invariant in `estimateAttitude()` |
| 50 consecutive scheduled MPU read failures (nominally about 50 ms) | `TRIPPED` | 129, 3032–3039 |
| Explicit KILL | `TRIPPED`, start motor values forced to idle | 2057–2060 |

Normal TRIPPED handling ramps each last motor command to 1000 us over 1.5 s.
KILL overrides the stored start values to 1000 us, making that output shutdown
immediate on the next control handling path. The code contains no hardware
watchdog configuration, battery-voltage failsafe, ESC telemetry, motor-failure
detection, geofence, RC override, or redundant IMU. Their status is absent,
not inferred healthy.

## 9. Logging and observability

Available outputs:

- Startup/configuration logs: sensor identity/configuration, IMU transaction
  timing sample, calibration outcomes, task/radio initialization.
- Event logs: commands, public-mode and internal transition-phase changes,
  link state, takeoff/landing/failsafe reasons, selected warnings.
- ESP-NOW telemetry v5: 248 bytes at 10 Hz, including loop rate, mode/flags,
  attitude/rates/controller outputs/integrals, learned and calibration biases,
  vibration/clip data, four motor commands, throttle/hover values, link packet
  counters, reason, fused/range/baro altitude, Vz, setpoint, altitude correction,
  sensor ages/source/strength, transition phase, flow velocity/position/tilt/quality/age, altitude
  velocity setpoint/integral, build-config signature, control period/overruns,
  MTF/BMP rates, timing maxima, UART backlog, and sensor error counters.
- OLED at 4 Hz while DISARMED/ARMED_IDLE: compact mode/link/throttle,
  altitude/setpoint/Vz/source, flow/quality, integrated position, reason and
  action. Its I2C transfer is suppressed while airborne.
- Optional local periodic serial line at 10 Hz exists but is compiled out by
  `TELEM_SERIAL=0`.

Missing or UNKNOWN observability:

- No persistent onboard log/storage.
- No stack high-water or CPU-load metrics.
- MTF rate/max sensor-time gap and UART backlog are instrumented, but no
  hardware value has yet been recorded.
- BMP successful-read rate, maximum call duration and AUX wait/miss are
  instrumented, but no hardware value has yet been recorded.
- Altitude-task maximum service gap and control current/max period/overrun
  count are instrumented, but no hardware value has yet been recorded.
- `loopOverrun` reports the most recent scheduler condition; telemetry v5 also
  reports the cumulative `controlOverruns` counter and maximum control period.
- MTF checksum/stale counters are telemetered; other parser breakdowns and
  physical sample latency remain unavailable.

## 10. Parameter source map

All active flight parameters are compile-time constants or globals in
`drone_alt_hold_1m.ino`, except hover can be changed in RAM from the sender
while DISARMED.

| Subsystem | Symbols / region |
|---|---|
| Hardware pins and buses | Lines 39–70 |
| ESP-NOW identity, channel, keys, queue | Lines 72–89 |
| Telemetry rate/version | Lines 91–99 |
| IMU/control scheduler | Lines 101–107 |
| ESC idle/hover/band | Lines 109–120 |
| Arm/spool/trip safety | Lines 122–129 |
| Takeoff, landing, altitude PI, timeouts/freshness | Lines 131–178 |
| MTF/BMP fusion | Lines 180–196 |
| Optical flow and position hold | Lines 198–227 |
| Attitude trip, vibration, mount and mixer signs | Lines 229–243 |
| Mechanical/attitude trim | Lines 245–249 |
| Attitude filters/Madgwick | Lines 251–263 |
| Pitch/roll/yaw PID and limits | Lines 265–300 |
| Thrust curve and angle boost | Lines 302–309 |
| IMU calibration convergence | Lines 311–315 |
| Sender heartbeat/repeat/link timers and local hover default | `sender_1m.ino` lines 15–24 and 79–87 |

## 11. Documentation-value reconciliation

These differences were confirmed at TASK 0 so later tasks used code values
rather than silently tuning against prose. TASK 6 reconciles the README to the
unchanged authoritative constants. The sender's initial `hoverUs=1400` is only
an ignored payload on non-`SET_HOVER` commands; an actual `HOVER n` command
parses and sends the explicit `n`, so it does not override the drone's 1430 us
boot default.

| Topic | Current code | README after Task 6 |
|---|---:|---:|
| Flight-controller hover default | 1430 us | 1430 us; explicitly uncalibrated |
| Flow minimum quality | 40 | 40; >=80 is described only as an initial-test margin |
| Flow minimum range | 50 mm | 50 mm |
| Position-hold confirmation | 0.20 s | 0.20 s |
| Position-hold minimum altitude | 0.10 m | 0.10 m |
| Position-hold max tilt | 3.5 deg | 3.5 deg |
| Position tilt slew | 8 deg/s | 8 deg/s |
| Low-quality authority | 40% at quality 40; full at 70 | Same, explicitly unvalidated |
| Landing throttle decrease | 70 us/s, 50 us/s near ground | Reconciled in README by TASK 3 |
| Landing confirmation | 0.40 s | Reconciled in README by TASK 3; followed by explicit motor ramp |

## 12. TASK 0 completion-gate assessment

- Architecture understood: **YES** — execution, estimation, control, state,
  actuation, communication, and observability paths are mapped above.
- Relevant source files located: **YES** — active onboard, sender, wiring and
  calibration sources are identified; legacy sketches are explicitly excluded
  from the active call graph.
- Loop frequencies identified or marked UNKNOWN: **YES** — configured,
  derived, and runtime-unknown rates are separated in section 4.
- Parameter locations identified: **YES** — section 10 maps every requested
  subsystem to its source region.

Compilation and flight validation are not completion criteria for this
documentation-only baseline. A compile was not claimed: the managed workspace
could invoke `arduino-cli 1.5.2-rc.1`, but its Arduino15 platform directory was
not accessible in the sandbox during this task.

## 13. TASK 2 architecture revision (analysis checkpoint)

The detailed sensor and scheduling analysis is in
`SENSOR_FUSION_ANALYSIS.md`. This section records the architectural delta from
the TASK 0 baseline before code mutation.

### 13.1 Selected target

- Retain core 1 as the lock-free deterministic IMU/attitude/control/mixer/ESC
  path; verify the Arduino loop-task affinity and priority at runtime.
- Retain one core-0 owner for MTF parsing, BMP acquisition, altitude/flow
  preprocessing, fusion, and the altitude snapshot until timing evidence
  justifies splitting it.
- Retain the single-writer odd/even altitude snapshot. If sensors are later
  split across tasks, use one timestamped raw snapshot per producer and one
  estimator writer; never allow two writers to the fused snapshot.
- Select a confidence-aware, multi-rate alpha-beta/complementary observer as
  the future estimator. Defer numeric confidence/observer gains until installed
  noise, latency, and innovation distributions exist.
- Do not add vertical IMU acceleration or a Kalman/EKF solely for complexity;
  both require a measured bias/vibration/process model.

### 13.2 Task 2 implementation boundary

The evidence justifies only these changes before new bench data:

1. replace indefinite AUX-mutex acquisition with bounded skip/retry behavior;
2. prevent noncritical OLED I2C transfers while airborne;
3. drain MTF UART before and after BMP work;
4. timestamp BMP freshness at read completion rather than the pre-read poll;
5. publish sensor/task/control timing, rate, backlog, failure, altitude
   integral, and velocity-setpoint diagnostics in a versioned <=250-byte
   telemetry packet;
6. print build/config identity, task affinity, and priority at startup.

No flight gain, filter constant, sensor weight, confidence threshold, source
timeout, takeoff, landing, position-hold, mixer, or motor behavior is changed
by this architecture step.

### 13.3 Rate and latency truth table

| Quantity | Source/configuration | Runtime status before instrumentation |
|---|---:|---|
| MTF manufacturer output capability | 100 Hz | Not configured by sketch; installed rate **UNKNOWN** |
| MTF parser service | nominal 1 RTOS tick | actual gap/backlog **UNKNOWN** |
| BMP host attempt interval | >=22 ms | about 45.5 attempts/s by code |
| BMP configured ODR field | 50 Hz | not actual cadence because Adafruit read uses forced mode |
| BMP conversion/read call | datasheet up to about 21.53 ms at 8x/1x | installed/library duration **UNKNOWN** |
| Successful BMP samples | — | **UNKNOWN** |
| Altitude task publication | nominal 1 RTOS tick | actual maximum gap **UNKNOWN** |
| Control | nominal 250 Hz | actual current/max period and cumulative overruns not yet reported |

Telemetry v4 now exposes the fields needed to turn these runtime UNKNOWNs into
measured values. Documentation continues to label them UNKNOWN until bench logs
exist.

### 13.4 Implementation and build status

The six bounded changes in section 13.2 are implemented. MTF range/flow filter
time and range-velocity regression now use MTF `sensorMs` deltas, while local
receive/drain time remains the freshness clock. This prevents BMP work from
making buffered MTF frames appear to have near-zero sample spacing.

Both fresh Arduino CLI builds pass with FQBN `esp32:esp32:esp32`, using
`.tmp_task2_compile_20260909_01/drone` and
`.tmp_task2_compile_20260909_01/sender`. Telemetry v4 is statically asserted to
248 bytes:

- flight controller: 975095 bytes flash (74%), 49532 bytes global RAM (15%);
- sender: 889068 bytes flash (67%), 45784 bytes global RAM (13%).

Hardware timing, sensor accuracy, bench, restrained, and flight validation are
still pending. Build success does not close those gates.

## 14. TASK 3 takeoff/landing revision

TASK 3 implements the internal transition phases documented in section 7 and
the parameter/evidence ledger in `TAKEOFF_LANDING_DESIGN.md`. Its bounded source
changes are:

1. activate the intended takeoff timers and begin the 40 us/s ramp from actual
   collective instead of stepping from nominal 1050 to 1350 us;
2. require a continuous liftoff predicate and keep altitude/attitude stored
   demand gated until confirmation;
3. split takeoff into spool, ground transition, controlled ascent, and capture;
4. enter landing from actual throttle without an entry clamp, suppress
   horizontal hold, latch touchdown, monotonically remove per-motor thrust,
   then perform a bounded final ramp before disarming;
5. expose the internal phase in the existing telemetry-v4 reserved byte and
   sender status output without changing packet size or version.

Static schema, phase/timer, no-control-lock, single-writer, timeout,
monotonic-ramp, packet-size, and whitespace checks pass. Warning-enabled fresh
Arduino CLI builds of both final sketches with FQBN `esp32:esp32:esp32` pass in
`.tmp_task3_compile_20260909_02`: flight controller 976771 bytes flash (74%) and
49556 bytes global RAM (15%); sender 889324 bytes flash (67%) and 45784 bytes
global RAM (13%). No compiler warning was emitted. No hardware test is claimed;
all new numeric transition values remain unvalidated engineering bounds.

## 15. TASK 4 tilted-surface takeoff revision

The detailed root-cause audit, reference limitation, parameter ledger, and
rollback plan are in `TILTED_TAKEOFF_DESIGN.md`. The public flight mode and Task
3 transition-phase schema do not change.

At TAKEOFF, the controller now rejects reported pitch/roll above 8 deg, captures
the current attitude, resets controllers, and presets both setpoint LPFs to that
capture. SPOOL_UP holds the capture. GROUND_TRANSITION slews the independent
profile toward existing trim at 3 deg/s; position hold is suppressed until the
profile and authority transfer finish.

Before confirmed liftoff, pitch/roll/yaw integrators remain gated, angle boost
is bypassed, and a ground-only X-frame mixer scales the combined correction so
no motor exceeds the requested Task 3 collective by 50 us. It never shifts its
base upward to preserve a low-side correction. A reported angle above 12 deg or
pitch/roll rate above 45 deg/s for 0.10 s enters the existing pre-lift failsafe
motor ramp. At Task 4 completion the timers were 4/15 s; Task 10 retains the
4 s no-rise bound and changes only the total airborne-transition bound to 20 s.

After liftoff confirmation, target slew continues and rate integrals remain
gated while each motor output crossfades from the ground-limited result to the
unchanged normal mixer/angle-boost result over 0.50 s. Normal airborne mixer,
gains, filters, motor bounds, altitude, position, landing, and failsafe behavior
outside this handover are unchanged.

Static/schema/ownership/lock/whitespace checks and ground/target/transfer model
tests pass. Warning-enabled fresh Arduino CLI builds with FQBN
`esp32:esp32:esp32` pass in `.tmp_task4_compile_20260909_01`: flight controller
979267 bytes flash/49588 bytes global RAM and sender 889324 bytes flash/45784
bytes global RAM, with no compiler warning.

Calibration must occur stationary on a known level surface before moving the
craft to a mild launch tilt. Boot offsets conflate sensor mounting bias and
surface tilt, so boot-on-tilt earth-level attitude remains unobservable. No
hardware run is claimed; all new numeric bounds remain unvalidated.

## 16. TASK 5 robustness revision

The detailed fault audit and evidence boundary are in `ROBUSTNESS_AUDIT.md`.
Task 5 changes synchronization and invalid-state handling only; it does not
change a controller gain, filter/fusion weight, sensor/flight threshold,
takeoff/landing transition, position-hold parameter, mixer limit, or motor
value from Tasks 3–4.

- The ESP-NOW task is the only command producer. It writes associated payload
  before release-CAS publication into an empty mailbox, does not overwrite a
  pending normal action, and permits KILL to preempt. The control loop reads the
  payload while occupied and claims/clears the command with one acquire atomic
  exchange. There is no lock or wait in the control path.
- The sender sets its pending state before `esp_now_send()`, so a callback that
  runs before the call returns cannot be overwritten by a later `true` store.
  An accepted send is never timeout-cleared; a delayed callback therefore
  cannot clear a newer send. Complete callback loss stops new sends and lets
  the existing drone link watchdog select its controlled failsafe behavior.
- `altitudeRead()` copies into a temporary candidate and commits only after a
  stable sequence. Four collisions retain the last coherent control snapshot;
  failures are counted locally, and the unchanged 100 ms publication watchdog
  invalidates an old retained snapshot. The control side sanitizes altitude,
  vertical speed, range/barometer/flow values, ages, source enum, and freshness.
- Madgwick quaternion normalization, both PID inputs, final fused altitude/
  velocity, MTF tilt compensation, attitude/rates/yaw state, and pre-arm values
  explicitly reject NaN and infinity. Invalid controlling altitude becomes no
  source and follows the existing hold-then-failsafe-land behavior. Invalid
  attitude enters the existing latched TRIPPED motor ramp.
- Setup attaches all ESC channels and writes 1000 us idle before task/resource
  allocation can stop initialization. If the AUX mutex is unavailable, OLED is
  disabled and BMP remains the sole AUX-bus task; failed OLED task allocation
  also disables it and reports the degraded state.

Timing telemetry still records current/max control period and overruns, but no
control-deadline trip threshold was added. Installed loop-task affinity, stack
high-water marks, scheduling jitter, stall duration, and airframe response are
**UNKNOWN** pending prop-off/restrained runtime evidence. An evidence-based
policy belongs after those measurements, not in this static pass.

Deterministic models pass command and sender callback interleavings, coherent/
torn snapshot attempts, NaN/+Inf/-Inf fallback, and the unchanged 100/600/800
ms freshness boundaries. Schema/ownership/no-control-lock and Task 3/4 models
also pass: 81 matching telemetry fields at 248 bytes, 625 ground-mixer cases,
3 deg/s target slew, 0.50 s transfer endpoints, and monotonic landing ramps.
Warning-enabled fresh Arduino CLI builds with FQBN `esp32:esp32:esp32` pass:

- flight controller: 980391 bytes flash (74%), 49588 bytes global RAM (15%);
- sender: 889368 bytes flash (67%), 45792 bytes global RAM (13%).

No compiler warning was emitted. Config signature `0xA41F0501` identifies this
firmware. No hardware or flight validation is claimed.

## 17. TASK 6 full-system audit revision

The complete post-Task-5 audit and remaining-issue matrix are in
`FINAL_SYSTEM_AUDIT.md`. No CRITICAL defect was found by static inspection;
that result does not close any hardware or flight gate. Two HIGH source bugs
have bounded corrections:

- MPU initialization now requires every configuration readback to succeed and
  match the filter, gyro range, accelerometer range, and sample divider that
  the fixed estimator scale factors assume. A failure enters the existing
  fatal startup path after ESC idle output is already active.
- Landing entry is idempotent. A repeated LAND cannot restart phase/timers or
  downgrade an active FAILSAFE; a safety request may promote LANDING to
  FAILSAFE while preserving the current phase and bounded landing clock.

The current position path uses per-sample SI flow normalization, a retained
post-calibration origin, position error in metres with the existing 0.40 1/s
gain, and an isotropic 0.35 m/s velocity-vector limit. The altitude target is
0.50 m clearance. Velocity PI, tilt, slew, sensor/fusion thresholds,
takeoff/landing bounds, mixer/motor values, task rates and control-deadline
policy otherwise remain unchanged in Task 9. Task 10 supersedes the build
identity with config signature `0xA41F1001`; telemetry
v5 identify this source; the detailed ledger and evidence limits are in
`POSITION_HOLD_HEIGHT_NORMALIZATION.md`.

Final static/model/schema/build results are recorded in
`FINAL_SYSTEM_AUDIT.md`; installed timing, stack margin, sensor response,
controller stability, motor response, and flight behavior remain **UNKNOWN**.

## 18. TASK 10 CG compensation and gentle vertical profile

The reported battery position is forward of the thrust centre and produces a
nose-down/forward takeoff transient. Increasing attitude Kp/Kd would amplify
all disturbances but would not provide the missing static moment before an
error develops; the rate integrator is intentionally gated through ground and
handover. The current architecture therefore separates load feed-forward from
feedback control:

- level demand is restored from `PITCH_TRIM_DEG/ROLL_TRIM_DEG=-0.57/-0.16` to
  `0/0 deg`, avoiding a permanent commanded translation;
- `CG_PITCH_COMP_US_AT_HOVER=+15 us` adds pitch-axis mixer moment, raising
  M1/M2 and lowering M3/M4 for the reported forward CG; roll compensation is
  zero;
- compensation is zero at `MOTOR_MIN_US`, ramps linearly with collective,
  saturates at hover, and is hard-bounded to +/-30 us. It passes through the
  existing ground/transfer/normal mixers and their actuator clamps;
- rate-PID gains, filters, integrator bounds, angle controller, mixer priority,
  and motor ceilings remain unchanged because no current-build high-rate flight
  trace justifies retuning them.

The post-liftoff altitude-setpoint slew changes 0.25 -> 0.12 m/s, vertical
velocity limits +0.35/-0.20 -> +0.20/-0.12 m/s, velocity-demand acceleration
limits 0.40/0.60 -> 0.20/0.35 m/s2, and normal collective up-slew 180 ->
90 us/s. Landing target slew changes 0.12/0.05 -> 0.08/0.04 m/s and landing
collective down-slew changes 70/50 -> 50/35 us/s; the 55 us/s upward braking
authority is retained. Total takeoff/landing timeouts change 15/18 -> 20/24 s
to contain the slower profiles, while the 4 s ground no-rise limit is unchanged.

These are bounded initial settings, not airworthiness evidence. The preferred
fix is still mechanical CG correction. If the battery cannot be moved, the
15 us value must be confirmed on a restrained rig and adjusted by no more than
2 us per signed build using pitch/rate, front/rear motor pair, forward-flow and
saturation evidence.
