# CODEX PROJECT STATE

PROJECT_STATUS: COMPLETE
CURRENT_TASK: NONE
LAST_COMPLETED_TASK: TASK_10_FORWARD_CG_AND_GENTLE_VERTICAL_PROFILE

## COMPLETED

- Read `AGENTS.md`, `CODEX_TASK.md` (the repository uses the singular filename),
  and the prior `CODEX_STATE.md` completely.
- Inspected `git status`, all tracked text diffs in complete chunks, and the
  metadata for the modified binary `main/drone.fzz`.
- Read the complete current flight-controller sketch and sender sketch.
- Read the current project README and the ESC calibration utility.
- Located the control, estimator, motor, command, telemetry, display, and
  calibration paths needed for the TASK 0 architecture baseline.
- Created and cross-checked `docs/FLIGHT_CONTROL_ARCHITECTURE.md`, including
  MCU/platform, task/core scheduling, all sensor/control rates, estimator and
  PID structure, mixer/motor path, takeoff/landing/state transitions, failsafes,
  logging, shared-state synchronization, parameter locations, and explicit
  runtime UNKNOWNs.
- TASK 0 completion gate passed and TASK 0 was marked DONE in `CODEX_TASK.md`.
- Read all 286 lines and parsed all 63 telemetry rows in
  `FLIGHT_LOGS_ANALYSIS.md`.
- Correlated every requested failure category with current source and parameter
  locations while separating logged historical revisions from current code.
- Created `docs/FLIGHT_ISSUES.md` with 12 evidence records. Every record has
  Evidence, Likely root cause, Confidence, Relevant code, Relevant parameter,
  Recommended change, and How to verify.
- Classified claims as confirmed, strongly suspected, or hypothesis/unresolved;
  recorded safe staged verification and rollback conditions.
- TASK 1 completion checks passed and TASK 1 was marked DONE in
  `CODEX_TASK.md`. No flight behavior or parameter was changed.
- TASK 2 sensor/fusion and dual-core analysis was completed before bounded
  implementation; installed-system unknowns remain explicitly UNKNOWN.
- TASK 2 implemented bounded AUX scheduling, airborne OLED suppression,
  pre/post-BMP MTF draining, BMP completion timestamps, and telemetry v4
  diagnostics without changing flight gains, weights, thresholds, timeouts,
  takeoff/landing, position, mixer, or motor parameters.
- TASK 2 schema, packet-size, ownership, no-control-lock, timeout, whitespace,
  documentation coverage, and fresh dual-sketch compile gates all passed.
- TASK 2 was marked DONE only after its completion gate passed.
- TASK 3 audited the dead takeoff clock, 1050-to-1350 us command edge, ground
  stored-demand path, and landing entry/touchdown hazards before source mutation.
- TASK 3 implemented and documented explicit takeoff/landing transition phases,
  active abort clocks, continuous actual-output ramps, confirmed liftoff,
  monotonic touchdown/final motor ramps, and phase telemetry.
- TASK 3 schema, packet, phase/timer, single-writer/no-control-lock, timeout,
  monotonic-ramp, whitespace, documentation, and final fresh dual-build gates
  all passed. TASK 3 was marked DONE.
- TASK 4 audited attitude reference, ground P/D and I gating, mixer
  saturation/base shift, motor headroom, angle boost, ground detection, aborts,
  setpoint filtering, and bumpless-transfer limitations before mutation.
- TASK 4 implemented captured/preset ground attitude, limited target leveling,
  sustained angle/rate aborts, a no-base-shift ground mixer capped at base+50 us,
  and a 0.50 s per-motor handover to the unchanged normal mixer.
- TASK 4 source/schema/ownership/no-lock/whitespace, exhaustive ground-mixer,
  target-slew and transfer models, documentation, and warning-enabled fresh
  dual-build gates all passed. TASK 4 was marked DONE.
- TASK 5 audited command/sender races, estimator/control finite validity,
  seqlock fallback, source freshness/recovery, task/resource failure, deadline
  evidence, and preserved Task 3/4 motor/state invariants before completing its
  bounded robustness implementation.
- TASK 5 command/sender interleaving, snapshot/non-finite/staleness fault models,
  schema/ownership/no-control-lock/Task-3/4/whitespace gates, documentation
  reconciliation, and final warning-enabled dual builds all passed. TASK 5 was
  marked DONE. Hardware timing/fault response remains explicitly UNKNOWN.
- TASK 6 completed the post-Task-5 full-system audit and classified every
  remaining finding by severity and type in `docs/FINAL_SYSTEM_AUDIT.md` before
  source mutation.
- TASK 6 fixed two HIGH static bugs: MPU configuration readback is fail-closed,
  and active LANDING/FAILSAFE entry is idempotent with one-way failsafe
  promotion. It changed no gain or physical flight parameter.
- TASK 6 fault/regression, static ownership/no-lock, schema, documentation,
  whitespace, artifact-cleanup, and fresh warning-enabled dual-build gates all
  passed. TASK 6 was marked DONE; hardware/flight evidence remains pending.
- TASK 7 reconciled every important source parameter/symbol into
  `docs/PARAMETER_HANDBOOK.md`, completed the staged and repeatable
  `docs/TEST_PLAN.md`, linked the technical documents from README, and passed
  the final Tasks 0--7 manifest, schema, ownership, model, link, and whitespace
  gates. TASK 7 was marked DONE. This completes the software/document pipeline;
  hardware and flight validation remain pending and are not claimed.
- TASK 9 corrected the 50 cm requirement: automatic altitude target is 0.50 m,
  while horizontal position targets zero error at the calibration origin with
  no 0.50 m acceptance radius. Height-scaled SI flow, static/model/link checks,
  and clean warning-enabled dual builds passed. Hardware validation remains
  pending and is not claimed.
- TASK 10 replaced the nonzero angle-trim CG workaround with bounded,
  collective-scaled +15 us pitch mixer feed-forward for the reported forward
  battery offset, restored the level target to 0/0 deg, and slowed takeoff and
  landing trajectories. Exhaustive ground-mixer/compensation and timeout models,
  current-document/whitespace checks, and warning-enabled dual builds passed.
  The exact installed CG compensation remains hardware-validation dependent.

## CURRENT WORK

TASK 10 responds to the reported forward battery/CG offset, nose-down takeoff
and excessive takeoff/landing speed. Static analysis confirms that the existing
nonzero angle trim commands horizontal acceleration, while increasing Kp/Kd
would not supply the missing pre-error static moment and the rate integrator is
gated through the ground/transfer phases. The implementation now uses bounded
collective-scaled pitch mixer feed-forward (+15 us at hover, zero at 1050 us),
zero level angle trim, slower vertical trajectories and matching longer total
timeouts without changing the 4 s ground no-rise exposure. Both warning-enabled
sketch builds, 625-case ground mixer model, compensation endpoint/monotonicity
model, timeout-margin model, documentation/static checks and final diff audit
pass. TASK 10 is complete for the authorized software/document scope; hardware
behavior remains UNKNOWN.

TASK 9 corrects the Task 8 requirement interpretation. The 0.50 m value is the
altitude-clearance target, not a horizontal acceptance radius. Horizontal
odometry retains the initial post-calibration origin, and the controller drives
signed metric error directly toward `(0,0)` using `POS_HOLD_KP_PER_S=0.40`
with a vector 0.35 m/s cap. ARM/TAKEOFF requires fresh flow plus a valid
initialized origin, with no invented horizontal distance threshold. Each
synchronized flow/range sample remains converted to SI velocity before
filtering so the horizontal controller is independent of commanded altitude.
Task 9 checks and clean warning-enabled dual builds passed; no physical
accuracy is claimed without independent measurement.

Tasks 0--7 are complete for the software, static-analysis, deterministic-model,
build, and documentation scope. TASK 7 documentation now matches the exact
Task 6 firmware: the handbook uses authoritative symbols and values; the test
plan distinguishes the sender's printed subset from packet-level/diagnostic
capture; and README links the complete technical set without presenting the
historical Task 5 signature as current. No firmware parameter or behavior was
changed in TASK 7. The next work is staged physical validation under
`docs/TEST_PLAN.md`; no hardware or flight result is claimed by project
completion status.

## FILES_INSPECTED

- `testZone/drone_alt_hold_1m/AGENTS.md`
- `testZone/drone_alt_hold_1m/CODEX_TASK.md`
- `testZone/drone_alt_hold_1m/CODEX_STATE.md`
- `testZone/drone_alt_hold_1m/drone_alt_hold_1m.ino` (complete current file)
- `testZone/drone_alt_hold_1m/sender_1m/sender_1m.ino` (complete current file)
- `testZone/drone_alt_hold_1m/README.md` (complete current file)
- `main/PIN_MAPPING.md` (complete tracked diff)
- `main/drone.fzz` (binary-change metadata; content is not representable by
  normal `git diff`)
- `main/cali/cali.ino`
- `main/esc.txt`
- `testZone/drone_alt_hold_1m/docs/FLIGHT_CONTROL_ARCHITECTURE.md`
- `testZone/drone_alt_hold_1m/FLIGHT_LOGS_ANALYSIS.md` (complete, all 286 lines)
- `testZone/drone_alt_hold_1m/docs/FLIGHT_ISSUES.md`
- `testZone/drone_alt_hold_1m/docs/SENSOR_FUSION_ANALYSIS.md`
- `testZone/drone_alt_hold_1m/docs/TAKEOFF_LANDING_DESIGN.md`
- `testZone/drone_alt_hold_1m/docs/TILTED_TAKEOFF_DESIGN.md`
- `testZone/drone_alt_hold_1m/docs/ROBUSTNESS_AUDIT.md`
- `testZone/drone_alt_hold_1m/docs/FINAL_SYSTEM_AUDIT.md`
- `testZone/drone_alt_hold_1m/docs/PARAMETER_HANDBOOK.md` (complete current
  TASK 7 draft)
- `testZone/drone_alt_hold_1m/docs/TEST_PLAN.md` (complete existing TASK 7
  draft)
- `testZone/testPeripheral/mtf01p_altitude_test/README.md`
- `testZone/testPeripheral/mtf01p_altitude_test/mtf01p_altitude_test.ino`
  (parser freshness and rate/jitter instrumentation regions)
- `testZone/testPeripheral/bmp388_serial/README.md`
- `testZone/testPeripheral/bmp388_serial/bmp388_serial.ino` (sampling,
  conversion, filter, and calibration regions)
- MicoAir MTF-01P manufacturer specification
- Bosch BMP388 datasheet revision 1.7
- Adafruit BMP3XX driver `performReading()` implementation
- Complete tracked diffs for `main/PIN_MAPPING.md`,
  `testZone/drone_alt_hold_1m/README.md`,
  `testZone/drone_alt_hold_1m/drone_alt_hold_1m.ino`, and
  `testZone/drone_alt_hold_1m/sender_1m/sender_1m.ino`

## FILES_MODIFIED

- `testZone/drone_alt_hold_1m/docs/FLIGHT_CONTROL_ARCHITECTURE.md` (new TASK 0
  architecture baseline)
- `testZone/drone_alt_hold_1m/CODEX_TASK.md` (TASK 0 status only)
- `testZone/drone_alt_hold_1m/CODEX_STATE.md` (TASK 0 checkpoints)
- `testZone/drone_alt_hold_1m/CODEX_STATE.md` (TASK 1 evidence checkpoint)
- `testZone/drone_alt_hold_1m/docs/FLIGHT_ISSUES.md` (new TASK 1 root-cause
  baseline)
- `testZone/drone_alt_hold_1m/CODEX_TASK.md` (TASK 1 status only)
- `testZone/drone_alt_hold_1m/CODEX_STATE.md` (TASK 1 checkpoints and handoff)
- `testZone/drone_alt_hold_1m/docs/SENSOR_FUSION_ANALYSIS.md` (new TASK 2
  pre-mutation analysis)
- `testZone/drone_alt_hold_1m/docs/FLIGHT_CONTROL_ARCHITECTURE.md` (TASK 2
  target architecture and implementation boundary)
- `testZone/drone_alt_hold_1m/CODEX_STATE.md` (TASK 2 analysis checkpoint)
- `testZone/drone_alt_hold_1m/drone_alt_hold_1m.ino` (TASK 2 bounded scheduling,
  timestamp, and diagnostic telemetry implementation)
- `testZone/drone_alt_hold_1m/sender_1m/sender_1m.ino` (matching telemetry v4
  schema and diagnostic output)
- `testZone/drone_alt_hold_1m/CODEX_STATE.md` (TASK 2 implementation checkpoint)
- `testZone/drone_alt_hold_1m/docs/TAKEOFF_LANDING_DESIGN.md` (TASK 3
  pre-mutation audit, state machine, parameter ledger, test and rollback plan)
- `testZone/drone_alt_hold_1m/CODEX_STATE.md` (TASK 3 analysis checkpoint)
- `testZone/drone_alt_hold_1m/drone_alt_hold_1m.ino` (TASK 3 transition phases,
  takeoff timing/ramp fixes, landing touchdown and motor-ramp state machine)
- `testZone/drone_alt_hold_1m/sender_1m/sender_1m.ino` (decode/print Task 3
  transition subphase without changing packet size)
- `testZone/drone_alt_hold_1m/CODEX_STATE.md` (TASK 3 implementation checkpoint)
- `testZone/drone_alt_hold_1m/README.md` (Task 3 takeoff/landing behavior,
  phases, telemetry, troubleshooting, and validation checklist)
- `testZone/drone_alt_hold_1m/docs/FLIGHT_CONTROL_ARCHITECTURE.md` (Task 3
  state-machine, implementation boundary, and verification results)
- `testZone/drone_alt_hold_1m/docs/TAKEOFF_LANDING_DESIGN.md` (implementation
  and evidence-limited verification status)
- `testZone/drone_alt_hold_1m/docs/TILTED_TAKEOFF_DESIGN.md` (Task 4
  pre-mutation root-cause audit, bounded design, parameter ledger, validation,
  and rollback plan)
- `testZone/drone_alt_hold_1m/CODEX_STATE.md` (Task 4 analysis checkpoint)
- `testZone/drone_alt_hold_1m/drone_alt_hold_1m.ino` (Task 4 ground attitude
  capture/leveling, abort monitor, limited mixer, and authority handover)
- `testZone/drone_alt_hold_1m/CODEX_STATE.md` (Task 4 implementation checkpoint)
- `testZone/drone_alt_hold_1m/README.md` (Task 4 supported calibration workflow,
  ground limits, abort behavior, troubleshooting, and staged test sequence)
- `testZone/drone_alt_hold_1m/docs/FLIGHT_CONTROL_ARCHITECTURE.md` (Task 4
  ground/profile/handover architecture and verification results)
- `testZone/drone_alt_hold_1m/docs/TILTED_TAKEOFF_DESIGN.md` (Task 4 final
  implementation/model/build status and retained hardware UNKNOWNs)
- `testZone/drone_alt_hold_1m/CODEX_TASK.md` (TASK 4 status only)
- `testZone/drone_alt_hold_1m/CODEX_STATE.md` (Task 4 completion checkpoint)
- `testZone/drone_alt_hold_1m/drone_alt_hold_1m.ino` (Task 5 command/snapshot/
  finite/resource/ESC-idle robustness implementation)
- `testZone/drone_alt_hold_1m/sender_1m/sender_1m.ino` (Task 5 transmit-pending
  callback ordering and late-callback robustness)
- `testZone/drone_alt_hold_1m/CODEX_STATE.md` (Task 5 resume and implementation
  checkpoints)
- `testZone/drone_alt_hold_1m/docs/ROBUSTNESS_AUDIT.md` (Task 5 findings,
  bounded implementation, models/builds, evidence limits, and rollback gate)
- `testZone/drone_alt_hold_1m/README.md` (Task 5 operator-facing robustness and
  remaining runtime UNKNOWNs)
- `testZone/drone_alt_hold_1m/docs/FLIGHT_CONTROL_ARCHITECTURE.md` (Task 5
  synchronization, fallback, allocation, and verification architecture)
- `testZone/drone_alt_hold_1m/CODEX_TASK.md` (TASK 5 status only)
- `testZone/drone_alt_hold_1m/CODEX_STATE.md` (Task 5 test/build/completion
  checkpoints and Task 6 handoff)
- `testZone/drone_alt_hold_1m/docs/FINAL_SYSTEM_AUDIT.md` (Task 6 complete
  severity/type matrix, pre-mutation boundary, change ledger, final unresolved
  issues, and verification results)
- `testZone/drone_alt_hold_1m/drone_alt_hold_1m.ino` (Task 6 fail-closed MPU
  readback, idempotent landing/failsafe entry, configuration signature, and
  corrected comments)
- `testZone/drone_alt_hold_1m/README.md` (Task 6 source-value reconciliation,
  operator-facing bug-fix behavior, and unresolved evidence boundary)
- `testZone/drone_alt_hold_1m/docs/FLIGHT_CONTROL_ARCHITECTURE.md` (Task 6
  documentation reconciliation and full-system audit revision)
- `testZone/drone_alt_hold_1m/CODEX_TASK.md` (TASK 6 status only)
- `testZone/drone_alt_hold_1m/CODEX_STATE.md` (Task 6 audit, implementation,
  test/build/completion checkpoints and Task 7 handoff)
- `testZone/drone_alt_hold_1m/docs/PARAMETER_HANDBOOK.md` (Task 7 exact-value,
  exact-symbol, observability, safe-step, and UNKNOWN reference)
- `testZone/drone_alt_hold_1m/docs/TEST_PLAN.md` (Task 7 staged prerequisites,
  configuration/telemetry capture, acceptance, abort, rollback, and evidence
  procedure)
- `testZone/drone_alt_hold_1m/README.md` (Task 7 document links, current config
  identity, and sender-output evidence boundary)
- `testZone/drone_alt_hold_1m/CODEX_TASK.md` (TASK 7 status only)
- `testZone/drone_alt_hold_1m/CODEX_STATE.md` (Task 7 final audit and project
  completion checkpoint)

Pre-existing uncommitted flight-code, sender, README, pin-map, Fritzing, and
untracked project changes were inspected and preserved; TASK 0 has not changed
flight behavior.

- TASK 10 modified `drone_alt_hold_1m.ino`, `README.md`,
  `docs/FLIGHT_CONTROL_ARCHITECTURE.md`, `docs/PARAMETER_HANDBOOK.md`,
  `docs/TEST_PLAN.md`, `CODEX_TASK.md`, and `CODEX_STATE.md`. The sender source
  and telemetry schema were not changed; its existing pre-task dirty changes
  were preserved.

## CONFIRMED_FINDINGS

- Active airframe target: ESP32 Classic / Arduino `ESP32 Dev Module`, using the
  Arduino core's FreeRTOS facilities.
- Flight source of truth is the single-file sketch
  `testZone/drone_alt_hold_1m/drone_alt_hold_1m.ino`; the paired ground station
  is `sender_1m/sender_1m.ino`.
- The main sketch schedules MPU6050 reads at 1000 Hz and executes one control
  tick after four samples (nominal 250 Hz). `dt` is measured from `micros()` and
  constrained to 0.5--50 ms; missed sample periods are skipped rather than
  replayed.
- Explicit core-0 tasks are altitude acquisition/fusion (priority 2, nominal
  1 ms service loop), ESP-NOW packet validation (priority
  `configMAX_PRIORITIES-1`, queue driven), telemetry (priority 1, 10 Hz), and
  OLED (priority 1, 4 Hz). BMP388 and OLED share I2C1 behind a mutex.
- The Arduino `loop()` path performs MPU read, attitude estimation, state
  transitions, altitude/position controllers, mixer, and ESC write. Its actual
  core is not pinned by this sketch; ESP32 Arduino normally places `loopTask`
  on core 1, so this remains a runtime/build-setting verification item rather
  than an unconditional code fact.
- MPU6050 is configured for 1 kHz output, DLPF register setting `0x03`,
  +/-500 deg/s gyro and +/-4 g accelerometer on a dedicated 400 kHz I2C bus.
- Attitude uses accel-weighted Madgwick plus filtered gyro rates. Pitch/roll
  use cascaded angle-to-rate controllers; yaw uses integrated gyro heading and
  a yaw-rate PID because there is no compass.
- MTF01P Micolink frames arrive on UART1 at 115200 baud. Firmware parses message
  `0x51`, rejects checksum/status/strength/range/stale data, applies tilt correction,
  median-5 plus EMA height filtering, and estimates vertical speed by
  regression over 15 range samples plus a low-pass filter. The external MTF
  publish frequency is not enforced or measured in the sketch and is UNKNOWN
  until runtime logs/measurement confirm it.
- BMP388 is configured for 50 Hz ODR, pressure oversampling 8x and IIR
  coefficient 7; reads are attempted no more often than every 22 ms (about
  45.5 attempts/s). Successful sample frequency and I2C latency are not logged
  and remain UNKNOWN.
- Altitude fusion aligns BMP altitude to MTF slowly, gates innovation at
  0.45 m, normally blends 90/10 percent height and 70/30 percent vertical
  speed (MTF/BMP), falls back to either valid source, and coasts for up to
  600 ms when both are stale.
- Horizontal hold uses MTF flow gated by range/status/quality/freshness,
  zero-bias calibration, median/EMA filtering, position integration, a
  position-to-velocity P stage, and velocity PI stages that command limited
  pitch/roll setpoints.
- Altitude control is cascaded altitude-P to acceleration-limited vertical
  velocity target to velocity PI, followed by asymmetric correction limits and
  a two-way throttle slew limiter.
- The X-frame mixer prioritizes pitch/roll, preserves bounded yaw headroom,
  shifts collective throttle into the feasible motor interval, publishes
  saturation flags for next-cycle anti-windup, and writes all four LEDC outputs
  from one function at 200 Hz PWM.
- Flight states are DISARMED, ARMED_IDLE, TAKEOFF, ALT_HOLD, LANDING, FAILSAFE,
  and TRIPPED. Pre-arm requires sender link, both altitude sensors, completed
  calibration, near-ground/level/still conditions, and no vibration/clip.
- Command transport is encrypted unicast ESP-NOW with peer filtering,
  magic/version, CRC-16/CCITT, sequence de-duplication, a nonblocking callback
  queue, 200 ms heartbeats, and a 3 s link watchdog. The TASK 0 telemetry
  baseline was version 3/208 bytes. TASK 2 upgraded both active sketches to
  version 4/248 bytes at 10 Hz, and TASK 3 repurposes the compatible reserved
  byte as the internal transition phase.
- Failsafes cover link loss, stale MTF, excessive altitude, flight/takeoff/
  landing timeouts, loss of both altitude sources, sustained attitude error,
  invalid attitude, MPU read failure, and explicit KILL. Controlled faults
  enter landing; severe faults enter latched TRIPPED and ramp motors down.
- Static inspection found `takeoffElapsedS` is reset and checked but never
  incremented, so its no-rise and takeoff-timeout checks have no advancing
  timebase in the current baseline. This was documented for later analysis and
  not changed in TASK 0.
- `groundRangeM` is measured during altitude calibration but current control
  altitude subtracts the fixed 0.12 m installation offset rather than that
  measured value. This is documented as current behavior without yet judging
  correctness.
- The supplied flight rows span multiple intermediate firmware/parameter
  revisions rather than one reproducible binary: observed altitude correction
  reaches -100/+131 although the current limits are -45/+70; observed flow
  tilt reaches 7 degrees although the current limit is 3.5 degrees; the report
  describes a 200 Hz controller while current code schedules 250 Hz. No
  firmware hash or configuration snapshot accompanies a flight.
- Historical flight behavior is confirmed severe: vertical speed reached
  -0.967 to +0.893 m/s, height cycled from near ground to 1.388 m, commanded
  motors repeatedly hit limits, lateral speed reached about 1.04 m/s, and the
  last landing ended with roll exceeding 150 degrees after impact.
- Gross sensor staleness is not supported in the displayed oscillation rows:
  range/barometer ages are commonly below about 22 ms and source remains
  `FUSED`. This does not establish measurement accuracy because raw range,
  barometer altitude, innovations, and estimator weights are not logged.
- A historical landing/failsafe bounce is confirmed: one row at 0.094 m and
  -0.790 m/s retained throttle 1580 and was followed by a rebound above 1 m.
  Current landing clamps are only statically present; no post-change flight
  validates them.
- A current takeoff regression is confirmed: `takeoffElapsedS` is reset and
  checked but never incremented. The tracked diff removed the increment while
  editing takeoff behavior, disabling no-rise and timeout progress.
- Current takeoff altitude demand is not gated by confirmed lift: after the
  open-loop ramp reaches hover throttle, the 1 m setpoint proceeds even while
  `takeoffLiftConfirmed` remains false. Together with the dead timer, this is a
  strong current ground-transition/slingshot risk.
- Current MTF acceptance is effectively binary at strength 10. A sample at
  strength 11 receives the same fixed fusion weighting as a strong sample;
  logs show strength 11--28 during the largest high-altitude excursions.
- Current flow confidence reaches full authority at quality 70, not quality
  110 as its nearby comment states. Lowering the acceptance threshold from the
  historical 80 to 40 and reducing gains are static changes without current
  flight validation.
- Current horizontal position corrections remain active in LANDING and
  FAILSAFE and are retained for 0.35 s after flow becomes unusable. This is a
  strong contributor candidate for lateral touchdown/flip risk, but the logs
  do not isolate it from pre-existing oscillation and impact dynamics.
- Commanded mixer saturation is confirmed by repeated motor floor/cap values
  and very large motor spreads. Physical motor/ESC imbalance, altitude
  integral windup, derivative noise, and axis/sign errors are not identifiable
  from the available telemetry because controller components, setpoints,
  integrators, achieved collective, and saturation flags were not printed.
- MTF01P manufacturer capability is 100 Hz output, 2 cm dead zone, 12 m under
  stated target/light conditions, 2 cm accuracy from 0.1--2 m at stated 90%
  reflectance, and optical flow above 8 cm/more than 60 lux. Installed rate,
  accuracy, latency, and surface/light envelope remain UNKNOWN.
- BMP388 at pressure 8x/temperature 1x has a datasheet conversion time of
  18.69 ms typical and 21.53 ms maximum. The Adafruit `performReading()` path
  selects forced mode per call, so the configured 50 Hz ODR does not establish
  actual sample rate; successful cadence and installed read latency remain
  UNKNOWN.
- Current fusion uses binary MTF trust and fixed weights, while the available
  data cannot justify a numeric strength-confidence map or new filter gains.
  A confidence-aware alpha-beta/complementary observer is the selected future
  structure; numeric correction gains are deferred to bench/replay evidence.
- Current single-writer seqlock keeps the 250 Hz control path lock-free. The
  main scheduling risks are indefinite AUX-mutex waits, OLED bus ownership, and
  BMP work postponing UART drain/publication on core 0.
- Task 2 implementation preserves that ownership: all new altitude diagnostics
  are written only by `altitudeTask`, copied in its existing snapshot, and
  observed by the control loop without locks. Control timing counters are owned
  by the Arduino loop/control context.
- Telemetry v4 is statically asserted at 248 bytes on both drone and sender,
  below the 250-byte ESP-NOW payload limit; sender receive storage is 250 bytes.
- TASK 3 confirms `takeoffElapsedS` has no increment and therefore cannot drive
  either current takeoff abort.
- TASK 3 confirms TAKEOFF overwrites the 1050 us ARMED_IDLE collective with
  1350 us at entry, bypassing `TAKEOFF_RAMP_US_PER_S` for a 300 us step.
- TASK 3 confirms reaching hover without lift currently enables the altitude
  trajectory/integrator while the frame may remain ground-constrained.
- TASK 3 confirms landing entry can step its internal/output slew state down to
  hover and that high-speed threshold crossing does not latch a no-increase
  touchdown phase.
- TASK 3 implementation preserves telemetry v4 at 248 bytes by assigning the
  former reserved byte to `transitionPhase`; phase values 1--8 cover required
  takeoff and landing subphases.
- Pre-lift aborts now enter per-motor MOTOR_RAMP_DOWN. Airborne landing retains
  controlled descent/ground approach, then latches touchdown and removes
  controller/mixer authority before final disarm.
- TASK 4 confirms that ground `iGate=0` prevents new integral accumulation but
  leaves full pitch/roll P/D output available up to 120 us per axis.
- TASK 4 confirms the normal mixer can raise collective to preserve a low-side
  correction, allowing a corner motor to run hundreds of microseconds above a
  low takeoff base even while the base itself slews at 40 us/s.
- TASK 4 confirms TAKEOFF resets the cascaded setpoint filters to zero and
  immediately targets trim; a nonzero reported surface attitude therefore has
  no bumpless reference capture.
- TASK 4 confirms angle boost is applied during ground phases and that no
  ongoing pre-lift angle/rate abort exists below the general 45 deg error trip.
- Boot attitude offsets conflate sensor mounting bias and surface tilt. The
  supported Task 4 procedure must calibrate on a known level surface before
  moving to a mild launch tilt; true earth-level attitude after boot-on-tilt is
  unobservable from current data and remains UNKNOWN.
- Task 4 implementation keeps its independent captured target after
  `updatePositionHold()` clears suppressed horizontal state, preventing that
  helper from overwriting the ground/transfer attitude profile.
- The ground mixer never shifts collective upward to preserve a low motor and
  bypasses angle boost. Its cap is relative to the actual Task 3 throttle ramp;
  the normal mixer is called unchanged outside ground/transfer.
- TASK 4 preserves telemetry v4 at 248 bytes and changes no sender schema. The
  flight config signature is `0xA41F0401` so logs can distinguish this binary.
- Hardware earth-level attitude remains unknowable if calibration occurs on a
  tilted surface because the boot offsets conflate surface and mounting angle;
  Task 4 deliberately does not invent a mounting correction.

## HYPOTHESES

- Cascaded range EMA/regression/vertical-speed/fusion filtering may add enough
  phase lag to amplify vertical overshoot, but raw-versus-filtered traces and
  timestamps are absent; this is strongly suspected, not confirmed.
- Enabling OLED on the same core and I2C/mutex path as the altitude task may
  add current BMP/MTF service jitter. The recorded flights appear to predate
  that change and sender output omits loop/jitter metrics, so this is a current
  architectural risk rather than a logged cause.
- Weak MTF return may arise from surface reflectivity, installation geometry,
  protective film, vibration, or sensor configuration. The low strengths are
  evidence; their physical cause is presently unknown.
- Barometer drift or source-transition discontinuity is possible but not
  demonstrated by `src=FUSED`; separate range/barometer/innovation telemetry
  is required.

## TESTS

- TASK 10 compensation model: **PASS**. Scale is monotonic from 0 at 1050 us
  to 15 us at 1430 us (maximum 0.0395 us compensation change per 1 us base
  change); the uncompensated mean collective is preserved and the hover pair
  is front/rear 1445/1415 us.
- TASK 10 exhaustive ground-mixer model: **PASS** for 625 combinations of five
  throttles and pitch/roll/yaw demands -120/-60/0/+60/+120 us with the CG
  feed-forward included; every output remained in `[1050, base+50] us`.
- TASK 10 profile/timeout model: **PASS**. Conservative takeoff sequence model
  is 15.77 s <20 s; nominal landing through confirmation/final ramp is 10.15 s
  <24 s. The 4 s no-rise bound remains unchanged.
- TASK 10 warning-enabled Arduino builds (`esp32:esp32:esp32`): **PASS**.
  Flight controller uses 981887 bytes flash/49588 bytes RAM; sender uses 889956
  bytes flash/45792 bytes RAM. No compiler warning was emitted.
- TASK 10 current-document/static and `git diff --check` gates: **PASS** for
  config `0xA41F1001`, CG constants/application, gentle profile values, matching
  timeouts, unchanged telemetry v5/248-byte schema and no whitespace error.

- Static source inspection: PASS for locating every required subsystem and
  nominal timing constant.
- Git preservation audit: PASS; no pre-existing modification was reset or
  overwritten.
- `arduino-cli version`: PASS (`1.5.2-rc.1`).
- `arduino-cli core list` / library-environment probe: BLOCKED by the managed
  sandbox denying access/writes below the user's Arduino15 data directory.
  No firmware compile was claimed.
- Architecture completion-gate cross-check: PASS; all four gate conditions are
  explicitly assessed in section 12 of `FLIGHT_CONTROL_ARCHITECTURE.md`.
- New-document whitespace check: PASS; no whitespace errors.
- Automated final TASK 0 manifest check: PASS for platform, tasks/cores,
  frequencies/UNKNOWNs, dependency map, controllers, state/takeoff/landing,
  failsafes, logging, parameter map, four gate assertions, TASK 0 DONE, TASK 1
  TODO, and `CURRENT_TASK: TASK_1`.
- TASK 1 log extraction: PASS; all 63 data rows were parsed and grouped by
  flight/time segment to cross-check height, vertical speed, throttle,
  correction, motor limits, flow quality/strength, and tilt extrema.
- TASK 1 source correlation: PASS for current altitude, flow, fusion, takeoff,
  landing, failsafe, mixer, telemetry, timing, and parameter paths.
- TASK 1 issue-schema check: PASS; all 12 issue records contain all seven
  required evidence/root-cause fields.
- TASK 1 topic-coverage check: PASS for parameter, algorithm, sensor,
  estimator, timing, mixer/motor, altitude oscillation/overshoot, attitude,
  throttle/motor behavior, drift, sensor/filter delay, saturation, windup,
  derivative noise, and `dt`.
- `docs/FLIGHT_ISSUES.md` whitespace check: PASS.
- TASK 1 completion gate: PASS; root causes and uncertainties are evidence
  classified, source/parameters are mapped, recommendations are non-blind, and
  every issue has a safe verification method.
- TASK 2 pre-read: PASS; AGENTS/state/TASK 2 and both complete TASK 0/1
  baselines reread; current status/diff matches the preserved worktree hashes.
- TASK 2 analysis gate: PASS; sensor characteristics/failures/stale behavior,
  rate/latency UNKNOWNs, filter/IMU interaction, strategy comparison, selected
  estimator equations/states, dual-core ownership/priority/races/blocking,
  instrumentation, verification, and rollback are documented.
- Fresh Arduino CLI build (`esp32:esp32:esp32`): PASS for the flight controller
  in `.tmp_task2_compile_20260909_01/drone`; 975095 bytes flash (74%) and
  49532 bytes global RAM (15%).
- Fresh Arduino CLI build (`esp32:esp32:esp32`): PASS for `sender_1m` in
  `.tmp_task2_compile_20260909_01/sender`; 889068 bytes flash (67%) and
  45784 bytes global RAM (13%).
- The temporary fresh-build directory was removed after artifact timestamps
  and size results were verified; no generated build products remain in the
  worktree.
- TASK 2 telemetry schema/packet check: PASS; drone and sender have the same 81
  packed fields in the same order, both use version 4, and the computed/static
  size is 248 bytes (within the 250-byte ESP-NOW limit).
- TASK 2 ownership/control-path check: PASS; `altitudeShared` has one writer,
  its reader retries at most four times, and no semaphore/queue wait or
  `portMAX_DELAY` exists in the `controlTick()`/`loop()` region.
- TASK 2 timeout check: PASS; BMP AUX acquisition is bounded to 2 ms, OLED uses
  zero-wait, and AUX/MPU I2C timeouts are 10/5 ms.
- TASK 2 whitespace check: PASS for the implementation/checkpoint files and
  for all tracked diffs. A wider untracked-tree scan found only preserved
  pre-existing issues in `AGENTS.md` (no final newline) and
  `FLIGHT_LOGS_ANALYSIS.md` (trailing spaces on lines 193--194); they were not
  changed because they are outside the Task 2 regression boundary.
- TASK 2 final documentation/coverage gate: PASS for all 22 checked topics,
  including sensor characteristics, timing/range/failures/staleness, filtering
  and IMU interaction, fusion choice, core scheduling/ownership/races/blocking,
  implementation boundary/results, retained runtime UNKNOWNs, staged tests,
  and rollback conditions.
- TASK 3 state/timer static check: PASS; all eight transition phases are
  present, takeoff elapsed time advances, both abort clocks are reachable,
  TAKEOFF starts from current throttle, and confirmed-liftoff gating exists.
- TASK 3 schema/packet check: PASS; drone/sender retain the same 81-field,
  248-byte telemetry v4 layout and use the former reserved byte for phase.
- TASK 3 no-control-lock and tracked-whitespace checks: PASS.
- TASK 3 representative numerical ramp simulation: PASS; touchdown and final
  motor commands were monotonic non-increasing for starts from 1000 to 1780 us,
  and final ramps reached 1000 us.
- Fresh Task 3 Arduino CLI build with warnings enabled (`esp32:esp32:esp32`):
  PASS for drone in `.tmp_task3_compile_20260909_01/drone`; 976771 bytes flash
  (74%) and 49556 bytes global RAM (15%).
- Fresh Task 3 sender build with warnings enabled and the same FQBN: PASS in
  `.tmp_task3_compile_20260909_01/sender`; 889308 bytes flash (67%) and 45784
  bytes global RAM (13%).
- Final Task 3 source/schema gate: PASS; both sketches still have the same 81
  fields and 248-byte telemetry-v4 layout, including `transitionPhase`.
- Final Task 3 control-path/ownership/timeout gate: PASS; the control region has
  no semaphore/queue wait, `altitudeShared` retains one writer, AUX waits remain
  bounded at 2 ms/zero-wait, and AUX/MPU I2C timeouts remain 10/5 ms.
- Final Task 3 regression simulation: PASS for touchdown/final monotonicity
  from 1000, 1050, 1350, 1430, 1580, and 1780 us, ending at 1000 us; no legacy
  TAKEOFF_START assignment or landing `min(current, hover)` entry clamp remains.
- Final warning-enabled fresh Arduino CLI build (`esp32:esp32:esp32`): PASS for
  the flight controller in `.tmp_task3_compile_20260909_02/drone`; 976771 bytes
  flash (74%) and 49556 bytes global RAM (15%). Artifact timestamp verified as
  2026-09-09 14:27:55 local time.
- Final warning-enabled fresh sender build with the same FQBN: PASS in
  `.tmp_task3_compile_20260909_02/sender`; 889324 bytes flash (67%) and 45784
  bytes global RAM (13%). Artifact timestamp verified as 2026-09-09 14:29:03
  local time.
- Both Task 3 temporary build directories were path-verified inside the
  workspace and removed after artifact results/timestamps were recorded; no
  generated Task 3 build products remain in the worktree.
- Final Task 3 documentation/whitespace gate: PASS; design ledger records every
  new/semantic old-new parameter with evidence, expected effects, risks, and
  required observation; README/architecture match implementation; tracked
  `git diff --check` is clean. No compiler warning was emitted.
- TASK 3 completion gate: PASS for the required takeoff and landing phase
  sequence, no throttle jump, ground integrator/setpoint gating, controlled
  ascent/capture, abort/failsafe routing, touchdown detection, bounded motor
  ramp-down, parameter documentation, schema safety, and final dual compile.
- TASK 4 exhaustive ground-mixer model: PASS for 500 combinations spanning
  throttle 1050--1430 us and pitch/roll/yaw requests -120--+120 us; all outputs
  remained within [1050, requested base+50] us.
- TASK 4 target-profile model: PASS for captures -8, -5, 0, +5, +8 deg; target
  slope never exceeded 3 deg/s and converged to trim.
- TASK 4 transfer model: PASS for constant endpoint combinations 1050--1780 us;
  alpha progressed 0--1 across 125 nominal 250 Hz ticks/0.50 s, endpoints were
  exact, and the largest modeled per-tick interpolation step was 6 us.
- Task 4 tracked whitespace check after implementation: PASS.
- TASK 4 final source invariant check: PASS; ground mixer contains neither
  angle boost nor normal-mixer base shift, uses the base+50 us cap, ground and
  transfer paths are explicit, sustained abort routes through `startLanding`,
  and normal mixer/angle boost remains the final airborne branch.
- TASK 4 final schema/ownership/control-path gate: PASS; drone/sender retain 81
  matching fields/248 bytes, `altitudeShared` retains one writer, and no
  semaphore/queue wait exists in the control region.
- Fresh warning-enabled Arduino CLI build (`esp32:esp32:esp32`): PASS for the
  Task 4 flight controller in `.tmp_task4_compile_20260909_01/drone`; 979267
  bytes flash (74%) and 49588 bytes global RAM (15%). Artifact timestamp was
  verified as 2026-09-09 14:50:57 local time.
- Fresh warning-enabled sender build with the same FQBN: PASS in
  `.tmp_task4_compile_20260909_01/sender`; 889324 bytes flash (67%) and 45784
  bytes global RAM (13%). Artifact timestamp was verified as 2026-09-09
  14:52:03 local time.
- No compiler warning was emitted. The Task 4 build directory was path-verified
  inside the workspace and removed; no generated Task 4 build products remain.
- TASK 4 documentation/parameter gate: PASS; every new or phase-semantic value
  has old/new/unit/subsystem/reason/evidence/positive/negative/required flight
  observation, and README/architecture reflect the code and level-calibration
  constraint.
- TASK 4 completion gate: PASS for root-cause/reference audit, ground integrator
  anti-windup, bounded mixer/motor behavior, capture and limited leveling,
  liftoff detection reuse, bumpless authority transfer, abort/time limits,
  staged validation/rollback, static/model gates, and fresh dual compile.
- TASK 5 deterministic command model: PASS for empty publication, occupied
  non-KILL preservation, KILL preemption, producer-before/after-claim ordering,
  and associated payload coherence.
- TASK 5 sender pending model: PASS for callback-before-return,
  callback-after-return, immediate send failure, and delayed callback. A delayed
  callback leaves the accepted packet pending and warns instead of permitting a
  late callback to clear a newer packet.
- TASK 5 altitude fault model: PASS for stable seqlock commit, four-collision
  fallback retaining the previous coherent snapshot, finite/NaN/+Inf/-Inf
  classification, invalid fused source fallback, and exact existing >100 ms
  snapshot, <=600 ms coast, and >800 ms MTF-failsafe boundaries.
- TASK 5 static robustness gate: PASS for atomic command CAS/exchange and KILL
  priority, absence of the old plain clear, temporary snapshot candidate,
  local failed-read counter, control-side sanitizer, quaternion/PID/attitude/
  tilt/fusion finite guards, sender atomic pending state, ESC initialization
  before altitude-task allocation, AUX/OLED degradation, no control-path RTOS
  wait, one altitude snapshot writer, and no invented deadline trip.
- TASK 5 Task 3/4 numerical regression: PASS for 625 ground-mixer combinations,
  3 deg/s target slew/convergence, 0.50 s transfer endpoints (6 us largest
  modeled tick), and monotonic landing ramps from 1000--1780 us to 1000 us.
- TASK 5 telemetry schema gate: PASS; both sketches retain the same 81 typed
  fields in the same order and static 248-byte/250-byte-limit assertions.
- TASK 5 tracked source/schema whitespace check: PASS. Line-ending conversion
  notices are informational and no whitespace error was reported.
- Fresh warning-enabled Task 5 Arduino CLI build (`esp32:esp32:esp32`): PASS
  for the flight controller in `.tmp_task5_compile_20260909_01/drone`; 980395
  bytes flash (74%) and 49588 bytes global RAM (15%). Artifact timestamp was
  verified as 2026-09-09 18:56:22 local time.
- Fresh warning-enabled sender build with the same FQBN: PASS in
  `.tmp_task5_compile_20260909_01/sender`; 889368 bytes flash (67%) and 45792
  bytes global RAM (13%). Artifact timestamp was verified as 2026-09-09
  18:57:29 local time.
- Neither warning-enabled build emitted a compiler warning. The Task 5 build
  directory was path-verified inside the workspace and removed; no generated
  Task 5 build product remains in the worktree.
- A later final drone rebuild after making the snapshot-failure counter
  `volatile` emitted `-Wvolatile` for `volatile++`. This blocks the final gate
  despite a successful link. The increment was replaced by an explicit relaxed
  atomic add; fresh warning-enabled builds of both final sketches are required
  before completion.
- Final fresh warning-enabled Arduino CLI builds after the atomic-add fix:
  **PASS** in `.tmp_task5_compile_20260909_03`; flight controller 980391 bytes
  flash (74%)/49588 bytes RAM (15%), sender 889368 bytes flash (67%)/45792
  bytes RAM (13%). No compiler warning was emitted. Final binary timestamps
  were verified as 2026-09-09 19:09:15 and 19:10:20 local time. The failed-gate
  `_02` and passing `_03` build directories were path-verified inside the
  workspace and removed; neither remains in the worktree.
- Final exact-source Task 5 gate: **PASS** after the warning fix for command and
  sender interleavings, seqlock/non-finite/staleness faults, 81-field/248-byte
  schema, one altitude writer, no control-path wait, no deadline trip, all Task
  3/4 phase/mixer/ramp tokens, 625 ground-mixer cases, monotonic landing ramps,
  documentation coverage, temporary-artifact removal, and whitespace.
- TASK 6 pre-mutation full-system audit: **PASS**; complete current drone,
  sender, README, and all six design/audit documents were reviewed. The final
  unresolved list is severity/type classified in `FINAL_SYSTEM_AUDIT.md`.
- TASK 6 MPU/landing models: **PASS** for readback transaction/value failure and
  exact success, repeated LAND idempotence, no FAILSAFE downgrade, one-way
  LANDING-to-FAILSAFE promotion, and preservation of active phase/timer.
- TASK 6 Task 3--5 regressions: **PASS** for command/KILL interleavings,
  coherent snapshot fallback, non-finite classification, exact 100/600/800 ms
  boundaries, 625 ground-mixer combinations, monotonic touchdown/final ramps,
  3 deg/s target slew, and 0.50 s transfer endpoints with a 6 us maximum model
  tick.
- TASK 6 static/schema/documentation/whitespace gate: **PASS** for one altitude
  writer, no control-path blocking primitive, atomic mailbox and coherent
  snapshot retention, all Task 3/4 phase/motor paths, 81 matching ordered
  telemetry fields and 248-byte assertions, reconciled README values, and no
  `git diff --check` error beyond informational LF/CRLF notices.
- Final fresh Task 6 `--clean --warnings all` Arduino CLI builds with FQBN
  `esp32:esp32:esp32`: **PASS**; flight controller 980567 bytes flash/49588
  bytes RAM, sender 889368 bytes flash/45792 bytes RAM. No compiler warning was
  emitted. Binary timestamps were verified at 2026-09-09 19:42 local time; the
  path-verified `.tmp_task6_compile_20260909_01` directory was removed.
- Final exact-source Task 6 gate: **PASS**. Config signature is `0xA41F0601`;
  no gain, physical flight parameter, Task 3/4 bound, or deadline policy was
  changed, and no hardware or flight result is claimed.
- TASK 7 test-plan structure check: **PASS** for staged static/replay,
  props-off, guarded/restrained, netted vertical, takeoff/landing, horizontal,
  tilted-launch and bounded-fault procedures, with prerequisites, run identity,
  acceptance, abort, rollback and evidence boundaries.
- TASK 7 exact-source documentation check: **PASS**. The 128 handbook rows all
  have the required seven fields; all 201 parsed uppercase source symbols are
  represented except `SDA`, `SCL`, and `WHO`, which are fragments of source
  header/log text rather than declarations. Hard-coded takeoff, flow-loss,
  position-clamp, airborne-floor, ARM, config, and packet-size values are
  explicitly documented.
- TASK 7 telemetry/schema check: **PASS**; drone and sender retain 81 fields in
  identical order and both assert exactly 248 bytes and no more than 250 bytes.
  The handbook/test plan require packet-level or traceable diagnostic capture
  for fields omitted by sender `STATUS`.
- TASK 7 static/model regression: **PASS**; the control region contains no
  queue/semaphore/display/AUX/UART blocking path, `altitudeShared` retains one
  writer and a bounded four-try reader, the atomic command mailbox and sender
  pending protocol remain present, and the clean target-slew model passed 10
  pitch/roll cases at a maximum 0.012 degree step (3.0 deg/s). Task 6 already
  recorded 625 ground-mixer cases, monotonic landing ramps, and 25 transfer
  cases with a 6 us maximum tick against these exact unchanged sources.
- TASK 7 cross-task manifest: **PASS** for every stated Task 0--7 document and
  implementation requirement. TASK 1 retains exactly 12 issue records and 12
  instances of every required per-issue field. All nine README document links
  resolve, and final Task 7 whitespace/diff checks pass.
- TASK 7 build disposition: no rebuild required. TASK 7 changed documentation
  only; current FC/sender timestamps (2026-09-09 19:30:34/18:43:23) predate the
  exact-source warning-enabled clean Task 6 builds recorded at 19:42. Those
  builds passed with no warning at 980567/49588 bytes (FC) and 889368/45792
  bytes (sender), config `0xA41F0601`.
- TASK 7 completion gate: **PASS** for the software/document scope. Hardware,
  props-off, restrained, netted, and flight stages remain pending; completion
  is not an airworthiness or flight-safety claim.
- TASK 9 height-normalization model: **PASS** at 0.10, 0.25, 0.50, 1.00 and
  1.50 m for equal physical velocity; conversion before median/EMA remains in
  m/s and does not reference `ALT_TARGET_M`.
- TASK 9 controller model: **PASS** for direct signed metric position error,
  `0.40/s` position gain and desired velocity-vector magnitude <=0.35 m/s
  across axial, 3-4-5 diagonal and saturated displacement cases.
- TASK 9 static gate: **PASS** for `ALT_TARGET_M=0.50`, sender `targetCm=50`,
  calibration-origin odometry outside the airborne-only path, retained
  estimate on flow loss, 0.20 s recovery confirm, fresh-flow/valid-origin ARM
  and TAKEOFF admission, absence of horizontal-radius symbols, matching
  telemetry v5 and 248-byte assertions.
- Final Task 9 clean warning-enabled Arduino builds (`esp32:esp32:esp32`):
  **PASS**. Flight controller uses 981539 bytes flash/49588 bytes RAM; sender
  uses 889956 bytes flash/45792 bytes RAM. No compiler warning was emitted.
- Task 9 documentation/link/whitespace gate: **PASS**. README, architecture,
  issue baseline, tilted-launch constraint, handbook and test plan match
  `cfg=0xA41F0901`; the corrected ledger and multi-height acceptance plan are
  in `docs/POSITION_HOLD_HEIGHT_NORMALIZATION.md`. Generated build artifacts were
  path-verified inside the workspace and removed.

## OPEN_ISSUES

- The reported forward battery/CG direction is sufficient to choose the mixer
  sign, but no arm length, mass offset, thrust curve, restrained motor trace or
  current-build flight log quantifies the required moment. The +15 us pitch
  compensation is a bounded starting hypothesis, not a proven final value.
  Mechanical battery relocation remains preferred; otherwise validate and tune
  by at most 2 us per restrained build before any free flight.

- No battery-voltage/current failsafe, independent hardware kill, ESC/RPM
  feedback, motor-failure detection, redundant IMU, or persistent onboard log
  exists in this target. These are unresolved architectural gaps, not inferred
  healthy from a build.
- Runtime MTF publish frequency, successful BMP sample rate/I2C latency,
  Arduino `loopTask` affinity/priority, CPU clock, task execution time, and task
  jitter remain UNKNOWN pending build/runtime instrumentation.
- No current-build flight log exists, so all current mitigations remain static
  and unvalidated until later staged tests.
- Task 4 has no hardware validation. Surface/mount reference error, safe
  installed tilt/rate envelope, ground torque from 50 us, target slew adequacy,
  false-abort rate, crossfade disturbance rejection, and actual per-tick motor
  continuity remain UNKNOWN pending the documented staged tests.
- Runtime confidence mapping, numeric observer tuning, and any estimator/filter
  replacement remain evidence-gated. TASK 2 documented the target architecture
  and added the measurements needed; it intentionally did not invent values
  without bench/replay data.
- Horizontal accuracy is not yet quantified: optical flow is relative, yaw has
  no compass reference, and displacement during a flow outage is unobservable.
  Per-height velocity accuracy, long-duration drift and actual closed-loop
  position error remain UNKNOWN until independent-position tests pass.

## NEXT_ACTION

Execute `docs/TEST_PLAN.md` sequentially and include the props-off, independent
0.25/0.50 m translation and multi-height checks in
`docs/POSITION_HOLD_HEIGHT_NORMALIZATION.md`. Begin Stage B only after the listed
personnel, site, capture, independent power-disconnect, `cfg=0xA41F1001`/
telemetry-v5 identity and rollback prerequisites are met. Before release,
execute the added D3 CG checks: four equal outputs at 1050 us, correct M1/M2
raised pair as collective grows, bounded +15/-15 us pitch split at hover and
no worsening nose-down response. Prefer moving the battery to the thrust centre;
if software compensation is still needed, tune by no more than 2 us per signed
restrained build. Do not advance a stage on missing evidence or reinterpret
project completion as permission for free flight.

## LAST_CHECKPOINT

TASK 10 is COMPLETE for the authorized software/document scope. Current identity
is `cfg=0xA41F1001`, telemetry v5/248 bytes. The controller now separates the
reported forward-CG static moment from PID feedback, commands a 0 deg level
target, ramps +15 us pitch feed-forward from zero at armed minimum to hover,
and uses the slower documented vertical profiles. Models, static/document/
whitespace checks and clean warning-enabled dual builds passed. Hardware and
flight validation are still pending; no airworthiness, exact CG correction or
physical-position-accuracy claim is made, and unrelated dirty-worktree changes
remain preserved.

## INTERRUPTION

- 2026-09-09: the required GPT-5.6 Sol HIGH worker hit its usage limit while
  resuming TASK 2 validation. No downgrade or substitute model was used.
- No additional source mutation or validation result was produced by the
  failed resumed turn.
- Resume after the reported quota reset (12:36 PM) from the existing
  `NEXT_ACTION`: validate the complete TASK 2 implementation diff, run the
  static gates, compile both sketches, update documents/state, and only then
  decide whether TASK 2 can be marked DONE.
