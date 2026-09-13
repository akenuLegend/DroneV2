# Task 6 — Full-system audit and retuning decision

Audit date: 2026-09-09

Status: complete for static/model/build scope; hardware and flight validation pending

## 1. Scope and evidence boundary

This audit reviews the complete current drone and sender sketches after Tasks
1--5, plus the README and all design/audit documents. It covers estimator and
sensor validity, attitude/rate/altitude/position controllers, mixer and motor
authority, takeoff/landing/failsafe transitions, inter-core ownership, task
timing, radio commands, telemetry, initialization, and documentation.

Evidence is source inspection, the historical mixed-version flight report,
and deterministic host models. There is still no prop-off, restrained, or
flight log from configuration `0xA41F0501`. Installed timing, sensor behavior,
motor response, and current closed-loop stability remain **UNKNOWN**. A build
or model pass is not hardware evidence.

## 2. Pre-mutation findings

No remaining CRITICAL defect was identified by static inspection. That is not
a claim that the aircraft is safe to fly freely; the HIGH evidence gaps below
still block such a conclusion.

| Severity | Type | Finding | Task 6 disposition |
|---|---|---|---|
| HIGH | BUG | `mpuInit()` ignores failure of its four configuration readbacks and returns success even when the read values differ. The estimator then applies fixed +/-4 g and +/-500 deg/s scale factors that may not match the device configuration. | Require all readbacks to succeed and all values to match. Failure uses the existing fatal initialization path; ESC output is already initialized to 1000 us. |
| HIGH | BUG | A new `LAND` command while already LANDING or FAILSAFE re-enters `startLanding(false)`: it can downgrade FAILSAFE to LANDING, clear the failsafe result, reset the 18 s landing timer, and restart controller state. Repeated unique LAND commands can therefore postpone the timeout. | Make landing entry idempotent. A normal LAND never downgrades/restarts an active landing; a safety request may promote LANDING to FAILSAFE without resetting phase or timers. |
| HIGH | POSSIBLE ISSUE REQUIRING FLIGHT DATA | The current Tasks 2--5 firmware has no matching hardware/flight dataset. Historical flights show vertical oscillation, saturation, lateral pendulum motion, rebound, and flip, but span incompatible binaries. | Do not retune from mixed-version data. Require the staged props-off, guarded/restrained, then netted low-height plan already documented. |
| HIGH | TUNING ISSUE / POSSIBLE ISSUE REQUIRING FLIGHT DATA | `HOVER_DEFAULT=1430 us`, thrust linearization, motor matching, mixer authority, vertical gains/limits, and all Task 3/4 transition bounds remain unvalidated on the installed battery/prop/frame. | Preserve values. Identify thrust/feed-forward and achieved authority from current-signature logs and a guarded stand before tuning. |
| HIGH | TUNING ISSUE / POSSIBLE ISSUE REQUIRING FLIGHT DATA | The stacked range/velocity/fusion filters have unknown end-to-end delay; MTF strength receives binary trust and fixed fusion weight. Current altitude stability and source-recovery transients are unknown. | Preserve filters, weights, gates, and timeouts. Collect raw/separate sensor timing, innovation, strength, and reference-motion data before implementing the Task 2 target observer. |
| HIGH | TUNING ISSUE / POSSIBLE ISSUE REQUIRING FLIGHT DATA | Flow axes/sign, error versus quality/range/surface, confidence mapping, velocity damping, and position integration remain unvalidated. The current formula reaches full authority at quality 70. | Preserve parameters and keep position tuning after vertical-loop validation. Correct prose to the actual code and require props-off sign tests first. |
| HIGH | ARCHITECTURAL ISSUE | The target has no battery-voltage/current failsafe, independent hardware kill, ESC/RPM feedback, motor-failure detection, redundant IMU, or persistent flight recorder. | Record explicitly as unresolved. Hardware selection and new safety policy exceed a bounded static Task 6 fix. |
| MEDIUM | OBSERVABILITY ISSUE / POSSIBLE ISSUE REQUIRING FLIGHT DATA | MTF/BMP rates and latency, task service gaps, UART backlog, loop affinity/priority, control jitter, CPU load, and stack margins have no runtime evidence. Existing telemetry measures several timing quantities but no justified control-deadline trip threshold exists. | Preserve timing counters and do not invent a deadline threshold. Collect prop-off/restrained distributions and stack high-water data before policy changes. |
| MEDIUM | ARCHITECTURAL ISSUE / POSSIBLE ISSUE REQUIRING FLIGHT DATA | MTF parsing, BMP blocking reads, fusion, and snapshot publication share one core-0 task; telemetry and radio also use core 0. The practical interference margin is unknown. | Keep the Task 2 single-writer design until measurements justify splitting producers. |
| MEDIUM | TUNING ISSUE / POSSIBLE ISSUE REQUIRING FLIGHT DATA | Clearance subtracts the fixed 0.12 m sensor-to-bottom geometry while calibration's measured `groundRangeM` is diagnostic only. The installed geometry and ground/touchdown thresholds are unverified. | Preserve semantics; physically measure mounting offset and validate ground/touchdown behavior before changing values. |
| MEDIUM | ARCHITECTURAL ISSUE / POSSIBLE ISSUE REQUIRING FLIGHT DATA | Gyro-only yaw drifts and rotates the horizontal hold reference; there is no absolute position or heading source. | Retain the bounded short-duration mission assumption and quantify drift before expanding position-hold duration. |
| MEDIUM | OBSERVABILITY ISSUE | The 248-byte telemetry packet is near the 250-byte legacy ESP-NOW limit and lacks raw sensor/innovation, requested-versus-achieved collective, per-axis saturation causes, stack margin, and persistent provenance storage. | Do not change the stable Task 2 schema in this fix. Use a future separate diagnostic/logging channel rather than overflowing the control telemetry packet. |
| LOW | BUG (documentation) | README values disagree with authoritative code: hover default, flow quality/range gates, hold confirmation, minimum hold altitude, max tilt, tilt slew, and low-quality authority. Two source comments also describe 1400 us or quality 110 although code uses 1430 us and reaches full flow authority at quality 70. | Reconcile prose/comments to current code without changing flight behavior. |
| LOW | OBSERVABILITY ISSUE | Altitude seqlock read-collision count remains local, and lifetime maxima/counters saturate in the compact telemetry representation. | Defer schema expansion; stale-publication handling is already safe and the limitation is documented. |

## 3. Cross-subsystem conclusions

- Estimator validity and fallback: Task 5 finite guards and coherent-snapshot
  fallback are present. Numeric confidence, delay, and recovery behavior remain
  evidence-gated rather than statically "fixed."
- Controllers: rate/attitude and altitude loops include output limits,
  conditional integration, slew limiting, and finite-input defenses. Their
  gains cannot be justified from the mixed historical log.
- Mixer/authority: normal airborne priority, Task 4 ground cap, integrator
  gating, and per-motor transfer are intact. Physical thrust/CG/motor matching
  is unknown.
- Position hold: loss decay and re-acquisition are bounded, but axis mapping,
  confidence and gains require staged tests. It must remain suppressed during
  the first vertical validation.
- Failsafe/state machine: Task 3/4 transition and monotonic motor-ramp paths are
  intact; the re-entrant LAND defect above is the remaining confirmed state
  bug selected for correction.
- Execution/ownership: control remains lock-free; altitude, telemetry, and
  OLED snapshots each retain one writer. No new task split or deadline trip is
  justified without runtime evidence.

## 4. Authorized bounded mutation

Only these source changes are authorized by this checkpoint:

1. fail MPU initialization when configuration readback fails or mismatches;
2. make active landing/failsafe entry idempotent while allowing one-way
   promotion from LANDING to FAILSAFE;
3. update the manual configuration signature to identify the Task 6 binary;
4. reconcile inaccurate comments and documentation to the unchanged code.

No controller gain, sensor/fusion weight, validity threshold, timeout,
takeoff/landing numeric bound, position parameter, mixer limit, motor value,
task rate, or deadline policy is authorized for change.

## 5. Parameter/change ledger before implementation

| Parameter/behavior | Old | New | Unit | Reason/evidence | Expected positive effect | Possible negative effect | Required hardware/flight observation |
|---|---:|---:|---|---|---|---|---|
| `FLIGHT_CONFIG_SIGNATURE` | `0xA41F0501` | `0xA41F0601` | identifier | Distinguish Task 6 bug-fix firmware from Task 5 in logs. | Prevents mixed-binary evidence. | Log tooling must recognize the new value. | Confirm signature before every staged test. |
| MPU configuration acceptance | Log mismatch, return success | Require successful exact readback | boolean behavior | Fixed scale factors require the programmed full-scale and filter/divider configuration. | Prevents control with an unverified IMU scale/configuration. | A transient readback fault now stops initialization; this is fail-safe but may expose marginal wiring. | Props-off: verify correct startup and deliberate read/config failure stops with motors at 1000 us. |
| Repeated LAND during active landing | Reinitialize/downgrade/reset timeout | Idempotent; optional one-way FAILSAFE promotion | state behavior | Confirmed re-entry path defeats state provenance and the bounded landing clock. | Preserves monotonic phase/timer and failsafe status. | A repeated command no longer restarts landing setup. | Host model plus props-off repeated LAND/FAILSAFE sequence. |

## 6. Planned final gate

- deterministic MPU readback and repeated-LAND/FAILSAFE transition models;
- Task 3/4 takeoff, mixer, transfer, touchdown, and monotonic motor regressions;
- command/snapshot finite/staleness models from Task 5;
- static one-writer, no-control-lock, schema/order/size, configuration and
  documentation-value checks;
- tracked whitespace/diff check;
- fresh warning-enabled Arduino CLI builds of both exact final sketches.

Any compiler warning, schema drift, new control-path wait, transition/motor
regression, or failed model blocks Task 6 completion. Hardware and flight gates
remain pending even if this static/model/build gate passes.

## 7. Implementation and final verification

The pre-mutation checkpoint above was completed before source changes. Both
authorized bug fixes are now implemented:

- `mpuInit()` requires four successful readbacks and exact values before it
  returns success. A failed transaction or mismatch returns false to the
  existing fatal startup path, after ESC channels have already been attached
  and written to 1000 us.
- `startLanding()` returns without reinitialization when LANDING/FAILSAFE is
  already active. A normal LAND cannot downgrade FAILSAFE. A new safety event
  may promote LANDING to FAILSAFE while preserving phase, setpoint, motor ramp,
  and the original landing timer.

Source comments, README, and the architecture document now state the actual
1430 us hover default, flow gate 40/50 mm, hold confirmation 0.20 s, minimum
hold altitude 0.10 m, 3.5 deg tilt cap, 8 deg/s tilt slew, and confidence map
from 40% at quality 40 to full authority at quality 70. These are truth-in-
documentation corrections, not retuning. The sender's unused 1400 us payload
initial value is documented and does not alter the drone boot default.

Final verification against the exact source:

- MPU read-success/mismatch truth model and repeated LAND/FAILSAFE transition
  model: **PASS**; active phase/timer are retained and promotion is one-way.
- Task 5 command/KILL interleavings, coherent/colliding snapshot behavior,
  finite/NaN/infinity classification, and exact >100 ms publication,
  <=600 ms coast, and >800 ms MTF failsafe boundary models: **PASS**.
- Task 3/4 regressions: **PASS** for 625 ground-mixer combinations, monotonic
  touchdown/final ramps from representative 1000--1780 us starts, 3 deg/s
  target slew, and 0.50 s transfer endpoints with a 6 us largest modeled tick.
- Static ownership/control gate: **PASS** for one altitude snapshot writer,
  atomic command publish/claim, coherent temporary snapshot, retained phase/
  motor paths, and no blocking primitive in `controlTick()`/`loop()`.
- Schema/documentation/whitespace: **PASS**; drone and sender retain 81 typed
  telemetry fields in identical order, both assert 248 bytes under the 250-byte
  limit, stale README-value searches are clean, and `git diff --check` reports
  no whitespace error (only the repository's informational LF/CRLF notices).
- Fresh warning-enabled Arduino CLI builds (`esp32:esp32:esp32`, `--clean`,
  `--warnings all`): **PASS**; flight controller 980567 bytes flash/49588 bytes
  global RAM, sender 889368 bytes flash/45792 bytes global RAM, with no compiler
  warning. Final binary timestamps were checked at 2026-09-09 19:42 local time.
  The path-verified Task 6 build directory was removed afterward.

Config signature `0xA41F0601` identifies this result. No gain or physical
flight parameter changed. The unresolved HIGH/MEDIUM/LOW items in section 2
are the final unresolved-issues list; none is converted into a hardware-success
claim by these gates.
