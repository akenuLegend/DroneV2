# DRONE DEVELOPMENT PIPELINE

Do tasks sequentially.

Do NOT begin TASK N+1 until TASK N completion criteria are satisfied.

---

# TASK 0 — SYSTEM BASELINE

STATUS: DONE

Objectives:

- Map the entire flight-controller architecture.
- Identify:
  - MCU/platform
  - RTOS/tasks if present
  - core affinity
  - control frequencies
  - sensor frequencies
  - estimator architecture
  - PID loops
  - mixer
  - takeoff
  - landing
  - altitude hold
  - failsafes
  - logging

Produce a dependency map of:

Sensor
→ filtering
→ estimator
→ controller
→ mixer
→ motor

Do not modify flight behavior during this task.

Completion gate:

- architecture understood
- relevant source files located
- loop frequencies identified or marked UNKNOWN
- parameter locations identified

---

# TASK 1 — FLIGHT LOG ROOT-CAUSE ANALYSIS

STATUS: DONE

Read:

testZone/drone_alt_hold_1m/FLIGHT_LOGS_ANALYSIS.md

Also inspect the source code controlling the parameters mentioned there.

Goals:

- analyze observed flight behavior
- identify parameter-selection mistakes
- distinguish:
  - parameter problem
  - algorithm problem
  - sensor problem
  - estimator problem
  - timing problem
  - mixer/motor problem

For each issue provide:

Evidence
Likely root cause
Confidence
Relevant code
Relevant parameter
Recommended change
How to verify

Pay special attention to:

- altitude oscillation
- overshoot
- attitude disturbance
- throttle excursions
- motor imbalance
- estimator drift
- delayed sensor response
- PID saturation
- integral windup
- derivative noise
- incorrect dt
- filtering latency

Update:

docs/FLIGHT_ISSUES.md

Do not blindly modify parameters until root causes are understood.

---

# TASK 2 — MTF01P + BMP388 + DUAL-CORE ARCHITECTURE

STATUS: DONE

Analyze how adding:

- MTF01P
- BMP388

changes the existing system.

Determine:

- measurement characteristics
- update frequency
- latency
- effective operating range
- failure conditions
- stale-data handling
- filtering requirements
- interaction with existing IMU/estimator

Design altitude estimation architecture.

Evaluate suitable fusion strategy such as:

- complementary fusion
- weighted fusion
- Kalman-based estimator
- confidence/range-dependent fusion

Do not choose an algorithm solely because it is sophisticated.
Prefer deterministic and robust behavior.

MTF01P should primarily contribute where its measurement is trustworthy.
BMP388 should provide longer-term altitude information where appropriate.

Analyze two-core scheduling.

Explicitly identify:

- critical real-time tasks
- non-critical tasks
- CPU affinity
- priorities
- expected frequencies
- communication method
- race conditions
- blocking paths

Goal:

keep attitude/motor-control timing deterministic while using both sensors.

Produce:

docs/SENSOR_FUSION_ANALYSIS.md
docs/FLIGHT_CONTROL_ARCHITECTURE.md

Only implement architectural changes after analysis.

---

# TASK 3 — TAKEOFF AND LANDING REDESIGN

STATUS: DONE

Current important fact:

Throttle approximately 1300 does NOT imply airborne hover.
The drone may still be on the ground around this command region.

Do not assume 1300 is true airborne hover throttle.

Inspect real throttle/motor behavior before selecting values.

Redesign takeoff such that:

DISARMED
→ ARMED_IDLE
→ SPOOL_UP
→ GROUND_TRANSITION
→ CONTROLLED_ASCENT
→ ALTITUDE_CAPTURE
→ ALT_HOLD

Design landing:

ALT_HOLD
→ CONTROLLED_DESCENT
→ GROUND_APPROACH
→ TOUCHDOWN_DETECTION
→ MOTOR_RAMP_DOWN
→ DISARM

Avoid a single throttle jump.

Consider:

- throttle ramp / slew limit
- vertical velocity target
- altitude target trajectory
- acceleration limits
- ground-effect region
- integrator handling
- altitude capture smoothing
- touchdown detection

Goal:

smooth climb to approximately 1 m without aggressive boost,
and smooth landing without dropping.

Document all tunable constants.

---

# TASK 4 — TAKEOFF ON SLIGHTLY TILTED SURFACE

STATUS: DONE

Analyze why the current system may heavily boost one motor when starting
from a slightly tilted surface.

Inspect:

- attitude error during ground contact
- attitude PID integral accumulation
- mixer saturation
- motor minimum command
- throttle headroom
- anti-windup
- integrator reset/freeze
- spool-up logic
- ground-state detection

Prevent the controller from accumulating large corrective commands while
the drone is physically constrained by the ground.

Once sufficient thrust exists for control authority, transition smoothly
to normal attitude stabilization.

Evaluate:

- integrator gating
- anti-windup
- mixer desaturation
- correction limiting during spool-up
- attitude target initialization
- bumpless controller transfer

Goal:

drone can launch from a mildly tilted surface without one motor receiving
a destructive/aggressive transient command.

---

# TASK 5 — FLIGHT-CONTROL ROBUSTNESS PASS

STATUS: DONE

Treat:

attitude stability
altitude stability
position stability

as higher priorities than aggressive responsiveness.

Audit:

- estimator validity
- control-loop deadlines
- state transitions
- PID saturation
- anti-windup
- actuator authority
- mixer
- filtering
- sensor confidence
- failsafes
- invalid readings
- stale samples
- NaN/Inf handling
- task overruns
- watchdog behavior
- timing jitter

Do not merely increase filters or decrease gains.
Determine root cause before tuning.

---

# TASK 6 — FULL SYSTEM AUDIT AND RETUNING

STATUS: DONE

Review the entire system after Tasks 1–5.

Look for remaining issues that previous tasks did not address.

For every issue classify:

CRITICAL
HIGH
MEDIUM
LOW

Separate:

BUG
ARCHITECTURAL ISSUE
TUNING ISSUE
OBSERVABILITY ISSUE
POSSIBLE ISSUE REQUIRING FLIGHT DATA

Modify parameters only where justified.

Run available tests/builds.

Produce final unresolved-issues list.

---

# TASK 7 — PARAMETER HANDBOOK

STATUS: DONE

Create:

docs/PARAMETER_HANDBOOK.md

For every important flight parameter provide:

Parameter
Current value
Unit
Source file
Approximate code location / symbol
Subsystem
Meaning
Effect of increasing
Effect of decreasing
Symptoms if too high
Symptoms if too low
Safe direction for tuning
Recommended tuning step
What log variables to observe

Organize parameters by:

- motors / ESC
- mixer
- attitude
- rate PID
- altitude
- vertical velocity
- MTF01P
- BMP388
- sensor fusion
- filtering
- takeoff
- landing
- task frequencies
- failsafes

Also create:

docs/TEST_PLAN.md

with a repeatable real-flight tuning procedure.

---

# FINAL COMPLETION

Only mark PROJECT COMPLETE when Tasks 0–7 are DONE.

Final response must summarize:

- root causes found
- architecture changed
- files changed
- parameters changed
- remaining uncertainties
- recommended real-flight test sequence

---

# TASK 8 — CALIBRATION-ORIGIN 0.50 M POSITION HOLD (SUPERSEDED)

STATUS: SUPERSEDED BY TASK 9 — the 0.50 m requirement was incorrectly applied
to horizontal radius instead of altitude.

- Convert each accepted MTF01P flow sample to m/s with its current vertical
  range before median/EMA filtering.
- Keep downstream horizontal velocity, position and controller gains in SI
  units so behavior is not tied to the fixed 1.00 m altitude target.
- Retain one horizontal origin from completion of initial calibration; do not
  silently reset/relock it after flow loss.
- Express the horizontal objective as radial normalization around 0.50 m while
  preserving the old small-signal axial response and existing authority.
- Limit desired horizontal velocity by vector magnitude.
- Update matching sender telemetry identity/status, technical documentation,
  parameter ledger, test plan and persistent state.
- Pass static/model/schema/whitespace checks and warning-enabled builds of both
  sketches. Hardware/flight validation remains required and cannot be inferred
  from build success.

---

# TASK 9 — CORRECT 0.50 M ALTITUDE AND ZERO-ERROR POSITION TARGET

STATUS: DONE

- Set the automatic takeoff and altitude-hold target to `0.50 m` clearance.
- Remove the invented 0.50 m horizontal acceptance radius from control,
  ARM/TAKEOFF admission, telemetry text and documentation.
- Hold horizontal position at the calibration origin `(0,0)` as accurately as
  validated sensing and stable controller authority permit.
- Preserve per-sample optical-flow scaling by current measured height so the
  same SI-unit position controller remains valid at future commanded heights.
- Update build identity, sender default target, docs, tests and persistent
  state, then rebuild both sketches with warnings enabled.

---

# TASK 10 — FORWARD-CG COMPENSATION AND GENTLE TAKEOFF/LANDING

STATUS: DONE

- Correct the reported nose-down/high-forward-speed takeoff caused by the
  battery being forward of the thrust centre.
- Do not increase attitude Kp/Kd blindly to mask a near-static load moment.
- Remove nonzero angle trim as a CG workaround and introduce bounded,
  collective-scaled pitch-axis mixer feed-forward for the known forward load.
- Preserve ground mixer caps, motor mean command, feedback PID anti-windup,
  transition continuity and all existing emergency/touchdown behavior.
- Reduce post-liftoff vertical setpoint speed, velocity/acceleration bounds and
  collective up-slew; reduce landing setpoint/down-slew and update total
  transition timeouts without extending the 4 s ground no-rise exposure.
- Update build identity and current documentation; run compensation/profile
  models, static/whitespace gates, and warning-enabled builds of both sketches.
- Hardware CG moment and the exact compensation remain UNKNOWN until the
  required props-off/restraint/netted sequence passes. Prefer mechanical
  battery relocation over software compensation whenever possible.
