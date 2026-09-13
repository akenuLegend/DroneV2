# TASK 5 — Flight-control robustness audit

## 1. Scope and evidence boundary

This audit covers the current Task 4 firmware only: sensor validity/freshness,
MTF01P/BMP388/MPU6050 loss and recovery, ESP-NOW command loss, cross-core
ownership, task/resource failure, loop deadlines, non-finite values, controller
reset/fallback transitions, and safe land/disarm behavior. It preserves the
Task 2 single-writer lock-free altitude snapshot and the Task 3/4 transition,
ground-mixer, motor-ramp, and authority-transfer invariants.

Evidence in this document is static source inspection and deterministic local
models. There is no current-build hardware or flight evidence. Installed bus
latency, stack high-water marks, scheduler jitter, sensor recovery dynamics,
and physical response to any fault remain **UNKNOWN**.

## 2. Confirmed findings before mutation

| Severity | Classification | Finding and evidence | Bounded response |
|---|---|---|---|
| CRITICAL | cross-core race | `handlePendingCommand()` reads `pendingCommand` and later writes `CMD_HEARTBEAT`; the ESP-NOW task can publish a new command, including KILL, between those operations, and the control write can erase it. `volatile` does not make the read/clear transaction atomic. | Claim one pending command with an atomic exchange. Publish from the single producer with release ordering and preserve KILL priority. No mutex or blocking call enters control. |
| HIGH | sender callback race | Sender sets `sendPending=true` only after `esp_now_send()` returns, so an early callback can clear it first and the later store can leave it stuck. Its 100 ms timeout then clears an accepted send; a late callback can clear the state of a newer packet. | Publish pending before the asynchronous call, atomically clear in the callback or immediate-error path, and warn without timeout-clearing an accepted send. Complete callback loss safely stops sends and lets the drone link watchdog act. |
| HIGH | invalid-value path | Madgwick rejects only a zero/NaN quaternion norm, while attitude safety and pre-arm reject only NaN pitch/roll. Infinity can therefore evade the named checks and make comparisons false. `pid_compute()` also omits infinite setpoints. | Use `isfinite()` for quaternion norm, attitude/rates, pre-arm values, and both PID inputs. A non-finite airborne attitude enters the existing latched TRIPPED motor ramp. |
| HIGH | torn snapshot fallback | The four-try altitude seqlock reader copies directly into the live control snapshot on every try. If all tries collide with the writer, it returns false after potentially leaving a torn value, and the caller ignores the return. | Copy to a temporary candidate and commit only after a stable sequence. Count failures locally; retain the last coherent snapshot, which the existing 100 ms publication watchdog will invalidate. |
| HIGH | estimator invariant | Individual BMP pressure/altitude and flow outputs are checked, but no final finite invariant exists for the fused altitude/vertical speed snapshot. Range tilt compensation can also consume a non-finite cross-core tilt value. | Reject a non-finite tilt before range mutation, reset fusion state if a final result is non-finite, and sanitize control-facing snapshot fields before use. Invalid controlling altitude is treated as no source, leading to the existing hold-then-failsafe-land path. |
| MEDIUM | observability / unresolved policy | Control timing records overruns and clamps `dt` to 50 ms, but an arbitrarily long airborne scheduling stall only resumes the controller; it has no evidence-based deadline safety transition. Motors retain the previous command throughout the stall. | Keep the existing counters and document the gap. Do not invent a trip threshold without measured scheduler/airframe evidence; collect prop-off/restrained timing evidence before selecting one. |
| MEDIUM | resource failure | If the AUX mutex allocation fails, the code silently runs BMP and a later OLED task without mutual exclusion. OLED task creation failure is ignored. | Keep altitude acquisition available, but disable OLED when the mutex is unavailable or its task cannot be created. Report the degradation. No flight parameter changes. |

## 3. Existing defenses retained

- MTF frames require peer protocol structure, checksum, message/length, forward
  sensor time (with reboot handling), status, strength, distance, step
  confirmation, range freshness, and finite/bounded derived flow speed.
- BMP reads are mutex-bounded to 2 ms, require driver success, finite pressure
  in 30--110 kPa, finite derived altitude, and 250 ms freshness.
- Fusion prefers agreeing MTF/BMP data, falls back to either source, coasts for
  at most 600 ms, and publishes from one core through a bounded seqlock.
- Pre-arm requires calibrated IMU/altitude, fresh MTF and BMP, link, level/still
  attitude, vibration and clipping limits. MPU read loss trips after the
  existing consecutive-failure limit.
- Valid ESP-NOW packets require fixed peer, magic/version, CRC, command range,
  and sequence-repeat suppression. Only valid packets refresh the 3 s link
  watchdog. Link loss disarms ARMED_IDLE and starts controlled failsafe landing
  from TAKEOFF/ALT_HOLD.
- Task 3/4 pre-lift abort, controlled landing, blind descent, touchdown latch,
  monotonic motor ramps, ground motor cap, integrator gating, and authority
  crossfade remain unchanged.

## 4. Recovery and fallback assessment

| Event | Current/target behavior | Classification |
|---|---|---|
| MTF invalid/stale | Reject sample; use BMP/coast temporarily; after 800 ms MTF age in TAKEOFF/ALT_HOLD, start failsafe landing. A recovered valid MTF may support landing but does not restore ALT_HOLD automatically. | Confirmed by source; physical recovery transient UNKNOWN. |
| BMP invalid/stale | Reject non-finite/out-of-range sample; continue MTF-only fusion. Pre-arm requires BMP fresh. | Confirmed by source; installed drift/recovery UNKNOWN. |
| Both altitude sources lost | Coast to 600 ms, then no source. Normal flight holds current throttle until MTF failsafe; landing reduces throttle blindly and TRIPPED-latches after 8 s without source. | Confirmed by source; physical blind-land outcome UNKNOWN. |
| Flow lost/invalid | Clear validity/output; position hold slews back to trim and is suppressed during ground/landing/Task-4 transfer. Recovery requires fresh qualifying samples. | Confirmed by source; optical surface envelope UNKNOWN. |
| MPU consecutive read loss | Existing threshold enters TRIPPED, captures actual motor outputs, ramps to idle, and stays reset-latched. | Confirmed by source; exact elapsed time under scheduler stalls UNKNOWN. |
| Command/link loss | Queue overflow drops packets without blocking Wi-Fi; repeated actions/heartbeats provide retries. Drone watchdog acts only on validated packets. | Confirmed; RF loss rate and range UNKNOWN. |

## 5. Parameter ledger

| Parameter | Old | New | Unit | Reason/evidence | Expected positive effect | Expected negative effect | Required flight observation |
|---|---:|---:|---|---|---|---|---|
| `ALTITUDE_SNAPSHOT_STALE_MS` | literal 100 | 100 | ms | Names the existing publication watchdog; no semantic change. | Makes fault-injection/model assertions and future review explicit. | None expected; behavior unchanged. | UNKNOWN: verify publication-loss detection timing on hardware. |
| `FLIGHT_CONFIG_SIGNATURE` | `0xA41F0401` | `0xA41F0501` | identifier | Distinguishes Task 5 robustness firmware in logs. | Prevents mixing Task 4 and Task 5 evidence. | Old analysis tools must accept the new identifier. | Confirm the signature in first telemetry/log before any test. |

No gains, fusion weights, sensor thresholds, takeoff/landing values, position
parameters, mixer limits, or motor values are changed by Task 5.

## 6. Deterministic verification plan

1. Static assertions: atomic command claim/publish; no old read-then-clear;
   temporary seqlock candidate; finite guards; deadline path; OLED resource
   degradation; no control-path semaphore/queue wait.
2. Model/fault injection: enumerate writer-before/after atomic exchange,
   coherent/torn snapshot attempts, finite/NaN/+Inf/-Inf sensor values, source
   staleness boundaries, and existing deadline-counter behavior.
3. Regression checks: telemetry schema remains 81 fields/248 bytes, Task 3/4
   phase paths and ground mixer constants remain present, packet size and
   single-writer ownership hold, and tracked whitespace is clean.
4. Fresh warning-enabled Arduino CLI compile for drone and sender with FQBN
   `esp32:esp32:esp32` in new build directories.

Rollback condition: any new compiler warning, packet/schema drift, control-path
blocking, altered Task 3/4 transition or motor behavior, model failure, or
unexpected source/motor transition blocks Task 5 completion and reverts only
the responsible Task 5 change.

## 7. Implementation and verification result

The bounded responses above are implemented. The drone mailbox now publishes
normal actions only into an empty slot with release CAS, preserves an occupied
action for sender repetition to retry, allows KILL to preempt, and claims once
with an acquire atomic exchange. Associated hover payload is copied while the
mailbox remains occupied. The sender marks pending before `esp_now_send()`,
atomically clears it from the callback/immediate-error path, and does not create
an ambiguous second outstanding send after its warning interval.

Altitude seqlock reads use a temporary candidate and retain the last coherent
snapshot after four collisions. Failed reads are counted locally; the control
side sanitizes the retained/new snapshot and uses the named unchanged 100 ms
publication watchdog. Quaternion normalization, both PID inputs, MTF tilt,
final fusion output, attitude/rates/yaw, altitude, and pre-arm values use finite
invariants. Invalid controlling altitude becomes `ALT_SOURCE_NONE`; invalid
attitude uses the existing latched `TRIPPED` ramp.

ESC channels are attached and commanded to 1000 us before task allocation. A
missing AUX mutex or failed OLED task creation disables OLED and reports the
degraded state; altitude/BMP service remains available with one AUX-bus owner.
No control-loop lock, gain, flight threshold, Task 3/4 transition/motor value,
or deadline trip threshold was added.

Verification results:

- command/sender interleaving models: **PASS**, including producer before/after
  claim, occupied action, KILL preemption, callback before/after return,
  immediate error, and delayed callback;
- altitude fault models: **PASS** for stable/colliding seqlock reads,
  finite/NaN/+Inf/-Inf classification and no-source fallback, and exact existing
  >100 ms snapshot, <=600 ms coast, and >800 ms MTF-failsafe boundaries;
- static ownership/control gate: **PASS** for one altitude writer, no blocking
  RTOS primitive in `controlTick()`, atomic mailbox operations, finite guards,
  resource failure paths, and absence of a deadline-triggered state transition;
- Task 3/4 regression models: **PASS** for 625 ground-mixer combinations,
  0.012 deg nominal per-tick target slew, 0.50 s transfer endpoints with a 6 us
  largest modeled tick, and monotonic landing ramps to 1000 us;
- telemetry/schema/whitespace: **PASS**, with 81 matching typed fields in order,
  248-byte static size under the 250-byte ESP-NOW limit, and clean tracked diff;
- warning-enabled fresh Arduino CLI builds (`esp32:esp32:esp32`): **PASS**;
  flight controller 980391 bytes flash/49588 bytes RAM, sender 889368 bytes
  flash/45792 bytes RAM, with no compiler warning.

The build artifacts were timestamp-checked and removed. Hardware timing,
stack/jitter behavior, sensor recovery dynamics, and physical fault response
remain **UNKNOWN**. Task 5 is complete only as a static/model/build robustness
gate; prop-off, restrained, and low-altitude validation remain mandatory.
