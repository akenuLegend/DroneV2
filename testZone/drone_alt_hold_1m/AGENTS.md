# DRONE PROJECT AGENT RULES

Use GPT-5.6 Sol with HIGH reasoning for ALL work.

Do not downgrade model or reasoning level to save quota.

The AI is fully responsible for:
- reading and understanding the project
- analyzing logs
- finding bugs
- modifying code
- restructuring tasks
- testing/building
- documenting changes
- maintaining progress state
- resuming unfinished work after interruption

Do not ask the user for routine technical decisions.
Make the best engineering decision from code, logs, tests, and evidence.

Priority:

1. flight stability
2. attitude stability
3. altitude stability
4. position stability
5. safe takeoff/landing
6. sensor reliability
7. performance

Never trade deterministic flight-control timing for convenience.

Before every work session:

1. Read AGENTS.md
2. Read CODEX_TASKS.md
3. Read CODEX_STATE.md
4. Inspect git status
5. Inspect git diff
6. Resume exactly from NEXT_ACTION

The repository is persistent memory.
Do not rely on conversation memory.

## CHECKPOINT

Update CODEX_STATE.md frequently.

Always record:
- current task
- completed work
- findings
- files inspected
- files modified
- tests
- remaining issues
- exact NEXT_ACTION

Assume quota can interrupt execution at any moment.

If interrupted:
- preserve all work
- update state if possible
- do not switch model
- stop safely

When execution becomes available again:
- continue from CODEX_STATE.md
- do not restart completed work

## ENGINEERING RULES

Do not blindly tune parameters.

For every important parameter change document:
- old value
- new value
- reason
- expected effect
- possible negative effect
- how to test

Separate:
- confirmed bug
- probable issue
- tuning hypothesis

Inspect especially:
- incorrect dt
- PID saturation
- integral windup
- derivative noise
- filter latency
- sensor latency
- stale sensor data
- blocking I/O
- CPU affinity
- task priority
- race conditions
- mixer saturation
- motor saturation
- throttle headroom
- invalid sensor data
- state transition errors

## DUAL CORE

Keep critical flight-control work deterministic.

Prefer:

CORE A:
- IMU
- attitude estimator
- rate controller
- attitude controller
- mixer
- motor output

CORE B:
- MTF01P
- BMP388
- altitude preprocessing
- telemetry
- logging
- diagnostics

Adapt this based on actual code and measured behavior.

Do not allow slow sensors/logging/telemetry to block the critical control loop.

## SAFETY

Takeoff and landing must be controlled state transitions.

Do not assume throttle 1300 is airborne hover throttle merely because it is currently used near hover logic.

Prevent aggressive single-motor boost during tilted-ground takeoff.

Use anti-windup, saturation handling, integrator control, and smooth controller transitions where justified.

Build/test after meaningful changes.

Continue autonomously until all tasks are complete.