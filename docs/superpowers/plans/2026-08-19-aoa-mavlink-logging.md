# AOA MAVLink Logging Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stream complete AOA sensor, filter, controller, motor-command, and vehicle-response diagnostics to a computer without an SD card.

**Architecture:** A fixed driver-side event queue preserves every FIT event. The AOA mode emits dynamic DataFlash records at the sensor rate and calls a post-mixer hook from `Rover::set_servos()` so the motor record contains final scaled and PWM outputs. Mission Planner receives a 5 Hz three-line diagnostic snapshot while the full binary records use the MAVLink logger backend.

**Tech Stack:** ArduPilot C++, AP_Logger `WriteStreaming`, MAVLink remote logging, GoogleTest/Waf, ChibiOS `dimah743`.

**Spec:** `docs/superpowers/plans/2026-08-19-aoa-mavlink-logging-design.md`

## Global Constraints

- Do not change AOA control, filter, safety, mixer, or sign behavior.
- Preserve all existing uncommitted changes.
- Full-rate logging is enabled only by `AOA_DEBUG=1`.
- `STATUSTEXT` output is limited to 5 Hz and three lines per snapshot.
- Do not claim wheel-speed or motor-feedback measurement without encoders or ESC telemetry.

---

### Task 1: Preserve every FIT diagnostic event

**Files:**
- Modify: `libraries/AP_AOA/AP_AOA_ALX_FIT.h`
- Modify: `libraries/AP_AOA/AP_AOA_ALX_FIT.cpp`
- Test: `libraries/AP_AOA/tests/test_aoa_fit.cpp`

**Interfaces:**
- Produces: `GateResult`, `DiagnosticEvent`, `get_diagnostic_event(DiagnosticEvent&)`, `diagnostic_drop_count()`, and an optional accepted event sequence from `get_raw_data(...)`.

- [x] **Step 1: Write failing tests** for accepted and rejected event metadata, event ordering, reason codes, and fixed-queue overwrite counting.
- [x] **Step 2: Run `./waf test --tests test_aoa_fit`** and confirm the new API tests fail because the diagnostics interface is absent.
- [x] **Step 3: Implement the fixed-size event queue** and record accepted, gate-rejected, decode-error, and heartbeat events without dynamic allocation.
- [x] **Step 4: Re-run `./waf test --tests test_aoa_fit`** and confirm all FIT tests pass.

### Task 2: Expose controller and filter state without changing behavior

**Files:**
- Modify: `libraries/AP_AOA/AP_AOAPID.h`
- Modify: `libraries/AP_AOA/AP_AOAPID.cpp`
- Modify: `libraries/AP_AOA/AP_AOAFollowControl.h`
- Modify: `libraries/AP_AOA/AP_AOAFollowControl.cpp`
- Modify: `libraries/AP_AOA/AP_AOAFilter.h`
- Test: `libraries/AP_AOA/tests/test_aoa_control.cpp`
- Test: `libraries/AP_AOA/tests/test_aoa_filter.cpp`

**Interfaces:**
- Produces: `AP_AOAPID::Info`, `AP_AOAFollowControl::Diagnostics`, PID-info getters, and `AOAKalmanFilter::jump_candidate_count()`.

- [x] **Step 1: Write failing tests** using hand-calculated P/I/D values and observable controller/filter states.
- [x] **Step 2: Run the AOA control and filter tests** and confirm failures are due to missing diagnostics APIs.
- [x] **Step 3: Populate read-only snapshots** at existing calculation points, clearing them during existing reset paths.
- [x] **Step 4: Re-run the AOA control and filter tests** and confirm prior control-behavior tests remain green.

### Task 3: Emit full-rate DataFlash records and low-rate MP summaries

**Files:**
- Modify: `Rover/mode.h`
- Modify: `Rover/mode_aoafollow.cpp`
- Modify: `Rover/Steering.cpp`

**Interfaces:**
- Consumes: FIT diagnostic events, filter/controller snapshots, `SRV_Channels` post-mixer output, AHRS, and battery monitor.
- Produces: `AOAR`, `AOAS`, `AOAF`, `AOAC`, `AOAP`, `AOAM`, `AOAV`, and `ModeAoafllow::write_actuator_log()`.

- [x] **Step 1: Add logger-format compile coverage** by declaring the exact label and format strings in the production calls, keeping each within AP_Logger's 4/16/64-character limits.
- [x] **Step 2: Drain FIT events and write sensor/filter/controller records** once per event or accepted sample when `AOA_DEBUG=1`.
- [x] **Step 3: Add the post-mixer hook** after `g2.motors.output()` and gate `AOAM`/`AOAV` to one record per consumed sample, including zero-output rejection records.
- [x] **Step 4: Replace named-float spam with a compact 5 Hz, three-line `STATUSTEXT` snapshot.**
- [x] **Step 5: Build SITL** with `./waf rover` to catch logger format and integration errors.

### Task 4: Verify target build and handoff settings

**Files:**
- Verify only: all files above

**Interfaces:**
- Produces: tested firmware and exact field-test parameter instructions.

- [x] **Step 1: Run all AOA unit tests** and record pass/fail counts.
- [x] **Step 2: Build `dimah743` Rover firmware** and record the artifact path.
- [x] **Step 3: Inspect the final diff** for unrelated changes and verify every design requirement is represented.
- [x] **Step 4: Report field settings:** `AOA_DEBUG=1`, `LOG_BACKEND_TYPE=2`, `LOG_MAV_BUFSIZE=16` or `32`, `LOG_DISARMED=1` for bench sessions, MAVLink2, and a 921600-baud USB/high-speed telemetry path.
