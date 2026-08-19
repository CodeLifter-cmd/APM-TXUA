# AOA MAVLink Logging Design

## Objective

Capture every AOA input event and every accepted 40 Hz control update without an SD card, stream the resulting DataFlash records over MAVLink to a computer, and retain a compact Mission Planner message-pane summary for field checks.

## Data path

The FIT driver queues decoded, rejected, malformed, and heartbeat events. `ModeAoafllow` drains those events into `AOAR`, records filter and controller state as `AOAF`, `AOAC`, and `AOAP`, then defers actuator logging until `AP_MotorsUGV::output()` has completed. The deferred hook records post-mixer scaled outputs and PWM in `AOAM`, plus AHRS and battery response in `AOAV`.

`AOA_DEBUG=0` disables this diagnostic traffic. `AOA_DEBUG=1` enables the full-rate binary stream and a 5 Hz, three-line `STATUSTEXT` diagnostic snapshot. The binary stream is intended for `LOG_BACKEND_TYPE=2` and a high-speed MAVLink link; Mission Planner's message pane remains a diagnostic fallback rather than a lossless bulk-data transport.

## Log records

- `AOAR`: event timestamp/sequence, raw FIT fields, and gate result.
- `AOAS`: cumulative decode/gate/heartbeat counters and diagnostic-queue drops.
- `AOAF`: sample period, raw and filtered measurements, body angle, jump-confirmation state, and filter validity.
- `AOAC`: controller errors, output requests, accepted/too-close/deadzone state, and recovery count.
- `AOAP`: distance and angle PID P/I/D/total terms.
- `AOAM`: requested throttle/steering, post-mixer left/right scaled outputs, final PWM, and motor limit flags.
- `AOAV`: AHRS ground speed, yaw rate, horizontal acceleration, battery voltage/current, and current-valid flag.

There is no encoder or ESC telemetry. Therefore `AOAM` proves what the autopilot actually sent to the two motor channels, while `AOAV` is the vehicle-response proxy. It must not be described as measured wheel speed or measured motor torque.

## Safety and performance

- A fixed-size queue avoids allocation in the sensor path and counts any overwritten events.
- Binary records are written only when diagnostics are enabled.
- Actuator records are emitted once per consumed AOA sample after PWM calculation, including zero-output records for filter/control rejection, not at the 400 Hz servo loop rate.
- Message-pane snapshots are limited to 5 Hz and three lines per snapshot.
- No PID, filtering, safety latch, mixer, or motor sign is changed by this feature.

## Validation

Unit tests cover ordered diagnostic events, reject reasons, queue-overrun accounting, PID-term snapshots, controller diagnostics, and filter jump state. SITL and `dimah743` builds validate logger call formats and target compatibility. The first hardware run must still verify end-to-end MAVLink capture and link-loss statistics under the actual 40 Hz sensor load.
