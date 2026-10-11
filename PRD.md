# Stage 1 Product Requirements

This repository implements Stage 1 of a camera-based hand-tracking robotic arm. This specification describes the shipped scope and its acceptance checks.

## Objective

Track one deliberately presented hand with a stationary-base, three-MG995 robotic arm. The base corrects horizontal image error. Shoulder and elbow move in a calibrated coupled relationship to correct vertical image error while approximately preserving camera pitch. The camera is rigidly mounted to the final link. All joints begin at a verified 90-degree home pose.

## Required behavior

- Detect up to two hands and select one target consistently. Use the average of wrist and four finger MCP landmarks as the palm center.
- Filter target jitter and reject isolated jumps. Hold the previous target through brief detection loss, then stop motion and reacquire only after consistent detections.
- Apply configurable deadbands and proportional incremental control with measured loop time. Keep coupled shoulder/elbow corrections synchronized at limits.
- Keep tracking off at startup; require an operator command to start. Show camera diagnostics, commanded angles, state, limits, and serial status.
- Enforce safe joint limits in both host and Arduino firmware. Confirm limit changes explicitly and store them in EEPROM.
- Smooth firmware output with speed and acceleration limits. On serial timeout or hand loss, cancel motion and hold the current commanded pose.
- Support manual per-joint calibration without camera dependencies. Keep Stage 1 operational with no ToF sensor.

## Acceptance gates

1. Verify wiring, external servo power, common ground, and physical power disconnect.
2. With the arm supported, verify 90/90/90 home and each joint independently at low speed.
3. Confirm direction signs and safe limits; verify firmware rejects a command outside them.
4. Run vision-only diagnostics with no servo movement.
5. Test base tracking, then coupled vertical tracking, at conservative gains.
6. Verify deadband stability, hand-loss HOLD, serial timeout, camera failure, and clean shutdown.
7. Run a supervised 10-minute test with no limit violations. Measure at least 15 processed frames per second on the selected host.

Stage 2 distance regulation is outside this repository's implemented scope. `host/arm_tracker/tof.py` is the disabled interface for that future extension.
