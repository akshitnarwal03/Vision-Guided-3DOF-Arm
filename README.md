# Vision Guided 3DOF Robotic Arm

A camera-guided robotic arm that keeps a detected hand near the center of its view. The project combines real-time hand landmark detection on a host computer with safety-constrained motion control on an Arduino.

The arm has three controlled degrees of freedom:

- **Base yaw** tracks horizontal hand movement.
- **Shoulder pitch** and **elbow pitch** move together to track vertical hand movement while approximately preserving camera pitch.
- A USB camera mounted on the final link provides continuous visual feedback.

Stage 1 implements image-based left/right and up/down tracking. Distance regulation with a Time-of-Flight sensor is reserved for Stage 2.

## Technology stack

| Layer | Technology | Purpose |
|---|---|---|
| Vision | MediaPipe Hand Landmarker Full | Detects up to two hands and returns 21 normalized landmarks per hand |
| Image processing | OpenCV | Captures camera frames and renders the diagnostic interface |
| Host control | Python 3.10–3.12 | Runs selection, filtering, control, configuration, and state supervision |
| Configuration | YAML | Stores camera, serial, controller, and calibrated servo settings |
| Communication | PySerial | Exchanges newline-terminated commands with Arduino at 115200 baud |
| Firmware | Arduino C++ | Enforces limits, smooths movement, stores calibration, and runs the watchdog |
| Actuation | Arduino Servo library | Generates positional PWM commands for three MG995 servos |
| Persistence | Arduino EEPROM | Stores confirmed firmware joint limits with a checksum |
| Verification | Python unittest | Tests control, filtering, configuration, protocol, and state transitions |

The current implementation uses MediaPipe's default CPU delegate. It does not use ONNX Runtime and does not automatically select a GPU.

## System architecture

~~~mermaid
flowchart LR
    CAM[USB camera] --> CV[OpenCV frame capture]
    CV --> MP[MediaPipe Hand Landmarker]
    MP --> SELECT[Target selection]
    SELECT --> FILTER[Jump rejection and EMA]
    FILTER --> ERROR[Normalized image error]
    ERROR --> CTRL[Visual servo controller]
    CTRL --> HOSTLIMIT[Host rate and angle limits]
    HOSTLIMIT --> SERIAL[USB serial protocol]
    SERIAL --> FW[Arduino safety controller]
    FW --> PROFILE[Velocity and acceleration profile]
    PROFILE --> SERVOS[Three MG995 servos]
    SERVOS --> ARM[Camera pose changes]
    ARM --> CAM

    CONFIG[YAML configuration] --> CV
    CONFIG --> FILTER
    CONFIG --> CTRL
    STATE[Runtime state machine] --> CTRL
    STATE --> SERIAL
    EEPROM[EEPROM joint limits] --> FW
~~~

The host performs perception and decides where the arm should move. The Arduino remains the final motion boundary: it validates commands, rejects angles outside stored limits, smooths the requested trajectory, and freezes movement when communication becomes stale.

## Vision and target model

### Hand landmark detection

The project uses Google's **MediaPipe Hand Landmarker Full** float16 task model in video mode. Video mode reuses tracking information between frames, reducing how often the more expensive palm detector must run.

Each detected hand contains 21 normalized landmarks. The controller follows a palm center instead of a fingertip because finger articulation would create unnecessary arm movement. The palm center is the mean of the wrist and four finger MCP joints:

~~~text
P = (L0 + L5 + L9 + L13 + L17) / 5
~~~

Here, Li = (xi, yi) is a normalized landmark coordinate and P = (px, py) is the raw palm center.

### Multiple-hand selection

When two hands are visible:

1. The hand closest to the previous target center remains selected.
2. When no previous target exists, the hand with the highest confidence is selected.
3. A distant replacement hand is ignored until the previous target has been absent for the configured loss threshold.

This gives the target identity persistence and prevents rapid switching when two hands cross.

### Measurement filtering

An exponential moving average reduces landmark jitter:

~~~text
P_filtered[k] = alpha * P_raw[k] + (1 - alpha) * P_filtered[k-1]
~~~

The default alpha is 0.25. Smaller values provide more smoothing but add response delay.

A single displacement larger than the configured jump threshold is rejected. A new position must appear consistently for multiple frames before it is accepted. The filter resets after sustained target loss.

## Visual servo control model

This is an **image-based visual servo system**. It minimizes hand-position error in the camera image rather than reconstructing a global 3D hand position.

### Normalized image error

~~~text
error_x = filtered_hand_x - target_x
error_y = filtered_hand_y - target_y
~~~

Positive error_x means the hand appears to the right of the target. Positive error_y means it appears below the target. Calibrated direction multipliers convert those image errors into the correct physical servo directions.

### Deadband

Small errors inside the center window are treated as zero:

~~~text
error_axis = 0, when abs(error_axis) <= deadband_axis
~~~

This prevents continuous correction caused by landmark noise, servo backlash, and camera vibration.

### Horizontal controller

The base uses proportional incremental control:

~~~text
delta_base = direction_base * gain_base * error_x * dt
~~~

The controller measures dt with a monotonic clock, caps unusually large frame intervals, limits each update, and clamps the resulting target to the calibrated base range.

### Coordinated vertical controller

Shoulder and elbow receive opposing physical rotations so the camera changes height while approximately preserving final-link pitch:

~~~text
vertical_delta = gain_vertical * error_y * dt

delta_shoulder = direction_shoulder * vertical_delta
delta_elbow    = -direction_elbow * vertical_delta * coupling_ratio
~~~

The initial coupling ratio is 1.0 and can be tuned for the real linkage and servo mounting. This is a calibrated local approximation rather than full inverse kinematics.

When either planar joint approaches a limit, the controller scales both increments by the same fraction. This maintains shoulder/elbow coupling instead of independently clipping one joint and abruptly changing camera pitch.

## Commanded motion model

MG995 servos accept position commands but do not return their actual shaft position. The project smooths the **commanded trajectory**; it cannot measure servo lag, backlash, load, a stall, or a joint displaced by hand.

The host first limits the maximum target change per update. The Arduino adds bounded velocity and acceleration. For each joint, it calculates the maximum speed from which it can still stop at the target:

~~~text
v_stop = sqrt(2 * max_acceleration * abs(position_error))
v_desired = sign(position_error) * min(max_speed, v_stop)
~~~

Velocity changes are acceleration-limited:

~~~text
delta_v = clamp(v_desired - v_current,
                -max_acceleration * dt,
                 max_acceleration * dt)
~~~

| Firmware parameter | Default |
|---|---:|
| Motion update period | 20 ms |
| Maximum commanded speed | 35 degrees/s |
| Maximum commanded acceleration | 90 degrees/s² |
| Serial watchdog timeout | 750 ms |

The stopping-speed term causes gradual braking near the target and reduces sudden velocity reversals when new targets arrive.

## Runtime state machine

~~~mermaid
stateDiagram-v2
    [*] --> STARTUP
    STARTUP --> HOMING: configuration and serial valid
    HOMING --> IDLE: home motion complete
    IDLE --> TRACKING: operator presses t
    TRACKING --> IDLE: operator presses t
    TRACKING --> HAND_LOST: loss threshold reached
    HAND_LOST --> TRACKING: stable reacquisition
    TRACKING --> LIMITED: joint reaches limit
    LIMITED --> TRACKING: movement away from limit
    STARTUP --> FAULT: initialization failure
    TRACKING --> FAULT: camera serial or operator stop
    HAND_LOST --> FAULT: camera or serial failure
    LIMITED --> FAULT: camera or serial failure
    FAULT --> IDLE: explicit reset
~~~

## Layered safety model

- Tracking is disabled at startup and begins only after the operator presses t.
- The host sends HOLD after sustained hand loss and stops changing targets.
- Firmware rejects malformed commands and angles outside its stored limits.
- Confirmed joint limits are stored in Arduino EEPROM, independent of the host YAML file.
- A stale serial connection cancels the in-progress trajectory and holds the current commanded angle.
- Normal shutdown sends HOLD; it does not release servo torque automatically.
- Calibration never discovers limits by intentionally driving a servo into a stall.

## Hardware specifications and wiring

| Device | Default signal connection |
|---|---|
| Base MG995 | Arduino D9 |
| Shoulder MG995 | Arduino D10 |
| Elbow MG995 | Arduino D11 |
| USB camera | Host computer USB |

The older `Basic_servo_mapping.ino` sketch in the parent directory uses D3 for the base. If the arm is still wired that way, change `PINS[0]` in `firmware/arm_controller/arm_controller.ino` to `3` and set `servos.base.pin: 3` in `host/config.yaml` before uploading.

Power all three MG995 servos from a regulated external supply appropriate to the actual servo voltage and combined stall current. Do **not** power them from the Arduino 5 V pin. Connect the external supply ground to Arduino ground and all servo grounds. Keep the supply positive rail separate from Arduino 5 V when Arduino is USB powered. Put an easily reachable physical power disconnect on the servo supply.

The camera must be rigidly fixed to the final link. At commanded 90/90/90, the intended pose is centered base, vertical first link, horizontal final link, and camera facing forward. Verify this with the arm supported before any tracking.

MG995 servos have no position feedback to this software. `STATE` reports the last commanded angle, not a measured shaft angle. The acceleration limiter softens changes in PWM commands but cannot compensate for a stall, power loss, or a joint moved by hand while torque is off. Reattaching may cause a jump; support and realign the arm first.

## Install

1. Install Arduino IDE and its standard Servo library. Open `firmware/arm_controller/arm_controller.ino`, check pins, 90-degree home pose, and the conservative 60–120 degree defaults. Set `PULSE_MIN_US` and `PULSE_MAX_US` only after checking the MG995 variant. Select the correct Uno/Nano board and port, then upload.
2. Install Python 3.10–3.12 on Windows. From the `host` directory, create a virtual environment and install dependencies:

   ```powershell
   py -3.12 -m venv .venv
   .\.venv\Scripts\Activate.ps1
   python -m pip install -r requirements.txt
   Copy-Item config.example.yaml config.yaml
   python download_model.py
   ```

   The model downloader retrieves Google's official [MediaPipe Hand Landmarker bundle](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker). The model file is excluded from Git because it is a third-party binary.
3. Set `serial.port` in `host/config.yaml` to the Arduino COM port if auto discovery finds zero or multiple USB serial devices. Set `camera.index` if the camera is not index 0. Start with `mirror_preview: false`.

For calibration alone, install `requirements-calibration.txt`; it does not need OpenCV or MediaPipe.

## Calibrate before tracking

Support the arm, remove nearby objects, and use the physical power disconnect if motion becomes unsafe. Never discover an end stop by intentionally stalling an MG995.

```powershell
cd host
python calibrate_servos.py
```

Use `show`, `home`, `step base +`, `step shoulder -`, and `step elbow +` to move one joint in 5-degree increments. Watch for cable tension, collision, buzzing, and strain. `limits base 65 115` edits a draft limit; `save` asks for explicit confirmation, writes all draft limits to Arduino EEPROM, reads them back, then writes `config.yaml`. Firmware rejects out-of-range `SET` commands independently of the host.

Set `direction JOINT 1` or `direction JOINT -1` to indicate which command sign reduces image error. With the camera looking forward, test a hand slightly right of center: the correct base direction moves the camera so the hand approaches the center. For a hand below center, the correct shoulder direction and opposite elbow command should move the hand upward in the image. Start with low gains, check one axis at a time, and reverse a direction value if image error grows.

Firmware starts with 60–120 degree limits. To expand them, first inspect the mechanism manually, then enter and explicitly save a wider range. This makes the firmware and YAML agree. `DETACH` releases torque and may let the arm fall. `ATTACH` writes the last commanded pose; it cannot know whether a detached joint moved.

## Run

First test vision without servo movement:

```powershell
cd host
python main.py --vision-only
```

Then connect the Arduino and run tracking:

```powershell
python main.py
```

The program homes slowly and stays idle. The overlay shows the raw and filtered palm centers, target, deadband, state, angles, loop rate, and connection. Keep the arm supported for the first run.

| Key | Action |
|---|---|
| `h` | Home slowly and return to idle |
| `t` | Toggle tracking; starts from idle only |
| `x` | Hold immediately and enter a fault state |
| `r` | Reset an operator fault to idle after checking the arm |
| `d` | Toggle detailed overlay |
| `q` | Hold and exit |

`HAND_LOST` sends `HOLD` after five missing processed frames by default. Reacquisition requires three consecutive detections. If serial replies fail or the camera fails, the host sends `HOLD` if it can; the firmware watchdog freezes the last commanded angle after 750 ms without a valid message. The watchdog also cancels an in-progress interpolated move. On a normal exit the host holds rather than detaching.

## Tuning

Tune in this order: servo directions, firmware limits, image deadbands, target filtering, gains, then rate limits. In `config.yaml`, lower `base_gain_deg_per_second` and `vertical_gain_deg_per_second` if the arm overshoots or hunts. Increase the deadband slightly if it chatters around center. `ema_alpha` lower values smooth more but add lag. `max_angle_change_per_update` limits host target changes. Firmware additionally uses a 35 degree/s maximum commanded speed and 90 degree/s² maximum commanded acceleration; edit the constants in the sketch and reupload only after slow tests.

The proportional controller uses measured time between frames, capped at 100 ms per update. Its signs and shoulder/elbow ratio require calibration because servo mounting and camera mirroring vary. Camera pitch is only approximately preserved; no inertial or joint feedback is installed.

## Serial protocol

Commands are newline terminated ASCII at 115200 baud: `PING`, `HOME`, `SET B:90 S:90 E:90`, `HOLD`, `DETACH`, `ATTACH`, `STATUS`, `LIMITS?`, and `LIMITS B:60:120 S:60:120 E:60:120`. `OK HOME` means homing was accepted; poll `STATUS` until `MODE:IDLE` for completion. `STATUS` reports commanded angles and targets. Limits persist in EEPROM with a checksum; invalid EEPROM data falls back to conservative defaults.

## Tests

From the repository root:

```powershell
python -m unittest discover -s . -v
```

The unit tests cover control direction and coupling, deadband, limits, jump rejection, target persistence, protocol formatting, hand loss, and fault reset. The Arduino sketch still needs a compile on your selected board and a supervised hardware check; this repository does not claim a physical motion test.

## Common problems

- **No port found:** set `serial.port` explicitly; close Arduino Serial Monitor first.
- **Firmware limits differ from YAML:** run `calibrate_servos.py`, inspect with `show`, then confirm `save` to synchronize EEPROM and config.
- **Arm moves away from hand:** reverse the relevant `direction` value; confirm whether preview mirroring changed the image sign.
- **A joint does not move:** check external power, common ground, signal pin, firmware limits, and whether `MODE:DETACHED` is reported.
- **Tracking oscillates:** lower gains or increase deadband/filtering. Keep the arm supported while tuning.
- **Camera opens but no landmarks appear:** check model path, indoor lighting, and hand visibility.
- **`TIMEOUT` appears:** the host is too slow or serial communication stalled; stop tracking and inspect the connection before retrying.

Use the arm only under direct supervision. Software limits and warnings do not make it safe for unsupervised interaction with people.

