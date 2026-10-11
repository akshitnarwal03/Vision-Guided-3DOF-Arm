# Camera Based Hand Tracking Robotic Arm

A Stage 1, three-servo robotic arm controller for Arduino Uno/Nano class boards, MG995 positional servos, a USB camera, and a Windows Python host. The base corrects left/right image error. Shoulder and elbow move in opposite calibrated directions to correct up/down error while approximately preserving camera pitch. Tracking begins only after the operator presses `t`.

This repository includes Arduino firmware, a manual calibration tool, MediaPipe hand tracking, a preview UI, and unit tests. Distance tracking is reserved for a later stage; the disabled `tof.py` adapter does not affect Stage 1.

## Hardware and wiring

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

