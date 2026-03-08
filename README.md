# Porteur Dynamics Track 1.0 — V1 Firmware

IMU-based head tracker for RP2040 + BMI270, outputting USB HID gamepad data
readable by OpenTrack (Joystick input) and mappable to any DirectInput game.

---

## A. Why earlier versions failed to appear as a controller

Several common failure modes when mixing TinyUSB + I2C sensor code:

| Root cause | Effect |
|---|---|
| `Wire.begin()` called before `TinyUSBDevice.begin()` / `usb_hid.begin()` | USB descriptor sent before the stack is ready → Windows sees a broken device |
| Blocking `while(!imu.begin())` loop in `setup()` before `usb_hid.begin()` | USB enumeration never starts → device not recognised |
| `delay()` or tight I2C polling inside the USB interrupt context | Starves the TinyUSB task → Windows ejects the device mid-session |
| Calling `TinyUSBDevice.detach()/attach()` *after* a successful mount without a guard | Unnecessarily re-enumerates; can confuse Windows driver caching |
| Forgetting `#ifdef TINYUSB_NEED_POLLING_TASK` in the loop | Missed on RP2040 TinyUSB non-IRQ builds → HID packets never sent |

**Fix applied here:** TinyUSB init happens first (lines identical to MK6 skeleton),
then I2C / BMI270 init happens *after*, with a 100 ms delay to let USB settle.
If BMI270 fails, the firmware enters an infinite loop that still sends zero HID
reports so Windows continues to see the controller.

---

## B. Hardware

| Signal | RP2040 GPIO |
|---|---|
| I2C SDA | GP5 |
| I2C SCL | GP4 |
| BMI270 I2C address | 0x68 |
| Recenter/Calibrate button | GP15 → GND |

---

## C. HID axis mapping — where Windows/OpenTrack will see yaw/pitch/roll

The firmware uses the standard `TUD_HID_REPORT_DESC_GAMEPAD()` descriptor.
Windows enumerates the six analog axes in HID Usage ID order:

| HID Usage | DirectInput axis name | Windows axis number (1-based) | Firmware field | Carries |
|---|---|---|---|---|
| 0x30 X  | X  | 1 | `gp.x`  | (unused, always 0) |
| 0x31 Y  | Y  | 2 | `gp.y`  | (unused, always 0) |
| 0x32 Z  | Z  | 3 | `gp.z`  | (unused, always 0) |
| 0x33 Rx | Rx | 4 | `gp.rx` | **Roll** |
| 0x34 Ry | Ry | 5 | `gp.ry` | **Pitch** |
| 0x35 Rz | Rz | 6 | `gp.rz` | **Yaw** |

> **Note:** `hid_gamepad_report_t` stores the axes in memory order  
> `x, y, z, rz, rx, ry` but the *HID descriptor* declares them  
> X, Y, Z, Rx, Ry, Rz.  Windows follows the *descriptor*, not the  
> struct byte order, so axis 4 = Rx = roll, axis 5 = Ry = pitch,  
> axis 6 = Rz = yaw in joy.cpl and OpenTrack.

### OpenTrack "Joystick input" mapping

In OpenTrack → Input → Joystick input:

| OpenTrack channel | Joystick axis to select |
|---|---|
| Pitch | Axis 5 (Ry) |
| Yaw   | Axis 6 (Rz) |
| Roll  | Axis 4 (Rx) |

(X, Y, Z translations are not provided; leave them unmapped or at 0.)

---

## D. Debug checklist

### 1 — Verify controller enumeration (Windows)

1. Plug in the RP2040.  
2. Open **Device Manager** → Human Interface Devices.  
   You should see **"HID-compliant game controller"** (or similar).  
3. If it appears with a yellow warning, the USB descriptor is broken —  
   flash only the MK6 skeleton to confirm the TinyUSB stack works, then  
   re-flash V1.

### 2 — Verify one axis moves in joy.cpl

1. Open **Run → joy.cpl** → Properties of your controller.  
2. Tilt or rotate the tracker slowly.  
3. One of the six bars in the **Axes** section should move.  
   - Roll  (left/right tilt) → Rx bar (4th bar)  
   - Pitch (up/down tilt)   → Ry bar (5th bar)  
   - Yaw   (left/right turn) → Rz bar (6th bar)  
4. If no bar moves, check Serial Monitor at 115200 baud —  
   the firmware will be stuck in the BMI270-failed loop (sending zero reports).

### 3 — Verify OpenTrack sees the tracker

1. Launch OpenTrack.  
2. Set **Input** to **Joystick input**.  
3. Click the wrench icon next to Input → select your controller.  
4. Map Pitch → Axis 5, Yaw → Axis 6, Roll → Axis 4.  
5. Click **Start** and slowly move your head.  
6. The OpenTrack preview axes should move.  
7. If axes are inverted, enable **Invert** for that axis in OpenTrack.

---

## E. Sensitivity tuning

| Where | Parameter | Default | Effect |
|---|---|---|---|
| Firmware (`PorteurDynamics_Track_v1.ino`) | `SCALE` | `1.0` | 1 degree head movement = 1 axis unit out of 127.  Increase for more range. |
| Firmware | `ALPHA` | `0.97` | Complementary filter weight.  Lower = accel corrects faster but is noisier. |
| OpenTrack → Mapping curves | — | — | Best place to add non-linear response, dead zones, max angle. |

---

## F. Button behaviour

| Press duration | Action |
|---|---|
| < 2 seconds (short press) | **Recenter** — stores current pitch/roll/yaw as the new zero position |
| ≥ 2 seconds (long press) | **Recalibrate gyro bias** (hold tracker still for ~1 s) + Recenter |

Startup also performs an automatic gyro bias calibration (~200 samples, ~1 s).  
Keep the tracker still immediately after plugging it in.

---

## G. Required Arduino libraries

| Library | Install via Arduino Library Manager |
|---|---|
| Adafruit TinyUSB Library | `Adafruit TinyUSB Library` |
| SparkFun BMI270 Arduino Library | `SparkFun BMI270 Arduino Library` |

Board package: **Raspberry Pi Pico/RP2040 by Earle F. Philhower, III**  
(Boards Manager URL: `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`)

Board setting: **Raspberry Pi Pico**  
USB Stack: **Adafruit TinyUSB**
