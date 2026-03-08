/*
 * Porteur Dynamics Track 1.0 - V1 Firmware
 * =========================================
 * MCU   : RP2040
 * IMU   : Bosch BMI270 @ I2C address 0x68
 *           SDA = GP5 | SCL = GP4
 * USB   : Adafruit TinyUSB - HID Gamepad
 * Button: GPIO15 -> GND (INPUT_PULLUP)
 *
 * HID axis mapping (Windows / OpenTrack):
 *   gp.rx  -> Roll  (tilt left/right)
 *   gp.ry  -> Pitch (tilt up/down)
 *   gp.rz  -> Yaw   (turn left/right)
 *
 *   In DirectInput / joy.cpl the six gamepad axes appear ordered by
 *   HID Usage ID: X(1), Y(2), Z(3), Rx(4), Ry(5), Rz(6).
 *   OpenTrack "Joystick input" typically labels them as:
 *     Axis 4 -> Rx (roll)   - gp.rx
 *     Axis 5 -> Ry (pitch)  - gp.ry
 *     Axis 6 -> Rz (yaw)    - gp.rz
 *   Map those three axes in the OpenTrack "Input" tab accordingly.
 *
 * Button behaviour:
 *   Short press (released before 2 s) : Recenter
 *   Long  press (released after  2 s) : Recalibrate gyro bias + Recenter
 *
 * Sensitivity:
 *   SCALE = 1.0  -> 1 degree of head movement ≈ 1 axis unit (very calm).
 *   Increase SCALE or use OpenTrack curves to add more range.
 *
 * Complementary filter:
 *   Pitch & roll: gyro integration + accel correction (ALPHA ≈ 0.97).
 *   Yaw         : gyro integration only (no magnetometer; use Recenter
 *                 periodically to reset accumulated drift).
 */

#include "Adafruit_TinyUSB.h"
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>

// -- HID descriptor - identical to the known-working MK6 skeleton ----------
uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_GAMEPAD()
};

Adafruit_USBD_HID usb_hid;
hid_gamepad_report_t gp;

// -- Pin definitions -------------------------------------------------------
static const uint8_t BTN_PIN = 15;
static const uint8_t SDA_PIN =  5;
static const uint8_t SCL_PIN =  4;
static const uint8_t IMU_ADDR = 0x68;

// -- IMU -------------------------------------------------------------------
BMI270 imu;

// -- Filter constants ------------------------------------------------------
// ALPHA: fraction of gyro vs. accelerometer each tick.
// 0.97 = very smooth; lower if you want accel to correct faster.
static const float ALPHA = 0.97f;

// SCALE: int8_t axis units per degree of angular displacement.
// 1.0 -> ±127 axis = ±127 °.  Deliberately calm for initial testing.
static const float SCALE = 1.0f;

// -- Filter state ----------------------------------------------------------
static float pitch_cf  = 0.0f;   // complementary-filtered pitch (°)
static float roll_cf   = 0.0f;   // complementary-filtered roll  (°)
static float yaw_int   = 0.0f;   // gyro-integrated yaw          (°)

// Reference angles removed on recenter
static float ref_pitch = 0.0f;
static float ref_roll  = 0.0f;
static float ref_yaw   = 0.0f;

// Gyro bias (subtracted every tick)
static float bias_gx = 0.0f;
static float bias_gy = 0.0f;
static float bias_gz = 0.0f;

// -- Timing ----------------------------------------------------------------
static unsigned long last_us = 0;

// -- Button state machine --------------------------------------------------
static const unsigned long LONG_PRESS_MS = 2000;
static bool               btn_was_low   = false;
static unsigned long      btn_down_ms   = 0;

// -- clamp8: saturate a float (scaled angle in degrees) to int8_t range.
// Parameters: v - scaled angle value to clamp.
// Returns:    value clamped to [-127, 127] and cast to int8_t.
static inline int8_t clamp8(float v)
{
  if (v >  127.0f) return  127;
  if (v < -127.0f) return -127;
  return (int8_t)v;
}

// -- Calibrate gyro bias (~200 samples, keep tracker still) ---------------
static void calibrateGyroBias()
{
  const int N = 200;
  float sx = 0.0f, sy = 0.0f, sz = 0.0f;
  for (int i = 0; i < N; i++) {
    imu.getSensorData();
    sx += imu.data.gyroX;
    sy += imu.data.gyroY;
    sz += imu.data.gyroZ;
    delay(5);
  }
  bias_gx = sx / N;
  bias_gy = sy / N;
  bias_gz = sz / N;
}

// -- recenter: capture current pitch/roll/yaw as the new zero reference.
// Call on button press so subsequent relative angles are computed from
// the current head position instead of the original startup orientation.
static void recenter()
{
  ref_pitch = pitch_cf;
  ref_roll  = roll_cf;
  ref_yaw   = yaw_int;
}

// -------------------------------------------------------------------------
// setup()
// -------------------------------------------------------------------------
void setup()
{
  // 1. TinyUSB init - must happen first, exactly as in the MK6 skeleton
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  Serial.begin(115200);

  // 2. I2C - explicit pin assignment for GP5 (SDA) / GP4 (SCL)
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);  // 400 kHz fast-mode

  // 3. Button
  pinMode(BTN_PIN, INPUT_PULLUP);

  // 4. HID - identical to MK6 skeleton
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  // 5. BMI270 - initialise after TinyUSB to avoid I2C/USB race
  delay(100);
  if (imu.beginI2C(IMU_ADDR, Wire) != BMI2_OK) {
    // IMU failed: still enumerate as a gamepad so Windows shows
    // the controller.  Loop sending zero reports until the IMU
    // is fixed, making debugging easier.
    while (true) {
#ifdef TINYUSB_NEED_POLLING_TASK
      TinyUSBDevice.task();
#endif
      if (TinyUSBDevice.mounted() && usb_hid.ready()) {
        memset(&gp, 0, sizeof(gp));
        usb_hid.sendReport(0, &gp, sizeof(gp));
      }
      delay(20);
    }
  }

  // 6. Gyro bias calibration - hold tracker still for ~1 second
  calibrateGyroBias();

  // 7. Seed the complementary filter with accel-derived angles
  //    so pitch/roll start at a sensible value instead of 0.
  imu.getSensorData();
  const float ax = imu.data.accelX;
  const float ay = imu.data.accelY;
  const float az = imu.data.accelZ;
  pitch_cf = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f;
  roll_cf  = atan2f( ay, az)                        * 57.2957795f;
  yaw_int  = 0.0f;

  last_us = micros();
}

// -------------------------------------------------------------------------
// loop()
// -------------------------------------------------------------------------
void loop()
{
#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif

  // Gate on USB enumeration - identical logic to MK6 skeleton
  if (!TinyUSBDevice.mounted()) return;
  if (!usb_hid.ready())         return;

  // -- 1. Timing ----------------------------------------------------------
  const unsigned long now_us = micros();
  float dt = (float)(now_us - last_us) * 1.0e-6f;
  last_us = now_us;
  // Clamp dt: ignore any huge gap (boot, USB re-enum, debugger pause)
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

  // -- 2. Read IMU --------------------------------------------------------
  imu.getSensorData();

  // Gyro (dps) with bias removed
  const float gx = imu.data.gyroX - bias_gx;
  const float gy = imu.data.gyroY - bias_gy;
  const float gz = imu.data.gyroZ - bias_gz;

  // Accel (g)
  const float ax = imu.data.accelX;
  const float ay = imu.data.accelY;
  const float az = imu.data.accelZ;

  // -- 3. Complementary filter --------------------------------------------
  // Accel-derived reference angles (degrees).
  // Sign convention: pitch nose-up positive, roll right-side-up positive.
  const float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f;
  const float accel_roll  = atan2f( ay, az)                        * 57.2957795f;

  // High-pass the gyro integral; low-pass the accel angle.
  pitch_cf = ALPHA * (pitch_cf + gx * dt) + (1.0f - ALPHA) * accel_pitch;
  roll_cf  = ALPHA * (roll_cf  + gy * dt) + (1.0f - ALPHA) * accel_roll;

  // Yaw: pure gyro integration (drift accumulates; use Recenter to reset).
  yaw_int += gz * dt;

  // -- 4. Button state machine --------------------------------------------
  const bool btn_low     = (digitalRead(BTN_PIN) == LOW);
  const unsigned long now_ms = millis();

  if (btn_low && !btn_was_low) {
    // Falling edge: record press time
    btn_down_ms  = now_ms;
  } else if (!btn_low && btn_was_low) {
    // Rising edge: act on release
    const unsigned long held = now_ms - btn_down_ms;
    if (held >= LONG_PRESS_MS) {
      // Long press: recalibrate gyro bias then recenter
      calibrateGyroBias();
      yaw_int = 0.0f;
    }
    recenter();
  }
  btn_was_low = btn_low;

  // -- 5. Relative angles (offset from last recenter) --------------------
  const float rel_pitch = pitch_cf - ref_pitch;
  const float rel_roll  = roll_cf  - ref_roll;
  const float rel_yaw   = yaw_int  - ref_yaw;

  // -- 6. Build HID report ------------------------------------------------
  //   x, y, z, hat, buttons stay zero (no physical joystick axes / hat).
  //   rx = roll, ry = pitch, rz = yaw carry the tracking data.
  gp.x       = 0;
  gp.y       = 0;
  gp.z       = 0;
  gp.rx      = clamp8(rel_roll  * SCALE);
  gp.ry      = clamp8(rel_pitch * SCALE);
  gp.rz      = clamp8(rel_yaw   * SCALE);
  gp.hat     = 0;
  gp.buttons = 0;

  usb_hid.sendReport(0, &gp, sizeof(gp));

  // ~200 Hz loop; USB poll interval is 2 ms so 5 ms is a safe floor.
  delay(5);
}
