// Arduino Nano + MPU6050: print the angle between the sensor's Y axis and the horizon,
// plus roll about the Y axis.
//
// Wiring (Nano):  VCC -> 5V (module has a regulator) or 3.3V
//                 GND -> GND
//                 SDA -> A4
//                 SCL -> A5
//                 LED -> D9 through a 220-330 ohm resistor to the LED anode,
//                        LED cathode to GND
//
// No library needed beyond Wire.
//
// Fusion: a complementary filter on the gravity direction vector.
//   - The gyro rotates the estimated gravity vector between samples (fast, drifts).
//   - The accelerometer pulls the estimate back toward measured gravity (slow, noisy).
// The angle is then taken from the fused vector, so it stays stable under
// vibration and short accelerations but does not drift over time.
//
// Output: Y vs horizon: 0 deg when Y is horizontal,
//                       +90 deg when +Y points straight up,
//                       -90 deg when +Y points straight down.
//         Roll:         tilt about the Y axis, reported 0..90 deg:
//                       0 when Z is vertical (up or down), 90 when X is vertical (up or down).
//                       Gyro-integrated so it works when Y is vertical; corrected by
//                       gravity whenever Y is tilted away from vertical. While Y is
//                       held vertical it relies on the gyro alone and will drift slowly.
//
// LED: brightness follows the Y elevation. Off when Y is at or below the horizon,
//      ramping (squared, for an even-looking fade) to fully on when +Y points straight up.
//      Roll scales it down: full effect at roll 0, fully off at roll 90.
//
// Keep the sensor still for the first ~2 seconds after reset (gyro bias calibration).

#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;        // 0x69 if AD0 is tied high
const uint8_t LED_PIN = 9;            // must be a PWM pin on the Nano: 3, 5, 6, 9, 10, 11

const float GYRO_LSB_PER_DPS = 131.0; // +/-250 deg/s full scale
const float ALPHA = 0.98;             // gyro weight; (1-ALPHA) is accel weight
const uint16_t CALIB_SAMPLES = 500;   // gyro bias samples at startup

float gBiasX = 0, gBiasY = 0, gBiasZ = 0;   // gyro bias, rad/s
float gx_est = 0, gy_est = 0, gz_est = 1;   // estimated gravity direction (unit vector)
float rollDeg = 0;                           // signed roll about Y, degrees, gyro-integrated
unsigned long lastMicros = 0;

float wrap180(float a) {
  while (a > 180.0)  a -= 360.0;
  while (a <= -180.0) a += 360.0;
  return a;
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

// Reads accel (raw counts) and gyro (rad/s, bias NOT removed). Returns false on I2C failure.
bool readRaw(int16_t &ax, int16_t &ay, int16_t &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                   // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() < 14) return false;

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();           // temperature, unused
  int16_t rgx = (Wire.read() << 8) | Wire.read();
  int16_t rgy = (Wire.read() << 8) | Wire.read();
  int16_t rgz = (Wire.read() << 8) | Wire.read();

  const float k = (PI / 180.0) / GYRO_LSB_PER_DPS;
  gx = rgx * k;
  gy = rgy * k;
  gz = rgz * k;
  return true;
}

void normalize(float &x, float &y, float &z) {
  float n = sqrt(x * x + y * y + z * z);
  if (n > 0) { x /= n; y /= n; z /= n; }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, 0);

  writeReg(0x6B, 0x00);               // PWR_MGMT_1: wake up
  writeReg(0x1C, 0x00);               // ACCEL_CONFIG: +/-2g
  writeReg(0x1B, 0x00);               // GYRO_CONFIG: +/-250 deg/s
  writeReg(0x1A, 0x03);               // CONFIG: DLPF ~44 Hz accel / 42 Hz gyro
  delay(100);

  // Gyro bias calibration: average readings while stationary
  Serial.println("Calibrating gyro, hold still...");
  int16_t ax, ay, az;
  float gx, gy, gz;
  float sx = 0, sy = 0, sz = 0;
  uint16_t n = 0;
  for (uint16_t i = 0; i < CALIB_SAMPLES; i++) {
    if (readRaw(ax, ay, az, gx, gy, gz)) {
      sx += gx; sy += gy; sz += gz;
      n++;
    }
    delay(3);
  }
  if (n > 0) { gBiasX = sx / n; gBiasY = sy / n; gBiasZ = sz / n; }

  // Seed the gravity estimate and roll from the accelerometer
  if (readRaw(ax, ay, az, gx, gy, gz)) {
    gx_est = ax; gy_est = ay; gz_est = az;
    normalize(gx_est, gy_est, gz_est);
    rollDeg = atan2(-gx_est, gz_est) * 180.0 / PI;
  }
  Serial.println("Done.");
  lastMicros = micros();
}

void loop() {
  int16_t ax, ay, az;
  float wx, wy, wz;
  if (!readRaw(ax, ay, az, wx, wy, wz)) {
    Serial.println("MPU6050 read failed");
    delay(500);
    return;
  }

  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6;
  lastMicros = now;

  wx -= gBiasX; wy -= gBiasY; wz -= gBiasZ;

  // Gyro prediction: a world-fixed vector seen from a rotating body changes as
  // dv/dt = -(omega x v). Apply one Euler step to the gravity estimate.
  float px = gx_est - (wy * gz_est - wz * gy_est) * dt;
  float py = gy_est - (wz * gx_est - wx * gz_est) * dt;
  float pz = gz_est - (wx * gy_est - wy * gx_est) * dt;
  normalize(px, py, pz);

  // Accelerometer measurement of gravity direction
  float mx = ax, my = ay, mz = az;
  normalize(mx, my, mz);

  // Complementary blend
  gx_est = ALPHA * px + (1.0 - ALPHA) * mx;
  gy_est = ALPHA * py + (1.0 - ALPHA) * my;
  gz_est = ALPHA * pz + (1.0 - ALPHA) * mz;
  normalize(gx_est, gy_est, gz_est);

  // Elevation of the Y axis above the horizontal plane
  float angleDeg = atan2(gy_est, sqrt(gx_est * gx_est + gz_est * gz_est)) * 180.0 / PI;

  // Roll: rotation about the Y axis. Internally signed and continuous:
  // 0 when +Z points up, +90 when -X points up, +/-180 when +Z points down.
  // The Y gyro is integrated every step, so roll keeps working when Y is vertical.
  // When Y is near the horizon, gravity's XZ direction gives an absolute roll
  // reference and is blended in to cancel gyro drift. The blend weight is the
  // fraction of gravity in the XZ plane: full strength with Y horizontal, zero
  // with Y vertical.
  rollDeg = wrap180(rollDeg + wy * (180.0 / PI) * dt);

  float horiz = sqrt(mx * mx + mz * mz);           // 0 Y vertical .. 1 Y horizontal
  if (horiz > 0.05) {
    float accelRoll = atan2(-mx, mz) * 180.0 / PI;
    float err = wrap180(accelRoll - rollDeg);
    rollDeg = wrap180(rollDeg + (1.0 - ALPHA) * horiz * err);
  }

  // Fold the signed roll to 0..90: 0 with Z up or down, 90 with X up or down.
  float rollOut = fabs(wrap180(rollDeg * 2.0) / 2.0);

  // LED brightness: 0 at or below the horizon, 255 with +Y straight up,
  // scaled down by roll so it is fully off at roll 90 and unaffected at roll 0.
  // The level is squared so the fade looks even to the eye.
  float level = constrain(angleDeg, 0.0, 90.0) / 90.0;
  level *= (90.0 - rollOut) / 90.0;
  level *= level;
  analogWrite(LED_PIN, (uint8_t)(level * 255.0 + 0.5));

  Serial.print("Y vs horizon: ");
  Serial.print(angleDeg, 1);
  Serial.print(" deg   Roll: ");
  Serial.print(rollOut, 1);
  Serial.println(" deg");

  delay(10);                          // ~100 Hz loop
}
