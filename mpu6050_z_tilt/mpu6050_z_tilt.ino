// Arduino Nano + MPU6050: print the angle between the sensor's Z axis and the horizon.
//
// Wiring (Nano):  VCC -> 5V (module has a regulator) or 3.3V
//                 GND -> GND
//                 SDA -> A4
//                 SCL -> A5
//
// No library needed beyond Wire.
//
// Fusion: a complementary filter on the gravity direction vector.
//   - The gyro rotates the estimated gravity vector between samples (fast, drifts).
//   - The accelerometer pulls the estimate back toward measured gravity (slow, noisy).
// The angle is then taken from the fused vector, so it stays stable under
// vibration and short accelerations but does not drift over time.
//
// Output: Z vs horizon: 0 deg when Z is horizontal (board on its edge),
//                       +90 deg when Z points straight up (board flat, chip facing up),
//                       -90 deg when Z points straight down.
//         Roll:         rotation about the Z axis, 0 when +Y is up, +90 when +X is up.
//                       Valid when Z is near the horizon; undefined when Z is vertical.
//
// Keep the sensor still for the first ~2 seconds after reset (gyro bias calibration).

#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;        // 0x69 if AD0 is tied high

const float GYRO_LSB_PER_DPS = 131.0; // +/-250 deg/s full scale
const float ALPHA = 0.98;             // gyro weight; (1-ALPHA) is accel weight
const uint16_t CALIB_SAMPLES = 500;   // gyro bias samples at startup

float gBiasX = 0, gBiasY = 0, gBiasZ = 0;   // gyro bias, rad/s
float gx_est = 0, gy_est = 0, gz_est = 1;   // estimated gravity direction (unit vector)
unsigned long lastMicros = 0;

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

  // Seed the gravity estimate from the accelerometer
  if (readRaw(ax, ay, az, gx, gy, gz)) {
    gx_est = ax; gy_est = ay; gz_est = az;
    normalize(gx_est, gy_est, gz_est);
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

  // Elevation of the Z axis above the horizontal plane
  float angleDeg = atan2(gz_est, sqrt(gx_est * gx_est + gy_est * gy_est)) * 180.0 / PI;

  // Roll: rotation about the Z axis, from the direction of gravity in the XY plane.
  // 0 when +Y points up, +90 when +X points up, +/-180 when +Y points down.
  // Only meaningful when Z is near the horizon; it becomes undefined as Z
  // approaches vertical (gravity has no XY component to measure against).
  float rollDeg = atan2(gx_est, gy_est) * 180.0 / PI;

  Serial.print("Z vs horizon: ");
  Serial.print(angleDeg, 1);
  Serial.print(" deg   Roll: ");
  Serial.print(rollDeg, 1);
  Serial.println(" deg");

  delay(10);                          // ~100 Hz loop
}
