// Arduino Nano + MPU6050: print the angle between the sensor's Z axis and the horizon.
//
// Wiring (Nano):  VCC -> 5V (module has a regulator) or 3.3V
//                 GND -> GND
//                 SDA -> A4
//                 SCL -> A5
//
// No library needed beyond Wire. Uses the accelerometer only (gravity vector),
// so it is accurate when the sensor is stationary or moving slowly.
//
// Output: 0 deg when Z is horizontal (board on its edge),
//         +90 deg when Z points straight up (board flat, chip facing up),
//         -90 deg when Z points straight down.

#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;   // 0x69 if AD0 is tied high

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake the MPU6050 (it powers up in sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);              // PWR_MGMT_1
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Accelerometer full scale +/-2g (default); configure explicitly
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);              // ACCEL_CONFIG
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Digital low-pass filter ~21 Hz to tame noise
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);              // CONFIG
  Wire.write(0x04);
  Wire.endTransmission(true);

  delay(100);
}

void loop() {
  // Read 6 bytes: ACCEL_XOUT_H .. ACCEL_ZOUT_L
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)6, (uint8_t)true);

  if (Wire.available() < 6) {
    Serial.println("MPU6050 read failed");
    delay(500);
    return;
  }

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  // Angle of Z axis above the horizontal plane:
  // elevation = atan2(z, sqrt(x^2 + y^2))
  float fx = ax, fy = ay, fz = az;
  float angleDeg = atan2(fz, sqrt(fx * fx + fy * fy)) * 180.0 / PI;

  Serial.print("Z vs horizon: ");
  Serial.print(angleDeg, 1);
  Serial.println(" deg");

  delay(100);
}
