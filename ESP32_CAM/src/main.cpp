// ===========================================================
// ESP32 MPU6050 Reader (I2C using Wire)
// - Continuously reads accel + gyro data
// - Prints Roll, Pitch, and raw sensor values
// ===========================================================

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>  // from jarzebski/Arduino-MPU6050 or equivalent

MPU6050 mpu;  // create IMU object

void setup() {
  Serial.begin(115200);
  Wire.begin(); // SDA=21, SCL=22 by default on ESP32
  Serial.println(F("\n=== ESP32 MPU6050 Test ==="));

  // Try both common addresses
  if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G, 0x68)) {
    Serial.println(F("MPU6050 not found at 0x68, retrying 0x69..."));
    if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G, 0x69)) {
      Serial.println(F("ERROR: MPU6050 not detected. Check wiring/power."));
      while (1);
    }
  }

  mpu.calibrateGyro();
  mpu.setThreshold(3);

  Serial.println(F("MPU6050 initialized.\n"));
  delay(500);
}

void loop() {
  // --- Read normalized data ---
  Vector normAccel = mpu.readNormalizeAccel();
  Vector normGyro  = mpu.readNormalizeGyro();

  // --- Compute simple tilt estimation ---
  float roll  = atan2(normAccel.YAxis, normAccel.ZAxis) * 57.2958;
  float pitch = atan(-normAccel.XAxis /
                    sqrt(normAccel.YAxis * normAccel.YAxis +
                         normAccel.ZAxis * normAccel.ZAxis)) * 57.2958;

  // --- Display data ---
  Serial.print(F("Roll: "));
  Serial.print(roll, 2);
  Serial.print(F("  Pitch: "));
  Serial.print(pitch, 2);
  Serial.print(F("  AccX: "));
  Serial.print(normAccel.XAxis, 2);
  Serial.print(F("  AccY: "));
  Serial.print(normAccel.YAxis, 2);
  Serial.print(F("  AccZ: "));
  Serial.print(normAccel.ZAxis, 2);
  Serial.print(F("  GyroZ: "));
  Serial.println(normGyro.ZAxis, 2);

  delay(250);
}
