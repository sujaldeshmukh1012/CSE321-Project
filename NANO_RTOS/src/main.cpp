// ===========================================================
// ARDUINO NANO UNIVERSAL HARDWARE TEST (with Motion Toggle + MPU6050 Test)
// - PCA9685 (4 Servos on channels 4–7)
// - MPU6050 (Jarzebski library)
// - Two VL53L0X sensors (Pololu library)
// - GPS (AltSoftSerial muted)
// - Haptic Motors, LEDs, Buzzer
// - Button toggles all motion
// ===========================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050.h>
#include <VL53L0X.h>
#include <AltSoftSerial.h>

// ===== Devices =====
Adafruit_PWMServoDriver pca(0x40);
MPU6050 mpu;
VL53L0X vl1;
VL53L0X vl2;
AltSoftSerial gpsSerial; // D8 RX, D9 TX

// ===== Pins =====
#define BUZZER_PIN 7
#define LED_RED A0
#define LED_BLUE A1
#define BUTTON_PIN A2
#define MOTOR1 3
#define MOTOR2 5
#define MOTOR3 6
#define MOTOR4 11
#define XSHUT1 12
#define XSHUT2 13

// ===== Servo Constants =====
#define SERVO_MIN 150
#define SERVO_MAX 600
#define SERVO_CENTER ((SERVO_MIN + SERVO_MAX) / 2)
#define SWEEP_STEP 5

// ===== State Variables =====
unsigned long lastPrint = 0;
unsigned long lastBeep = 0;
unsigned long lastButtonCheck = 0;
unsigned long lastMPUtest = 0;
bool dir = true;
int pwmVal = 0;
int angleFull = 0;
int dirFull = 1;
int angleSmall = 60;
int dirSmall = 1;
bool motionEnabled = true;
bool prevButtonState = HIGH;

// -----------------------------------------------------------
// Helper: Set servo angle (0–180°)
// -----------------------------------------------------------
void setServoAngle(uint8_t ch, int angleDeg) {
  int pulse = map(angleDeg, 0, 180, SERVO_MIN, SERVO_MAX);
  pca.setPWM(ch, 0, pulse);
}

// -----------------------------------------------------------
// Helper: PWM Buzzer beep
// -----------------------------------------------------------
void pwmBeep(unsigned long durationMs, int toneHz) {
  unsigned long endTime = millis() + durationMs;
  unsigned long halfPeriod = 500000L / toneHz;
  while (millis() < endTime) {
    analogWrite(BUZZER_PIN, 200);
    delayMicroseconds(halfPeriod);
    analogWrite(BUZZER_PIN, 0);
    delayMicroseconds(halfPeriod);
  }
}

// -----------------------------------------------------------
// SETUP
// -----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  Wire.begin();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(MOTOR1, OUTPUT);
  pinMode(MOTOR2, OUTPUT);
  pinMode(MOTOR3, OUTPUT);
  pinMode(MOTOR4, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);

  // ---- VL53L0X Init ----
  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  delay(50);
  digitalWrite(XSHUT1, HIGH);
  delay(10);
  vl1.init();
  vl1.setAddress(0x30);
  digitalWrite(XSHUT2, HIGH);
  delay(10);
  vl2.init();
  vl2.setAddress(0x31);
  vl1.startContinuous();
  vl2.startContinuous();

  // ---- PCA9685 Init ----
  pca.begin();
  pca.setPWMFreq(60);
  for (int ch = 0; ch <= 3; ch++) setServoAngle(ch, 0);

  // ---- MPU6050 Init ----
  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  mpu.calibrateGyro();
  mpu.setThreshold(3);

  // ---- Startup Beep ----
  pwmBeep(150, 2000);
  delay(100);
  pwmBeep(150, 2500);

  Serial.println(F("=== SYSTEM TEST START ==="));
  Serial.println(F("Press the button to toggle all motion ON/OFF"));
  Serial.println(F("Tilt or move board to verify MPU6050 readings"));
}

// -----------------------------------------------------------
// LOOP
// -----------------------------------------------------------
void loop() {
  // ---- Button Handling (Debounced Toggle) ----
  bool btn = (digitalRead(BUTTON_PIN) == LOW);
  if (btn != prevButtonState && (millis() - lastButtonCheck > 150)) {
    lastButtonCheck = millis();
    if (btn == LOW) {
      motionEnabled = !motionEnabled;
      digitalWrite(LED_RED, motionEnabled ? HIGH : LOW);
      digitalWrite(LED_BLUE, motionEnabled ? LOW : HIGH);
      pwmBeep(80, motionEnabled ? 2500 : 1200);
    }
    prevButtonState = btn;
  }

  // ---- MPU6050 Data Test ----
  // Every 500ms, print roll, pitch, yaw estimate
  if (millis() - lastMPUtest > 500) {
    lastMPUtest = millis();
    mpu.readNormalizeGyro();
    Vector normAccel = mpu.readNormalizeAccel();

    // simple tilt estimation (approx)
    float roll  = atan2(normAccel.YAxis, normAccel.ZAxis) * 57.2958;
    float pitch = atan(-normAccel.XAxis / sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * 57.2958;

    Serial.print(F("[MPU] Roll: "));
    Serial.print(roll);
    Serial.print(F("  Pitch: "));
    Serial.print(pitch);
    Serial.print(F("  AccZ: "));
    Serial.println(normAccel.ZAxis);
  }

  // ---- If motion OFF: freeze actuators ----
  if (!motionEnabled) {
    analogWrite(MOTOR1, 0);
    analogWrite(MOTOR2, 0);
    analogWrite(MOTOR3, 0);
    analogWrite(MOTOR4, 0);
    for (int ch = 0; ch <= 3; ch++) setServoAngle(ch, 90);
  } 
  else {
    // ---- Haptic Motor Sweep ----
    analogWrite(MOTOR1, pwmVal);
    analogWrite(MOTOR2, pwmVal);
    analogWrite(MOTOR3, pwmVal);
    analogWrite(MOTOR4, pwmVal);
    if (dir) pwmVal += 10; else pwmVal -= 10;
    if (pwmVal >= 255) dir = false;
    if (pwmVal <= 0) dir = true;

    // ---- Servo Control ----
    // 4–5 sweep full range
    angleFull += dirFull * SWEEP_STEP;
    if (angleFull >= 180 || angleFull <= 0) dirFull = -dirFull;
    setServoAngle(0, angleFull);
    setServoAngle(1, angleFull);

    // 6–7 oscillate 60–120°
    angleSmall += dirSmall * SWEEP_STEP;
    if (angleSmall >= 120 || angleSmall <= 60) dirSmall = -dirSmall;
    setServoAngle(2, angleSmall);
    setServoAngle(3, angleSmall);

    // ---- Periodic Buzzer ----
    if (millis() - lastBeep > 2000) {
      lastBeep = millis();
      pwmBeep(100, 1800);
    }
  }

  // ---- Range Sensors ----
  uint16_t d1 = vl1.readRangeContinuousMillimeters();
  uint16_t d2 = vl2.readRangeContinuousMillimeters();

  // ---- Periodic Serial Summary ----
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print(F("Motion=")); Serial.print(motionEnabled ? "ON" : "OFF");
    Serial.print(F("  VL1=")); Serial.print(d1);
    Serial.print(F("  VL2=")); Serial.print(d2);
    Serial.print(F("  PWM=")); Serial.println(pwmVal);
  }

  delay(50);
}
