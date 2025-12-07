#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

// Nano Every digital pins 10, 11, 12, 13 are used for Servo control.
#define SERVO_CH_H_RIG1_PIN  3
#define SERVO_CH_H_RIG2_PIN  9 
#define SERVO_CH_V_RIG1_PIN  5 
#define SERVO_CH_V_RIG2_PIN  6 

// for changing the hardware i2c address of the individual ToF Sensor
#define SENSOR_1_XSHUT       10
#define SENSOR_2_XSHUT       11

// --- HAPTIC PINS ---
#define HAPTIC_FRONT_LEFT    7
#define HAPTIC_BACK_LEFT     2
#define HAPTIC_FRONT_RIGHT   4
#define HAPTIC_BACK_RIGHT    8

// I2C Address for Sensor 1 (default for S2 will be 0x29)
#define SENSOR_1_NEW_ADDRESS 0x30


// --- THRESHOLDS AND TIMING ---
#define DETECTION_THRESHOLD_MM 200      // Distance in mm to trigger haptic feedback
#define SERVO_MIN_US        500          // Min pulse width for servo (fully counter-clockwise)
#define SERVO_MAX_US        2500        // Max pulse width for servo (fully clockwise)
#define SWEEP_PERIOD_US     3000000UL   /// 3 seconds period for one full sweep (up and down)
#define LOOP_DELAY_MS       5           // I2C throttling delay to prevent bus contention

#define SYSTEM_RUN_TIME_MS   20000UL   //- System halts after 20 seconds (20,000ms)

#define SERVO_RANGE_H_DEG    90.0f     // New horizontal sweep range (Reduced to 90 degrees)
#define SERVO_MIN_H_DEG      45.0f     // (180 - 90) / 2 = 45.0 to start the sweep

#define SERVO_RANGE_V_DEG    45.0f     // New vertical sweep range
#define SERVO_MIN_V_DEG      22.5f     // (90 - 45) / 2 =22.5 to start the sweep


Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

Servo servo_h_rig1;
Servo servo_h_rig2;
Servo servo_v_rig1;
Servo servo_v_rig2;

unsigned long startTime = 0; 

static void writeServoUs(Servo& servo, float angleDeg, float maxDeg) {
  if (angleDeg < 0.0f) angleDeg = 0.0f;
  if (angleDeg > maxDeg) angleDeg = maxDeg;
  float ratio = angleDeg / maxDeg;
  const float pulse_range = (float)(SERVO_MAX_US - SERVO_MIN_US);
  float us_float = SERVO_MIN_US + (ratio * pulse_range);
  servo.writeMicroseconds((uint16_t)us_float);
}

void initSensor(Adafruit_VL53L0X &lox, int xshutPin, uint8_t newAddr, bool firstSensor) {
  pinMode(xshutPin, OUTPUT);
  digitalWrite(xshutPin, LOW);
  delay(10);
  digitalWrite(xshutPin, HIGH);
  delay(10); // Give time for boot

  if (!lox.begin()) {
    Serial.print(F("Failed to boot sensor on XSHUT D"));
    Serial.println(xshutPin);
    while(1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }

  if (firstSensor) {
    lox.setAddress(newAddr);
    Serial.print(F("Sensor 1 (Left) initialized and address changed to 0x"));
    Serial.println(newAddr, HEX);
  } else {
    Serial.println(F("Sensor 2 (Right) initialized and kept default 0x29."));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Starting Nano Every System Initialization..."));
  pinMode(HAPTIC_FRONT_LEFT, OUTPUT);
  pinMode(HAPTIC_BACK_LEFT, OUTPUT);
  pinMode(HAPTIC_FRONT_RIGHT, OUTPUT);
  pinMode(HAPTIC_BACK_RIGHT, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(HAPTIC_FRONT_LEFT, LOW);
  digitalWrite(HAPTIC_BACK_LEFT, LOW);
  digitalWrite(HAPTIC_FRONT_RIGHT, LOW);
  digitalWrite(HAPTIC_BACK_RIGHT, LOW);
  servo_h_rig1.attach(SERVO_CH_H_RIG1_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo_h_rig2.attach(SERVO_CH_H_RIG2_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo_v_rig1.attach(SERVO_CH_V_RIG1_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo_v_rig2.attach(SERVO_CH_V_RIG2_PIN, SERVO_MIN_US, SERVO_MAX_US);
  writeServoUs(servo_h_rig1, 90.0f, 180.0f); // Center horizontal
  writeServoUs(servo_h_rig2, 90.0f, 180.0f); // Center horizontal
  writeServoUs(servo_v_rig1, 45.0f, 90.0f);  // Middle vertical sweep range
  writeServoUs(servo_v_rig2, 45.0f, 90.0f);  // Middle vertical sweep range

  Serial.println(F("Servos initialized for Nano Every."));
  Wire.begin();
  Wire.setClock(400000UL);    // Fast-mode I2C for faster communication
  pinMode(SENSOR_1_XSHUT, OUTPUT);
  pinMode(SENSOR_2_XSHUT, OUTPUT);
  digitalWrite(SENSOR_1_XSHUT, LOW);
  digitalWrite(SENSOR_2_XSHUT, LOW);

  initSensor(lox1, SENSOR_1_XSHUT, SENSOR_1_NEW_ADDRESS, true);
  initSensor(lox2, SENSOR_2_XSHUT, 0x29, false); // S2 keeps default 0x29

  startTime = millis(); 

  Serial.println(F("System ready. Starting loop..."));
}

/*********** Loop ***********/
void loop() {
  // 1. Calculate the sweep factor (0.0 to 1.0 back to 0.0) based on time
  uint32_t t_us = micros() % SWEEP_PERIOD_US;
  float half_period_us = SWEEP_PERIOD_US / 2.0f;
  float sweep_factor;

  if (t_us < half_period_us) {
    // Phase 1: 0 to 1.0
    sweep_factor = (float)t_us / half_period_us;
  } else {
    // Phase 2: 1.0 back to 0.0
    sweep_factor = 1.0f - (((float)t_us - half_period_us) / half_period_us);
  }

  float h_angle = SERVO_MIN_H_DEG + (sweep_factor * SERVO_RANGE_H_DEG);
  float v_angle = SERVO_MIN_V_DEG + (sweep_factor * SERVO_RANGE_V_DEG);
  writeServoUs(servo_h_rig1, h_angle, 180.0f);
  writeServoUs(servo_h_rig2, h_angle, 180.0f);
  writeServoUs(servo_v_rig1, v_angle, 90.0f);
  writeServoUs(servo_v_rig2, v_angle, 90.0f);
  VL53L0X_RangingMeasurementData_t measure1; // Left
  VL53L0X_RangingMeasurementData_t measure2; // Right
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);
  bool obstacleLeft = (measure1.RangeStatus != 4 && measure1.RangeMilliMeter < DETECTION_THRESHOLD_MM);
  bool obstacleRight = (measure2.RangeStatus != 4 && measure2.RangeMilliMeter < DETECTION_THRESHOLD_MM);
  digitalWrite(HAPTIC_FRONT_LEFT, obstacleLeft ? HIGH : LOW);
  digitalWrite(HAPTIC_BACK_LEFT, obstacleLeft ? HIGH : LOW);
  digitalWrite(HAPTIC_FRONT_RIGHT, obstacleRight ? HIGH : LOW);
  digitalWrite(HAPTIC_BACK_RIGHT, obstacleRight ? HIGH : LOW);
  Serial.print(F("S1 (L): "));
  Serial.print(obstacleLeft ? measure1.RangeMilliMeter : 9999);
  Serial.print(F("mm, Haptic L: "));
  Serial.print(obstacleLeft ? F("ON") : F("OFF"));

  Serial.print(F(" | S2 (R): "));
  Serial.print(obstacleRight ? measure2.RangeMilliMeter : 9999);
  Serial.print(F("mm, Haptic R: "));
  Serial.println(obstacleRight ? F("ON") : F("OFF"));
  delay(LOOP_DELAY_MS);

  if (millis() - startTime >= SYSTEM_RUN_TIME_MS) {
    Serial.println(F("\n--- 20 SECOND RUNTIME COMPLETE. SYSTEM HALTING. ---"));
    
    writeServoUs(servo_h_rig1, 90.0f, 180.0f);
    writeServoUs(servo_h_rig2, 90.0f, 180.0f);
    writeServoUs(servo_v_rig1, 45.0f, 90.0f);
    writeServoUs(servo_v_rig2, 45.0f, 90.0f);
    
    servo_h_rig1.detach();
    servo_h_rig2.detach();
    servo_v_rig1.detach();
    servo_v_rig2.detach();

    digitalWrite(HAPTIC_FRONT_LEFT, LOW);
    digitalWrite(HAPTIC_BACK_LEFT, LOW);
    digitalWrite(HAPTIC_FRONT_RIGHT, LOW);
    digitalWrite(HAPTIC_BACK_RIGHT, LOW);

    for(int i = 0; i < 5; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
    
    // Halt execution indefinitely
    while (true) {
      Serial.println(F("System Halted."));
      delay(5000); 
    }
  }
}