#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>

#define SERVO_CH_H_RIG1_PIN  10 
#define SERVO_CH_H_RIG2_PIN  11 
#define SERVO_CH_V_RIG1_PIN  12 
#define SERVO_CH_V_RIG2_PIN  13 


#define SENSOR_1_XSHUT       4
#define SENSOR_2_XSHUT       5

#define SENSOR_1_NEW_ADDRESS 0x30

#define HAPTIC_FRONT_LEFT    2
#define HAPTIC_BACK_LEFT     7
#define HAPTIC_FRONT_RIGHT   8
#define HAPTIC_BACK_RIGHT    9
#define DETECTION_THRESHOLD_MM 200
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

Servo servo_h_rig1; // Servo H_RIG1
Servo servo_h_rig2; // Servo H_RIG2
Servo servo_v_rig1; // Servo V_RIG1
Servo servo_v_rig2; // Servo V_RIG2

#define SERVO_MIN_US        500 // Min pulse width
#define SERVO_MAX_US        2500 // Max pulse width
#define SWEEP_PERIOD_US     3000000UL // 3 seconds period

// Loop execution rate for I2C throttling (5ms delay)
#define LOOP_DELAY_MS       5 // can increase till 10ms try smaller increements though

static void writeServoUs(Servo& servo, float angleDeg, float maxDeg) {
  if (angleDeg < 0) angleDeg = 0;
  if (angleDeg > maxDeg) angleDeg = maxDeg;

  uint16_t us = (uint16_t)map(angleDeg * 100, 0, maxDeg * 100, SERVO_MIN_US, SERVO_MAX_US);
  servo.writeMicroseconds(us);
}



void initSensor(Adafruit_VL53L0X &lox, int xshutPin, uint8_t newAddr, bool firstSensor) {
  pinMode(xshutPin, OUTPUT);
  digitalWrite(xshutPin, LOW); // Hold in reset
  delay(10);

  if (firstSensor) {
    digitalWrite(xshutPin, HIGH); delay(10);
    if (!lox.begin()) { Serial.print(F("Boot S1 fail on D")); Serial.println(xshutPin); while(1); }
    lox.setAddress(newAddr);
    Serial.print(F("Sensor 1 (Left) initialized and address changed to 0x")); Serial.println(newAddr, HEX);
  } else {
    digitalWrite(xshutPin, HIGH); delay(10);
    if (!lox.begin()) { Serial.print(F("Boot S2 fail on D")); Serial.println(xshutPin); while(1); }
    Serial.println(F("Sensor 2 (Right) initialized and kept default 0x29."));
  }
}

/*********** Setup ***********/
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Starting Nano System Initialization (Non-RTOS)..."));

  // Haptic Pin Setup
  pinMode(HAPTIC_FRONT_LEFT, OUTPUT);
  pinMode(HAPTIC_BACK_LEFT, OUTPUT);
  pinMode(HAPTIC_FRONT_RIGHT, OUTPUT);
  pinMode(HAPTIC_BACK_RIGHT, OUTPUT);

  digitalWrite(HAPTIC_FRONT_LEFT, LOW);
  digitalWrite(HAPTIC_BACK_LEFT, LOW);
  digitalWrite(HAPTIC_FRONT_RIGHT, LOW);
  digitalWrite(HAPTIC_BACK_RIGHT, LOW);

  // --- Servo Initialization (Direct PWM) ---
  // Attach servos to their respective pins
  servo_h_rig1.attach(SERVO_CH_H_RIG1_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo_h_rig2.attach(SERVO_CH_H_RIG2_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo_v_rig1.attach(SERVO_CH_V_RIG1_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo_v_rig2.attach(SERVO_CH_V_RIG2_PIN, SERVO_MIN_US, SERVO_MAX_US);


  writeServoUs(servo_h_rig1, 90.0f, 180.0f);
  writeServoUs(servo_h_rig2, 90.0f, 180.0f);
  writeServoUs(servo_v_rig1, 45.0f, 90.0f);
  writeServoUs(servo_v_rig2, 45.0f, 90.0f);

  Serial.println(F("Servos (Servo.h) initialized."));


  Wire.begin();
  Wire.setClock(400000UL);    // Fast-mode I2C 

  digitalWrite(SENSOR_1_XSHUT, LOW);
  digitalWrite(SENSOR_2_XSHUT, LOW);
  pinMode(SENSOR_1_XSHUT, OUTPUT);
  pinMode(SENSOR_2_XSHUT, OUTPUT);

  initSensor(lox1, SENSOR_1_XSHUT, SENSOR_1_NEW_ADDRESS, true);

  initSensor(lox2, SENSOR_2_XSHUT, 0x29, false);

  Serial.println(F("System ready. Starting loop..."));
}

void loop() {
  uint32_t t_us = micros() % SWEEP_PERIOD_US;
  float half_period_us = SWEEP_PERIOD_US / 2.0f;
  float sweep_factor;

  if (t_us < half_period_us) {
    sweep_factor = (float)t_us / half_period_us;
  } else {
    sweep_factor = 1.0f - (((float)t_us - half_period_us) / half_period_us); 
  }

  float h_angle = sweep_factor * 180.0f;
  float v_angle = sweep_factor * 90.0f;

  writeServoUs(servo_h_rig1, h_angle, 180.0f);
  writeServoUs(servo_h_rig2, h_angle, 180.0f);
  writeServoUs(servo_v_rig1, v_angle, 90.0f);
  writeServoUs(servo_v_rig2, v_angle, 90.0f);


  VL53L0X_RangingMeasurementData_t measure1;
  VL53L0X_RangingMeasurementData_t measure2;


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
}