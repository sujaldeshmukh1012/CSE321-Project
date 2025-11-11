/*
  Master Controller — Arduino Nano + FreeRTOS

  Components:
    - 2x VL53L0X ToF (via I2C)  [requires unique I2C addresses; see XSHUT pins below]
    - PCA9685 16-ch servo driver @ 0x40 (I2C)
      * servos on channels: 0,1 = horizontal (270° sweep), 2,3 = vertical (90° sweep)
    - Neo M6N GPS (AltSoftSerial: RX=D8, TX=D9)
    - IMU MPU6050 (I2C)
    - Red LED  on A0 (via 220Ω)
    - Blue LED on A1 (via 220Ω)
    - Push Button on A2 (INPUT_PULLUP → GND when pressed)
    - Passive Buzzer on D7 (via 100–220Ω)
    - 4x Haptic motors on D3,D5,D6,D11 (NPN transistor base via 1k; flyback diode across motor)

  Notes:
    - VL53L0X sensors power-up at the same default address (0x29). You MUST wire each sensor’s XSHUT pin
      to its own Arduino pin so we can bring them up one-by-one and assign unique I2C addresses.
      Below we use:
        * TOF1_XSHUT → D10
        * TOF2_XSHUT → D12
    - If you already hard-wired one sensor to a different address, adjust TOF*_ADDR below and skip XSHUT handling.
    - This sketch creates parallel FreeRTOS tasks. Stack sizes are tight for Nano (2 KB SRAM). Keep Serial prints concise.

  Sector semantics for haptics (heuristic, horizontal-only):
    - On Rig (0,2): angle ~135° ≈ “right”
    - On Rig (1,3): angle ~135° ≈ “left”
    - “front” ≈ ~75–105° window
    - “back”  ≈ ~165–195° window
    Windows are approximate within the 0–270° sweep.

  Requires Libraries:
    - Arduino_FreeRTOS (or FreeRTOS for AVR)
    - AltSoftSerial
    - TinyGPSPlus
    - Wire
    - Adafruit PWM Servo Driver (Adafruit_PWMServoDriver)
    - Pololu VL53L0X (lighter than Adafruit’s; install “VL53L0X by Pololu”)
    - MPU6050 (from i2cdevlib) or “MPU6050 by Electronic Cats”
*/

/*
  Master Controller — Arduino Nano + FreeRTOS (Simplified)

  Components:
    - 2x VL53L0X ToF sensors (I2C)
    - PCA9685 16-ch servo driver (I2C)
      * servos on channels:
        0,1 = horizontal sweep (270°)
        2,3 = vertical sweep (90°)
    - IMU MPU6050 (I2C)
    - Red LED (A0)
    - Blue LED (A1)
    - Passive Buzzer (D7)
    - 4x Haptic motors on D3, D5, D6, D11
*/

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L0X.h>
#include <MPU6050.h>

/*********** Pin Map ***********/
#define PIN_LED_RED      A0
#define PIN_LED_BLUE     A1
#define PIN_BUZZER       7

#define PIN_HAPTIC_1     3    // Front
#define PIN_HAPTIC_2     5    // Right
#define PIN_HAPTIC_3     6    // Back
#define PIN_HAPTIC_4     11   // Left

#define PIN_TOF1_XSHUT   10
#define PIN_TOF2_XSHUT   12

/*********** PCA9685 ***********/
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_PWM_FREQ      50
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2500
#define US_TO_TICKS(us)     (uint16_t)((float)(us) * SERVO_PWM_FREQ * 4096.0f / 1000000.0f)

#define H_SWEEP_MAX_DEG     270.0f
#define V_SWEEP_MAX_DEG      90.0f
#define CH_H_RIG1            0
#define CH_H_RIG2            1
#define CH_V_RIG1            2
#define CH_V_RIG2            3

/*********** ToF Sensors ***********/
VL53L0X tof1, tof2;
#define TOF1_ADDR 0x29
#define TOF2_ADDR 0x2A
#define TOF_TIMING_BUDGET_US 20000UL

/*********** IMU ***********/
MPU6050 imu;

/*********** FreeRTOS Sync ***********/
SemaphoreHandle_t mapMutex;

/*********** Mapping ***********/
#define H_CELLS   18
#define V_CELLS    6

static uint16_t mapRig1[V_CELLS][H_CELLS];
static uint16_t mapRig2[V_CELLS][H_CELLS];

#define OBSTACLE_NEAR_MM     350
#define OBSTACLE_MID_MM      700
#define OBSTACLE_FAR_MM     1200

#define HAPTIC_PWM_NEAR      255
#define HAPTIC_PWM_MID       160
#define HAPTIC_PWM_FAR        80
#define HAPTIC_PWM_OFF         0

/*********** Utilities ***********/
static uint16_t angleToUs(float angleDeg, float maxDeg) {
  if (angleDeg < 0) angleDeg = 0;
  if (angleDeg > maxDeg) angleDeg = maxDeg;
  float t = angleDeg / maxDeg;
  return (uint16_t)(SERVO_MIN_US + t * (SERVO_MAX_US - SERVO_MIN_US));
}

static void writeServo(uint8_t channel, float angleDeg, float maxDeg) {
  uint16_t ticks = US_TO_TICKS(angleToUs(angleDeg, maxDeg));
  pwm.setPWM(channel, 0, ticks);
}

static void beep(uint16_t ms) {
  tone(PIN_BUZZER, 3000);
  vTaskDelay(pdMS_TO_TICKS(ms));
  noTone(PIN_BUZZER);
}

static void initToFTwoSensors() {
  pinMode(PIN_TOF1_XSHUT, OUTPUT);
  pinMode(PIN_TOF2_XSHUT, OUTPUT);

  digitalWrite(PIN_TOF1_XSHUT, LOW);
  digitalWrite(PIN_TOF2_XSHUT, LOW);
  vTaskDelay(pdMS_TO_TICKS(10));

  digitalWrite(PIN_TOF1_XSHUT, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));
  tof1.init(true);
  tof1.setAddress(TOF1_ADDR);
  tof1.setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
  tof1.startContinuous();

  digitalWrite(PIN_TOF2_XSHUT, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));
  tof2.init(true);
  tof2.setAddress(TOF2_ADDR);
  tof2.setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
  tof2.startContinuous();
}

static float hIndexToDeg(int h) { return (H_SWEEP_MAX_DEG * h) / (H_CELLS - 1); }
static float vIndexToDeg(int v) { return (V_SWEEP_MAX_DEG * v) / (V_CELLS - 1); }

static uint16_t minDistanceInWindow(uint16_t grid[V_CELLS][H_CELLS], int hCenterDeg, int hHalfWidthDeg) {
  int minmm = 0;
  for (int v = V_CELLS/3; v <= (2*V_CELLS)/3; ++v) {
    for (int h = 0; h < H_CELLS; ++h) {
      float ang = hIndexToDeg(h);
      if (ang >= (hCenterDeg - hHalfWidthDeg) && ang <= (hCenterDeg + hHalfWidthDeg)) {
        uint16_t d = grid[v][h];
        if (d != 0 && (minmm == 0 || d < minmm)) minmm = d;
      }
    }
  }
  return (uint16_t)minmm;
}

static uint8_t levelFromDistance(uint16_t mm) {
  if (mm == 0) return HAPTIC_PWM_OFF;
  if (mm <= OBSTACLE_NEAR_MM) return HAPTIC_PWM_NEAR;
  if (mm <= OBSTACLE_MID_MM)  return HAPTIC_PWM_MID;
  if (mm <= OBSTACLE_FAR_MM)  return HAPTIC_PWM_FAR;
  return HAPTIC_PWM_OFF;
}

/*********** Tasks ***********/
void taskScanRig1(void *pv) {
  bool forward = true;
  for (;;) {
    for (int v = 0; v < V_CELLS; ++v) {
      writeServo(CH_V_RIG1, vIndexToDeg(v), V_SWEEP_MAX_DEG);
      vTaskDelay(pdMS_TO_TICKS(8));
      if (forward) {
        for (int h = 0; h < H_CELLS; ++h) {
          writeServo(CH_H_RIG1, hIndexToDeg(h), H_SWEEP_MAX_DEG);
          vTaskDelay(pdMS_TO_TICKS(6));
          uint16_t dist = tof1.readRangeContinuousMillimeters();
          xSemaphoreTake(mapMutex, portMAX_DELAY);
          mapRig1[v][h] = (tof1.timeoutOccurred() ? 0 : dist);
          xSemaphoreGive(mapMutex);
        }
      } else {
        for (int h = H_CELLS-1; h >= 0; --h) {
          writeServo(CH_H_RIG1, hIndexToDeg(h), H_SWEEP_MAX_DEG);
          vTaskDelay(pdMS_TO_TICKS(6));
          uint16_t dist = tof1.readRangeContinuousMillimeters();
          xSemaphoreTake(mapMutex, portMAX_DELAY);
          mapRig1[v][h] = (tof1.timeoutOccurred() ? 0 : dist);
          xSemaphoreGive(mapMutex);
        }
      }
      forward = !forward;
    }
  }
}

void taskScanRig2(void *pv) {
  bool forward = false;
  for (;;) {
    for (int v = 0; v < V_CELLS; ++v) {
      writeServo(CH_V_RIG2, vIndexToDeg(v), V_SWEEP_MAX_DEG);
      vTaskDelay(pdMS_TO_TICKS(8));
      if (forward) {
        for (int h = 0; h < H_CELLS; ++h) {
          writeServo(CH_H_RIG2, hIndexToDeg(h), H_SWEEP_MAX_DEG);
          vTaskDelay(pdMS_TO_TICKS(6));
          uint16_t dist = tof2.readRangeContinuousMillimeters();
          xSemaphoreTake(mapMutex, portMAX_DELAY);
          mapRig2[v][h] = (tof2.timeoutOccurred() ? 0 : dist);
          xSemaphoreGive(mapMutex);
        }
      } else {
        for (int h = H_CELLS-1; h >= 0; --h) {
          writeServo(CH_H_RIG2, hIndexToDeg(h), H_SWEEP_MAX_DEG);
          vTaskDelay(pdMS_TO_TICKS(6));
          uint16_t dist = tof2.readRangeContinuousMillimeters();
          xSemaphoreTake(mapMutex, portMAX_DELAY);
          mapRig2[v][h] = (tof2.timeoutOccurred() ? 0 : dist);
          xSemaphoreGive(mapMutex);
        }
      }
      forward = !forward;
    }
  }
}

void taskHaptics(void *pv) {
  for (;;) {
    uint16_t r_front=0, r_back=0, r_right=0, r_left=0;
    xSemaphoreTake(mapMutex, portMAX_DELAY);
    r_right = minDistanceInWindow(mapRig1, 135, 20);
    r_left  = minDistanceInWindow(mapRig2, 135, 20);
    uint16_t f1 = minDistanceInWindow(mapRig1, 90, 15);
    uint16_t f2 = minDistanceInWindow(mapRig2, 90, 15);
    r_front = (f1==0)?f2:((f2==0)?f1:min(f1,f2));
    uint16_t b1 = minDistanceInWindow(mapRig1, 180, 20);
    uint16_t b2 = minDistanceInWindow(mapRig2, 180, 20);
    r_back  = (b1==0)?b2:((b2==0)?b1:min(b1,b2));
    xSemaphoreGive(mapMutex);

    uint8_t pwmFront = levelFromDistance(r_front);
    uint8_t pwmRight = levelFromDistance(r_right);
    uint8_t pwmBack  = levelFromDistance(r_back);
    uint8_t pwmLeft  = levelFromDistance(r_left);

    analogWrite(PIN_HAPTIC_1, pwmFront);
    analogWrite(PIN_HAPTIC_2, pwmRight);
    analogWrite(PIN_HAPTIC_3, pwmBack);
    analogWrite(PIN_HAPTIC_4, pwmLeft);

    // LED logic:
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_BLUE, LOW);

    if (pwmRight > 0 && pwmLeft == 0) {
      digitalWrite(PIN_LED_RED, HIGH); // obstacle right
    } else if (pwmLeft > 0 && pwmRight == 0) {
      digitalWrite(PIN_LED_BLUE, HIGH); // obstacle left
    } else if (pwmFront > 0 && pwmBack == 0) {
      digitalWrite(PIN_LED_RED, HIGH);
      digitalWrite(PIN_LED_BLUE, HIGH); // both = front
      beep(60);
    } else if (pwmBack > 0 && pwmFront == 0) {
      digitalWrite(PIN_LED_RED, HIGH);
      beep(60);
      vTaskDelay(pdMS_TO_TICKS(100));
      digitalWrite(PIN_LED_RED, LOW);
      digitalWrite(PIN_LED_BLUE, HIGH);
      beep(60); // alternate = back
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void taskIMU(void *pv) {
  for (;;) {
    int16_t ax, ay, az, gx, gy, gz;
    imu.getAcceleration(&ax, &ay, &az);
    imu.getRotation(&gx, &gy, &gz);
    Serial.print(F("[IMU] Acc: "));
    Serial.print(ax); Serial.print(' ');
    Serial.print(ay); Serial.print(' ');
    Serial.print(az); Serial.print(F(" | Gyro: "));
    Serial.print(gx); Serial.print(' ');
    Serial.print(gy); Serial.print(' ');
    Serial.println(gz);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

/*********** Setup ***********/
void setup() {
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_HAPTIC_1, OUTPUT);
  pinMode(PIN_HAPTIC_2, OUTPUT);
  pinMode(PIN_HAPTIC_3, OUTPUT);
  pinMode(PIN_HAPTIC_4, OUTPUT);

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000UL);

  pwm.begin();
  pwm.setPWMFreq(SERVO_PWM_FREQ);

  imu.initialize();
  if (!imu.testConnection()) Serial.println(F("[IMU] FAIL"));
  else Serial.println(F("[IMU] OK"));

  initToFTwoSensors();

  memset(mapRig1, 0, sizeof(mapRig1));
  memset(mapRig2, 0, sizeof(mapRig2));
  mapMutex = xSemaphoreCreateMutex();

  xTaskCreate(taskScanRig1, "scan1", 150, NULL, 3, NULL);
  xTaskCreate(taskScanRig2, "scan2", 150, NULL, 3, NULL);
  xTaskCreate(taskHaptics,  "hap",   160, NULL, 2, NULL);
  xTaskCreate(taskIMU,      "imu",   180, NULL, 1, NULL);

  beep(100);
}

void loop() {}
