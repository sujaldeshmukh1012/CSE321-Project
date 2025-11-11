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

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <AltSoftSerial.h>
#include <TinyGPSPlus.h>

#include <VL53L0X.h>            // Pololu library
#include <MPU6050.h>            // uses Wire

/*********** Pin Map ***********/
#define PIN_LED_RED      A0
#define PIN_LED_BLUE     A1
#define PIN_BUTTON       A2   // INPUT_PULLUP
#define PIN_BUZZER       7

#define PIN_HAPTIC_1     3    // PWM (Front)
#define PIN_HAPTIC_2     5    // PWM (Right)
#define PIN_HAPTIC_3     6    // PWM (Back)
#define PIN_HAPTIC_4     11   // PWM (Left)

#define PIN_TOF1_XSHUT   10
#define PIN_TOF2_XSHUT   12

// GPS on AltSoftSerial (Nano: RX=8, TX=9 fixed by library)
AltSoftSerial gpsSerial;        // RX(D8) ← GPS TX,  TX(D9) → GPS RX (optional)
/*******************************/

/*********** PCA9685 ***********/
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// 50 Hz for analog servos
#define SERVO_PWM_FREQ      50

// PCA9685 resolution and pulse mapping
// 1 tick ≈ (1/50Hz)/4096 ≈ 4.88us
// We'll map [500..2500] us to ticks
#define US_TO_TICKS(us)     (uint16_t)((float)(us) * SERVO_PWM_FREQ * 4096.0f / 1000000.0f)

// Tunables per-servo (adjust to your actual servos!)
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2500

// application angles
#define H_SWEEP_MAX_DEG     270.0f  // channels 0,1
#define V_SWEEP_MAX_DEG      90.0f  // channels 2,3

// channels
#define CH_H_RIG1            0
#define CH_H_RIG2            1
#define CH_V_RIG1            2
#define CH_V_RIG2            3

/*********** ToF ***********/
VL53L0X tof1, tof2;
#define TOF1_ADDR            0x29      // after reassignment can remain 0x29
#define TOF2_ADDR            0x2A      // second sensor gets 0x2A

// Ranging settings (trade range vs speed)
#define TOF_TIMING_BUDGET_US 20000UL    // 20 ms per sample (decent frame rate). You can lower to 10000 for faster, shorter-range.

/*********** IMU ***********/
MPU6050 imu;

/*********** GPS ***********/
TinyGPSPlus gps;

/*********** FreeRTOS Sync ***********/
SemaphoreHandle_t mapMutex;

/*********** Mapping (memory-safe coarse grid) ***********/
// Coarse grid to fit Nano RAM (2KB). 27x9 = 243 per rig. uint16_t => 486 bytes each.
// Two rigs = ~972 bytes + metadata. Still tight but OK.
#define H_CELLS   18   // 270° / 10°
#define V_CELLS    6   //  90° / 10°

static uint16_t mapRig1[V_CELLS][H_CELLS];  // mm; 0 = invalid
static uint16_t mapRig2[V_CELLS][H_CELLS];  // mm; 0 = invalid

// thresholds (mm) for haptic feedback
#define OBSTACLE_NEAR_MM     350
#define OBSTACLE_MID_MM      700
#define OBSTACLE_FAR_MM     1200

// PWM intensity for haptics [0..255]
#define HAPTIC_PWM_NEAR      255
#define HAPTIC_PWM_MID       160
#define HAPTIC_PWM_FAR        80
#define HAPTIC_PWM_OFF         0

/*********** Utilities ***********/
static inline uint16_t clamp16(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (uint16_t)v;
}

// angle → microseconds (linear map). Adjust end-points to match your servo travel.
// For a 270° servo across [SERVO_MIN_US..SERVO_MAX_US].
static uint16_t angleToUs(float angleDeg, float maxDeg) {
  if (angleDeg < 0) angleDeg = 0;
  if (angleDeg > maxDeg) angleDeg = maxDeg;
  float t = angleDeg / maxDeg; // 0..1
  float us = SERVO_MIN_US + t * (SERVO_MAX_US - SERVO_MIN_US);
  return (uint16_t)us;
}

static void writeServo(uint8_t channel, float angleDeg, float maxDeg) {
  uint16_t us = angleToUs(angleDeg, maxDeg);
  uint16_t ticks = US_TO_TICKS(us);
  pwm.setPWM(channel, 0, ticks);
}

static void beep(uint16_t ms) {
  tone(PIN_BUZZER, 3000);  // ~3kHz
  vTaskDelay(pdMS_TO_TICKS(ms));
  noTone(PIN_BUZZER);
}

/*********** ToF bring-up with XSHUT ***********/
static void initToFTwoSensors() {
  pinMode(PIN_TOF1_XSHUT, OUTPUT);
  pinMode(PIN_TOF2_XSHUT, OUTPUT);

  // Hold both in reset
  digitalWrite(PIN_TOF1_XSHUT, LOW);
  digitalWrite(PIN_TOF2_XSHUT, LOW);
  vTaskDelay(pdMS_TO_TICKS(10));

  // Start ToF1 only
  digitalWrite(PIN_TOF1_XSHUT, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));
  tof1.init(true);
  tof1.setAddress(TOF1_ADDR);
  tof1.setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
  tof1.startContinuous();

  // Start ToF2 only
  digitalWrite(PIN_TOF2_XSHUT, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));
  tof2.init(true);
  tof2.setAddress(TOF2_ADDR);
  tof2.setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
  tof2.startContinuous();
}

/*********** Angle grid helpers ***********/
static float hIndexToDeg(int h) { // 0..H_CELLS-1 -> 0..270
  return (H_SWEEP_MAX_DEG * h) / (H_CELLS - 1);
}
static float vIndexToDeg(int v) { // 0..V_CELLS-1 -> 0..90
  return (V_SWEEP_MAX_DEG * v) / (V_CELLS - 1);
}

/*********** Sector extraction (heuristic windows) ***********/
static uint16_t minDistanceInWindow(uint16_t grid[V_CELLS][H_CELLS], int hCenterDeg, int hHalfWidthDeg) {
  int minmm = 0; // 0 means none found
  for (int v = V_CELLS/3; v <= (2*V_CELLS)/3; ++v) { // focus near middle vertical third
    for (int h = 0; h < H_CELLS; ++h) {
      float ang = hIndexToDeg(h);
      if (ang >= (hCenterDeg - hHalfWidthDeg) && ang <= (hCenterDeg + hHalfWidthDeg)) {
        uint16_t d = grid[v][h];
        if (d != 0) {
          if (minmm == 0 || d < minmm) minmm = d;
        }
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
// Task: Rig1 scan (servos 0 & 2 with tof1)
void taskScanRig1(void *pv) {
  bool forward = true;
  for (;;) {
    for (int v = 0; v < V_CELLS; ++v) {
      float vdeg = vIndexToDeg(v);
      writeServo(CH_V_RIG1, vdeg, V_SWEEP_MAX_DEG);
      vTaskDelay(pdMS_TO_TICKS(8));

      if (forward) {
        for (int h = 0; h < H_CELLS; ++h) {
          float hdeg = hIndexToDeg(h);
          writeServo(CH_H_RIG1, hdeg, H_SWEEP_MAX_DEG);
          vTaskDelay(pdMS_TO_TICKS(6)); // settle a bit

          uint16_t dist = tof1.readRangeContinuousMillimeters();
          xSemaphoreTake(mapMutex, portMAX_DELAY);
          mapRig1[v][h] = (tof1.timeoutOccurred() ? 0 : dist);
          xSemaphoreGive(mapMutex);
        }
      } else {
        for (int h = H_CELLS-1; h >= 0; --h) {
          float hdeg = hIndexToDeg(h);
          writeServo(CH_H_RIG1, hdeg, H_SWEEP_MAX_DEG);
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

// Task: Rig2 scan (servos 1 & 3 with tof2)
void taskScanRig2(void *pv) {
  bool forward = false;
  for (;;) {
    for (int v = 0; v < V_CELLS; ++v) {
      float vdeg = vIndexToDeg(v);
      writeServo(CH_V_RIG2, vdeg, V_SWEEP_MAX_DEG);
      vTaskDelay(pdMS_TO_TICKS(8));

      if (forward) {
        for (int h = 0; h < H_CELLS; ++h) {
          float hdeg = hIndexToDeg(h);
          writeServo(CH_H_RIG2, hdeg, H_SWEEP_MAX_DEG);
          vTaskDelay(pdMS_TO_TICKS(6));

          uint16_t dist = tof2.readRangeContinuousMillimeters();
          xSemaphoreTake(mapMutex, portMAX_DELAY);
          mapRig2[v][h] = (tof2.timeoutOccurred() ? 0 : dist);
          xSemaphoreGive(mapMutex);
        }
      } else {
        for (int h = H_CELLS-1; h >= 0; --h) {
          float hdeg = hIndexToDeg(h);
          writeServo(CH_H_RIG2, hdeg, H_SWEEP_MAX_DEG);
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

// Task: Haptics (reads maps, drives motors)
void taskHaptics(void *pv) {
  for (;;) {
    uint16_t r_front=0, r_back=0, r_right=0, r_left=0;

    xSemaphoreTake(mapMutex, portMAX_DELAY);
    // Windows (deg): center ± halfwidth
    // Right  from Rig1 around 135°
    r_right = minDistanceInWindow(mapRig1, 135, 20);
    // Left   from Rig2 around 135°
    r_left  = minDistanceInWindow(mapRig2, 135, 20);
    // Front: around ~90° (use best of both)
    uint16_t f1 = minDistanceInWindow(mapRig1, 90, 15);
    uint16_t f2 = minDistanceInWindow(mapRig2, 90, 15);
    r_front = (f1==0)?f2:((f2==0)?f1:min(f1,f2));
    // Back: around ~180°
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

    // LEDs as simple status: blue if any obstacle in mid range; red if near
    bool anyNear = (pwmFront==HAPTIC_PWM_NEAR || pwmRight==HAPTIC_PWM_NEAR || pwmBack==HAPTIC_PWM_NEAR || pwmLeft==HAPTIC_PWM_NEAR);
    bool anyMid  = (pwmFront || pwmRight || pwmBack || pwmLeft);

    digitalWrite(PIN_LED_RED,  anyNear ? HIGH : LOW);
    digitalWrite(PIN_LED_BLUE, anyMid  ? HIGH : LOW);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Task: GPS read + print
void taskGPS(void *pv) {
  for (;;) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isUpdated()) {
      Serial.print(F("[GPS] "));
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
      Serial.print(F(" | sats="));
      Serial.print(gps.satellites.value());
      Serial.print(F(" | hdop="));
      Serial.println(gps.hdop.hdop());
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

// Task: IMU read + print (simple)
void taskIMU(void *pv) {
  for (;;) {
    imu.getMotion6(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);

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

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Task: Button + Buzzer heartbeat
void taskUI(void *pv) {
  bool lastBtn = HIGH;
  for (;;) {
    bool btn = digitalRead(PIN_BUTTON);
    if (btn == LOW && lastBtn == HIGH) { // press edge
      beep(80);
    }
    lastBtn = btn;

    // tiny tick on buzzer every 3 seconds as alive
    static uint32_t t0 = 0;
    if (millis() - t0 > 3000) {
      tone(PIN_BUZZER, 2200);
      vTaskDelay(pdMS_TO_TICKS(20));
      noTone(PIN_BUZZER);
      t0 = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Task: Telemetry (periodic concise dump)
void taskTelemetry(void *pv) {
  for (;;) {
    xSemaphoreTake(mapMutex, portMAX_DELAY);
    // Print a thin slice for sanity: middle row from both rigs
    int vmid = V_CELLS / 2;
    Serial.print(F("[RIG1 mid-row mm]: "));
    for (int h=0; h<H_CELLS; ++h) { Serial.print(mapRig1[vmid][h]); Serial.print(' '); }
    Serial.println();

    Serial.print(F("[RIG2 mid-row mm]: "));
    for (int h=0; h<H_CELLS; ++h) { Serial.print(mapRig2[vmid][h]); Serial.print(' '); }
    Serial.println();
    xSemaphoreGive(mapMutex);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/*********** Setup ***********/
void setup() {
  // IO
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);

  pinMode(PIN_HAPTIC_1, OUTPUT);
  pinMode(PIN_HAPTIC_2, OUTPUT);
  pinMode(PIN_HAPTIC_3, OUTPUT);
  pinMode(PIN_HAPTIC_4, OUTPUT);

  analogWrite(PIN_HAPTIC_1, 0);
  analogWrite(PIN_HAPTIC_2, 0);
  analogWrite(PIN_HAPTIC_3, 0);
  analogWrite(PIN_HAPTIC_4, 0);

  Serial.begin(115200);
  while (!Serial) { ; }

  Wire.begin();               // A4/A5
  Wire.setClock(400000UL);    // fast-mode I2C where possible

  // PCA9685
  pwm.begin();
  pwm.setPWMFreq(SERVO_PWM_FREQ);
  delay(10);

  // Center servos at mid
  writeServo(CH_H_RIG1, H_SWEEP_MAX_DEG/2, H_SWEEP_MAX_DEG);
  writeServo(CH_H_RIG2, H_SWEEP_MAX_DEG/2, H_SWEEP_MAX_DEG);
  writeServo(CH_V_RIG1, V_SWEEP_MAX_DEG/2, V_SWEEP_MAX_DEG);
  writeServo(CH_V_RIG2, V_SWEEP_MAX_DEG/2, V_SWEEP_MAX_DEG);

  // IMU
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println(F("[IMU] connection FAILED"));
  } else {
    Serial.println(F("[IMU] OK"));
  }

  // GPS
  gpsSerial.begin(9600);

  // ToF
  initToFTwoSensors();
  Serial.println(F("[ToF] both sensors initialized"));

  // Map init
  memset(mapRig1, 0, sizeof(mapRig1));
  memset(mapRig2, 0, sizeof(mapRig2));

  // Mutex
  mapMutex = xSemaphoreCreateMutex();

  // Tasks
  xTaskCreate(taskScanRig1, "scan1", 150, NULL, 3, NULL);
  xTaskCreate(taskScanRig2, "scan2", 150, NULL, 3, NULL);

  xTaskCreate(taskHaptics,  "hap",   160, NULL, 2, NULL);
  xTaskCreate(taskGPS,      "gps",   180, NULL, 1, NULL);
  xTaskCreate(taskIMU,      "imu",   180, NULL, 1, NULL);
  xTaskCreate(taskUI,       "ui",    140, NULL, 1, NULL);
  // xTaskCreate(taskTelemetry,"tel",   200, NULL, 1, NULL);

  // Small startup beep
  beep(120);
}

void loop() {
  // unused — FreeRTOS scheduler runs the tasks
}
