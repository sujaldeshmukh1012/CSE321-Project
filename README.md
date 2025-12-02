# Project Guardian: Smart Wearable Assistance Hat for the Visually Impaired

## Overview

This project is a real-time wearable system designed for visually impaired individuals. It combines ultrasonic obstacle detection, fall anomaly detection, sound-event monitoring, and automatic emergency dispatch using both an ESP32 and ESP32-CAM.

The system provides:
- Haptic vibration feedback for nearby obstacles
- Real-time IMU-based fall detection
- Microphone-based anomaly sound recognition
- Automatic SOS transmission with GPS, audio, and video
- Low-latency communication suitable for real-time response scenarios

## Features
### 1. Ultrasonic Obstacle Detection
- Three ultrasonic sensors (front, left, right)
- Distance-based vibration intensity mapping
- Processed at consistent real-time intervals (20–50ms)

### 2. Fall Detection using IMU
- Real-time accelerometer + gyroscope sampling
- Threshold-based anomaly state machine
- Detects forward falls, backward falls, side falls

### 3. Sound Event Monitoring (Microphone)
- Detects loud peaks or repeated distress patterns (>75dB)
- Triggers early-warning logging events

### 4. Emergency SOS System (ESP32 & ESP32-CAM)
- When a fall or sound event is confirmed:
- Sends GPS coordinates
- Streams live picture/video
- Uploads 5 seconds of audio
- Sends SOS alert to caregiver phone/server endpoint

### 5. Real-Time Architecture
- Sensor processing loop every 20ms
- IMU sampling at 100Hz
- Non-blocking asynchronous communication
- Fatigue-resistant haptic alerts

## Hardware Components

| Component                                     | Purpose                 |
| --------------------------------------------- | ----------------------- |
| ESP32 Dev Board                               | Core microcontroller    |
| ESP32-CAM                                     | Video + image capture   |
| Ultrasonic Sensors (HC-SR04)                  | Obstacle detection      |
| MPU6050 / MPU9250                             | IMU for fall detection  |
| Electret Microphone + MAX9814                 | Sound anomaly detection |
| Vibration Motor                               | Haptic feedback         |
| GPS Module (NEO-6M or ESP32 WiFi geolocation) | Location tracking       |
| Li-ion Battery + Boost Converter              | Power source            |

# Project Setup Guide (PlatformIO)

## Prerequisites
- **VS Code** installed
- **PlatformIO IDE extension** installed in VS Code  
  (Install via: *Extensions → Search “PlatformIO IDE” → Install*)

## Cloning the Repository
```bash
git clone git@github.com:sujaldeshmukh1012/CSE321-Project.git
cd CSE321-Project
```

## Open the Project

1. Launch **VS Code**.
2. Open the cloned folder (`CSE321-Project`).
3. PlatformIO will automatically detect the environment.

## Building the Project

```bash
pio run
```

## Uploading to Device

```bash
pio run --target upload
```

## Serial Monitor

```bash
pio device monitor
```

## Notes

* Configuration files are located in `platformio.ini`.
* Modify board, framework, and library settings inside `platformio.ini` before building if required.
* Ensure proper USB permissions for upload on Linux/macOS (`sudo usermod -a -G dialout $USER`).

```
```
