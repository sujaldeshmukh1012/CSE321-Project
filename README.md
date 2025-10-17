
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
