# ISM330BX Arduino Library

This library provides an Arduino port for the STMicroelectronics ISM330BX 6-axis IMU, originally designed for the STEVAL-MKI245KA evaluation kit.

## Features
- Read accelerometer, gyroscope, and gravity vector data
- Sensor fusion support for gravity vector calculation
- FreeRTOS-friendly with I2C mutexes
- Configurable gravity filtering (threshold, low-pass, hybrid)

## Requirements
- Arduino framework (1.8.10 or later)
- Wire library
- FreeRTOS library

## Installation
### Arduino IDE
1. Clone this repository into your Arduino `libraries` folder:
   ```bash
   git clone https://github.com/MoonLighTingPY/ISM330BX.git
   ```
2. Restart the Arduino IDE.

### PlatformIO
In your `platformio.ini`, add the following under `lib_deps`:
```ini
lib_deps =
    https://github.com/MoonLighTingPY/ISM330BX.git
```


## Examples

### RTOS Examples

- **Basic**: Read raw accelerometer and gyroscope data. See `examples/rtos/Basic.ino`.
- **SensorFusion**: Read gravity vector using sensor fusion. See `examples/rtos/SensorFusion.ino`.
- **RawGravity**: Read raw gravity vector without reference. See `examples/rtos/RawGravity.ino`.
- **GravityReference**: Calibrate and apply a custom gravity zero reference. See `examples/rtos/GravityReference.ino`.
- **Filtering**: Demonstrate threshold, low-pass, and hybrid gravity filters in one sketch. See `examples/rtos/Filtering.ino`.
- **FullFeature**: Combine accelerometer, gyroscope, sensor fusion, custom reference, and hybrid filtering. See `examples/rtos/FullFeature.ino`.

### No-RTOS Examples

- **Basic**: Read raw accelerometer and gyroscope data without RTOS. See `examples/no_rtos/Basic.ino`.
- **SensorFusion**: Read gravity vector using sensor fusion without RTOS. See `examples/no_rtos/SensorFusion.ino`.
- **RawGravity**: Read raw gravity vector without reference, no RTOS. See `examples/no_rtos/RawGravity.ino`.
- **GravityReference**: Calibrate and apply custom gravity zero reference without RTOS. See `examples/no_rtos/GravityReference.ino`.
- **Filtering**: Demonstrate all gravity filters without RTOS. See `examples/no_rtos/Filtering.ino`.
- **FullFeature**: All features combined (hybrid filter) without RTOS. See `examples/no_rtos/FullFeature.ino`.
