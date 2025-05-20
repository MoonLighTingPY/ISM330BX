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
- **Basic**: Read raw accelerometer and gyroscope data. See `examples/Basic/Basic.ino`.
- **SensorFusion**: Read gravity vector using built-in sensor fusion. See `examples/SensorFusion/SensorFusion.ino`.

## License
This project is licensed under the MIT License.
