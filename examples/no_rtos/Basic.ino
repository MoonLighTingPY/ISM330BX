#include <Wire.h>
#include "ISM330BX.h"

// Example: Basic accelerometer and gyroscope without RTOS
ISM330BXSensor sensor(&Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Initialize sensor (no RTOS mode)
  sensor.initRTOS();  // stub in non-RTOS mode
  if (sensor.begin() != ISM330BX_STATUS_OK) {
    Serial.println("Sensor initialization failed!");
    while (1);
  }

  // Enable accelerometer and gyroscope
  sensor.enableAccelerometer();
  sensor.enableGyroscope();
  Serial.println("Basic IMU example started.");
}

void loop() {
  if (sensor.checkDataReady()) {
    int32_t accel[3];
    int32_t gyro[3];
    sensor.readAcceleration(accel);
    sensor.readGyroscope(gyro);

    Serial.print("Accel (mg): X="); Serial.print(accel[0]);
    Serial.print(" Y="); Serial.print(accel[1]);
    Serial.print(" Z="); Serial.println(accel[2]);

    Serial.print("Gyro (mdps): X="); Serial.print(gyro[0]);
    Serial.print(" Y="); Serial.print(gyro[1]);
    Serial.print(" Z="); Serial.println(gyro[2]);

    Serial.println();
  }
  delay(100);
}
