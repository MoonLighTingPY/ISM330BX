// Example: Full feature demonstration without RTOS
// Shows accelerometer, gyroscope, and gravity vector with hybrid filter
#include <Wire.h>
#include "ISM330BX.h"

ISM330BXSensor sensor(&Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }


  if (sensor.begin() != ISM330BX_STATUS_OK) {
    Serial.println("Sensor initialization failed!");
    while (1);
  }

  // Enable features
  sensor.enableAccelerometer();
  sensor.enableGyroscope();
  sensor.enableSensorFusion();   // enable gravity output
  sensor.enableGravityFilter(GRAVITY_FILTER_HYBRID);
  sensor.configureThreshold(400);
  sensor.configureAlpha(0.7f);

  Serial.println("Full feature IMU example started.");
}

void loop() {
  if (sensor.checkDataReady()) {
    int32_t accel[3], gyro[3], gravity[3];
    sensor.readAcceleration(accel);
    sensor.readGyroscope(gyro);
    if (sensor.checkDataReady() && sensor.checkGravityDataReady()) {
      sensor.readGravityVector(gravity);
    }

    Serial.print("Accel (mg): X="); Serial.print(accel[0]);
    Serial.print(" Y="); Serial.print(accel[1]);
    Serial.print(" Z="); Serial.println(accel[2]);

    Serial.print("Gyro (mdps): X="); Serial.print(gyro[0]);
    Serial.print(" Y="); Serial.print(gyro[1]);
    Serial.print(" Z="); Serial.println(gyro[2]);

    Serial.print("Gravity (mg): X="); Serial.print(gravity[0]);
    Serial.print(" Y="); Serial.print(gravity[1]);
    Serial.print(" Z="); Serial.println(gravity[2]);
    Serial.println();
  }
  delay(100);
}
