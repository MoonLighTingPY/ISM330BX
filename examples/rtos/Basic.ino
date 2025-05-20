// Example: Basic accelerometer and gyroscope with RTOS
#include <Arduino.h>
#include <Wire.h>
#include "ISM330BX.h"

ISM330BXSensor sensor(&Wire);

void imuTask(void *pvParameters) {
  (void)pvParameters;
  int32_t accel[3];
  int32_t gyro[3];
  for (;;) {
    if (sensor.checkDataReady()) {
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
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  if (!sensor.initRTOS()) {
    Serial.println("RTOS initialization failed!");
  }

  if (sensor.begin() != ISM330BX_STATUS_OK) {
    Serial.println("Sensor initialization failed!");
  }

  sensor.enableAccelerometer();
  sensor.enableGyroscope();

  xTaskCreate(imuTask, "IMU", 4096, NULL, 1, NULL);
}

void loop() {
  // Empty. All work done in imuTask.
}
