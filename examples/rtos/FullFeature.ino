// Example: Full feature demonstration with RTOS
// Shows accelerometer, gyroscope, and gravity vector with hybrid filter in RTOS task
#include <Arduino.h>
#include <Wire.h>
#include "ISM330BX.h"

ISM330BXSensor sensor(&Wire);

void imuTask(void *pvParameters) {
  (void)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
  int32_t accel[3], gyro[3], gravity[3];

  // Initialize once before loop
  sensor.enableAccelerometer();
  sensor.enableGyroscope();
  sensor.enableSensorFusion();
  sensor.enableGravityFilter(GRAVITY_FILTER_HYBRID);
  sensor.configureThreshold(400);
  sensor.configureAlpha(0.7f);

  for (;;) {
    if (sensor.checkDataReady()) {
      sensor.readAcceleration(accel);
      sensor.readGyroscope(gyro);
      if (sensor.checkGravityDataReady()) {
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
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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

  xTaskCreate(imuTask, "IMU", 4096, NULL, 1, NULL);
}

void loop() {
}
