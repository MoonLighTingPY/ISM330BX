// Example: Calibrate and apply a custom gravity zero reference with RTOS
#include <Arduino.h>
#include <Wire.h>
#include "ISM330BX.h"

ISM330BXSensor sensor(&Wire);

void imuTask(void *pvParameters) {
  (void)pvParameters;
  int32_t gravityVector[3];

  sensor.enableSensorFusion();

  Serial.println("Place sensor flat & reset gravity reference...");
  if (sensor.setGravityReference()) {
    sensor.applyGravityReference(gravityVector);
    Serial.println("Reference set.");
  }

  for (;;) {
    if (sensor.checkDataReady() && sensor.checkGravityDataReady()) {
      sensor.readGravityVector(gravityVector);
      Serial.print("Gravity w/ref (mg): X="); Serial.print(gravityVector[0]);
      Serial.print(" Y="); Serial.print(gravityVector[1]);
      Serial.print(" Z="); Serial.println(gravityVector[2]);
      Serial.println();
    }
    vTaskDelay(pdMS_TO_TICKS(200));
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

  xTaskCreate(imuTask, "IMU_GravRef", 4096, NULL, 1, NULL);
}

void loop() {
}
