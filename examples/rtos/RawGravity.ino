// Example: Read raw gravity vector with RTOS
#include <Arduino.h>
#include <Wire.h>
#include "ISM330BX.h"
#define USE_FREERTOS

ISM330BXSensor sensor(&Wire);

void imuTask(void *pvParameters) {
  (void)pvParameters;
  int32_t rawGravity[3];

  sensor.enableSensorFusion();

  Serial.println("Raw Gravity Vector example started");

  for (;;) {
    if (sensor.checkDataReady() && sensor.checkGravityDataReady()) {
      sensor.readRawGravityVector(rawGravity);
      Serial.print("Raw Gravity (mg): X="); Serial.print(rawGravity[0]);
      Serial.print(" Y="); Serial.print(rawGravity[1]);
      Serial.print(" Z="); Serial.println(rawGravity[2]);
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

  xTaskCreate(imuTask, "IMU_RawGrav", 4096, NULL, 1, NULL);
}

void loop() {
}
