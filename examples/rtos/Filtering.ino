// Example: Demonstrate all gravity filters with RTOS
#include <Arduino.h>
#include <Wire.h>
#include "ISM330BX.h"

ISM330BXSensor sensor(&Wire);

void imuTask(void *pvParameters) {
  (void)pvParameters;

  sensor.enableSensorFusion();

  // Serial.println("Threshold filter (300 mg)");
  // sensor.enableGravityFilter(GRAVITY_FILTER_THRESHOLD);
  // sensor.configureThreshold(300);
  // vTaskDelay(pdMS_TO_TICKS(2000));

  // Serial.println("Low-pass filter (alpha=0.5)");
  // sensor.enableGravityFilter(GRAVITY_FILTER_LOWPASS);
  // sensor.configureAlpha(0.5f);
  // vTaskDelay(pdMS_TO_TICKS(2000));

  Serial.println("Hybrid filter (threshold=200, alpha=0.7)");
  sensor.enableGravityFilter(GRAVITY_FILTER_HYBRID);
  sensor.configureThreshold(200);
  sensor.configureAlpha(0.7f);

  Serial.println("Filtering demo started.");
  for (;;) {
    if (sensor.checkDataReady() && sensor.checkGravityDataReady()) {
      int32_t g[3];
      sensor.readGravityVector(g);
      Serial.print("Filtered Gravity: X="); Serial.print(g[0]);
      Serial.print(" Y="); Serial.print(g[1]);
      Serial.print(" Z="); Serial.println(g[2]);
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

  xTaskCreate(imuTask, "IMU_Filter", 4096, NULL, 1, NULL);
}

void loop() {
}
