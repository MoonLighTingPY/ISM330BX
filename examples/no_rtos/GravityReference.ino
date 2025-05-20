#include <Wire.h>
#include "ISM330BX.h"

// Example: Calibrate and apply a custom gravity zero reference without RTOS
ISM330BXSensor sensor(&Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  if (sensor.begin() != ISM330BX_STATUS_OK) {
    Serial.println("Sensor init failed!");
    while (1);
  }
  sensor.enableSensorFusion();

  Serial.println("Place sensor flat & press key to set reference.");
  while (!Serial.available()) { delay(10); }
  sensor.setGravityReference();
  Serial.println("Reference set.");
}

void loop() {
  if (sensor.checkDataReady() && sensor.checkGravityDataReady()) {
    int32_t g[3]; sensor.readGravityVector(g);
    Serial.print("Gravity w/ref (mg): X="); Serial.print(g[0]); Serial.print(" Y="); Serial.print(g[1]); Serial.print(" Z="); Serial.println(g[2]);
    Serial.println();
  }
  delay(200);
}
