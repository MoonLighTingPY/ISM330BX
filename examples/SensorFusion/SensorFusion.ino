#include <Wire.h>
#include "ISM330BX.h"

// Create sensor object
ISM330BXSensor sensor(&Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Initialize RTOS resources
  if (!sensor.initRTOS()) {
    Serial.println("RTOS initialization failed!");
    while (1);
  }

  // Initialize and enable sensor fusion for gravity vector
  if (sensor.begin() != ISM330BX_STATUS_OK) {
    Serial.println("Sensor init failed!");
    while (1);
  }
  sensor.enableSensorFusion();

  // Optionally set a reference vector
  sensor.setGravityReference();

  Serial.println("Sensor fusion example started.");
}

void loop() {
  if (sensor.checkGravityDataReady()) {
    int32_t gravity[3];
    // Read gravity vector with reference and filter
    sensor.readGravityVector(gravity);

    Serial.print("Gravity (mg): X="); Serial.print(gravity[0]);
    Serial.print(" Y="); Serial.print(gravity[1]);
    Serial.print(" Z="); Serial.println(gravity[2]);
    Serial.println();
  }
  delay(200);
}
