#include <Wire.h>
#include "ISM330BX.h"

// Example: Read raw gravity vector without reference
ISM330BXSensor sensor(&Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Initialize sensor and enable fusion for gravity vector
  if (sensor.begin() != ISM330BX_STATUS_OK) {
    Serial.println("Sensor init failed");
    while (1);
  }
  sensor.enableSensorFusion();

  Serial.println("Raw Gravity Vector example started");
}

void loop() {
  if (sensor.checkDataReady() && sensor.checkGravityDataReady()) {
    int32_t rawGravity[3];
    sensor.readRawGravityVector(rawGravity);

    Serial.print("Raw Gravity (mg): X="); Serial.print(rawGravity[0]);
    Serial.print(" Y="); Serial.print(rawGravity[1]);
    Serial.print(" Z="); Serial.println(rawGravity[2]);
    Serial.println();
  }
  delay(200);
}
