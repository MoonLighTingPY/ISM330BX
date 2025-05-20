#include <Wire.h>
#include "ISM330BX.h"

// Example: Demonstrate all gravity filters without RTOS
ISM330BXSensor sensor(&Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  sensor.begin();
  sensor.enableSensorFusion();

  // Serial.println("Threshold filter (300 mg)");
  // sensor.enableGravityFilter(GRAVITY_FILTER_THRESHOLD);
  // sensor.configureThreshold(300);
  // delay(2000);

  // Serial.println("Low-pass filter (alpha=0.5)");
  // sensor.enableGravityFilter(GRAVITY_FILTER_LOWPASS);
  // sensor.configureAlpha(0.5f);
  // delay(2000);

  Serial.println("Hybrid filter (threshold=200, alpha=0.7)");
  sensor.enableGravityFilter(GRAVITY_FILTER_HYBRID);
  sensor.configureThreshold(200);
  sensor.configureAlpha(0.7f);

  Serial.println("Filtering demo started.");
}

void loop() {
  if (sensor.checkDataReady() && sensor.checkGravityDataReady()) {
    int32_t g[3]; sensor.readGravityVector(g);
    Serial.print("Filtered Gravity: X="); Serial.print(g[0]); Serial.print(" Y="); Serial.print(g[1]); Serial.print(" Z="); Serial.println(g[2]);
    Serial.println();
  }
  delay(200);
}
