#include <Arduino.h>
#include "AS5600.h"

AS5600 as5600(&Wire1);

void setup() {
  Serial.begin(115200);
  Wire1.begin();
  as5600.begin(); // Software direction control disabled. Set to clockwise in hardware.
  Serial.print("Connected: ");
  Serial.println(as5600.isConnected());
}

uint32_t lastTime = 0;
void loop() {
  lastTime = millis();
  as5600.getCumulativePosition();
  if (millis() - lastTime <= 100) {
    Serial.print("0\t"); // Scale ardino serial plotter minimum
    Serial.println(as5600.getCumulativePosition());
  }
}