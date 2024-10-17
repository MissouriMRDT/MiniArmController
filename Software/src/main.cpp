#include "AS5600.h"
#include <Arduino.h>

AS5600 as5600[2] = {AS5600(&Wire1), AS5600(&Wire2)};

void setup() {
  Serial.begin(115200);
  Wire1.begin();
  Wire2.begin();
  for (int i = 0; i < 2; i++) {
    as5600[i].begin(); // Software direction control disabled. Set to clockwise
                       // in hardware.
    Serial.print("Connected: ");
    Serial.println(as5600[i].isConnected());
  }
}

const uint32_t OUTPUT_INTERVAL = 500; // ms
uint32_t lastOutput = 0;
double deltaX = 0, deltaY = 0;
uint8_t moveX = 0, moveY = 0;
int32_t currentX = 0, currentY = 0, lastX = 0, lastY = 0;
double sensitivity = 5;

int8_t clamp(double x) {
  if (x >= INT8_MAX)
    return INT8_MAX;
  if (x <= INT8_MIN)
    return INT8_MIN;
  return x;
}

void loop() {
  currentX = as5600[0].getCumulativePosition();
  currentY = as5600[1].getCumulativePosition();
  deltaX += (currentX - lastX) / sensitivity;
  deltaY += (currentY - lastY) / sensitivity;
  if (deltaX < 1 || deltaX > 1 || deltaY < 1 || deltaY > 1) {
    int8_t moveX = clamp(deltaX);
    int8_t moveY = clamp(deltaY);
    Mouse.move(moveX, moveY);
    deltaX -= moveX;
    deltaY -= moveY;
  }
  lastX = currentX;
  lastY = currentY;

  if (millis() - lastOutput >= OUTPUT_INTERVAL) {
    lastOutput = millis();
    Serial.print('0'); // Scale ardino serial plotter minimum
    for (int i = 0; i < 2; i++) {
      Serial.print('\t');
      Serial.print(as5600[i].getCumulativePosition());
    }
    Serial.println();
  }
}