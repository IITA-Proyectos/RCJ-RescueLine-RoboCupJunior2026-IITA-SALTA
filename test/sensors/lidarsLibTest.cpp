#include <Arduino.h>
#include <Wire.h>
#include "lidar.h"

Lidar left(&Wire1), right(&Wire);

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  Serial.print(left.getDist());
  Serial.print(" ");
  Serial.println(right.getDist());
}