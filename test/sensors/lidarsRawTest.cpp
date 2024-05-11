#include <Wire.h>
#include <VL53L1X.h>
#include <Arduino.h>

VL53L1X LeftLidar, RightLidar;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire1.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  Wire1.setClock(400000);

  LeftLidar.setBus(&Wire1);

  LeftLidar.setTimeout(500);
  RightLidar.setTimeout(500);

  if (!LeftLidar.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  if (!RightLidar.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  LeftLidar.setDistanceMode(VL53L1X::Medium);
  LeftLidar.setMeasurementTimingBudget(33000);
  RightLidar.setDistanceMode(VL53L1X::Medium);
  RightLidar.setMeasurementTimingBudget(33000);

  // Start continuous readings at a rate of one measurement every 50 ms
  LeftLidar.startContinuous(33);
  RightLidar.startContinuous(33);
}

void loop()
{
  Serial.print("L: ");
  Serial.print(LeftLidar.read());
  if (LeftLidar.timeoutOccurred()) { Serial.print(" L TIMEOUT"); }

  Serial.print(" R: ");
  Serial.print(RightLidar.read());
  if (RightLidar.timeoutOccurred()) { Serial.print(" R TIMEOUT"); }

  Serial.println();
}