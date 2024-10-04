#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
#include <claw.h>
#include <Ultrasonic.h>

DFServo sort(20, 540, 2390, 274);
DFServo left(14, 540, 2390, 274);
DFServo right(21, 540, 2390, 274);
DFServo lift(22, 540, 2390, 274);
DFServo deposit(23, 540, 2390, 274);
Claw claw(&lift, &left, &right, &sort, &deposit);

// Fr
Ultrasonic front_ultrasonic(8, 9);
int distance;
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    claw.lower();
}

void loop()
{
  distance = front_ultrasonic.read();
  Serial.print("Distance in CM: ");
  Serial.println(distance);
  delay(1000);
}