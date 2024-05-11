#include <Arduino.h>

void setup()
{
  // put your setup code here, to run once:
  pinMode(32, INPUT_PULLUP);
  pinMode(13, OUTPUT);
}

void loop()
{
  digitalWrite(13, digitalRead(32));
}