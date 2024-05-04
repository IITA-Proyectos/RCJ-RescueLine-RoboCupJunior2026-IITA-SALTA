#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>

// https://www.arduino.cc/reference/en/
// https://www.pjrc.com/teensy/td_timing_IntervalTimer.html
// https://www.arduino.cc/en/Hacking/libraryTutorial
Moto bl(4, 3, 2);
Moto fl(35, 34, 36); // pwm dir enc
Moto br(7, 6, 5);
Moto fr(30, 28, 29);
DriveBase robot(&fl, &fr, &bl, &br);

void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(36), ISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), ISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(29), ISR4, CHANGE);
}

int counter = 0;
void loop()
{
  // put your main code here, to run repeatedly:
  robot.steer(20, LOW, 0);
}