#include <Wire.h>
#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
#include <elapsedMillis.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"
#include <Servo.h>
#include <Adafruit_I2CDevice.h>
#include <claw.h>
#include "Adafruit_APDS9960.h"
#include <NewPing.h>
#include <Wire.h>
#include <VL53L0X.h>

#define FORWARD 0         // Def direction ADELANTE
#define BACKWARD 1        // Def direction ATRAS

Moto bl(29, 28, 27, "BL"); // pwm, dir, enc
Moto fl(7, 6, 5, "FL");
Moto br(36, 37, 38, "BR");
Moto fr(4, 3, 2, "FR");
DriveBase robot(&fl, &fr, &bl, &br);

double speed;          // speed (0 to 100)
double steer;          // ang

void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }

void runTime(int speed, int dir, double steer, unsigned long long time)
{
    unsigned long long startTime = millis();
    while ((millis() - startTime) < time)
    {
        robot.steer(speed, dir, steer);
        digitalWrite(13, HIGH);
        if (Serial5.available() > 0)
        {
            int lecturas = Serial5.read();
            Serial.print(lecturas);
        }

        if (digitalRead(32) == 1)
        { // switch is off
            Serial5.clear();
            Serial5.write(255);
            break;
        }
    }

    digitalWrite(13, LOW);
}

void setup()
{
    robot.steer(0, 0, 0);

    attachInterrupt(digitalPinToInterrupt(27), ISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(5), ISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(38), ISR3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(2), ISR4, CHANGE);
}
void loop()
{
    runTime(25,FORWARD,0,2000);
    delay(1000);
    runTime(25,BACKWARD,0,2000);
    delay(1000);
}