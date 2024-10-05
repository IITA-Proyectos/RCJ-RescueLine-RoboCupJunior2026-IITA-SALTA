#include <Wire.h>
#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
#include <elapsedMillis.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"

// CONSTANTS //
#define FORWARD 0
#define BACKWARD 1
#define BUZZER 33
#define LED_ROJO 39
#define SWITCH 32

// INITIALISE BNO055 //
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// INITIALISE ACTUATORS //
Moto bl(29, 28, 27); // pwm, dir, enc
Moto fl(7, 6, 5);
Moto br(36, 37, 38);
Moto fr(4, 3, 2);
DriveBase robot(&fl, &fr, &bl, &br);

// STATE VARIABLES & FLAGS //
int serial5state = 0; // serial code e.g. 255
double speed = 0;        // speed (0 to 100)
double steer = 0;        // angle (0 to 180 deg, will -90 later)
int incoming_task = 0; // 0 = no green squares, 1 = left, 2 = right, 3 = double, 4 = pick up cube, 10 = deposit
int line_middle = 0; // if there is a line to reacquire after obstacle

// ISR for updating motor pulses
void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }

void serialEvent5() {
    while (Serial5.available()) {
        int data = Serial5.read(); // read serial code
        if (data == 255) { // speed incoming
            serial5state = 0;
        } else if (data == 254) { // steer incoming
            serial5state = 1;
        } else if (data == 253) { // task incoming
            serial5state = 2;
        } else if (serial5state == 0) { // set speed
            speed = (double)data / 100 * 100; // max speed = 100
        } else if (serial5state == 1) { // set steer
            steer = ((double)data - 90) / 90;
        } else if (serial5state == 2) { // set task
            incoming_task = data;
        } else if (serial5state == 3) { // set line_middle
            line_middle = data;
        }
    }
}

// HELPER FUNCTIONS //
void runTime(int speed, int dir, double steer, unsigned long long time) {
    unsigned long long startTime = millis();
    while ((millis() - startTime) < time) {
        robot.steer(speed, dir, steer);
        digitalWrite(13, HIGH);
        if (digitalRead(32) == 1) // switch is off
            Serial5.write(255);
    }
    digitalWrite(13, LOW);
}

void setup() {
    robot.steer(0, 0, 0);
    attachInterrupt(digitalPinToInterrupt(27), ISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(5), ISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(38), ISR3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(2), ISR4, CHANGE);
    pinMode(SWITCH, INPUT_PULLUP); // SWITCH
    pinMode(BUZZER, OUTPUT); // BUZZER
    Serial5.begin(115200);
    Serial.println("Teensy Ready!");
}

void loop() {
    // BNO Data Collection
    sensors_event_t event;
    bno.getEvent(&event);

    // Update Motor States
    serialEvent5();
    robot.steer(speed, 0, steer);

    // Further processing can be done here...
}
