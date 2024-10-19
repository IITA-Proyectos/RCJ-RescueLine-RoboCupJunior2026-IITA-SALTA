Caso 2:

#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_I2CDevice.h>
#include "Adafruit_APDS9960.h"
#include <claw.h>
#include <Ultrasonic.h>
#include <drivebase.h>
#include <PID.h>
#include "lidar.h"
#include <Wire.h>
#include <elapsedMillis.h>
#include <Adafruit_BNO055.h>
#include "math.h"
#include <VL53L0X.h>

#define FORWARD 0
#define BACKWARD 1
#define BUZZER 33
#define LED_ROJO 39
#define SWITCH 32

Adafruit_APDS9960 apds;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

Moto bl(29, 28, 27);
Moto fl(7, 6, 5);
Moto br(36, 37, 38);
Moto fr(4, 3, 2);
DriveBase robot(&fl, &fr, &bl, &br);

VL53L0X left_tof;
VL53L0X right_tof;
Ultrasonic front_ultrasonic(8, 9);
int distance;

int counter = 0;
int laststeer = 0;
int serial5state = 0;
double speed;
double steer;
int green_state = 0;
int line_middle = 0;
int action;
bool taskDone = false;
int angle0;
bool startUp = false;
float frontUSReading;
unsigned int distanceFront = front_ultrasonic.read();
unsigned long long lastfk = millis(), lastdblgreen = millis();
int cccounter, leftLidarReading, rightLidarReading;

void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }

DFServo sort(20, 540, 2390, 274);
DFServo left(14, 540, 2390, 274);
DFServo right(21, 540, 2390, 274);
DFServo lift(22, 540, 2390, 274);
DFServo deposit(23, 540, 2390, 274);
Claw claw(&lift, &left, &right, &sort, &deposit);

struct Color
{
    const char *name;
    uint16_t r, g, b, c;
};

void serialEvent5()
{
    if (Serial5.available() > 0)
    {
        int data = Serial5.read();
        Serial.println(data);
        if (data == 255)
            serial5state = 0;
        else if (data == 254)
            serial5state = 1;
        else if (data == 253)
            serial5state = 2;
        else if (serial5state == 0)
            speed = (double)data / 100 * 100;
        else if (serial5state == 1)
            steer = ((double)data - 90) / 90;
        else if (serial5state == 2)
            green_state = data;
    }
}

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
        {
            Serial5.write(255);
            break;
        }
    }
    digitalWrite(13, LOW);
}

void runAngle(int speed, int dir, double angle)
{
    sensors_event_t event;
    bno.getEvent(&event);
    float initialAngle = event.orientation.x;
    float targetAngle = initialAngle + angle;
    targetAngle = fmod(targetAngle, 360.0);
    if (targetAngle < 0)
        targetAngle += 360;
    while (true)
    {
        bno.getEvent(&event);
        float currentAngle = event.orientation.x;
        if (digitalRead(32) == 1)
        {
            Serial5.write(255);
            break;
        }
        float error = targetAngle - currentAngle;
        if (error > 180)
            error -= 360;
        if (error < -180)
            error += 360;
        if (fabs(error) <= 1.0)
            break;
        if (angle == 180)
            robot.steer(speed, dir, 1);
        else if (angle == 90 || angle == -270)
        {
            if (error > 0 && error <= 180)
                robot.steer(speed, dir, -1);
            else
                robot.steer(speed, dir, 1);
        }
        else if (angle == -90 || angle == 270)
        {
            if (error < 0 && error >= -180)
                robot.steer(speed, dir, 1);
            else
                robot.steer(speed, dir, -1);
        }
        else if (angle == 45 || angle == -315)
        {
            if (error > 0 && error <= 180)
                robot.steer(speed, dir, -1);
            else
                robot.steer(speed, dir, 1);
        }
        else if (angle == -45 || angle == 315)
        {
            if (error < 0 && error >= -180)
                robot.steer(speed, dir, 1);
            else
                robot.steer(speed, dir, -1);
        }
        else if (angle > 0)
            robot.steer(speed, dir, -1);
        else if (angle < 0)
            robot.steer(speed, dir, 1);
    }
    robot.steer(0, FORWARD, 0);
}

Color known_colors[] = {
    {"Blanco", 29, 38, 49, 141},
    {"Negro", 2, 2, 3, 9},
    {"Verde", 3, 10, 7, 28},
    {"Rojo", 13, 3, 5, 25},
    {"Plateado", 14, 20, 34, 108}};

const char *get_color()
{
    uint16_t r, g, b, c;

    // Esperar a que los datos de color estén listos
    while (!apds.colorDataReady())
    {
        delay(5);
    }

    // Obtener los datos del sensor
    apds.getColorData(&r, &g, &b, &c);

    // Calcular el color más cercano utilizando mínimos cuadrados
    const char *closest_color = "Desconocido";
    uint32_t min_error = UINT32_MAX;

    // Imprimir los valores de R, G, B y Clear
    Serial.print("red: ");
    Serial.print(r);
    Serial.print(" green: ");
    Serial.print(g);
    Serial.print(" blue: ");
    Serial.print(b);
    Serial.print(" clear: ");
    Serial.println(c);

    return closest_color;
}

void setup()
{
    robot.steer(0, 0, 0);
    attachInterrupt(digitalPinToInterrupt(27), ISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(5), ISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(38), ISR3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(2), ISR4, CHANGE);
    pinMode(SWITCH, INPUT_PULLUP);
    pinMode(BUZZER, OUTPUT);
    pinMode(LED_ROJO, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial1.begin(57600);
    Serial5.begin(115200);
    Serial.begin(115200);
    claw.lower();
    Wire1.begin();
    Wire2.begin();
    left_tof.setBus(&Wire2);
    right_tof.setBus(&Wire1);
    left_tof.setAddress(0x30);
    right_tof.setAddress(0x30);
    left_tof.init();
    left_tof.setTimeout(500);
    left_tof.startContinuous();
    right_tof.init();
    right_tof.setTimeout(500);
    right_tof.startContinuous();
    if (!bno.begin())
    {
        while (1)
            ;
    }
    bno.setExtCrystalUse(true);
    Serial.begin(115200);
}

void loop()
{
    if (digitalRead(32) == 1)
    {
        robot.steer(0, FORWARD, 0);
        action = 7;
        startUp = false;
        taskDone = true;
        Serial5.write(255);
        while (true)
        {
            robot.steer(0, 0, 0);
            digitalWrite(LED_BUILTIN, HIGH);
            digitalWrite(LED_ROJO, HIGH);
            delay(500);
            robot.steer(0, 0, 0);
            digitalWrite(LED_BUILTIN, LOW);
            digitalWrite(BUZZER, LOW);
            digitalWrite(LED_ROJO, LOW);
            delay(500);
            if (digitalRead(SWITCH) == 0)
                break;
        }
    }
    else if (digitalRead(32) == 0 && !startUp)
    {
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(BUZZER, LOW);
        digitalWrite(LED_ROJO, LOW);
        runTime(20, BACKWARD, 0, 300);
        runTime(20, FORWARD, 0, 300);
        startUp = true;
        action = 7;
    }
    else
    {
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(BUZZER, LOW);
        digitalWrite(LED_ROJO, HIGH);
        int distance_left_tof = left_tof.readRangeContinuousMillimeters();
        int distance_right_tof = right_tof.readRangeContinuousMillimeters();
        robot.steer(20, FORWARD, 0);
        claw.lift();

        if (front_ultrasonic.read() < 16)
        {
            runTime(0, FORWARD, 0, 3000);

            if (distance_left_tof < 150)
                runAngle(20, FORWARD, 90);
            else if (distance_right_tof < 150)
                runAngle(20, FORWARD, -90);
        }

#define DISTANCE_THRESHOLD 15
        if (distance < DISTANCE_THRESHOLD)
        {
            robot.steer(0, FORWARD, 0);
            const char *color_detected = get_color();
            if (strcmp(color_detected, "Negro") == 1)
            {
                digitalWrite(BUZZER, HIGH);
                delay(1000);
                runTime(0, FORWARD, 0, 2000);
                runAngle(20, FORWARD, 90);
                robot.steer(20, FORWARD, 0);
                claw.lift();

                if (distance_right_tof > 150 && distance_left_tof > 150)
                {
                    runTime(0, FORWARD, 0, 2000);
                    runAngle(20, FORWARD, 135);
                    robot.steer(20, FORWARD, 0);
                }
            }
            digitalWrite(BUZZER, LOW);
        }
    }
}