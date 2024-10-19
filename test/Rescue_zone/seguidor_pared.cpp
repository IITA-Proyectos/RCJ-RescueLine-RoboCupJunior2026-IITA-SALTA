#include <Wire.h>
#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
#include <elapsedMillis.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"
#include <Ultrasonic.h>
#include <VL53L0X.h>
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

Ultrasonic front(8, 9); // right

int distance;

VL53L0X left_tof;  // Sensor 1
VL53L0X right_tof; // Sensor 2

// STATE VARIABLES & FLAGS //
int counter = 0;
int laststeer = 0;
int serial5state = 0;  // serial code e.g. 255
double speed;          // speed (0 to 100)
double steer;          // angle (0 to 180 deg, will -90 later)
int green_state = 0;   // 0 = no green squares, 1 = left, 2 = right, 3 = double
int line_middle = 0;   // if there is a line to reacquire after obstacle
int action;            // action to take (part of a task)
bool taskDone = false; // if true, update current_task
int angle0;            // initial IMU reading
bool wallRight = false;
bool startUp = false;
float frontUSReading;
int distance_left_tof = left_tof.readRangeContinuousMillimeters();
int distance_right_tof = right_tof.readRangeContinuousMillimeters();
unsigned int distanceFront = front.read();
unsigned long long lastfk = millis(), lastdblgreen = millis();
int cccounter, leftLidarReading, rightLidarReading;
// ISR for updating motor pulses
void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }

// Read Data from Raspberry by Serial TX-RX
void serialEvent5()
{
    if (Serial5.available() > 0)
    {
        int data = Serial5.read(); // read serial code
        Serial.println(data);
        if (data == 255) // speed incoming
            serial5state = 0;
        else if (data == 254) // steer incoming
            serial5state = 1;
        else if (data == 253) // task incoming
            serial5state = 2;
        /*else if (data == 252) // line_middle incoming
            serial5state = 3;*/
        else if (serial5state == 0)           // set speed
            speed = (double)data / 100 * 100; // max speed = 100
        else if (serial5state == 1)           // set steer
            steer = ((double)data - 90) / 90;
        else if (serial5state == 2) // set task
            green_state = data;
        /* else if (serial5state == 3) // set line_middle
             line_middle = data;*/
    }
}

// HELPER FUNCTIONS //

// Do a predefined move by time
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

    // Normalizar el ángulo objetivo al rango 0-360
    targetAngle = fmod(targetAngle, 360.0);
    if (targetAngle < 0)
        targetAngle += 360;

    while (true)
    {
        bno.getEvent(&event);
        float currentAngle = event.orientation.x;
        if (digitalRead(32) == 1)
        { // switch is off
            Serial5.write(255);
            break;
        }

        // Calcular la diferencia más corta entre los ángulos
        float error = targetAngle - currentAngle;
        if (error > 180)
            error -= 360;
        if (error < -180)
            error += 360;

        Serial.print("Error actual: ");
        Serial.println(fabs(error));

        if (fabs(error) <= 1.0)
            break;

        // Lógica para manejar los 5 valores de ángulo específicos
        if (angle == 180)
        {
            // Girar 180 grados (media vuelta)
            robot.steer(speed, dir, 1); // Girar a la derecha
        }
        else if (angle == 90 || angle == -270)
        {
            // Girar 90 grados a la derecha
            if (error > 0 && error <= 180)
            {
                robot.steer(speed, dir, -1);
            }
            else
            {
                robot.steer(speed, dir, 1);
            }
        }
        else if (angle == -90 || angle == 270)
        {
            // Girar 90 grados a la izquierda
            if (error < 0 && error >= -180)
            {
                robot.steer(speed, dir, 1);
            }
            else
            {
                robot.steer(speed, dir, -1);
            }
        }
        else if (angle == 45 || angle == -315)
        {
            // Girar 45 grados a la derecha
            if (error > 0 && error <= 180)
            {
                robot.steer(speed, dir, -1);
            }
            else
            {
                robot.steer(speed, dir, 1);
            }
        }
        else if (angle == -45 || angle == 315)
        {
            // Girar 45 grados a la izquierda
            if (error < 0 && error >= -180)
            {
                robot.steer(speed, dir, 1);
            }
            else
            {
                robot.steer(speed, dir, -1);
            }
        }
        else if (angle > 0)
        {
            robot.steer(speed, dir, -1);
        }
        else if (angle < 0)
        {
            robot.steer(speed, dir, 1);
        }
    }
    robot.steer(0, FORWARD, 0);
}

void setup()
{
    robot.steer(0, 0, 0);
    attachInterrupt(digitalPinToInterrupt(27), ISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(5), ISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(38), ISR3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(2), ISR4, CHANGE);
    pinMode(SWITCH, INPUT_PULLUP); // SWITCH
    pinMode(BUZZER, OUTPUT);       // BUZZER
    pinMode(LED_ROJO, OUTPUT);     // LED ROJO
    pinMode(LED_BUILTIN, OUTPUT);  // LED BUILT-IN for debugging
    Serial1.begin(57600);          // for reading IMU
    Serial5.begin(115200);         // for reading data from rpi and state
    Serial.begin(115200);          // displays ultrasound ping result
    Wire1.begin();                 // Initialize the first I2C bus
    Wire2.begin();                 // Initialize the second I2C bus
    left_tof.setBus(&Wire2);       // Assign the first bus to Sensor 1
    right_tof.setBus(&Wire1);      // Assign the second bus to Sensor 2

    left_tof.setAddress(0x30);  // Set unique address for Sensor 1
    right_tof.setAddress(0x30); // Set unique address for Sensor 2

    // Continue with your setup and loop functions as before

    left_tof.init();
    left_tof.setTimeout(500);
    left_tof.startContinuous();

    right_tof.init();
    right_tof.setTimeout(500);
    right_tof.startContinuous();
    // Initialise BNO055
    /*if (distanceRight < distanceLeft)
    {
        wallRight = true;
    }*/
    if (!bno.begin())
    {
        Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
            ;
    }
    bno.setExtCrystalUse(true);
}

void loop()
{
    if (digitalRead(32) == 1)
    {                               // switch is off
        robot.steer(0, FORWARD, 0); // stop moving
        action = 7;
        startUp = false;
        taskDone = true;
        Serial5.write(255);
        while (true)
        {
            robot.steer(0, 0, 0);
            digitalWrite(LED_BUILTIN, HIGH);
            // digitalWrite(BUZZER, HIGH);
            digitalWrite(LED_ROJO, HIGH);
            delay(500);
            robot.steer(0, 0, 0);
            digitalWrite(LED_BUILTIN, LOW);
            digitalWrite(BUZZER, LOW);
            digitalWrite(LED_ROJO, LOW);
            delay(500);
            if (digitalRead(SWITCH) == 0)
            {
                break;
            }
        }
    }
    else if (digitalRead(32) == 0 && !startUp)
    {
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(BUZZER, LOW);
        digitalWrite(LED_ROJO, LOW);
        runTime(20, BACKWARD, 0, 300);
        runTime(20, FORWARD, 0, 300);
        // Serial5.write(254);
        startUp = true;
        action = 7;
        distance_right_tof = left_tof.readRangeContinuousMillimeters();;
        distance_left_tof = left_tof.readRangeContinuousMillimeters();;
        if (distance_left_tof > distance_right_tof){
            wallRight = true;
            }
        else{
            wallRight= false;
            }
    
  
        
    }
    else
    {

        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(BUZZER, LOW);
        digitalWrite(LED_ROJO, HIGH);
        distance_left_tof = left_tof.readRangeContinuousMillimeters() ;
        distance_right_tof = right_tof.readRangeContinuousMillimeters();
        distanceFront = front.read();

        robot.steer(30, FORWARD, 0);

        if (wallRight == true){
            if(distance_right_tof > 170){
                robot.steer(30, FORWARD, 1);
            }
            else if(distance_right_tof < 130){
                robot.steer(30, FORWARD, -1);
            }}
        else{
            if(distance_left_tof > 170){
                robot.steer(30, FORWARD, 1);
            }
            else if(distance_left_tof < 130){
                robot.steer(30, FORWARD, -1);

        }
        }


        if (front.read() <= 15)
        {
            runTime(0, FORWARD, 0, 2000);
        

            if (distance_left_tof < 350)
            {
                runAngle(30, FORWARD, 90);
                runTime(0, FORWARD, 0, 2000);
            }

            else{
            {
                runAngle(30, FORWARD, -90);
                runTime(0, FORWARD, 0, 2000);
            }
        }


}}}