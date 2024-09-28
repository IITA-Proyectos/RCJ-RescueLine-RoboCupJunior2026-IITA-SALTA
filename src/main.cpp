#include <Wire.h>
#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
#include <elapsedMillis.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"  
#include <Ultrasonic.h>                        
// CONSTANTS //
#define FORWARD 0 // Def direction ADELANTE
#define BACKWARD 1 // Def direction ATRAS
#define BUZZER 33 // Definicion de PIN BUZZER
#define LED_ROJO 39 // Definicion de PIN LED_ROJO
#define SWITCH 32 // Definicion de PIN SWITCH
// INITIALISE BNO055 //
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
// INITIALISE ACTUATORS //
Moto bl(29, 28, 27); // pwm, dir, enc
Moto fl(7, 6, 5);
Moto br(36, 37, 38);
Moto fr(4, 3, 2);
DriveBase robot(&fl, &fr, &bl, &br);
Ultrasonic ultrasonic(8, 9); // Front Ultrasonic HC-SR04 Sensor
// STATE VARIABLES & FLAGS //
int counter=0;
int laststeer=0;
int serial5state = 0; // serial code e.g. 255
double speed;        // speed (0 to 100)
double steer;        // angle (0 to 180 deg, will -90 later)
int green_state = 0; // 0 = no green squares, 1 = left, 2 = right, 3 = double
int line_middle = 0; // if there is a line to reacquire after obstacle
int action;          // action to take (part of a task)
bool taskDone = false; // if true, update current_task
int angle0;             // initial IMU reading
bool startUp = false;
float frontUSReading;
unsigned long long lastfk = millis(), lastdblgreen = millis();
int cccounter, leftLidarReading, rightLidarReading;
// ISR for updating motor pulses
void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }

// Read Data from Raspberry by Serial TX-RX
void serialEvent5() {
    if (Serial5.available() > 0) {
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
        else if (serial5state == 1) // set steer
            steer = ((double)data - 90) / 90;
        else if (serial5state == 2) // set task
            green_state = data;
       /* else if (serial5state == 3) // set line_middle
            line_middle = data;*/
        }
    }
    
// HELPER FUNCTIONS //

// Do a predefined move by time
void runTime(int speed, int dir, double steer, unsigned long long time) {
    unsigned long long startTime = millis();
    while ((millis() - startTime) < time) {
        robot.steer(speed, dir, steer);
        digitalWrite(13, HIGH);
        if (Serial5.available() > 0) {
        int lecturas = Serial5.read();
        Serial.print(lecturas);
        }
        
        if (digitalRead(32) == 1){ //switch is off
            Serial5.write(255);
            break;
            }
        }
        
    digitalWrite(13, LOW);
}

void runAngle(int speed, int dir, double angle) {
    sensors_event_t event;
    bno.getEvent(&event);
    float initialAngle = event.orientation.x;
    float targetAngle = initialAngle + angle;

    // Normalizar el ángulo objetivo al rango 0-360
    targetAngle = fmod(targetAngle, 360.0); 
    if (targetAngle < 0) targetAngle += 360;

    while (true) {
        bno.getEvent(&event);
        float currentAngle = event.orientation.x;
        if (digitalRead(32) == 1){ //switch is off
            Serial5.write(255);
            break;
        }

        // Calcular la diferencia más corta entre los ángulos
        float error = targetAngle - currentAngle;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        Serial.print("Error actual: ");
        Serial.println(fabs(error));

        if (fabs(error) <= 1.0) break; 

        // Lógica para manejar los 5 valores de ángulo específicos
        if (angle == 180) {
            // Girar 180 grados (media vuelta)
            robot.steer(speed, dir, 1); // Girar a la derecha
        } else if (angle == 90 || angle == -270) {
            // Girar 90 grados a la derecha
            if (error > 0 && error <= 180) {
                robot.steer(speed, dir, -1); 
            } else {
                robot.steer(speed, dir, 1); 
            }
        } else if (angle == -90 || angle == 270) {
            // Girar 90 grados a la izquierda
            if (error < 0 && error >= -180) {
                robot.steer(speed, dir, 1); 
            } else {
                robot.steer(speed, dir, -1); 
            }
        } else if (angle == 45 || angle == -315) {
            // Girar 45 grados a la derecha
            if (error > 0 && error <= 180) {
                robot.steer(speed, dir, -1); 
            } else {
                robot.steer(speed, dir, 1); 
            }
        } else if (angle == -45 || angle == 315) {
            // Girar 45 grados a la izquierda
            if (error < 0 && error >= -180) {
                robot.steer(speed, dir, 1); 
            } else {
                robot.steer(speed, dir, -1); 
            }
        }
        else if (angle > 0) {   
            robot.steer(speed, dir, -1); 
        }
        else if (angle < 0) {   
            robot.steer(speed, dir, 1); 
        }
    }
    robot.steer(0, FORWARD, 0);
}

void setup() {
    robot.steer (0,0,0);
    attachInterrupt(digitalPinToInterrupt(27), ISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(5), ISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(38), ISR3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(2), ISR4, CHANGE);
    pinMode(SWITCH, INPUT_PULLUP); // SWITCH
    pinMode(BUZZER, OUTPUT); // BUZZER
    pinMode(LED_ROJO, OUTPUT); // LED ROJO
    pinMode(LED_BUILTIN, OUTPUT); //  LED BUILT-IN for debugging
    Serial1.begin(57600);  // for reading IMU
    Serial5.begin(115200); // for reading data from rpi and state
    Serial.begin(115200);  // displays ultrasound ping result
    // Initialise BNO055
    if(!bno.begin()) {
        Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    bno.setExtCrystalUse(true);
}

void loop() {
    if (digitalRead(32) == 1) { // switch is off
        robot.steer(0, FORWARD, 0); // stop moving
        action = 7;
        startUp = false;
        taskDone = true;
        Serial5.write(255);
        while (true){
            robot.steer(0,0,0);
            digitalWrite(LED_BUILTIN, HIGH);
            //digitalWrite(BUZZER, HIGH);
            digitalWrite(LED_ROJO, HIGH);
            delay(500); 
            robot.steer(0,0,0);
            digitalWrite(LED_BUILTIN, LOW);
            digitalWrite(BUZZER, LOW);
            digitalWrite(LED_ROJO, LOW);
            delay(500);
            if (digitalRead(SWITCH) == 0){
                break;
            }
        }
    } else if (digitalRead(32) == 0 && !startUp) {
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(BUZZER, LOW);
        digitalWrite(LED_ROJO, LOW);
        runTime(20, BACKWARD, 0, 300);
        runTime(20, FORWARD, 0, 300);
        //Serial5.write(254);
        startUp = true;
        action =7;
    }
    else{
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(BUZZER, LOW);
        digitalWrite(LED_ROJO, HIGH);
        //int lectura = ultrasonic.read();
        
        /*if(steer<30 or steer>150){
            counter++;
        }
        
        if(laststeer<30 and steer>30 and counter>15){
            runTime(20,1,0.5,500);
            counter=0;
        }
        if(laststeer>150 and steer<150 and counter>15){
            
            runTime(20,1,-0.5,500);
            counter=0;
        }
        */
        if (taskDone) { // robot is currently not performing any task
            Serial.println("Incoming Task: ");
            Serial.println(green_state);
            if (green_state == 0){
                action = 7;
            }
            if(green_state==1){
                action=6;//verde izquierda
            }
            if(green_state==2){
                action=5;//verde derecha
            }
            if (green_state ==3){
                action=14;
            }
            /*
            if (lectura < 15 && lectura != 357)
            {

            action = 4;
            }
            */
            switch (action) {
                
                case 6: // first imu 90 deg turn
                    runTime(15,FORWARD,0, 500);
                    runTime(0,FORWARD,0, 500);
                    serialEvent5();
                    if (green_state == 1){
                        runTime(15,FORWARD,0, 1600); // Avanza 2.5 segundo
                        runAngle(15,FORWARD,-90);  // Gira
                        runTime(20,FORWARD,0, 300); // Avanza 300 msegundos
                        runTime(0,FORWARD,0, 2000);
                    }
                    break;
                    
                case 5: // arc turn with line_middle
                    runTime(15,FORWARD,0, 500);
                    runTime(0,FORWARD,0, 500);
                    serialEvent5();
                    if (green_state == 2){
                        runTime(15,FORWARD,0, 1600); // Avanza 1 segundo
                        runAngle(15,FORWARD,90);  // Gira
                        runTime(20,FORWARD,0, 300); // Avanza 300 msegundos
                        runTime(0,FORWARD,0, 2000);
                    }
                    break;
                
                /*case 2: // final one-wheel imu turn for obstacle
                    runAngle(50, FORWARD, -0.5, 90);
                    taskDone = true;
                    break;
                */
                //case 4: // Detecta algo con el ultrasonido
                //    runTime(0,FORWARD,0, 3000); //Se queda parado 3 segundos
                //    break;
                case 7: // linetrack
                    if (steer < -0.7){
                        runTime(15,FORWARD,0, 1000); // Avanza 1 segundo
                        runAngle(15,FORWARD,80);  // Gira
                        runTime(15,BACKWARD,0, 300); // Retrocede 500 msegundos
                    }if(steer > 0.7 ){
                        runTime(15,FORWARD,0, 1000); // Avanza 1 segundos
                        runAngle(15,FORWARD,-80); // Gira
                        runTime(15,BACKWARD,0, 300); // Retrocede 500 msegundos
                    }else
                    {
                        robot.steer(speed, FORWARD, steer);
                    }
                     break;
                case 14: // turn 180 deg for double green squares 
                    runTime(15,FORWARD,0, 500);
                    runTime(0,FORWARD,0, 500);
                    serialEvent5();
                    if (green_state == 3){
                        runTime(15,FORWARD,0, 500);
                        runAngle(30,FORWARD,180);
                    }
                     // Gira a la derecha 180°
                    break;
            }
        }
    }
}