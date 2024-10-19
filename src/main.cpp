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

//SERVOS
DFServo sort(20, 540, 2390, 274);
DFServo left(14, 540, 2390, 274);
DFServo right(21, 540, 2390, 274);
DFServo lift(22, 540, 2390, 274);
DFServo deposit(23, 540, 2390, 274);
Claw claw(&lift, &left, &right, &sort, &deposit);

// CONSTANTS //
#define FORWARD 0 // Def direction ADELANTE
#define BACKWARD 1 // Def direction ATRAS
#define BUZZER 35 // Definicion de PIN BUZZER
#define LED_ROJO 34 // Definicion de PIN LED_ROJO
#define SWITCH 32 // Definicion de PIN SWITCH

// INITIALISE BNO055 //
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
// INITIALISE ACTUATORS //
Moto bl(29, 28, 27); // pwm, dir, enc
Moto fl(7, 6, 5);
Moto br(36, 37, 38);
Moto fr(4, 3, 2);
DriveBase robot(&fl, &fr, &bl, &br);
// STATE VARIABLES & FLAGS //
String color_detected;
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
String rutina = "linea";

int cccounter, leftLidarReading, rightLidarReading;
VL53L0X left_tof;   // Sensor 1
VL53L0X right_tof;  // Sensor 2
int distance_left_tof;
int distance_right_tof;
#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 50 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(8, 9, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(11, 10, MAX_DISTANCE), 
  NewPing(39, 33, MAX_DISTANCE)
};

int front_distance;
int left_distance;
int right_distance;


// -----------  FUNCTIONS  -----------
// ULTRASONIDOS FRENTE IZQ DER
void leer_ultrasonidos() {
    front_distance = sonar[0].ping_cm();
    left_distance = sonar[1].ping_cm(); 
    right_distance = sonar[2].ping_cm();
}

void imprimir_ultrasonidos() {
    Serial.print("|D: ");
    Serial.print(right_distance);
    Serial.println("cm ");
}


// TOF
void leer_tof(){
    distance_left_tof = left_tof.readRangeContinuousMillimeters();
    distance_right_tof = right_tof.readRangeContinuousMillimeters();
}


void imprimir_tof(){
    Serial.print("Distance Left: ");
    Serial.print(distance_left_tof);
    Serial.print("mm");

    if (left_tof.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

    Serial.print("   Distance Right: ");
    Serial.print(distance_right_tof);
    Serial.print("mm");

    if (right_tof.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

}

// Color Sensor
Adafruit_APDS9960 apds;
struct Color {
  String name;
  uint16_t r, g, b, c;
};

Color known_colors[] = {
  {"Blanco", 24, 31, 39, 113},
  {"Negro", 2, 2, 3, 9},
  {"Verde", 3, 10, 7, 28},
  {"Plateado", 19, 20, 36, 100}
};

// Función para leer los valores del sensor y determinar el color
String get_color() {
  uint16_t r, g, b, c;
  
  // Esperar a que los datos de color estén listos
  while (!apds.colorDataReady()) {
    delay(5);
  }

  // Obtener los datos del sensor
  apds.getColorData(&r, &g, &b, &c);

  // Calcular el color más cercano utilizando mínimos cuadrados
  String closest_color = "Desconocido";
  uint32_t min_error = UINT32_MAX;

  for (size_t i = 0; i < sizeof(known_colors) / sizeof(known_colors[0]); i++) {
    uint32_t error = pow(known_colors[i].r - r, 2) +
                     pow(known_colors[i].g - g, 2) +
                     pow(known_colors[i].b - b, 2) +
                     pow(known_colors[i].c - c, 2);
    if (error < min_error) {
      min_error = error;
      closest_color = known_colors[i].name;
    }
  }

  // Imprimir los valores de R, G, B y Clear
  /*
  Serial.print("red: ");
  Serial.print(r);
  Serial.print(" green: ");
  Serial.print(g);
  Serial.print(" blue: ");
  Serial.print(b);
  Serial.print(" clear: ");
  Serial.println(c);
  */

  return closest_color;
}

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
        else if (data == 252) // line_middle incoming
            serial5state = 3;
        else if (serial5state == 0)           // set speed
            speed = (double)data / 100 * 100; // max speed = 100
        else if (serial5state == 1) // set steer
            steer = ((double)data - 90) / 90;
        else if (serial5state == 2) // set task
            green_state = data;
        else if (serial5state == 3) // set line_middle
            line_middle = data;
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
    claw.lift();
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
    
    // Initialise APDS9960 Color Sensor
    if(!apds.begin()){
        Serial.println("failed to initialize device! Please check your wiring.");
    }
    else Serial.println("Device initialized!");

    //enable color sensign mode
    apds.enableColor(true);

    // Initialise TOF
    Wire1.begin();   // Initialize the first I2C bus
    Wire2.begin();  // Initialize the second I2C bus

    left_tof.setBus(&Wire2);   // Assign the first bus to Sensor 1
    right_tof.setBus(&Wire1); // Assign the second bus to Sensor 2

    left_tof.setAddress(0x30); // Set unique address for Sensor 1
    right_tof.setAddress(0x30); // Set unique address for Sensor 2

    // Continue with your setup and loop functions as before

    left_tof.init();
    left_tof.setTimeout(500);
    left_tof.startContinuous();

    right_tof.init();
    right_tof.setTimeout(500);
    right_tof.startContinuous();
}

void loop() {
    if (digitalRead(32) == 1) { // switch is off
        robot.steer(0, FORWARD, 0); // stop moving
        claw.lift();
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
        rutina = "linea";
        claw.lift();
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
       while (rutina == "linea" && digitalRead(32) == 0){

        color_detected = get_color();
        leer_tof();
        leer_ultrasonidos();
        /*
        if (color_detected == "Plateado"){
            runTime(20,FORWARD,0,100);
            color_detected = get_color();
            if (color_detected == "Plateado"){
            rutina = "rescate";
            break;
        }
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
            if (front_distance != 0 && front_distance < 20){
                action = 1;
            }
            switch (action) {
                case 1:
                    digitalWrite(BUZZER,HIGH);
                    delay(100);
                    digitalWrite(BUZZER,LOW);
                    runTime(0,FORWARD,0,2000);
                    
                    break;
                case 6: 
                    runTime(30,FORWARD,0,750);
                    serialEvent5();
                    if (green_state == 1){                        
                        runAngle(20,FORWARD,-90);
                        runTime(20,FORWARD,0,250);
                            
                    }
                    break;
                case 5: 
                    runTime(30,FORWARD,0,750);
                    serialEvent5();
                    if (green_state == 2){                        
                        runAngle(20,FORWARD,90);
                        runTime(20,FORWARD,0,250);
                    }
                    break;
                case 7: // linetrack
                    /*if (steer < -0.7){
                        runTime(15,FORWARD,0, 1000); // Avanza 1 segundo
                        runAngle(15,FORWARD,80);  // Gira
                        runTime(15,BACKWARD,0, 300); // Retrocede 500 msegundos
                    }if(steer > 0.7 ){
                        runTime(15,FORWARD,0, 1000); // Avanza 1 segundos
                        runAngle(15,FORWARD,-80); // Gira
                        runTime(15,BACKWARD,0, 300); // Retrocede 500 msegundos
                    }else
                    {*/
                        robot.steer(speed, FORWARD, steer);
                    //}
                     break;
                case 14: // turn 180 deg for double green squares 
                    serialEvent5();
                    if (green_state == 3){
                        runAngle(30,FORWARD,180);
                    }
                    action=7;
                    break;
                }
            }
        }
        while (rutina == "rescate" && digitalRead(32) == 0){
            claw.lower();
            for (int i = 0; i < 2; i++) {
                digitalWrite(LED_BUILTIN, HIGH);
                digitalWrite(BUZZER, HIGH);
                digitalWrite(LED_ROJO, HIGH);
                delay(200); 
                digitalWrite(LED_BUILTIN, LOW);
                digitalWrite(BUZZER, LOW);
                digitalWrite(LED_ROJO, LOW);
                delay(200); 
            }
            
            // Here the code for RESCUE
        }
    }
}