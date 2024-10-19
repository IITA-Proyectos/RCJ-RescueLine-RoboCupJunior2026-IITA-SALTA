
#include "Arduino.h"
#include "Adafruit_APDS9960.h"
#include "Adafruit_BNO055.h"
#include  <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <NewPing.h>

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

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}

void leer_ultrasonidos(){
    front_distance = sonar[0].ping_cm();
    left_distance = sonar[1].ping_cm(); 
    right_distance = sonar[2].ping_cm();
}

void imprimir_ultrasonidos(){
    Serial.print("F: ");
    Serial.print(front_distance);
    Serial.print("cm ");
    Serial.print("|I: ");
    Serial.print(left_distance);
    Serial.print("cm ");
    Serial.print("|D: ");
    Serial.print(right_distance);
    Serial.println("cm ");
}

void loop() { 
    leer_ultrasonidos();
    imprimir_ultrasonidos();
    delay(500);
}