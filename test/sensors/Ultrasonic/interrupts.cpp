#include <Wire.h>
#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
#include <elapsedMillis.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"                 
#include "Servo.h"
#include "claw.h"
#include "HCSR04.h"

volatile unsigned long microinicial = 0;
volatile bool leyendo_ultrasonido = 0;
int distance = 0;
int contador_interrupcion = 0;

const int Trigger1 = 8;
const int Echo1 = 9;

void medir(){
  digitalWrite(Trigger1, HIGH);
  delayMicroseconds(10); 
  digitalWrite(Trigger1, LOW);
  noInterrupts();
  leyendo_ultrasonido = 1;
  microinicial = micros();
  interrupts();
}

void leer_distancia(){
  unsigned long tiempo_transcurrido;
  contador_interrupcion ++;
  if (leyendo_ultrasonido == 1){
    leyendo_ultrasonido = 0;
    tiempo_transcurrido = micros() - microinicial;
    distance = (tiempo_transcurrido /2) /29;
  }
}

void setup() {
  attachInterrupt (digitalPinToInterrupt(9), leer_distancia, RISING);
  Serial.begin(115200);
  pinMode(8, OUTPUT);
  pinMode(9, INPUT);
}

void loop() {
  medir();
  delay(100);
  Serial.println(distance);
  Serial.print("contador");
  Serial.println(contador_interrupcion);
}