#include <Wire.h>
#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
#include <elapsedMillis.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "math.h"
#include <NewPing.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X left_tof;   // Sensor 1
VL53L0X right_tof;  // Sensor 2
int distance_left_tof;
int distance_right_tof;

// CONSTANTES
#define FORWARD 0
#define BACKWARD 1
#define TARGET_DISTANCE 5.0 // distancia deseada en cm
#define KP_DISTANCE 0.1      // constante proporcional para la distancia
#define KP_ANGLE 0.1        // constante proporcional para el ángulo de rotación
#define MAX_STEER 1        // valor máximo de steer permitido
#define ANGLE_THRESHOLD 5.0 // umbral de inclinación en grados (yaw)
#define TARGET_ANGLE 0       // ángulo objetivo (robot paralelo a la pared)

// PINES
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // inicializar BNO055

// Motores
Moto bl(29, 28, 27); // pwm, dir, enc
Moto fl(7, 6, 5);
Moto br(36, 37, 38);
Moto fr(4, 3, 2);
DriveBase robot(&fl, &fr, &bl, &br);

// VARIABLES
double steer = 0;  // Valor de dirección que vamos a ajustar
float distance = 0; // Distancia medida por el ultrasonido
unsigned long last_time = 0;
float yaw = 0;     // Ángulo de rotación (yaw)

// ISR para actualizar pulsos de motores
void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }

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

// Función para calcular la diferencia de ángulo en un rango circular de 0 a 360 grados
float calcularDiferenciaAngulo(float anguloActual, float anguloObjetivo) {
    float error = anguloObjetivo - anguloActual;

    // Ajustar la diferencia para que esté en el rango [-180, 180]
    if (error > 180) {
        error -= 360;
    } else if (error < -180) {
        error += 360;
    }

    return error;
}

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

void leer_yaw() {
    sensors_event_t event;
    bno.getEvent(&event);
    yaw = event.orientation.x; // Yaw es el ángulo de rotación (en grados)
}

void imprimir_yaw() {
    Serial.print("Yaw: ");
    Serial.println(yaw);
}

void setup() {
    // Parar los motores al inicio
    robot.steer(0, 0, 0);
    // Inicialización
    Serial.begin(9600);
    
    attachInterrupt(digitalPinToInterrupt(27), ISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(5), ISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(38), ISR3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(2), ISR4, CHANGE);

    // Configuración de actuadores y sensores
    if (!bno.begin()) {
        Serial.println("No BNO055 detected... Check your wiring!");
        while (1);
    }
    bno.setExtCrystalUse(true);

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
    leer_tof();
    imprimir_tof();
    leer_ultrasonidos();
    imprimir_ultrasonidos();
    leer_yaw();
    imprimir_yaw();
    // Calcular el error de ángulo correctamente con la función circular
    float angle_error = calcularDiferenciaAngulo(yaw, TARGET_ANGLE);  // Diferencia angular ajustada

    if (front_distance != 0 && front_distance < 20){
        robot.steer(0, FORWARD, 0);
        delay(3000);
    }
    else{

        // Si el ángulo de giro es mayor que el umbral, ignorar el ultrasonido y corregir el ángulo
        if (abs(angle_error - TARGET_ANGLE) > ANGLE_THRESHOLD) {
            steer = KP_ANGLE * (-angle_error);  // Invertir el signo del error angular
            // Limitar el valor de `steer` entre [-MAX_STEER, MAX_STEER]
            if (steer > MAX_STEER) steer = MAX_STEER;
            if (steer < -MAX_STEER) steer = -MAX_STEER;

            // Mover el robot con la corrección de ángulo
            robot.steer(20, FORWARD, steer);
            
            // Imprimir para depuración
            Serial.print("Corrigiendo con ángulo. Steer: ");
            Serial.println(steer);
        } else {
            // Si el ángulo está dentro del umbral, usar el ultrasonido para ajustar la distancia
            if (right_distance != 0) {
                double distance_error = TARGET_DISTANCE - right_distance;  // Error de distancia
                steer = KP_DISTANCE * distance_error;  // Corrección proporcional de la distancia

                // Limitar el valor de `steer` entre [-MAX_STEER, MAX_STEER]
                if (steer > MAX_STEER) steer = MAX_STEER;
                if (steer < -MAX_STEER) steer = -MAX_STEER;

                // Mover el robot con la corrección de distancia
                robot.steer(20, FORWARD, steer);

                // Imprimir para depuración
                Serial.print("Corrigiendo con distancia. Steer: ");
                Serial.println(steer);
                }
            }
    }
}
