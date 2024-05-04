#include <Arduino.h>
#include <NewPing.h>
#include <Wire.h>
#include "lidar.h"

#define IMU_SERIAL Serial1
#define RPI_SERIAL Serial6
#define SONAR_NUM 3
#define MAX_DISTANCE 400
#define PING_INTERVAL 33

Lidar left(&Wire1), right(&Wire);
NewPing sonar[SONAR_NUM] = {
    NewPing(11, 10, MAX_DISTANCE), // left
    NewPing(8, 9, MAX_DISTANCE),   // right
    NewPing(39, 33, MAX_DISTANCE), // front
};                                 // trigger, echo, max distance in cm

unsigned long pingTimer[SONAR_NUM];
unsigned int cm[SONAR_NUM];
uint8_t currentSonar = 0;
int usleft, usright, usfront, lleft, lright;

void ISR5()
{
    if (sonar[currentSonar].check_timer())
        cm[currentSonar] = sonar[currentSonar].ping_result / US_ROUNDTRIP_CM;
}

void setup()
{
    Serial.begin(115200);
    IMU_SERIAL.begin(57600);
    RPI_SERIAL.begin(9600);
    pingTimer[0] = millis() + 75;
    for (uint8_t i = 1; i < SONAR_NUM; i++)
        pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop()
{
    for (uint8_t i = 0; i < SONAR_NUM; i++)
    {
        if (millis() >= pingTimer[i])
        {
            pingTimer[i] += PING_INTERVAL * SONAR_NUM;
            if (i == 0 && currentSonar == SONAR_NUM - 1)
            {
                usleft = cm[0];
                usright = cm[1];
                usfront = cm[2];
            }
            sonar[currentSonar].timer_stop();
            currentSonar = i;
            cm[currentSonar] = 0;
            sonar[currentSonar].ping_timer(ISR5);
        }
    }

    lleft = left.getDist();
    lright = right.getDist();
    Serial.print(usleft);
    Serial.print(" ");
    Serial.print(usright);
    Serial.print(" ");
    Serial.print(usfront);
    Serial.print(" ");
    Serial.print(lleft);
    Serial.print(" ");
    Serial.print(lright);
    Serial.print(" ");
    Serial.println();
}
