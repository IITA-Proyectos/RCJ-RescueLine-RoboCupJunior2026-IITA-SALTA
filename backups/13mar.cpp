#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
#include <claw.h>
#include <NewPing.h>

#define SONAR_NUM 3
#define MAX_DISTANCE 400
#define PING_INTERVAL 33

double steer, speed;
int dir, green_state, line_middle, serial6state = 0; // 0 is motor angle, 1 is motor speed
int pickedUp = 0;

Moto bl(4, 3, 2);
Moto fl(35, 34, 36); // pwm dir enc
Moto br(7, 6, 5);
Moto fr(30, 28, 29);
DriveBase robot(&fl, &fr, &bl, &br);

DFServo sort(22, 540, 2390, 274);
DFServo left(14, 540, 2390, 274);
DFServo right(21, 540, 2390, 274);
DFServo lift(20, 540, 2390, 274);
DFServo deposit(23, 540, 2390, 274);
Claw claw(&lift, &left, &right, &sort, &deposit);

void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }

int imuIndex0, imuIndex1, imuIndex2, imuIndex3, imuIndex;
int imu[3];
char imuBuffer[30] = "                             ";
String imuString = String(imuBuffer);

void serialEvent1()
{
  char data = char(Serial1.read());
  if (data == '#')
  {
    imuIndex = 0;
    imuString = String(imuBuffer);
    if (imuString.startsWith("#"))
    {
      imuIndex0 = imuString.indexOf("=");
      imuIndex1 = imuString.indexOf(",");
      imuIndex2 = imuString.lastIndexOf(",");
      imuIndex3 = imuString.lastIndexOf(" ");
      imu[0] = imuString.substring(imuIndex0 + 1, imuIndex1).toInt();
      imu[1] = imuString.substring(imuIndex1 + 1, imuIndex2).toInt();
      imu[2] = imuString.substring(imuIndex2 + 1, imuIndex3).toInt();
    }
    // Serial.print(imu[0]);
    // Serial.print(" ");
    // Serial.print(imu[1]);
    // Serial.print(" ");
    // Serial.println(imu[2]);
  }
  else
    imuIndex++;
  imuBuffer[imuIndex] = data;
}

void readIMU() 
{
  while (!Serial1.available());
  char data = char(Serial1.read());
  if (data == '#')
  {
    imuIndex = 0;
    imuString = String(imuBuffer);
    if (imuString.startsWith("#"))
    {
      imuIndex0 = imuString.indexOf("=");
      imuIndex1 = imuString.indexOf(",");
      imuIndex2 = imuString.lastIndexOf(",");
      imuIndex3 = imuString.lastIndexOf(" ");
      imu[0] = imuString.substring(imuIndex0 + 1, imuIndex1).toInt();
      imu[1] = imuString.substring(imuIndex1 + 1, imuIndex2).toInt();
      imu[2] = imuString.substring(imuIndex2 + 1, imuIndex3).toInt();
    }
    // Serial.print(imu[0]);
    // Serial.print(" ");
    // Serial.print(imu[1]);
    // Serial.print(" ");
    // Serial.println(imu[2]);
  }
  else
    imuIndex++;
  imuBuffer[imuIndex] = data;
}

NewPing sonar[SONAR_NUM] = {
    NewPing(8, 9, MAX_DISTANCE),   // right
    NewPing(11, 10, MAX_DISTANCE), // left
    NewPing(39, 33, MAX_DISTANCE), // front
};                                 // trigger, echo, max distance in cm

unsigned long pingTimer[SONAR_NUM]; // Holds the next ping time.
unsigned int cm[SONAR_NUM] = {999, 999, 999};
uint8_t currentSonar = 0;

void ISR5()
{ // If ping echo, set distance to array.
    if (sonar[currentSonar].check_timer())
        cm[currentSonar] = sonar[currentSonar].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle()
{ // Do something with the results.
    for (uint8_t i = 0; i < SONAR_NUM; i++)
    {
        Serial.print(i);
        Serial.print("=");
        Serial.print(cm[i]);
        Serial.print("cm ");
    }
    Serial.println();
}

int reverse = 0;
int angle0;

void serialEvent6()
{
  int data = Serial6.read();
  if (data == 255)
    serial6state = 0;
  else if (data == 254)
    serial6state = 1;
  else if (data == 253)
    serial6state = 2;
  else if (data == 252)
    serial6state = 3;
  else if (serial6state == 0)
  {
    speed = (double)data / 100 * 100; // 100 set as max speed
    // speed = max(20, speed);           // 20 set as min speed
  }
  else if (serial6state == 1)
  {
    steer = ((double)data - 90) / 90;
  }
  else if (serial6state == 2)
  {
    green_state = data;

    if (green_state == 3 && !reverse && digitalRead(17) == 0)
    {
      reverse = 1;
      angle0 = imu[0];
    }

    else if (green_state == 10 && digitalRead(17) == 0) 
    {
      angle0 = imu[0];
    }
  }
  else if (serial6state == 3)
  {
      line_middle = data;
  }
}

void readLineMiddle()
{
  while (!Serial6.available());
  int data = Serial6.read();

  if (data == 255)
    serial6state = 0;
  else if (data == 254)
    serial6state = 1;
  else if (data == 253)
    serial6state = 2;
  else if (data == 252)
    serial6state = 3;
  else if (serial6state == 0)
  {
    speed = (double)data / 100 * 100; // 100 set as max speed
    // speed = max(20, speed);           // 20 set as min speed
  }
  else if (serial6state == 1)
  {
    steer = ((double)data - 90) / 90;
  }
  else if (serial6state == 2)
  {
    green_state = data;

    if (green_state == 3 && !reverse && digitalRead(17) == 0)
    {
      reverse = 1;
      angle0 = imu[0];
    }

    // else if (green_state == 10 && digitalRead(17) == 0) 
    // {
    //   angle0 = imu[0];
    // }
  }
  else if (serial6state == 3)
  {
      line_middle = data;
  }
}

void setup()
{
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(2), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(36), ISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), ISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(29), ISR4, CHANGE);
  pinMode(16, OUTPUT);
  pinMode(17, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  Serial1.begin(57600);
  Serial6.begin(57600);

  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
    pingTimer[0] = millis() + 75;
    for (uint8_t i = 1; i < SONAR_NUM; i++)
        pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop()
{
  if (digitalRead(17) == 1)
  {
    robot.steer(0, dir, 0);
    digitalWrite(13, HIGH);
    // dir = !dir;
  }
  else
  {
    if (reverse)
    {
      int error = abs((int)imu[0] - angle0) % 180;

      if (error < 175)
      { // turn 180 degrees
        // Serial.print(error);
        // Serial.print(" ");
        // Serial.print(angle0);
        // Serial.print(" ");
        // Serial.println((int)imu[0]);
        robot.steer(40, 0, 1);
      }

      else
      {
        reverse = 0;
      }
    }
    else
    {
      for (uint8_t i = 0; i < SONAR_NUM; i++) // update ultrasound values
      {
          if (millis() >= pingTimer[i])
          {
              pingTimer[i] += PING_INTERVAL * SONAR_NUM;
              // if (i == 0 && currentSonar == SONAR_NUM - 1)
                  // oneSensorCycle(); // Do something with results.
              sonar[currentSonar].timer_stop();
              currentSonar = i;
              cm[currentSonar] = 999;
              sonar[currentSonar].ping_timer(ISR5);
          }
      }

      Serial.println(cm[2]);  // print front US
      Serial.println("Line Middle:" + String(line_middle));

      if (cm[2] < 12) { // avoid obstacle
        readIMU();
        angle0 = imu[0] + 80;

        while (true) {
          readIMU();
          int angle = ((((int)imu[0] - angle0) + 180) % 360 - 180);

          if (angle < -180) {
            angle += 360;
          }

          if (angle > 0) 
          {
            robot.steer(abs(angle), 0, -1);
          }
          else {
            robot.steer(abs(angle), 0, 1);
          }

          if (abs(angle) < 2) {
            break;
          }
          
          Serial.println(angle);
        }

        Serial.println("Done turning!");

        unsigned long long startTime = millis();  // move forward a bit
        while ((millis() - startTime) < 1000)
        {
          robot.steer(50, 0, -0.3);
        }

        for (int i = 0; i < 50; i++) readLineMiddle();
        while (line_middle < 170)    // arc turn
        {
          readLineMiddle();
          robot.steer(50, 0, -0.3);
        }

        for (int i = 0; i < 50; i++) readIMU();
        angle0 = imu[0] - 90;

        while (true) {
          readIMU();
          int angle = ((((int)imu[0] - angle0) + 180) % 360 - 180);

          if (angle < -180) {
            angle += 360;
          }

          if (angle > 0) 
          {
            robot.steer(abs(angle), 0, -0.5);
          }
          else {
            robot.steer(abs(angle), 0, 0.5);
          }

          if (abs(angle) < 2) {
            break;
          }
          
          Serial.println(angle);
        }

        Serial.println("Done turning!");
      }

      if (green_state == 4) {  // pick up blue cube
        angle0 = imu[0];

        unsigned long long startTime = millis();  // move forward a bit
        while ((millis() - startTime) < 100)
        {
          robot.steer(0, 0, 0);
        }

        startTime = millis();
        while ((millis() - startTime) < 1000)
        {
          robot.steer(20, 0, 0);
        }

        startTime = millis();
        while ((millis() - startTime) < 100)
        {
          robot.steer(0, 0, 0);
        }

        // robot.reset();

        while (!claw.available());  // pick up and reset
        claw.pickupLeft();
        while (!claw.available());
        claw.reset();
        
        startTime = millis(); // reverse a bit
        while ((millis() - startTime) < 1000)
        {
          robot.steer(20, 1, 0);
        }

        readIMU();
        while (true) {
          readIMU();
          int angle = ((((int)imu[0] - angle0) + 180) % 360 - 180);

          if (angle < -180) {
            angle += 360;
          }

          if (angle > 0) 
          {
            robot.steer(abs(angle), 0, -1);
          }
          else {
            robot.steer(abs(angle), 0, 1);
          }

          if (abs(angle) < 2) {
            break;
          }
          // Serial.println(angle);
        }

      }

      if (green_state == 10) {  // deposit
        readIMU();
        while (true) {
          readIMU();
          int error = abs((int)imu[0] - angle0) % 180;
          Serial.println("Error" + String(error));
          robot.steer(20, 0, 1);
          
          if (error > 178) {
            break;
          }
        }

        unsigned long long startTime = millis(); // reverse a bit
        while ((millis() - startTime) < 2000)
        {
          robot.steer(100, 1, 0);
        }

        claw.depositLeft();
        delay(2000);
        claw.depositRight();
        delay(2000);

        while (true) {
          robot.steer(0, 0, 0);
          deposit.setAngle(135);

          if (digitalRead(17) == 1) {
            break;
          }
        }
      }

      else {
        robot.steer(speed, 0, steer);
        digitalWrite(13, LOW);
      }
    }
    // while (!claw.available());
    // claw.pickupLeft();
    // while (!claw.available());
    // claw.reset();
    // while (!claw.available());
    // claw.pickupRight();
    // while (!claw.available());
    // claw.reset();
    //lift.setAngle(0);
  }
  // Serial.print(speed);
  // Serial.print(" ");
  // Serial.println(steer);
}