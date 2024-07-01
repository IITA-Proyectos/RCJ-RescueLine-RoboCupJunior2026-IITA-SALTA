#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
#include <claw.h>
#include "lidar.h"
#include <Wire.h>
#include <elapsedMillis.h>

// CONSTANTS //
#define FORWARD 0
#define BACKWARD 1
#define TRIG_FRONT_PIN 39
#define ECHO_FRONT_PIN 33

elapsedMicros sinceFrontFire;
volatile unsigned long long frontVal;
volatile bool frontFireTime = true;
elapsedMillis sinceFrontPrint;

// INITIALISE ACTUATORS //
Moto bl(4, 3, 2); // pwm, dir, enc
Moto fl(35, 34, 36);
Moto br(7, 6, 5);
Moto fr(30, 28, 29);
DriveBase robot(&fl, &fr, &bl, &br);

DFServo sort(22, 540, 2390, 274);
DFServo left(14, 540, 2390, 274);
DFServo right(21, 540, 2390, 274);
DFServo lift(20, 540, 2390, 274);
DFServo deposit(23, 540, 2390, 274);
Claw claw(&lift, &left, &right, &sort, &deposit);

// INITIALISE SENSORS //
Lidar leftLidar(&Wire1);
int imuIndex0, imuIndex1, imuIndex2, imuIndex3, imuIndex;
int imu[3];
char imuBuffer[30] = "                             ";
String imuString = String(imuBuffer);

// STATE VARIABLES & FLAGS //
int serial6state = 0; // serial code e.g. 255

double speed;          // speed (0 to 100)
double steer;          // angle (0 to 180 deg, will -90 later)
int incoming_task = 0; // 0 = no green squares, 1 = left, 2 = right, 3 = double, 4 = pick up cube, 10 = deposit
int line_middle = 0;   // if there is a line to reacquire after obstacle

int action;            // action to take (part of a task)
bool taskDone = false; // if true, update current_task

int evacAngle0;
int angle0;             // initial IMU reading
int depositCounter = 0; // number of loops where robot is moving parallel to deposit zone
unsigned long long depositTime = 0;
bool startUp = false;

double wallTrackTarget = 100;
bool seenEvac = false;
bool deposited = false;
float frontUSReading;

// imu 4 point calibration, must be in ascneding order
// int a = 95 + 180;
// int b = -140 + 180;
// int c = -59 + 180;
// int d = 9 + 180;

int a = -112 + 180;
int b = -75 + 180;
int c = -23 + 180;
int d = 36 + 180;

// 95, -132
// -140, -42
// -59, 37
// 9, 150

// 36, -23, -75, -112

int imuLookup[360];
unsigned long long lastfk = millis(), lastdblgreen = millis();
int cccounter, leftLidarReading;
// UPDATE MOTORS, IMU, ULTRASOUNDS, TASK //
void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }

void serialEvent1() // read IMU
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

void serialEvent6() // read speed, steer, task, line_middle from rpi
{
  while (Serial6.available())
  {
    int data = Serial6.read(); // read serial code

    if (data == 255) // speed incoming
      serial6state = 0;

    else if (data == 254) // steer incoming
      serial6state = 1;

    else if (data == 253) // task incoming
      serial6state = 2;

    else if (data == 252) // line_middle incoming
      serial6state = 3;

    else if (serial6state == 0)         // set speed
      speed = (double)data / 100 * 100; // max speed = 100

    else if (serial6state == 1) // set steer
      steer = ((double)data - 90) / 90;

    else if (serial6state == 2) // set task
      incoming_task = data;

    else if (serial6state == 3) // set line_middle
      line_middle = data;
  }
}

void getFrontAnalog()
{
  frontVal = sinceFrontFire;
  frontFireTime = true;
}

// HELPER FUNCTIONS //
void runTime(int speed, int dir, double steer, unsigned long long time)
{
  unsigned long long startTime = millis();
  // elapsedMillis startTime;
  while ((millis() - startTime) < time)
  {
    robot.steer(speed, dir, steer);
    digitalWrite(13, HIGH);
    if (digitalRead(17) == 1) // switch is off
    {
      Serial6.write(255);
    }
  }

  digitalWrite(13, LOW);

  // while ((millis() - startTime) < time + 500) {
  // robot.steer(0, FORWARD, 0);
  // }
}

void runTime2(int speed, int dir, double steer, unsigned long long time)
{
  unsigned long long startTime = millis();
  // elapsedMillis startTime;
  while ((millis() - startTime) < time)
  {
    robot.steer(speed, dir, steer);
    digitalWrite(13, HIGH);
    Serial6.write(255);
  }

  digitalWrite(13, LOW);
}

void setup()
{
  pinMode(TRIG_FRONT_PIN, OUTPUT);
  pinMode(ECHO_FRONT_PIN, INPUT_PULLUP);
  attachInterrupt(ECHO_FRONT_PIN, getFrontAnalog, FALLING);

  attachInterrupt(digitalPinToInterrupt(2), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(36), ISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), ISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(29), ISR4, CHANGE);

  pinMode(16, OUTPUT);
  pinMode(17, INPUT_PULLUP); // switch
  pinMode(13, OUTPUT);       // in-built LED for debugging

  Serial1.begin(57700);  // for reading IMU
  Serial6.begin(115200); // for reading data from rpi and state
  Serial.begin(115200);  // displays ultrasound ping results

  for (int i = 0; i < 360; i++)
  {
    if ((i - a + 360) % 360 < (b - a + 360) % 360)
    { // q1
      imuLookup[i] = (int)(((float)((i - a + 360) % 360) / (float)((b - a + 360) % 360)) * 90);
    }
    else if ((i - a + 360) % 360 < (c - a + 360) % 360)
    { // q2
      imuLookup[i] = (int)(((float)((i - b + 360) % 360) / (float)((c - b + 360) % 360)) * 90) + 90;
    }
    else if ((i - a + 360) % 360 < (d - a + 720) % 360)
    { // q3
      imuLookup[i] = (int)(((float)((i - c + 360) % 360) / (float)((d - c + 360) % 360)) * 90) + 180;
    }
    else
    {
      imuLookup[i] = (int)(((float)((i - d + 360) % 360) / (float)((a - d + 360) % 360)) * 90) + 270;
    }
  }
}

void loop()
{
  int rawimu = imu[0];                              // ranges -180 to 180
  int correctedimu = imuLookup[rawimu + 180] - 180; // lookup table ranges from 0-360, but final result ranges from 0-180
  Serial.print("Raw IMU: ");
  Serial.print(rawimu);
  Serial.println(" ");
  // Serial.print("Corrected IMU: ");
  // Serial.print(correctedimu);
  // Serial.println(" ");
  digitalWrite(13, LOW);

  // UPDATE ULTRASOUND READINGS //
  if (frontFireTime)
  {
    digitalWrite(TRIG_FRONT_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_FRONT_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(TRIG_FRONT_PIN, LOW);
    sinceFrontFire = 0;
    frontFireTime = false;
    if (sinceFrontPrint > 50)
    {
      // Serial.print("f:\t");
      // Serial.println(((frontVal)*0.034) / 2.0);
      sinceFrontPrint = 0;
      frontUSReading = ((frontVal)*0.034) / 2.0;
    }
  }

  // Serial.print("Front US: ");
  // Serial.println(frontUSReading);

  leftLidarReading = leftLidar.getDist();

  // Serial.print("Left Lidar: ");
  // Serial.println(leftLidarReading);

  if (digitalRead(17) == 1) // switch is off
  {
    robot.steer(0, FORWARD, 0); // stop moving
    // digitalWrite(13, HIGH);     // turn on LED
    action = 7;
    startUp = false;
    taskDone = true;
    seenEvac = false;
    deposited = false;
    wallTrackTarget = 100;
    Serial6.write(255);
    // runTime(40, FORWARD, 1, 1050); // anticlockwise turn 90
    // runTime(0, FORWARD, 0, 1000);
    // runTime(40, FORWARD, 1, 2050); // anticlockwise turn 180
    // runTime(0, FORWARD, 0, 1000);
    // runTime(40, FORWARD, 0.33, 3000);
    // runTime(0, FORWARD, 0, 2000);
  }

  else if (digitalRead(17) == 0 && !startUp)
  {
    runTime(50, BACKWARD, 0, 250);
    runTime(50, FORWARD, 0, 200);
    startUp = true;
  }

  else
  { // Serial.print("leftLidar: ");
    // Serial.println(leftLidarReading);

    // DECIDE WHICH ACTION TO DO BASED ON TASK //
    // if (incoming_task == 11)
    // {
    //   //  action = 50;
    //   if (cccounter == 0)
    //   {
    //     cccounter += 1;
    //     action = 40;
    //     angle0 = correctedimu - 45;
    //     taskDone = false;
    //   }
    //   else
    //   {
    //     action = 50;
    //   }
    // }

    if (taskDone) // robot is currently not performing any task
    {
      // Serial.print("Incoming Task: ");
      // Serial.println(incoming_task);
      if (frontUSReading < 19 && digitalRead(17) == 0 && !(seenEvac)) // obstacle detected
      {
        action = 0;                 // 1st action = imu turn 90 deg
        angle0 = correctedimu - 85; // initialise setpoint
      }
      // if ((abs(imu[0] - angle0) > 45) && (leftLidarReading > 500)) // exit evac
      //{
      //   action = 11;
      // }
      else if (incoming_task == 11)
      {
        if (deposited)
          action = 9;
      }

      else if (incoming_task == 4) // pick up FOR BLUE CUBE
      {
        action = 3;
      }

      else if (incoming_task == 60)
      {
        if (!(deposited))
          action = 40;
      }

      else if (incoming_task == 40) // centring FOR BLACK BALL
      {
        if (!(deposited))

          action = 7;
      }
      else if (incoming_task == 50) // pick up FOR BLACK BALL
      {
        if (!(deposited))

          action = 30;
      }

      else if (incoming_task == 8)
      {
        angle0 = correctedimu;
        action = 7;
      }

      else if (incoming_task == 3) // double green, turn 180 deg
      {
        // action = 4;
        action = 14;
        //angle0 = correctedimu + 178;
      }

      else if (incoming_task == 10) // deposit
      {
        if (wallTrackTarget == 700)
        {
          action = 6;
        }
        else
        {
          action = 10;
        }
      }

      else if (incoming_task == 5) // walltrack mode
      {
        // if (action != 9) {
        //   action = 8;
        // }
        // action = 9;
        action = 10;
      }

      else if (incoming_task == 6) // seen silver, move straight for a while in evac
      {
        action = 8;
        seenEvac = true;
      }

      else if (incoming_task == 7)
      {
        action = 11;
        angle0 = correctedimu - 88;
      }

      else
        action = 7; // linetrack

      taskDone = false; // prevents action from changing
    }

    // PERFORM ACTION & UPDATE STATE VARIABLES //
    Serial.print("action: ");
    Serial.println(action);

    // action = 10;
    switch (action)
    {

    case 30:
    {
      runTime(0, FORWARD, 0, 100);   // stop for 100ms
      runTime(20, FORWARD, 0, 1000); // move forward for 1s
      runTime(0, FORWARD, 0, 100);   // stop for 100ms

      while (!claw.available())
        ; // pick up cube and reset
      claw.pickupLeft();
      while (!claw.available())
        ;
      claw.reset();

      runTime(20, BACKWARD, 0, 1500); // move backward for 1s
      action = 10;                    // next action = turn back to line
      break;
    }

    case 40:
    {
      runTime(0, FORWARD, 0, 100);   // stop for 100ms
      runTime(20, FORWARD, 0, 1000); // move forward for 1s
      runTime(0, FORWARD, 0, 100);   // stop for 100ms

      while (!claw.available())
        ; // pick up cube and reset
      claw.pickupRight();
      while (!claw.available())
        ;
      claw.reset();

      runTime(20, BACKWARD, 0, 1500); // move backward for 1s
      action = 10;                    // next action = turn back to line
      break;
    }

    case 0: // first imu 90 deg turn
    {
      // Serial6.write(255);
      // int error = ((((int)correctedimu - angle0) + 180) % 360 - 180);
      // if (error < -180)
      //   error += 360;

      // if (error > 0)
      //   robot.steer(abs(error), 0, -1);
      // else
      //   robot.steer(abs(error), 0, 1);

      // if (abs(error) < 2)
      // {
      //   // not sure if need to update line_middle multiple times first
      //   runTime(50, FORWARD, 0.4, 100); // turn a little bit
      //   action = 1;                     // next action = arc turn with line_middle
      // }
      runTime2(40, FORWARD, -1, 1050);  // hard-coded turn
      runTime2(50, FORWARD, 0.4, 100);
      action = 1;
      break;
    }

    case 1: // arc turn with line_middle
    {
      // Serial6.write(255);

      if (line_middle < 170) // arc turn
      {
        robot.steer(40, 0, 0.35);
      }
      else
      {
        runTime(50, FORWARD, 0.0, 300);
        // runTime(50, BACKWARD, 0.0, 500);
        action = 2;                     // next action = imu turn 90 deg back to line
        angle0 = correctedimu - 70;
      }
      break;
    }

    case 2: // final one-wheel imu turn for obstacle
    {
      int error = ((((int)correctedimu - angle0) + 180) % 360 - 180);
      if (error < -180)
        error += 360;

      if (error > 0)
        robot.steer(abs(error), 0, -1);
      else
        robot.steer(abs(error), 0, 1);

      if (abs(error) < 2)
      {
        taskDone = true; // action will be updated in the next loop
      }
      // runTime2(40, FORWARD, -1, 1050);
      // taskDone = true;
      break;
    }

    case 3: // move forward, pick up cube, move backward
    {
      runTime(0, FORWARD, 0, 100);   // stop for 100ms
      runTime(20, FORWARD, 0, 1000); // move forward for 1s
      runTime(0, FORWARD, 0, 100);   // stop for 100ms

      while (!claw.available())
        ; // pick up cube and reset
      claw.pickupLeft();
      while (!claw.available())
        ;
      claw.reset();

      runTime(20, BACKWARD, 0, 1500); // move backward for 1s
      action = 4;                     // next action = turn back to line
      break;
    }

    case 4: // final two-wheel imu turn for blue cube & double green
    {
      int error = ((((int)correctedimu - angle0) + 180) % 360 - 180);
      if (error < -180)
        error += 360;

      if (error > 0)
        robot.steer(abs(error), 0, -1);
      else
        robot.steer(abs(error), 0, 1);

      if (abs(error) < 2)
      {
        taskDone = true; // action will be updated in the next loop
      }
      break;
    }

    case 5: // turn 180 deg before deposit
    {
      // int error = ((((int)correctedimu - angle0) + 180) % 360 - 180);
      // if (error < -180)
      //   error += 360;

      // if (error > 0)
      //   robot.steer(abs(error), 0, -1);
      // else
      //   robot.steer(abs(error), 0, 1);

      // if (abs(error) < 2)
      // {
      //   action = 6; // next action = rest of deposit
      // }
      runTime(40, FORWARD, 1, 2100);  // turn 180 deg
      action = 6;
      break;
    }

    case 6: // deposit
    {
      runTime(40, FORWARD, 1, 2050);
      runTime(50, BACKWARD, 0, 3500);
      claw.depositRight();
      runTime(50, FORWARD, 0, 50);
      runTime(50, BACKWARD, 0, 100);
      runTime(50, FORWARD, 0, 50);
      runTime(50, BACKWARD, 0, 100);
      runTime(50, FORWARD, 0, 50);
      runTime(50, BACKWARD, 0, 100);
      runTime(50, FORWARD, 0, 50);
      runTime(50, BACKWARD, 0, 100);
      claw.depositLeft();
      runTime(50, FORWARD, 0, 50);
      runTime(50, BACKWARD, 0, 100);
      runTime(50, FORWARD, 0, 50);
      runTime(50, BACKWARD, 0, 100);
      runTime(50, FORWARD, 0, 50);
      runTime(50, BACKWARD, 0, 100);
      runTime(50, FORWARD, 0, 50);
      runTime(50, BACKWARD, 0, 100);

      deposit.setAngle(135);
      // runTime(50, FORWARD, 0.5, 500); // tune later! (turning right)
      // taskDone = true;
      runTime(40, FORWARD, 1, 1050);
      deposited = true;
      action = 10;
      break;
    }

    case 7: // linetrack
    {
      robot.steer(speed, FORWARD, steer);
      digitalWrite(13, LOW);
      taskDone = true;
      break;
    }

    case 8: // turn 90deg, move straight in evac
    {
      runTime(40, FORWARD, 0, 1000);
      runTime(40, FORWARD, 0.3, 3000);
      action = 10;
      taskDone = false;
      break;
    }

    case 9: // end evac, move forward and stop
    {
      runTime(20, FORWARD, 0, 3000); // forward for 3s
      runTime(0, FORWARD, 0, 20000); // stop for 20s
      taskDone = false;
      break;
    }

    case 10: // wall track with left sensor and a certain target
    {
      double rate = ((double)leftLidarReading - wallTrackTarget) / wallTrackTarget;
      if ((millis() - lastfk) > 500)
      {
        wallTrackTarget += 10;
        lastfk = millis();
      }

      if (rate > 1.0)
        rate = 1.0;
      else if (rate < -1.0)
        rate = -1.0;

      if (frontUSReading > 300 && (!deposited))
      {
        runTime(40, FORWARD, -1, 1050);
      }

      robot.steer(30, FORWARD, rate);
      action = 10;
      taskDone = true;

      if (wallTrackTarget > 700)
      {
        wallTrackTarget = 700;
      }

      if (deposited)
      {
        wallTrackTarget = 100;
      }
      break;
    }

    case 13:  // not used
    {
      int error = ((((int)correctedimu - angle0) + 180) % 360 - 180);
      if (error < -180)
        error += 360;

      if (error > 0)
        robot.steer(abs(error), 0, -1);
      else
        robot.steer(abs(error), 0, 1);

      if (abs(error) < 2)
      {
        action = 10; // action will be updated in the next loop
      }
      break;
    }

    case 14:  // turn 180 deg for double green squares
    {
      if ((millis() - lastdblgreen) > 5000)
      {
      runTime2(40, FORWARD, 1, 2100);
      lastdblgreen = millis();
      }
      //action = 7;
      taskDone = true;
      break;
    }
    }
  }
}
