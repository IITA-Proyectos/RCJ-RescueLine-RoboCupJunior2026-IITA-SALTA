#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
#include <claw.h>
#include <NewPing.h>
#include "lidar.h"
#include <Wire.h>
#include <elapsedMillis.h>

// CONSTANTS //
#define SONAR_NUM 3
#define MAX_DISTANCE 400
#define PING_INTERVAL 33
#define FORWARD 0
#define BACKWARD 1

// INITIALISE ACTUATORS //
Moto bl(4, 3, 2);   // pwm, dir, enc
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
NewPing sonar[SONAR_NUM] = {
  NewPing(8, 9, MAX_DISTANCE),   // right ultrasound
  NewPing(11, 10, MAX_DISTANCE), // left ultrasound
  NewPing(39, 33, MAX_DISTANCE), // front ultrasound
};                               // trigger, echo, max distance in cm

Lidar leftLidar(&Wire1);

unsigned long pingTimer[SONAR_NUM]; // Holds the next ping time.
unsigned int cm[SONAR_NUM] = {999, 999, 999};
uint8_t currentSonar = 0;

int imuIndex0, imuIndex1, imuIndex2, imuIndex3, imuIndex;
int imu[3];
char imuBuffer[30] = "                             ";
String imuString = String(imuBuffer);

// STATE VARIABLES & FLAGS //
int serial6state = 0;   // serial code e.g. 255

double speed;           // speed (0 to 100)
double steer;           // angle (0 to 180 deg, will -90 later)
int incoming_task = 0;  // 0 = no green squares, 1 = left, 2 = right, 3 = double, 4 = pick up cube, 10 = deposit
int line_middle = 0;    // if there is a line to reacquire after obstacle

int action;             // action to take (part of a task)
bool taskDone = false;  // if true, update current_task 

int evacAngle0;
int angle0;             // initial IMU reading
int depositCounter = 0; // number of loops where robot is moving parallel to deposit zone
unsigned long long depositTime = 0;
bool startUp = false;

double wallTrackTarget = 200;
bool deposited = false;
bool seenEvac = false;

int a = -123 + 180;
int b = -44 + 180;
int c = 44 + 180;
int d = 139 + 180;

int imuLookup[360];

// UPDATE MOTORS, IMU, ULTRASOUNDS, TASK //
void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }

void ISR5() // read ultrasounds
{ // If ping echo, set distance to array.
  if (sonar[currentSonar].check_timer())
    cm[currentSonar] = sonar[currentSonar].ping_result / US_ROUNDTRIP_CM;
}

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

  if (data == 255)           // speed incoming
    serial6state = 0;

  else if (data == 254)      // steer incoming
    serial6state = 1;

  else if (data == 253)      // task incoming
    serial6state = 2;

  else if (data == 252)      // line_middle incoming
    serial6state = 3;

  else if (serial6state == 0)   // set speed
    speed = (double)data / 100 * 100; // max speed = 100
  
  else if (serial6state == 1)   // set steer
    steer = ((double)data - 90) / 90;
  
  else if (serial6state == 2)   // set task
    incoming_task = data;

  else if (serial6state == 3)   // set line_middle
    line_middle = data;
  }
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
  }

  digitalWrite(13, LOW);

  // while ((millis() - startTime) < time + 500) {
  // robot.steer(0, FORWARD, 0);
  // }
}

void setup()
{
  attachInterrupt(digitalPinToInterrupt(2), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(36), ISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), ISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(29), ISR4, CHANGE);

  pinMode(16, OUTPUT);
  pinMode(17, INPUT_PULLUP);  // switch
  pinMode(13, OUTPUT);  // in-built LED for debugging

  Serial1.begin(57600); // for reading IMU
  Serial6.begin(57600); // for reading data from rpi and state
  Serial.begin(115200); // displays ultrasound ping results

  pingTimer[0] = millis() + 75;
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  for (int i = 0; i<360; i++) {
    if ((i-a+360)%360 < (b-a+360)%360) { //q1
      imuLookup[i] = (int)(((float)((i-a+360)%360)/(float)((b-a+360)%360))*90);
    }
    else if ((i-a+360)%360 < (c-a+360)%360) { //q2
      imuLookup[i] = (int)(((float)((i-b+360)%360)/(float)((c-b+360)%360))*90)+90;
    }
    else if ((i-a+360)%360 < (d-a+720)%360) { //q3
      imuLookup[i] = (int)(((float)((i-c+360)%360)/(float)((d-c+360)%360))*90)+180;	  
    }
    else {
      imuLookup[i] = (int)(((float)((i-d+360)%360)/(float)((a-d+360)%360))*90)+270;	  
    }
  }
}

void loop()
{
  // Serial.print("IMU: ");
  // Serial.print(imu[0]);
  // Serial.println(" ");

  int rawimu = imu[0] + 180;
  int correctedimu = imuLookup[rawimu] - 180;
  digitalWrite(13, LOW);

  if (digitalRead(17) == 1)     // switch is off
  {
    robot.steer(0, FORWARD, 0); // stop moving
    // digitalWrite(13, HIGH);     // turn on LED
    action = 7;
    startUp = false;
    deposited = false;
    taskDone = true;
  }

  else if (digitalRead(17) == 0 && !startUp) {
    runTime(50, BACKWARD, 0, 250);
    runTime(50, FORWARD, 0, 200);
    startUp = true;
  }

  else 
  {
    // UPDATE ULTRASOUND READINGS //
    for (uint8_t i = 0; i < SONAR_NUM; i++)
    {
      if (millis() >= pingTimer[i])
      {
          pingTimer[i] += PING_INTERVAL * SONAR_NUM;
          sonar[currentSonar].timer_stop();
          currentSonar = i;
          cm[currentSonar] = 999;
          sonar[currentSonar].ping_timer(ISR5);
      }
    }

    int leftLidarReading = leftLidar.getDist();
    // Serial.print("leftLidar: ");
    // Serial.println(leftLidarReading);

    // DECIDE WHICH ACTION TO DO BASED ON TASK //
    if (taskDone) // robot is currently not performing any task
    {
      // Serial.println("In taskDone");
      // Serial.println(incoming_task);
      // if (cm[2] < 12) // obstacle detected
      // {
      //   action = 0;            // 1st action = imu turn 90 deg
      //   angle0 = imu[0] - 80;  // initialise setpoint
      // }
      // if ((abs(imu[0] - angle0) > 45) && (leftLidarReading > 500)) // exit evac
      // {
      //   action = 11;
      // }

      if (incoming_task == 4) // pick up
      {
        action = 3;
        angle0 = correctedimu;
      }

      else if (incoming_task == 3)  // double green, turn 180 deg
      {
        action = 4;
        angle0 = correctedimu + 175;
      }

      else if (incoming_task == 10) // deposit
      {
        action = 5; // first action = turn 180 deg
        angle0 = correctedimu + 178;
      }

      else if (incoming_task == 5) // walltrack mode
      {
        // if (action != 9) {
        //   action = 8;
        // }
        // action = 9;
        action = 10;
      }

      else if (incoming_task == 6)  // move straight for a while in evac
      {
        // if (action != 7) action = 8;
        // else action = 7;
        action = 8;
      }

      else if (incoming_task == 7)
      {
        action = 11;
        angle0 = correctedimu - 90;
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
      case 0: // first imu 90 deg turn
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
          // not sure if need to update line_middle multiple times first
          runTime(50, FORWARD, 0.33, 100); // turn a little bit
          action = 1; // next action = arc turn with line_middle
        }
        break;
      }
      
      case 1: // arc turn with line_middle
      {
        if (line_middle < 170)    // arc turn
          robot.steer(50, 0, 0.33);

        else
        {
          action = 2; // next action = imu turn 90 deg back to line
          angle0 = correctedimu - 80;
        }
        break;
      }

      case 2: // final one-wheel imu turn for obstacle
      {
        int error = ((((int)correctedimu - angle0) + 180) % 360 - 180);
        if (error < -180) 
          error += 360;

        if (error > 0) 
          robot.steer(abs(error), 0, -0.5);
        else
          robot.steer(abs(error), 0, 0.5);
        
        if (abs(error) < 2)
        {
          taskDone = true;  // action will be updated in the next loop
        }
        break;
      }
      
      case 3: // move forward, pick up cube, move backward
      {
        runTime(0, FORWARD, 0, 100);    // stop for 100ms
        runTime(20, FORWARD, 0, 1000);  // move forward for 1s
        runTime(0, FORWARD, 0, 100);    // stop for 100ms

        while (!claw.available());      // pick up cube and reset
        claw.pickupLeft();
        while (!claw.available());
        claw.reset();

        runTime(20, BACKWARD, 0, 1000);    // move backward for 1s
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
          taskDone = true;  // action will be updated in the next loop
        }
        break;
      }

      case 5: // turn 180 deg before deposit
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
          action = 6; // next action = rest of deposit
        }
        break;
      }

      case 6: // deposit
      {
        runTime(50, BACKWARD, 0, 3000);
        claw.depositLeft();
        delay(2500);
        claw.depositRight();
        delay(2000);

        deposit.setAngle(135);
        // runTime(50, FORWARD, 0.5, 500); // tune later! (turning right)
        // taskDone = true;
        deposited = true;
        angle0 = correctedimu + 90;
        action = 13;
        break;
      }

      case 7: // linetrack
      {
        robot.steer(speed, FORWARD, steer);
        digitalWrite(13, LOW);
        taskDone = true;
        break;
      }

      // case 8:
      // {
      //   runTime(50, FORWARD, -0.3, 1000);
      //   delay(2000);
      //   taskDone = true;
      //   break;
      // }
      case 8: // move straight in evac
      {
        runTime(100, FORWARD, 0, 500);
        runTime(70, FORWARD, 0.2, 3000);  // arc turn to centre
        runTime(70, FORWARD, 0, 500);  // arc turn to centre
        
        deposited = false;
        taskDone = true;
        // action = 7;
        // action = 10;
        // evacAngle0 = correctedimu;
        break;
      }
      
      case 9:
      {
        double closest_wall = min(cm[2], min(cm[0],cm[1]));
        if (cm[2] <= 150) {
          double rate = pow(1.0 - (closest_wall/150.0), 3.0);
          if (cm[0] > cm[1]) {
            rate = -rate; // turn left when more space on left
          }
          // double spd = 50.0 * pow(cm[2]/150.0, 0.25);
          robot.steer(20, FORWARD, rate);
        }
        
        break;
      }

      case 10:  // wall track with left sensor and a certain target
      {

        // if (((double)leftLidarReading - wallTrackTarget) > 100)
        // {
        //   runTime(50, FORWARD, -0.3, 500);
        //   runTime(50, FORWARD, 0, 1000);
        //   taskDone = true;
        // }

        // else
        // {
        double rate = ((double)leftLidarReading - wallTrackTarget) / wallTrackTarget;
        if (rate > 1.0) rate = 1.0;
        else if (rate < -1.0) rate = -1.0;

        robot.steer(30, FORWARD, rate);
        action = 10;
        taskDone = false;
        // }
        
        // double rate;
        // if (cm[0] == 999) rate = 0;
        // else rate = (wallTrackTarget - (double)cm[0]) / wallTrackTarget;

        // Serial.print("rate: ");
        // Serial.print(rate);
        // Serial.println(" ");

        // Serial.print("error: ");
        // Serial.print(wallTrackTarget - (double)cm[0]);
        // Serial.println(" ");

        // Serial.print("US: ");
        // Serial.print((double)cm[0]);
        // Serial.println(" ");
        // rate /= 4.0;
        // if (rate < 0) rate = pow(abs(rate), 0.75) * -1; 
        // else rate = pow(rate, 0.75);

        // if ((leftLidarReading > 500) && (deposited)) {
        //   runTime(100, FORWARD, 0, 500);
        //   action = 14;
        //   angle0 = correctedimu - 90; // change next time!
        // }
        // double absError = (double)(abs(correctedimu - evacAngle0) % 90);

        // Serial.print("correctedIMU: ");
        // Serial.print(correctedimu);
        // Serial.println(" ");

        // Serial.print("evacAngle0: ");
        // Serial.print(evacAngle0);
        // Serial.println(" ");

        // Serial.print("absError: ");
        // Serial.print(absError);
        // Serial.println(" ");

        // if (absError > 22.5 && absError < 67.5) // parallel to deposit point
        // {
        //   if (depositTime == 0) depositTime = millis();

        //   else if (millis() - depositTime > 1500 && !deposited)
        //   {
        //     action = 11;  // turn left then deposit
        //     angle0 = correctedimu + 80; // change for right sensor!!!
        //   }
        // }

        // else 
        // {
        //   depositTime = 0;

          // if (leftLidarReading > 500) 
          // {
          //   double rate = -1 * pow(1.0 - (cm[2] / 150.0), 3.0);
          //   robot.steer(30, FORWARD, rate);
          // }
          // else 
          // {
          // }
          // taskDone = true;
        // }
        
        // Serial.println(rate);
        break;
      }

      case 11:
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
          action = 6;  // action will be updated in the next loop
        }
        break;
      }

      case 12:
      {
        runTime(100, FORWARD, 0, 500);
        taskDone = true;
        break;
      }

      case 13:
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
          action = 10;  // action will be updated in the next loop
        }
        break;
      }

      case 14: // move forward for exit
      {
        runTime(100, FORWARD, 0, 500);
        action = 15;
        angle0 = correctedimu - 90;
        break;
      }

      case 15:  // turn left to exit after move forward
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
          action = 16;  // action will be updated in the next loop
        }
        break;
      }

      case 16:
      {
        runTime(100, FORWARD, 0, 500);
        runTime(0, FORWARD, 0, 20000);
        taskDone = true;
      }

      case 17:
      {
        robot.steer(0, FORWARD, 0);
        action = 17;
      }
    }
  }
}