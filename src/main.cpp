#include <Arduino.h>
#include <drivebase.h>
#include <PID.h>
#include <Wire.h>
#include <MPU6050.h>

double steer, speed;
int dir, green_state, serial5state = 0; // 0 is motor angle, 1 is motor speed
MPU6050 mpu;
IntervalTimer timermpu;
bool startUp = false;
// Timers
unsigned long lastmputime = 0;
float timeStep = 0;
// Pitch, Roll and Yaw values
volatile float pitch = 0;
volatile float roll = 0;
volatile float yaw = 0;
int imu[3];
Moto bl(29, 28, 27);
Moto fl(7, 6, 5); // pwm dir enc
Moto br(36, 37, 38);
Moto fr(4, 3, 2);
DriveBase robot(&fl, &fr, &bl, &br);

void ISR1() { bl.updatePulse(); }
void ISR2() { fl.updatePulse(); }
void ISR3() { br.updatePulse(); }
void ISR4() { fr.updatePulse(); }


int reverse = 0;
int angle0;
void leermpu() {
  unsigned long timenow = millis();
  timeStep = (timenow - lastmputime) / 1000.0;
  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();
  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;
  // Output raw
  Serial.println(yaw);
  imu[0] = static_cast<int>(roll);
  imu[1] = static_cast<int>(pitch);
  imu[2] = static_cast<int>(yaw);

  lastmputime = timenow;
}

void serialEvent5()
{
    int data = Serial5.read();
    if (data == 255)
        serial5state = 0;
    else if (data == 254)
        serial5state = 1;
    else if (data == 253)
        serial5state = 2;
    else if (data == 252)
        serial5state = 3;
    else if (serial5state == 0)
    {
        speed = (double)data / 100 * 100; // 100 set as max speed
        speed = max(20, speed);           // 20 set as min speed
    }
    else if (serial5state == 1)
    {
        steer = ((double)data - 90) / 90;
    }
    else if (serial5state == 2)
    {
        green_state = data;

        if (green_state == 3 && !reverse)
        {
            reverse = 1;
            angle0 = yaw;
        }
    }
}
void setup()
{
  robot.steer(0,0,0);
  Serial.begin(115200);
  Serial5.begin(57600);
  pinMode(32, INPUT_PULLUP); // switch
  pinMode(13, OUTPUT);       // in-built LED for debugging
  attachInterrupt(digitalPinToInterrupt(27), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), ISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(38), ISR3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), ISR4, CHANGE);
  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want to calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensitivity. Default 3.
  // If you don't want to use threshold, comment this line or set 0.
  mpu.setThreshold(1);
    while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want to calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensitivity. Default 3.
  // If you don't want to use threshold, comment this line or set 0.
  mpu.setThreshold(1);
  timermpu.begin(leermpu, 20000); 
}

void loop()
{
  
    if (digitalRead(32) == 1)
    {
        robot.steer(0, dir, 0);
        digitalWrite(13, HIGH);
        dir = !dir;
    }
    else
    {
        if (reverse)
        {
            int error = abs((int)yaw - angle0);

            if (error < 175)
            { // turn 180 degrees
                // Serial.println(abs((int)imu[0] - angle0));
                robot.steer(40, 1, 1);
            }

            else
            {
                reverse = 0;
            }
        }
        else
        {
            robot.steer(speed, 0, steer);
            digitalWrite(13, LOW);
        }
    }
    Serial.print(speed);
    Serial.print(" ");
    Serial.println(steer);
}