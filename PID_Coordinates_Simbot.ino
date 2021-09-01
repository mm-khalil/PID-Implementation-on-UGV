// Muneeb PID+Coordinates
/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 *License: MIT
 */
#include "Wire.h"
#include <MatrixMath.h>
#include <MPU6050_light.h>
MPU6050 mpu(Wire);

//------------- SIMBOT ------------------------
// PWM Pins for
int R_PWM = 10;
int L_PWM = 9;

//  Motor Direction Pins
int R_direction = 5;
int L_direction = 6;
#define Forward 1
#define Reverse 0

//  Motor Encoder Pins
int R_encoder = 2;
int L_encoder = 3;
long int encoderLPos = 0;
long int encoderRPos = 0;

// Simbot Design Features
float  dia = 0.034  ; // Wheel Diameter in m
float ER = 2900;      // Wheel Revolotions per meter
float b = 0.084;      // Distance between both wheels

// Universal Frame Grid size
float grid = 0.01; //in metres

// Transformation Variables
#include <MatrixMath.h>


//// Robot Rotation and Translation in Universal Frame
#define N  (3)
#define M  (1)
mtx_type T[N][N];   // Transformation Matrix
mtx_type Up[N][M];  // Target Point in Universal Frame
mtx_type Rp[N][M];  // Target Point in Robot Frame


float Current_Heading = 0;    // Robot rotation in universal Frame
//float th_rad = 0;    // Robot rotation in universal Frame
//
float Tx = 0;    // Robot Translation in Universal Frame (Position in Universal Frame)
float Ty = 0;
float T_th = 0;
//
//// Target Point In Universal Frame
float Ux = 0;
float Uy = 0;
//
// Next Point In Robot Frame
float Rx = 0;
float Ry = 0;
//
//// Action parameters(Angle and Distance)to reacg target coordinate
//float Angle = 0;
float A_Dist = 0; // Angular distance of wheels to rotate at a particular angle
//float Dist = 0;   // Linear distance of wheels to reach Target coordinate
float A_360 = 360.0;


// Flags for Forward, CW and CCW Rotation
bool F, L, R;

// Rotation Variable
float Dr = 0;
float Dl = 0;
float Dc = 0;

// Calibration
int nl = 0;    //no of left turns
int nr = 0;    //no. of right turns
int Angle_Direction = 0;    // 1 for left,   -1 for right, 0 for straight

// Rotation
float tol = 0;
float l_tol = 0;
float fl_tol = 0.01;
float ll_tol = 0.01;
float rl_tol = 0.03;
float r_tol = 0;
float rr_tol = 0;
float lr_tol = 0;

// Left Speed
int CCW_R = 180;           // Forward (max 0 - min 255)
int CCW_L = 40;           // Reverse (max 255 - min 0)

// Left Speed
int CW_R = 40;           // Forward (max 0 - min 255)
int CW_L = 180;           // Reverse (max 255 - min 0)

// Target Coordinates in Universal Frame (Loading Mission coordinates)
int tar_x[] = {0, 10,10};//, 0, 0, 0, 1, 1, 0, 0};
int tar_y[] = {0, 10, 0};//, 1, 0, 0, 0, -1, -1, 0};
int tar_th[] = {0, 30, 0};//, 0, 0, 0, 0, 0, 0, 0};
int n = 0; // No. of Target Points
int p = 0; // Current Target Point number

float Action_Angle = 0;
float Action_Distance = 0;

//------------- PID Controller ------------------------
unsigned long timer = 0;
long int Position=0;

float Kp =2.5; //set up the constants value// for simbot 3 we have kp= 3.5//
float Ki = 0.5;
float Kd = 2;

//for each simbot we have the fo llowing values of kp, ki and kd except for a simbot#03,
//float Kp = 2.5; //set up the constants value
//float Ki = 0.5;
//float Kd = 2;

int P;
int I;
int D;

int lastError = 0;

//Increasing the maxspeed can damage the motors - at a value of 255 the 6V motors will receive 7,4 V 
const uint8_t maxspeeda = 150;
const uint8_t maxspeedb = 150;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;

void setup()
{
 Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  // Taking Inpute from Encoders
  pinMode(R_encoder, INPUT);
  pinMode(L_encoder, INPUT);

  // Counting revolutions of right and left wheel
  attachInterrupt(digitalPinToInterrupt(R_encoder), doEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_encoder), doEncoder, CHANGE);
}

void loop()
{
  n = sizeof(tar_x) / sizeof(int);   //get total num of points
  if (p < n - 1) {
    Tx = tar_x[p];
    Ty = tar_y[p];
    T_th = tar_th[p + 1];
    Ux = tar_x[p + 1];
    Uy = tar_y[p + 1];
    
    Action_Angle = calculate_angle(Tx, Ty, Ux, Uy, Current_Heading);                           //Calculate angle to goal using transformation matrix
    Action_Distance = calculate_dist(Tx, Ty, Ux, Uy);                                          //Calculate distance to goal using distance formula
    Current_Heading = update_heading(Action_Angle, Current_Heading);                           //update heading angle by adding or subtracting action angle
    rotate_right(Action_Angle, 40, 180, 1);                                                    //rotate right if calculated angle is negative
    Serial.println("Angle taken: ");Serial.print(Action_Angle);
    Serial.println("Current HEading: ");Serial.print(Current_Heading);
    SetEncoder();                                                                              //need to reset encoder value to calculate angle/distance again
    rotate_left(Action_Angle, 180, 40, 1);                                                     //rotate left if calculated angle is positive
    Serial.println("Angle taken: ");Serial.print(Action_Angle);
    Serial.println("Current HEading: ");Serial.print(Current_Heading);
    SetEncoder();
  
    forward(grid, Action_Distance, 40, 40, 1);                                                 //finally move forward towards goal 
    SetEncoder();
    
    //*********function to move towards desired angle after moving to goal****************
    
    if (T_th < Current_Heading)          //if desired angle is less than current heading then rotate left
    {
      rotate_right(T_th - Current_Heading, 40, 180, 1);
      Serial.println("Angle to rotate right: ");Serial.print(Current_Heading - T_th);
      Current_Heading = update_heading(T_th - Current_Heading, Current_Heading);
      Serial.println("Current HEading: ");Serial.print(Current_Heading);
      SetEncoder();
    }
    if (T_th > Current_Heading)           //if desired angle is more than current heading then rotate right
    {
      rotate_left(T_th - Current_Heading, 180, 40, 1);
      Serial.println("Angle to rotate left: ");Serial.print(T_th - Current_Heading);
      Current_Heading = update_heading(T_th - Current_Heading, Current_Heading);
      Serial.println("Current HEading: ");Serial.print(Current_Heading);
      SetEncoder();
    }
    p++;
  }
}



// Simbot Functions
void doEncoder()

{
  encoderLPos++;
  encoderRPos++;
}

void SetEncoder()

{
  encoderLPos = 0;
  encoderRPos = 0;
}

float calculate_dist(float x1, float y1, float x2, float y2 )

{         //distance formula
  float Dist = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
  return Dist;
}

float calculate_angle(float x1, float y1, float x2, float y2, float Heading) 
{  //Transformation Matrix
  float Angle =0;
  float Heading_rad = Heading * 3.14 / 180;
  T[0][0] = cos(Heading_rad);
  T[0][1] = -sin(Heading_rad);
  T[0][2] = x1;
  T[1][0] = sin(Heading_rad);
  T[1][1] = cos(Heading_rad);
  T[1][2] = y1;
  T[2][0] = 0;
  T[2][1] = 0;
  T[2][2] = 1;
  // Target Point in Universal Frame
  Up[0][0] = x2;
  Up[1][0] = y2;
  Up[2][0] = 1;
  // Finding Target points in Robot Frame
  Matrix.Invert((mtx_type*)T, N);
  Matrix.Multiply((mtx_type*)T, (mtx_type*)Up, N, N, M, (mtx_type*)Rp);
  // Target Point in Robot Frame
  float Rx2 = Rp[0][0];
  float Ry2 = Rp[1][0];
  // Rotation required by Robot to reach Target Point and the Angular Distance to create that angle
  Angle = (atan2(Ry2, Rx2)) * (180 / 3.14);
  return Angle;
}

float update_heading(float Angle_moved, float Heading) 
{
  Heading = (Angle_moved + Heading);
  if (Heading >= 360) {
    Heading = Heading - 360;          // to define the range as 0 to 360
  }
  else if (Heading <= -360)

{
    Heading = Heading + 360;          // to define the range as 0 to -360
}
  return Heading;
}

void forward(float grid, float Dist, float right_speed, float left_speed, bool Flag) 
{
  mpu.update(); // IMU code for forward measuring position
Position= mpu.getAngleZ();
  timer = millis(); 
      if ( Dist > 0 && Flag==1) {
      Serial.println("Move Forward");
  while (Flag != 0) {
    Dr = 3.14 * 0.034 * (encoderRPos / ER);
    Dl = 3.14 * 0.034 * (encoderLPos / ER);
    Dc = (Dr + Dl) / 2;
    if (Dc < grid * Dist)
    {
    PID_control() ;
    Position=0;
    }
    if (Dc >= grid * Dist)
    {
      analogWrite(R_PWM, 255);
      analogWrite(L_PWM, 255);
      Flag = 0;
      delay(1000);
    }
  }
      }
    else {
      F = 0;
      Serial.println("No Forward");
    }
}

/////////////////
void PID_control() 
{
  int error = 0 - Position;

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd;
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0)

{
    motorspeedb = 0;
  } 
  //Serial.print(motorspeeda);Serial.print(" ");Serial.println(motorspeedb);
  forward_brake(motorspeeda, motorspeedb);
}

void forward_brake(int posa, int posb)
{
  digitalWrite(L_direction, Forward);  
  digitalWrite(R_direction, Forward);
  analogWrite(L_PWM, posa);  
  analogWrite(R_PWM, posb);
}



void rotate_right(float Angle, float right_speed, float left_speed, bool Flag)
{
      mpu.update(); // IMU code for forward Angle position
Position= mpu.getAngleZ();
  timer = millis(); 
  if (Angle < -0.01 && Flag==1) 

  {
  
    Serial.println("Rotate Right");

    nr++;
    A_Dist = abs((Angle / 360.0) * 3.14 * b);
    while (Flag!= 0) {
      Dr = 3.14 * 0.034 * (encoderRPos / ER);
      Dl = 3.14 * 0.034 * (encoderLPos / ER);
      Dc = (Dr + Dl) / 2;
      if (Dc < A_Dist)
      {

if (Position>= 0.01|Position<=0.01)
    {     if (Position!=Angle)
          {    
                analogWrite(R_PWM, right_speed);
        digitalWrite(R_direction, Reverse);
        analogWrite(L_PWM, left_speed);
        digitalWrite(L_direction, Forward);
          }
    }
Position=0;
      }
      if (Dc >= A_Dist)
      {
        analogWrite(R_PWM, 0);
        analogWrite(L_PWM, 255);
        Flag= 0;
       delay(1000);
      }
    }
  }
      else {
      Serial.println("No Right");
    }
}


void rotate_left(float Angle, float right_speed, float left_speed, bool Flag)
{
    mpu.update(); // IMU code for forward Angle position
Position= mpu.getAngleZ();
  timer = millis(); 
  if (Angle > 0 && Flag==1) {

      Serial.println("Rotate Left");

    nl++;
    A_Dist = abs((Angle / 360.0) * 3.14 * b);
    while (Flag != 0) {
      Dr = 3.14 * 0.034 * (encoderRPos / ER);
      Dl = 3.14 * 0.034 * (encoderLPos / ER);
      Dc = (Dr + Dl) / 2;
      if (Dc < A_Dist)
      {
if (Position>= 0.01|Position<=0.01)
    {     if (Position!=Angle)
          {    
        analogWrite(R_PWM, right_speed);
        digitalWrite(R_direction, Forward);
        analogWrite(L_PWM, left_speed);
        digitalWrite(L_direction, Reverse);
          }
    }
      Position=0;
}
      if (Dc >= A_Dist)
      {
        analogWrite(R_PWM, 255);
        analogWrite(L_PWM, 0);
        Flag= 0;
       delay(1000);

      }
    }
  }        else 
        {
      Serial.println("No Left");
    }
}
