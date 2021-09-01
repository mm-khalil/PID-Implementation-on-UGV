
/// Muneeb PID Controller
/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 *License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>
MPU6050 mpu(Wire);
unsigned long timer = 0;
long int Position=0;

float Kp = 3.5; //set up the constants value// for simbot 3 we have kp= 3.5//
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
}

void loop() 
{
  mpu.update();
  if((millis()-timer)>10){ // print data every 10ms
  //Serial.print("X : ");
  //Serial.print(mpu.getAngleX());
  //Serial.print("\tY : ");
  //Serial.print(mpu.getAngleY());
    Serial.print("\tIMU Sensor(Yaw motion) : ");
Position= mpu.getAngleZ();
  Serial.println(Position);
  timer = millis();  
PID_control();
}
}
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
  if (motorspeedb < 0) {
    motorspeedb = 0;
  } 
  //Serial.print(motorspeeda);Serial.print(" ");Serial.println(motorspeedb);
  forward_brake(motorspeeda, motorspeedb);
}

//------------- ROSBOT Muneeb ------------------------
// PWM Pins for
int R_PWM = 10;
int L_PWM = 9;

//  Motor Direction Pins
int R_direction = 5;
int L_direction = 6;
#define Forward 1
#define Reverse 0

void forward_brake(int posa, int posb)
{
  digitalWrite(L_direction, Forward);  
  digitalWrite(R_direction, Forward);
  analogWrite (L_PWM, posa);  
  analogWrite (R_PWM, posb);
}
