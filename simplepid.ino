int R_PWM = 10;
int L_PWM = 9;

//  Motor Direction Pins
int R_direction = 5;
int L_direction = 6;
#define Forward 1
#define Reverse 0 


//IMU
#include "Wire.h"
#include <MPU6050_light.h>
MPU6050 mpu(Wire);
long int IMU_value  = 0;

//PID
float Kp = 2.5; 
float Ki = 0.5; 
float Kd = 2; 
int P, I, D, lastError = 0;

//Increasing the maxspeed can damage the motors - at a value of 255 the 6V motors will receive 7,4 V 
const uint8_t maxspeeda = 150, maxspeedb = 150,  basespeeda = 100,  basespeedb = 100;

 
void setup()
{ Serial.begin(9600);

  //IMU
  Wire.begin();
  byte status = mpu.begin();
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets();  // gyro 

 
}

void loop() 
{ //IMU
  mpu.update();
  IMU_value = mpu.getAngleZ();
  Serial.print("Yaw = ");  Serial.println(IMU_value);

  
  PID_control();

}
void PID_control() 
{
  int error = 0 - IMU_value;
  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd;
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  if (motorspeeda > maxspeeda) 
  {   motorspeeda = maxspeeda;  }
  if (motorspeedb > maxspeedb) 
  {   motorspeedb = maxspeedb;  }
  if (motorspeeda < 0) 
  {   motorspeeda = 0;          }
  if (motorspeedb < 0) 
  {   motorspeedb = 0;          }  
 
  forward_brake(motorspeeda, motorspeedb);
}
void forward_brake(int posa, int posb)
 { digitalWrite( L_direction, Forward);  
  digitalWrite(R_direction, Forward);
  analogWrite ( L_PWM , posa);  
  analogWrite (R_PWM , posb); 
}



   
