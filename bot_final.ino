/*
  recieve_counts TF and Odom topcic publisher
  Copyright (c) 2015 Kartik Madhira.  All right reserved.

  Kalman.h Library and I2C.ino code are exclusive properties of TKJ Electronics.
  This specfic code takes in data from the MPU6050 using the I2C node. The data
  is the kalman filtered angle using Accelerometer and Gyroscope. The absolute 
  filtered angle minus 180 degrees is taken as error and sent to a PID controller.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Kalman.h>

#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

////////////////////////////motor driver////////////////////////////////////
#define ER 11
#define EL 10

float kp=100.00;
float stableangle =182.00;
#define m11         13
#define m12         12
#define PWM_m1      11
#define PWM_m2      10
#define m21         9
#define m22         8

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;


#define ki 1.5

#define kd 0
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float proportional,difference,integral,integrald,rate,prevposition,derivative,controlf;
float control;


/* IMU Data */
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

double accXangle, accYangle; // Angle calculate using the accelerometer
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX; // Calculate the angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

void setup() 
{ 
  
  Serial.begin(115200);
  Wire.begin();
   TWBR = ((F_CPU / 400000L) - 16) / 2;//set gyro rate to 400khz 
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x03; // Set Gyro Full Scale Range to Â±250deg/s
  i2cData[3] = 0x03; // Set Accelerometer Full Scale Range to Â±2g
  while(i2cWrite(0x19,i2cData,4,false)); // Write to all four registers at once
  while(i2cWrite(0x6B,0x01,true)); // PLL with X axis gyroscope reference and disable sleep mode 
  
  while(i2cRead(0x75,i2cData,1));
  if(i2cData[0] != 0x68) 
  {
    // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while(1);
  }
   pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
    
  
  /////////////////logic supply////////////////
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  
  digitalWrite(3,HIGH);
  digitalWrite(2,LOW);
 
  /////////////////Switch//////////////////////
  pinMode(4,INPUT);
  
  //////////////////motor driver///////////////
  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(8,OUTPUT);
  
  delay(100); // Wait for sensor to stabilize
  
  /* Set kalman and gyro starting angle */
  while(i2cRead(0x3B,i2cData,6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  // atan2 outputs the value of -Ï€ to Ï€ (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2Ï€ and then from radians to degrees
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  
  kalmanX.setAngle(accXangle); // Set starting angle
  gyroXangle = accXangle;
  gyroYangle = accYangle;
 
  
 /////////////////////LED//////////////////////// 

  
  timer = micros();
}
int i=0;
double avg=0;
void loop() 
{
 while(i2cRead(0x3B,i2cData,14))
  {
   analogWrite(ER,0);/////right motor
   analogWrite(EL,0);/////left motor   
  } 
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);
  
 // atan2 outputs the value of -Ï€ to Ï€ (radians)
 // We then convert it to 0 to 2Ï€ and then from radians to degrees
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  Serial.print(accXangle);
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  
  double gyroXrate = (double)gyroX/131.0;
  double gyroYrate = -((double)gyroY/131.0);
  gyroXangle += gyroXrate*((double)(micros()-timer)/1000000); // Calculate gyro angle without any filter  
  gyroYangle += gyroYrate*((double)(micros()-timer)/1000000);
  
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
Serial.print(constrain((control),0,255));
Serial.println('/n');
//  avg=avg+kalAngleX;
  timer = micros();
  ////////////////////////////////////////////////////////////Main Logic//////////////////////////////////////////////////// 
 // }
 // kalAngleX=avg/(5.00);
  avg=0;
  getError();
  rate = difference-prevposition;
 
  if(abs(rate)>0.1)
  {
    pid();
  }
  //////////////////delay/////////////////////
  //delayMicroseconds(20);
}

void getError()
{
 if (kalAngleX<=stableangle)
 {
   difference=kalAngleX-stableangle;
   
  /* if(abs(difference)<float(0.80))
   {
     kp=44.00;//32.00;
   }
   else
   {
    kp=60.00;//48.00;
   }
   */
 }
 else if (kalAngleX>=stableangle)
 {
   difference=kalAngleX-stableangle;
   //digitalWrite(Red,LOW);
   //digitalWrite(Green,HIGH);
   
   /*if(abs(difference)<float(0.80))
   {
     kp=41.00;//36.00;
   }
   else
   {
    kp=55.00;//48.00;
   }
    */
 }

 else
 {
   difference=0;
   
 } 
 
}

void pid()
{
  //------------------------PID Algorithm-----------------------------------
	if(difference!=0)	
	{

		//-----------------Proportional------------------------
		proportional = difference * kp;
		//-------------------Integral--------------------------
		integral += difference;
                integral=constrain(integral,-50,50);
		integrald = integral * ki;      
		//------------------Derivative-------------------------
		rate = difference-prevposition;
		derivative = rate * kd;
		//--------------------Control--------------------------
		controlf = proportional+derivative+integrald;
		//-----------------------------------------------------		
                control =abs(controlf);
              /*  if (control>=205)
                {
                  control=255;
                }
                else
                {
                  control=control+50;
                }
                */
                if(difference>float(0))
                {
                  /////////////////direction//////////////////
                   digitalWrite(m11,HIGH);
  digitalWrite(m21,HIGH);
  digitalWrite(m12,LOW);
  digitalWrite(m22,LOW);
  analogWrite (PWM_m1,(constrain((control),0,255)));
  analogWrite (PWM_m2,(constrain((control),0,255)));/////left motor
                }
                else if(difference<float(0))
                {
                 
                   /////////////////direction//////////////////
                          digitalWrite(m11,LOW);
  digitalWrite(m21,LOW);
  digitalWrite(m12,HIGH);
  digitalWrite(m22,HIGH);
  analogWrite (PWM_m1,(constrain((control),0,255)));
  analogWrite (PWM_m2,(constrain((control),0,255)));//left motor
                 
                }

		//--------------------PID Ends-------------------------
       
      }
     
    

     prevposition=difference;
  
}
