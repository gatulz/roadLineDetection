#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include <Wire.h>
#include <Servo.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3
SFEVL53L1X sensor1;
Servo servo1;
int servoAngle1;
int maxAngle1= 1160; //65
int inAngle1 = 1000; //35
int dist_cos1;
int dist_sin1;
int servo_position1 = inAngle1;
int jarakX1;
int jarakY1;
int nilaiMin1;
int Xmin1;
int realDist1;
int posisiServo1; 
boolean forward1 = false ;   // state variable
//unsigned long ts2 = 0;
//int delayServo = 0;
unsigned long ts1 = millis () ;   // time accounting.
#define DELAY1 15

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  servo1.attach(9);
    if (sensor1.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");
  sensor1.setDistanceModeLong();
}

void loop() {
  int dist1;
  servoAngle1 = ((servo_position1) - (inAngle1))/10;
  dist1 = sensor1.getDistance();
  dist_cos1=  int (dist1 * cos(abs(servoAngle1)/57.2958));
  dist_sin1 = int (dist1 * sin(abs(servoAngle1)/57.2958));
  // Perhitungan Jarak Sensor VL53L1x
//  dist = sensor.read();
//  dist_cos=  int (dist * cos(abs(servo_position-85)/57.2958));
//  dist_sin = int (dist * sin(abs(servo_position-85)/57.2958));
//
//if ((dist_cos<nilaiMin)&&(dist_cos>30)&&(dist_sin<=90))
//{
//  Xmin = dist_sin;
//  nilaiMin = dist_cos;
//  realDist = dist;
//  posisiServo = servo_position-85;
//}

  //Gerak Servo dalam Millis
  if (millis () - ts1 >= DELAY1)
  {
//    Serial.print(delayServo);
//    Serial.print("\t");
    
    ts1 = millis() ;   // setup timestamp for next time.
    if (forward1)
    {
      servo1.writeMicroseconds(servo_position1);
      servo_position1 = servo_position1 - 10;
//      servo.write (-- servo_position) ;  // progress the servo
//      if (servo_position % 2 == 0)
//      {
//        dist1 = sensor1.getDistance();
//        dist_cos1=  int (dist1 * cos(abs(servoAngle1)/57.2958));
//        dist_sin1 = int (dist1 * sin(abs(servoAngle1)/57.2958));
//      }
      if ((dist_cos1<nilaiMin1)&&(dist_sin1<=900))
      {
        Xmin1 = dist_sin1;
        nilaiMin1 = dist_cos1;
        realDist1 = dist1;
        posisiServo1 = servoAngle1;
      }
      if (servo_position1 == inAngle1) // test for reverse
      {
        forward1 = false ;
        Serial.print(nilaiMin1);
        Serial.print("\t");
        Serial.print(Xmin1);
        Serial.print("\t");
        Serial.print(realDist1);
        Serial.print("\t");
        Serial.println(posisiServo1);
        nilaiMin1 = 4000;
//        delayServo = millis() - ts2;
//        ts2 = millis();
      }
    }
    else
    {
      servo1.writeMicroseconds(servo_position1);
      servo_position1 = servo_position1 + 10;
//      servo.write (++ servo_position) ;  // progress the servo
//      if (servo_position % 2 == 0)
//      {
//        dist1 = sensor1.getDistance();
//        dist_cos1=  int (dist1 * cos(abs(servoAngle1)/57.2958));
//        dist_sin1 = int (dist1 * sin(abs(servoAngle1)/57.2958));
//      }
      if ((dist_cos1<nilaiMin1)&&(dist_cos1>3)&&(dist_sin1<=900))
      {
        Xmin1 = dist_sin1;
        nilaiMin1 = dist_cos1;
        realDist1 = dist1;
        posisiServo1 = servoAngle1;
      }
      if (servo_position1 == maxAngle1)  // test for reverse
      {
        forward1 = true ;
        Serial.print(nilaiMin1);
        Serial.print("\t");
        Serial.print(Xmin1);
        Serial.print("\t");
        Serial.print(realDist1);
        Serial.print("\t");
        Serial.println(posisiServo1);
        nilaiMin1 = 4000;
//        delayServo = millis() - ts2;
//        ts2 = millis();
      }
    }
  }
}
