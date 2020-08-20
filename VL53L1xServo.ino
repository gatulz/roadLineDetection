/*
This example shows how to take simple range measurements with the VL53L1X. The
range readings are in units of mm.
*/

#include <Wire.h>
#include <VL53L1X.h>
#include <Servo.h>

VL53L1X sensor;
Servo servo;
int maxAngle= 66; //65
int inAngle = 50; //35
int dist_cos;
int dist_sin;
int servo_position = 0;
int jarakX;
int jarakY;
int nilaiMin;
int Xmin;
int realDist;
int posisiServo;  
int position = 0 ;    // state variable
boolean forward = false ;   // state variable
unsigned long ts2 = 0;
int delayServo = 0;
unsigned long ts = millis () ;   // time accounting.
#define DELAY 15
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  servo.attach(9);
//  servo.write(0);
  sensor.setTimeout(50);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);
}

void loop()
{
  int dist;
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
  if (millis () - ts >= DELAY)
  {
//    Serial.print(delayServo);
//    Serial.print("\t");
    
    ts = millis() ;   // setup timestamp for next time.
    if (forward)
    {

      servo.write (-- servo_position) ;  // progress the servo
//      if (servo_position % 2 == 0)
//      {
        dist = sensor.read();
        dist_cos=  int (dist * cos(abs(servo_position-66)/57.2958));
        dist_sin = int (dist * sin(abs(servo_position-66)/57.2958));
//      }
      if ((dist_cos<nilaiMin)&&(dist_cos>30)&&(dist_sin<=90))
      {
        Xmin = dist_sin;
        nilaiMin = dist_cos;
        realDist = dist;
        posisiServo = servo_position-66;
      }
      if (servo_position == inAngle) // test for reverse
      {
        forward = false ;
        Serial.print(nilaiMin);
        Serial.print("\t");
        Serial.print(Xmin);
        Serial.print("\t");
        Serial.print(realDist);
        Serial.print("\t");
        Serial.println(posisiServo);
        nilaiMin = 4000;
//        delayServo = millis() - ts2;
//        ts2 = millis();
      }
    }
    else
    {
      servo.write (++ servo_position) ;  // progress the servo
//      if (servo_position % 2 == 0)
//      {
        dist = sensor.read();
        dist_cos=  int (dist * cos(abs(servo_position-66)/57.2958));
        dist_sin = int (dist * sin(abs(servo_position-66)/57.2958));
//      }
      if ((dist_cos<nilaiMin)&&(dist_cos>30)&&(dist_sin<=90))
      {
        Xmin = dist_sin;
        nilaiMin = dist_cos;
        realDist = dist;
        posisiServo = servo_position-66;
      }
      if (servo_position == maxAngle)  // test for reverse
      {
        forward = true ;
        Serial.print(nilaiMin);
        Serial.print("\t");
        Serial.print(Xmin);
        Serial.print("\t");
        Serial.print(realDist);
        Serial.print("\t");
        Serial.println(posisiServo);
        nilaiMin = 4000;
//        delayServo = millis() - ts2;
//        ts2 = millis();
      }
    }
  }
}
