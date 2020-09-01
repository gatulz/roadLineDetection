#include <LIDARLite.h>
#include <Servo.h>
#include <Wire.h>
int TriggerPIN1 = 4;
int EchoPIN1 = 5;
int TriggerPIN2 = 6;
int EchoPIN2 = 7;
int TriggerPIN3 = A2;
int EchoPIN3 = A3;
int TriggerPIN4 = 2;
int EchoPIN4 = 3;
int i = 0;
Servo servo;
LIDARLite lidarLite;
float servoAngle;
int cal_cnt = 0;
int maxAngle= 1100; //65
int inAngle = 500; //35
int dist_cos;
int dist_sin;
int servo_position = inAngle;
int jarakX;
int jarakY;
int nilaiMin;
int Xmin;
int realDist;
float posisiServo; 
//int position = 0 ;    // state variable
boolean forward = false ;   // state variable
int deltamillis;
int delayServo = 0;
unsigned long ts = 0;   // time accounting.
unsigned long ts2;
int last_pos;
unsigned long last_print=0;
#define DELAY 5
void setup()
{
  Serial.begin(115200);
  servo.attach(9);
  servo.write(0);
  lidarLite.begin(0, true);
  lidarLite.configure(3);
  pinMode(TriggerPIN1,OUTPUT);
  pinMode(EchoPIN2,INPUT);
  pinMode(TriggerPIN2,OUTPUT);
  pinMode(EchoPIN2,INPUT);
  pinMode(TriggerPIN3,OUTPUT);
  pinMode(EchoPIN3,INPUT);
  pinMode(TriggerPIN4,OUTPUT);
  pinMode(EchoPIN4,INPUT);  
}


void loop ()
{
  int dist;
  String dist_string;
  //Baca dan Kalkulasi Lidar
  deltamillis = millis() - ts;
//  if (deltamillis!=0){
//    Serial.println(deltamillis);
//  }
  if (forward) { 
    servoAngle = -float(deltamillis)/DELAY;
  }else {
    servoAngle = float(deltamillis)/DELAY;
  }
  servoAngle = servoAngle + ((last_pos) - (900))/10;
  if ( cal_cnt == 0 )
  {
    dist = lidarLite.distance();    // With bias correction
    dist_cos=  int (dist * cos((servoAngle)/57.2958));
    dist_sin = int (dist * sin((servoAngle)/57.2958)); //*0.6
  }
  else
  {
    dist = lidarLite.distance(false); // Without bias correction
    dist_cos=  int (dist * cos((servoAngle)/57.2958));
    dist_sin = int (dist * sin((servoAngle)/57.2958)); //*0.6
  }
  // Increment reading counter
  cal_cnt++;
  cal_cnt = cal_cnt % 100;

if ((dist_cos<nilaiMin)&&(dist_cos>20)&&(abs(dist_sin)<=70))
{
  Xmin = dist_sin;
  nilaiMin = dist_cos;
  realDist = dist;
  posisiServo = servoAngle;
}
  //Kalkulasi Lidar
//  if (dist_sin <= 80)
//  {
//    jarakX = 1;
//    //digitalWrite(LED_BUILTIN, HIGH);
//    if (dist_cos >= 1000) 
//    {
//      jarakY = 0;
//    }
//    else 
//    {
//      jarakY = 1;
//    }
//  }
//  else
//  {
//    jarakX = 0;
//  }

  //Print Jarak Y
  dist_string= String(dist_cos);
  //Serial.println(dist_string);
  /*if (jarakX == 1)
  {
    if (jarakY == 1)
    {
      Serial.println("1");
    }
    else
    {
      Serial.println("2");
    }
  }
  else
  {
    Serial.println("3");
  }*/
  if (millis() - last_print>=301){
    last_print = millis();
    
  }
  //Gerak Servo dalam Millis
  if (millis () - ts >= DELAY)
  {
//    Serial.print(delayServo);
//    Serial.print("\t");
    
    ts = millis() ;   // setup timestamp for next time.
    if (forward)
    {
       last_pos = servo_position;
       servo.writeMicroseconds(servo_position);
       servo_position = servo_position - 10;
//      servo.write (-- servo_position) ;  // progress the servo
      if (servo_position == inAngle) // test for reverse
      {
        
    Serial.print(nilaiMin);
    Serial.print("\t");
    Serial.print(Xmin);
    Serial.print("\t");
    Serial.print(realDist);
    Serial.print("\t");
    Serial.print(posisiServo,2);
    Serial.print("\t");
        forward = false ;
        nilaiMin = 10000;
        
//        delayServo = millis() - ts2;
//        ts2 = millis();
      }
    }
    else
    {
      last_pos = servo_position;
      servo.writeMicroseconds(servo_position);
      servo_position = servo_position + 10;
//      servo.write (++ servo_position) ;  // progress the servo
      if (servo_position == maxAngle)  // test for reverse
      {
        
    Serial.print(nilaiMin);
    Serial.print("\t");
    Serial.print(Xmin);
    Serial.print("\t");
    Serial.print(realDist);
    Serial.print("\t");
    Serial.print(posisiServo,2);
    Serial.print("\t");
    
        forward = true ;
        nilaiMin = 10000;
//        delayServo = millis() - ts2;
//        ts2 = millis();
      }
    }
  }
    if (millis()-ts2 >= 40){
    ts2 = millis();
    i++;
    if (i % 4 == 0){
      
      digitalWrite(TriggerPIN1,LOW);
      delayMicroseconds(4);
      digitalWrite(TriggerPIN1,HIGH);
      delayMicroseconds(13);
      digitalWrite(TriggerPIN1,LOW);
      
      long td1 = pulseIn(EchoPIN1,HIGH);
      int distance1 = 0.0343 * (td1/2);
    
      Serial.print("Sensor 1 : ");
      Serial.print(distance1);
    }
    else if (i % 4 == 1){
      
      digitalWrite(TriggerPIN2,LOW);
      delayMicroseconds(4);
      digitalWrite(TriggerPIN2,HIGH);
      delayMicroseconds(13);
      digitalWrite(TriggerPIN2,LOW);
  
      long td2 = pulseIn(EchoPIN2,HIGH);
      int distance2 = 0.0343 * (td2/2);
  
      Serial.print("\tSensor 2 : ");
      Serial.print(distance2);
    }
    else if (i%4 == 2){
      
      digitalWrite(TriggerPIN3,LOW);
      delayMicroseconds(4);
      digitalWrite(TriggerPIN3,HIGH);
      delayMicroseconds(13);
      digitalWrite(TriggerPIN3,LOW);
  
      long td3 = pulseIn(EchoPIN3,HIGH);
      int distance3 = 0.0343 * (td3/2);
  
      Serial.print("\tSensor 3 : ");
      Serial.print(distance3);
    }
    else if (i%4 == 3){
    
      digitalWrite(TriggerPIN4,LOW);
      delayMicroseconds(4);
      digitalWrite(TriggerPIN4,HIGH);
      delayMicroseconds(13);
      digitalWrite(TriggerPIN4,LOW);
  
      long td4 = pulseIn(EchoPIN4,HIGH);
      int distance4 = 0.0343 * (td4/2);
  
      Serial.print("\tSensor 4 : ");
      Serial.println(distance4);
    }
  if (i >= 20){
    i = 1;
  }
  } 
}
