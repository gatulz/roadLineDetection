// 1 rev = 2400 pulse
// vin == 6.1-6.4 volt daya motor
// tipe motor RS775, tanpa gearbox
// motor steering 1425.2 pulse/rev (4*
/*
sensor-sensor (untuk simulasi):
- compass (CMPS12) : theta (sudut kendaraan)
- encoder motor steering  :  alpha (sudut steering relatif terhadap sudut kendaraan)
- encoder motor penggerak : Vs / speed (kecepatan kendaraan)
- GPS : pos[0] (posisi aktual kendaraan (x,y))
- kamera : roadError (posisi kendaraan terhadap jalan)
sensor" lain
- subsistem deteksi objek
- sensor 
*/
#include "ext_lib.h"
#define pulseToAngleSteer 360/1425.2
#define pulseToMeter 4/300
double v1_f =0, v2_f=0;
int temp1=0;

void setup() {
  sp_v2 = 0;
//  pos1=20000;
//  sp_pos = 20000;
  
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  
  //posisi motor Steering
  pinMode(enc1A,INPUT_PULLUP);
  pinMode(enc1B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1A), ISR_INT0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc1B), ISR_INT1, CHANGE);
  
  //kecepatan sensor HAL
  pinMode(enc2A,INPUT_PULLUP);//_PULLUP);
  pinMode(enc2B,INPUT_PULLUP);//_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc2A), ISR_INT3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2B), ISR_INT4, CHANGE);
  
  // set timer 4(D6,D7,D8)/ motor2 divisor to  
  //  8 for PWM frequency of  3921.16 Hz
  
//  TCCR4B = TCCR4B & B11111000 | B00000010;    
  //motor1
  pinMode(rev1, OUTPUT);   
  pinMode(fwd1, OUTPUT);
  pinMode(pwm1, OUTPUT);
  //motor2
  pinMode(rev2, OUTPUT);   
  pinMode(fwd2, OUTPUT);
  pinMode(pwm2, OUTPUT);

  
  
//  sp_pos = 1200;
//  sp_v2 = 100;
//  pos1 = 1200; //+27.5
//  v1 = 70; //1.6
//  v2 = 100; // 2.67
//  temp_pos1 = float(pos1)*55/2400;
//  temp_v1 = float(v1)*55/2400;
//  temp_v2 = float(v2)*4/150;
//  motor_power(pwm2, rev2, fwd2, 90);
//  
//  motor_power(pwm1, rev1, fwd1, 90);
    motor_power(pwm2, rev2, fwd2, 0);
    motor_power(pwm1, rev1, fwd1, 0);
}
int ramp=0;
void loop() {
  
  //read/write command from PC
  Serial_IO();
  if (ramp==1 && millis() - last_delay>=40){
    last_delay = millis();
    if (servoTarget>sp_pos){
      sp_pos+=1;
    } else if (servoTarget<sp_pos){
      sp_pos-=1;
    }
  }
  
  if (millis() - last_pid1>=TS1){
    last_pid1 = millis();
    pidPosisi();
    motor_power(pwm1, rev1, fwd1, pwmPos);
//    if (abs(sp_pos-pos1)>=5){
////      pidKecPos();
////      motor_power(pwm1, rev1, fwd1, pwmSpeedPos);
//      motor_power(pwm1, rev1, fwd1, pwmPos);
//    } 
//    else {
////      sumErrPos = 0;
////      motor_power(pwm1, rev1, fwd1, 0);
//    }
    
  } // end of position pid
  
  if (millis() - last_pid2>=TS2){
    last_pid2 = millis();
    pidKecepatan();
    
    motor_power(pwm2, rev2, fwd2, pwmSpeed2);
  } // end of speed pid

//  if (millis() - last_print>=21){ //101
//    last_print = millis();
////    Serial.print(pos1*pulseToAngleSteer);//*pulseToAngleSteer
////    Serial.print('\t');
////    
////    Serial.print(sp_pos*pulseToAngleSteer);//*360/2400);
////    Serial.print('\t');
//    
////    Serial.print((sp_pos-pos1)*pulseToAngleSteer);//*360/2400);
////    Serial.print(v1);
////    Serial.print('\t');
////    Serial.print(pwmPos);
////    Serial.print('\t');
//
////    Serial.print(pos2);
////    Serial.print('\t');
//float speed1 =float(v2)*pulseToMeter ;
//float t_speed1 = float(sp_v2)*pulseToMeter;
//    Serial.print(speed1,2);
//    Serial.print('\t');
//    Serial.print(t_speed1,2);
//    Serial.print('\t');
////    Serial.print(pwmSpeed2);
//    Serial.print('\n');
//  }
  
}


void Serial_IO(){
  if (Serial.available()>0) {
    char inChar = Serial.read();
    if (inChar=='p'){
      Serial.print(pos1); // pos1 //sp_pos
      Serial.print(';');
      Serial.print(v2); //v2 //sp_v2real speed
      Serial.print(';');
      
    } else {
      if (isDigit(inChar) || inChar=='.' || inChar=='-') {
        // convert the incoming byte to a char and add it to the string:
        inString += (char)inChar;
      } else if (inChar == '\n') {
        inByte = inString.toInt();
        inString = "";
      } else if (inChar == 'a'){
        servoTarget = inString.toInt();
        ramp=0;
        sp_pos = servoTarget;
        inString = "";
      } else if (inChar == 't'){
        servoTarget = inString.toInt();
        ramp=1;
        inString = "";
      } else if (inChar == 'v'){
        servoSpeed = inString.toInt();
        sp_v2 = servoSpeed;
        inString = "";
      } else if (inChar == 'q'){
        temp1 = inString.toInt();
        Kps = float(temp1)/100;
        inString = "";
      } else if (inChar == 'w'){
        temp1 = inString.toInt();
        Kis = float(temp1)/100000;
        inString = "";
      } else if (inChar == 'e'){
        temp1 = inString.toInt();
        Kds = float(temp1)/10;
        inString = "";
      } 
      if (inByte>500){
        dig=1;
      } else if (inByte>100){
        dig=0;
      }
      digitalWrite(13, dig);
    }
  }
}


//if (millis()>500 && millis()<1500){
//    motor_power(pwm1, rev1, fwd1, 150);
//    temp1 = 100;
//  } else if (millis()>=1500 && millis()<2500){
//    motor_power(pwm1, rev1, fwd1, 0);
//    temp1 = -100;
//  } else if (millis()>=4000 && millis()<5000){
//    motor_power(pwm1, rev1, fwd1, -100);
//    temp1 = -100;
//  } else {
//    motor_power(pwm1, rev1, fwd1, 0);
//    temp1=0;
//  }
//  if (millis() - last_print>=7 && millis()<6500){
//    last_delay = millis();
//    Serial.print(temp1);
//    Serial.print('\t');
//    Serial.print(pos1);
//    Serial.print('\n');
//  }
