//read 50V : 104,8 - 114,6
//relay di throttle hrs DITUKAR
//P : posisi steering
//v : speed throttle
//s : schedule gain
//steering 3220 max
//max 2300 positif steer ke kiri

#include <Wire.h>
#include "konstanta.h"

#define Kpvs 0.21//0.05//0.1
int manualMode=0; //0:auto 1:manual


unsigned long lastDigital = 0, lastButton=0;
//SETPOINT
int max_vs = 40, max_vs1=70.5, max_vs2=50; //70
int maxErrorV = 30;
int sp_vs=0;//setpoint
int sp_v = 10;
int sp_v2 = 0, last_sp_v2;
int sp_pos=0;
int schedule_pos = 950;//360; // posisi utk scheduling gain Vmax motor steering


unsigned char theta8bit=0;
int theta16=0;
char pitch=0, roll=0;
float arus;
float maxxx = 0;
float a1,b1;
String inString = "";    // string to hold input
String inputString = "";
int complete;
int maxEEAddr;
float tesFloat = 0;
void ISR_INT0(){
  int pinA,pinB;
  pinA=digitalRead(enc1A);
  pinB=digitalRead(enc1B);
  if(pinA==LOW && pinB==LOW){
    pos1--; // CCW
  } else if(pinA==LOW && pinB==HIGH){
    pos1++;
  }else if(pinA==HIGH&& pinB==LOW){
    pos1++; // CCW
  } else if(pinA==HIGH && pinB==HIGH){
    pos1--; // CCW
  }
}
void ISR_INT1(){
  int pinA,pinB;
  pinA=digitalRead(enc1A);
  pinB=digitalRead(enc1B);
  if(pinA==LOW && pinB==LOW){
    pos1++; // CCW
  } else if(pinA==LOW && pinB==HIGH){
    pos1--;
  } else if(pinA==HIGH&& pinB==LOW){
    pos1--; // CCW
  } else if(pinA==HIGH && pinB==HIGH){
    pos1++; // CCW
  }
}

void ISR_INT3(){
  int pinA,pinB;
  pinA=digitalRead(enc2A);
  pinB=digitalRead(enc2B);
  if(pinA==LOW && pinB==LOW){
    pos2--; // CCW
  } else if(pinA==LOW && pinB==HIGH){
    pos2++;
  }else if(pinA==HIGH&& pinB==LOW){
    pos2++; // CCW
  } else if(pinA==HIGH && pinB==HIGH){
    pos2--; // CCW
  }
}
void ISR_INT4(){
  int pinA,pinB;
  pinA=digitalRead(enc2A);
  pinB=digitalRead(enc2B);
  if(pinA==LOW && pinB==LOW){
    pos2++; // CCW
  } else if(pinA==LOW && pinB==HIGH){
    pos2--;
  } else if(pinA==HIGH&& pinB==LOW){
    pos2--; // CCW
  } else if(pinA==HIGH && pinB==HIGH){
    pos2++; // CCW
  }
}


void samplingEnc(){
  v1 = pos1 - last_pos1;
  last_pos1 = pos1;
  v2 = pos2 - last_pos2;
  last_pos2 = pos2;
}
void threshold(int *pwmval, int maxValue){
  if (*pwmval > maxValue){
    *pwmval = maxValue;
  } else if (*pwmval < -maxValue){
    *pwmval = -maxValue;
  } else 
    *pwmval = *pwmval;
}

void motor_power(int pwmPin, int fwd, int rev, int power){
  int pwm_value;
  pwm_value=abs(power);
  if (pwm_value>255)
    pwm_value=255;
  if(power>0){
    digitalWrite(rev,LOW); // turn off
    digitalWrite(fwd,HIGH); // turn off
    analogWrite(pwmPin, pwm_value);
  } else if(power<0){// && v2<2){
    digitalWrite(rev,HIGH); // turn off
    digitalWrite(fwd,LOW); // turn off
    analogWrite(pwmPin, pwm_value);
  }else{
    digitalWrite(fwd,LOW); // turn off
    digitalWrite(rev,LOW); // turn off
    analogWrite(pwmPin, 0);
  }
}

void readCMPS(unsigned char *angle8, int *angle16, char *pitch, char *roll){
  unsigned char high_byte, low_byte;
  
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(ANGLE_8);                     //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 5 bytes from the CMPS12
  // this will give us the 8 bit bearing, 
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS12_ADDRESS, 5);       
  unsigned long last_wire = millis();
  while(Wire.available() < 5){
    if (millis() - last_wire>=200){
      break;
    }
    }// Wait for all bytes to come back
  
  *angle8 = Wire.read();       // Read back the 5 bytes 0-255
  high_byte = Wire.read();     // 0-3599
  low_byte = Wire.read();     //0-3599
  *pitch = Wire.read(); 
  *roll = Wire.read();
  
  *angle16 = high_byte;                 // Calculate 16 bit angle
  *angle16 <<= 8;
  *angle16 += low_byte;

  
}

void throttle(int v_in){
  //float temp_val = v_in*4096/5;
  dac_val = v_in;//temp_val;
  if (dac_val>3000)//3200)
    dac_val = 3000;//3200; //4 volt
  else if (dac_val<0)
    dac_val=0;
  dac.setVoltage(dac_val, false); // false to not save data on eeprom
}

void vehicle_speed(int speed_ve){
  if (speed_ve>=0){
    digitalWrite(revSpeed, LOW);
    throttle(speed_ve);
    stopSign=0;
  } else if (speed_ve<0){
    //digitalWrite(revSpeed,1);// HIGH);
    throttle(0);//-1*speed_ve);
    stopSign=1;
  }
}
//
void readInput(){
  volt = analogRead(A0);//*5.0*10.8/(1023);
  speedometer = analogRead(A1)*5.0*6.0/1023;
  pot = analogRead(A2);//*360.0/1023;
  in1 = digitalRead(inp1);  in2 = digitalRead(inp2);  in3 = digitalRead(inp3);
  in4 = digitalRead(inp4);  in5 = digitalRead(inp5);  in6 = digitalRead(inp6);
  in7 = digitalRead(inp7);
}

void calculateSteering(int angle_vehicle, int theta, int *sp_theta){
  
}

void getGPS(){
   while (Serial2.available()){ // check for gps data
    if(gps.encode(Serial2.read()))// encode gps data
    { 
      gps.f_get_position(&pos0.lat,&pos0.lng); // get latitude and longitude
    }
   }
}

void strToInt(int *sp1,int *sp2,int *sp3){
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == 'w'){
      EEPROM.put(eeAddress, tesFloat);//pos0.lat);
      eeAddress += 2;//(sizeof) pos0.lat;
      tesFloat = (float)eeAddress;
      EEPROM.put(eeAddress, eeAddress);//pos0.lng);
      eeAddress += 2;//(sizeof) pos0.lng;
      tesFloat = (float)eeAddress;
    } else if (inChar == 'p') {
      *sp1 = inString.toInt()-10000;
      if ( (*sp1>4000) || (*sp1<-4000) )
        *sp1=0;
      Serial.print("sp1stee:");
      Serial.println(*sp1);
      // clear the string for new input:
      inString = "";
    } else if (inChar == 'v') {
      *sp3 = inString.toInt()-1000;
      Serial.print("sp2v:");
      Serial.println(*sp2);
      // clear the string for new input:
      inString = "";
    } else if (inChar == 's') {
      *sp3 = inString.toInt();
      Serial.print("sp3:");
      Serial.println(*sp3);
      // clear the string for new input:
      inString = "";
    } 
  }
}

void printData(){
 /* Serial.print("pos: ");         Serial.print(arus);//pos1);  
  Serial.print("\tsp: ");     Serial.print(sp_pos);  
  Serial.print("\terr: ");      Serial.print(sumErrPos);  
  Serial.print("\tpwm: ");      Serial.print(pwmPos);  
  
  Serial.print("\tlat: ");      Serial.print(pos1);//pos0.lat,6);  
  Serial.print("\tlng: ");      Serial.print(pos0.lng,6); 
  
  Serial.print("\t_t: ");    Serial.print(theta16);  
  Serial.print("\t_p: ");    Serial.print(int(pitch)); 
  Serial.print("\t_r: ");    Serial.println(int(roll)); */
  Serial.print(manualMode);
  Serial.print("p1: ");    Serial.print(pos1); 
  Serial.print("p2: ");    Serial.print(pos2); 
  Serial.print("\tv1: ");    Serial.print(v1);  
  
  Serial.print("\tsp: ");    Serial.print(sp_pos); 
  Serial.print("\tpwm: ");    Serial.print(pwmSpeed); 
  Serial.print("\tpwm2: ");    Serial.print(pwmSpeed2); 
  Serial.print("\tv2: ");    Serial.print(last_sp_v2); 
  Serial.print("\tevB2: ");    Serial.print(sumErrVel2); 
  
  Serial.print("\ttarget: ");    Serial.println(sp_v2); 
  //Serial.print("\t_p: ");    Serial.print(pos0.lat,6); 
  //Serial.print("\t_r: ");    Serial.println(pos0.lng),6); 
}

void setup() {
  
  pinMode(speedON, OUTPUT);
  digitalWrite(speedON,0);
  
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  Wire.begin();
  // intitialize GPS
  Serial2.begin(GPSBaud);
  // initialize dac i2c
  dac.begin(0x60); // i2C DAC throttle 
  dac.setVoltage(0, true);
  delay(500);
  // initialize the LCD
  lcd.begin();
  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.print("Hello, world!");
  
  //posisi motor Steering
  pinMode(enc1A,INPUT_PULLUP);
  pinMode(enc1B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1A), ISR_INT0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc1B), ISR_INT1, CHANGE);
  
  //kecepatan sensor HAL
  pinMode(enc2A,INPUT);//_PULLUP);
  pinMode(enc2B,INPUT);//_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc2A), ISR_INT3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2B), ISR_INT4, CHANGE);

  TCCR4B = TCCR4B & B11111000 | B00000010;    // set timer 4(D6,D7,D8 divisor to  8 for PWM frequency of  3921.16 Hz
  //motor1
  pinMode(rev1, OUTPUT);   
  pinMode(fwd1, OUTPUT);
  pinMode(pwm1, OUTPUT);
  //motor2
  pinMode(rev2, OUTPUT);   
  pinMode(fwd2, OUTPUT);
  pinMode(pwm2, OUTPUT);


  sp_pos = 0;
  sp_v=0;

  pinMode(revSpeed, OUTPUT);
  pinMode(autoThrottle, OUTPUT);
  
  pinMode(45, OUTPUT);
  pinMode(revOFF, OUTPUT);
  pinMode(A4, INPUT);

  delay(500);
  
  digitalWrite(revOFF, manualMode); //1:manual, 0:auto
  digitalWrite(autoThrottle, manualMode);//manualMode); //1:manual 0:auto
  digitalWrite(revSpeed,0);// HIGH); //0:fwd 1:rev

  pinMode(brakeA, OUTPUT);
  pinMode(brakeB, OUTPUT);
  pinMode(inp1, INPUT_PULLUP);  pinMode(inp2, INPUT_PULLUP);  pinMode(inp3, INPUT_PULLUP);  
  pinMode(inp4, INPUT_PULLUP);  pinMode(inp5, INPUT_PULLUP);
  pinMode(inp6, INPUT_PULLUP);  pinMode(inp7, INPUT_PULLUP);
  
  digitalWrite(speedON,1);
}
void loop() {
  //last_sp_v2 = sp_v2;
  //strToInt(&sp_pos, &sp_v2, &schedule_pos);//sp_pos);
  //threshold(&sp_v2, maxVThrottle);
  if(manualMode==0){
    vehicle_speed(pwmSpeed2);
    if (millis() - lastPID1>=TS ){
      if (abs(sp_pos-pos1)>20){
        //pidPosisi();
        pidKecPos();
      }
      else{ 
        pwmSpeed=0;
        sumErrVel = 0;
      }
      
      motor_power(pwm2, rev2, fwd2, pwmSpeed);
      lastPID1 = millis();
    }
    
    if (millis() - lastPID2>=TS2){
      pidKecepatan();
      //pidPosisi();
      lastPID2 = millis();
    }
  } else {
    vehicle_speed(0);
    motor_power(pwm2, rev2, fwd2, 0);
    pwmSpeed=0;
    pwmSpeed2=0;
    sumErrVel = 0;
  }
    
//  if (millis() - lastTheta>=11){
//    readCMPS(&theta8bit, &theta16, &pitch, &roll);
//    lastTheta = millis();
//  }
  
  if (millis() - lastPrint>=51){  
    //printData();  
    Serial.print(manualMode);
    Serial.print(stopSign);Serial.print("\t");
    Serial.print(sp_v2);
    Serial.print("\t");
    Serial.print(v2);
    Serial.print("\t");
    Serial.print(pwmSpeed2);
    Serial.print("\t");
//    Serial.print(sp_pos);
//    Serial.print("\t");
//    Serial.print(pwmSpeed);
//    Serial.print("\t");
//    Serial.println(pos1);
    Serial.print(pos0.lat);
    Serial.print("\t");
    Serial.print(pos0.lng);
    
    Serial.print("\t");;
    Serial.println(pos1);
    lastPrint=millis();
  }

  if (millis() - lastDigital>=29){
    brakeSignal();
    readInput();
    controller(in5,in6,in2,in4,in7);
    lastDigital = millis();
  }
  getGPS();
//  if (millis() - lastGPS>=801){
//    getGPS();
//    lastGPS = millis();
//  }
//  if (millis() - lastPrint>=51){  
//    //printData();  
//    Serial.print(pos0.lat);
//    Serial.print("\t");
//    Serial.print(pos0.lng);
//    
//    Serial.print("\t");;
//    Serial.println(pos1);
//    lastPrint=millis();
//  }
  
  
}


void pidKecepatan(){
  
  v2 = pos2 - last_pos2;
  last_pos2 = pos2;
  
  err_vB1 = sp_v2 - v2;
  if ( (err_vB1<-1*maxErrorV && sp_v2>0) || (err_vB1>maxErrorV && sp_v2<0) ){
    pwmSpeed2 = 0;
  } else {
    if (sp_v2==0 && err_vB1==0)
      sumErrVel2=0;
    if ( (Kpv2*err_vB1>-maxThrottle)&& (Kpv2*err_vB1<maxThrottle)){
      sumErrVel2 = sumErrVel2 + err_vB1;
    }
    //threshold(&sumErrVel2,(int) maxThrottle/(Kiv2*TS2));
    int temp_Pi2 = (int)( Kiv2*TS2*sumErrVel2);
    threshold(&temp_Pi2, maxThrottle);
      
    pwmSpeed2 = (int)(Kpv2*err_vB1 + temp_Pi2 + Kdv2*(err_vB1 - err_vB2)/TS2);
    //if (pwmSpeed2>100){
      pwmSpeed2 +=700;
    //}
  }
  threshold(&pwmSpeed2,  maxThrottle);
  err_vB2 = err_vB1;
}

void pidKecPos(){
  v1 = pos1 - last_pos1;
  last_v1 = v1;
  last_pos1 = pos1;
  
  errPos = sp_pos - pos1;
  sp_vs = Kpvs*errPos;
  
  if (pos1>schedule_pos || pos1<-1*schedule_pos)
    max_vs = max_vs2;
  else 
    max_vs = max_vs1;
  threshold(&sp_vs, max_vs);
  
  err_v1 = sp_vs - v1;
//  if (sp_vs==0 && err_v1==0)
//    sumErrVel=0;
  if ( (Kpv*err_v1>-255)&& (Kpv*err_v1<255)){
    sumErrVel = sumErrVel + err_v1;
  }
  int temp_Pi = (int)( Kiv*TS*sumErrVel);
  threshold(&temp_Pi, 255);
    
  pwmSpeed = (int)(Kpv*err_v1 + temp_Pi + Kdv*(err_v1 - err_v2)/TS);
  threshold(&pwmSpeed, 250);
  err_v2 = err_v1;
}

void pidPosisi(){
  v1 = pos1 - last_pos1;
  last_pos1 = pos1;
  last_v1 = v1;
  errPos = sp_pos - pos1;
  if (Kps*errPos<255 && Kps*errPos>-255){
    sumErrPos = sumErrPos + errPos;
  }
  
  int temp_Pi = (int)( Kis*TS*sumErrPos);
  threshold(&temp_Pi, 255);
  
  pwmPos = (int)(Kps*errPos + Kds*(errPos-lastErrPos)/TS);//temp_Pi + 
  threshold(&pwmPos, 230);
  lastErrPos = errPos;
}

void brakeSignal(){
  if (manualMode==0){ //auto mode
    if ((stopSign ==1) || (v2-sp_v2>=7)){// && (v1 - sp_vs)>30){
      digitalWrite(brakeA, 1);
      digitalWrite(brakeB, 1);
  //  } else if ( (v1 - sp_vs)>20){ 
  //    digitalWrite(brakeA, 1);
  //    digitalWrite(brakeB, 0);
  //  }  else if ( (v1 - sp_vs)>12){
  //    digitalWrite(brakeA, 0);
  //    digitalWrite(brakeB, 1);
    } else {
      digitalWrite(brakeA, 1);
      digitalWrite(brakeB, 0);
    }
  } else { //manual mode
      digitalWrite(brakeA, 0);
      digitalWrite(brakeB, 0);
  }
}

void controller(bool atas, bool bawah, bool kiri, bool kanan, bool rst){
  if (!rst && millis()-lastButton>1500){ //for change from manual to auto
    manualMode = !manualMode;
    digitalWrite(revOFF, manualMode); //1:manual, 0:auto
    digitalWrite(autoThrottle, manualMode);//manualMode); //1:manual 0:auto
    pos1=0;// for reset posisition in middle
    lastButton=millis();
  }
  if (!rst){
    sp_v2 = 0;
    sp_pos=0;
  } else {
    if (!atas){
      sp_v2 = 15;
      sp_pos = 0;
    } else if (!bawah) {
      sp_v2 = -15;
      sp_pos = 0;
    } else if (!kiri) {
      sp_v2 = 7;
      sp_pos = 1150;
    } else if (!kanan) {
      sp_v2 = 7;
      sp_pos = -1150;
    } else {
      sp_v2=0;
    }
  }

  //SAFETY MOTOR
  if (pwmSpeed2>2200 && v2<5){
    pwmSpeed2=0;
    manualMode=1; //menjadi manual
    sp_v2=0;
  }
    
}
