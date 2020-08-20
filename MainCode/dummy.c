
#include <Wire.h>
#include "konstanta.h"

/*struct GPSPosition {
	float lat;
	float lng;
};

GPSPosition pos;*/


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


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  Wire.begin();
  // intitialize GPS
  ss.begin(GPSBaud);
  // initialize dac i2c
  dac.begin(0x60); // i2C DAC throttle 
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
  pinMode(enc2A,INPUT_PULLUP);
  pinMode(enc2B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc2A), ISR_INT3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2B), ISR_INT4, CHANGE);
  
  //motor1
  pinMode(rev1, OUTPUT);   
  pinMode(fwd1, OUTPUT);
  pinMode(pwm1, OUTPUT);
  //motor2
  pinMode(rev2, OUTPUT);   
  pinMode(fwd2, OUTPUT);
  pinMode(pwm2, OUTPUT);
  
}
void loop() {
  
  if (millis() - lastPID1>=TS){
    pidKecepatan();
    if (abs(sp_pos-pos1)>20){
      pidPosisi();
    } else {
      pwmPos=0;
    }
    lastPID1 = millis();
  }
  if (millis() - lastPrint>=200){  
    Serial.print("pos1: ");
    Serial.print(v1);  //menampilkan kecepatan
    Serial.print("\tsp_pos: ");
    Serial.print(sp_v);  //menampilkan kecepatan
    Serial.print("\terr: ");
    Serial.print(abs(sp_v-v1));  //menampilkan kecepatan
    Serial.print("\tcontrol: ");
    Serial.println(pwmSpeed);  //menampilkan kecepatan
    lastPrint=millis();
  }
  if(millis()<5000){
//   motor_power(pwm1, rev1, fwd1, -255);
      //motor_power(pwm2, rev2, fwd2, pwmPos);
      motor_power(pwm2, rev2, fwd2, pwmSpeed);

  } else {
    motor_power(pwm2, rev2, fwd2, 0);
  }
  //motor_power(pwm1, rev1, fwd1, pwmPos);
}

void pidPosisi(){
  
  errPos = sp_pos - pos1;
  if (Kps*errPos<255 && Kps*errPos>-255)
    sumErrPos = sumErrPos + errPos;
  pwmPos = (int)(Kps*errPos + Kis*sumErrPos + Kds*(errPos-lastErrPos));
  //threshold(&pwmPos, 255);
  
  lastErrPos = errPos;
}

void pidKecepatan(){
  v1 = pos1 - last_pos1;
  last_pos1 = pos1;

  err_v1 = sp_v - v1;
  if ((Kpv*err_v1<255) && (Kpv*err_v1>-255))
    sumErrVel = sumErrVel + err_v1;
  pwmSpeed = (int)(Kpv*err_v1 + Kiv*TS*sumErrVel + Kdv*(err_v1 - err_v2)/TS);
  //threshold(&pwmSpeed, 255);
  err_v2 = err_v1;
}

void threshold(int* pwm, int maxValue){
  if (*pwm > maxValue){
    *pwm = maxValue;
  } else if (*pwm < -1*maxValue){
    *pwm = -1*maxValue;
  } 
}

void motor_power(int pwm, int fwd, int rev, int power){
  int pwm_value;
  pwm_value=abs(power);
  if (pwm_value>255)
    pwm_value=255;
  if(power>0){
    digitalWrite(rev,LOW); // turn off
    digitalWrite(fwd,HIGH); // turn off
    analogWrite(pwm, pwm_value);
  } else if(power<0){
    digitalWrite(rev,HIGH); // turn off
    digitalWrite(fwd,LOW); // turn off
    analogWrite(pwm, pwm_value);
  }else{
    digitalWrite(fwd,LOW); // turn off
    digitalWrite(rev,LOW); // turn off
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
  
  while(Wire.available() < 5);        // Wait for all bytes to come back
  
  *angle8 = Wire.read();               // Read back the 5 bytes 0-255
  high_byte = Wire.read();			// 0-3599
  low_byte = Wire.read();			//0-3599
  *pitch = Wire.read();	
  *roll = Wire.read();
  
  *angle16 = high_byte;                 // Calculate 16 bit angle
  *angle16 <<= 8;
  *angle16 += low_byte;
    
}

void throttle(float v_in){
  float temp_val = v_in*4096/5;
  dac_val = temp_val;
  if (dac_val>3200)
    dac_val = 3200; //4 volt
  dac.setVoltage(dac_val, false); // false to not save data on eeprom
}
//
void readAnalogInput(){
  volt = analogRead(A0);//*5.0*10.8/(1023);
  speedometer = analogRead(A1)*5.0*6.0/1023;
  pot = analogRead(A2);//*360.0/1023;
}

void calculateSteering(int angle_vehicle, int theta, int *sp_theta){
	
}

void getGPS(){
	
}
