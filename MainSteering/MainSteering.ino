#include <Wire.h>
#include "konstanta.h"

int manualMode=0; //0:auto 1:manual

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


void samplingEnc(){
  v1 = pos1 - last_pos1;
  last_pos1 = pos1;
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

void readInput(){
  pot = analogRead(A2);//*360.0/1023;
  in1 = digitalRead(inp1);  in2 = digitalRead(inp2);  in3 = digitalRead(inp3);
  in4 = digitalRead(inp4);  in5 = digitalRead(inp5);  in6 = digitalRead(inp6);
  in7 = digitalRead(inp7);
}


void setup() {
  
  pinMode(speedON, OUTPUT);
  digitalWrite(speedON,0);
  
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  
  //posisi motor Steering
  pinMode(enc1A,INPUT_PULLUP);
  pinMode(enc1B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1A), ISR_INT0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc1B), ISR_INT1, CHANGE);
  
  // set timer 4(D6,D7,D8 divisor to  8 for PWM frequency of  3921.16 Hz
  TCCR4B = TCCR4B & B11111000 | B00000010;   
  
  //motor steering pin
  pinMode(rev2, OUTPUT);   
  pinMode(fwd2, OUTPUT);
  pinMode(pwm2, OUTPUT);

  sp_pos = 0;
  sp_v=0;


  //deklarasi pin digital input - output sistem
  pinMode(revSpeed, OUTPUT);
  pinMode(autoThrottle, OUTPUT);
  
  pinMode(45, OUTPUT);
  pinMode(revOFF, OUTPUT);
  pinMode(A4, INPUT);
  
  digitalWrite(revOFF, manualMode); //1:manual, 0:auto
  digitalWrite(autoThrottle, manualMode);//manualMode); //1:manual 0:auto
  digitalWrite(revSpeed,0);// HIGH); //0:fwd 1:rev

  pinMode(brakeA, OUTPUT);
  pinMode(brakeB, OUTPUT);
  pinMode(inp1, INPUT_PULLUP);  pinMode(inp2, INPUT_PULLUP);  pinMode(inp3, INPUT_PULLUP);  
  pinMode(inp4, INPUT_PULLUP);  pinMode(inp5, INPUT_PULLUP);
  pinMode(inp6, INPUT_PULLUP);  pinMode(inp7, INPUT_PULLUP);
  
  digitalWrite(speedON,1);
  delay(500);
}
void loop() {
  if(manualMode==0){
    if (millis() - lastPID1>=TS ){
      if (abs(sp_pos-pos1)>20){
        pidKecPos();
      }
      else{ 
        pwmSpeed=0;
        sumErrVel = 0;
      }
      
      motor_power(pwm2, rev2, fwd2, pwmSpeed);
      lastPID1 = millis();
    }
  } else {
    motor_power(pwm2, rev2, fwd2, 0);
    pwmSpeed=0;
    sumErrVel = 0;
  }

  if (millis() - lastDigital>=29){
    readInput();
    controller(in5,in6,in2,in4,in7);
    lastDigital = millis();
  }
  
  
}

void pidKecPos(){
  v1 = pos1 - last_pos1;
  last_v1 = v1;
  last_pos1 = pos1;
  
  errPos = sp_pos - pos1;
  sp_vs = Kpvs*errPos; // speed target (pulse/TS)

  //to control max speed of steering in certain position
  if (pos1>schedule_pos || pos1<-1*schedule_pos) 
    max_vs = max_vs2;
  else 
    max_vs = max_vs1;
  threshold(&sp_vs, max_vs);
  
  err_v1 = sp_vs - v1;
  if ( (Kpv*err_v1>-255)&& (Kpv*err_v1<255)){
    sumErrVel = sumErrVel + err_v1;
  }
  int temp_Pi = (int)( Kiv*TS*sumErrVel);
  threshold(&temp_Pi, 255);
    
  pwmSpeed = (int)(Kpv*err_v1 + temp_Pi + Kdv*(err_v1 - err_v2)/TS);
  threshold(&pwmSpeed, 250);
  err_v2 = err_v1;
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
    sp_pos=0;
  } else { // target posisi steering (pulse)
    if (!atas){ 
      sp_pos = 0;
    } else if (!bawah) {
      sp_pos = 0;
    } else if (!kiri) {
      sp_pos = 1150;
    } else if (!kanan) {
      sp_pos = -1150;
    }
  }
    
}
