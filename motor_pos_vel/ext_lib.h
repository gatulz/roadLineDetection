#include "konstanta.h"
#include "PID.h"

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
float temp_pos1=0, temp_v1=0, temp_v2=0;
float K_pos1=55/2400, // 2 putaran utk 55 derajat
    K_v1 = 55/2400, // 2 putaran utk 55 derajat
    K_v2 = 4/150;//asumsi kecepatan maksimum 150 pulse per TS2 = 11.5k/sec
