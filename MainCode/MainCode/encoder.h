#include "konstanta.h"

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
