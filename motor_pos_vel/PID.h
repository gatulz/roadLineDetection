
#define Kpvs 0.25//0.05//0.1

//float Kps=0.27;//0.32//0.95 27
//float Kis=0.00008;//4//0.00045
////float Kds=25;
//float Kps=0.27;//0.32//0.95 27
//float Kis=0.00021;//4//0.00045
//float Kds=28;

//float Kps=0.75;//0.32//0.95 27 47 67
//float Kis=0.00041;//21;//4//0.00045
//float Kds=19;

float Kps=5.0;//0.32//0.95 27 47 67 4.5
float Kis=0.0008;//91;//21;//4//0.00045
float Kds=30;//14; 55

//float Kps=0.7;//0.32//0.95 27 47 67
//float Kis=0.00151;//21;//4//0.00045
//float Kds=17;
//
#define Kpv 1.8//3.2//1.5//3.2
#define Kiv 0.0009//0.003//0.011
#define Kdv 0.0//0
#define Kpv2 1.5//3.2//1.5//3.2
#define Kiv2 0.0016//0.003//0.011
#define Kdv2 0.0//0

//#define Kpv2 1.2 //20.2
//#define Kiv2 0.0009//0.055 //0.04
//#define Kdv2 0.0



// PID Variable
long sumErrPos=0;
long schedule_pos = 950;
long pos1=0;
long v1=0, 
    last_pos1=0, last_v1=0,
     errPos=0,
    err_v1=0, err_v2=0,
    sumErrVel=0, sp_pos=0,
     lastErrPos=0;
//pidkecpos
long last_pos2=0,sumErrVel2=0;
long pos2=0;
int  maxErrorV = 30,
    v2 = 0, err_vB1=0, err_vB2=0,
    sp_v2=0, sp_vs = 0;
    
int pwmPos=0,pwmSpeedPos=0;
int pwmSpeed2 = 0,
    pwmSpeed = 0,
    max_vs=40,
    max_vs1=50,
    max_vs2=50;
//end of PID variable

void threshold(int *pwmval, int maxValue){
  if (*pwmval > maxValue){
    *pwmval = maxValue;
  } else if (*pwmval < -maxValue){
    *pwmval = -maxValue;
  } else 
    *pwmval = *pwmval;
}

void pidKecepatan(){
  
  v2 = pos2 - last_pos2;
  last_pos2 = pos2;
  
  err_vB1 = sp_v2 - v2;
  if ( (err_vB1<-1*maxErrorV && sp_v2>0) || (err_vB1>maxErrorV && sp_v2<0) ){
    pwmSpeed2 = 0;
  } else {
//    if (sp_v2==0 && err_vB1==0)
//      sumErrVel2=0;
//    if ( (Kpv2*err_vB1>-maxPWM)&& (Kpv2*err_vB1<maxPWM)){
//      sumErrVel2 = sumErrVel2 + err_vB1;
//    }
    sumErrVel2 = sumErrVel2 + err_vB1;
    //threshold(&sumErrVel2,(int) maxThrottle/(Kiv2*TS2));
    int temp_Pi2 = round( Kiv2*TS2*sumErrVel2);
    threshold(&temp_Pi2, round(0.8*maxPWM));
      
    pwmSpeed2 = round(Kpv2*err_vB1 + temp_Pi2 + Kdv2*(err_vB1 - err_vB2)/TS2);
    //if (pwmSpeed2>100){
      //pwmSpeed2 +=700;
    //}
  }
  if (pwmSpeed2>maxPWM)
    pwmSpeed2 = maxPWM;
  else if (pwmSpeed2<-maxPWM)
    pwmSpeed2 = -maxPWM;

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
  if ( (Kpv*err_v1>-255)&& (Kpv*err_v1<255)){
    sumErrVel = sumErrVel + err_v1;
  }
  int temp_Pi = round( Kiv*TS1*sumErrVel);
  threshold(&temp_Pi, maxPWM);
    
  pwmSpeedPos = round(Kpv*err_v1 + temp_Pi + Kdv*(err_v1 - err_v2)/TS1);
  threshold(&pwmSpeedPos, maxPWM);
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
  
  
  int temp_Pi = round( Kis*TS1*sumErrPos);
  threshold(&temp_Pi, 0.85*255);
  
  pwmPos = round(Kps*errPos + Kds*(errPos-lastErrPos)/TS1)+ temp_Pi; 
  threshold(&pwmPos, 255);
  lastErrPos = errPos;
}

float pos1_f = 0, errPos1_f = 0, sp_pos_f,lastErrPos1_f=0;
void pidPosisi2(){
  pos1_f = pos1*(360/2400); // 1 rev = 360
  errPos1_f = sp_pos - pos1_f;
  if (Kps*errPos1_f<255 && Kps*errPos1_f>-255){
    sumErrPos = sumErrPos + errPos1_f;
  }
  
  int temp_Pi = (int)( Kis*TS1*sumErrPos);
  threshold(&temp_Pi, 0.8*maxPWM);
  
  pwmPos = (int)(Kps*errPos1_f + Kds*(errPos1_f-lastErrPos)/TS1)+ temp_Pi; 
  threshold(&pwmPos, maxPWM);
  lastErrPos1_f = errPos1_f;
}

void motor_power(int pwmPin, int fwd, int rev, int power){
  int pwm_value;
  pwm_value=abs(power);
  if (pwm_value>255)
    pwm_value=255;
  if(power>5){
    digitalWrite(rev,LOW); // turn off
    digitalWrite(fwd,HIGH); // turn off
    analogWrite(pwmPin, pwm_value);
  } else if(power<-5){// && v2<2){
    digitalWrite(rev,HIGH); // turn off
    digitalWrite(fwd,LOW); // turn off
    analogWrite(pwmPin, pwm_value);
  }else{
    digitalWrite(fwd,HIGH); // turn off
    digitalWrite(rev,HIGH); // turn off
    analogWrite(pwmPin, 240);
  }
}
