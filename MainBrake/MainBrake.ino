
#define inp1 4 
#define inp2 5
#define oneRev 1420
#define batasCek 300
#define stallSpeed 10
#define torqueThreshold 170
//encoder
#define enc1A 2 
#define enc1B 3 
//PID constant
#define Kpvs 0.1
#define Kpv 3.2
#define Kiv 0.011
#define Kdv 0
#define TS 11*8+1//33*8 //prescaller timer0 ~7800Hz
#define max_vs 10
int calibrate=0;
int ain=0;
int kode=0;
int brake=0;
int init_ = 0;
int posMax = -1000;
bool in1=0;
bool in2=0;

int brakeON=550*8;
int brakeREV=400*8;
int brakeOFF=130*8;

unsigned long lastBrake=0;

int stall =0 ;

//Motor1
const char fwd1=7; // pin untuk kendali arah motor
const char rev1=8;
const char pwm1=6; // pin untuk pwm1


unsigned long lastPID1=0, lastPrint=0;

//PID Pos
#define Kps 0.355
#define Kis 0.0//000145
#define Kds 1//30
int errPos=0, sumErrPos=0;
int pwmPos=0, lastErrPos=0;

String inString = "";    // string to hold input

int sp_pos=0, last_sp_pos = 0;
int sp_vs,  err_vs=0, sumErrVel=0;
long pos1=0, last_pos1=0;
int err_v1=0,err_v2=0, v1=0;
int pwmSpeed=0;



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

//use to generate PWM to motor driver
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
    analogWrite(pwm, 0);
  }
}

//set a maximum and minimum threshold of a variable
void threshold(int *pwmval, int maxValue){
  if (*pwmval > maxValue){
    *pwmval = maxValue;
  } else if (*pwmval < -1*maxValue){
    *pwmval = -1*maxValue;
  } 
}

// funtion to give 
void strToInt(int *setpoint){
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      *setpoint = inString.toInt();//-10000;
      Serial.print("setpoint:");
      Serial.println(*setpoint);
      // clear the string for new input:
      inString = "";
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
 //posisi motor bRAKE
  pinMode(enc1A,INPUT_PULLUP);
  pinMode(enc1B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc1A), ISR_INT0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc1B), ISR_INT1, CHANGE);
  
  TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
  //motor1
  pinMode(rev1, OUTPUT);   
  pinMode(fwd1, OUTPUT);
  pinMode(pwm1, OUTPUT);
  delay(700);
  lastBrake = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  //last_sp_pos = sp_pos;

  //strToInt(&kode);//sp_pos);
  readInput();

  if (millis() - lastPID1>=(TS)){
    samplingEnc();
    if ((abs(sp_pos - pos1)>40) )
      pidPosisi();//KecPos();
    lastPID1 = millis();
  }
  if (millis() - lastPrint>=561){  
    printData();
    lastPrint=millis();
  }

  if (!init_){
    init_brake();
  } else {
    remON();
  }
}

void printData(){
  Serial.print(pos1);
  Serial.print("\t");
  Serial.print(sp_pos);
  Serial.print("\t");
  Serial.print(pwmPos);
  Serial.print("\t");
  Serial.print(brake);
  Serial.print(in2);
  Serial.println(ain);
  
}

void readInput(){
  ain=analogRead(A0); //current of brake motor I (mA) = (ain/1023)*(40/13.3)
  in1 = digitalRead(inp1); //brake signal from main controller
  in2 = digitalRead(inp2); //brake signal from main controller

  if ( (in1==1 && in2==1) || (kode==3) ){ //full brake
    brake=1;
  } else if ( (in1==1 && in2==0) || (kode==2) ){ //auto no brake
    brake=2;
  } else if ( (in1==0 && in2==1) || (kode==1) ){ // auto no brake
    brake=3;
  } else { //manual, brake system OFF
    brake=0;
  }
}


void samplingEnc(){
  v1 = pos1 - last_pos1;
  last_pos1 = pos1;
}

void pidPosisi(){
  
  errPos = sp_pos - pos1;
  
  int temp_Pi = (int)( Kis*TS*sumErrPos);
  threshold(&temp_Pi, 255);
  
  pwmPos = (int)(Kps*errPos + temp_Pi + Kds*(errPos-lastErrPos)/TS);
  threshold(&pwmPos, 140);
  
  lastErrPos = errPos;
}

void init_brake(){
  
  if (millis() <17000){ //2 detik (2*8000
    if (calibrate==0){
      motor_power(pwm1, rev1, fwd1, 230);
      if (ain>torqueThreshold){
        calibrate=1;
        //sp_pos = pos1-440;
        lastBrake=millis();
      }
      
    } else {
      if (millis() - lastBrake<1600)
        motor_power(pwm1, rev1, fwd1, -110);
      else 
        motor_power(pwm1, rev1, fwd1, 0);
    }
    
  } else {
    init_=1;
    motor_power(pwm1, rev1, fwd1, 0);
    sp_pos=pos1;
  }
}

void remON(){
  if (brake==1){
    if (ain<torqueThreshold){      
      lastBrake=millis();
      motor_power(pwm1, rev1, fwd1, 220);
    } else if (millis() - lastBrake<brakeOFF){
        motor_power(pwm1, rev1, fwd1, 0);
    } else {
        motor_power(pwm1, rev1, fwd1, pwmPos);
    }
  } else if ((brake==2) || (brake==3)){
        motor_power(pwm1, rev1, fwd1, pwmPos);
  } else if (brake==0){
      motor_power(pwm1, rev1, fwd1,0);
  }
}
