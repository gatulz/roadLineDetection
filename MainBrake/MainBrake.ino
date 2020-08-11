
#define inp1 4 // digital pin
#define inp2 5 // digital pin
#define oneRev 1420 // pulse / rev
#define torqueThreshold 170
//encoder pin
#define enc1A 2 
#define enc1B 3 
//PID constant
#define Kps 0.355
#define Kis 0.0//000145
#define Kds 1//30

#define TS 11*8+1//33*8 //prescaller timer0 ~7800Hz
#define max_vs 10
#define brakeOFF 130*8 

//Motor1
const char fwd1=7; // pin untuk kendali arah motor
const char rev1=8;
const char pwm1=6; // pin untuk pwm1

int calibrate=0;
int ain=0; // torque sensor read (raw)
int kode=0; // command from serial input
int brake=0; // brake sign
int init_ = 0; // variable to initialize brake position
bool in1=0; // read signal 1 from main arduino
bool in2=0; // read signal 2 from main arduino

int errPos=0, sumErrPos=0;
int pwmPos=0, lastErrPos=0;
int sp_pos=0, last_sp_pos = 0;
int sp_vs,  err_vs=0, sumErrVel=0;
int err_v1=0,err_v2=0, v1=0;
int pwmSpeed=0;

String inString = "";    // string to hold input

unsigned long lastBrake=0;
unsigned long lastPID1=0, lastPrint=0;
long pos1=0, last_pos1=0;

//function to read position and direction of hall sensor (encoder)
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

//function to generate PWM to motor driver
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

//function to set a maximum and minimum threshold of a variable
void threshold(int *pwmval, int maxValue){
  if (*pwmval > maxValue){
    *pwmval = maxValue;
  } else if (*pwmval < -1*maxValue){
    *pwmval = -1*maxValue;
  } 
}

// funtion to give command to brake motor through serial monitor
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

  // set timer 0 divisor to  8 for PWM frequency of  7812.50 Hz
  // timer set to 8kHz to reduce sound of motor while moving
  TCCR0B = TCCR0B & B11111000 | B00000010;    
  //motor1
  pinMode(rev1, OUTPUT);   
  pinMode(fwd1, OUTPUT);
  pinMode(pwm1, OUTPUT);
  delay(700);
  lastBrake = millis();
}

void loop() {

  //strToInt(&kode);//sp_pos); // use to test command using serial monitor
  readInput();

  if (millis() - lastPID1>=(TS)){
    samplingEnc();
    if ((abs(sp_pos - pos1)>40) ) // use for move the motor to initial position
      pidPosisi();//KecPos();
    lastPID1 = millis();
  }
  if (millis() - lastPrint>=561){  //print data for debugging
    printData();
    lastPrint=millis();
  }

  if (!init_){
    init_brake(); // initialize brake position
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

// function ro read input from main arduino
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

// sampling of motor speed
void samplingEnc(){
  v1 = pos1 - last_pos1;
  last_pos1 = pos1;
}

// Position PID Algoritm
void pidPosisi(){
  
  errPos = sp_pos - pos1;
  
  int temp_Pi = (int)( Kis*TS*sumErrPos);
  threshold(&temp_Pi, 255);
  
  pwmPos = (int)(Kps*errPos + temp_Pi + Kds*(errPos-lastErrPos)/TS);
  threshold(&pwmPos, 140);
  
  lastErrPos = errPos;
}

// function to initialize brake position 
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

// torque control
void remON(){
  if (brake==1){ // brake is on
    if (ain<torqueThreshold){      
      lastBrake=millis();
      motor_power(pwm1, rev1, fwd1, 220);
    } else if (millis() - lastBrake<brakeOFF){
        motor_power(pwm1, rev1, fwd1, 0);
    } else {
        motor_power(pwm1, rev1, fwd1, pwmPos);
    }
  } else if ((brake==2) || (brake==3)){ // brake off -  move to initial position
        motor_power(pwm1, rev1, fwd1, pwmPos);
  } else if (brake==0){ // brake system is inactive
      motor_power(pwm1, rev1, fwd1,0);
  }
}
