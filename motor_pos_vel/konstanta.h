#define maxPWM 200


#define TS1 11
#define TS2 13

////////////
// PIN MODE
////////////
//Motor1
const char fwd1=9; // pin untuk kendali arah motor
const char rev1=10;
const char pwm1=11; // pin untuk pwm1

//Motor2
const char fwd2=6;//7; // pin untuk kendali arah motor
const char rev2=7;//6;
const char pwm2=8; // pin untuk pwm1

//encoder
const char enc1A=2;
const char enc1B=3;
const char enc2A=18;
const char enc2B=19;

////////////
// VARIABLE
////////////


//other variable
unsigned long last_print=0, 
              last_delay=0,
              last_pid1=0, 
              last_pid2=0;

String inString = "";
int inByte =0 , servoSpeed=0,
    servoPos=0, servoTarget=0,
    dig=0;
