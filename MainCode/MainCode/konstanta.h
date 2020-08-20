#define inp1 31
#define inp2 32 //kiri
#define inp3 33
#define inp4 34//kanan
#define inp5 35 //atas
#define inp6 36 //bawah
#define inp7 37 //reset
bool in1, in2, in3, in4, in5, in6, in7;
#define brakeA 47
#define brakeB 49
bool stopSign=0;


#define speedON 45//41//44
#define autoThrottle 44//45
#define revSpeed 42
#define revOFF 43
// deklarasi library
// DAC THROTTLE
#include <Adafruit_MCP4725.h> //DAC for throttle
Adafruit_MCP4725 dac; // constructor

#include <LiquidCrystal_I2C.h>
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define Kps 0.32//32//65
#define Kis 0.0000//4//0.00045
#define Kds 10

#define Kpv 2.8//3.2//1.5//3.2
#define Kiv 0.0015//0.003//0.011
#define Kdv 7//0
#define TS 33

#define Kpv2 22.2 //20.2
#define Kiv2 0.055 //0.04
#define Kdv2 0.0
#define TS2 73

#define maxThrottle 3000
#define maxVThrottle 30
int v2,sumErrVel2=0, err_vB1=0, err_vB2=0, sp_vB2=10;
int pwmSpeed2=0;



//CMPS12
#define CMPS12_ADDRESS 0x61  // Address of CMPS12 shifted right one bit for arduino wire library
#define ANGLE_8  1           // Register to read 8bit angle from




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

const int analogInPin = A0;

int sensorValue = 0; // value read from the pot
int outputValue = 0; // value output to the pwm1 (analog out)
int state=0;
int error=0, error_integral=0,
      control = 0, target=0;
unsigned long lastSample=0, lastPrint=0, 
  lastPID1=0, lastZero=0, lastSampling=0,
  lastTheta=0, lastGPS=0, lastPID2=0;
int kec=0;

int v1, last_v1, errPos, sumErrPos, lastErrPos;
int pwmPos, pwmSpeed;
int err_v1=0, sumErrVel=0, err_v2=0;
long pos1, pos2,last_pos1=0, last_pos2 =0;


//TA1
//Variable
float volt, speedometer, pot;
byte disp=1;
int dac_val=0, delay_time =0;

bool loggerON=0;
bool lcdON = 1;
bool gpsON = 0;

///////
//GPS//
///////
#include <TinyGPS.h>
#include <SoftwareSerial.h>
static const int RXPin = 17, TXPin = 16;
static const uint32_t GPSBaud = 9600;
double test;

// The TinyGPS++ object
TinyGPS gps;
// The serial connection to the GPS device
//SoftwareSerial ssGPS(RXPin, TXPin);

#include <EEPROM.h>
int eeAddress = 0;
int nSetPoint = 100;

struct GPSPosition {
  float lat;
  float lng;
};
GPSPosition pos0;
