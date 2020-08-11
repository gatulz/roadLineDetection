// pin declaration
#define inp1 31
#define inp2 32 //kiri
#define inp3 33
#define inp4 34//kanan
#define inp5 35 //atas
#define inp6 36 //bawah
#define inp7 37 //reset
#define speedON 45//41//44
#define autoThrottle 44//45
#define revSpeed 42
#define revOFF 43
#define brakeA 47
#define brakeB 49

// PID constant
#define Kpvs 0.21//0.05//0.1
#define Kpv 2.8//3.2//1.5//3.2
#define Kiv 0.0015//0.003//0.011
#define Kdv 7//0
#define TS 33

bool in1, in2, in3, in4, in5, in6, in7;
bool stopSign=0;


//Steering Motor Pin
const char fwd2=6;//7; // pin untuk kendali arah motor
const char rev2=7;//6;
const char pwm2=8; // pin untuk pwm1

//encoder pin
const char enc1A=2;
const char enc1B=3;

//Variable
int sensorValue = 0; // value read from the pot
int outputValue = 0; // value output to the pwm1 (analog out)
int state=0;
int error=0, error_integral=0,
      control = 0, target=0;
unsigned long  lastPrint=0, lastPID1=0, lastSampling=0;
int kec=0;

int v1, last_v1, errPos, sumErrPos, lastErrPos;
int pwmPos, pwmSpeed;
int err_v1=0, sumErrVel=0, err_v2=0;
long pos1, pos2,last_pos1=0, last_pos2 =0;

float pot;
byte disp=1;
int dac_val=0, delay_time =0;
bool loggerON=0;
bool lcdON = 1;
bool gpsON = 0;


unsigned long lastDigital = 0, lastButton=0;
//SETPOINT
int max_vs = 40, max_vs1=70.5, max_vs2=50; //70
int maxErrorV = 30;
int sp_vs=0;//setpoint
int sp_v = 10;
int sp_pos=0;
int schedule_pos = 950;//360; // posisi utk scheduling gain Vmax motor steering
