#include <LiquidCrystal.h>
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Display mode enums
#define MODE_LAUNCH 0
#define MODE_ORBIT 1
#define MODE_NORM_MAX 1
#define MODE_AUTO_READY 900
#define MODE_AUTO_P01_READY 901
#define MODE_AUTO_P02_READY 902
#define MODE_AUTO_P03_READY 903
#define MODE_AUTO_P04_READY 904
#define MODE_AUTO_MAX 904
#define MODE_AUTO 9900  // autopilot-ready
#define MODE_AUTO_P01 9901
#define MODE_AUTO_P02 9902
#define MODE_AUTO_P03 9903
#define MODE_AUTO_P04 9904
#define MODE_AUTO_OFFSET  (MODE_AUTO-MODE_AUTO_READY)
#define MODE_MAX 2
int mode;
bool mode_pressed=false;  // for debounce

//shifter bits for LEDs

#define GLED 5
#define YLED 6
#define RLED 7
#define SASLED 0
#define RCSLED 1

//pins for input
#define SASPIN 8
//#define RCSPIN 9
#define THROTTLEPIN 0
#define MODEPIN 6
#define AUTOPIN 10

#define THROTTLEDB 4 //Throttle axis deadband

//Input enums
#define SAS 7
#define RCS 6
#define LIGHTS 5
#define GEAR 4
#define BRAKES 3
#define PRECISION 2
#define ABORT 1
#define STAGE 0

//Action group statuses
#define AGSAS      0
#define AGRCS      1
#define AGLight    2
#define AGGear     3
#define AGBrakes   4
#define AGAbort    5
#define AGCustom01 6
#define AGCustom02 7
#define AGCustom03 8
#define AGCustom04 9
#define AGCustom05 10
#define AGCustom06 11
#define AGCustom07 12
#define AGCustom08 13
#define AGCustom09 14
#define AGCustom10 15

//SAS Modes
#define SMOFF           0
#define SMSAS           1
#define SMPrograde      2
#define SMRetroGrade    3
#define SMNormal        4
#define SMAntinormal    5
#define SMRadialIn      6
#define SMRadialOut     7
#define SMTarget        8
#define SMAntiTarget    9
#define SMManeuverNode  10

//Navball Target Modes
#define NAVBallIGNORE   0
#define NAVBallORBIT    1
#define NAVBallSURFACE  2
#define NAVBallTARGET   3

//macro
#define details(name) (uint8_t*)&name,sizeof(name)

//if no message received from KSP for more than 2s, go idle
#define IDLETIMER 2000
#define CONTROLREFRESH 25

//warnings
#define GWARN 9                  //9G Warning
#define GCAUTION 5               //5G Caution
#define FUELCAUTION 10.0         //10% Fuel Caution
#define FUELWARN 5.0             //5% Fuel warning

unsigned long deadtime, deadtimeOld, controlTime, controlTimeOld;
unsigned long now;

boolean Connected = false;

byte caution = 0, warning = 0, id;

struct VesselData
{
    byte id;                //1
    float AP;               //2
    float PE;               //3
    float SemiMajorAxis;    //4
    float SemiMinorAxis;    //5
    float VVI;              //6
    float e;                //7
    float inc;              //8
    float G;                //9
    long TAp;               //10
    long TPe;               //11
    float TrueAnomaly;      //12
    float Density;          //13
    long period;            //14
    float RAlt;             //15
    float Alt;              //16
    float Vsurf;            //17
    float Lat;              //18
    float Lon;              //19
    float LiquidFuelTot;    //20
    float LiquidFuel;       //21
    float OxidizerTot;      //22
    float Oxidizer;         //23
    float EChargeTot;       //24
    float ECharge;          //25
    float MonoPropTot;      //26
    float MonoProp;         //27
    float IntakeAirTot;     //28
    float IntakeAir;        //29
    float SolidFuelTot;     //30
    float SolidFuel;        //31
    float XenonGasTot;      //32
    float XenonGas;         //33
    float LiquidFuelTotS;   //34
    float LiquidFuelS;      //35
    float OxidizerTotS;     //36
    float OxidizerS;        //37
    uint32_t MissionTime;   //38
    float deltaTime;        //39
    float VOrbit;           //40
    uint32_t MNTime;        //41  Time to next node (s) [0 when no node]
    float MNDeltaV;         //42  Delta V for next node (m/s) [0 when no node]
    float Pitch;            //43
    float Roll;             //44
    float Heading;          //45
    uint16_t ActionGroups;  //46  status bit order:SAS, RCS, Light, Gear, Brakes, Abort, Custom01 - 10
    byte SOINumber;         //47  SOI Number (decimal format: sun-planet-moon e.g. 130 = kerbin, 131 = mun)
    byte MaxOverHeat;       //48  Max part overheat (% percent)
    float MachNumber;       //49
    float IAS;              //50  Indicated Air Speed
    byte CurrentStage;      //51  Current stage number
    byte TotalStage;        //52  TotalNumber of stages
    float TargetDist;       //53  Distance to targeted vessel (m)
    float TargetV;          //54  Target vessel relative velocity
    byte NavballSASMode;    //55  Combined byte for navball target mode and SAS mode
                                    // First four bits indicate AutoPilot mode:
                                    // 0 SAS is off  //1 = Regular Stability Assist //2 = Prograde
                                    // 3 = RetroGrade //4 = Normal //5 = Antinormal //6 = Radial In
                                    // 7 = Radial Out //8 = Target //9 = Anti-Target //10 = Maneuver node
                                    // Last 4 bits set navball mode. (0=ignore,1=ORBIT,2=SURFACE,3=TARGET)
};

struct HandShakePacket
{
  byte id;
  byte M1;
  byte M2;
  byte M3;
};

struct ControlPacket {
  byte id;
  byte MainControls;                  //SAS RCS Lights Gear Brakes Precision Abort Stage
  byte Mode;                          //0 = stage, 1 = docking, 2 = map
  unsigned int ControlGroup;          //control groups 1-10 in 2 bytes
  byte NavballSASMode;                //AutoPilot mode
  byte AdditionalControlByte1;
  int Pitch;                          //-1000 -> 1000
  int Roll;                           //-1000 -> 1000
  int Yaw;                            //-1000 -> 1000
  int TX;                             //-1000 -> 1000
  int TY;                             //-1000 -> 1000
  int TZ;                             //-1000 -> 1000
  int WheelSteer;                     //-1000 -> 1000
  
  int Throttle;                       //    0 -> 1000
  int WheelThrottle;                  //    0 -> 1000
};

HandShakePacket HPacket;
VesselData VData;
ControlPacket CPacket;

class Shift595
{
  int m_latchPin;
  int m_clockPin;
  int m_dataPin;
  unsigned int m_state;

  void shift_state()
  {
    digitalWrite(m_latchPin,LOW);
    shiftOut(m_dataPin,m_clockPin,MSBFIRST,m_state);
    digitalWrite(m_latchPin,HIGH);
  }
  
public:
  Shift595(int latchPin, int clockPin, int dataPin):m_latchPin(latchPin),m_clockPin(clockPin),m_dataPin(dataPin)
  {
    m_state=0;
    pinMode(latchPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, OUTPUT);
  }

  void setBit(int bitNum)
  {
    byte mask=1<<bitNum;
    m_state|=mask;
    shift_state();
  }

  void clearBit(int bitNum)
  {
    byte mask=~(1<<bitNum);
    m_state&=mask;
    shift_state();
  }

  void allOn()
  {
    m_state=-1;
    shift_state();
  }

  void allOff()
  {
    m_state=0;
    shift_state();
  }
};

//Pin connected to ST_CP of 74HC595
//int latchPin = 9; // green
//Pin connected to SH_CP of 74HC595
//int clockPin = 13; // yellow
////Pin connected to DS of 74HC595
//int dataPin = 7;  // blue
Shift595* shifter;

void setup() {
  shifter=new Shift595(9,13,7);
  Serial.begin(38400);
  lcd.begin(16,2);
  lcd.clear();
  lcd.print("WAITING FOR LINK");

  initLEDS();
  InitTxPackets();
  controlsInit();

  LEDSAllOff();
}

void loop()
{
  input();
  output();
}


















