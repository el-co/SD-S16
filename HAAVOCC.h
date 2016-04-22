//****************************************************************************
//Filename:		HAAVOCC.h
//
//Description:	HAAVOCC Header file
//
//Date:         03.12.16
//
//Complier:		XC32
//
//****************************************************************************

#ifndef HAVVOCC_H
#define	HAVVOCC_H

#include <xc.h>
#include <p32xxxx.h>   // include chip specific header file
#include <plib.h>      // include peripheral library functions
#include <stdlib.h>
#include <sys/attribs.h>

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef signed short sint16;
typedef unsigned int uint32;
typedef signed int sint32;

#define MED_SPEED_INIT          1880 
#define SLOW_SPEED_INIT         700 
#define SUPER_SLOW_SPEED_INIT   530 

#define MED_SPEED           225
#define SLOW_SPEED          80
#define SUPER_SLOW_SPEED    60

#define MED_SPEED_CHK           375
#define SLOW_SPEED_CHK          135
#define SUPER_SLOW_SPEED_CHK    105

#define RIGHT_TURN_RVS          705
#define LEFT_TURN_RVS           705
#define FULL_TURN_RVS           1415
#define SCAN_RVS                20

#define RIGHT_TURN_FWD          705
#define LEFT_TURN_FWD           705
#define FULL_TURN_FWD           1415
#define SCAN_FWD                20

#define CENTER_FLAME     2

//****************************************************************************
//
//
//                              Global Functions
//
//
//****************************************************************************
//****************************************************************************
//
//                          Initializing Functions     
//
//****************************************************************************
void Init(void);
void GPIOInit(void);
void TimerInit(void);
void PWMInit(void);
void ADCInit(void);
void ICInit(void);
void MapInit(void);

//****************************************************************************
//
//                          Motor Control Functions     
//
//****************************************************************************
void MotorSpeedCtrl( uint32 LSpeed, uint32 RSpeed );
void MotorDirectionCtrl( uint8 LDirection, uint8 RDirection);
uint16 PI( uint16 ActualEncoder, uint16 TargetEncoder, uint8 Motor );
void SetSpeed( uint32 Speed);
void SetDirection( uint32 Direction);
uint8 CheckFlameDetectors();
uint32 CenterFlame();
void CheckMap();
uint32 CheckCollisionSensors();
uint8 ReRoute();
void ShootWater();
uint8 FireVerifyTemp();
uint8 FireVerifySens();
void CheckFrontSensor();
uint8 CheckWalls();

//****************************************************************************
// 
//                              enum Variables
//
//****************************************************************************
enum HAAVOCC_STATES
{
    START,
    NAVIGATE,
    FIRE_DETECTED,
    FIRE_VERIFY,
    RETURN_ROUTE,
    FIRE_EXTINGUISH,
    IDLE
};

//enum NAVIGATE_STATES
//{
//    FOLLOW_ROUTE,
//    REROUTE_TO_FIRE,
//    STOP,
//    RETURN_TO_ROUTE
//};

enum SPEEDS
{
    OFF,
    SUPER_SLOW,
    SLOW,
    MED,
};

enum DIRECTION
{
    FORWARD,
    REVERSE,
    STALL_M1,
    STALL_M2,
    LEFT_90,
    RIGHT_90,
    TURN_180,
    LEFT_SCAN,
    RIGHT_SCAN
};

enum WHEEL_DIRECTION
{
    RVS = 0,
    FWD = 1,
    STLL  = 2,
    IGN = 3
};

enum MOTORS
{
    MOTOR_1 = 0,
    MOTOR_2 = 1
};

enum VEIRFY_FIRE_TEMP
{
    VRFY_FIRE = 0,
    VRFY_FIRE_EXT = 1
};

enum COLLISION_TYPE
{
    NO_COLLISION,
    LEFT_COLLISION,
    RIGHT_COLLISION,
    BOTH_COLLISION
};

enum IR_DETECT
{
    STEADY,
    INC,
    DEC
};

enum WALL_DETECT
{
    AWAY,       //0
    CLSR,       //1
    ALGN_AWAY,  //2
    ALGN_CLSR,  //3
    ALGN        //4
};

//****************************************************************************
//
//                              Global Variables
//
//****************************************************************************
uint8 Map[84][30];
uint8 State;

uint16 FlameSens[5];
uint16 IRSens[4];
uint8 SensorEvalFlag;
uint32 CntrFlame;
uint32 NextDir;
uint32 NextSpeed;
uint32 Rev;
uint32 ScanTotal;
uint32 MotorDir;

uint32 M1PosEdgeCnt;
uint32 M2PosEdgeCnt;

uint16 IC1PosEdgeTime;
uint16 IC1NegEdgeTime;
uint16 DC;
float  Hz;
uint32 TM364PS;
float  CalcSensPer;
float  Input;
uint8  USSensorFlag;
uint32 IC1EdgeCnt;
uint32 PersistantBuffer;

uint8  AdjustSpeedFlag;
uint16 Motor2Speed;
uint16 Motor1Speed;
uint32 TargetEncoder;
uint8 Speed;

float M1Integral;
float M2Integral;

uint8 Extinguish;

uint32 MapDistance[60];
uint8 MapDirection[60];
uint8 FollowingMap;
uint16 MapIndex;
uint32 MapDist;


// DEBUG
uint32 TEST4;
uint32 CatchUp;
uint32 Fixed;

uint32 M1Distance;
uint32 M2Distance;
uint32 M1Skipped;
uint32 M2Skipped;
uint32 FwdTurnDist;
uint8  FwdTurnCheck;
uint32 RvsTurnDist;
uint8  RvsTurnCheck;
uint8  TurnFlag;
uint32 FwdTurnCnt;
uint32 RvsTurnCnt;
uint8  FwdTurnDone;
uint8  RvsTurnDone;

//uint32 StartTurnCnt;
uint32 MaxPWM;
uint32 MinPWM;
uint32 SpeedCheck;

uint32 M2Faster;
uint32 M1Faster;

// Sensor
uint16 i;
float  val;
uint32 SensorCnt;
float  Inches[40];
uint16 ADC10[40];

float  inch;
uint32 distance;
sint32 distanceDiff;
float  encAdjust;
uint8  SpeedUp;
uint32 distDifCnt;
sint32 maxDistDiff;
sint32 maxDistDiffNeg;
        
uint32 TestAdjust;
uint8  Debug;
sint32 AfterDiffM1[20];
sint32 AfterDiffM2[20];
uint32 AfterFixM1[20];
uint32 AfterFixM2[20];
uint32 aft1;
uint32 aft2;
uint32 turnFixCnt1;
uint32 turnFixCnt2;
uint16 lastM1;
uint16 lastM2;

uint16 m1target;
uint16 m2target;

//uint16 M1EncCounts[1002];
//uint16 M2EncCounts[1002]; 
//uint16 M1PWMCounts[1002];
//uint16 M2PWMCounts[1002]; 
////sint16 M1PI[1002];
////sint16 M2PI[1002]; 
//float  M1PIf[1002];
//float  M2PIf[1002]; 
//sint16  M1PIerror[1002];
//sint16  M2PIerror[1002]; 
//sint16  MdistDiff[1002]; 
uint16 encCnt;    
uint8 fl;

uint32 tempRvs;
uint32 tempFwd;
//uint16 slowDown[6000];
//uint16 sD;
float SensDiff;

float Sens[100];
uint32 in;

uint16 Rsens[400];
uint16 Rsens2[400];
sint16 dif[400];


uint16 Rs[400];
uint16 Rs2[400];


uint16 Ls[400];
uint16 Ls2[400];


uint32 avg;
uint32 avg2;
sint16 maxPos;
sint16 maxNeg;
sint16 minPos;
sint16 minNeg;
uint32 xin;
float L1;
uint16 c1;
uint16 c2;
float L2;
uint32 ii;



uint16 FlL[200];
uint16 FlR[200];
uint16 FlM[200];
uint32 flcnt;

uint16 FlameSensData[500];
uint16 FlameDataMin;
uint16 FlameDataMax;
uint32 FlameSensIdx;
uint32 FlameSensCnt;
uint8 ScLeft;
uint8 ScRight;
uint32 StartUp;

uint32 OneSec;
uint32 SmplCnt;
uint32 IRSamp;

uint32 difCnt;
uint32 difIRTemp;
uint32 difIRSamp;
uint32 xin2;

uint32 RBCnt;
uint32 RBTemp;
uint32 RBSamp;
uint32 RFCnt;
uint32 RFTemp;
uint32 RFSamp;

sint32 RThresh;
sint32 LThresh;
uint16 ThreshCnt;
uint8 clsr[400];
uint8 cls;
uint8 EncEq;

uint16 mmaxB;
uint16 mminB;
uint16 mmaxF;
uint16 mminF;

uint16 loops;

uint8 IRCnt;
//uint32 RBCnt;
uint32 RBState;
uint32 RBSamp;
sint32 RBChng;
//uint32 RFCnt;
uint32 RFState;
uint32 RBSamp;
sint32 RFChng;
uint32 WallState;
uint32 WallStCnt;
uint32 WS[400];
uint32 WSC[400];
uint16 RBSmp[5];
uint16 RFSmp[5];
uint16 IRSmpCnt;
uint32 throwaway;
uint32 l;

uint32 incCnt;
uint32 lastSamp;
uint32 RBThresh;
uint32 RFThresh;



#endif	/* HAVVOCC_H */
