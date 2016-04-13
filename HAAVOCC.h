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
#define SCAN_RVS                175

#define RIGHT_TURN_FWD          705
#define LEFT_TURN_FWD           705
#define FULL_TURN_FWD           1415
#define SCAN_FWD                175


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
uint8 CenterFlame();
void CheckMap();
void CheckCollisionSensors();
uint8 ReRoute();
void ShootWater();
uint8 FireVerify();
void CheckFrontSensor();
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

//****************************************************************************
//
//                              Global Variables
//
//****************************************************************************
uint8 Map[84][30];
uint8 State;

uint16 AN0ADC;
uint16 AN1ADC;
uint16 AN4ADC;
uint16 AN5ADC;
uint16 AN6ADC;
uint16 AN9ADC;
uint16 AN10ADC;
uint16 AN11ADC;
uint16 AN12ADC;
uint16 ANADC;
uint8 SensorEvalFlag;

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
#endif	/* HAVVOCC_H */
