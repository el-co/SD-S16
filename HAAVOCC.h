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
#define SLOW_SPEED_INIT         740 
#define SUPER_SLOW_SPEED_INIT   530 

#define MED_SPEED               225
#define SLOW_SPEED              80
#define SUPER_SLOW_SPEED        60

#define MED_SPEED_CHK           395
#define SLOW_SPEED_CHK          140
#define SUPER_SLOW_SPEED_CHK    105

#define TURN_30_ENC             60
#define TURN_45_ENC             320
#define TURN_90_ENC             695
#define TURN_180_ENC            1415
#define TURN_SCAN_ENC           20

#define CENTER_FLAME     2
#define MAP_MAX          37
#define REROUTE_MAX      3

#define MLX90614_I2CADDR    0x5A

#define MLX90614_TA         0x06
#define MLX90614_TOBJ       0x07
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
void I2C2Init(int BRG);

//****************************************************************************
//
//                          Motor Control Functions     
//
//****************************************************************************
void MotorSpeedCtrl( uint32 LSpeed, uint32 RSpeed );
void MotorDirectionCtrl( uint8 LDirection, uint8 RDirection);
uint16 PI( uint16 ActualEncoder, uint16 TargetEncoder, uint8 Motor );
void SetSpeed( uint32 Spd );
void SaveCurrEnc( void );
void SetDirection( uint32 Direction );
uint8 CheckFlameDetectors();
uint32 CenterFlame();
void CheckMap();
uint32 CheckCollisionSensors();
void ReRoute();
uint8 ShootWater();
uint8 DecoyCheck();
uint8 FireVerifySens();
uint8 CheckFrontSensor();
void CheckWalls(uint8 Side);
void CheckForDoor();
void SetReRouteTurn(uint8 TurnSide, uint32 TurnEnc);


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

enum ROBOT_MODE
{
    MODE_1,
    MODE_2
};

enum WALL_SENSOR
{
    CLEAR_SENS,
    FRONT_SENS,
    BACK_SENS,
    BOTH_SENS
};

enum SENSOR_SIDE
{
    LEFT_SIDE = 0,
    RIGHT_SIDE = 1,
};

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
    LEFT_30,
    RIGHT_30,  
    LEFT_45,
    RIGHT_45,
    LEFT_90,
    RIGHT_90,
    TURN_180,
    LEFT_SCAN,
    RIGHT_SCAN,
    LEFT_RRT,
    RIGHT_RRT,
    DIR_OFF        
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

enum HLLWY
{
    ONE_BLK = 2,
    TWO_BLK = 1,
    TRD_BLK = 0
};

enum RROUTE
{
  LEFT_REROUTE = 1,
  RIGHT_REROUTE = 2,
  FWD_REROUTE = 0,
  DONE = 3
};

enum FIRE_LOC
{
  ROOM = 0,
  HLLWAY_1 = 1,
  HLLWAY_2 = 2
};

//****************************************************************************
//
//                              Global Variables
//
//****************************************************************************
uint8 State;
uint8 Mode;

uint16 FlameSens[5];
uint16 IRSens[4];
uint8 SensorEvalFlag;
uint32 CntrFlame;
uint32 NextDir;
uint32 NextSpeed;
uint32 MotorDir;

uint32 M1PosEdgeCnt;
uint32 M2PosEdgeCnt;

uint16 IC1PosEdgeTime;
uint16 IC1NegEdgeTime;
uint8  USSensorFlag;
uint32 IC1EdgeCnt;
uint32 PersistantBuffer;

uint8  AdjustSpeedFlag;
uint16 Motor2Speed;
uint16 Motor1Speed;
uint32 TargetEncoder;
uint8 CurrSpeed;
uint8 CurrDir;

float M1Integral;
float M2Integral;

uint8 Extinguish;

uint32 MapDistance[65];
uint8 MapDirection[65];
uint8 FollowingMap;
uint16 MapIndex;
uint32 MapDist;

uint16 RRIndex;
uint32 RRDist;
uint8 RRCheck;

uint32 CatchUp;
uint32 Fixed;
sint32 M1Distance;
sint32 M2Distance;
uint32 FwdTurnDist;
uint8  FwdTurnCheck;
uint32 RvsTurnDist;
uint8  RvsTurnCheck;
uint8  TurnFlag;
uint32 FwdTurnCnt;
uint32 RvsTurnCnt;
uint8  FwdTurnDone;
uint8  RvsTurnDone;

uint32 MaxPWM;
uint32 MinPWM;
uint32 SpeedCheck;

uint8 M2Faster;
uint8 M1Faster;

sint32 distanceDiff;
float  encAdjust;

uint16 m1target;
uint16 m2target;

uint32 tempRvs;
uint32 tempFwd;

float USSensDiff;

uint32 BckAvg;
uint32 FntAvg;

uint32 UnMappedTurn;

uint8 MapDone;
uint32 WaterPulseCnt;
uint8 WaterPulse;

uint32 M1_SSlow;
uint32 M2_SSlow;
uint32 M1_Slow;
uint32 M2_Slow;
uint32 M1_Med;
uint32 M2_Med;

//sint32 SwerveDistance[3];
//uint8 SwerveDirection[3];
sint32 RRouteDistance[3];
uint8 RRouteDirection[3];
uint32 MInd;
uint8 MDist;
uint8 M1Dist;
uint8 M2Dist;

uint8 CllTurn;

uint16 HllwyUSS[3];
uint16 HllwyFWD[3];
uint16 HllwyBCK[3];

uint8 WllChck;

// DEBUG
uint32 TEST4;
uint8  SpeedUp;
sint32 maxDistDiff;
sint32 maxDistDiffNeg;
//float USSens[100];
//uint32 USInd;

uint16 loops;
uint32 l;

// sample for 1 sec
uint32 SixtnHz;
uint32 SecCnt;

/// ir sensors

uint32 xinp;

uint32 xin;

/// Flame Sensor stuff
sint16 flmMidMax;
sint16 flmMidMin;
sint16 PrvMidFlame;
sint16 flMidDif;

uint8 IgnFirst;
uint32 flcnt;
uint32 flcn;

uint8 ScLeft;
uint8 ScRight;
uint8 ScFwd;


uint8 IRCnt;
uint32 RBState;
sint32 RBChng;
uint32 RFState;
sint32 RFChng;

uint16 IRSmpCnt;
uint32 throwaway;

uint8 dir;
uint8 tf;

uint32 AftTrn;

uint16 ss;

uint8 FireCll;

uint32 mmm;
uint8 RRRtrn;
uint8 fwd;
uint32 maxfwd;

float USSens[100];
uint32 USInd; 
uint8 RRTurn;

//uint16 MFireSens[2000];
//uint16 LMFireSens[2000];
//uint16 RMFireSens[2000];
//uint16 RFireSens[2000];
//uint16 LFireSens[2000];
//uint16 LR[2000];

//uint16 FireSens;
//
//uint16 LF[3000];
//uint16 RF[3000];
//uint16 LB[3000];
//uint16 RB[3000];
uint8 filt;

uint16 Sidx;


uint16 WFInitCnt;
uint16 FntMax;
uint16 FntMin;
uint16 BckMax;
uint16 BckMin;
uint32 BckAvg;
uint32 FntAvg;

uint8 WFInit; //
uint8 WFRun; // CheckWalls
uint8 DrChck; // CheckForDoor
uint8 CnslChck; // ConsoleCheck
//uint8 TurnConsole; // Set in ConsoleCheck
uint32 WallCheckSide;
uint8 WallCll;
uint8 ReturnNav;
uint32 FireLocation;

#endif	/* HAVVOCC_H */
