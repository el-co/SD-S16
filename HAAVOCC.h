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
#define SLOW_SPEED_INIT         660 
#define SUPER_SLOW_SPEED_INIT   530 

#define MED_SPEED           225
#define SLOW_SPEED          80
#define SUPER_SLOW_SPEED    60

#define MED_SPEED_CHK           375
#define SLOW_SPEED_CHK          135
#define SUPER_SLOW_SPEED_CHK    105

#define RIGHT_45_TURN_RVS       320 // inc to to turn more, dec to turn less
#define RIGHT_45_TURN_FWD       320 // change both rvs and fwd to match

#define LEFT_45_TURN_RVS        320 
#define LEFT_45_TURN_FWD        320   

#define RIGHT_TURN_RVS          695
#define RIGHT_TURN_FWD          695

#define LEFT_TURN_RVS           695
#define LEFT_TURN_FWD           695

#define FULL_TURN_RVS           1415
#define FULL_TURN_FWD           1415

#define SCAN_RVS                20
#define SCAN_FWD                20

#define CENTER_FLAME     2
#define MAP_MAX          62
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
uint8 ShootWater();
uint8 FireVerifyTemp();
uint8 DecoyCheck();
uint8 FireVerifySens();
void CheckFrontSensor();
uint8 CheckWalls();
uint8 CheckForDoor( uint8 Side );

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
//enum NAVIGATE_STATES
//{
//    FOLLOW_ROUTE,
//    REROUTE_TO_FIRE,
//    STOP,
//    RETURN_TO_ROUTE
//};

enum WALL_SENSOR
{
    CLEAR_SENS,
    FRONT_SENS,
    BACK_SENS,
    BOTH_SENS
};

enum SENSOR_SIDE
{
    LEFT_SIDE,
    RIGHT_SIDE
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
    LEFT_45,
    RIGHT_45,
    LEFT_90,
    RIGHT_90,
    TURN_180,
    LEFT_SCAN,
    RIGHT_SCAN,
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
    ALGN,       //0 
    AWAY,       //1
    CLSR,       //2
    ALGN_AWAY,  //3
    ALGN_CLSR   //4
};

//****************************************************************************
//
//                              Global Variables
//
//****************************************************************************
//uint8 Map[84][30];
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
uint8 Speed;

float M1Integral;
float M2Integral;

uint8 Extinguish;

uint32 MapDistance[65];
uint8 MapDirection[65];
uint8 FollowingMap;
uint16 MapIndex;
uint32 MapDist;

uint32 CatchUp;
uint32 Fixed;
uint32 M1Distance;
uint32 M2Distance;
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

uint32 RBThresh;
uint32 RFThresh;
uint32 ThrshCnt;

sint16 M1Wall;
sint16 M2Wall;

uint32 UnMappedTurn;

uint8 MapDone;
uint32 WaterPulseCnt;
uint8 WaterPulse;


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
sint16 RB_s[6500];
sint16 RF_s[6500];
sint16 RBPI_s[200];
sint16 RFPI_s[200];
uint32 xinp;

//sint16 RB_s[1013];
//sint16 RF_s[1013];
//uint16 LB_s[1013];
//uint16 LF_s[1013];

uint32 xin;

/// Flame Sensor stuff
//uint16 FlL[400];
//uint16 FlLM[2025];
//sint16 FlM[2025];
//uint16 FlRM[2025];
//uint16 FlR[400];
sint16 flmMidMax;
sint16 flmMidMin;
sint16 PrvMidFlame;
sint16 flMidDif;

sint16 flmLftMax;
sint16 flmLftMin;
sint16 PrvLftFlame;
sint16 flLftDif;

sint16 flmRgtMax;
sint16 flmRgtMin;
sint16 PrvRgtFlame;
sint16 flRgtDif;

uint8 IgnFirst;
uint32 flcnt;
uint32 flcn;

uint8 ScLeft;
uint8 ScRight;

uint8 IRCnt;
uint32 RBState;
sint32 RBChng;
uint32 RFState;
sint32 RFChng;

uint16 IRSmpCnt;
uint32 throwaway;

uint8 dir;

#endif	/* HAVVOCC_H */
