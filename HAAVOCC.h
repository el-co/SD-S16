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

// Motor variables
#define MED_SPEED_INIT          1880
#define SLOW_SPEED_INIT         740
#define SUPER_SLOW_SPEED_INIT   530

#define MED_SPEED               225
#define SLOW_SPEED              80
#define SUPER_SLOW_SPEED        60

#define MED_SPEED_CHK           395
#define SLOW_SPEED_CHK          140
#define SUPER_SLOW_SPEED_CHK    105

// Turn variables
#define TURN_30_ENC             60
#define TURN_45_ENC             320
#define TURN_50_ENC             340
#define TURN_90_ENC             695
#define TURN_180_ENC            1415
#define TURN_SCAN_ENC           20

// Map variables
#define MAP_MAX                 37
#define REROUTE_MAX             3

// Flame array variable
#define CENTER_FLAME            2

// UltraSonic variables
#define ONE_FT                  1102
#define TWO_FT                  2205
#define FIVE_FT                 5880

// typedef variables
#define SINT16_MAX              0x7FFF
#define SINT16_MIN              0x8000
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
void Init( void );
void GPIOInit( void );
void TimerInit( void );
void PWMInit( void );
void ADCInit( void );
void ICInit( void );
void MapInit( void );
void ClearVariables( void );

//****************************************************************************
//
//                          Motor Control Functions     
//
//****************************************************************************
void MotorSpeedCtrl( uint32 M1Speed, uint32 M2Speed );
void MotorDirectionCtrl( uint8 LDirection, uint8 RDirection);
uint16 PI( uint16 ActualEncoder, uint16 TrgtEncoder, uint8 Motor );
void SetSpeed( uint32 Spd );
void SaveCurrEnc( void );
void SetDirection( uint32 Direction );
void CheckMap( void );
void ReRoute( void );

//****************************************************************************
//
//                          Sensor Evaluation Functions     
//
//****************************************************************************
uint32 CenterFlame( void );
uint32 CheckCollisionSensors( void );
uint8 CheckFlameDetectors( void );
uint8 CheckFrontSensor( uint32 Dist );
uint8 DecoyCheck( void );
uint8 FireVerifySens( void );
uint8 ShootWater( void );
void ConsoleCheck( void );
void SetUpWall( uint8 Side );
void CheckWalls( uint8 Side );

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
    LEFT_20,
    RIGHT_20,  
    LEFT_45,
    RIGHT_45,
    LEFT_50,
    RIGHT_50,    
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

//****************************************************************************
//
//                              Global Variables
//
//****************************************************************************

// State Machine Variables
uint8  State;
uint32 SixtnHzCnt;
uint32 SecCnt;

// Map and Rerouting Variables
uint32 MapDistance[40];
uint8  MapDirection[40];
uint16 MapIndex;
uint32 MapDist;
uint16 SavedMapIndex;
uint8  SavedMapDist;
uint8  SavedM1Dist;
uint8  SavedM2Dist;
sint32 RRouteDistance[3];
uint8  RRouteDirection[3];
uint16 RRIndex;
uint32 RRDist;

// Motor Variables
uint16 Motor1Speed;
uint16 M1_SuperSlowPWM;
uint16 M1_SlowPWM;
uint16 M1_MedPWM;
uint16 Motor2Speed;
uint16 M2_SuperSlowPWM;
uint16 M2_SlowPWM;
uint16 M2_MedPWM;
uint16 MaxPWM;
uint16 MinPWM;
uint8  CurrSpeed;
uint8  CurrDir;
uint16 SpeedCheck;
uint32 MtrsOffCnt;

// Encoder Variables
uint32 TargetEncoder;
float  EncAdjust;
uint32 WaitForAcclCnt;
float  M1Integral;
sint32 M1Distance;
uint32 M1PosEdgeCnt;
uint16 M1AdjustedTargetEnc;
float  M2Integral;
sint32 M2Distance;
uint32 M2PosEdgeCnt;
uint16 M2AdjustedTargetEnc;
sint32 DistanceDiff;

// Turning Variables
uint32 FwdTurnDist;
uint8  FwdTurnCheck;
uint32 RvsTurnDist;
uint8  RvsTurnCheck;
uint32 FwdTurnCnt;
uint32 RvsTurnCnt;
uint32 NextDir;
uint32 NextSpeed;
uint32 CmpltStopAfterTrnCnt;

// IR Distance Sensor Variables
uint16 IRSens[4];
uint32 IRSensCnt;
uint16 WallFollowInitCnt;
uint32 BckThrsh;
uint32 FntThrsh;
uint32 WallCheckSide;

// Flame Sensors and Water Pump Variables
uint16 FlameSens[5];
uint32 CntrFlame;
sint16 FlmMidMax;
sint16 FlmMidMin;
sint16 PrvMidFlame;
sint16 FlmMidDif;
uint32 FlmDataCnt;
uint8  FlameFirstLoc;
uint8  FireSide;
uint32 WaterPulseCnt;

// UltraSonic Sensor Variables
uint16 USSensPosEdgeTime;
uint16 USSensNegEdgeTime;
uint32 USSensEdgeCnt;
uint32 PersistantBuffer;
sint32 USSensDiff;

// Flags
uint8 SensorEvalFlag;
uint8 USSensorFlag;
uint8 AdjustSpeedFlag;
uint8 ExtinguishFlag;
uint8 TurnFlag;
uint8 FwdTurnDoneFlag;
uint8 RvsTurnDoneFlag;
uint8 MapDoneFlag;
uint8 WaterPulseFlag;
uint8 M2FasterFlag;
uint8 M1FasterFlag;
uint8 FrntSensFlag;
uint8 AdjstLeftFlag;
uint8 AdjstRightFlag;
uint8 AdjstFwdFlag;
uint8 MtrsOffFlag;
uint8 WFInitFlag;   
uint8 WFRunFlag;    
uint8 CnslChckFlag; 
uint8 WallCllFlag;
uint8 DcyCllFlag;
uint8 ExitRoomFlag;
uint8 IgnFirstFlameFlag;
uint8 DesignDayBtn;

#endif	/* HAVVOCC_H */
